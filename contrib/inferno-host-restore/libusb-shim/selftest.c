/*
 * selftest.c — in-process end-to-end test of the host-direct USB path, with no
 * emulator involved. It wires up:
 *
 *   [libusb shim] -> [inferno-usbd broker] -> [tcp_usb] -> [fake iPhone device]
 *
 * A child process plays the Inferno main VM: it connects to the tcp_usb socket
 * and answers USB transactions as a minimal device presenting the usbmux
 * interface (class 0xFF / subclass 0xFE / protocol 0x02) with two bulk
 * endpoints. The parent drives the *real* libusb API (backed by the shim) to
 * enumerate that device, read + parse its descriptors, and run a control and a
 * bulk transfer. This validates the transaction state machine, enumeration,
 * broker fan-out, and descriptor parsing — everything except the live VM.
 *
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/wait.h>
#include <errno.h>

#include "libusb.h"
#include "tcp_usb_proto.h"

#define USBSOCK "/tmp/inferno-selftest-usb.sock"
#define BROKERSOCK "/tmp/inferno-selftest-broker.sock"

static const uint8_t DEV_DESC[18] = {
    18, 1, 0x00, 0x02, 0, 0, 0, 64,
    0xac, 0x05, 0x81, 0x12, 0, 1, 0, 0, 0, 1,
};
static const uint8_t CFG_DESC[32] = {
    /* config */ 9, 2, 32, 0, 1, 1, 0, 0x80, 250,
    /* iface  */ 9, 4, 0, 0, 2, 0xff, 0xfe, 0x02, 0,
    /* ep in  */ 7, 5, 0x81, 2, 0x00, 0x02, 0,
    /* ep out */ 7, 5, 0x02, 2, 0x00, 0x02, 0,
};

static int read_all(int fd, void *b, size_t n)
{ size_t k = 0; while (k < n) { ssize_t r = recv(fd, (char*)b+k, n-k, 0); if (r<=0) return 0; k+=r; } return 1; }
static int write_all(int fd, const void *b, size_t n)
{ size_t k = 0; while (k < n) { ssize_t r = send(fd, (const char*)b+k, n-k, 0); if (r<0) return -1; k+=r; } return 0; }

/* The fake iPhone: connect to the tcp_usb socket and serve USB transactions. */
static void fake_device(void)
{
    int fd = -1;
    for (int i = 0; i < 100; i++) {
        fd = socket(AF_UNIX, SOCK_STREAM, 0);
        struct sockaddr_un a = { .sun_family = AF_UNIX };
        strncpy(a.sun_path, USBSOCK, sizeof(a.sun_path)-1);
        if (connect(fd, (struct sockaddr*)&a, sizeof(a)) == 0) break;
        close(fd); fd = -1; usleep(20000);
    }
    if (fd < 0) { fprintf(stderr, "fake_device: cannot connect\n"); _exit(1); }

    usb_setup_packet last = { 0 };
    for (;;) {
        tcp_usb_header_t h;
        if (!read_all(fd, &h, sizeof(h))) break;
        if (h.type != TCP_USB_REQUEST) continue;
        tcp_usb_request_header req;
        if (!read_all(fd, &req, sizeof(req))) break;

        uint8_t inbuf[1024];
        if (req.pid != TCP_USB_TOKEN_IN && req.length)
            if (!read_all(fd, inbuf, req.length)) break;

        tcp_usb_header_t rh = { .type = TCP_USB_RESPONSE };
        tcp_usb_response_header resp = { 0 };
        resp.addr = req.addr; resp.pid = req.pid; resp.ep = req.ep; resp.id = req.id;
        resp.status = TCP_USB_RET_SUCCESS;
        uint8_t out[1024]; uint16_t outlen = 0;

        if (req.pid == TCP_USB_TOKEN_SETUP) {
            memcpy(&last, inbuf, sizeof(last));
        } else if (req.pid == TCP_USB_TOKEN_IN && req.ep == 0) {
            /* control IN data stage: serve descriptor per last setup */
            if (last.bRequest == 6) { /* GET_DESCRIPTOR */
                uint8_t type = last.wValue >> 8;
                const uint8_t *src = NULL; uint16_t slen = 0;
                if (type == 1) { src = DEV_DESC; slen = sizeof(DEV_DESC); }
                else if (type == 2) { src = CFG_DESC; slen = sizeof(CFG_DESC); }
                if (src) {
                    outlen = req.length < slen ? req.length : slen;
                    memcpy(out, src, outlen);
                }
            }
        } else if (req.pid == TCP_USB_TOKEN_IN && (req.ep & 0x0f) == 1) {
            /* bulk IN on ep1: return a known token */
            const char *msg = "PONG";
            outlen = 4; memcpy(out, msg, 4);
        }
        /* OUT/status stages just succeed. */

        resp.length = outlen;
        if (write_all(fd, &rh, sizeof(rh))) break;
        if (write_all(fd, &resp, sizeof(resp))) break;
        if (req.pid == TCP_USB_TOKEN_IN && outlen)
            if (write_all(fd, out, outlen)) break;
    }
    _exit(0);
}

int main(void)
{
    unlink(USBSOCK); unlink(BROKERSOCK);
    signal(SIGPIPE, SIG_IGN);

    /* Start the broker. */
    pid_t broker = fork();
    if (broker == 0) {
        execl("./inferno-usbd", "inferno-usbd", "-s", USBSOCK, "-b", BROKERSOCK, (char*)NULL);
        perror("exec inferno-usbd"); _exit(127);
    }
    usleep(200000);

    /* Start the fake device (main VM). */
    pid_t dev = fork();
    if (dev == 0) fake_device();
    usleep(300000); /* let enumeration happen */

    setenv("INFERNO_USBD_SOCK", BROKERSOCK, 1);

    int failures = 0;
    #define CHECK(cond, msg) do { if (!(cond)) { printf("FAIL: %s\n", msg); failures++; } else printf("ok  : %s\n", msg); } while (0)

    libusb_context *ctx = NULL;
    CHECK(libusb_init(&ctx) == 0, "libusb_init connects to broker");

    libusb_device **list = NULL;
    ssize_t n = libusb_get_device_list(ctx, &list);
    CHECK(n == 1, "enumeration reports exactly one device");

    if (n >= 1) {
        struct libusb_device_descriptor dd;
        CHECK(libusb_get_device_descriptor(list[0], &dd) == 0, "read device descriptor");
        CHECK(dd.idVendor == 0x05ac, "device descriptor VID == 0x05ac");
        CHECK(dd.idProduct == 0x1281, "device descriptor PID == 0x1281 (recovery)");

        struct libusb_config_descriptor *cfg = NULL;
        CHECK(libusb_get_config_descriptor_by_value(list[0], 1, &cfg) == 0, "read config descriptor");
        if (cfg) {
            CHECK(cfg->bNumInterfaces == 1, "config has 1 interface");
            const struct libusb_interface_descriptor *id = &cfg->interface[0].altsetting[0];
            CHECK(id->bInterfaceClass == 0xff && id->bInterfaceSubClass == 0xfe && id->bInterfaceProtocol == 0x02,
                  "interface is the usbmux class/subclass/protocol");
            CHECK(id->bNumEndpoints == 2, "interface has 2 endpoints");
            CHECK(id->endpoint[0].bEndpointAddress == 0x81, "endpoint 0 is bulk IN 0x81");
            CHECK(id->endpoint[1].bEndpointAddress == 0x02, "endpoint 1 is bulk OUT 0x02");
            libusb_free_config_descriptor(cfg);
        }

        libusb_device_handle *h = NULL;
        CHECK(libusb_open(list[0], &h) == 0, "open device");
        if (h) {
            CHECK(libusb_set_configuration(h, 1) == 0, "set configuration");
            CHECK(libusb_claim_interface(h, 0) == 0, "claim interface 0");

            uint8_t buf[64];
            int r = libusb_control_transfer(h, 0x80, 6, (1<<8), 0, buf, 18, 1000);
            CHECK(r == 18 && buf[8] == 0xac && buf[9] == 0x05, "control GET_DESCRIPTOR round-trips 18 bytes");

            int transferred = 0;
            r = libusb_bulk_transfer(h, 0x81, buf, sizeof(buf), &transferred, 1000);
            CHECK(r == 0 && transferred == 4 && memcmp(buf, "PONG", 4) == 0, "bulk IN round-trips 'PONG'");

            libusb_close(h);
        }
    }
    libusb_free_device_list(list, 1);
    libusb_exit(ctx);

    kill(dev, SIGKILL); waitpid(dev, NULL, 0);
    kill(broker, SIGKILL); waitpid(broker, NULL, 0);
    unlink(USBSOCK); unlink(BROKERSOCK);

    printf("\n%s (%d failure(s))\n", failures ? "SELFTEST FAILED" : "SELFTEST PASSED", failures);
    return failures ? 1 : 0;
}
