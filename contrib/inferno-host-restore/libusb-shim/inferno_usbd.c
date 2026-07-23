/*
 * inferno-usbd — host-side USB broker for the Inferno emulator.
 *
 * Replaces the companion VM. Inferno's main VM (`usb-tcp-host`) connects to a
 * single socket and behaves as the emulated iPhone's USB *device*; this daemon
 * listens on that socket and plays the USB *host controller* the companion VM
 * used to provide. It performs enumeration, then multiplexes the one device to
 * multiple local clients (usbmuxd, libirecovery via the libinferno-usb shim)
 * over a Unix control socket — the role the Linux kernel USB stack played
 * inside the companion.
 *
 * Design notes / status: the tcp_usb framing, the control/bulk transaction
 * state machine, enumeration, and the broker fan-out are implemented here and
 * compile cleanly. The parts that need live bring-up against a booting VM are
 * marked LIVE-TUNE: NAK/retry pacing and the exact mode-change (DFU ->
 * recovery -> restore -> normal) re-enumeration trigger, which depends on
 * whether the guest drops the socket or resets in place.
 *
 * Copyright (c) 2026 Inferno host-restore contributors.
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <unistd.h>

#include "broker_proto.h"
#include "tcp_usb_proto.h"

#define MAX_CLIENTS 16
#define MAX_XFERS 256
#define CFG_DESC_MAX 4096

static bool g_verbose = false;

static void logmsg(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
}
#define VLOG(...)                    \
    do {                             \
        if (g_verbose) logmsg(__VA_ARGS__); \
    } while (0)

/* ------------------------------------------------------------------ */
/* Low-level socket helpers                                           */
/* ------------------------------------------------------------------ */

static int read_all(int fd, void *buf, size_t len)
{
    size_t n = 0;
    while (n < len) {
        ssize_t r = recv(fd, (char *)buf + n, len - n, 0);
        if (r == 0) return 0;          /* peer closed */
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        n += (size_t)r;
    }
    return 1;
}

static int write_all(int fd, const void *buf, size_t len)
{
    size_t n = 0;
    while (n < len) {
        ssize_t r = send(fd, (const char *)buf + n, len - n, 0);
        if (r < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        n += (size_t)r;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* tcp_usb host transport — one synchronous USB transaction at a time. */
/*                                                                     */
/* The main VM answers each REQUEST with exactly one RESPONSE, matched */
/* by id (see hw/usb/hcd-tcp.c). We serialize transactions on the main */
/* socket, so simple id sequencing suffices. A persistent bulk-IN does */
/* NOT block here because we only issue an IN REQUEST when a client has */
/* an IN transfer queued, and we poll the device with bounded waits.   */
/* ------------------------------------------------------------------ */

typedef struct {
    int fd;                 /* accepted main-VM connection, or -1 */
    uint8_t dev_addr;       /* current USB address assigned to device */
    uint64_t next_id;
} tcpusb_link;

/* Perform one USB token transaction. For IN, up to *io_len bytes are read into
 * io_buf and *io_len set to the actual length. For OUT/SETUP, io_len bytes from
 * io_buf are sent. Returns the device status (TCP_USB_RET_*) or <0 on I/O loss
 * of the link (link->fd is closed and set to -1). */
static int tcpusb_txn(tcpusb_link *l, int pid, uint8_t ep, void *io_buf,
                      uint16_t *io_len)
{
    tcp_usb_header_t hdr = { .type = TCP_USB_REQUEST };
    tcp_usb_request_header req = { 0 };
    uint16_t len = io_len ? *io_len : 0;

    req.addr = l->dev_addr;
    req.pid = pid;
    req.ep = ep;
    req.id = l->next_id++;
    req.stream = 0;
    req.short_not_ok = 0;
    req.int_req = 0;
    req.length = len;

    if (write_all(l->fd, &hdr, sizeof(hdr)) < 0) goto lost;
    if (write_all(l->fd, &req, sizeof(req)) < 0) goto lost;
    if (pid != TCP_USB_TOKEN_IN && len) {
        if (write_all(l->fd, io_buf, len) < 0) goto lost;
    }

    /* Read the matching response. */
    tcp_usb_header_t rhdr = { 0 };
    int rr = read_all(l->fd, &rhdr, sizeof(rhdr));
    if (rr <= 0) goto lost;
    if (rhdr.type != TCP_USB_RESPONSE) {
        VLOG("unexpected response header type %u", rhdr.type);
        goto lost;
    }
    tcp_usb_response_header resp = { 0 };
    if (read_all(l->fd, &resp, sizeof(resp)) <= 0) goto lost;

    if (resp.length && pid == TCP_USB_TOKEN_IN) {
        uint16_t want = resp.length;
        if (io_len && want > *io_len) want = *io_len;
        if (read_all(l->fd, io_buf, want) <= 0) goto lost;
        if (io_len) *io_len = want;
        /* Drain any surplus the device announced beyond our buffer. */
        for (uint16_t extra = resp.length - want; extra; ) {
            char junk[512];
            uint16_t chunk = extra > sizeof(junk) ? sizeof(junk) : extra;
            if (read_all(l->fd, junk, chunk) <= 0) goto lost;
            extra -= chunk;
        }
    } else if (io_len && pid == TCP_USB_TOKEN_IN) {
        *io_len = 0;
    }

    return (int)resp.status;

lost:
    if (l->fd >= 0) { close(l->fd); l->fd = -1; }
    return -1000;
}

/* A full control transfer: SETUP, optional DATA, STATUS. Returns bytes
 * transferred in the data stage (>=0) or a negative TCP_USB_RET_* / -1000. */
static int tcpusb_control(tcpusb_link *l, const usb_setup_packet *sp,
                          void *data, uint16_t wlen)
{
    bool dev_to_host = (sp->bmRequestType & 0x80) != 0;
    uint16_t len;
    int st;

    /* SETUP stage. */
    len = sizeof(*sp);
    uint8_t setup_buf[sizeof(*sp)];
    memcpy(setup_buf, sp, sizeof(*sp));
    st = tcpusb_txn(l, TCP_USB_TOKEN_SETUP, 0, setup_buf, &len);
    if (st == -1000) return -1000;
    if (st != TCP_USB_RET_SUCCESS && st != TCP_USB_RET_NAK) {
        VLOG("control SETUP failed status=%d", st);
        return st < 0 ? st : -1;
    }

    int data_done = 0;
    /* DATA stage. */
    if (wlen) {
        len = wlen;
        st = tcpusb_txn(l, dev_to_host ? TCP_USB_TOKEN_IN : TCP_USB_TOKEN_OUT, 0,
                        data, &len);
        if (st == -1000) return -1000;
        if (st == TCP_USB_RET_SUCCESS) data_done = dev_to_host ? len : wlen;
    }

    /* STATUS stage (opposite direction, zero length). */
    len = 0;
    st = tcpusb_txn(l, dev_to_host ? TCP_USB_TOKEN_OUT : TCP_USB_TOKEN_IN, 0,
                    NULL, &len);
    if (st == -1000) return -1000;

    /* Track SET_ADDRESS so subsequent transactions use the new address. */
    if (sp->bmRequestType == 0x00 && sp->bRequest == 5 /* SET_ADDRESS */) {
        l->dev_addr = (uint8_t)sp->wValue;
        VLOG("device address set to %u", l->dev_addr);
    }
    return data_done;
}

/* ------------------------------------------------------------------ */
/* Enumeration                                                        */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t dev_desc[18];
    uint8_t cfg_desc[CFG_DESC_MAX];
    uint16_t cfg_len;
    bool present;
} device_cache;

static int get_descriptor(tcpusb_link *l, uint8_t type, uint8_t index,
                          void *out, uint16_t len)
{
    usb_setup_packet sp = {
        .bmRequestType = 0x80,
        .bRequest = 6, /* GET_DESCRIPTOR */
        .wValue = (uint16_t)((type << 8) | index),
        .wIndex = 0,
        .wLength = len,
    };
    return tcpusb_control(l, &sp, out, len);
}

static bool enumerate(tcpusb_link *l, device_cache *dc)
{
    memset(dc, 0, sizeof(*dc));
    l->dev_addr = 0;

    /* Device descriptor (full 18 bytes; addr 0). */
    if (get_descriptor(l, 1 /*DEVICE*/, 0, dc->dev_desc, sizeof(dc->dev_desc)) < 8) {
        VLOG("enumerate: device descriptor read failed");
        return false;
    }

    /* Assign an address. */
    usb_setup_packet setaddr = {
        .bmRequestType = 0x00, .bRequest = 5, .wValue = 2, .wIndex = 0,
        .wLength = 0,
    };
    if (tcpusb_control(l, &setaddr, NULL, 0) == -1000) return false;

    /* Config descriptor: first 9 bytes for wTotalLength, then the whole blob. */
    uint8_t head[9];
    if (get_descriptor(l, 2 /*CONFIG*/, 0, head, sizeof(head)) < 9) {
        VLOG("enumerate: config header read failed");
        return false;
    }
    uint16_t total = (uint16_t)(head[2] | (head[3] << 8));
    if (total < 9) total = 9;
    if (total > CFG_DESC_MAX) total = CFG_DESC_MAX;
    if (get_descriptor(l, 2 /*CONFIG*/, 0, dc->cfg_desc, total) < 9) {
        VLOG("enumerate: full config read failed");
        return false;
    }
    dc->cfg_len = total;
    dc->present = true;

    uint16_t vid = dc->dev_desc[8] | (dc->dev_desc[9] << 8);
    uint16_t pid = dc->dev_desc[10] | (dc->dev_desc[11] << 8);
    logmsg("inferno-usbd: device enumerated vid=%04x pid=%04x cfg_len=%u",
           vid, pid, dc->cfg_len);
    return true;
}

/* ------------------------------------------------------------------ */
/* Broker: fan out the one device to local shim clients               */
/* ------------------------------------------------------------------ */

typedef struct {
    int fd;
    bool hello_ok;
} client_t;

static void send_msg(int fd, uint32_t kind, uint32_t tag, int32_t status,
                     const void *payload, uint32_t len)
{
    broker_msg_hdr h = { .kind = kind, .tag = tag, .status = status,
                         .length = len };
    if (write_all(fd, &h, sizeof(h)) < 0) return;
    if (len && payload) write_all(fd, payload, len);
}

static void broadcast_arrived(client_t *clients, int nclients, device_cache *dc)
{
    uint8_t buf[18 + CFG_DESC_MAX];
    memcpy(buf, dc->dev_desc, 18);
    memcpy(buf + 18, dc->cfg_desc, dc->cfg_len);
    for (int i = 0; i < nclients; i++) {
        if (clients[i].fd >= 0 && clients[i].hello_ok)
            send_msg(clients[i].fd, BROKER_EVENT_ARRIVED, 0, 0, buf,
                     18 + dc->cfg_len);
    }
}

static void broadcast_left(client_t *clients, int nclients)
{
    for (int i = 0; i < nclients; i++)
        if (clients[i].fd >= 0 && clients[i].hello_ok)
            send_msg(clients[i].fd, BROKER_EVENT_LEFT, 0, 0, NULL, 0);
}

/* Handle one request from a shim client. Returns 0 to keep the client, -1 to
 * drop it. Device transactions are issued synchronously on the main link; the
 * reply (with any IN data) carries the same tag. */
static int handle_client_request(client_t *c, tcpusb_link *l, device_cache *dc)
{
    broker_msg_hdr h;
    int r = read_all(c->fd, &h, sizeof(h));
    if (r <= 0) return -1;

    switch (h.kind) {
    case BROKER_HELLO: {
        uint32_t ver = 0;
        if (h.length >= 4 && read_all(c->fd, &ver, 4) <= 0) return -1;
        c->hello_ok = true;
        send_msg(c->fd, BROKER_REPLY, h.tag, INFERNO_BROKER_VERSION, NULL, 0);
        if (dc->present) {
            uint8_t buf[18 + CFG_DESC_MAX];
            memcpy(buf, dc->dev_desc, 18);
            memcpy(buf + 18, dc->cfg_desc, dc->cfg_len);
            send_msg(c->fd, BROKER_EVENT_ARRIVED, 0, 0, buf, 18 + dc->cfg_len);
        }
        return 0;
    }
    case BROKER_GET_DESCRIPTORS: {
        if (!dc->present) { send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_NODEV, NULL, 0); return 0; }
        uint8_t buf[18 + CFG_DESC_MAX];
        memcpy(buf, dc->dev_desc, 18);
        memcpy(buf + 18, dc->cfg_desc, dc->cfg_len);
        send_msg(c->fd, BROKER_REPLY, h.tag, 0, buf, 18 + dc->cfg_len);
        return 0;
    }
    case BROKER_SET_CONFIG: {
        uint32_t cfg = 0;
        if (h.length >= 4 && read_all(c->fd, &cfg, 4) <= 0) return -1;
        usb_setup_packet sp = { .bmRequestType = 0, .bRequest = 9,
                                .wValue = (uint16_t)cfg, .wIndex = 0, .wLength = 0 };
        int st = tcpusb_control(l, &sp, NULL, 0);
        send_msg(c->fd, BROKER_REPLY, h.tag, st == -1000 ? TCP_USB_RET_NODEV : 0, NULL, 0);
        return 0;
    }
    case BROKER_SET_ALT: {
        uint32_t v[2] = { 0, 0 };
        if (h.length >= 8 && read_all(c->fd, v, 8) <= 0) return -1;
        usb_setup_packet sp = { .bmRequestType = 0x01, .bRequest = 11 /*SET_INTERFACE*/,
                                .wValue = (uint16_t)v[1], .wIndex = (uint16_t)v[0],
                                .wLength = 0 };
        int st = tcpusb_control(l, &sp, NULL, 0);
        send_msg(c->fd, BROKER_REPLY, h.tag, st == -1000 ? TCP_USB_RET_NODEV : 0, NULL, 0);
        return 0;
    }
    case BROKER_CLAIM_INTERFACE:
    case BROKER_RELEASE_INTERFACE: {
        uint32_t iface = 0;
        if (h.length >= 4 && read_all(c->fd, &iface, 4) <= 0) return -1;
        /* Interface ownership is advisory here (single device, cooperating
         * clients that use disjoint modes). LIVE-TUNE: enforce exclusivity if
         * two clients ever contend for the same interface simultaneously. */
        send_msg(c->fd, BROKER_REPLY, h.tag, 0, NULL, 0);
        return 0;
    }
    case BROKER_CLEAR_HALT: {
        uint32_t ep = 0;
        if (h.length >= 4 && read_all(c->fd, &ep, 4) <= 0) return -1;
        usb_setup_packet sp = { .bmRequestType = 0x02, .bRequest = 1 /*CLEAR_FEATURE*/,
                                .wValue = 0 /*ENDPOINT_HALT*/, .wIndex = (uint16_t)ep,
                                .wLength = 0 };
        int st = tcpusb_control(l, &sp, NULL, 0);
        send_msg(c->fd, BROKER_REPLY, h.tag, st == -1000 ? TCP_USB_RET_NODEV : 0, NULL, 0);
        return 0;
    }
    case BROKER_CONTROL: {
        broker_control_req req;
        if (read_all(c->fd, &req, sizeof(req)) <= 0) return -1;
        usb_setup_packet sp;
        memcpy(&sp, req.setup, sizeof(sp));
        uint16_t wlen = sp.wLength;
        bool in = (sp.bmRequestType & 0x80) != 0;
        uint8_t *data = wlen ? malloc(wlen) : NULL;
        if (wlen && !in) {
            if (read_all(c->fd, data, wlen) <= 0) { free(data); return -1; }
        }
        int st = tcpusb_control(l, &sp, data, wlen);
        if (st >= 0 && in)
            send_msg(c->fd, BROKER_REPLY, h.tag, st, data, (uint32_t)st);
        else
            send_msg(c->fd, BROKER_REPLY, h.tag, st == -1000 ? TCP_USB_RET_NODEV : st, NULL, 0);
        free(data);
        return 0;
    }
    case BROKER_BULK:
    case BROKER_SUBMIT_ASYNC: {
        broker_transfer_req req;
        if (read_all(c->fd, &req, sizeof(req)) <= 0) return -1;
        bool in = (req.ep & 0x80) != 0;
        uint8_t *data = req.length ? malloc(req.length) : NULL;
        if (req.length && !in) {
            if (read_all(c->fd, data, req.length) <= 0) { free(data); return -1; }
        }
        uint16_t len = (uint16_t)req.length;
        int st = tcpusb_txn(l, in ? TCP_USB_TOKEN_IN : TCP_USB_TOKEN_OUT,
                            req.ep & 0x0f, data, &len);
        uint32_t reply_kind = (h.kind == BROKER_SUBMIT_ASYNC) ? BROKER_ASYNC_COMPLETE
                                                              : BROKER_REPLY;
        if (st == -1000)
            send_msg(c->fd, reply_kind, h.tag, TCP_USB_RET_NODEV, NULL, 0);
        else if (in)
            send_msg(c->fd, reply_kind, h.tag, len, data, len);
        else
            send_msg(c->fd, reply_kind, h.tag, (int32_t)req.length, NULL, 0);
        free(data);
        return 0;
    }
    case BROKER_RESET: {
        if (enumerate(l, dc))
            send_msg(c->fd, BROKER_REPLY, h.tag, 0, NULL, 0);
        else
            send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_NODEV, NULL, 0);
        return 0;
    }
    default:
        VLOG("unknown client opcode %u", h.kind);
        /* Skip any payload we don't understand to stay in sync. */
        for (uint32_t left = h.length; left; ) {
            char junk[256];
            uint32_t chunk = left > sizeof(junk) ? sizeof(junk) : left;
            if (read_all(c->fd, junk, chunk) <= 0) return -1;
            left -= chunk;
        }
        send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_STALL, NULL, 0);
        return 0;
    }
}

/* ------------------------------------------------------------------ */
/* Socket setup + main loop                                           */
/* ------------------------------------------------------------------ */

static int bind_unix(const char *path)
{
    struct sockaddr_un a = { 0 };
    if (strlen(path) >= sizeof(a.sun_path)) { logmsg("path too long: %s", path); return -1; }
    unlink(path);
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) { perror("socket"); return -1; }
    a.sun_family = AF_UNIX;
    strncpy(a.sun_path, path, sizeof(a.sun_path) - 1);
    if (bind(fd, (struct sockaddr *)&a, sizeof(a)) < 0) { perror("bind"); close(fd); return -1; }
    chmod(path, 0666);
    if (listen(fd, 4) < 0) { perror("listen"); close(fd); return -1; }
    return fd;
}

static void usage(const char *p)
{
    fprintf(stderr,
        "Usage: %s [-s tcp_usb_socket] [-b broker_socket] [-v]\n"
        "  -s  path the main VM connects to (default %s)\n"
        "  -b  path shim clients connect to (default %s)\n"
        "  -v  verbose\n",
        p, USB_TCP_REMOTE_UNIX_DEFAULT, INFERNO_USBD_SOCK_DEFAULT);
}

int main(int argc, char **argv)
{
    const char *usb_path = USB_TCP_REMOTE_UNIX_DEFAULT;
    const char *broker_path = INFERNO_USBD_SOCK_DEFAULT;
    int opt;
    while ((opt = getopt(argc, argv, "s:b:vh")) != -1) {
        switch (opt) {
        case 's': usb_path = optarg; break;
        case 'b': broker_path = optarg; break;
        case 'v': g_verbose = true; break;
        default: usage(argv[0]); return opt == 'h' ? 0 : 2;
        }
    }

    signal(SIGPIPE, SIG_IGN);

    int usb_listen = bind_unix(usb_path);
    int broker_listen = bind_unix(broker_path);
    if (usb_listen < 0 || broker_listen < 0) return 1;
    logmsg("inferno-usbd: listening for main VM on %s, clients on %s",
           usb_path, broker_path);

    tcpusb_link link = { .fd = -1, .dev_addr = 0, .next_id = 1 };
    device_cache dc = { 0 };
    client_t clients[MAX_CLIENTS];
    for (int i = 0; i < MAX_CLIENTS; i++) { clients[i].fd = -1; clients[i].hello_ok = false; }

    for (;;) {
        struct pollfd pfds[2 + MAX_CLIENTS];
        int n = 0;
        pfds[n].fd = usb_listen; pfds[n].events = POLLIN; n++;
        pfds[n].fd = broker_listen; pfds[n].events = POLLIN; n++;
        int cli_base = n;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].fd >= 0) { pfds[n].fd = clients[i].fd; pfds[n].events = POLLIN; n++; }
        }

        int pr = poll(pfds, n, -1);
        if (pr < 0) { if (errno == EINTR) continue; perror("poll"); break; }

        /* New main-VM connection. */
        if (pfds[0].revents & POLLIN) {
            int fd = accept(usb_listen, NULL, NULL);
            if (fd >= 0) {
                if (link.fd >= 0) close(link.fd);
                link.fd = fd; link.dev_addr = 0; link.next_id = 1;
                logmsg("inferno-usbd: main VM connected; enumerating...");
                if (enumerate(&link, &dc)) broadcast_arrived(clients, MAX_CLIENTS, &dc);
                else logmsg("inferno-usbd: enumeration failed (device may still be booting)");
            }
        }

        /* New shim client. */
        if (pfds[1].revents & POLLIN) {
            int fd = accept(broker_listen, NULL, NULL);
            if (fd >= 0) {
                int slot = -1;
                for (int i = 0; i < MAX_CLIENTS; i++) if (clients[i].fd < 0) { slot = i; break; }
                if (slot < 0) { logmsg("too many clients"); close(fd); }
                else { clients[slot].fd = fd; clients[slot].hello_ok = false; VLOG("client %d connected", slot); }
            }
        }

        /* Client requests. */
        int idx = cli_base;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].fd < 0) continue;
            short re = pfds[idx].revents;
            idx++;
            if (re & (POLLIN | POLLHUP | POLLERR)) {
                if (link.fd < 0) {
                    /* No device: fail fast so clients don't block. */
                    broker_msg_hdr h;
                    if (read_all(clients[i].fd, &h, sizeof(h)) <= 0) {
                        close(clients[i].fd); clients[i].fd = -1; continue;
                    }
                    for (uint32_t left = h.length; left; ) {
                        char junk[256]; uint32_t c = left > sizeof(junk) ? sizeof(junk) : left;
                        if (read_all(clients[i].fd, junk, c) <= 0) break;
                        left -= c;
                    }
                    if (h.kind == BROKER_HELLO) { clients[i].hello_ok = true; send_msg(clients[i].fd, BROKER_REPLY, h.tag, INFERNO_BROKER_VERSION, NULL, 0); }
                    else send_msg(clients[i].fd, BROKER_REPLY, h.tag, TCP_USB_RET_NODEV, NULL, 0);
                    continue;
                }
                if (handle_client_request(&clients[i], &link, &dc) < 0) {
                    VLOG("client %d disconnected", i);
                    close(clients[i].fd); clients[i].fd = -1;
                }
                /* A transaction may have lost the main link. */
                if (link.fd < 0 && dc.present) {
                    dc.present = false;
                    logmsg("inferno-usbd: main VM link lost");
                    broadcast_left(clients, MAX_CLIENTS);
                }
            }
        }
    }

    close(usb_listen);
    close(broker_listen);
    unlink(usb_path);
    unlink(broker_path);
    return 0;
}
