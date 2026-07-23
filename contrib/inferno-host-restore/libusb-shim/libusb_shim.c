/*
 * libinferno-usb — a libusb-1.0 API-compatible shim backed by inferno-usbd.
 *
 * Link the libimobiledevice restore stack (usbmuxd, libirecovery,
 * idevicerestore) against THIS instead of the real libusb, and every USB
 * access is routed to inferno-usbd, which owns the Inferno tcp_usb socket.
 * The emulated iPhone then appears to the unmodified tools exactly as a real
 * device would — no companion VM.
 *
 * It uses the vendored upstream libusb.h so struct layouts / enum values match
 * whatever the tools were compiled against.
 *
 * Implemented: enumeration, descriptor parsing, sync control/bulk/interrupt,
 * async transfers with a broker-fd-backed event loop, and the pollfd/timeout
 * integration usbmuxd needs. Hotplug is intentionally reported UNSUPPORTED so
 * usbmuxd uses its libusb_get_device_list() discovery-polling fallback.
 *
 * Copyright (c) 2026 Inferno host-restore contributors.
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include <errno.h>
#include <poll.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "libusb.h"
#include "broker_proto.h"
#include "tcp_usb_proto.h"

#define CFG_DESC_MAX 4096
#define MAX_PENDING 256

/* ------------------------------------------------------------------ */
/* Internal objects                                                   */
/* ------------------------------------------------------------------ */

static int shim_dbg = -1;
#define SDBG(...)                                                    \
    do {                                                             \
        if (shim_dbg < 0) shim_dbg = getenv("INFERNO_SHIM_DEBUG") ? 1 : 0; \
        if (shim_dbg) { fprintf(stderr, "[shim] " __VA_ARGS__); fflush(stderr); } \
    } while (0)

struct libusb_context {
    int broker_fd;
    bool device_present;
    uint8_t dev_desc[18];
    uint8_t cfg_desc[CFG_DESC_MAX];
    uint16_t cfg_len;
    uint32_t next_tag;
    /* pending async transfers, keyed by tag */
    struct { uint32_t tag; struct libusb_transfer *xfer; bool active; } pend[MAX_PENDING];
};

struct libusb_device {
    struct libusb_context *ctx;
    int refcnt;
};

struct libusb_device_handle {
    struct libusb_device *dev;
    struct libusb_context *ctx;
};

static struct libusb_context *default_ctx;

/* ------------------------------------------------------------------ */
/* Broker transport                                                   */
/* ------------------------------------------------------------------ */

static int io_read(int fd, void *buf, size_t len)
{
    size_t n = 0;
    while (n < len) {
        ssize_t r = recv(fd, (char *)buf + n, len - n, 0);
        if (r == 0) return 0;
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        n += (size_t)r;
    }
    return 1;
}

static int io_write(int fd, const void *buf, size_t len)
{
    size_t n = 0;
    while (n < len) {
        ssize_t r = send(fd, (const char *)buf + n, len - n, 0);
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        n += (size_t)r;
    }
    return 0;
}

static void send_request(struct libusb_context *ctx, uint32_t kind, uint32_t tag,
                         const void *payload, uint32_t len)
{
    broker_msg_hdr h = { .kind = kind, .tag = tag, .status = 0, .length = len };
    io_write(ctx->broker_fd, &h, sizeof(h));
    if (len && payload) io_write(ctx->broker_fd, payload, len);
}

/* Complete a pending async transfer identified by tag. */
static void complete_async(struct libusb_context *ctx, uint32_t tag, int32_t status,
                           uint8_t *data, uint32_t dlen)
{
    for (int i = 0; i < MAX_PENDING; i++) {
        if (!ctx->pend[i].active || ctx->pend[i].tag != tag) continue;
        struct libusb_transfer *x = ctx->pend[i].xfer;
        ctx->pend[i].active = false;
        if (status < 0) {
            x->status = (status == TCP_USB_RET_NODEV) ? LIBUSB_TRANSFER_NO_DEVICE
                      : (status == TCP_USB_RET_STALL) ? LIBUSB_TRANSFER_STALL
                                                      : LIBUSB_TRANSFER_ERROR;
            x->actual_length = 0;
        } else {
            x->status = LIBUSB_TRANSFER_COMPLETED;
            /* Copy IN data into the transfer, never past its capacity and never
             * to a NULL buffer (for control the data area starts 8 bytes in). */
            unsigned char *dst = NULL;
            uint32_t cap = 0;
            if (x->buffer) {
                if (x->type == LIBUSB_TRANSFER_TYPE_CONTROL) {
                    dst = libusb_control_transfer_get_data(x);
                    cap = (x->length > (int)LIBUSB_CONTROL_SETUP_SIZE)
                              ? (uint32_t)(x->length - (int)LIBUSB_CONTROL_SETUP_SIZE) : 0;
                } else {
                    dst = x->buffer;
                    cap = (x->length > 0) ? (uint32_t)x->length : 0;
                }
            }
            uint32_t copy = dlen < cap ? dlen : cap;
            if (copy && data && dst) memcpy(dst, data, copy);
            x->actual_length = (int)copy;
        }
        /*
         * Snapshot the flag BEFORE the callback: a callback is allowed to call
         * libusb_free_transfer(x) itself (usbmuxd's rx_callback does on error),
         * so x may be freed once the callback returns — reading x->flags after
         * it would be a use-after-free and freeing again a double-free. Only
         * when FREE_TRANSFER is set does libusb (not the callback) own the free.
         * FREE_BUFFER is applied only inside libusb_free_transfer(), never here
         * (usbmuxd re-uses one transfer lang-ID -> serial and resubmits it).
         */
        int free_after = (x->flags & LIBUSB_TRANSFER_FREE_TRANSFER) != 0;
        if (x->callback) x->callback(x);
        if (free_after) libusb_free_transfer(x);
        return;
    }
}

/* Read and dispatch one broker message. If want_tag != 0, returns 1 with *out_*
 * filled when a REPLY with that tag arrives. Async completions and device
 * arrival/removal events are handled as they stream in. Returns 0 on the wanted
 * reply, -1 on broker loss, 1 if some other message was handled. */
static int pump_one(struct libusb_context *ctx, uint32_t want_tag,
                    int32_t *out_status, uint8_t *out_buf, uint32_t out_cap,
                    uint32_t *out_len)
{
    broker_msg_hdr h;
    int r = io_read(ctx->broker_fd, &h, sizeof(h));
    if (r <= 0) return -1;

    uint8_t stackbuf[64];
    uint8_t *pl = NULL;
    if (h.length) {
        pl = (h.length <= sizeof(stackbuf)) ? stackbuf : malloc(h.length);
        if (io_read(ctx->broker_fd, pl, h.length) <= 0) { if (pl != stackbuf) free(pl); return -1; }
    }

    int rc = 1;
    switch (h.kind) {
    case BROKER_EVENT_ARRIVED:
        if (h.length >= 18) {
            memcpy(ctx->dev_desc, pl, 18);
            ctx->cfg_len = (h.length - 18 > CFG_DESC_MAX) ? CFG_DESC_MAX : (uint16_t)(h.length - 18);
            memcpy(ctx->cfg_desc, pl + 18, ctx->cfg_len);
            ctx->device_present = true;
        }
        break;
    case BROKER_EVENT_LEFT:
        ctx->device_present = false;
        break;
    case BROKER_ASYNC_COMPLETE:
        complete_async(ctx, h.tag, h.status, pl, h.length);
        break;
    case BROKER_REPLY:
        if (want_tag && h.tag == want_tag) {
            if (out_status) *out_status = h.status;
            if (out_len) *out_len = 0;
            if (h.length && out_buf) {
                uint32_t c = h.length > out_cap ? out_cap : h.length;
                memcpy(out_buf, pl, c);
                if (out_len) *out_len = c;
            }
            rc = 0;
        }
        break;
    default:
        break;
    }
    if (pl && pl != stackbuf) free(pl);
    return rc;
}

/* Block until the REPLY for want_tag arrives (dispatching other traffic). */
static int wait_reply(struct libusb_context *ctx, uint32_t tag, int32_t *status,
                      uint8_t *buf, uint32_t cap, uint32_t *len)
{
    for (;;) {
        int r = pump_one(ctx, tag, status, buf, cap, len);
        if (r <= 0) return r; /* 0 = got it, -1 = broker loss */
    }
}

/* ------------------------------------------------------------------ */
/* Descriptor parsing                                                 */
/* ------------------------------------------------------------------ */

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }

static void parse_device_desc(const uint8_t *d, struct libusb_device_descriptor *o)
{
    memset(o, 0, sizeof(*o));
    o->bLength = d[0]; o->bDescriptorType = d[1];
    o->bcdUSB = rd16(d + 2);
    o->bDeviceClass = d[4]; o->bDeviceSubClass = d[5]; o->bDeviceProtocol = d[6];
    o->bMaxPacketSize0 = d[7];
    o->idVendor = rd16(d + 8); o->idProduct = rd16(d + 10);
    o->bcdDevice = rd16(d + 12);
    o->iManufacturer = d[14]; o->iProduct = d[15]; o->iSerialNumber = d[16];
    o->bNumConfigurations = d[17];
}

/* Build a libusb_config_descriptor tree from a raw config blob. */
static struct libusb_config_descriptor *parse_config(const uint8_t *blob, uint16_t len)
{
    if (len < 9) return NULL;
    struct libusb_config_descriptor *cfg = calloc(1, sizeof(*cfg));
    cfg->bLength = blob[0]; cfg->bDescriptorType = blob[1];
    cfg->wTotalLength = rd16(blob + 2);
    cfg->bNumInterfaces = blob[4];
    cfg->bConfigurationValue = blob[5];
    cfg->iConfiguration = blob[6];
    cfg->bmAttributes = blob[7];
    cfg->MaxPower = blob[8];

    struct libusb_interface *ifaces = calloc(cfg->bNumInterfaces ? cfg->bNumInterfaces : 1,
                                             sizeof(struct libusb_interface));
    /* First pass: count altsettings per interface number. */
    /* We store altsettings sequentially; libimobiledevice reads altsetting[0]
     * and, for recovery, iface 1 alt 1. We build per-interface altsetting arrays
     * generously (up to 8 alts each). */
    int max_if = cfg->bNumInterfaces ? cfg->bNumInterfaces : 8;
    struct libusb_interface_descriptor **alts = calloc(max_if, sizeof(void *));
    int *nalt = calloc(max_if, sizeof(int));
    for (int i = 0; i < max_if; i++)
        alts[i] = calloc(8, sizeof(struct libusb_interface_descriptor));

    uint16_t off = blob[0]; /* skip config header */
    struct libusb_interface_descriptor *cur = NULL;
    struct libusb_endpoint_descriptor *eps = NULL;
    int ep_i = 0;
    while (off + 2 <= len) {
        uint8_t blen = blob[off], btype = blob[off + 1];
        if (blen == 0 || off + blen > len) break;
        if (btype == 0x04 /* INTERFACE */ && blen >= 9) {
            uint8_t inum = blob[off + 2];
            uint8_t alt = blob[off + 3];
            if (inum < max_if && alt < 8) {
                cur = &alts[inum][alt];
                if (alt + 1 > nalt[inum]) nalt[inum] = alt + 1;
                cur->bLength = blen; cur->bDescriptorType = btype;
                cur->bInterfaceNumber = inum; cur->bAlternateSetting = alt;
                cur->bNumEndpoints = blob[off + 4];
                cur->bInterfaceClass = blob[off + 5];
                cur->bInterfaceSubClass = blob[off + 6];
                cur->bInterfaceProtocol = blob[off + 7];
                cur->iInterface = blob[off + 8];
                eps = cur->bNumEndpoints ? calloc(cur->bNumEndpoints,
                        sizeof(struct libusb_endpoint_descriptor)) : NULL;
                cur->endpoint = eps;
                ep_i = 0;
            }
        } else if (btype == 0x05 /* ENDPOINT */ && cur && eps && ep_i < cur->bNumEndpoints && blen >= 7) {
            struct libusb_endpoint_descriptor *e = &eps[ep_i++];
            e->bLength = blen; e->bDescriptorType = btype;
            e->bEndpointAddress = blob[off + 2];
            e->bmAttributes = blob[off + 3];
            e->wMaxPacketSize = rd16(blob + off + 4);
            e->bInterval = blob[off + 6];
        }
        off += blen;
    }

    for (int i = 0; i < (cfg->bNumInterfaces ? cfg->bNumInterfaces : 0); i++) {
        ifaces[i].altsetting = alts[i];
        ifaces[i].num_altsetting = nalt[i] ? nalt[i] : 1;
    }
    cfg->interface = ifaces;
    /* alts arrays are owned by ifaces now; free the index arrays only. */
    free(alts);
    free(nalt);
    return cfg;
}

/* ------------------------------------------------------------------ */
/* Lifecycle                                                          */
/* ------------------------------------------------------------------ */

static int broker_connect(const char *path)
{
    int fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0) return -1;
    struct sockaddr_un a = { 0 };
    a.sun_family = AF_UNIX;
    strncpy(a.sun_path, path, sizeof(a.sun_path) - 1);
    if (connect(fd, (struct sockaddr *)&a, sizeof(a)) < 0) { close(fd); return -1; }
    return fd;
}

int libusb_init(libusb_context **ctx)
{
    struct libusb_context *c = calloc(1, sizeof(*c));
    if (!c) return LIBUSB_ERROR_NO_MEM;
    const char *path = getenv("INFERNO_USBD_SOCK");
    c->broker_fd = broker_connect(path ? path : INFERNO_USBD_SOCK_DEFAULT);
    if (c->broker_fd < 0) { free(c); return LIBUSB_ERROR_OTHER; }
    c->next_tag = 1;

    uint32_t ver = INFERNO_BROKER_VERSION;
    uint32_t tag = c->next_tag++;
    send_request(c, BROKER_HELLO, tag, &ver, sizeof(ver));
    int32_t st = 0;
    wait_reply(c, tag, &st, NULL, 0, NULL);

    if (ctx) *ctx = c; else default_ctx = c;
    return LIBUSB_SUCCESS;
}

void libusb_exit(libusb_context *ctx)
{
    if (!ctx) ctx = default_ctx;
    if (!ctx) return;
    if (ctx->broker_fd >= 0) close(ctx->broker_fd);
    if (ctx == default_ctx) default_ctx = NULL;
    free(ctx);
}

const struct libusb_version *libusb_get_version(void)
{
    static const struct libusb_version v = { 1, 0, 27, 0, "-inferno", "libinferno-usb" };
    return &v;
}

int libusb_set_option(libusb_context *ctx, enum libusb_option option, ...) { (void)ctx; (void)option; return LIBUSB_SUCCESS; }
void libusb_set_debug(libusb_context *ctx, int level) { (void)ctx; (void)level; }
int libusb_has_capability(uint32_t capability) { (void)capability; return 0; /* no hotplug: force discovery fallback */ }

const char *libusb_error_name(int code)
{
    switch (code) {
    case LIBUSB_SUCCESS: return "LIBUSB_SUCCESS";
    case LIBUSB_ERROR_IO: return "LIBUSB_ERROR_IO";
    case LIBUSB_ERROR_INVALID_PARAM: return "LIBUSB_ERROR_INVALID_PARAM";
    case LIBUSB_ERROR_ACCESS: return "LIBUSB_ERROR_ACCESS";
    case LIBUSB_ERROR_NO_DEVICE: return "LIBUSB_ERROR_NO_DEVICE";
    case LIBUSB_ERROR_NOT_FOUND: return "LIBUSB_ERROR_NOT_FOUND";
    case LIBUSB_ERROR_BUSY: return "LIBUSB_ERROR_BUSY";
    case LIBUSB_ERROR_TIMEOUT: return "LIBUSB_ERROR_TIMEOUT";
    case LIBUSB_ERROR_OVERFLOW: return "LIBUSB_ERROR_OVERFLOW";
    case LIBUSB_ERROR_PIPE: return "LIBUSB_ERROR_PIPE";
    case LIBUSB_ERROR_INTERRUPTED: return "LIBUSB_ERROR_INTERRUPTED";
    case LIBUSB_ERROR_NO_MEM: return "LIBUSB_ERROR_NO_MEM";
    case LIBUSB_ERROR_NOT_SUPPORTED: return "LIBUSB_ERROR_NOT_SUPPORTED";
    default: return "LIBUSB_ERROR_OTHER";
    }
}

const char *libusb_strerror(int code) { return libusb_error_name(code); }

/* ------------------------------------------------------------------ */
/* Enumeration                                                        */
/* ------------------------------------------------------------------ */

ssize_t libusb_get_device_list(libusb_context *ctx, libusb_device ***list)
{
    if (!ctx) ctx = default_ctx;
    if (!ctx) return LIBUSB_ERROR_INVALID_PARAM;

    /* Drain any pending events so device_present is current, then, if we still
     * don't have descriptors, ask the broker explicitly. */
    struct pollfd pfd = { .fd = ctx->broker_fd, .events = POLLIN };
    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        if (pump_one(ctx, 0, NULL, NULL, 0, NULL) < 0) break;
    }
    if (!ctx->device_present) {
        uint32_t tag = ctx->next_tag++;
        send_request(ctx, BROKER_GET_DESCRIPTORS, tag, NULL, 0);
        uint8_t buf[18 + CFG_DESC_MAX]; uint32_t got = 0; int32_t st = 0;
        if (wait_reply(ctx, tag, &st, buf, sizeof(buf), &got) == 0 && st == 0 && got >= 18) {
            memcpy(ctx->dev_desc, buf, 18);
            ctx->cfg_len = (got - 18 > CFG_DESC_MAX) ? CFG_DESC_MAX : (uint16_t)(got - 18);
            memcpy(ctx->cfg_desc, buf + 18, ctx->cfg_len);
            ctx->device_present = true;
        }
    }

    libusb_device **arr = calloc(2, sizeof(*arr));
    ssize_t n = 0;
    if (ctx->device_present) {
        struct libusb_device *d = calloc(1, sizeof(*d));
        d->ctx = ctx; d->refcnt = 1;
        arr[n++] = d;
    }
    arr[n] = NULL;
    *list = arr;
    return n;
}

void libusb_free_device_list(libusb_device **list, int unref_devices)
{
    if (!list) return;
    for (int i = 0; list[i]; i++) if (unref_devices) libusb_unref_device(list[i]);
    free(list);
}

libusb_device *libusb_ref_device(libusb_device *dev) { if (dev) dev->refcnt++; return dev; }
void libusb_unref_device(libusb_device *dev) { if (dev && --dev->refcnt <= 0) free(dev); }

int libusb_get_device_descriptor(libusb_device *dev, struct libusb_device_descriptor *desc)
{
    if (!dev || !dev->ctx->device_present) return LIBUSB_ERROR_NO_DEVICE;
    parse_device_desc(dev->ctx->dev_desc, desc);
    return LIBUSB_SUCCESS;
}

int libusb_get_config_descriptor_by_value(libusb_device *dev, uint8_t value,
                                          struct libusb_config_descriptor **config)
{
    (void)value;
    if (!dev || !dev->ctx->device_present) return LIBUSB_ERROR_NO_DEVICE;
    struct libusb_config_descriptor *c = parse_config(dev->ctx->cfg_desc, dev->ctx->cfg_len);
    if (!c) return LIBUSB_ERROR_NOT_FOUND;
    *config = c;
    return LIBUSB_SUCCESS;
}

int libusb_get_config_descriptor(libusb_device *dev, uint8_t index,
                                 struct libusb_config_descriptor **config)
{
    (void)index;
    return libusb_get_config_descriptor_by_value(dev, 0, config);
}

int libusb_get_active_config_descriptor(libusb_device *dev,
                                        struct libusb_config_descriptor **config)
{
    return libusb_get_config_descriptor_by_value(dev, 0, config);
}

void libusb_free_config_descriptor(struct libusb_config_descriptor *config)
{
    if (!config) return;
    for (int i = 0; i < config->bNumInterfaces; i++) {
        const struct libusb_interface *itf = &config->interface[i];
        for (int a = 0; a < itf->num_altsetting; a++)
            free((void *)itf->altsetting[a].endpoint);
        free((void *)itf->altsetting);
    }
    free((void *)config->interface);
    free(config);
}

uint8_t libusb_get_bus_number(libusb_device *dev) { (void)dev; return 1; }
uint8_t libusb_get_device_address(libusb_device *dev) { (void)dev; return 2; }
uint8_t libusb_get_port_number(libusb_device *dev) { (void)dev; return 1; }

int libusb_get_device_speed(libusb_device *dev) { (void)dev; return LIBUSB_SPEED_HIGH; }

int libusb_get_max_packet_size(libusb_device *dev, unsigned char endpoint)
{
    (void)dev; (void)endpoint; return 512;
}

/* ------------------------------------------------------------------ */
/* Open / config / interface                                          */
/* ------------------------------------------------------------------ */

int libusb_open(libusb_device *dev, libusb_device_handle **handle)
{
    SDBG("libusb_open dev=%p present=%d\n", (void *)dev, dev ? dev->ctx->device_present : -1);
    if (!dev || !dev->ctx->device_present) return LIBUSB_ERROR_NO_DEVICE;
    struct libusb_device_handle *h = calloc(1, sizeof(*h));
    h->dev = libusb_ref_device(dev);
    h->ctx = dev->ctx;
    *handle = h;
    return LIBUSB_SUCCESS;
}

void libusb_close(libusb_device_handle *handle)
{
    if (!handle) return;
    libusb_unref_device(handle->dev);
    free(handle);
}

libusb_device *libusb_get_device(libusb_device_handle *handle) { return handle ? handle->dev : NULL; }

static int simple_cmd(struct libusb_context *ctx, uint32_t kind, const void *pl, uint32_t len)
{
    uint32_t tag = ctx->next_tag++;
    send_request(ctx, kind, tag, pl, len);
    int32_t st = 0;
    if (wait_reply(ctx, tag, &st, NULL, 0, NULL) < 0) return LIBUSB_ERROR_NO_DEVICE;
    return st < 0 ? LIBUSB_ERROR_IO : LIBUSB_SUCCESS;
}

int libusb_get_configuration(libusb_device_handle *handle, int *config)
{
    /* Report UNCONFIGURED so clients (usbmuxd) always issue a real
     * SET_CONFIGURATION, which the broker forwards to the device to activate
     * its endpoints. Claiming it is already configured skips that step and
     * leaves the mux endpoints inert (every bulk transfer NAKs). */
    (void)handle; if (config) *config = 0; return LIBUSB_SUCCESS;
}

int libusb_set_configuration(libusb_device_handle *handle, int configuration)
{
    uint32_t cfg = (uint32_t)configuration;
    return simple_cmd(handle->ctx, BROKER_SET_CONFIG, &cfg, sizeof(cfg));
}

int libusb_claim_interface(libusb_device_handle *handle, int interface_number)
{
    uint32_t i = (uint32_t)interface_number;
    return simple_cmd(handle->ctx, BROKER_CLAIM_INTERFACE, &i, sizeof(i));
}

int libusb_release_interface(libusb_device_handle *handle, int interface_number)
{
    uint32_t i = (uint32_t)interface_number;
    return simple_cmd(handle->ctx, BROKER_RELEASE_INTERFACE, &i, sizeof(i));
}

int libusb_set_interface_alt_setting(libusb_device_handle *handle, int interface_number,
                                     int alternate_setting)
{
    uint32_t v[2] = { (uint32_t)interface_number, (uint32_t)alternate_setting };
    return simple_cmd(handle->ctx, BROKER_SET_ALT, v, sizeof(v));
}

int libusb_clear_halt(libusb_device_handle *handle, unsigned char endpoint)
{
    uint32_t ep = endpoint;
    return simple_cmd(handle->ctx, BROKER_CLEAR_HALT, &ep, sizeof(ep));
}

int libusb_reset_device(libusb_device_handle *handle)
{
    return simple_cmd(handle->ctx, BROKER_RESET, NULL, 0);
}

int libusb_kernel_driver_active(libusb_device_handle *handle, int interface_number)
{ (void)handle; (void)interface_number; return 0; }
int libusb_detach_kernel_driver(libusb_device_handle *handle, int interface_number)
{ (void)handle; (void)interface_number; return LIBUSB_SUCCESS; }
int libusb_attach_kernel_driver(libusb_device_handle *handle, int interface_number)
{ (void)handle; (void)interface_number; return LIBUSB_SUCCESS; }
int libusb_set_auto_detach_kernel_driver(libusb_device_handle *handle, int enable)
{ (void)handle; (void)enable; return LIBUSB_SUCCESS; }

/* ------------------------------------------------------------------ */
/* Synchronous transfers                                              */
/* ------------------------------------------------------------------ */

int libusb_control_transfer(libusb_device_handle *handle, uint8_t bmRequestType,
                            uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
                            unsigned char *data, uint16_t wLength, unsigned int timeout)
{
    struct libusb_context *ctx = handle->ctx;
    bool in = (bmRequestType & 0x80) != 0;
    broker_control_req req = { 0 };
    usb_setup_packet sp = { bmRequestType, bRequest, wValue, wIndex, wLength };
    memcpy(req.setup, &sp, sizeof(sp));
    req.timeout_ms = timeout;

    uint32_t tag = ctx->next_tag++;
    uint32_t plen = sizeof(req) + (in ? 0 : wLength);
    uint8_t *pl = malloc(plen);
    memcpy(pl, &req, sizeof(req));
    if (!in && wLength) memcpy(pl + sizeof(req), data, wLength);
    send_request(ctx, BROKER_CONTROL, tag, pl, plen);
    free(pl);

    int32_t st = 0; uint32_t got = 0;
    if (wait_reply(ctx, tag, &st, in ? data : NULL, wLength, &got) < 0)
        return LIBUSB_ERROR_NO_DEVICE;
    if (st < 0) return (st == TCP_USB_RET_STALL) ? LIBUSB_ERROR_PIPE : LIBUSB_ERROR_IO;
    return in ? (int)got : (int)wLength;
}

static int sync_bulk(libusb_device_handle *handle, unsigned char ep, unsigned char *data,
                     int length, int *transferred, unsigned int timeout, uint8_t type)
{
    struct libusb_context *ctx = handle->ctx;
    bool in = (ep & 0x80) != 0;
    broker_transfer_req req = { 0 };
    req.ep = ep; req.type = type; req.length = (uint32_t)length; req.timeout_ms = timeout;

    uint32_t tag = ctx->next_tag++;
    uint32_t plen = sizeof(req) + (in ? 0 : (uint32_t)length);
    uint8_t *pl = malloc(plen);
    memcpy(pl, &req, sizeof(req));
    if (!in && length) memcpy(pl + sizeof(req), data, length);
    send_request(ctx, BROKER_BULK, tag, pl, plen);
    free(pl);

    int32_t st = 0; uint32_t got = 0;
    if (wait_reply(ctx, tag, &st, in ? data : NULL, (uint32_t)length, &got) < 0)
        return LIBUSB_ERROR_NO_DEVICE;
    if (st < 0) return (st == TCP_USB_RET_STALL) ? LIBUSB_ERROR_PIPE : LIBUSB_ERROR_IO;
    if (transferred) *transferred = in ? (int)got : length;
    return LIBUSB_SUCCESS;
}

int libusb_bulk_transfer(libusb_device_handle *handle, unsigned char endpoint,
                         unsigned char *data, int length, int *transferred, unsigned int timeout)
{
    return sync_bulk(handle, endpoint, data, length, transferred, timeout, LIBUSB_TRANSFER_TYPE_BULK);
}

int libusb_interrupt_transfer(libusb_device_handle *handle, unsigned char endpoint,
                              unsigned char *data, int length, int *transferred, unsigned int timeout)
{
    return sync_bulk(handle, endpoint, data, length, transferred, timeout, LIBUSB_TRANSFER_TYPE_INTERRUPT);
}

int libusb_get_string_descriptor_ascii(libusb_device_handle *handle, uint8_t desc_index,
                                       unsigned char *data, int length)
{
    unsigned char tmp[255];
    /* Language IDs (index 0). */
    int r = libusb_control_transfer(handle, LIBUSB_ENDPOINT_IN, LIBUSB_REQUEST_GET_DESCRIPTOR,
                                    (uint16_t)((LIBUSB_DT_STRING << 8) | 0), 0, tmp, sizeof(tmp), 1000);
    if (r < 4) return r < 0 ? r : LIBUSB_ERROR_IO;
    uint16_t langid = (uint16_t)(tmp[2] | (tmp[3] << 8));
    r = libusb_control_transfer(handle, LIBUSB_ENDPOINT_IN, LIBUSB_REQUEST_GET_DESCRIPTOR,
                                (uint16_t)((LIBUSB_DT_STRING << 8) | desc_index), langid,
                                tmp, sizeof(tmp), 1000);
    if (r < 2) return r < 0 ? r : LIBUSB_ERROR_IO;
    if (tmp[1] != LIBUSB_DT_STRING) return LIBUSB_ERROR_IO;
    int di = 0;
    for (int si = 2; si < tmp[0] && si < r; si += 2) {
        if (di >= length - 1) break;
        if (tmp[si + 1]) data[di++] = '?';
        else data[di++] = tmp[si];
    }
    data[di] = 0;
    return di;
}

/* ------------------------------------------------------------------ */
/* Asynchronous transfers + event loop                                */
/* ------------------------------------------------------------------ */

struct libusb_transfer *libusb_alloc_transfer(int iso_packets)
{
    size_t sz = sizeof(struct libusb_transfer)
              + iso_packets * sizeof(struct libusb_iso_packet_descriptor);
    struct libusb_transfer *x = calloc(1, sz);
    return x;
}

void libusb_free_transfer(struct libusb_transfer *transfer)
{
    if (!transfer) return;
    if ((transfer->flags & LIBUSB_TRANSFER_FREE_BUFFER) && transfer->buffer)
        free(transfer->buffer);
    free(transfer);
}

int libusb_submit_transfer(struct libusb_transfer *transfer)
{
    struct libusb_device_handle *h = transfer->dev_handle;
    struct libusb_context *ctx = h->ctx;
    SDBG("submit_transfer type=%d ep=0x%x len=%d fd=%d\n",
         transfer->type, transfer->endpoint, transfer->length, ctx->broker_fd);

    int slot = -1;
    for (int i = 0; i < MAX_PENDING; i++) if (!ctx->pend[i].active) { slot = i; break; }
    if (slot < 0) return LIBUSB_ERROR_NO_MEM;

    uint32_t tag = ctx->next_tag++;
    ctx->pend[slot].tag = tag;
    ctx->pend[slot].xfer = transfer;
    ctx->pend[slot].active = true;

    if (transfer->type == LIBUSB_TRANSFER_TYPE_CONTROL) {
        struct libusb_control_setup *s = libusb_control_transfer_get_setup(transfer);
        bool in = (s->bmRequestType & 0x80) != 0;
        broker_transfer_req req = { 0 };
        req.ep = 0; req.type = LIBUSB_TRANSFER_TYPE_CONTROL;
        req.length = s->wLength; req.timeout_ms = transfer->timeout;
        memcpy(req.setup, s, sizeof(*s));
        uint32_t plen = sizeof(req) + (in ? 0 : s->wLength);
        uint8_t *pl = malloc(plen);
        memcpy(pl, &req, sizeof(req));
        if (!in && s->wLength)
            memcpy(pl + sizeof(req), libusb_control_transfer_get_data(transfer), s->wLength);
        send_request(ctx, BROKER_SUBMIT_ASYNC, tag, pl, plen);
        free(pl);
    } else {
        bool in = (transfer->endpoint & 0x80) != 0;
        broker_transfer_req req = { 0 };
        req.ep = transfer->endpoint; req.type = LIBUSB_TRANSFER_TYPE_BULK;
        req.length = (uint32_t)transfer->length; req.timeout_ms = transfer->timeout;
        uint32_t plen = sizeof(req) + (in ? 0 : (uint32_t)transfer->length);
        uint8_t *pl = malloc(plen);
        memcpy(pl, &req, sizeof(req));
        if (!in && transfer->length) memcpy(pl + sizeof(req), transfer->buffer, transfer->length);
        send_request(ctx, BROKER_SUBMIT_ASYNC, tag, pl, plen);
        free(pl);
    }
    return LIBUSB_SUCCESS;
}

int libusb_cancel_transfer(struct libusb_transfer *transfer)
{
    struct libusb_context *ctx = transfer->dev_handle->ctx;
    for (int i = 0; i < MAX_PENDING; i++) {
        if (ctx->pend[i].active && ctx->pend[i].xfer == transfer) {
            uint32_t tag = ctx->pend[i].tag;
            send_request(ctx, BROKER_CANCEL_ASYNC, tag, &tag, sizeof(tag));
            return LIBUSB_SUCCESS;
        }
    }
    return LIBUSB_ERROR_NOT_FOUND;
}

static int handle_events(struct libusb_context *ctx, int timeout_ms)
{
    struct pollfd pfd = { .fd = ctx->broker_fd, .events = POLLIN };
    int pr = poll(&pfd, 1, timeout_ms);
    if (pr < 0) return (errno == EINTR) ? LIBUSB_SUCCESS : LIBUSB_ERROR_IO;
    if (pr == 0) return LIBUSB_SUCCESS;
    if (pump_one(ctx, 0, NULL, NULL, 0, NULL) < 0) return LIBUSB_ERROR_NO_DEVICE;
    /* Drain anything else immediately available. */
    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN))
        if (pump_one(ctx, 0, NULL, NULL, 0, NULL) < 0) break;
    return LIBUSB_SUCCESS;
}

int libusb_handle_events_timeout(libusb_context *ctx, struct timeval *tv)
{
    if (!ctx) ctx = default_ctx;
    int ms = tv ? (int)(tv->tv_sec * 1000 + tv->tv_usec / 1000) : 0;
    return handle_events(ctx, ms);
}

int libusb_handle_events_timeout_completed(libusb_context *ctx, struct timeval *tv, int *completed)
{
    if (!ctx) ctx = default_ctx;
    if (completed && *completed) return LIBUSB_SUCCESS;
    return libusb_handle_events_timeout(ctx, tv);
}

int libusb_handle_events(libusb_context *ctx)
{
    if (!ctx) ctx = default_ctx;
    return handle_events(ctx, 60000);
}

int libusb_handle_events_completed(libusb_context *ctx, int *completed)
{
    if (!ctx) ctx = default_ctx;
    if (completed && *completed) return LIBUSB_SUCCESS;
    return handle_events(ctx, 60000);
}

int libusb_get_next_timeout(libusb_context *ctx, struct timeval *tv)
{
    (void)ctx; (void)tv; return 0; /* no internal timeouts to report */
}

const struct libusb_pollfd **libusb_get_pollfds(libusb_context *ctx)
{
    if (!ctx) ctx = default_ctx;
    const struct libusb_pollfd **arr = calloc(2, sizeof(*arr));
    struct libusb_pollfd *p = calloc(1, sizeof(*p));
    p->fd = ctx->broker_fd; p->events = POLLIN;
    arr[0] = p; arr[1] = NULL;
    return arr;
}

void libusb_free_pollfds(const struct libusb_pollfd **pollfds)
{
    if (!pollfds) return;
    for (int i = 0; pollfds[i]; i++) free((void *)pollfds[i]);
    free((void *)pollfds);
}

void libusb_set_pollfd_notifiers(libusb_context *ctx, libusb_pollfd_added_cb added_cb,
                                 libusb_pollfd_removed_cb removed_cb, void *user_data)
{ (void)ctx; (void)added_cb; (void)removed_cb; (void)user_data; }

/* ------------------------------------------------------------------ */
/* Hotplug — advertised unsupported; provide stubs so linkage succeeds */
/* ------------------------------------------------------------------ */

int libusb_hotplug_register_callback(libusb_context *ctx, int events, int flags,
                                     int vendor_id, int product_id, int dev_class,
                                     libusb_hotplug_callback_fn cb_fn, void *user_data,
                                     libusb_hotplug_callback_handle *handle)
{
    (void)ctx; (void)events; (void)flags; (void)vendor_id; (void)product_id;
    (void)dev_class; (void)cb_fn; (void)user_data; (void)handle;
    return LIBUSB_ERROR_NOT_SUPPORTED;
}

void libusb_hotplug_deregister_callback(libusb_context *ctx, libusb_hotplug_callback_handle handle)
{ (void)ctx; (void)handle; }
