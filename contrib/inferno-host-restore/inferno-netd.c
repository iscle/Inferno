/*
 * inferno-netd — reverse-tethering / internet-sharing for the Inferno emulated
 * iPhone, entirely in userspace on macOS (arm64). No root, no pf, no macOS
 * Internet Sharing.
 *
 * Topology:
 *
 *   [emulated iPhone USB] -> [inferno-usbd broker] <- (this) inferno-netd
 *                                                        |
 *                                                     libslirp (NAT+DHCP+DNS)
 *                                                        |
 *                                                    host network
 *
 * usbmuxd (run separately with USBMUXD_DEFAULT_DEVICE_MODE=3) switches the
 * device into USB config 5, which exposes a CDC-NCM ethernet function next to
 * the usbmux interface. inferno-netd is a second broker client: it claims the
 * CDC-NCM comm+data interfaces, speaks NCM (NTB16) to move ethernet frames
 * to/from the device, and hands them to libslirp for user-mode NAT. libslirp's
 * built-in DHCP server hands the device 10.0.2.15 and gives it internet.
 *
 * This tool talks the inferno-usbd broker wire protocol directly
 * (broker_proto.h); it does NOT depend on the libusb shim.
 *
 * Build (macOS arm64, libslirp from Homebrew):
 *
 *   cc -O2 -Wall -I libusb-shim $(pkg-config --cflags --libs slirp) \
 *      inferno-netd.c -o inferno-netd
 *
 * or simply `make inferno-netd`.
 *
 * Copyright (c) 2026 Inferno host-restore contributors.
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include <errno.h>
#include <poll.h>
#include <signal.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/un.h>

#include "broker_proto.h"
#include <libslirp.h>

/* ------------------------------------------------------------------ */
/* Logging                                                            */
/* ------------------------------------------------------------------ */

static int g_verbose = -1;

static void logf(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    fprintf(stderr, "[netd] ");
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fputc('\n', stderr);
    fflush(stderr);
}

#define VLOG(...)                                                         \
    do {                                                                  \
        if (g_verbose < 0) g_verbose = getenv("INFERNO_NETD_VERBOSE") ? 1 : 0; \
        if (g_verbose) logf(__VA_ARGS__);                                 \
    } while (0)

/* ------------------------------------------------------------------ */
/* Little-endian helpers (device wire format is LE)                   */
/* ------------------------------------------------------------------ */

static uint16_t rd16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}
static void wr16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }
static void wr32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

/* ------------------------------------------------------------------ */
/* NCM constants                                                      */
/* ------------------------------------------------------------------ */

/* Signatures, read as little-endian uint32 from the byte stream. */
#define NCM_NTH16_SIG 0x484D434Eu /* "NCMH" */
#define NCM_NDP16_SIG 0x304D434Eu /* "NCM0" */

/* CDC class-specific requests (bRequest). */
#define NCM_GET_NTB_PARAMETERS   0x80
#define NCM_SET_NTB_INPUT_SIZE   0x86
#define CDC_SET_ETHERNET_FILTER  0x43

/* Ethernet packet filter bits for SET_ETHERNET_PACKET_FILTER wValue. */
#define ETH_FILTER_PROMISCUOUS   0x0001
#define ETH_FILTER_ALL_MULTICAST 0x0002
#define ETH_FILTER_DIRECTED      0x0004
#define ETH_FILTER_BROADCAST     0x0008

/* USB class codes / subclass. */
#define USB_CLASS_COMM 0x02
#define USB_SUBCLASS_NCM 0x0d
#define USB_CLASS_DATA 0x0a

/* Tunables. */
#define NUM_RX_INFLIGHT 4       /* bulk-IN transfers kept outstanding */
#define MAX_PEND 64             /* async transfer slots */
#define MAX_POLLFDS 512         /* poll set capacity (broker + slirp sockets) */
#define CFG_DESC_MAX 4096
#define RX_NTB_CAP 16384        /* cap on the RX NTB size we request */
#define TX_NTB_FALLBACK 2048

/* ------------------------------------------------------------------ */
/* Timer list backing the SlirpCb timer callbacks                     */
/* ------------------------------------------------------------------ */

struct sl_timer {
    SlirpTimerCb cb;
    void *cb_opaque;
    int64_t expire_ms;   /* virtual-clock ms; <0 = disarmed */
    bool active;
    struct sl_timer *next;
};

/* ------------------------------------------------------------------ */
/* NCM NTB parameters (from GET_NTB_PARAMETERS)                       */
/* ------------------------------------------------------------------ */

struct ntb_params {
    uint32_t dwNtbInMaxSize;
    uint32_t dwNtbOutMaxSize;
    uint16_t wNdpInDivisor;
    uint16_t wNdpInPayloadRemainder;
    uint16_t wNdpInAlignment;
    uint16_t wNdpOutDivisor;
    uint16_t wNdpOutPayloadRemainder;
    uint16_t wNdpOutAlignment;
    uint16_t wNtbOutMaxDatagrams;
    uint16_t bmNtbFormatsSupported;
};

/* ------------------------------------------------------------------ */
/* netd context                                                       */
/* ------------------------------------------------------------------ */

enum { PEND_FREE = 0, PEND_RX, PEND_TX };

struct netd {
    int broker_fd;
    uint32_t next_tag;

    /* Cached descriptors from BROKER_EVENT_ARRIVED / GET_DESCRIPTORS. */
    bool device_present;
    bool device_left;
    uint8_t dev_desc[18];
    uint8_t cfg_desc[CFG_DESC_MAX];
    uint16_t cfg_len;

    /* Discovered CDC-NCM function. */
    uint8_t comm_iface;
    uint8_t data_iface;
    uint8_t data_alt;        /* alt setting with the bulk endpoints (usually 1) */
    uint8_t ep_in;           /* bulk IN endpoint address (0x8x) */
    uint8_t ep_out;          /* bulk OUT endpoint address */
    uint16_t ep_in_mps;
    uint16_t ep_out_mps;
    uint8_t cfg_value;       /* active bConfigurationValue */

    struct ntb_params ntb;
    uint32_t rx_ntb_size;    /* length requested for each bulk-IN */
    uint32_t tx_ntb_max;     /* max NTB we may send on bulk-OUT */
    uint16_t tx_seq;         /* NTB wSequence counter */

    /* Async transfer bookkeeping. */
    struct { uint32_t tag; int kind; } pend[MAX_PEND];
    int rx_inflight;

    /* slirp. */
    Slirp *slirp;
    struct sl_timer *timers;

    /* poll set; index 0 is always the broker fd. */
    struct pollfd pfds[MAX_POLLFDS];
    int npfds;

    /* stats */
    uint64_t rx_frames;
    uint64_t tx_frames;
};

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

static void send_request(struct netd *nd, uint32_t kind, uint32_t tag,
                         const void *payload, uint32_t len)
{
    broker_msg_hdr h = { .kind = kind, .tag = tag, .status = 0, .length = len };
    io_write(nd->broker_fd, &h, sizeof(h));
    if (len && payload) io_write(nd->broker_fd, payload, len);
}

/* ------------------------------------------------------------------ */
/* Forward declarations                                               */
/* ------------------------------------------------------------------ */

static void ncm_rx_parse(struct netd *nd, const uint8_t *buf, uint32_t len);
static void rx_topup(struct netd *nd);

/* ------------------------------------------------------------------ */
/* Async pending-slot bookkeeping                                     */
/* ------------------------------------------------------------------ */

static int pend_alloc(struct netd *nd, uint32_t tag, int kind)
{
    for (int i = 0; i < MAX_PEND; i++) {
        if (nd->pend[i].kind == PEND_FREE) {
            nd->pend[i].tag = tag;
            nd->pend[i].kind = kind;
            return i;
        }
    }
    return -1;
}

static void handle_async(struct netd *nd, uint32_t tag, int32_t status,
                         const uint8_t *data, uint32_t dlen)
{
    for (int i = 0; i < MAX_PEND; i++) {
        if (nd->pend[i].kind == PEND_FREE || nd->pend[i].tag != tag) continue;
        int kind = nd->pend[i].kind;
        nd->pend[i].kind = PEND_FREE;
        if (kind == PEND_RX) {
            nd->rx_inflight--;
            if (status < 0) {
                VLOG("bulk-IN completed with status %d", status);
            } else {
                nd->rx_frames++;
                ncm_rx_parse(nd, data, dlen);
            }
            rx_topup(nd);
        } else { /* PEND_TX */
            if (status < 0) VLOG("bulk-OUT completed with status %d", status);
        }
        return;
    }
    VLOG("async completion for unknown tag %u (status %d)", tag, status);
}

/* Store descriptors from an ARRIVED event or GET_DESCRIPTORS reply. */
static void store_descriptors(struct netd *nd, const uint8_t *pl, uint32_t len)
{
    if (len < 18) return;
    memcpy(nd->dev_desc, pl, 18);
    uint32_t clen = len - 18;
    if (clen > CFG_DESC_MAX) clen = CFG_DESC_MAX;
    memcpy(nd->cfg_desc, pl + 18, clen);
    nd->cfg_len = (uint16_t)clen;
    nd->device_present = true;
}

/*
 * Read and dispatch exactly one broker message. If want_tag != 0, returns 0
 * when the matching BROKER_REPLY arrives (filling out_*). Returns -1 on broker
 * loss, 1 when some other message was handled.
 */
static int pump_one(struct netd *nd, uint32_t want_tag, int32_t *out_status,
                    uint8_t *out_buf, uint32_t out_cap, uint32_t *out_len)
{
    broker_msg_hdr h;
    int r = io_read(nd->broker_fd, &h, sizeof(h));
    if (r <= 0) return -1;

    uint8_t stackbuf[256];
    uint8_t *pl = NULL;
    if (h.length) {
        pl = (h.length <= sizeof(stackbuf)) ? stackbuf : malloc(h.length);
        if (!pl) return -1;
        if (io_read(nd->broker_fd, pl, h.length) <= 0) {
            if (pl != stackbuf) free(pl);
            return -1;
        }
    }

    int rc = 1;
    switch (h.kind) {
    case BROKER_EVENT_ARRIVED:
        store_descriptors(nd, pl, h.length);
        VLOG("event: device arrived (%u desc bytes)", h.length);
        break;
    case BROKER_EVENT_LEFT:
        nd->device_left = true;
        logf("event: device left");
        break;
    case BROKER_ASYNC_COMPLETE:
        handle_async(nd, h.tag, h.status, pl, h.length);
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

/* Block until the REPLY for tag arrives, dispatching other traffic. */
static int wait_reply(struct netd *nd, uint32_t tag, int32_t *status,
                      uint8_t *buf, uint32_t cap, uint32_t *len)
{
    for (;;) {
        int r = pump_one(nd, tag, status, buf, cap, len);
        if (r <= 0) return r; /* 0 = got reply, -1 = broker loss */
    }
}

/* Drain any messages already available without blocking. */
static void drain_nonblocking(struct netd *nd)
{
    struct pollfd pfd = { .fd = nd->broker_fd, .events = POLLIN };
    while (poll(&pfd, 1, 0) > 0 && (pfd.revents & POLLIN)) {
        if (pump_one(nd, 0, NULL, NULL, 0, NULL) < 0) break;
    }
}

/* ------------------------------------------------------------------ */
/* Simple broker commands                                             */
/* ------------------------------------------------------------------ */

static int simple_cmd(struct netd *nd, uint32_t kind, const void *pl, uint32_t len)
{
    uint32_t tag = nd->next_tag++;
    send_request(nd, kind, tag, pl, len);
    int32_t st = 0;
    if (wait_reply(nd, tag, &st, NULL, 0, NULL) < 0) return -1;
    return st < 0 ? -1 : 0;
}

/* Synchronous control transfer against the comm interface. */
static int broker_control(struct netd *nd, uint8_t bmRequestType, uint8_t bRequest,
                          uint16_t wValue, uint16_t wIndex, uint8_t *data,
                          uint16_t wLength, uint8_t *out_buf, uint32_t out_cap,
                          uint32_t *out_len)
{
    bool in = (bmRequestType & 0x80) != 0;
    broker_control_req req = { 0 };
    req.setup[0] = bmRequestType;
    req.setup[1] = bRequest;
    wr16(req.setup + 2, wValue);
    wr16(req.setup + 4, wIndex);
    wr16(req.setup + 6, wLength);
    req.timeout_ms = 1000;

    uint32_t plen = sizeof(req) + (in ? 0 : wLength);
    uint8_t *pl = malloc(plen);
    if (!pl) return -1;
    memcpy(pl, &req, sizeof(req));
    if (!in && wLength && data) memcpy(pl + sizeof(req), data, wLength);

    uint32_t tag = nd->next_tag++;
    send_request(nd, BROKER_CONTROL, tag, pl, plen);
    free(pl);

    int32_t st = 0;
    if (wait_reply(nd, tag, &st, in ? out_buf : NULL, out_cap, out_len) < 0) return -1;
    return st < 0 ? -1 : 0;
}

/* Submit an async bulk transfer. For OUT, data/len carry the payload. */
static int submit_bulk(struct netd *nd, uint8_t ep, uint32_t length,
                       const uint8_t *data, int kind)
{
    bool in = (ep & 0x80) != 0;
    broker_transfer_req req = { 0 };
    req.ep = ep;
    req.type = BROKER_XFER_BULK;
    req.length = length;
    req.timeout_ms = 0; /* no timeout; RX transfers park until data arrives */

    uint32_t plen = sizeof(req) + (in ? 0 : length);
    uint8_t *pl = malloc(plen);
    if (!pl) return -1;
    memcpy(pl, &req, sizeof(req));
    if (!in && length && data) memcpy(pl + sizeof(req), data, length);

    uint32_t tag = nd->next_tag++;
    if (pend_alloc(nd, tag, kind) < 0) {
        free(pl);
        logf("pend table full; dropping transfer");
        return -1;
    }
    send_request(nd, BROKER_SUBMIT_ASYNC, tag, pl, plen);
    free(pl);
    return 0;
}

/* Keep NUM_RX_INFLIGHT bulk-IN transfers outstanding. */
static void rx_topup(struct netd *nd)
{
    while (nd->rx_inflight < NUM_RX_INFLIGHT) {
        if (submit_bulk(nd, nd->ep_in, nd->rx_ntb_size, NULL, PEND_RX) < 0) break;
        nd->rx_inflight++;
    }
}

/* ------------------------------------------------------------------ */
/* NCM RX: parse an NTB16 and feed datagrams to slirp                 */
/* ------------------------------------------------------------------ */

static void ncm_rx_parse(struct netd *nd, const uint8_t *buf, uint32_t len)
{
    if (len < 12) {
        VLOG("RX NTB too short (%u bytes)", len);
        return;
    }
    if (rd32(buf) != NCM_NTH16_SIG) {
        VLOG("RX NTB bad NTH signature 0x%08x", rd32(buf));
        return;
    }
    uint16_t wBlockLength = rd16(buf + 8);
    uint32_t ndp = rd16(buf + 10); /* wNdpIndex */
    VLOG("RX NTB: block=%u ndpIndex=%u (got %u bytes)", wBlockLength, ndp, len);

    int guard = 0;
    while (ndp != 0 && ndp + 8 <= len) {
        if (++guard > 64) break; /* defend against cyclic wNextNdpIndex */
        if (rd32(buf + ndp) != NCM_NDP16_SIG) {
            VLOG("RX NDP bad signature 0x%08x at %u", rd32(buf + ndp), ndp);
            break;
        }
        uint16_t wLength = rd16(buf + ndp + 4);
        uint16_t wNext = rd16(buf + ndp + 6);
        uint32_t end = ndp + wLength;
        if (end > len) end = len;

        uint32_t off = ndp + 8; /* first (index,length) pair */
        while (off + 4 <= end) {
            uint16_t di = rd16(buf + off);
            uint16_t dl = rd16(buf + off + 2);
            off += 4;
            if (di == 0 && dl == 0) break; /* terminator */
            if (dl == 0) continue;
            if ((uint32_t)di + dl > len) {
                VLOG("RX datagram out of range (idx=%u len=%u)", di, dl);
                continue;
            }
            VLOG("RX frame %u bytes -> slirp", dl);
            slirp_input(nd->slirp, buf + di, (int)dl);
        }
        ndp = wNext;
    }
}

/* ------------------------------------------------------------------ */
/* NCM TX: wrap an ethernet frame in a minimal NTB16 and send it      */
/* ------------------------------------------------------------------ */

/*
 * Fixed, widely-compatible NTB16 layout for a single datagram:
 *   [0]  NTH16   (12 bytes)
 *   [12] NDP16   (16 bytes: sig, wLength=16, wNext=0, one (idx,len) + (0,0))
 *   [28] datagram (28 is already 4-byte aligned)
 */
#define TX_NTH_LEN 12
#define TX_NDP_OFF 12
#define TX_NDP_LEN 16
#define TX_DGRAM_OFF 28

static void ncm_tx_frame(struct netd *nd, const uint8_t *frame, uint32_t frame_len)
{
    uint32_t ntb_len = TX_DGRAM_OFF + frame_len;
    if (ntb_len > nd->tx_ntb_max) {
        VLOG("TX frame too large (%u > NTB max %u); dropping", ntb_len, nd->tx_ntb_max);
        return;
    }
    uint8_t *buf = calloc(1, ntb_len);
    if (!buf) return;

    /* NTH16 */
    wr32(buf + 0, NCM_NTH16_SIG);
    wr16(buf + 4, TX_NTH_LEN);        /* wHeaderLength */
    wr16(buf + 6, nd->tx_seq++);      /* wSequence */
    wr16(buf + 8, (uint16_t)ntb_len); /* wBlockLength */
    wr16(buf + 10, TX_NDP_OFF);       /* wNdpIndex */

    /* NDP16 */
    wr32(buf + TX_NDP_OFF + 0, NCM_NDP16_SIG);
    wr16(buf + TX_NDP_OFF + 4, TX_NDP_LEN); /* wLength */
    wr16(buf + TX_NDP_OFF + 6, 0);          /* wNextNdpIndex */
    wr16(buf + TX_NDP_OFF + 8, TX_DGRAM_OFF);        /* wDatagramIndex */
    wr16(buf + TX_NDP_OFF + 10, (uint16_t)frame_len); /* wDatagramLength */
    wr16(buf + TX_NDP_OFF + 12, 0);         /* terminator index */
    wr16(buf + TX_NDP_OFF + 14, 0);         /* terminator length */

    memcpy(buf + TX_DGRAM_OFF, frame, frame_len);

    VLOG("TX frame %u bytes -> device (NTB %u)", frame_len, ntb_len);
    submit_bulk(nd, nd->ep_out, ntb_len, buf, PEND_TX);
    nd->tx_frames++;
    free(buf); /* send_request already copied the payload onto the socket */
}

/* ------------------------------------------------------------------ */
/* CDC-NCM function discovery in the active config descriptor         */
/* ------------------------------------------------------------------ */

/*
 * Walk the active config descriptor and locate the CDC-NCM function:
 *   - a Communications-class interface (0x02) with subclass NCM (0x0d)
 *   - its paired Data-class interface (0x0a) whose alt 1 has bulk IN+OUT.
 * Returns 0 on success, -1 if no NCM function is present.
 */
static int discover_ncm(struct netd *nd)
{
    const uint8_t *b = nd->cfg_desc;
    uint16_t len = nd->cfg_len;
    if (len < 9) return -1;

    nd->cfg_value = b[5];
    logf("active config bConfigurationValue=%u, %u interfaces, %u desc bytes",
         nd->cfg_value, b[4], len);

    bool found_comm = false;
    bool found_data = false;
    int comm_iface = -1, data_iface = -1;

    /* Track the interface we are currently walking endpoints for. */
    int cur_alt = -1, cur_class = -1;
    uint8_t alt1_in = 0, alt1_out = 0;
    uint16_t alt1_in_mps = 0, alt1_out_mps = 0;

    uint16_t off = b[0]; /* skip config header */
    while ((uint32_t)off + 2 <= len) {
        uint8_t blen = b[off], btype = b[off + 1];
        if (blen == 0 || (uint32_t)off + blen > len) break;

        if (btype == 0x04 /* INTERFACE */ && blen >= 9) {
            uint8_t inum = b[off + 2];
            uint8_t alt = b[off + 3];
            uint8_t iclass = b[off + 5];
            uint8_t isub = b[off + 6];

            cur_alt = alt;
            cur_class = iclass;

            if (iclass == USB_CLASS_COMM && isub == USB_SUBCLASS_NCM) {
                comm_iface = inum;
                found_comm = true;
                logf("found CDC-NCM comm interface %u (alt %u)", inum, alt);
            } else if (iclass == USB_CLASS_DATA) {
                data_iface = inum;
                found_data = true;
                if (alt == 1)
                    VLOG("data interface %u alt 1 (%u endpoints)", inum, b[off + 4]);
            }
        } else if (btype == 0x05 /* ENDPOINT */ && blen >= 7) {
            /* Only care about the data interface's alt 1 bulk endpoints. */
            if (cur_class == USB_CLASS_DATA && cur_alt == 1) {
                uint8_t addr = b[off + 2];
                uint8_t attr = b[off + 3];
                uint16_t mps = rd16(b + off + 4);
                if ((attr & 0x03) == 0x02) { /* bulk */
                    if (addr & 0x80) { alt1_in = addr; alt1_in_mps = mps; }
                    else { alt1_out = addr; alt1_out_mps = mps; }
                }
            }
        }
        off += blen;
    }

    if (!found_comm || !found_data) {
        VLOG("no CDC-NCM function in config value %u", nd->cfg_value);
        return -1;
    }
    if (!alt1_in || !alt1_out) {
        logf("CDC-NCM data interface has no bulk IN/OUT endpoints on alt 1");
        return -1;
    }

    nd->comm_iface = (uint8_t)comm_iface;
    nd->data_iface = (uint8_t)data_iface;
    nd->data_alt = 1;
    nd->ep_in = alt1_in;
    nd->ep_out = alt1_out;
    nd->ep_in_mps = alt1_in_mps;
    nd->ep_out_mps = alt1_out_mps;

    logf("CDC-NCM: comm iface=%u data iface=%u alt=%u  IN=0x%02x (mps %u) OUT=0x%02x (mps %u)",
         nd->comm_iface, nd->data_iface, nd->data_alt,
         nd->ep_in, nd->ep_in_mps, nd->ep_out, nd->ep_out_mps);
    return 0;
}

/* ------------------------------------------------------------------ */
/* NCM control setup                                                  */
/* ------------------------------------------------------------------ */

static void ncm_defaults(struct netd *nd)
{
    nd->ntb.dwNtbInMaxSize = RX_NTB_CAP;
    nd->ntb.dwNtbOutMaxSize = TX_NTB_FALLBACK;
    nd->ntb.wNdpInDivisor = 4;
    nd->ntb.wNdpInAlignment = 4;
    nd->ntb.wNdpOutDivisor = 4;
    nd->ntb.wNdpOutAlignment = 4;
}

static int ncm_setup(struct netd *nd)
{
    ncm_defaults(nd);

    /* GET_NTB_PARAMETERS: class IN to the comm interface, 28-byte struct. */
    uint8_t p[28];
    uint32_t got = 0;
    if (broker_control(nd, 0xA1, NCM_GET_NTB_PARAMETERS, 0, nd->comm_iface,
                       NULL, sizeof(p), p, sizeof(p), &got) == 0 && got >= 28) {
        nd->ntb.bmNtbFormatsSupported     = rd16(p + 2);
        nd->ntb.dwNtbInMaxSize            = rd32(p + 4);
        nd->ntb.wNdpInDivisor             = rd16(p + 8);
        nd->ntb.wNdpInPayloadRemainder    = rd16(p + 10);
        nd->ntb.wNdpInAlignment           = rd16(p + 12);
        /* p+14: wReserved */
        nd->ntb.dwNtbOutMaxSize           = rd32(p + 16);
        nd->ntb.wNdpOutDivisor            = rd16(p + 20);
        nd->ntb.wNdpOutPayloadRemainder   = rd16(p + 22);
        nd->ntb.wNdpOutAlignment          = rd16(p + 24);
        nd->ntb.wNtbOutMaxDatagrams       = rd16(p + 26);
        logf("NTB params: InMax=%u OutMax=%u  In(div=%u rem=%u align=%u) "
             "Out(div=%u rem=%u align=%u) maxDgrams=%u formats=0x%04x",
             nd->ntb.dwNtbInMaxSize, nd->ntb.dwNtbOutMaxSize,
             nd->ntb.wNdpInDivisor, nd->ntb.wNdpInPayloadRemainder,
             nd->ntb.wNdpInAlignment, nd->ntb.wNdpOutDivisor,
             nd->ntb.wNdpOutPayloadRemainder, nd->ntb.wNdpOutAlignment,
             nd->ntb.wNtbOutMaxDatagrams, nd->ntb.bmNtbFormatsSupported);
    } else {
        logf("GET_NTB_PARAMETERS failed; using defaults (InMax=%u OutMax=%u)",
             nd->ntb.dwNtbInMaxSize, nd->ntb.dwNtbOutMaxSize);
    }

    /* Choose our RX NTB size and tell the device via SET_NTB_INPUT_SIZE. */
    nd->rx_ntb_size = nd->ntb.dwNtbInMaxSize;
    if (nd->rx_ntb_size > RX_NTB_CAP) nd->rx_ntb_size = RX_NTB_CAP;
    if (nd->rx_ntb_size < 2048) nd->rx_ntb_size = 2048;

    nd->tx_ntb_max = nd->ntb.dwNtbOutMaxSize ? nd->ntb.dwNtbOutMaxSize : TX_NTB_FALLBACK;

    uint8_t insz[4];
    wr32(insz, nd->rx_ntb_size);
    if (broker_control(nd, 0x21, NCM_SET_NTB_INPUT_SIZE, 0, nd->comm_iface,
                       insz, sizeof(insz), NULL, 0, NULL) < 0)
        logf("SET_NTB_INPUT_SIZE(%u) failed (continuing)", nd->rx_ntb_size);
    else
        logf("SET_NTB_INPUT_SIZE=%u", nd->rx_ntb_size);

    /* Enable RX: SET_ETHERNET_PACKET_FILTER, no data stage. */
    uint16_t filt = ETH_FILTER_PROMISCUOUS | ETH_FILTER_ALL_MULTICAST |
                    ETH_FILTER_DIRECTED | ETH_FILTER_BROADCAST;
    if (broker_control(nd, 0x21, CDC_SET_ETHERNET_FILTER, filt, nd->comm_iface,
                       NULL, 0, NULL, 0, NULL) < 0)
        logf("SET_ETHERNET_PACKET_FILTER failed (continuing)");
    else
        logf("SET_ETHERNET_PACKET_FILTER=0x%04x (RX enabled)", filt);

    return 0;
}

/* ------------------------------------------------------------------ */
/* slirp callbacks                                                    */
/* ------------------------------------------------------------------ */

static slirp_ssize_t cb_send_packet(const void *buf, size_t len, void *opaque)
{
    struct netd *nd = opaque;
    ncm_tx_frame(nd, buf, (uint32_t)len);
    return (slirp_ssize_t)len;
}

static void cb_guest_error(const char *msg, void *opaque)
{
    (void)opaque;
    logf("slirp guest error: %s", msg ? msg : "(null)");
}

static int64_t now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int64_t cb_clock_get_ns(void *opaque)
{
    (void)opaque;
    return now_ns();
}

static void *cb_timer_new(SlirpTimerCb cb, void *cb_opaque, void *opaque)
{
    struct netd *nd = opaque;
    struct sl_timer *t = calloc(1, sizeof(*t));
    if (!t) return NULL;
    t->cb = cb;
    t->cb_opaque = cb_opaque;
    t->expire_ms = -1;
    t->active = false;
    t->next = nd->timers;
    nd->timers = t;
    return t;
}

static void cb_timer_free(void *timer, void *opaque)
{
    struct netd *nd = opaque;
    struct sl_timer **pp = &nd->timers;
    while (*pp) {
        if (*pp == timer) {
            struct sl_timer *dead = *pp;
            *pp = dead->next;
            free(dead);
            return;
        }
        pp = &(*pp)->next;
    }
}

static void cb_timer_mod(void *timer, int64_t expire_time, void *opaque)
{
    (void)opaque;
    struct sl_timer *t = timer;
    if (!t) return;
    t->expire_ms = expire_time;
    t->active = true;
}

static void cb_notify(void *opaque) { (void)opaque; /* single-threaded: nothing to kick */ }

/* These two are deprecated in libslirp 4.9.3 but still invoked for config
 * version < 6. We re-fill the poll set every iteration, so they are no-ops. */
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
static void cb_register_poll_fd(int fd, void *opaque) { (void)fd; (void)opaque; }
static void cb_unregister_poll_fd(int fd, void *opaque) { (void)fd; (void)opaque; }

static const SlirpCb g_slirp_cb = {
    .send_packet = cb_send_packet,
    .guest_error = cb_guest_error,
    .clock_get_ns = cb_clock_get_ns,
    .timer_new = cb_timer_new,
    .timer_free = cb_timer_free,
    .timer_mod = cb_timer_mod,
    .register_poll_fd = cb_register_poll_fd,
    .unregister_poll_fd = cb_unregister_poll_fd,
    .notify = cb_notify,
};
#pragma clang diagnostic pop

/* ------------------------------------------------------------------ */
/* Timer firing + poll-set integration                                */
/* ------------------------------------------------------------------ */

static void fire_due_timers(struct netd *nd)
{
    int64_t now = now_ns() / 1000000; /* ms */
    /* Snapshot: callbacks may re-arm or free timers. Iterate carefully. */
    for (struct sl_timer *t = nd->timers; t; ) {
        struct sl_timer *next = t->next;
        if (t->active && t->expire_ms >= 0 && t->expire_ms <= now) {
            t->active = false;
            if (t->cb) t->cb(t->cb_opaque);
        }
        t = next;
    }
}

/* Nearest timer deadline in ms from now, or -1 if none armed. */
static int next_timer_ms(struct netd *nd)
{
    int64_t now = now_ns() / 1000000;
    int64_t best = -1;
    for (struct sl_timer *t = nd->timers; t; t = t->next) {
        if (!t->active || t->expire_ms < 0) continue;
        int64_t d = t->expire_ms - now;
        if (d < 0) d = 0;
        if (best < 0 || d < best) best = d;
    }
    return best < 0 ? -1 : (int)best;
}

static int cb_add_poll(slirp_os_socket fd, int events, void *opaque)
{
    struct netd *nd = opaque;
    if (nd->npfds >= MAX_POLLFDS) return -1;
    int i = nd->npfds++;
    nd->pfds[i].fd = fd;
    short e = 0;
    if (events & SLIRP_POLL_IN) e |= POLLIN;
    if (events & SLIRP_POLL_OUT) e |= POLLOUT;
    if (events & SLIRP_POLL_PRI) e |= POLLPRI;
    nd->pfds[i].events = e;
    nd->pfds[i].revents = 0;
    return i;
}

static int cb_get_revents(int idx, void *opaque)
{
    struct netd *nd = opaque;
    if (idx < 0 || idx >= nd->npfds) return 0;
    short r = nd->pfds[idx].revents;
    int e = 0;
    if (r & POLLIN) e |= SLIRP_POLL_IN;
    if (r & POLLOUT) e |= SLIRP_POLL_OUT;
    if (r & POLLPRI) e |= SLIRP_POLL_PRI;
    if (r & POLLERR) e |= SLIRP_POLL_ERR;
    if (r & POLLHUP) e |= SLIRP_POLL_HUP;
    return e;
}

/* ------------------------------------------------------------------ */
/* slirp bring-up                                                     */
/* ------------------------------------------------------------------ */

static int slirp_bringup(struct netd *nd)
{
    SlirpConfig cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.version = 1; /* only version-1 fields + callbacks are used */
    cfg.restricted = 0;
    cfg.in_enabled = true;
    cfg.in6_enabled = false;
    cfg.disable_host_loopback = false;
    cfg.enable_emu = false;

    inet_pton(AF_INET, "10.0.2.0",  &cfg.vnetwork);
    inet_pton(AF_INET, "255.255.255.0", &cfg.vnetmask);
    inet_pton(AF_INET, "10.0.2.2",  &cfg.vhost);
    inet_pton(AF_INET, "10.0.2.15", &cfg.vdhcp_start);
    inet_pton(AF_INET, "10.0.2.3",  &cfg.vnameserver);

    nd->slirp = slirp_new(&cfg, &g_slirp_cb, nd);
    if (!nd->slirp) {
        logf("slirp_new failed");
        return -1;
    }
    logf("libslirp %s up: net 10.0.2.0/24 gw 10.0.2.2 dhcp 10.0.2.15 dns 10.0.2.3",
         slirp_version_string());
    return 0;
}

/* ------------------------------------------------------------------ */
/* Device acquisition                                                 */
/* ------------------------------------------------------------------ */

static int obtain_device(struct netd *nd)
{
    /* First absorb any spontaneous ARRIVED event already queued. */
    drain_nonblocking(nd);
    if (nd->device_present) return 0;

    /* Ask the broker explicitly. */
    uint32_t tag = nd->next_tag++;
    send_request(nd, BROKER_GET_DESCRIPTORS, tag, NULL, 0);
    uint8_t buf[18 + CFG_DESC_MAX];
    uint32_t got = 0;
    int32_t st = 0;
    if (wait_reply(nd, tag, &st, buf, sizeof(buf), &got) == 0 && st == 0 && got >= 18) {
        store_descriptors(nd, buf, got);
        return 0;
    }

    /* Otherwise wait for the device to enumerate (blocking). */
    logf("waiting for device to arrive...");
    while (!nd->device_present) {
        if (pump_one(nd, 0, NULL, NULL, 0, NULL) < 0) return -1;
        if (nd->device_left) return -1;
    }
    return 0;
}

/*
 * In CDC-NCM mode (get_mode 5:3:3:0) the device stays enumerated on the
 * default usbmux config while config 5 (which carries the NCM function) is
 * merely *available*. The host must select it. On the companion path the
 * Linux kernel picks the NCM config; host-direct we do it here: fetch every
 * configuration by index, find the one exposing the NCM function, then
 * SET_CONFIGURATION to it and leave its descriptor loaded in nd->cfg_desc
 * (discover_ncm having already populated the comm/data ifaces + endpoints).
 */
static int select_ncm_config(struct netd *nd)
{
    uint8_t nconf = nd->dev_desc[17]; /* bNumConfigurations */
    logf("active config value %u lacks NCM; scanning %u configuration(s)",
         nd->cfg_value, nconf);

    uint8_t saved_cfg[CFG_DESC_MAX];
    uint16_t saved_len = nd->cfg_len;
    memcpy(saved_cfg, nd->cfg_desc, saved_len);

    for (uint8_t idx = 0; idx < nconf; idx++) {
        uint8_t hdr[9];
        uint32_t got = 0;
        uint16_t wv = (uint16_t)((2 << 8) | idx); /* CONFIG descriptor, index */
        if (broker_control(nd, 0x80, 6, wv, 0, NULL, sizeof(hdr),
                           hdr, sizeof(hdr), &got) < 0 || got < 9)
            continue;
        uint16_t total = rd16(hdr + 2); /* wTotalLength */
        if (total < 9 || total > CFG_DESC_MAX) continue;

        uint8_t full[CFG_DESC_MAX];
        got = 0;
        if (broker_control(nd, 0x80, 6, wv, 0, NULL, total,
                           full, sizeof(full), &got) < 0 || got < total)
            continue;

        memcpy(nd->cfg_desc, full, total);
        nd->cfg_len = total;
        if (discover_ncm(nd) == 0) {
            uint32_t cfgval = nd->cfg_value;
            logf("configuration value %u (index %u) carries the NCM function; selecting it",
                 cfgval, idx);
            if (simple_cmd(nd, BROKER_SET_CONFIG, &cfgval, sizeof(cfgval)) < 0) {
                logf("SET_CONFIG(%u) failed", cfgval);
                return -1;
            }
            /* Confirm the device actually switched (GET_CONFIGURATION). */
            uint8_t cur = 0;
            uint32_t n = 0;
            if (broker_control(nd, 0x80, 8, 0, 0, NULL, 1, &cur, 1, &n) == 0 && n >= 1)
                logf("device active configuration is now %u (wanted %u)", cur, cfgval);
            else
                logf("could not confirm active configuration after SET_CONFIG");
            return 0; /* nd->cfg_* + discover_ncm results are populated */
        }
    }

    memcpy(nd->cfg_desc, saved_cfg, saved_len);
    nd->cfg_len = saved_len;
    logf("no configuration exposes a CDC-NCM function — is the device in mode 3 "
         "(usbmuxd USBMUXD_DEFAULT_DEVICE_MODE=3) and unlocked?");
    return -1;
}

/* ------------------------------------------------------------------ */
/* Main event loop                                                    */
/* ------------------------------------------------------------------ */

static volatile sig_atomic_t g_stop = 0;
static void on_signal(int s) { (void)s; g_stop = 1; }

static void event_loop(struct netd *nd)
{
    logf("entering event loop (%d bulk-IN outstanding)", NUM_RX_INFLIGHT);
    while (!g_stop && !nd->device_left) {
        /* Rebuild poll set: index 0 = broker, rest = slirp sockets. */
        nd->npfds = 1;
        nd->pfds[0].fd = nd->broker_fd;
        nd->pfds[0].events = POLLIN;
        nd->pfds[0].revents = 0;

        uint32_t timeout = UINT32_MAX;
        slirp_pollfds_fill_socket(nd->slirp, &timeout, cb_add_poll, nd);

        int poll_ms = (timeout == UINT32_MAX) ? -1 : (int)timeout;
        int tmr = next_timer_ms(nd);
        if (tmr >= 0 && (poll_ms < 0 || tmr < poll_ms)) poll_ms = tmr;
        /* Never block forever with no timer wake — keep RX topped up. */
        if (poll_ms < 0 || poll_ms > 1000) poll_ms = 1000;

        int pr = poll(nd->pfds, nd->npfds, poll_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            logf("poll error: %s", strerror(errno));
            break;
        }

        /* Broker traffic (async completions, events). */
        if (nd->pfds[0].revents & (POLLIN | POLLERR | POLLHUP)) {
            if (pump_one(nd, 0, NULL, NULL, 0, NULL) < 0) {
                logf("broker connection lost");
                break;
            }
            drain_nonblocking(nd);
        }

        /* slirp socket servicing (may invoke send_packet -> ncm_tx_frame). */
        slirp_pollfds_poll(nd->slirp, (pr < 0), cb_get_revents, nd);

        /* Fire any due slirp timers. */
        fire_due_timers(nd);

        /* Ensure RX stays armed (e.g. after transient failures). */
        rx_topup(nd);
    }
    logf("event loop exit (rx_frames=%llu tx_frames=%llu)",
         (unsigned long long)nd->rx_frames, (unsigned long long)nd->tx_frames);
}

/* ------------------------------------------------------------------ */
/* Setup / teardown                                                   */
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

int main(void)
{
    signal(SIGPIPE, SIG_IGN);
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    struct netd *nd = calloc(1, sizeof(*nd));
    if (!nd) { logf("out of memory"); return 1; }
    nd->next_tag = 1;

    const char *path = getenv("INFERNO_USBD_SOCK");
    if (!path) path = INFERNO_USBD_SOCK_DEFAULT;

    logf("inferno-netd starting; broker socket %s", path);
    nd->broker_fd = broker_connect(path);
    if (nd->broker_fd < 0) {
        logf("cannot connect to broker at %s: %s", path, strerror(errno));
        free(nd);
        return 1;
    }

    /* Handshake. */
    uint32_t ver = INFERNO_BROKER_VERSION;
    uint32_t tag = nd->next_tag++;
    send_request(nd, BROKER_HELLO, tag, &ver, sizeof(ver));
    int32_t st = 0;
    if (wait_reply(nd, tag, &st, NULL, 0, NULL) < 0) {
        logf("broker handshake failed");
        goto fail;
    }
    logf("broker handshake ok (protocol v%u)", ver);

    if (obtain_device(nd) < 0) {
        logf("no device available");
        goto fail;
    }

    if (discover_ncm(nd) < 0) {
        /* Active config has no NCM; try selecting the NCM config ourselves. */
        if (select_ncm_config(nd) < 0)
            goto fail;
    }

    /* Claim comm + data interfaces and enable the bulk endpoints. */
    uint32_t ci = nd->comm_iface, di = nd->data_iface;
    if (simple_cmd(nd, BROKER_CLAIM_INTERFACE, &ci, sizeof(ci)) < 0)
        logf("claim comm interface %u failed (continuing)", nd->comm_iface);
    if (simple_cmd(nd, BROKER_CLAIM_INTERFACE, &di, sizeof(di)) < 0)
        logf("claim data interface %u failed (continuing)", nd->data_iface);

    uint32_t alt[2] = { nd->data_iface, nd->data_alt };
    if (simple_cmd(nd, BROKER_SET_ALT, alt, sizeof(alt)) < 0)
        logf("SET_ALT(iface %u, alt %u) failed (continuing)", nd->data_iface, nd->data_alt);
    else
        logf("data interface %u set to alt %u (bulk endpoints active)",
             nd->data_iface, nd->data_alt);

    /* NCM control setup: NTB params, input size, RX filter. */
    ncm_setup(nd);

    /* Bring up libslirp. */
    if (slirp_bringup(nd) < 0)
        goto fail;

    /* Arm RX and run. */
    rx_topup(nd);
    event_loop(nd);

    if (nd->slirp) slirp_cleanup(nd->slirp);
    close(nd->broker_fd);
    free(nd);
    return 0;

fail:
    if (nd->slirp) slirp_cleanup(nd->slirp);
    if (nd->broker_fd >= 0) close(nd->broker_fd);
    free(nd);
    return 1;
}
