/*
 * inferno-usbd — host-side USB broker for the Inferno emulator.
 *
 * Replaces the companion VM. Inferno's main VM (`usb-tcp-host`) connects to a
 * single socket and behaves as the emulated iPhone's USB *device*; this daemon
 * listens on that socket and plays the USB *host controller* the companion VM
 * used to provide. It enumerates the device, then multiplexes it to multiple
 * local clients (usbmuxd, libirecovery via the libinferno-usb shim) over a Unix
 * control socket — the role the Linux kernel USB stack played in the companion.
 *
 * Concurrency model: the emulated dwc2/dwc3 device controller completes
 * transfers asynchronously (an immediate USB_RET_ASYNC placeholder, then a
 * follow-up carrying the terminal status + IN data, same id). usbmuxd also
 * keeps a persistent bulk-IN outstanding while sending on bulk-OUT. So after
 * enumeration a reader thread owns the link, demultiplexes responses by id, and
 * drives a small per-transaction state machine (control = SETUP/DATA/STATUS;
 * bulk = one token). Client requests become transactions; their tagged reply is
 * sent when the transaction completes. This lets IN and OUT overlap without
 * head-of-line blocking.
 *
 * Copyright (c) 2026 Inferno host-restore contributors.
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdatomic.h>
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
#define MAX_TXNS 128
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
#define VLOG(...)                            \
    do {                                     \
        if (g_verbose) logmsg(__VA_ARGS__);  \
    } while (0)

/* ------------------------------------------------------------------ */
/* Low-level socket helpers                                           */
/* ------------------------------------------------------------------ */

static int read_all(int fd, void *buf, size_t len)
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

static int write_all(int fd, const void *buf, size_t len)
{
    size_t n = 0;
    while (n < len) {
        ssize_t r = send(fd, (const char *)buf + n, len - n, 0);
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        n += (size_t)r;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Link + transaction state                                           */
/* ------------------------------------------------------------------ */

typedef struct {
    int fd;                       /* accepted main-VM connection, or -1 */
    uint8_t dev_addr;
    atomic_ullong next_id;
    pthread_mutex_t write_mutex;  /* serializes REQUEST writes to the link */
} tcpusb_link;

enum { TXN_BULK, TXN_CONTROL };
enum { ST_SETUP, ST_DATA, ST_STATUS, ST_DONE };

typedef struct {
    bool active;
    uint64_t id;          /* id of the in-flight token (updated per stage) */
    int client_fd;
    uint32_t tag;
    uint32_t reply_kind;  /* BROKER_REPLY or BROKER_ASYNC_COMPLETE */
    int kind;             /* TXN_BULK / TXN_CONTROL */
    int stage;            /* control staging */
    int cur_pid;          /* token currently in flight */
    uint8_t ep;           /* endpoint number (bulk) */
    usb_setup_packet setup;
    bool in;              /* data direction */
    uint8_t *buf;         /* OUT: data to send; IN: receive buffer */
    uint32_t buf_len;     /* requested length */
    uint32_t actual;      /* bytes received (IN) */
    int nak_retries;      /* serving-phase NAK re-issue counter */
} txn_t;

typedef struct {
    txn_t txns[MAX_TXNS];
    pthread_mutex_t mutex;
} txn_table;

static uint64_t next_id(tcpusb_link *l) { return atomic_fetch_add(&l->next_id, 1); }

/* Write one REQUEST (header + request header + optional payload) atomically. */
static int link_write_req(tcpusb_link *l, int pid, uint8_t ep, uint64_t id,
                          const void *data, uint16_t len)
{
    tcp_usb_header_t hdr = { .type = TCP_USB_REQUEST };
    tcp_usb_request_header req = { 0 };
    req.addr = l->dev_addr; req.pid = pid; req.ep = ep; req.id = id;
    req.length = len;
    int rc = 0;
    pthread_mutex_lock(&l->write_mutex);
    if (l->fd < 0) { rc = -1; goto out; }
    if (write_all(l->fd, &hdr, sizeof(hdr)) < 0) { rc = -1; goto out; }
    if (write_all(l->fd, &req, sizeof(req)) < 0) { rc = -1; goto out; }
    if (pid != TCP_USB_TOKEN_IN && len && write_all(l->fd, data, len) < 0) rc = -1;
out:
    pthread_mutex_unlock(&l->write_mutex);
    return rc;
}

/* Send a USB port reset (TCP_USB_RESET) — puts the device back to the default
 * state at address 0, as a real host controller does before enumerating. */
static void link_send_reset(tcpusb_link *l)
{
    tcp_usb_header_t hdr = { .type = TCP_USB_RESET };
    pthread_mutex_lock(&l->write_mutex);
    if (l->fd >= 0) { if (write_all(l->fd, &hdr, sizeof(hdr)) < 0) {} }
    pthread_mutex_unlock(&l->write_mutex);
}

/* ================================================================== */
/* Synchronous transport — used ONLY during connect-time enumeration,  */
/* before the reader thread is started (so no id demux is needed yet).  */
/* ================================================================== */

#define TXN_LOST    (-1000)
#define TXN_TIMEOUT (-1001)
#define SYNC_IDLE_LIMIT 3  /* ~3s of device silence per token before giving up */

/* Drain and discard a response's IN payload we don't want (stale id). */
static int drain_payload(int fd, uint32_t length)
{
    while (length) {
        char junk[512];
        uint32_t c = length < sizeof(junk) ? length : sizeof(junk);
        if (read_all(fd, junk, c) <= 0) return -1;
        length -= c;
    }
    return 0;
}

/*
 * One synchronous transaction with a bounded wait. Responses are matched by id
 * so a late response to a previously timed-out token cannot misalign the
 * stream — it is drained and skipped. The emulated controller occasionally
 * drops a token (USB is experimental), so a header that never arrives becomes
 * TXN_TIMEOUT and the caller re-issues rather than blocking forever.
 */
#define MAX_NAK_RETRIES 400  /* ~4s of NAKs (device busy during bring-up) */

static int tcpusb_txn_sync(tcpusb_link *l, int pid, uint8_t ep, void *io_buf,
                           uint16_t *io_len)
{
    uint16_t len = io_len ? *io_len : 0;
    int nak_retries = 0;
    tcp_usb_response_header resp = { 0 };

reissue:;
    uint64_t id = next_id(l);
    VLOG("[txn] issue pid=0x%x ep=%d id=%llu len=%u", pid, ep, (unsigned long long)id, len);
    if (link_write_req(l, pid, ep, id, io_buf, len) < 0) goto lost;
    if (io_len && pid == TCP_USB_TOKEN_IN) *io_len = 0;

    /*
     * Wait for THIS id's response. A NAK means "busy, retry" — real host
     * controllers re-issue automatically, so we do too (bounded). We never
     * abandon an id mid-wait for a non-NAK reason, so a slow device that
     * responds late can't livelock us; only full-idle silence yields
     * TXN_TIMEOUT for the caller to restart.
     */
    int idle = 0;
    for (;;) {
        struct pollfd pfd = { .fd = l->fd, .events = POLLIN };
        int pr = poll(&pfd, 1, 1000);
        if (pr == 0) {
            if (++idle >= SYNC_IDLE_LIMIT) return TXN_TIMEOUT;
            continue;
        }
        if (pr < 0) { if (errno == EINTR) continue; goto lost; }
        idle = 0;

        tcp_usb_header_t rhdr = { 0 };
        if (read_all(l->fd, &rhdr, sizeof(rhdr)) <= 0) goto lost;
        if (rhdr.type != TCP_USB_RESPONSE) { VLOG("[txn] non-response hdr type=%u", rhdr.type); goto lost; }
        if (read_all(l->fd, &resp, sizeof(resp)) <= 0) goto lost;
        bool is_async = ((int32_t)resp.status == TCP_USB_RET_ASYNC);
        bool mine = (resp.id == id);

        /* Payload follows ONLY for IN responses (hcd-tcp writes a buffer only
         * for the IN pid); a non-IN response may report a nonzero length with
         * no bytes on the wire, so key off resp.pid, not resp.length. */
        if (resp.length && !is_async && resp.pid == TCP_USB_TOKEN_IN) {
            if (mine && pid == TCP_USB_TOKEN_IN) {
                uint16_t want = resp.length; if (want > len) want = len;
                if (read_all(l->fd, io_buf, want) <= 0) goto lost;
                if (io_len) *io_len = want;
                if (drain_payload(l->fd, resp.length - want) < 0) goto lost;
            } else if (drain_payload(l->fd, resp.length) < 0) {
                goto lost;
            }
        }
        if (!mine) continue;      /* stale/late response for an old id */
        if (is_async) continue;   /* wait for the terminal follow-up */

        if ((int32_t)resp.status == TCP_USB_RET_NAK) {
            if (++nak_retries > MAX_NAK_RETRIES) { VLOG("[txn] NAK budget exhausted"); return TCP_USB_RET_NAK; }
            usleep(10000); /* 10ms, then retry the same token */
            goto reissue;
        }
        break;
    }
    return (int)resp.status;
lost:
    if (l->fd >= 0) { close(l->fd); l->fd = -1; }
    return TXN_LOST;
}

static int tcpusb_control_sync(tcpusb_link *l, const usb_setup_packet *sp,
                               void *data, uint16_t wlen)
{
    bool in = (sp->bmRequestType & 0x80) != 0;
    uint16_t len = sizeof(*sp);
    uint8_t setup_buf[sizeof(*sp)];
    memcpy(setup_buf, sp, sizeof(*sp));
    int st = tcpusb_txn_sync(l, TCP_USB_TOKEN_SETUP, 0, setup_buf, &len);
    if (st == TXN_LOST || st == TXN_TIMEOUT) return st;

    int data_done = 0;
    if (wlen) {
        len = wlen;
        st = tcpusb_txn_sync(l, in ? TCP_USB_TOKEN_IN : TCP_USB_TOKEN_OUT, 0, data, &len);
        if (st == TXN_LOST || st == TXN_TIMEOUT) return st;
        if (st == TCP_USB_RET_SUCCESS) data_done = in ? len : wlen;
    }
    len = 0;
    st = tcpusb_txn_sync(l, in ? TCP_USB_TOKEN_OUT : TCP_USB_TOKEN_IN, 0, NULL, &len);
    if (st == TXN_LOST || st == TXN_TIMEOUT) return st;

    if (sp->bmRequestType == 0x00 && sp->bRequest == 5) l->dev_addr = (uint8_t)sp->wValue;
    return data_done;
}

/* ------------------------------------------------------------------ */
/* Enumeration (connect-time, synchronous)                            */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t dev_desc[18];
    uint8_t cfg_desc[CFG_DESC_MAX];
    uint16_t cfg_len;
    bool present;
} device_cache;

static int get_descriptor_sync(tcpusb_link *l, uint8_t type, uint8_t index,
                               void *out, uint16_t len)
{
    usb_setup_packet sp = { .bmRequestType = 0x80, .bRequest = 6,
                            .wValue = (uint16_t)((type << 8) | index), .wIndex = 0,
                            .wLength = len };
    return tcpusb_control_sync(l, &sp, out, len);
}

/*
 * One enumeration attempt: reset the port, then read the device descriptor,
 * assign an address, and read the config descriptor. Each step is a single
 * bounded try — the outer loop re-resets and retries, which is what catches the
 * device coming up several seconds into guest boot: a reset must land on the
 * device *after* it exists, so tight reset+attempt cycling is essential.
 */
static bool enumerate_once(tcpusb_link *l, device_cache *dc)
{
    memset(dc, 0, sizeof(*dc));
    l->dev_addr = 0;

    link_send_reset(l);
    usleep(100000);

    if (get_descriptor_sync(l, 1, 0, dc->dev_desc, sizeof(dc->dev_desc)) < 8) {
        if (l->fd < 0) return false;
        return false;
    }

    uint16_t vid = dc->dev_desc[8] | (dc->dev_desc[9] << 8);
    uint16_t pid = dc->dev_desc[10] | (dc->dev_desc[11] << 8);
    VLOG("inferno-usbd: got device descriptor vid=%04x pid=%04x", vid, pid);

    usb_setup_packet setaddr = { .bmRequestType = 0, .bRequest = 5, .wValue = 2 };
    if (tcpusb_control_sync(l, &setaddr, NULL, 0) < 0) return false;

    uint8_t head[9];
    if (get_descriptor_sync(l, 2, 0, head, sizeof(head)) < 9) {
        VLOG("enumerate: config header not ready yet");
        return false;
    }
    uint16_t total = (uint16_t)(head[2] | (head[3] << 8));
    if (total < 9) total = 9;
    if (total > CFG_DESC_MAX) total = CFG_DESC_MAX;
    if (get_descriptor_sync(l, 2, 0, dc->cfg_desc, total) < total) {
        VLOG("enumerate: full config not ready yet");
        return false;
    }
    dc->cfg_len = total; dc->present = true;
    logmsg("inferno-usbd: device enumerated vid=%04x pid=%04x cfg_len=%u", vid, pid, dc->cfg_len);
    return true;
}

/* Tight reset+attempt loop: catches the device once the guest brings USB up
 * (~5s into boot for iOS restore mode) and self-heals dropped tokens. */
static bool enumerate(tcpusb_link *l, device_cache *dc)
{
    for (int attempt = 0; attempt < 120; attempt++) {
        if (enumerate_once(l, dc)) return true;
        if (l->fd < 0) return false; /* link genuinely gone */
        usleep(300000);
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Broker fan-out                                                     */
/* ------------------------------------------------------------------ */

typedef struct { int fd; bool hello_ok; } client_t;

static pthread_mutex_t g_client_write_mutex = PTHREAD_MUTEX_INITIALIZER;

static void send_msg(int fd, uint32_t kind, uint32_t tag, int32_t status,
                     const void *payload, uint32_t len)
{
    broker_msg_hdr h = { .kind = kind, .tag = tag, .status = status, .length = len };
    pthread_mutex_lock(&g_client_write_mutex);
    if (write_all(fd, &h, sizeof(h)) == 0 && len && payload) write_all(fd, payload, len);
    pthread_mutex_unlock(&g_client_write_mutex);
}

/* ------------------------------------------------------------------ */
/* Reader thread: demux responses, drive transaction state machines   */
/* ------------------------------------------------------------------ */

typedef struct {
    tcpusb_link *link;
    txn_table *table;
    client_t *clients;
    int self_pipe_w;   /* poked on link loss so the main loop wakes */
    atomic_bool running;
} reader_ctx;

static txn_t *find_txn_by_id(txn_table *t, uint64_t id)
{
    for (int i = 0; i < MAX_TXNS; i++)
        if (t->txns[i].active && t->txns[i].id == id) return &t->txns[i];
    return NULL;
}

static void txn_complete(txn_t *x, int32_t status)
{
    uint8_t stackbuf[512];
    uint8_t *pl = NULL; uint32_t plen = 0;
    if (status >= 0 && x->in && x->actual) { pl = x->buf; plen = x->actual; }
    (void)stackbuf;
    VLOG("[cmp] %s tag=%u status=%d inlen=%u", x->kind == TXN_BULK ? "bulk" : "control",
         x->tag, status, x->in ? plen : 0);
    if (x->in)
        send_msg(x->client_fd, x->reply_kind, x->tag, status, pl, plen);
    else
        send_msg(x->client_fd, x->reply_kind, x->tag, status, NULL, 0);
    free(x->buf); x->buf = NULL;
    x->active = false;
}

/* Re-issue the transaction's current in-flight token (NAK retry / serving). */
static void txn_reissue(tcpusb_link *l, txn_t *x)
{
    x->id = next_id(l);
    if (x->kind == TXN_BULK) {
        link_write_req(l, x->cur_pid, x->ep, x->id,
                       x->in ? NULL : x->buf, (uint16_t)x->buf_len);
    } else if (x->stage == ST_SETUP) {
        uint8_t sb[8]; memcpy(sb, &x->setup, 8);
        link_write_req(l, TCP_USB_TOKEN_SETUP, 0, x->id, sb, 8);
    } else if (x->stage == ST_DATA) {
        link_write_req(l, x->cur_pid, 0, x->id,
                       x->in ? NULL : x->buf, x->setup.wLength);
    } else {
        link_write_req(l, x->cur_pid, 0, x->id, NULL, 0);
    }
}

/* Issue the next token for a control transaction after a stage completes. */
static void control_advance(reader_ctx *rc, txn_t *x)
{
    tcpusb_link *l = rc->link;
    if (x->stage == ST_SETUP) {
        if (x->setup.wLength) {
            x->stage = ST_DATA;
            x->cur_pid = x->in ? TCP_USB_TOKEN_IN : TCP_USB_TOKEN_OUT;
            x->id = next_id(l);
            link_write_req(l, x->cur_pid, 0, x->id, x->buf, x->setup.wLength);
        } else {
            x->stage = ST_STATUS;
            x->cur_pid = x->in ? TCP_USB_TOKEN_OUT : TCP_USB_TOKEN_IN;
            x->id = next_id(l);
            link_write_req(l, x->cur_pid, 0, x->id, NULL, 0);
        }
    } else if (x->stage == ST_DATA) {
        x->stage = ST_STATUS;
        x->cur_pid = x->in ? TCP_USB_TOKEN_OUT : TCP_USB_TOKEN_IN;
        x->id = next_id(l);
        link_write_req(l, x->cur_pid, 0, x->id, NULL, 0);
    } else { /* ST_STATUS done */
        if (x->setup.bmRequestType == 0 && x->setup.bRequest == 5)
            l->dev_addr = (uint8_t)x->setup.wValue;
        txn_complete(x, TCP_USB_RET_SUCCESS);
    }
}

static void *reader_thread(void *arg)
{
    reader_ctx *rc = arg;
    tcpusb_link *l = rc->link;
    txn_table *t = rc->table;

    while (atomic_load(&rc->running)) {
        /*
         * Simple blocking demux. Bulk-IN persistence (keeping an idle RX-loop IN
         * pending until the guest has data) is handled device-side now, in
         * Inferno's hcd-tcp: it parks a NAK'd bulk/interrupt IN and re-polls it
         * locally, sending us a single response only when data is ready. So we
         * never see a NAK storm here and don't have to pace or re-poll — an IN
         * simply completes when its data arrives. NAK retry below only covers
         * transient control/OUT NAKs.
         */
        tcp_usb_header_t rhdr = { 0 };
        int r = read_all(l->fd, &rhdr, sizeof(rhdr));
        if (r <= 0 || rhdr.type != TCP_USB_RESPONSE) break;
        tcp_usb_response_header resp = { 0 };
        if (read_all(l->fd, &resp, sizeof(resp)) <= 0) break;
        bool is_async = ((int32_t)resp.status == TCP_USB_RET_ASYNC);

        if ((int32_t)resp.status != TCP_USB_RET_NAK)
            VLOG("[rdr] resp id=%llu status=%d len=%u pid=0x%x async=%d",
                 (unsigned long long)resp.id, (int32_t)resp.status, resp.length, resp.pid, is_async);
        pthread_mutex_lock(&t->mutex);
        txn_t *x = find_txn_by_id(t, resp.id);

        /* Payload follows only for IN responses (see tcpusb_txn_sync note). */
        if (resp.length && !is_async && resp.pid == TCP_USB_TOKEN_IN) {
            uint32_t remain = resp.length;
            if (x && x->in && x->cur_pid == TCP_USB_TOKEN_IN) {
                uint32_t space = (x->buf_len > x->actual) ? x->buf_len - x->actual : 0;
                uint32_t want = remain < space ? remain : space;
                if (want && read_all(l->fd, x->buf + x->actual, want) <= 0) { pthread_mutex_unlock(&t->mutex); break; }
                x->actual += want;
                remain -= want;
            }
            while (remain) {
                char junk[512]; uint32_t c = remain < sizeof(junk) ? remain : sizeof(junk);
                if (read_all(l->fd, junk, c) <= 0) { pthread_mutex_unlock(&t->mutex); goto done; }
                remain -= c;
            }
        }

        if (x && !is_async) {
            int32_t st = (int32_t)resp.status;
            if (st == TCP_USB_RET_NAK && x->nak_retries++ < MAX_NAK_RETRIES) {
                txn_reissue(l, x); /* transient control/OUT NAK */
            } else {
                x->nak_retries = 0;
                if (x->kind == TXN_BULK) txn_complete(x, st);
                else control_advance(rc, x);
            }
        }
        pthread_mutex_unlock(&t->mutex);
    }
done:
    atomic_store(&rc->running, false);
    /* Fail any outstanding transactions so clients don't hang. */
    pthread_mutex_lock(&t->mutex);
    for (int i = 0; i < MAX_TXNS; i++)
        if (t->txns[i].active) txn_complete(&t->txns[i], TCP_USB_RET_NODEV);
    pthread_mutex_unlock(&t->mutex);
    if (l->fd >= 0) { close(l->fd); l->fd = -1; }
    uint8_t poke = 1;
    if (write(rc->self_pipe_w, &poke, 1) < 0) { /* best effort */ }
    return NULL;
}

/* Allocate a transaction slot. Caller holds the table mutex. */
static txn_t *txn_alloc(txn_table *t)
{
    for (int i = 0; i < MAX_TXNS; i++) if (!t->txns[i].active) return &t->txns[i];
    return NULL;
}

/* ------------------------------------------------------------------ */
/* Client request handling (serving phase)                            */
/* ------------------------------------------------------------------ */

static int submit_control(tcpusb_link *l, txn_table *t, int cfd, uint32_t tag,
                          uint32_t reply_kind, const usb_setup_packet *sp,
                          const uint8_t *out_data)
{
    pthread_mutex_lock(&t->mutex);
    txn_t *x = txn_alloc(t);
    if (!x) { pthread_mutex_unlock(&t->mutex); send_msg(cfd, reply_kind, tag, TCP_USB_RET_STALL, NULL, 0); return 0; }
    memset(x, 0, sizeof(*x));
    x->active = true; x->client_fd = cfd; x->tag = tag; x->reply_kind = reply_kind;
    x->kind = TXN_CONTROL; x->stage = ST_SETUP; x->cur_pid = TCP_USB_TOKEN_SETUP;
    x->setup = *sp; x->in = (sp->bmRequestType & 0x80) != 0;
    x->buf_len = sp->wLength;
    if (sp->wLength) {
        x->buf = malloc(sp->wLength);
        if (!x->in && out_data) memcpy(x->buf, out_data, sp->wLength);
    }
    x->id = next_id(l);
    uint8_t setup_buf[8]; memcpy(setup_buf, sp, 8);
    VLOG("[sub] control bmReq=0x%x bReq=0x%x wVal=0x%x wIdx=0x%x wLen=%u id=%llu",
         sp->bmRequestType, sp->bRequest, sp->wValue, sp->wIndex, sp->wLength,
         (unsigned long long)x->id);
    pthread_mutex_unlock(&t->mutex);
    link_write_req(l, TCP_USB_TOKEN_SETUP, 0, x->id, setup_buf, 8);
    return 0;
}

static int submit_bulk(tcpusb_link *l, txn_table *t, int cfd, uint32_t tag,
                       uint32_t reply_kind, uint8_t ep, uint32_t length,
                       const uint8_t *out_data)
{
    bool in = (ep & 0x80) != 0;
    pthread_mutex_lock(&t->mutex);
    txn_t *x = txn_alloc(t);
    if (!x) { pthread_mutex_unlock(&t->mutex); send_msg(cfd, reply_kind, tag, TCP_USB_RET_STALL, NULL, 0); return 0; }
    memset(x, 0, sizeof(*x));
    x->active = true; x->client_fd = cfd; x->tag = tag; x->reply_kind = reply_kind;
    x->kind = TXN_BULK; x->in = in; x->ep = ep & 0x0f;
    x->cur_pid = in ? TCP_USB_TOKEN_IN : TCP_USB_TOKEN_OUT;
    x->buf_len = length;
    if (length) {
        x->buf = malloc(length);
        if (!in && out_data) memcpy(x->buf, out_data, length);
    }
    x->id = next_id(l);
    VLOG("[sub] bulk ep=0x%x %s len=%u id=%llu", ep, in ? "IN" : "OUT", length,
         (unsigned long long)x->id);
    pthread_mutex_unlock(&t->mutex);
    link_write_req(l, x->cur_pid, ep & 0x0f, x->id, out_data, (uint16_t)length);
    return 0;
}

/* Returns -1 to drop the client. Device ops become transactions completed by
 * the reader thread; advisory/cached ops reply immediately. */
static int handle_client_request(client_t *c, tcpusb_link *l, txn_table *t,
                                 device_cache *dc)
{
    broker_msg_hdr h;
    if (read_all(c->fd, &h, sizeof(h)) <= 0) { VLOG("[cli] read failed / client gone"); return -1; }
    bool have_dev = (l->fd >= 0) && dc->present;
    VLOG("[cli] req kind=%u tag=%u len=%u have_dev=%d", h.kind, h.tag, h.length, have_dev);

    switch (h.kind) {
    case BROKER_HELLO: {
        uint32_t ver = 0;
        if (h.length >= 4 && read_all(c->fd, &ver, 4) <= 0) return -1;
        c->hello_ok = true;
        send_msg(c->fd, BROKER_REPLY, h.tag, INFERNO_BROKER_VERSION, NULL, 0);
        if (have_dev) {
            uint8_t buf[18 + CFG_DESC_MAX];
            memcpy(buf, dc->dev_desc, 18); memcpy(buf + 18, dc->cfg_desc, dc->cfg_len);
            send_msg(c->fd, BROKER_EVENT_ARRIVED, 0, 0, buf, 18 + dc->cfg_len);
        }
        return 0;
    }
    case BROKER_GET_DESCRIPTORS: {
        if (!have_dev) { send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_NODEV, NULL, 0); return 0; }
        uint8_t buf[18 + CFG_DESC_MAX];
        memcpy(buf, dc->dev_desc, 18); memcpy(buf + 18, dc->cfg_desc, dc->cfg_len);
        send_msg(c->fd, BROKER_REPLY, h.tag, 0, buf, 18 + dc->cfg_len);
        return 0;
    }
    case BROKER_CLAIM_INTERFACE:
    case BROKER_RELEASE_INTERFACE: {
        uint32_t v; if (h.length >= 4 && read_all(c->fd, &v, 4) <= 0) return -1;
        send_msg(c->fd, BROKER_REPLY, h.tag, 0, NULL, 0);
        return 0;
    }
    case BROKER_SET_CONFIG: {
        uint32_t cfg = 0; if (h.length >= 4 && read_all(c->fd, &cfg, 4) <= 0) return -1;
        if (!have_dev) { send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_NODEV, NULL, 0); return 0; }
        usb_setup_packet sp = { .bmRequestType = 0, .bRequest = 9, .wValue = (uint16_t)cfg };
        return submit_control(l, t, c->fd, h.tag, BROKER_REPLY, &sp, NULL);
    }
    case BROKER_SET_ALT: {
        uint32_t v[2] = { 0, 0 }; if (h.length >= 8 && read_all(c->fd, v, 8) <= 0) return -1;
        if (!have_dev) { send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_NODEV, NULL, 0); return 0; }
        usb_setup_packet sp = { .bmRequestType = 0x01, .bRequest = 11,
                                .wValue = (uint16_t)v[1], .wIndex = (uint16_t)v[0] };
        return submit_control(l, t, c->fd, h.tag, BROKER_REPLY, &sp, NULL);
    }
    case BROKER_CLEAR_HALT: {
        uint32_t ep = 0; if (h.length >= 4 && read_all(c->fd, &ep, 4) <= 0) return -1;
        if (!have_dev) { send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_NODEV, NULL, 0); return 0; }
        usb_setup_packet sp = { .bmRequestType = 0x02, .bRequest = 1, .wValue = 0,
                                .wIndex = (uint16_t)ep };
        return submit_control(l, t, c->fd, h.tag, BROKER_REPLY, &sp, NULL);
    }
    case BROKER_CONTROL: {
        broker_control_req req;
        if (read_all(c->fd, &req, sizeof(req)) <= 0) return -1;
        usb_setup_packet sp; memcpy(&sp, req.setup, sizeof(sp));
        bool in = (sp.bmRequestType & 0x80) != 0;
        uint8_t *out = NULL;
        if (!in && sp.wLength) { out = malloc(sp.wLength); if (read_all(c->fd, out, sp.wLength) <= 0) { free(out); return -1; } }
        if (!have_dev) { send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_NODEV, NULL, 0); free(out); return 0; }
        int rc = submit_control(l, t, c->fd, h.tag, BROKER_REPLY, &sp, out);
        free(out);
        return rc;
    }
    case BROKER_BULK:
    case BROKER_SUBMIT_ASYNC: {
        broker_transfer_req req;
        if (read_all(c->fd, &req, sizeof(req)) <= 0) return -1;
        /* Direction: for control it's in the setup packet (bmRequestType bit 7);
         * for bulk it's the endpoint address bit 7. Only OUT transfers carry
         * data bytes after the request header. */
        bool in = (req.type == BROKER_XFER_CONTROL)
                      ? (req.setup[0] & 0x80) != 0
                      : (req.ep & 0x80) != 0;
        uint8_t *out = NULL;
        if (!in && req.length) { out = malloc(req.length); if (read_all(c->fd, out, req.length) <= 0) { free(out); return -1; } }
        uint32_t reply_kind = (h.kind == BROKER_SUBMIT_ASYNC) ? BROKER_ASYNC_COMPLETE : BROKER_REPLY;
        if (req.type == BROKER_XFER_CONTROL) {
            usb_setup_packet sp; memcpy(&sp, req.setup, sizeof(sp));
            if (!have_dev) { send_msg(c->fd, reply_kind, h.tag, TCP_USB_RET_NODEV, NULL, 0); free(out); return 0; }
            int rc = submit_control(l, t, c->fd, h.tag, reply_kind, &sp, out);
            free(out); return rc;
        }
        if (!have_dev) { send_msg(c->fd, reply_kind, h.tag, TCP_USB_RET_NODEV, NULL, 0); free(out); return 0; }
        int rc = submit_bulk(l, t, c->fd, h.tag, reply_kind, req.ep, req.length, out);
        free(out);
        return rc;
    }
    case BROKER_CANCEL_ASYNC: {
        uint32_t ctag = 0; if (h.length >= 4 && read_all(c->fd, &ctag, 4) <= 0) return -1;
        /* Best-effort: mark the matching transaction cancelled/failed. */
        pthread_mutex_lock(&t->mutex);
        for (int i = 0; i < MAX_TXNS; i++)
            if (t->txns[i].active && t->txns[i].tag == ctag && t->txns[i].client_fd == c->fd) {
                txn_complete(&t->txns[i], TCP_USB_RET_IOERROR);
                break;
            }
        pthread_mutex_unlock(&t->mutex);
        return 0;
    }
    case BROKER_RESET:
        /* Re-enumeration mid-serve requires quiescing the reader thread; not
         * needed for the normal restore flow. Acknowledge without action. */
        send_msg(c->fd, BROKER_REPLY, h.tag, 0, NULL, 0);
        return 0;
    default: {
        for (uint32_t left = h.length; left; ) {
            char junk[256]; uint32_t cc = left > sizeof(junk) ? sizeof(junk) : left;
            if (read_all(c->fd, junk, cc) <= 0) return -1;
            left -= cc;
        }
        send_msg(c->fd, BROKER_REPLY, h.tag, TCP_USB_RET_STALL, NULL, 0);
        return 0;
    }
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

static void broadcast_arrived(client_t *clients, device_cache *dc)
{
    uint8_t buf[18 + CFG_DESC_MAX];
    memcpy(buf, dc->dev_desc, 18); memcpy(buf + 18, dc->cfg_desc, dc->cfg_len);
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (clients[i].fd >= 0 && clients[i].hello_ok)
            send_msg(clients[i].fd, BROKER_EVENT_ARRIVED, 0, 0, buf, 18 + dc->cfg_len);
}
static void broadcast_left(client_t *clients)
{
    for (int i = 0; i < MAX_CLIENTS; i++)
        if (clients[i].fd >= 0 && clients[i].hello_ok)
            send_msg(clients[i].fd, BROKER_EVENT_LEFT, 0, 0, NULL, 0);
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
    logmsg("inferno-usbd: listening for main VM on %s, clients on %s", usb_path, broker_path);

    int wake[2];
    if (pipe(wake) < 0) { perror("pipe"); return 1; }

    tcpusb_link link = { .fd = -1, .dev_addr = 0 };
    atomic_init(&link.next_id, 1);
    pthread_mutex_init(&link.write_mutex, NULL);
    device_cache dc = { 0 };
    txn_table table; memset(&table, 0, sizeof(table)); pthread_mutex_init(&table.mutex, NULL);

    client_t clients[MAX_CLIENTS];
    for (int i = 0; i < MAX_CLIENTS; i++) { clients[i].fd = -1; clients[i].hello_ok = false; }

    reader_ctx rc = { .link = &link, .table = &table, .clients = clients,
                      .self_pipe_w = wake[1] };
    atomic_init(&rc.running, false);
    pthread_t reader; bool reader_started = false;

    for (;;) {
        struct pollfd pfds[3 + MAX_CLIENTS];
        int n = 0;
        pfds[n].fd = usb_listen; pfds[n].events = POLLIN; n++;
        pfds[n].fd = broker_listen; pfds[n].events = POLLIN; n++;
        pfds[n].fd = wake[0]; pfds[n].events = POLLIN; n++;
        int cli_base = n;
        for (int i = 0; i < MAX_CLIENTS; i++)
            if (clients[i].fd >= 0) { pfds[n].fd = clients[i].fd; pfds[n].events = POLLIN; n++; }

        if (poll(pfds, n, -1) < 0) { if (errno == EINTR) continue; perror("poll"); break; }

        /* Link-loss wakeup. */
        if (pfds[2].revents & POLLIN) {
            uint8_t b; if (read(wake[0], &b, 1) < 0) {}
            if (reader_started && !atomic_load(&rc.running)) {
                pthread_join(reader, NULL); reader_started = false;
                if (dc.present) { dc.present = false; logmsg("inferno-usbd: main VM link lost"); broadcast_left(clients); }
            }
        }

        /* New main-VM connection. */
        if (pfds[0].revents & POLLIN) {
            int fd = accept(usb_listen, NULL, NULL);
            if (fd >= 0) {
                if (reader_started) { atomic_store(&rc.running, false); if (link.fd >= 0) { close(link.fd); link.fd = -1; } pthread_join(reader, NULL); reader_started = false; }
                link.fd = fd; link.dev_addr = 0;
                logmsg("inferno-usbd: main VM connected; enumerating...");
                if (enumerate(&link, &dc)) {
                    broadcast_arrived(clients, &dc);
                    atomic_store(&rc.running, true);
                    pthread_create(&reader, NULL, reader_thread, &rc);
                    reader_started = true;
                } else {
                    logmsg("inferno-usbd: enumeration failed");
                    if (link.fd >= 0) { close(link.fd); link.fd = -1; }
                }
            }
        }

        /* New shim client. */
        if (pfds[1].revents & POLLIN) {
            int fd = accept(broker_listen, NULL, NULL);
            if (fd >= 0) {
                int slot = -1;
                for (int i = 0; i < MAX_CLIENTS; i++) if (clients[i].fd < 0) { slot = i; break; }
                if (slot < 0) { logmsg("too many clients"); close(fd); }
                else { clients[slot].fd = fd; clients[slot].hello_ok = false; VLOG("[cli] client %d connected (fd=%d)", slot, fd); }
            }
        }

        /* Client requests. */
        int idx = cli_base;
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (clients[i].fd < 0) continue;
            short re = pfds[idx++].revents;
            if (re & (POLLIN | POLLHUP | POLLERR)) {
                if (handle_client_request(&clients[i], &link, &table, &dc) < 0) {
                    close(clients[i].fd); clients[i].fd = -1;
                }
            }
        }
    }

    if (reader_started) { atomic_store(&rc.running, false); pthread_join(reader, NULL); }
    close(usb_listen); close(broker_listen);
    unlink(usb_path); unlink(broker_path);
    return 0;
}
