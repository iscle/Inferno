/*
 * broker_proto.h — IPC protocol between inferno-usbd (the broker that owns the
 * single tcp_usb socket) and libinferno-usb (the libusb shim linked into each
 * restore tool: usbmuxd, idevicerestore/libirecovery).
 *
 * Why a broker: the Inferno main VM exposes ONE USB connection (one socket)
 * that carries the emulated iPhone through every mode change (DFU -> recovery
 * -> restore -> normal). On a real machine the kernel owns the bus and many
 * userspace clients (usbmuxd, libirecovery) share it. inferno-usbd plays that
 * kernel role: it owns the tcp_usb socket, performs USB enumeration, and lets
 * multiple shim clients submit transfers and observe device arrival/removal.
 *
 * All integers are host-endian; the broker and shim are the same machine.
 * Every request carries a client-chosen tag echoed in its reply so the async
 * shim can match completions.
 *
 * Copyright (c) 2026 Inferno host-restore contributors.
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#ifndef INFERNO_BROKER_PROTO_H
#define INFERNO_BROKER_PROTO_H

#include <stdint.h>

#define INFERNO_USBD_SOCK_DEFAULT "/tmp/inferno-usbd.sock"
#define INFERNO_BROKER_VERSION 1

/* Client -> broker opcodes. */
enum {
    BROKER_HELLO = 1,       /* handshake: {version} */
    BROKER_GET_DESCRIPTORS, /* fetch cached device + active config descriptors */
    BROKER_CLAIM_INTERFACE, /* {iface} */
    BROKER_RELEASE_INTERFACE,/* {iface} */
    BROKER_SET_CONFIG,      /* {config value} */
    BROKER_SET_ALT,         /* {iface, alt} */
    BROKER_CLEAR_HALT,      /* {ep} */
    BROKER_RESET,           /* re-enumerate the device */
    BROKER_CONTROL,         /* {setup[8]} + optional OUT data; returns IN data */
    BROKER_BULK,            /* {ep, length}  (+ data if OUT) */
    BROKER_SUBMIT_ASYNC,    /* async bulk/control; completion delivered later */
    BROKER_CANCEL_ASYNC,    /* {tag} */
};

/* Broker -> client message kinds. */
enum {
    BROKER_REPLY = 100,       /* synchronous reply to a request (matches tag) */
    BROKER_EVENT_ARRIVED,     /* device (re)enumerated and ready */
    BROKER_EVENT_LEFT,        /* device disconnected */
    BROKER_ASYNC_COMPLETE,    /* async transfer finished (matches tag) */
};

#pragma pack(push, 1)

typedef struct broker_msg_hdr {
    uint32_t kind;    /* opcode (C->B) or message kind (B->C) */
    uint32_t tag;     /* client-chosen; echoed in reply/completion */
    int32_t  status;  /* libusb-style status on replies/completions; 0 on requests */
    uint32_t length;  /* bytes of payload following this header */
} broker_msg_hdr;

/* Payload for BROKER_CONTROL requests (followed by wLength OUT bytes if any). */
typedef struct broker_control_req {
    uint8_t  setup[8];   /* usb_setup_packet, wire order */
    uint32_t timeout_ms;
} broker_control_req;

/* Transfer type values on the wire — these match libusb's transfer-type enum
 * (LIBUSB_TRANSFER_TYPE_*), NOT inferno-usbd's internal TXN_* enum. */
enum {
    BROKER_XFER_CONTROL = 0,
    BROKER_XFER_ISO = 1,
    BROKER_XFER_BULK = 2,
    BROKER_XFER_INTERRUPT = 3,
};

/* Payload for BROKER_BULK / BROKER_SUBMIT_ASYNC requests. */
typedef struct broker_transfer_req {
    uint8_t  ep;         /* endpoint address incl. direction bit */
    uint8_t  type;       /* BROKER_XFER_* (== libusb transfer type) */
    uint16_t _pad;
    uint32_t length;     /* requested length */
    uint32_t timeout_ms;
    uint8_t  setup[8];   /* used when type == control */
} broker_transfer_req;

/*
 * BROKER_EVENT_ARRIVED payload: the cached 18-byte device descriptor, followed
 * by the full active config descriptor blob (wTotalLength bytes). The shim
 * parses these into libusb structs so no round-trip is needed for descriptors.
 */

#pragma pack(pop)

#endif /* INFERNO_BROKER_PROTO_H */
