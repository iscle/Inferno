/*
 * tcp_usb_proto.h — the Inferno "tcp_usb" wire protocol, host (server) side.
 *
 * This is a standalone mirror of the definitions in Inferno's
 * hw/usb/tcp-usb.h, so host-side tools can speak the exact same protocol the
 * companion VM's `usb-tcp-remote` device speaks — without pulling in QEMU
 * headers. Keep in sync with hw/usb/tcp-usb.h.
 *
 * Roles: the Inferno main VM (`usb-tcp-host`) is the CLIENT — it connect()s to
 * this socket and acts as the USB *device* (the emulated iPhone). Whoever
 * listen()s/accept()s is the USB *host* and drives transactions by sending
 * TCP_USB_REQUEST and reading TCP_USB_RESPONSE. That is the role the companion
 * VM used to play and that inferno-usbd now plays on the host.
 *
 * Copyright (c) 2026 Inferno host-restore contributors.
 * SPDX-License-Identifier: AGPL-3.0-or-later
 */

#ifndef INFERNO_TCP_USB_PROTO_H
#define INFERNO_TCP_USB_PROTO_H

#include <stdint.h>

#define USB_TCP_REMOTE_UNIX_DEFAULT "/tmp/InfernoUSBRemote"

/* USB PID values, matching QEMU's USB_TOKEN_* (include/hw/usb.h). */
#define TCP_USB_TOKEN_SETUP 0x2D
#define TCP_USB_TOKEN_IN    0x69
#define TCP_USB_TOKEN_OUT   0xE1

/*
 * USB return/status codes, matching QEMU's USB_RET_* (negative-space encoded
 * as the wire carries them verbatim in tcp_usb_response_header.status).
 */
#define TCP_USB_RET_SUCCESS       0
#define TCP_USB_RET_NODEV         (-1)
#define TCP_USB_RET_NAK           (-2)
#define TCP_USB_RET_STALL         (-3)
#define TCP_USB_RET_BABBLE        (-4)
#define TCP_USB_RET_IOERROR       (-5)
#define TCP_USB_RET_ASYNC         (-6)
#define TCP_USB_RET_ADD_TO_QUEUE  (-7)
#define TCP_USB_RET_REMOVE_FROM_QUEUE (-8)

enum {
    TCP_USB_REQUEST = 1,
    TCP_USB_RESPONSE,
    TCP_USB_RESET,
    TCP_USB_CANCEL,
};

#pragma pack(push, 1)

typedef struct tcp_usb_header {
    uint8_t type;
} tcp_usb_header_t;

typedef struct tcp_usb_request_header {
    uint8_t addr;
    int pid;
    uint8_t ep;
    uint64_t id;
    unsigned int stream;
    uint8_t short_not_ok;
    uint8_t int_req;
    uint16_t length;
} tcp_usb_request_header;

typedef struct tcp_usb_response_header {
    uint8_t addr;
    int pid;
    uint8_t ep;
    uint64_t id;
    uint32_t status;
    uint16_t length;
} tcp_usb_response_header;

typedef struct tcp_usb_cancel_header {
    uint8_t addr;
    int pid;
    uint8_t ep;
    uint64_t id;
} tcp_usb_cancel_header;

/* Standard 8-byte USB control setup packet. */
typedef struct usb_setup_packet {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} usb_setup_packet;

#pragma pack(pop)

#endif /* INFERNO_TCP_USB_PROTO_H */
