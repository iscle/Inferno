/*
 * TCP Remote USB Host.
 *
 * Copyright (c) 2023-2026 Visual Ehrmanntraut (VisualEhrmanntraut).
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef HW_USB_HCD_TCP_H
#define HW_USB_HCD_TCP_H

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/usb.h"
#include "hw/usb/tcp-usb.h"
#include "io/channel.h"
#include "qemu/coroutine.h"
#include "qemu/queue.h"
#include "qemu/timer.h"
#include "qom/object.h"

#define TYPE_USB_TCP_HOST "usb-tcp-host"
OBJECT_DECLARE_SIMPLE_TYPE(USBTCPHostState, USB_TCP_HOST)

typedef struct USBTCPPacket {
    USBPacket p;
    void *buffer;
    USBDevice *dev;
    USBTCPHostState *s;
    uint8_t addr;
} USBTCPPacket;

/*
 * A device-mode IN token (bulk/interrupt) that the device NAK'd because it has
 * no data queued yet. Rather than relay the NAK to the remote host — which
 * would make the host software-poll over the socket and throttle the bulk-OUT
 * stream — we hold the request here and re-run it locally until the guest
 * queues data, then send a single response. Re-running is a fresh transaction
 * each time, so the guest's DMA/DART mappings stay valid (the reason the device
 * model itself uses NAK instead of USB_RET_ASYNC here).
 */
typedef struct USBTCPPendingIn {
    tcp_usb_request_header hdr;
    uint8_t *data;     /* OUT data to (re-)send; NULL for IN */
    int nak_count;     /* consecutive NAKs, for adaptive poll backoff */
    uint64_t next_ns;  /* monotonic time this entry is next due for a re-poll */
    QTAILQ_ENTRY(USBTCPPendingIn) next;
} USBTCPPendingIn;

struct USBTCPHostState {
    SysBusDevice parent_obj;

    USBBus bus;
    USBPort ports[3];
    QIOChannel *ioc;
    CoMutex write_mutex;
    Error *migration_blocker;
    bool closed;
    bool stopped;
    USBTCPRemoteConnType conn_type;
    char *conn_addr;
    uint16_t conn_port;

    QTAILQ_HEAD(, USBTCPPendingIn) pending_ins;
    QEMUTimer *repoll_timer;
};

#endif /* HW_USB_HCD_TCP_H */
