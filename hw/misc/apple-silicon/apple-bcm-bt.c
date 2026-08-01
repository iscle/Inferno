/*
 * Apple iPhone 11 Broadcom BCM4378 Bluetooth (PCIe endpoint)
 *
 * Copyright (c) 2026 Inferno contributors.
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
 *
 * The Bluetooth half of the BCM4378 combo part. On T8030 it is function 1 of
 * the same PCI device as Wi-Fi (function 0), behind apcie pci-bridge2, with its
 * own DART stream (the device tree's "mapper-apcie2-bt").
 *
 * What is modelled: PCI configuration space, the BAR0 control registers and the
 * BAR2 status window, the firmware-download handshake (bootstage), the "RTI"
 * bring-up handshake, the "Converged IPC" ring protocol (transfer rings,
 * completion rings, doorbells, the shared head/tail index arrays), and an HCI
 * controller sitting on top of the HCI transfer/completion rings.
 *
 * There is no radio and no real firmware: the firmware image the host DMAs in
 * is accepted and discarded, and HCI is answered from a table. Everything
 * unhandled is logged so the next thing the host asks for can be observed.
 *
 * The protocol is the one Apple calls "Converged IPC" and drives from
 * AppleConvergedIPCOLYBTControl (whose IOKit personality in the iOS 18
 * kernelcache spells out every ring, doorbell index and footer size used
 * below); it is the same protocol the Linux hci_bcm4377 driver implements.
 */

#include "qemu/osdep.h"
#include "hw/arm/apple-silicon/dart.h"
#include "hw/arm/apple-silicon/dt.h"
#include "hw/irq.h"
#include "hw/misc/apple-silicon/apple-bcm-bt.h"
#include "hw/pci/msi.h"
#include "hw/pci/pci_device.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/dma.h"

#define TYPE_APPLE_BCM_BT_DEVICE "apple-bcm-bt-device"
OBJECT_DECLARE_SIMPLE_TYPE(AppleBCMBTDeviceState, APPLE_BCM_BT_DEVICE)

#define TYPE_APPLE_BCM_BT "apple-bcm-bt"
OBJECT_DECLARE_SIMPLE_TYPE(AppleBCMBTState, APPLE_BCM_BT)

/*
 * PCI identity. AppleConvergedPCI's IOPCIMatch in the iOS 18 kernelcache is
 * "0x5fa014e4 0x5f6914e4 0x5f3114e4 0x5f7114e4 0x5f7214e4 0x5f8314e4"; 0x5f69
 * is the BCM4378 Bluetooth function. Revision 3 makes ACIPCChip4378 log
 * "4378B1 detected", which is the part the iPhone 11 carries -- the OS only
 * ships BCM4378B1 firmware and PTB blobs.
 */
#define APPLE_BCM_BT_PCI_VENDOR_ID 0x14E4
#define APPLE_BCM_BT_PCI_DEVICE_ID 0x5F69
#define APPLE_BCM_BT_PCI_REVISION 0x03
/* Linux matches on PCI_CLASS_NETWORK_OTHER; that is what the part reports. */
#define APPLE_BCM_BT_PCI_CLASS 0x0280

/*
 * BAR sizes. ACIPCChip4378's constructor stores 0x02000000 as the BAR2 window
 * size and steers it at backplane 0x19000000; the status registers the host
 * polls live at BAR2 + 0x2004xx. BAR0 is the 32 KiB register aperture holding
 * the eight 4 KiB backplane windows.
 */
#define APPLE_BCM_BT_BAR0_SIZE (32 * KiB)
#define APPLE_BCM_BT_BAR2_SIZE (32 * MiB)

/*
 * BAR0 registers.
 *
 * These are the offsets ACIPCChip4378::registerOffset returns (a 39-entry jump
 * table at 0xfffffff008b9cf98 in the iOS 18.6.2 kernelcache). The first four
 * agree with the Linux hci_bcm4377 driver; the rest are ones only Apple's
 * driver uses.
 *
 * BAR0 is carved into eight 4 KiB backplane apertures, the same way the Wi-Fi
 * function's is, which is why the ChipCommon registers land at 0x3000 (window
 * 3 is pinned to ChipCommon) and the AXI2AHB bridge status at 0x1908.
 */
#define BT_BAR0_FW_DOORBELL 0x140
#define BT_BAR0_RTI_CONTROL 0x144
#define BT_BAR0_SLEEP_CONTROL 0x150
#define BT_BAR0_SLEEP_STATUS 0x154
/*
 * Linux rings every doorbell through one register with the index packed into
 * the written word. Apple's driver instead writes the new head straight into a
 * per-doorbell register, `0x6620 + 4 * index`. Both are modelled: the hardware
 * plainly implements both, and only one of the two drivers has to be believed
 * at a time.
 */
#define BT_BAR0_DOORBELL 0x174
#define BT_BAR0_DOORBELL_ARRAY 0x6620
#define BT_BAR0_DOORBELL_ARRAY_COUNT 32
#define BT_BAR0_HOST_WINDOW_LO 0x590
#define BT_BAR0_HOST_WINDOW_HI 0x594
#define BT_BAR0_HOST_WINDOW_SIZE 0x598
/* The OTP mirror, at core2-window1 (0x18011000) + 0x120. */
#define BT_BAR0_OTP_OFFSET 0x4120
#define BT_BAR0_OTP_SIZE 0xE0

/*
 * ChipCommon lives in BAR0 window 3. Only two registers are needed.
 *
 * ChipStatus bit 10 is the one that matters: ACIPCOLYBTControl's
 * checkBTClockStatus (@0xfffffff008b6a880) reads it and, if the bit is clear,
 * logs "BT not running 0x%x" and abandons bring-up without ever looking at the
 * boot stage. That is what a chip whose Bluetooth clock has not been turned on
 * looks like, and it is where an unmodelled endpoint stops.
 */
#define BT_BAR0_CHIPCOMMON 0x3000
#define BT_BAR0_CC_CHIPID (BT_BAR0_CHIPCOMMON + 0x00)
#define BT_BAR0_CC_CHIPSTATUS (BT_BAR0_CHIPCOMMON + 0x2C)
#define BT_CC_CHIPSTATUS_BT_CLOCK_RUNNING BIT(10)

#define BT_CHIP_ID 0x4378
#define BT_CHIP_REV 0x3
#define BT_CHIP_PACKAGE 0x0
#define BT_CC_CHIPID_VALUE                                 \
    ((BT_CHIP_ID & 0xFFFF) | ((BT_CHIP_REV & 0xF) << 16) | \
     ((BT_CHIP_PACKAGE & 0xF) << 20))

/*
 * The AXI2AHB bridge status, in BAR0 window 1. checkBTAXI2AHBStatus
 * (@0xfffffff008b6a684) rejects 0xFFFFFFFF (dead chip) and any value with
 * either of the low two bits set ("BT AXI2AHB bridge error"); zero is the
 * healthy answer.
 */
#define BT_BAR0_AXI2AHB_STATUS 0x1908

#define BT_DOORBELL_VALUE(v) (((v) >> 16) & 0xFFFF)
#define BT_DOORBELL_IDX(v) (((v) >> 8) & 0xFF)
#define BT_DOORBELL_RING BIT(5)

/*
 * BAR2 registers.
 *
 * Apple's driver reaches them at 0x4004xx and the Linux driver at 0x2004xx.
 * The low bits are identical, so the two only disagree on where the BAR2
 * backplane window is pointed; both aliases are decoded so either driver works
 * against this model.
 */
#define BT_BAR2_ALIAS_MASK 0x1FFFFF
#define BT_BAR2_ALIAS_BASE 0x200000
/*
 * 0x200450 is two registers in one: the host writes the high half of the
 * "Converged IPC" context address to it, and reads back the part's RTI
 * capability mask.
 *
 * ACIPCRTIDevice::setupConfiguration negotiates `host & device` and then
 * asserts on bit 1 -- with it clear the guest takes a kernel panic at
 * ACIPCRTIDevice.cpp:913. Bit 1 is the base capability Linux hardcodes as
 * `enabled_caps = 2`.
 *
 * Bits 4 and 5 are the optional "iso" and "thread" features (named by
 * ACIPCRTIDevice's "device RTI capabilities %#x, iso=%u, thread=%u"). They are
 * deliberately NOT advertised: nothing here implements either, and a
 * capability the host takes up and the device cannot honour is worse than one
 * that was never offered.
 */
#define BT_BAR2_CONTEXT_ADDR_HI 0x200450
#define BT_RTI_CAP_BASE BIT(1)
#define BT_RTI_CAP_ISO BIT(4)
#define BT_RTI_CAP_THREAD BIT(5)
#define BT_RTI_CAPABILITIES BT_RTI_CAP_BASE
#define BT_BAR2_BOOTSTAGE 0x200454
#define BT_BAR2_RTI_STATUS 0x20045C
#define BT_BAR2_FW_LO 0x200478
#define BT_BAR2_FW_HI 0x20047C
#define BT_BAR2_FW_SIZE 0x200480
#define BT_BAR2_CONTEXT_ADDR_LO 0x20048C
/*
 * Apple's driver puts the high half of the context address here rather than in
 * 0x200450 (which it uses for the capability read instead); the Linux driver
 * uses 0x200450. Both are accepted.
 */
#define BT_BAR2_CONTEXT_ADDR_HI_ALT 0x200490
#define BT_BAR2_RTI_WINDOW_LO 0x200494
#define BT_BAR2_RTI_WINDOW_HI 0x200498
#define BT_BAR2_RTI_WINDOW_SIZE 0x20049C

/* Bootstage / RTI values the host waits for. */
#define BT_BOOTSTAGE_COLD 0
#define BT_BOOTSTAGE_READY 2
#define BT_RTI_STATE_OFF 0
#define BT_RTI_STATE_STARTED 1
#define BT_RTI_STATE_RUNNING 2

/* How long the modelled firmware takes to come up / acknowledge RTI. */
#define BT_BOOT_DELAY_MS 20
#define BT_RTI_DELAY_MS 2

/* Converged IPC limits. The 4378 uses 9 transfer and 6 completion rings; the
 * V2 layout in the same personality uses 13 and 10. Sixteen covers both with
 * room to spare, and every index coming from the guest is checked against the
 * count it declared in the context. */
#define BT_MAX_XFER_RINGS 16
#define BT_MAX_COMPL_RINGS 16
#define BT_MAX_DOORBELLS 32

#define BT_XFER_ENTRY_SIZE 0x10
#define BT_COMPL_ENTRY_SIZE 0x10
#define BT_CONTROL_MSG_SIZE 0x34
/* Largest in-flight payload we will copy out of a ring. The biggest configured
 * footer is the ACL one at 4 KiB. */
#define BT_MAX_PAYLOAD 4096

/* Transfer ring entry flags. */
#define BT_XFER_FLAG_PAYLOAD_MAPPED BIT(0)
#define BT_XFER_FLAG_PAYLOAD_IN_FOOTER BIT(1)

/* Transfer ring creation flags. */
#define BT_XFER_RING_FLAG_VIRTUAL BIT(7)
#define BT_XFER_RING_FLAG_SYNC BIT(8)

/* Control message types. */
#define BT_CTRL_MSG_CREATE_XFER_RING 1
#define BT_CTRL_MSG_CREATE_COMPL_RING 2
#define BT_CTRL_MSG_DESTROY_XFER_RING 3
#define BT_CTRL_MSG_DESTROY_COMPL_RING 4

/*
 * Ring indices. These are fixed in the firmware -- Apple's personality and the
 * Linux driver agree on them, and the comment in hci_bcm4377.c notes that the
 * firmware ignores most of what the create message asks for.
 */
#define BT_XFER_RING_CONTROL 0
#define BT_XFER_RING_HCI_H2D 1
#define BT_XFER_RING_HCI_D2H 2
#define BT_XFER_RING_SCO_H2D 3
#define BT_XFER_RING_SCO_D2H 4
#define BT_XFER_RING_ACL_H2D 5
#define BT_XFER_RING_ACL_D2H 6

/* HCI packet plumbing. */
#define BT_HCI_MAX_EVENT (2 + 255)
#define BT_HCI_EVENT_QUEUE_LEN 64

#define BT_EVT_CONN_COMPLETE 0x03
#define BT_EVT_CMD_COMPLETE 0x0E
#define BT_EVT_CMD_STATUS 0x0F
#define BT_EVT_HARDWARE_ERROR 0x10

#define BT_STATUS_SUCCESS 0x00
#define BT_STATUS_UNKNOWN_COMMAND 0x01
#define BT_STATUS_UNSUPPORTED 0x11

#define BT_DPRINTF(fmt, ...)                                     \
    do {                                                         \
        if (APPLE_BCM_BT_DEBUG) {                                \
            fprintf(stderr, "apple-bcm-bt: " fmt, ##__VA_ARGS__); \
        }                                                        \
    } while (0)

#ifndef APPLE_BCM_BT_DEBUG
#define APPLE_BCM_BT_DEBUG 1
#endif

/*
 * Every BAR access, implemented or not. The host's exact register sequence is
 * the only ground truth available for the parts of this interface that are not
 * documented, so it is traced rather than inferred.
 */
#ifndef APPLE_BCM_BT_TRACE_MMIO
#define APPLE_BCM_BT_TRACE_MMIO 0
#endif

#define BT_TRACE(fmt, ...)                                              \
    do {                                                                \
        if (APPLE_BCM_BT_TRACE_MMIO) {                                  \
            fprintf(stderr, "apple-bcm-bt: " fmt, ##__VA_ARGS__);       \
        }                                                               \
    } while (0)

typedef struct {
    bool enabled;
    /* True once the host has told us this ring exists via a control message. */
    uint64_t iova;
    uint16_t n_entries;
    uint16_t header_bytes;
    uint16_t footer_bytes;
    uint16_t compl_ring;
    uint16_t doorbell;
    bool virt;
    bool sync;
    /* Our own copy of the consumer index. Mirrored into the host's array. */
    uint16_t tail;
    /* Incremented on every (re-)create, matching the host's generation. */
    uint8_t generation;
} AppleBCMBTXferRing;

typedef struct {
    bool enabled;
    uint64_t iova;
    uint16_t n_entries;
    uint16_t header_bytes;
    uint16_t footer_bytes;
    /* Producer index. Mirrored into the host's array. */
    uint16_t head;
} AppleBCMBTComplRing;

typedef struct {
    uint8_t data[BT_HCI_MAX_EVENT];
    uint16_t len;
} AppleBCMBTEvent;

struct AppleBCMBTState;

struct AppleBCMBTDeviceState {
    PCIDevice parent_obj;
    struct AppleBCMBTState *root;

    MemoryRegion bar0, bar2;

    ApplePCIEPort *port;
    MemoryRegion *dma_mr;
    AddressSpace *dma_as;
    /*
     * DART stream for this function. iOS gives Wi-Fi and Bluetooth separate
     * mappers ("mapper-apcie2-wlan" / "mapper-apcie2-bt"), so the stream this
     * function's requester id was mapped to is looked up in the port's
     * requester-id-to-stream-id table rather than assumed.
     */
    AddressSpace bt_dma_as;
    bool bt_dma_as_valid;
    uint32_t bt_sid;

    /* BAR0 shadow state. */
    uint32_t sleep_control;
    uint32_t sleep_status;
    uint32_t cc_chipstatus;
    uint32_t host_window_lo;
    uint32_t host_window_hi;
    uint32_t host_window_size;
    uint8_t otp[BT_BAR0_OTP_SIZE];

    /* BAR2 shadow state. */
    uint32_t bootstage;
    uint32_t rti_status;
    uint32_t fw_lo, fw_hi, fw_size;
    uint32_t ctx_lo, ctx_hi;
    uint32_t rti_window_lo, rti_window_hi, rti_window_size;

    QEMUTimer *boot_timer;
    QEMUTimer *rti_timer;
    uint32_t rti_target;

    /* "Converged IPC" context, as read out of guest memory. */
    bool ctx_valid;
    uint16_t ctx_version;
    uint64_t peripheral_info_addr;
    uint64_t compl_heads_addr;
    uint64_t compl_tails_addr;
    uint64_t xfer_heads_addr;
    uint64_t xfer_tails_addr;
    uint16_t n_compl_rings;
    uint16_t n_xfer_rings;

    AppleBCMBTXferRing xfer[BT_MAX_XFER_RINGS];
    AppleBCMBTComplRing compl_ring[BT_MAX_COMPL_RINGS];

    /* Controller identity. */
    uint8_t bdaddr[6];

    /* Events produced before the HCI device-to-host ring existed. */
    AppleBCMBTEvent evq[BT_HCI_EVENT_QUEUE_LEN];
    unsigned evq_head, evq_tail;
};

struct AppleBCMBTState {
    SysBusDevice parent_obj;
    PCIBus *pci_bus;
    AppleBCMBTDeviceState *device;
};

/* ------------------------------------------------------------------ */
/* Little-endian accessors for the packed on-the-wire structures.      */
/* ------------------------------------------------------------------ */

static uint16_t bt_ld16(const uint8_t *p)
{
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static uint32_t bt_ld32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}

static uint64_t bt_ld64(const uint8_t *p)
{
    return (uint64_t)bt_ld32(p) | ((uint64_t)bt_ld32(p + 4) << 32);
}

static void bt_st16(uint8_t *p, uint16_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
}

static void bt_st32(uint8_t *p, uint32_t v)
{
    p[0] = v & 0xFF;
    p[1] = (v >> 8) & 0xFF;
    p[2] = (v >> 16) & 0xFF;
    p[3] = (v >> 24) & 0xFF;
}

/* ------------------------------------------------------------------ */
/* DMA                                                                */
/* ------------------------------------------------------------------ */

static AddressSpace *bt_dma_as(AppleBCMBTDeviceState *s)
{
    return s->bt_dma_as_valid ? &s->bt_dma_as : s->dma_as;
}

static bool bt_dma_allowed(AppleBCMBTDeviceState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);

    /*
     * The device reaches memory through the apcie port's own AddressSpace,
     * which bypasses QEMU's bus-master gate, so the gate has to be honoured
     * here -- exactly as the Wi-Fi function does.
     */
    if ((pci_get_word(pci_dev->config + PCI_COMMAND) & PCI_COMMAND_MASTER) ==
        0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: DMA attempted with bus mastering "
                      "disabled\n");
        return false;
    }
    if (bt_dma_as(s) == NULL) {
        return false;
    }
    return true;
}

static bool bt_dma_read(AppleBCMBTDeviceState *s, uint64_t addr, void *buf,
                        size_t len)
{
    if (!bt_dma_allowed(s)) {
        return false;
    }
    if (dma_memory_read(bt_dma_as(s), addr, buf, len, MEMTXATTRS_UNSPECIFIED) !=
        MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: DMA read of 0x%zx bytes at 0x%" PRIx64
                      " failed\n",
                      len, addr);
        return false;
    }
    return true;
}

static bool bt_dma_write(AppleBCMBTDeviceState *s, uint64_t addr,
                         const void *buf, size_t len)
{
    if (!bt_dma_allowed(s)) {
        return false;
    }
    if (dma_memory_write(bt_dma_as(s), addr, buf, len,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: DMA write of 0x%zx bytes at 0x%" PRIx64
                      " failed\n",
                      len, addr);
        return false;
    }
    return true;
}

static bool bt_read_index(AppleBCMBTDeviceState *s, uint64_t base,
                          unsigned index, uint16_t *out)
{
    uint8_t raw[2];

    if (base == 0) {
        return false;
    }
    if (!bt_dma_read(s, base + index * 2u, raw, sizeof(raw))) {
        return false;
    }
    *out = bt_ld16(raw);
    return true;
}

static void bt_write_index(AppleBCMBTDeviceState *s, uint64_t base,
                           unsigned index, uint16_t value)
{
    uint8_t raw[2];

    if (base == 0) {
        return;
    }
    bt_st16(raw, value);
    bt_dma_write(s, base + index * 2u, raw, sizeof(raw));
}

/*
 * Resolve the DART stream this function's requester id has been mapped to.
 *
 * The apcie port keeps the requester-id-to-stream-id table the guest programs
 * at its 0x828 window. Wi-Fi and Bluetooth are different requester ids (device
 * 0 functions 0 and 1) with different mappers in the device tree, so they end
 * up on different streams and must not share an address space.
 */
static void bt_resolve_dma_as(AppleBCMBTDeviceState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);
    uint16_t rid;
    AppleDARTState *dart;
    IOMMUMemoryRegion *mr;
    unsigned i;

    if (s->bt_dma_as_valid || s->port == NULL) {
        return;
    }

    rid = ((uint16_t)pci_bus_num(pci_get_bus(pci_dev)) << 8) | pci_dev->devfn;

    for (i = 0; i < ARRAY_SIZE(s->port->port_rid_sid_map); i++) {
        uint32_t entry = s->port->port_rid_sid_map[i];

        if ((entry & BIT(31)) == 0) {
            continue;
        }
        if ((entry & 0xFFFF) != rid) {
            continue;
        }

        BT_DPRINTF("requester id 0x%04x is mapped to stream %u (0x%08x)\n", rid,
                   i, entry);

        if (i == 1) {
            /* The stream the port's shared address space already uses. */
            return;
        }

        dart = APPLE_DART(object_property_get_link(OBJECT(qdev_get_machine()),
                                                   "dart-apcie2", NULL));
        if (dart == NULL) {
            return;
        }
        mr = apple_dart_iommu_mr(dart, i);
        if (mr == NULL) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "apple-bcm-bt: no DART stream %u\n", i);
            return;
        }
        address_space_init(&s->bt_dma_as, MEMORY_REGION(mr), "apple-bcm-bt.dma");
        s->bt_dma_as_valid = true;
        s->bt_sid = i;
        return;
    }

    BT_DPRINTF("requester id 0x%04x has no stream mapping yet\n", rid);
}

static void bt_raise_msi(AppleBCMBTDeviceState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);

    if (!msi_enabled(pci_dev)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: MSI disabled, cannot signal the host\n");
        return;
    }
    /*
     * One message per item. The AIC latches message-signalled vectors until a
     * CPU takes them, so there is nothing to re-assert and no retry ladder.
     */
    msi_notify(pci_dev, 0);
}

/* ------------------------------------------------------------------ */
/* Completion rings                                                   */
/* ------------------------------------------------------------------ */

/*
 * Append one entry to a completion ring and tell the host about it.
 *
 * `xfer_ring` is the transfer ring the entry belongs to -- that is what the
 * host switches on to decide whether this is an acknowledgement or an inbound
 * packet. `msg_id` echoes the raw (generation | id) word from the transfer ring
 * entry that caused it.
 */
static bool bt_compl_post(AppleBCMBTDeviceState *s, unsigned compl_index,
                          uint16_t xfer_ring, uint16_t msg_id, uint8_t flags,
                          const void *payload, uint32_t payload_len)
{
    AppleBCMBTComplRing *ring;
    uint8_t entry[BT_COMPL_ENTRY_SIZE];
    uint16_t tail;
    uint16_t next;
    uint32_t stride;
    uint64_t addr;

    if (compl_index >= BT_MAX_COMPL_RINGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: completion ring %u out of range\n",
                      compl_index);
        return false;
    }
    ring = &s->compl_ring[compl_index];
    if (!ring->enabled || ring->n_entries == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: completion ring %u is not enabled\n",
                      compl_index);
        return false;
    }
    if (payload_len > ring->footer_bytes) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: %u byte payload does not fit the %u byte "
                      "footer of completion ring %u\n",
                      payload_len, ring->footer_bytes, compl_index);
        return false;
    }

    next = (ring->head + 1) % ring->n_entries;
    if (bt_read_index(s, s->compl_tails_addr, compl_index, &tail) &&
        next == tail) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: completion ring %u is full\n",
                      compl_index);
        return false;
    }

    stride = BT_COMPL_ENTRY_SIZE + ring->header_bytes + ring->footer_bytes;
    addr = ring->iova + (uint64_t)ring->head * stride + ring->header_bytes;

    memset(entry, 0, sizeof(entry));
    entry[0] = flags;
    bt_st16(entry + 2, xfer_ring);
    bt_st16(entry + 4, msg_id);
    bt_st32(entry + 6, payload_len);

    if (!bt_dma_write(s, addr, entry, sizeof(entry))) {
        return false;
    }
    if (payload_len != 0 &&
        !bt_dma_write(s, addr + BT_COMPL_ENTRY_SIZE, payload, payload_len)) {
        return false;
    }

    ring->head = next;
    bt_write_index(s, s->compl_heads_addr, compl_index, ring->head);
    bt_raise_msi(s);
    return true;
}

/* ------------------------------------------------------------------ */
/* HCI                                                                */
/* ------------------------------------------------------------------ */

static void bt_hci_flush_events(AppleBCMBTDeviceState *s);

static void bt_hci_queue_event(AppleBCMBTDeviceState *s, const uint8_t *data,
                               uint16_t len)
{
    unsigned next;

    if (len > BT_HCI_MAX_EVENT) {
        qemu_log_mask(LOG_UNIMP, "apple-bcm-bt: oversized HCI event (%u)\n",
                      len);
        return;
    }
    next = (s->evq_head + 1) % BT_HCI_EVENT_QUEUE_LEN;
    if (next == s->evq_tail) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: HCI event queue overflow, dropping\n");
        return;
    }
    memcpy(s->evq[s->evq_head].data, data, len);
    s->evq[s->evq_head].len = len;
    s->evq_head = next;

    bt_hci_flush_events(s);
}

static void bt_hci_flush_events(AppleBCMBTDeviceState *s)
{
    AppleBCMBTXferRing *ring = &s->xfer[BT_XFER_RING_HCI_D2H];

    if (!ring->enabled || ring->n_entries == 0) {
        return;
    }

    while (s->evq_tail != s->evq_head) {
        AppleBCMBTEvent *ev = &s->evq[s->evq_tail];
        uint16_t host_head;
        uint16_t msg_id;

        /*
         * A device-to-host ring is a credit pool: the host advances its head
         * for every buffer it is willing to receive into. Without a credit
         * there is nowhere to put the event, so hold it until the host grants
         * one (it does so from its own interrupt handler after consuming).
         */
        if (!bt_read_index(s, s->xfer_heads_addr, BT_XFER_RING_HCI_D2H,
                           &host_head)) {
            return;
        }
        if (host_head == ring->tail) {
            return;
        }

        msg_id = ((uint16_t)ring->generation << 8) |
                 (ring->tail % ring->n_entries);

        if (!bt_compl_post(s, ring->compl_ring, BT_XFER_RING_HCI_D2H, msg_id,
                           BT_XFER_FLAG_PAYLOAD_IN_FOOTER, ev->data, ev->len)) {
            return;
        }

        ring->tail = (ring->tail + 1) % ring->n_entries;
        bt_write_index(s, s->xfer_tails_addr, BT_XFER_RING_HCI_D2H, ring->tail);
        s->evq_tail = (s->evq_tail + 1) % BT_HCI_EVENT_QUEUE_LEN;
    }
}

static void bt_hci_cmd_complete(AppleBCMBTDeviceState *s, uint16_t opcode,
                                uint8_t status, const void *params,
                                uint8_t params_len)
{
    uint8_t ev[BT_HCI_MAX_EVENT];
    unsigned n = 0;

    if ((unsigned)params_len + 4u > sizeof(ev)) {
        return;
    }

    ev[n++] = BT_EVT_CMD_COMPLETE;
    ev[n++] = 3 + 1 + params_len; /* ncmd + opcode + status + params */
    ev[n++] = 1; /* the host may send one more command */
    bt_st16(ev + n, opcode);
    n += 2;
    ev[n++] = status;
    if (params_len != 0) {
        memcpy(ev + n, params, params_len);
        n += params_len;
    }
    bt_hci_queue_event(s, ev, n);
}

static void bt_hci_cmd_status(AppleBCMBTDeviceState *s, uint16_t opcode,
                              uint8_t status)
{
    uint8_t ev[6];

    ev[0] = BT_EVT_CMD_STATUS;
    ev[1] = 4;
    ev[2] = status;
    ev[3] = 1;
    bt_st16(ev + 4, opcode);
    bt_hci_queue_event(s, ev, sizeof(ev));
}

/*
 * Commands that are answered with a Command Status rather than a Command
 * Complete: the ones that start a procedure whose result arrives later.
 */
static bool bt_hci_is_status_command(uint16_t opcode)
{
    switch (opcode) {
    case 0x0401: /* Inquiry */
    case 0x0405: /* Create Connection */
    case 0x0406: /* Disconnect */
    case 0x0409: /* Accept Connection Request */
    case 0x040B: /* Link Key Request Reply is CC; 0x040B is Auth Requested */
    case 0x0411: /* Set Connection Encryption */
    case 0x0419: /* Remote Name Request */
    case 0x041B: /* Read Remote Supported Features */
    case 0x041C: /* Read Remote Extended Features */
    case 0x041D: /* Read Remote Version Information */
    case 0x0428: /* Setup Synchronous Connection */
    case 0x0429: /* Accept Synchronous Connection Request */
    case 0x043D: /* Enhanced Setup Synchronous Connection */
    case 0x200D: /* LE Create Connection */
    case 0x2013: /* LE Connection Update */
    case 0x2016: /* LE Read Remote Features */
    case 0x2017: /* LE Encrypt is CC; 0x2019 Start Encryption is CS */
    case 0x2019: /* LE Start Encryption */
    case 0x2043: /* LE Extended Create Connection */
    case 0x2044: /* LE Periodic Advertising Create Sync */
        return true;
    default:
        return false;
    }
}

static void bt_hci_handle_command(AppleBCMBTDeviceState *s, const uint8_t *cmd,
                                  uint16_t len)
{
    uint16_t opcode;
    uint8_t plen;
    const uint8_t *params;
    uint8_t buf[64];

    if (len < 3) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: runt HCI command (%u bytes)\n", len);
        return;
    }
    opcode = bt_ld16(cmd);
    plen = cmd[2];
    params = cmd + 3;
    if ((unsigned)plen + 3u > len) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: HCI command 0x%04x claims %u parameter "
                      "bytes but only %u are present\n",
                      opcode, plen, len - 3);
        plen = len - 3;
    }

    BT_DPRINTF("HCI command 0x%04x (%u bytes of parameters)\n", opcode, plen);

    switch (opcode) {
    /* ---- Informational parameters ---- */
    case 0x1001: /* Read Local Version Information */
        memset(buf, 0, 8);
        buf[0] = 0x0B; /* HCI version: Bluetooth 5.2 */
        bt_st16(buf + 1, 0x0DAC); /* HCI revision */
        buf[3] = 0x0B; /* LMP version */
        bt_st16(buf + 4, 0x000F); /* Manufacturer: Broadcom */
        bt_st16(buf + 6, 0x420E); /* LMP subversion */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 8);
        return;
    case 0x1002: /* Read Local Supported Commands */
        /*
         * Advertise nothing beyond what is answered below: a host that is told
         * a command exists and then gets "unknown command" back is in a worse
         * place than one that never asks. The bits set here are the ones the
         * table in this function really implements.
         */
        memset(buf, 0, 64);
        buf[0] = 0x20; /* Disconnect */
        buf[2] = 0x80; /* Read Remote Version Information */
        buf[5] = 0xC0; /* Set Event Mask, Reset */
        buf[6] = 0x01; /* Set Event Filter */
        buf[7] = 0x60; /* Read/Write Local Name */
        buf[10] = 0x80; /* Host Buffer Size */
        buf[13] = 0x0C; /* Read/Write Class of Device */
        buf[14] = 0xC8; /* Read Local Version/Supported Features/Buffer Size */
        buf[15] = 0x02; /* Read BD_ADDR */
        buf[17] = 0x40; /* Read Local Extended Features */
        buf[22] = 0x04; /* Set Event Mask Page 2 */
        buf[24] = 0x60; /* Read/Write LE Host Supported */
        buf[25] = 0xF7; /* LE Set Event Mask .. LE Set Advertising Data */
        buf[26] = 0xFF;
        buf[27] = 0xFF;
        buf[28] = 0xFF;
        buf[32] = 0x08; /* LE Read Supported States */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 64);
        return;
    case 0x1003: /* Read Local Supported Features */
        memset(buf, 0, 8);
        buf[4] = 0x40; /* BR/EDR not supported is *not* set; LE supported */
        buf[6] = 0x60; /* LE supported (controller), BR/EDR+LE simultaneous */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 8);
        return;
    case 0x1004: /* Read Local Extended Features */
        memset(buf, 0, 10);
        buf[0] = plen >= 1 ? params[0] : 0; /* page */
        buf[1] = 2; /* maximum page */
        if (buf[0] == 0) {
            buf[6] = 0x60;
        } else if (buf[0] == 1) {
            buf[2] = 0x03; /* SSP host support, LE host support */
        }
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 10);
        return;
    case 0x1005: /* Read Buffer Size */
        bt_st16(buf, 1021); /* ACL data packet length */
        buf[2] = 96; /* SCO data packet length */
        bt_st16(buf + 3, 8); /* total ACL packets */
        bt_st16(buf + 5, 8); /* total SCO packets */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 7);
        return;
    case 0x1009: /* Read BD_ADDR */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, s->bdaddr, 6);
        return;
    case 0x100B: /* Read Local Supported Codecs */
        buf[0] = 0; /* number of standard codecs */
        buf[1] = 0; /* number of vendor codecs */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 2);
        return;

    /* ---- Controller & baseband ---- */
    case 0x0C03: /* Reset */
        s->evq_head = s->evq_tail = 0;
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, NULL, 0);
        return;
    case 0x0C01: /* Set Event Mask */
    case 0x0C05: /* Set Event Filter */
    case 0x0C13: /* Write Local Name */
    case 0x0C16: /* Write Connection Accept Timeout */
    case 0x0C18: /* Write Page Timeout */
    case 0x0C1A: /* Write Scan Enable */
    case 0x0C1C: /* Write Page Scan Activity */
    case 0x0C1E: /* Write Inquiry Scan Activity */
    case 0x0C20: /* Write Authentication Enable */
    case 0x0C24: /* Write Class of Device */
    case 0x0C26: /* Write Voice Setting */
    case 0x0C2D: /* Write Automatic Flush Timeout */
    case 0x0C31: /* Set Controller To Host Flow Control */
    case 0x0C33: /* Host Buffer Size */
    case 0x0C3A: /* Write Current IAC LAP */
    case 0x0C3F: /* Write Page Scan Type */
    case 0x0C43: /* Write Inquiry Scan Type */
    case 0x0C45: /* Write Inquiry Mode */
    case 0x0C47: /* Write Page Scan Type (deprecated alias) */
    case 0x0C52: /* Write Extended Inquiry Response */
    case 0x0C56: /* Write Simple Pairing Mode */
    case 0x0C5B: /* Write Default Erroneous Data Reporting */
    case 0x0C63: /* Set Event Mask Page 2 */
    case 0x0C6D: /* Write LE Host Supported */
    case 0x0C7A: /* Write Secure Connections Host Support */
    case 0x0C7D: /* Read Local OOB Extended Data is CC+data; write path */
    case 0x0C81: /* Write Authenticated Payload Timeout */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, NULL, 0);
        return;
    case 0x0C35: /* Host Number Of Completed Packets: no event at all */
        return;
    case 0x0C14: /* Read Local Name */
        memset(buf, 0, sizeof(buf));
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, NULL, 0);
        return;
    case 0x0C23: /* Read Class of Device */
        buf[0] = buf[1] = buf[2] = 0;
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 3);
        return;
    case 0x0C25: /* Read Voice Setting */
        bt_st16(buf, 0x0060);
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 2);
        return;
    case 0x0C6C: /* Read LE Host Supported */
        buf[0] = 1;
        buf[1] = 0;
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 2);
        return;
    case 0x0C7B: /* Read Secure Connections Host Support */
        buf[0] = 1;
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 1);
        return;

    /* ---- LE controller ---- */
    case 0x2001: /* LE Set Event Mask */
    case 0x2005: /* LE Set Random Address */
    case 0x2006: /* LE Set Advertising Parameters */
    case 0x2008: /* LE Set Advertising Data */
    case 0x2009: /* LE Set Scan Response Data */
    case 0x200A: /* LE Set Advertise Enable */
    case 0x200B: /* LE Set Scan Parameters */
    case 0x200C: /* LE Set Scan Enable */
    case 0x2010: /* LE Clear Filter Accept List */
    case 0x2011: /* LE Add Device To Filter Accept List */
    case 0x2012: /* LE Remove Device From Filter Accept List */
    case 0x2020: /* LE Remove connection param request reply etc. */
    case 0x2024: /* LE Write Suggested Default Data Length */
    case 0x2029: /* LE Clear Resolving List */
    case 0x202D: /* LE Set Address Resolution Enable */
    case 0x202E: /* LE Set Resolvable Private Address Timeout */
    case 0x2031: /* LE Set Default PHY */
    case 0x2035: /* LE Set Extended Advertising Parameters (CC w/ tx power) */
    case 0x2036: /* LE Set Extended Advertising Data */
    case 0x2037: /* LE Set Extended Scan Response Data */
    case 0x2039: /* LE Set Extended Advertising Enable */
    case 0x203C: /* LE Remove Advertising Set */
    case 0x203D: /* LE Clear Advertising Sets */
    case 0x2041: /* LE Set Extended Scan Parameters */
    case 0x2042: /* LE Set Extended Scan Enable */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, NULL, 0);
        return;
    case 0x2002: /* LE Read Buffer Size */
        bt_st16(buf, 251);
        buf[2] = 8;
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 3);
        return;
    case 0x2060: /* LE Read Buffer Size v2 */
        bt_st16(buf, 251);
        buf[2] = 8;
        bt_st16(buf + 3, 0);
        buf[5] = 0;
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 6);
        return;
    case 0x2003: /* LE Read Local Supported Features */
        memset(buf, 0, 8);
        buf[0] = 0x21; /* LE Encryption, LE Data Packet Length Extension */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 8);
        return;
    case 0x2007: /* LE Read Advertising Channel TX Power */
        buf[0] = 7;
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 1);
        return;
    case 0x200F: /* LE Read Filter Accept List Size */
    case 0x202A: /* LE Read Resolving List Size */
    case 0x203A: /* LE Read Number of Supported Advertising Sets */
        buf[0] = 8;
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 1);
        return;
    case 0x201C: /* LE Read Supported States */
        memset(buf, 0xFF, 8);
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 8);
        return;
    case 0x2018: /* LE Rand */
        memset(buf, 0, 8);
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 8);
        return;
    case 0x2023: /* LE Read Maximum Data Length */
        bt_st16(buf, 251);
        bt_st16(buf + 2, 2120);
        bt_st16(buf + 4, 251);
        bt_st16(buf + 6, 2120);
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 8);
        return;
    case 0x2022: /* LE Read Suggested Default Data Length */
        bt_st16(buf, 27);
        bt_st16(buf + 2, 328);
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 4);
        return;
    case 0x203F: /* LE Read Maximum Advertising Data Length */
        bt_st16(buf, 1650);
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 2);
        return;
    case 0x2030: /* LE Read PHY */
        memset(buf, 0, 4);
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, buf, 4);
        return;

    /* ---- Broadcom / Apple vendor commands ---- */
    case 0xFC01: /* Change BD_ADDR */
        if (plen >= 6) {
            memcpy(s->bdaddr, params, 6);
        }
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, NULL, 0);
        return;
    case 0xFD97: /* Send calibration chunk ("Taurus") */
    case 0xFD98: /* Send PTB in one go */
    case 0xFE0D: /* Send PTB chunk */
        bt_hci_cmd_complete(s, opcode, BT_STATUS_SUCCESS, NULL, 0);
        return;

    default:
        break;
    }

    if (bt_hci_is_status_command(opcode)) {
        bt_hci_cmd_status(s, opcode, BT_STATUS_UNKNOWN_COMMAND);
        qemu_log_mask(LOG_UNIMP,
                      "apple-bcm-bt: unimplemented HCI command 0x%04x "
                      "(answered with Command Status \"unknown command\")\n",
                      opcode);
        return;
    }

    qemu_log_mask(LOG_UNIMP,
                  "apple-bcm-bt: unimplemented HCI command 0x%04x "
                  "(ogf %u ocf 0x%03x, %u parameter bytes)\n",
                  opcode, opcode >> 10, opcode & 0x3FF, plen);
    bt_hci_cmd_complete(s, opcode, BT_STATUS_UNKNOWN_COMMAND, NULL, 0);
}

/* ------------------------------------------------------------------ */
/* Control ring                                                       */
/* ------------------------------------------------------------------ */

static void bt_ctrl_create_compl_ring(AppleBCMBTDeviceState *s,
                                      const uint8_t *msg)
{
    unsigned id = bt_ld16(msg + 4);
    AppleBCMBTComplRing *ring;

    if (id >= BT_MAX_COMPL_RINGS || id >= s->n_compl_rings) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: create completion ring: bad index %u\n",
                      id);
        return;
    }
    ring = &s->compl_ring[id];
    ring->header_bytes = msg[1] * 4u;
    ring->footer_bytes = msg[2] * 4u;
    ring->iova = bt_ld64(msg + 8);
    ring->n_entries = bt_ld16(msg + 0x10);
    ring->head = 0;
    ring->enabled = ring->n_entries != 0;

    BT_DPRINTF("completion ring %u: %u entries at 0x%" PRIx64
               ", header %u footer %u\n",
               id, ring->n_entries, ring->iova, ring->header_bytes,
               ring->footer_bytes);
}

static void bt_ctrl_create_xfer_ring(AppleBCMBTDeviceState *s,
                                     const uint8_t *msg)
{
    unsigned id = bt_ld16(msg + 4);
    uint16_t flags;
    AppleBCMBTXferRing *ring;

    if (id >= BT_MAX_XFER_RINGS || id >= s->n_xfer_rings) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: create transfer ring: bad index %u\n", id);
        return;
    }
    ring = &s->xfer[id];
    flags = bt_ld16(msg + 0x1E);

    ring->header_bytes = msg[1] * 4u;
    ring->footer_bytes = msg[2] * 4u;
    ring->iova = bt_ld64(msg + 8);
    ring->n_entries = bt_ld16(msg + 0x18);
    ring->compl_ring = bt_ld16(msg + 0x1A);
    ring->doorbell = bt_ld16(msg + 0x1C);
    ring->virt = (flags & BT_XFER_RING_FLAG_VIRTUAL) != 0;
    ring->sync = (flags & BT_XFER_RING_FLAG_SYNC) != 0;
    ring->tail = 0;
    ring->generation++;
    ring->enabled = ring->n_entries != 0;

    BT_DPRINTF("transfer ring %u: %u entries at 0x%" PRIx64
               ", footer %u, completion ring %u, doorbell %u%s%s\n",
               id, ring->n_entries, ring->iova, ring->footer_bytes,
               ring->compl_ring, ring->doorbell, ring->virt ? " virtual" : "",
               ring->sync ? " sync" : "");

    if (id == BT_XFER_RING_HCI_D2H) {
        bt_hci_flush_events(s);
    }
}

static void bt_ctrl_handle_message(AppleBCMBTDeviceState *s, const uint8_t *msg,
                                   uint16_t len)
{
    unsigned id;

    if (len < 4) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: runt control message (%u bytes)\n", len);
        return;
    }

    switch (msg[0]) {
    case BT_CTRL_MSG_CREATE_XFER_RING:
        if (len < BT_CONTROL_MSG_SIZE) {
            break;
        }
        bt_ctrl_create_xfer_ring(s, msg);
        return;
    case BT_CTRL_MSG_CREATE_COMPL_RING:
        if (len < BT_CONTROL_MSG_SIZE) {
            break;
        }
        bt_ctrl_create_compl_ring(s, msg);
        return;
    case BT_CTRL_MSG_DESTROY_XFER_RING:
        id = bt_ld16(msg + 2);
        if (id < BT_MAX_XFER_RINGS) {
            s->xfer[id].enabled = false;
            BT_DPRINTF("transfer ring %u destroyed\n", id);
        }
        return;
    case BT_CTRL_MSG_DESTROY_COMPL_RING:
        id = bt_ld16(msg + 2);
        if (id < BT_MAX_COMPL_RINGS) {
            s->compl_ring[id].enabled = false;
            BT_DPRINTF("completion ring %u destroyed\n", id);
        }
        return;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "apple-bcm-bt: unknown control message type %u\n",
                      msg[0]);
        return;
    }

    qemu_log_mask(LOG_GUEST_ERROR,
                  "apple-bcm-bt: control message type %u is %u bytes, "
                  "expected %u\n",
                  msg[0], len, BT_CONTROL_MSG_SIZE);
}

/* ------------------------------------------------------------------ */
/* Transfer rings                                                     */
/* ------------------------------------------------------------------ */

/*
 * Consume one host-to-device transfer ring entry: fetch its payload (in the
 * entry's footer, or out of line through the mapped address), hand it to the
 * right consumer, then acknowledge it in the ring's completion ring.
 */
static void bt_xfer_process_entry(AppleBCMBTDeviceState *s, unsigned ring_id,
                                  const uint8_t *entry)
{
    AppleBCMBTXferRing *ring = &s->xfer[ring_id];
    uint8_t payload[BT_MAX_PAYLOAD];
    uint16_t msg_id = bt_ld16(entry + 0xC);
    uint16_t len = bt_ld16(entry + 1);
    uint8_t flags = entry[0];
    bool have_payload = false;

    if (len > sizeof(payload)) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: ring %u entry claims %u payload bytes, "
                      "clamping to %zu\n",
                      ring_id, len, sizeof(payload));
        len = sizeof(payload);
    }

    if (len != 0) {
        if (flags & BT_XFER_FLAG_PAYLOAD_MAPPED) {
            have_payload = bt_dma_read(s, bt_ld64(entry + 4), payload, len);
        } else {
            /* In the entry's own footer, right after the 16 byte header. */
            uint64_t at = ring->iova +
                          (uint64_t)((ring->tail) % ring->n_entries) *
                              (BT_XFER_ENTRY_SIZE + ring->header_bytes +
                               ring->footer_bytes) +
                          ring->header_bytes + BT_XFER_ENTRY_SIZE;

            if (len > ring->footer_bytes) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "apple-bcm-bt: ring %u in-footer payload of %u "
                              "bytes exceeds the %u byte footer\n",
                              ring_id, len, ring->footer_bytes);
                len = ring->footer_bytes;
            }
            have_payload = len == 0 || bt_dma_read(s, at, payload, len);
        }
    }

    if (have_payload || len == 0) {
        switch (ring_id) {
        case BT_XFER_RING_CONTROL:
            bt_ctrl_handle_message(s, payload, len);
            break;
        case BT_XFER_RING_HCI_H2D:
            bt_hci_handle_command(s, payload, len);
            break;
        case BT_XFER_RING_ACL_H2D:
        case BT_XFER_RING_SCO_H2D:
            /* Accepted and dropped: there is no peer to carry it to. */
            break;
        default:
            qemu_log_mask(LOG_UNIMP,
                          "apple-bcm-bt: %u byte message on unhandled "
                          "host-to-device ring %u\n",
                          len, ring_id);
            break;
        }
    }

    /* Acknowledge, so the host frees the message id and wakes any waiter. */
    bt_compl_post(s, ring->compl_ring, ring_id, msg_id, 0, NULL, 0);
}

static void bt_xfer_service(AppleBCMBTDeviceState *s, unsigned ring_id)
{
    AppleBCMBTXferRing *ring = &s->xfer[ring_id];
    uint16_t head;
    unsigned guard;

    if (!ring->enabled || ring->n_entries == 0) {
        return;
    }

    if (ring->virt) {
        /*
         * Nothing to read: the host only advanced its head to hand us credit
         * for device-to-host traffic. Anything queued can go out now.
         */
        bt_hci_flush_events(s);
        return;
    }

    if (!bt_read_index(s, s->xfer_heads_addr, ring_id, &head)) {
        return;
    }
    if (head >= ring->n_entries) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: ring %u head %u is past its %u entries\n",
                      ring_id, head, ring->n_entries);
        return;
    }

    for (guard = 0; ring->tail != head && guard < ring->n_entries; guard++) {
        uint8_t entry[BT_XFER_ENTRY_SIZE];
        uint32_t stride =
            BT_XFER_ENTRY_SIZE + ring->header_bytes + ring->footer_bytes;
        uint64_t at =
            ring->iova + (uint64_t)ring->tail * stride + ring->header_bytes;

        if (!bt_dma_read(s, at, entry, sizeof(entry))) {
            return;
        }
        bt_xfer_process_entry(s, ring_id, entry);

        ring->tail = (ring->tail + 1) % ring->n_entries;
        bt_write_index(s, s->xfer_tails_addr, ring_id, ring->tail);
    }
}

static void bt_doorbell(AppleBCMBTDeviceState *s, unsigned index)
{
    unsigned i;
    bool any = false;

    if (!s->ctx_valid) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: doorbell %u before the context was "
                      "handed over\n",
                      index);
        return;
    }

    for (i = 0; i < BT_MAX_XFER_RINGS; i++) {
        if (s->xfer[i].enabled && s->xfer[i].doorbell == index) {
            any = true;
            bt_xfer_service(s, i);
        }
    }

    if (!any) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: doorbell %u rung with no ring bound to "
                      "it\n",
                      index);
    }
}

/* ------------------------------------------------------------------ */
/* Context and bring-up                                               */
/* ------------------------------------------------------------------ */

static bool bt_load_context(AppleBCMBTDeviceState *s)
{
    uint8_t ctx[0x68];
    uint64_t addr = ((uint64_t)s->ctx_hi << 32) | s->ctx_lo;
    AppleBCMBTXferRing *ctrl_xfer;
    AppleBCMBTComplRing *ctrl_compl;
    uint8_t peripheral_info[0x20];

    if (addr == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: RTI stage 2 with a null context "
                      "address\n");
        return false;
    }
    bt_resolve_dma_as(s);
    if (!bt_dma_read(s, addr, ctx, sizeof(ctx))) {
        return false;
    }

    s->ctx_version = bt_ld16(ctx + 0x00);
    s->peripheral_info_addr = bt_ld64(ctx + 0x08);
    s->compl_heads_addr = bt_ld64(ctx + 0x10);
    s->xfer_tails_addr = bt_ld64(ctx + 0x18);
    s->compl_tails_addr = bt_ld64(ctx + 0x20);
    s->xfer_heads_addr = bt_ld64(ctx + 0x28);
    s->n_compl_rings = bt_ld16(ctx + 0x30);
    s->n_xfer_rings = bt_ld16(ctx + 0x32);

    if (s->n_compl_rings > BT_MAX_COMPL_RINGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: host asked for %u completion rings, "
                      "clamping to %u\n",
                      s->n_compl_rings, BT_MAX_COMPL_RINGS);
        s->n_compl_rings = BT_MAX_COMPL_RINGS;
    }
    if (s->n_xfer_rings > BT_MAX_XFER_RINGS) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: host asked for %u transfer rings, "
                      "clamping to %u\n",
                      s->n_xfer_rings, BT_MAX_XFER_RINGS);
        s->n_xfer_rings = BT_MAX_XFER_RINGS;
    }

    /*
     * The control rings are not created with a control message -- they are the
     * channel those messages travel on -- so they come straight out of the
     * context.
     */
    ctrl_compl = &s->compl_ring[0];
    ctrl_compl->iova = bt_ld64(ctx + 0x34);
    ctrl_compl->n_entries = bt_ld16(ctx + 0x46);
    ctrl_compl->header_bytes = ctx[0x52] * 4u;
    ctrl_compl->footer_bytes = ctx[0x53] * 4u;
    ctrl_compl->head = 0;
    ctrl_compl->enabled = ctrl_compl->n_entries != 0;

    ctrl_xfer = &s->xfer[BT_XFER_RING_CONTROL];
    ctrl_xfer->iova = bt_ld64(ctx + 0x3C);
    ctrl_xfer->n_entries = bt_ld16(ctx + 0x44);
    ctrl_xfer->doorbell = bt_ld16(ctx + 0x48);
    ctrl_xfer->header_bytes = ctx[0x50] * 4u;
    ctrl_xfer->footer_bytes = ctx[0x51] * 4u;
    ctrl_xfer->compl_ring = 0;
    ctrl_xfer->tail = 0;
    ctrl_xfer->virt = false;
    ctrl_xfer->generation++;
    ctrl_xfer->enabled = ctrl_xfer->n_entries != 0;

    BT_DPRINTF("context v%u at 0x%" PRIx64 ": %u transfer / %u completion "
               "rings; control transfer ring %u entries at 0x%" PRIx64
               " (doorbell %u, footer %u), control completion ring %u entries "
               "at 0x%" PRIx64 "\n",
               s->ctx_version, addr, s->n_xfer_rings, s->n_compl_rings,
               ctrl_xfer->n_entries, ctrl_xfer->iova, ctrl_xfer->doorbell,
               ctrl_xfer->footer_bytes, ctrl_compl->n_entries,
               ctrl_compl->iova);

    /*
     * The part writes 0x20 bytes of (undocumented) status here. The host only
     * needs the buffer to have been written to.
     */
    if (s->peripheral_info_addr != 0) {
        memset(peripheral_info, 0, sizeof(peripheral_info));
        bt_st32(peripheral_info, s->ctx_version);
        bt_dma_write(s, s->peripheral_info_addr, peripheral_info,
                     sizeof(peripheral_info));
    }

    s->ctx_valid = true;
    return true;
}

static void bt_boot_timer(void *opaque)
{
    AppleBCMBTDeviceState *s = opaque;

    s->bootstage = BT_BOOTSTAGE_READY;
    BT_DPRINTF("firmware booted, bootstage %u\n", s->bootstage);
    bt_raise_msi(s);
}

static void bt_rti_timer(void *opaque)
{
    AppleBCMBTDeviceState *s = opaque;

    if (s->rti_target == BT_RTI_STATE_RUNNING && !bt_load_context(s)) {
        /*
         * Leave the state where it is: the host times out and reports which
         * transition failed, which is more useful than pretending it worked.
         */
        return;
    }
    s->rti_status = s->rti_target;
    BT_DPRINTF("RTI state %u\n", s->rti_status);
    bt_raise_msi(s);
}

static void bt_firmware_stop(AppleBCMBTDeviceState *s)
{
    unsigned i;

    timer_del(s->boot_timer);
    timer_del(s->rti_timer);

    s->bootstage = BT_BOOTSTAGE_COLD;
    s->rti_status = BT_RTI_STATE_OFF;
    s->rti_target = BT_RTI_STATE_OFF;
    s->ctx_valid = false;
    s->ctx_lo = s->ctx_hi = 0;
    s->fw_lo = s->fw_hi = s->fw_size = 0;
    s->compl_heads_addr = 0;
    s->compl_tails_addr = 0;
    s->xfer_heads_addr = 0;
    s->xfer_tails_addr = 0;
    s->peripheral_info_addr = 0;
    s->n_compl_rings = 0;
    s->n_xfer_rings = 0;
    s->evq_head = s->evq_tail = 0;

    for (i = 0; i < BT_MAX_XFER_RINGS; i++) {
        s->xfer[i].enabled = false;
    }
    for (i = 0; i < BT_MAX_COMPL_RINGS; i++) {
        s->compl_ring[i].enabled = false;
    }
}

/* ------------------------------------------------------------------ */
/* BAR0                                                               */
/* ------------------------------------------------------------------ */

static uint64_t bt_bar0_read(void *opaque, hwaddr addr, unsigned size)
{
    AppleBCMBTDeviceState *s = opaque;
    uint64_t value = 0;

    switch (addr) {
    case BT_BAR0_FW_DOORBELL:
        value = 0;
        break;
    case BT_BAR0_RTI_CONTROL:
        value = s->rti_status;
        break;
    case BT_BAR0_SLEEP_CONTROL:
        value = s->sleep_control;
        break;
    case BT_BAR0_HOST_WINDOW_LO:
        value = s->host_window_lo;
        break;
    case BT_BAR0_HOST_WINDOW_HI:
        value = s->host_window_hi;
        break;
    case BT_BAR0_HOST_WINDOW_SIZE:
        value = s->host_window_size;
        break;
    case BT_BAR0_SLEEP_STATUS:
        value = s->sleep_status;
        break;
    case BT_BAR0_CC_CHIPID:
        value = BT_CC_CHIPID_VALUE;
        break;
    case BT_BAR0_CC_CHIPSTATUS:
        value = s->cc_chipstatus;
        break;
    case BT_BAR0_AXI2AHB_STATUS:
        /* Zero means "no bridge error"; see BT_BAR0_AXI2AHB_STATUS. */
        value = 0;
        break;
    default:
        if (addr >= BT_BAR0_DOORBELL_ARRAY &&
            addr < BT_BAR0_DOORBELL_ARRAY +
                       BT_BAR0_DOORBELL_ARRAY_COUNT * 4) {
            value = 0;
            break;
        }
        if (addr >= BT_BAR0_OTP_OFFSET &&
            addr + size <= BT_BAR0_OTP_OFFSET + BT_BAR0_OTP_SIZE) {
            memcpy(&value, s->otp + (addr - BT_BAR0_OTP_OFFSET), size);
            break;
        }
        qemu_log_mask(LOG_UNIMP,
                      "apple-bcm-bt: unimplemented BAR0 read at 0x%" HWADDR_PRIx
                      " (%u bytes)\n",
                      addr, size);
        break;
    }

    BT_TRACE("BAR0 read  0x%04" HWADDR_PRIx "/%u -> 0x%08" PRIx64 "\n", addr,
             size, value);
    return value;
}

static void bt_bar0_write(void *opaque, hwaddr addr, uint64_t data,
                          unsigned size)
{
    AppleBCMBTDeviceState *s = opaque;

    BT_TRACE("BAR0 write 0x%04" HWADDR_PRIx "/%u <- 0x%08" PRIx64 "\n", addr,
             size, data);

    switch (addr) {
    case BT_BAR0_FW_DOORBELL:
        /*
         * The firmware image has been placed at the address in the BAR2
         * FW_LO/HI/SIZE registers and is now ours to fetch. There is no
         * firmware to run here, so the only observable effect is the bootstage
         * transition the host is waiting for.
         */
        BT_DPRINTF("firmware download rung: %u bytes at 0x%08x%08x\n",
                   s->fw_size, s->fw_hi, s->fw_lo);
        if (s->bootstage != BT_BOOTSTAGE_COLD) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "apple-bcm-bt: firmware doorbell at bootstage %u\n",
                          s->bootstage);
        }
        timer_mod(s->boot_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                                     BT_BOOT_DELAY_MS);
        break;
    case BT_BAR0_RTI_CONTROL:
        if (data == BT_RTI_STATE_STARTED) {
            /*
             * A fresh bring-up. Every ring the previous one created is gone
             * along with the host memory behind it, so nothing may survive
             * into the new context -- otherwise a stale ring is serviced
             * against a mapping the guest has already torn down.
             */
            unsigned i;

            s->ctx_valid = false;
            s->compl_heads_addr = 0;
            s->compl_tails_addr = 0;
            s->xfer_heads_addr = 0;
            s->xfer_tails_addr = 0;
            s->evq_head = s->evq_tail = 0;
            for (i = 0; i < BT_MAX_XFER_RINGS; i++) {
                s->xfer[i].enabled = false;
            }
            for (i = 0; i < BT_MAX_COMPL_RINGS; i++) {
                s->compl_ring[i].enabled = false;
            }
            s->rti_status = BT_RTI_STATE_OFF;
        }
        if (data == BT_RTI_STATE_STARTED || data == BT_RTI_STATE_RUNNING) {
            s->rti_target = data;
            timer_mod(s->rti_timer, qemu_clock_get_ms(QEMU_CLOCK_VIRTUAL) +
                                        BT_RTI_DELAY_MS);
        } else if (data == 0) {
            bt_firmware_stop(s);
        } else {
            qemu_log_mask(LOG_UNIMP,
                          "apple-bcm-bt: unknown RTI control value %" PRIu64
                          "\n",
                          data);
        }
        break;
    case BT_BAR0_SLEEP_CONTROL:
        s->sleep_control = data;
        break;
    case BT_BAR0_DOORBELL:
        if (data & BT_DOORBELL_RING) {
            bt_doorbell(s, BT_DOORBELL_IDX(data));
        }
        break;
    case BT_BAR0_HOST_WINDOW_LO:
        s->host_window_lo = data;
        break;
    case BT_BAR0_HOST_WINDOW_HI:
        s->host_window_hi = data;
        break;
    case BT_BAR0_HOST_WINDOW_SIZE:
        s->host_window_size = data;
        break;
    case BT_BAR0_SLEEP_STATUS:
        s->sleep_status = data;
        break;
    default:
        if (addr >= BT_BAR0_DOORBELL_ARRAY &&
            addr < BT_BAR0_DOORBELL_ARRAY +
                       BT_BAR0_DOORBELL_ARRAY_COUNT * 4) {
            bt_doorbell(s, (addr - BT_BAR0_DOORBELL_ARRAY) / 4);
            break;
        }
        qemu_log_mask(LOG_UNIMP,
                      "apple-bcm-bt: unimplemented BAR0 write at "
                      "0x%" HWADDR_PRIx " (0x%" PRIx64 ", %u bytes)\n",
                      addr, data, size);
        break;
    }
}

static const MemoryRegionOps bt_bar0_ops = {
    .read = bt_bar0_read,
    .write = bt_bar0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/* BAR2                                                               */
/* ------------------------------------------------------------------ */

/*
 * Fold Apple's 0x4004xx view of the BAR2 registers onto the Linux driver's
 * 0x2004xx one; see BT_BAR2_ALIAS_MASK.
 */
static hwaddr bt_bar2_canonical(hwaddr addr)
{
    if (addr >= BT_BAR2_ALIAS_BASE) {
        return BT_BAR2_ALIAS_BASE + (addr & BT_BAR2_ALIAS_MASK);
    }
    return addr;
}

static uint64_t bt_bar2_read(void *opaque, hwaddr addr, unsigned size)
{
    AppleBCMBTDeviceState *s = opaque;
    uint64_t value = 0;
    hwaddr raw = addr;

    addr = bt_bar2_canonical(addr);

    switch (addr) {
    case BT_BAR2_BOOTSTAGE:
        value = s->bootstage;
        break;
    case BT_BAR2_RTI_STATUS:
        value = s->rti_status;
        break;
    case BT_BAR2_FW_LO:
        value = s->fw_lo;
        break;
    case BT_BAR2_FW_HI:
        value = s->fw_hi;
        break;
    case BT_BAR2_FW_SIZE:
        value = s->fw_size;
        break;
    case BT_BAR2_CONTEXT_ADDR_LO:
        value = s->ctx_lo;
        break;
    case BT_BAR2_CONTEXT_ADDR_HI:
        /* Read side: the capability mask, not the address written here. */
        value = BT_RTI_CAPABILITIES;
        break;
    case BT_BAR2_CONTEXT_ADDR_HI_ALT:
        value = s->ctx_hi;
        break;
    case BT_BAR2_RTI_WINDOW_LO:
        value = s->rti_window_lo;
        break;
    case BT_BAR2_RTI_WINDOW_HI:
        value = s->rti_window_hi;
        break;
    case BT_BAR2_RTI_WINDOW_SIZE:
        value = s->rti_window_size;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "apple-bcm-bt: unimplemented BAR2 read at 0x%" HWADDR_PRIx
                      " (%u bytes)\n",
                      addr, size);
        break;
    }

    BT_TRACE("BAR2 read  0x%06" HWADDR_PRIx "/%u -> 0x%08" PRIx64 "\n", raw,
             size, value);
    return value;
}

static void bt_bar2_write(void *opaque, hwaddr addr, uint64_t data,
                          unsigned size)
{
    AppleBCMBTDeviceState *s = opaque;

    BT_TRACE("BAR2 write 0x%06" HWADDR_PRIx "/%u <- 0x%08" PRIx64 "\n", addr,
             size, data);

    addr = bt_bar2_canonical(addr);

    switch (addr) {
    case BT_BAR2_FW_LO:
        s->fw_lo = data;
        break;
    case BT_BAR2_FW_HI:
        s->fw_hi = data;
        break;
    case BT_BAR2_FW_SIZE:
        s->fw_size = data;
        break;
    case BT_BAR2_CONTEXT_ADDR_LO:
        s->ctx_lo = data;
        break;
    case BT_BAR2_CONTEXT_ADDR_HI:
    case BT_BAR2_CONTEXT_ADDR_HI_ALT:
        s->ctx_hi = data;
        break;
    case BT_BAR2_RTI_WINDOW_LO:
        s->rti_window_lo = data;
        break;
    case BT_BAR2_RTI_WINDOW_HI:
        s->rti_window_hi = data;
        break;
    case BT_BAR2_RTI_WINDOW_SIZE:
        s->rti_window_size = data;
        break;
    case BT_BAR2_BOOTSTAGE:
    case BT_BAR2_RTI_STATUS:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "apple-bcm-bt: write to read-only BAR2 status register "
                      "0x%" HWADDR_PRIx "\n",
                      addr);
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "apple-bcm-bt: unimplemented BAR2 write at "
                      "0x%" HWADDR_PRIx " (0x%" PRIx64 ", %u bytes)\n",
                      addr, data, size);
        break;
    }
}

static const MemoryRegionOps bt_bar2_ops = {
    .read = bt_bar2_read,
    .write = bt_bar2_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .valid.min_access_size = 1,
    .valid.max_access_size = 4,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/* qdev / PCI                                                         */
/* ------------------------------------------------------------------ */

static void apple_bcm_bt_device_pci_realize(PCIDevice *dev, Error **errp)
{
    AppleBCMBTDeviceState *s = APPLE_BCM_BT_DEVICE(dev);
    uint8_t *pci_conf = dev->config;

    pci_conf[PCI_INTERRUPT_PIN] = 1;
    pci_set_word(pci_conf + PCI_SUBSYSTEM_VENDOR_ID,
                 APPLE_BCM_BT_PCI_VENDOR_ID);
    pci_set_word(pci_conf + PCI_SUBSYSTEM_ID, APPLE_BCM_BT_PCI_DEVICE_ID);

    memory_region_init_io(&s->bar0, OBJECT(dev), &bt_bar0_ops, s,
                          TYPE_APPLE_BCM_BT_DEVICE ".bar0",
                          APPLE_BCM_BT_BAR0_SIZE);
    memory_region_init_io(&s->bar2, OBJECT(dev), &bt_bar2_ops, s,
                          TYPE_APPLE_BCM_BT_DEVICE ".bar2",
                          APPLE_BCM_BT_BAR2_SIZE);

    /*
     * Same capability layout as the Wi-Fi function: the Broadcom-proprietary
     * config registers occupy 0x70-0xAB, so the Express capability goes at
     * 0xD0.
     */
    pcie_endpoint_cap_init(dev, 0xD0);
    pcie_cap_deverr_init(dev);
    pcie_cap_flr_init(dev);
    msi_init(dev, 0x50, 1, true, false, &error_fatal);
    pci_pm_init(dev, 0x40, &error_fatal);
    pcie_aer_init(dev, 1, 0x100, PCI_ERR_SIZEOF, &error_fatal);

    if (s->port != NULL && s->port->maximum_link_speed == 2) {
        pcie_cap_fill_link_ep_usp(dev, QEMU_PCI_EXP_LNK_X1,
                                  QEMU_PCI_EXP_LNK_8GT);
    } else if (s->port != NULL && s->port->maximum_link_speed == 1) {
        pcie_cap_fill_link_ep_usp(dev, QEMU_PCI_EXP_LNK_X2,
                                  QEMU_PCI_EXP_LNK_5GT);
    }

    /* The backplane window registers, as plain read-back scratch storage. */
    pci_set_long(dev->wmask + 0x70, 0xFFFFFFFF);
    pci_set_long(dev->wmask + 0x74, 0xFFFFFFFF);
    pci_set_long(dev->wmask + 0x78, 0xFFFFFFFF);
    pci_set_long(dev->wmask + 0x80, 0xFFFFFFFF);
    pci_set_long(dev->wmask + 0x84, 0xFFFFFFFF);
    pci_set_long(dev->wmask + 0x88, 0xFFFFFFFF);

    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar2);

    s->boot_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, bt_boot_timer, s);
    s->rti_timer = timer_new_ms(QEMU_CLOCK_VIRTUAL, bt_rti_timer, s);
}

static void apple_bcm_bt_device_pci_uninit(PCIDevice *dev)
{
    AppleBCMBTDeviceState *s = APPLE_BCM_BT_DEVICE(dev);

    timer_free(s->boot_timer);
    timer_free(s->rti_timer);
    pcie_aer_exit(dev);
    pcie_cap_exit(dev);
    msi_uninit(dev);
}

static void apple_bcm_bt_device_qdev_reset_hold(Object *obj, ResetType type)
{
    AppleBCMBTDeviceState *s = APPLE_BCM_BT_DEVICE(obj);
    PCIDevice *dev = PCI_DEVICE(obj);

    pci_set_word(dev->config + PCI_COMMAND,
                 PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

    bt_firmware_stop(s);

    s->sleep_control = 0;
    s->sleep_status = 0;
    /*
     * The Bluetooth clock is running as soon as the part is powered: the
     * platform turns it on through the combo chip's power control, which is
     * modelled by the Wi-Fi half's SMC "gP11" key, not by anything the
     * Bluetooth function itself can see.
     */
    s->cc_chipstatus = BT_CC_CHIPSTATUS_BT_CLOCK_RUNNING;
    s->host_window_lo = 0;
    s->host_window_hi = 0;
    s->host_window_size = 0;
    s->rti_window_lo = 0;
    s->rti_window_hi = 0;
    s->rti_window_size = 0;
}

static void apple_bcm_bt_device_class_init(ObjectClass *class, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *c = PCI_DEVICE_CLASS(class);
    ResettableClass *rc = RESETTABLE_CLASS(class);

    c->realize = apple_bcm_bt_device_pci_realize;
    c->exit = apple_bcm_bt_device_pci_uninit;
    c->vendor_id = APPLE_BCM_BT_PCI_VENDOR_ID;
    c->device_id = APPLE_BCM_BT_PCI_DEVICE_ID;
    c->revision = APPLE_BCM_BT_PCI_REVISION;
    c->class_id = APPLE_BCM_BT_PCI_CLASS;

    rc->phases.hold = apple_bcm_bt_device_qdev_reset_hold;

    dc->desc = "Apple Broadcom BCM4378 Bluetooth Device";
    dc->user_creatable = false;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
    dc->hotpluggable = false;
}

static void apple_bcm_bt_realize(DeviceState *dev, Error **errp)
{
    AppleBCMBTState *s = APPLE_BCM_BT(dev);

    qdev_realize(DEVICE(s->device), BUS(s->pci_bus), &error_fatal);
}

static const VMStateDescription vmstate_apple_bcm_bt = {
    .name = "apple_bcm_bt",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields =
        (const VMStateField[]){
            VMSTATE_END_OF_LIST(),
        }
};

static void apple_bcm_bt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = apple_bcm_bt_realize;
    dc->desc = "Apple Broadcom BCM4378 Bluetooth";
    dc->vmsd = &vmstate_apple_bcm_bt;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

SysBusDevice *apple_bcm_bt_create(AppleDTNode *node, AppleDTNode *bt_node,
                                  PCIBus *pci_bus, ApplePCIEPort *port)
{
    DeviceState *dev;
    AppleBCMBTState *s;
    PCIDevice *pci_dev;
    AppleDTProp *prop;

    dev = qdev_new(TYPE_APPLE_BCM_BT);
    s = APPLE_BCM_BT(dev);

    s->pci_bus = pci_bus;
    /*
     * Function 1 of the same device as Wi-Fi. The combo part presents both
     * halves on one PCIe link; iOS' device tree spells this out with the
     * endpoints' "AAPL,unit-string" ("00000000" for wlan, "00000001" here).
     */
    pci_dev = pci_new_multifunction(PCI_DEVFN(0, 1), TYPE_APPLE_BCM_BT_DEVICE);
    s->device = APPLE_BCM_BT_DEVICE(pci_dev);
    s->device->root = s;
    s->device->port = port;
    s->device->dma_mr = port->dma_mr;
    s->device->dma_as = &port->dma_as;

    /*
     * The BD_ADDR comes from the platform node, the same place iOS reads it
     * from, so the address the host stack believes in is the one the modelled
     * controller reports.
     */
    memset(s->device->bdaddr, 0, sizeof(s->device->bdaddr));
    prop = bt_node != NULL ? apple_dt_get_prop(bt_node, "local-mac-address")
                           : NULL;
    if (prop != NULL && prop->len >= sizeof(s->device->bdaddr)) {
        memcpy(s->device->bdaddr, prop->data, sizeof(s->device->bdaddr));
    }
    if (buffer_is_zero(s->device->bdaddr, sizeof(s->device->bdaddr))) {
        /* Locally administered, so it cannot collide with a real part. */
        static const uint8_t fallback[6] = { 0x02, 0x00, 0x5E, 0x10, 0x00, 0x01 };
        memcpy(s->device->bdaddr, fallback, sizeof(fallback));
    }

    object_property_add_child(OBJECT(s), "device", OBJECT(s->device));

    return SYS_BUS_DEVICE(dev);
}

static const TypeInfo apple_bcm_bt_types[] = {
    {
        .name = TYPE_APPLE_BCM_BT_DEVICE,
        .parent = TYPE_PCI_DEVICE,
        .instance_size = sizeof(AppleBCMBTDeviceState),
        .class_init = apple_bcm_bt_device_class_init,
        .interfaces = (InterfaceInfo[]){ { INTERFACE_PCIE_DEVICE }, {} },
    },
    {
        .name = TYPE_APPLE_BCM_BT,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AppleBCMBTState),
        .class_init = apple_bcm_bt_class_init,
    },
};

DEFINE_TYPES(apple_bcm_bt_types)
