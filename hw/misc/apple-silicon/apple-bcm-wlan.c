/*
 * Apple iPhone 11 Broadcom BCM4378 Wi-Fi (PCIe endpoint)
 *
 * Copyright (c) 2025-2026 Christian Inci (chris-pcguy).
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
 * PHASE 1 status: this is a *stub* endpoint. It presents plausible PCI config
 * space and a backplane/chipcommon window that reports a BCM4378 so that iOS
 * 14's AppleBCMWLANBusInterfacePCIe driver can probe, match and begin chip
 * recognition. The msgbuf / firmware-download / ring protocol is NOT
 * implemented yet (Phase 2). Every unhandled access is logged so the exact
 * host access pattern can be observed and implemented incrementally.
 */

#include "qemu/osdep.h"
#include "hw/arm/apple-silicon/dt.h"
#include "hw/irq.h"
#include "hw/misc/apple-silicon/apple-bcm-wlan.h"
#include "hw/pci/msi.h"
#include "hw/pci/pci_device.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "qemu/cutils.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "system/dma.h"
#include "system/runstate.h"

#define TYPE_APPLE_BCM_WLAN_DEVICE "apple-bcm-wlan-device"
OBJECT_DECLARE_SIMPLE_TYPE(AppleBCMWLANDeviceState, APPLE_BCM_WLAN_DEVICE)

#define TYPE_APPLE_BCM_WLAN "apple-bcm-wlan"
OBJECT_DECLARE_SIMPLE_TYPE(AppleBCMWLANState, APPLE_BCM_WLAN)

/*
 * PCI identity.
 *
 * vendor 0x14E4 = Broadcom, device 0x4425 = BCM4378 (the real PCI device id
 * used by Apple's iPhone 11 combo chip). The backplane chip-id reported through
 * the register window below is a *different* number (0x4378) living in the
 * SiliconBackplane ChipCommon namespace -- both are correct.
 *
 * TODO(phase-verify): confirm the real subsystem vendor/id against a live
 * iPhone 11 ioreg dump (`ioreg -l -w0` under the AppleBCMWLANBusInterfacePCIe /
 * pci* nub). Broadcom parts usually carry a subsystem id; Apple's may be
 * 0x106B (Apple) as the subsystem vendor. Left as Broadcom/0x4425 placeholder.
 */
#define APPLE_BCM_WLAN_PCI_VENDOR_ID 0x14E4 // Broadcom
#define APPLE_BCM_WLAN_PCI_DEVICE_ID 0x4425 // BCM4378
#define APPLE_BCM_WLAN_PCI_REVISION 0x03
// PCI class 0x02 (network controller) / subclass 0x80 (other) == 0x028000.
// Alternative accurate value would be 0x0d80 (wireless controller / other).
#define APPLE_BCM_WLAN_PCI_CLASS 0x0280
#define APPLE_BCM_WLAN_PCI_SUBSYS_VENDOR_ID 0x14E4 // TODO: confirm (Apple 0x106B?)
#define APPLE_BCM_WLAN_PCI_SUBSYS_ID 0x4425 // TODO: confirm from ioreg

/*
 * BAR layout. brcmfmac's pcie.c and the reverse-engineered AppleBCMWLAN driver
 * both expect a small register window and a large TCM/shared-RAM window.
 *
 * BAR0 is 32 KiB: the driver's backplane table (kBCOM4378ChipBackplaneWindows)
 * describes eight 4 KiB windows at BAR0 offsets 0x0000-0x7000, each steered by
 * a PCI config register (0x80, 0x70, 0x74, 0x78) or pinned to a fixed core.
 *
 * BAR2 maps the chip's memories identity-style. The highest region the driver
 * uses is 0x400000 + 0x120000, and it polls for the shared-info pointer at
 * dongle offset ramsize-4 = 0x51FFFC, so the window must cover at least
 * 0x520000; 8 MiB gives headroom.
 */
#define APPLE_BCM_WLAN_DEVICE_BAR0_SIZE (0x8000)
#define APPLE_BCM_WLAN_DEVICE_BAR2_SIZE (8 * MiB)

/*
 * SiliconBackplane / ChipCommon.
 *
 * The host slides a window over the backplane by writing the target backplane
 * base into BAR0_WINDOW (0x80). It then accesses the selected core through the
 * low part of BAR0. ChipCommon on AXI-backplane BCM43xx parts enumerates at
 * 0x18000000; its first register (offset 0) is CHIPID.
 *
 * CHIPID encoding (bcma/ChipCommon):
 *   [15:0]  chip id       -> 0x4378
 *   [19:16] chip rev      -> 0x3   (plausible BCM4378 A0-ish rev; TODO verify)
 *   [23:20] package       -> 0x0
 *   [27:24] num cores     -> 0x0   (not used for recognition here)
 *   [31:28] SoC interconnect type
 *
 * TODO(phase-verify): confirm rev/package/socitype encoding the AppleBCMWLAN
 * chip-recognition path actually checks; the driver may read the EROM to count
 * cores, which this stub does not model yet.
 */
#define BCM_BACKPLANE_CHIPCOMMON_BASE 0x18000000ULL
#define BCM_CHIPCOMMON_CHIPID_OFFSET 0x0
#define BCM4378_CHIP_ID 0x4378
#define BCM4378_CHIP_REV 0x3
#define BCM4378_CHIP_PACKAGE 0x0
#define BCM4378_CHIPID_VALUE                                    \
    ((BCM4378_CHIP_ID & 0xFFFF) | ((BCM4378_CHIP_REV & 0xF) << 16) | \
     ((BCM4378_CHIP_PACKAGE & 0xF) << 20))

/* BAR0 register offsets (brcmfmac PCIe core register names). */
#define BCM_PCIE_REG_INTMASK 0x24 // BRCMF_PCIE_PCIE2REG_INTMASK
#define BCM_PCIE_REG_MAILBOXINT 0x48 // BRCMF_PCIE_PCIE2REG_MAILBOXINT
#define BCM_PCIE_REG_MAILBOXMASK 0x4C // BRCMF_PCIE_PCIE2REG_MAILBOXMASK
#define BCM_PCIE_REG_CONFIGADDR 0x120
#define BCM_PCIE_REG_CONFIGDATA 0x124
#define BCM_PCIE_REG_H2D_MAILBOX_0 0x140 // BRCMF_PCIE_PCIE2REG_H2D_MAILBOX_0
#define BCM_PCIE_REG_H2D_MAILBOX_1 0x144 // BRCMF_PCIE_PCIE2REG_H2D_MAILBOX_1
#define BCM_PCIE_REG_D2H_MAILBOX_0 0x148
#define BCM_PCIE_REG_D2H_MAILBOX_1 0x14C
#define BCM_PCIE_BAR0_WINDOW 0x80 // BRCMF_PCIE_BAR0_WINDOW
#define BCM_PCIE_BAR0_CORE2_WINDOW 0x70

/*
 * Everything below the first control register is treated as the sliding
 * backplane window: after the host programs BAR0_WINDOW, reads in [0, 0x48)
 * are serviced as backplane[window_base + offset].
 */
#define BCM_PCIE_BAR0_WINDOW_REGION_END 0x48

struct AppleBCMWLANDeviceState {
    PCIDevice parent_obj;
    AppleBCMWLANState *root;

    MemoryRegion container;
    MemoryRegion bar0, bar2;
    MemoryRegion bar0_alias, bar2_alias;

    ApplePCIEPort *port;
    MemoryRegion *dma_mr;
    AddressSpace *dma_as;

    /* Host-programmed sliding backplane window base (BAR0_WINDOW @ 0x80). */
    uint32_t backplane_window;
    uint32_t backplane_window2; // BAR0_CORE2_WINDOW @ 0x70

    /* Mailbox / interrupt shadow registers (Phase-1 no-ops). */
    uint32_t intmask;
    uint32_t intstatus;
    uint32_t mailboxint;
    uint32_t mailboxmask;
    uint32_t h2d_mailbox_0;
    uint32_t h2d_mailbox_1;
    uint32_t d2h_mailbox_0;
    uint32_t d2h_mailbox_1;
    uint32_t configaddr;
};

struct AppleBCMWLANState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    AppleBCMWLANDeviceState *device;

    PCIBus *pci_bus;
};

/* Raise the endpoint's interrupt (MSI if enabled, else legacy INTx). */
/* Phase-1: plumbing kept for Phase 2 (msgbuf completion IRQs). */
static G_GNUC_UNUSED void apple_bcm_wlan_set_irq(void *opaque, int irq_num,
                                                 int level)
{
    AppleBCMWLANState *s = opaque;
    PCIDevice *pci_dev = PCI_DEVICE(s->device);

    if (msi_enabled(pci_dev)) {
        if (level) {
            msi_notify(pci_dev, 0);
        }
    } else {
        pci_set_irq(pci_dev, level);
    }
}

/* DMA helpers over the port's DART address space (Phase-2 msgbuf rings). */
static G_GNUC_UNUSED bool apple_bcm_wlan_dma_read(AppleBCMWLANDeviceState *s,
                                                  uint64_t offset,
                                                  uint64_t size, uint8_t *buf)
{
    if (dma_memory_read(s->dma_as, offset, buf, size, MEMTXATTRS_UNSPECIFIED) !=
        MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Failed to read from DMA.\n",
                      __func__);
        return false;
    }
    return true;
}

static G_GNUC_UNUSED bool apple_bcm_wlan_dma_write(AppleBCMWLANDeviceState *s,
                                                   uint64_t offset,
                                                   uint64_t size, uint8_t *buf)
{
    if (dma_memory_write(s->dma_as, offset, buf, size,
                         MEMTXATTRS_UNSPECIFIED) != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Failed to write to DMA.\n",
                      __func__);
        return false;
    }
    return true;
}

/*
 * Resolve a read that falls inside the currently-selected backplane window.
 * Returns true and fills *out if it is a register we model (currently only
 * ChipCommon CHIPID); otherwise returns false so the caller logs it.
 */
static bool apple_bcm_wlan_backplane_read(AppleBCMWLANDeviceState *s,
                                          hwaddr addr, uint64_t *out)
{
    uint64_t backplane_addr = (uint64_t)s->backplane_window + addr;

    if (backplane_addr ==
        (BCM_BACKPLANE_CHIPCOMMON_BASE + BCM_CHIPCOMMON_CHIPID_OFFSET)) {
        *out = BCM4378_CHIPID_VALUE;
        qemu_log_mask(LOG_UNIMP,
                      "%s: backplane ChipCommon CHIPID read -> 0x%08x "
                      "(window_base=0x%08x off=0x" HWADDR_FMT_plx ")\n",
                      __func__, BCM4378_CHIPID_VALUE, s->backplane_window, addr);
        return true;
    }

    return false;
}

static uint64_t apple_bcm_wlan_device_bar0_read(void *opaque, hwaddr addr,
                                                unsigned size)
{
    AppleBCMWLANDeviceState *s = opaque;
    uint64_t val = 0;

    switch (addr) {
    case BCM_PCIE_REG_INTMASK:
        val = s->intmask;
        break;
    case BCM_PCIE_REG_MAILBOXINT:
        val = s->mailboxint;
        break;
    case BCM_PCIE_REG_MAILBOXMASK:
        val = s->mailboxmask;
        break;
    case BCM_PCIE_BAR0_CORE2_WINDOW:
        val = s->backplane_window2;
        break;
    case BCM_PCIE_BAR0_WINDOW:
        val = s->backplane_window;
        break;
    case BCM_PCIE_REG_CONFIGADDR:
        val = s->configaddr;
        break;
    case BCM_PCIE_REG_H2D_MAILBOX_0:
        val = s->h2d_mailbox_0;
        break;
    case BCM_PCIE_REG_H2D_MAILBOX_1:
        val = s->h2d_mailbox_1;
        break;
    case BCM_PCIE_REG_D2H_MAILBOX_0:
        val = s->d2h_mailbox_0;
        break;
    case BCM_PCIE_REG_D2H_MAILBOX_1:
        val = s->d2h_mailbox_1;
        break;
    default:
        /* Anything in the low region is the sliding backplane window. */
        if (addr < BCM_PCIE_BAR0_WINDOW_REGION_END &&
            apple_bcm_wlan_backplane_read(s, addr, &val)) {
            break;
        }
        qemu_log_mask(LOG_UNIMP,
                      "%s: UNIMP READ @ 0x" HWADDR_FMT_plx
                      " size %u (window_base=0x%08x, in_window=%d)\n",
                      __func__, addr, size, s->backplane_window,
                      addr < BCM_PCIE_BAR0_WINDOW_REGION_END);
        break;
    }

    qemu_log_mask(LOG_UNIMP,
                  "%s: READ @ 0x" HWADDR_FMT_plx " value: 0x%" PRIx64
                  " size %u\n",
                  __func__, addr, val, size);
    return val;
}

static void apple_bcm_wlan_device_bar0_write(void *opaque, hwaddr addr,
                                             uint64_t data, unsigned size)
{
    AppleBCMWLANDeviceState *s = opaque;

    qemu_log_mask(LOG_UNIMP,
                  "%s: WRITE @ 0x" HWADDR_FMT_plx " value: 0x%" PRIx64
                  " size %u\n",
                  __func__, addr, data, size);

    switch (addr) {
    case BCM_PCIE_BAR0_WINDOW:
        s->backplane_window = (uint32_t)data;
        qemu_log_mask(LOG_UNIMP, "%s: BAR0_WINDOW slid to 0x%08x\n", __func__,
                      s->backplane_window);
        break;
    case BCM_PCIE_BAR0_CORE2_WINDOW:
        s->backplane_window2 = (uint32_t)data;
        break;
    case BCM_PCIE_REG_INTMASK:
        s->intmask = (uint32_t)data;
        break;
    case BCM_PCIE_REG_MAILBOXINT:
        /* Write-1-to-clear on real HW; Phase 1 just records the value. */
        s->mailboxint = (uint32_t)data;
        break;
    case BCM_PCIE_REG_MAILBOXMASK:
        s->mailboxmask = (uint32_t)data;
        break;
    case BCM_PCIE_REG_CONFIGADDR:
        s->configaddr = (uint32_t)data;
        break;
    case BCM_PCIE_REG_H2D_MAILBOX_0:
        /* Host doorbell -> device. Phase 2 will consume the msgbuf rings. */
        s->h2d_mailbox_0 = (uint32_t)data;
        break;
    case BCM_PCIE_REG_H2D_MAILBOX_1:
        s->h2d_mailbox_1 = (uint32_t)data;
        break;
    default:
        qemu_log_mask(LOG_UNIMP,
                      "%s: UNIMP WRITE @ 0x" HWADDR_FMT_plx " value 0x%" PRIx64
                      " (window_base=0x%08x)\n",
                      __func__, addr, data, s->backplane_window);
        break;
    }
}

static const MemoryRegionOps bar0_ops = {
    .read = apple_bcm_wlan_device_bar0_read,
    .write = apple_bcm_wlan_device_bar0_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl =
        {
            .min_access_size = 4,
            .max_access_size = 4,
        },
};

SysBusDevice *apple_bcm_wlan_create(AppleDTNode *node, PCIBus *pci_bus,
                                    ApplePCIEPort *port)
{
    DeviceState *dev;
    AppleBCMWLANState *s;
    SysBusDevice *sbd;
    PCIDevice *pci_dev;

    dev = qdev_new(TYPE_APPLE_BCM_WLAN);
    s = APPLE_BCM_WLAN(dev);
    sbd = SYS_BUS_DEVICE(dev);

    s->pci_bus = pci_bus;
    pci_dev = pci_new(-1, TYPE_APPLE_BCM_WLAN_DEVICE);
    s->device = APPLE_BCM_WLAN_DEVICE(pci_dev);
    s->device->root = s;
    s->device->port = port;
    s->device->dma_mr = port->dma_mr;
    s->device->dma_as = &port->dma_as;

    object_property_add_child(OBJECT(s), "device", OBJECT(s->device));

    return sbd;
}

static void apple_bcm_wlan_device_pci_realize(PCIDevice *dev, Error **errp)
{
    AppleBCMWLANDeviceState *s = APPLE_BCM_WLAN_DEVICE(dev);
    uint8_t *pci_conf = dev->config;

    pci_conf[PCI_INTERRUPT_PIN] = 1;
    // Broadcom Wi-Fi carries a subsystem id (unlike the baseband).
    pci_set_word(pci_conf + PCI_SUBSYSTEM_VENDOR_ID,
                 APPLE_BCM_WLAN_PCI_SUBSYS_VENDOR_ID);
    pci_set_word(pci_conf + PCI_SUBSYSTEM_ID, APPLE_BCM_WLAN_PCI_SUBSYS_ID);

    memory_region_init_io(&s->bar0, OBJECT(dev), &bar0_ops, s,
                          TYPE_APPLE_BCM_WLAN_DEVICE ".bar0",
                          APPLE_BCM_WLAN_DEVICE_BAR0_SIZE);
    /* TCM / shared-RAM window: backed by real RAM so firmware/NVRAM blits from
     * the host don't fault. No firmware behaviour yet (Phase 2). */
    memory_region_init_ram(&s->bar2, OBJECT(dev),
                           TYPE_APPLE_BCM_WLAN_DEVICE ".bar2",
                           APPLE_BCM_WLAN_DEVICE_BAR2_SIZE, &error_fatal);

    assert_true(pci_is_express(dev));
    /*
     * Put the PCI Express capability at 0xD0, where real BCM43xx parts have it.
     * It must not overlap the Broadcom-proprietary config registers: the
     * backplane window registers live at 0x70/0x74/0x78/0x80, SPROM control at
     * 0x88, BAR1 control at 0x8C and the backplane address/data pair at
     * 0xA0/0xA4. A capability placed at 0x70 covers 0x70-0xAB and buries all of
     * them. The driver finds the capability by walking the list, so its offset
     * only has to avoid collisions.
     */
    pcie_endpoint_cap_init(dev, 0xD0);
    pcie_cap_deverr_init(dev);

    /* Single MSI vector, mirroring the baseband endpoint. */
    msi_init(dev, 0x50, 1, true, false, &error_fatal);
    pci_pm_init(dev, 0x40, &error_fatal);

    /*
     * AER: version 1 with the standard cap size -- baseband notes
     * "0x3c for broadcom wifi, version 1" but PCI_ERR_SIZEOF is the safe,
     * spec-correct size and is what the in-tree baseband ships with.
     */
    pcie_aer_init(dev, 1, 0x100, PCI_ERR_SIZEOF, &error_fatal);

    /* T8030 combo chip links at 5GT; mirror the baseband link-cap fill. */
    if (s->port->maximum_link_speed == 2) {
        pcie_cap_fill_link_ep_usp(dev, QEMU_PCI_EXP_LNK_X1,
                                  QEMU_PCI_EXP_LNK_8GT);
    } else if (s->port->maximum_link_speed == 1) {
        pcie_cap_fill_link_ep_usp(dev, QEMU_PCI_EXP_LNK_X2,
                                  QEMU_PCI_EXP_LNK_5GT);
    }

    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar2);

    /*
     * Mirror the baseband BAR-visibility workaround: iOS reshuffles the PCI
     * subregions, so also expose aliases of the BARs through a private
     * container mapped into system memory. Uses a distinct sub-address from
     * the baseband's BASEBAND_BAR_SUB_ADDR (0x40000000) to avoid overlap.
     */
#define WLAN_BAR_SUB_ADDR 0x50000000ULL
    memory_region_init(&s->container, OBJECT(s), "wlan-bar-container",
                       APPLE_BCM_WLAN_DEVICE_BAR0_SIZE +
                           APPLE_BCM_WLAN_DEVICE_BAR2_SIZE);
    memory_region_init_alias(&s->bar0_alias, OBJECT(s), "wlan-bar0-alias",
                             &s->bar0, 0x0, APPLE_BCM_WLAN_DEVICE_BAR0_SIZE);
    memory_region_init_alias(&s->bar2_alias, OBJECT(s), "wlan-bar2-alias",
                             &s->bar2, 0x0, APPLE_BCM_WLAN_DEVICE_BAR2_SIZE);
    memory_region_add_subregion(&s->container, 0x0000, &s->bar2_alias);
    memory_region_add_subregion(&s->container,
                                APPLE_BCM_WLAN_DEVICE_BAR2_SIZE,
                                &s->bar0_alias);
    memory_region_add_subregion(get_system_memory(),
                                APCIE_ROOT_COMMON_ADDRESS + WLAN_BAR_SUB_ADDR,
                                &s->container);
}

static void apple_bcm_wlan_device_qdev_reset_hold(Object *obj, ResetType type)
{
    AppleBCMWLANDeviceState *s = APPLE_BCM_WLAN_DEVICE(obj);
    PCIDevice *dev = PCI_DEVICE(obj);

    pci_set_word(dev->config + PCI_COMMAND,
                 PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

    s->backplane_window = 0;
    s->backplane_window2 = 0;
    s->intmask = 0;
    s->intstatus = 0;
    s->mailboxint = 0;
    s->mailboxmask = 0;
    s->h2d_mailbox_0 = 0;
    s->h2d_mailbox_1 = 0;
    s->d2h_mailbox_0 = 0;
    s->d2h_mailbox_1 = 0;
    s->configaddr = 0;
}

static void apple_bcm_wlan_device_pci_uninit(PCIDevice *dev)
{
    pcie_aer_exit(dev);
    pcie_cap_exit(dev);
    msi_uninit(dev);
}

static void apple_bcm_wlan_device_class_init(ObjectClass *class,
                                             const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(class);
    PCIDeviceClass *c = PCI_DEVICE_CLASS(class);
    ResettableClass *rc = RESETTABLE_CLASS(class);

    c->realize = apple_bcm_wlan_device_pci_realize;
    c->exit = apple_bcm_wlan_device_pci_uninit;
    c->vendor_id = APPLE_BCM_WLAN_PCI_VENDOR_ID;
    c->device_id = APPLE_BCM_WLAN_PCI_DEVICE_ID;
    c->revision = APPLE_BCM_WLAN_PCI_REVISION;
    c->class_id = APPLE_BCM_WLAN_PCI_CLASS;

    rc->phases.hold = apple_bcm_wlan_device_qdev_reset_hold;

    dc->desc = "Apple Broadcom BCM4378 Wi-Fi Device";
    dc->user_creatable = false;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
    dc->hotpluggable = false;
}

static void apple_bcm_wlan_realize(DeviceState *dev, Error **errp)
{
    AppleBCMWLANState *s = APPLE_BCM_WLAN(dev);

    qdev_realize(DEVICE(s->device), BUS(s->pci_bus), &error_fatal);
}

static void apple_bcm_wlan_unrealize(DeviceState *dev)
{
    /* nothing to tear down in Phase 1 */
}

static const VMStateDescription vmstate_apple_bcm_wlan = {
    .name = "apple_bcm_wlan",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields =
        (const VMStateField[]){
            VMSTATE_END_OF_LIST(),
        }
};

static void apple_bcm_wlan_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = apple_bcm_wlan_realize;
    dc->unrealize = apple_bcm_wlan_unrealize;
    dc->desc = "Apple Broadcom BCM4378 Wi-Fi";
    dc->vmsd = &vmstate_apple_bcm_wlan;
    set_bit(DEVICE_CATEGORY_NETWORK, dc->categories);
}

static const TypeInfo apple_bcm_wlan_types[] = {
    {
        .name = TYPE_APPLE_BCM_WLAN_DEVICE,
        .parent = TYPE_PCI_DEVICE,
        .instance_size = sizeof(AppleBCMWLANDeviceState),
        .class_init = apple_bcm_wlan_device_class_init,
        .interfaces = (InterfaceInfo[]){ { INTERFACE_PCIE_DEVICE }, {} },
    },
    {
        .name = TYPE_APPLE_BCM_WLAN,
        .parent = TYPE_SYS_BUS_DEVICE,
        .instance_size = sizeof(AppleBCMWLANState),
        .class_init = apple_bcm_wlan_class_init,
    },
};

DEFINE_TYPES(apple_bcm_wlan_types)
