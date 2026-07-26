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
 * space plus the chip-recognition / backplane access layer -- the four
 * remappable BAR0 window registers in config space, the eight 4 KiB BAR0
 * windows they steer, and enough of ChipCommon, the GCI core, the PCIe2 core
 * and the AI wrappers for iOS 14's AppleBCMWLANBusInterfacePCIe driver to
 * probe, match and get through checkHardware()/prepareHardware(). The msgbuf /
 * firmware-download / ring protocol is NOT implemented yet (Phase 2). Every
 * unhandled access is logged so the exact host access pattern can be observed
 * and implemented incrementally.
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
 * Broadcom-proprietary PCI config registers.
 *
 * BAR0 is carved into eight 4 KiB windows (kBCOM4378ChipBackplaneWindows); four
 * of them are *remappable*, i.e. the host picks which backplane address the
 * window points at by writing that address into one of these config registers.
 * AppleBCMWLANChipBackplane::validateWindow() writes a base and then reads the
 * very same config register back, requiring an exact match, so these must
 * behave as plain 32-bit read/write scratch storage.
 *
 * 0x88 is the SPROM/backplane control register. Bit 6 is a backplane-access
 * enable that the driver sets before touching BAR0 and clears again on
 * detach; we accept and store it but do not gate accesses on it, because
 * nothing in the model needs the gate and mis-modelling the exact sequencing
 * would only be a source of spurious failures.
 */
#define BCM_PCI_CFG_BAR0_WINDOW1 0x70 // remappable wrapper window, BAR0 0x1000
#define BCM_PCI_CFG_BAR0_WINDOW4 0x74 // remappable core window, BAR0 0x4000
#define BCM_PCI_CFG_BAR0_WINDOW5 0x78 // remappable wrapper window, BAR0 0x5000
#define BCM_PCI_CFG_BAR0_WINDOW0 0x80 // remappable core window, BAR0 0x0000
#define BCM_PCI_CFG_SPROM_CONTROL 0x88 // bit 6 == backplane access enable

/*
 * SiliconBackplane layout, transcribed from the driver's hardcoded per-chip
 * tables (kBCOM4378ChipCores @0xfffffff00730d090 and kBCOM4378ChipWrappers
 * @0xfffffff00730d0fc). Apple's driver does NOT walk the EROM, so only the
 * addresses in those tables are ever generated.
 *
 * Cores (all 4 KiB): id0 ChipCommon 0x18000000, id1 D11 MAC 0x18031000,
 * id2 ARM 0x18030000, id3 PCIe2 0x18001000, id6 GCI 0x18010000,
 * id7 PMU 0x18012000. Wrappers (all 4 KiB, AI register layout) live in
 * 0x18100000-0x1813FFFF: id0 CommonMaster 0x18100000, id2 ARMMaster
 * 0x18130000 (the one used to reset the ARM core), id3 PCIeMaster 0x18101000.
 */
#define BCM_BACKPLANE_CHIPCOMMON_BASE 0x18000000ULL // ChipCoreID 0
#define BCM_BACKPLANE_PCIE2_BASE 0x18001000ULL // ChipCoreID 3
#define BCM_BACKPLANE_GCI_BASE 0x18010000ULL // ChipCoreID 6
#define BCM_BACKPLANE_OTP_BASE 0x18011000ULL // ChipCoreID 8
#define BCM_BACKPLANE_CORE_SIZE 0x1000ULL
/* Every AI wrapper shares one register layout, so one handler serves them all. */
#define BCM_BACKPLANE_WRAPPER_BASE 0x18100000ULL
#define BCM_BACKPLANE_WRAPPER_END 0x18140000ULL
#define BCM_BACKPLANE_NUM_WRAPPERS \
    ((BCM_BACKPLANE_WRAPPER_END - BCM_BACKPLANE_WRAPPER_BASE) / 0x1000)

/*
 * ChipCommon registers.
 *
 * CHIPID encoding (bcma/ChipCommon):
 *   [15:0]  chip id       -> 0x4378
 *   [19:16] chip rev      -> 0x3
 *   [23:20] package       -> 0x0
 *   [27:24] num cores / [31:28] SoC interconnect type
 *
 * Note the chip is recognised purely from the *PCI* device id (0x4425 ->
 * chipNumberFromDeviceID -> 0x111A); CHIPID is never actually read by
 * AppleBCMWLAN, so the exact rev/package encoding does not matter.
 */
#define BCM_CHIPCOMMON_CHIPID 0x000
#define BCM_CHIPCOMMON_CAPABILITIES 0x004
#define BCM_CHIPCOMMON_GPIOOUT 0x064
#define BCM_CHIPCOMMON_GPIOOUTEN 0x068
#define BCM_CHIPCOMMON_GPIOCONTROL 0x06C
#define BCM_CHIPCOMMON_GCI_INDIRECT_ADDR 0xC40
#define BCM_CHIPCOMMON_GCI_CHIPCTRL 0xE00

#define BCM4378_CHIP_ID 0x4378
#define BCM4378_CHIP_REV 0x3
#define BCM4378_CHIP_PACKAGE 0x0
#define BCM4378_CHIPID_VALUE                                         \
    ((BCM4378_CHIP_ID & 0xFFFF) | ((BCM4378_CHIP_REV & 0xF) << 16) | \
     ((BCM4378_CHIP_PACKAGE & 0xF) << 20))

/*
 * ChipCommon capabilities (offset 0x04), a.k.a. the SROM escape hatch.
 *
 * Bit 30 (CC_CAP_SROM) is the ONLY bit AppleBCMWLAN looks at, and we report it
 * CLEAR on purpose: readChipProvisioningData() then bails out immediately with
 * "Chip does not support SPROM" instead of running the SROM/OTP read sequence
 * (ChipCommon 0x190/0x194/0x198, 0x400 words) and parsing the result as
 * Broadcom CIS tuples with a valid checksum, SROM version 0x10 and a signature
 * word -- none of which we model. The traced call site treats the failure as
 * non-fatal (it only picks between two constants).
 */
#define BCM_CHIPCOMMON_CAP_SROM (1U << 30)
#define BCM_CHIPCOMMON_CAPABILITIES_VALUE (0x00000000U & ~BCM_CHIPCOMMON_CAP_SROM)

/*
 * PCIe2 core registers (ChipCoreID 3). Apple's mailbox registers are at
 * 0xC30/0xC34, not at brcmfmac's 0x48/0x4C.
 */
#define BCM_PCIE2_CONFIGADDR 0x120
#define BCM_PCIE2_CONFIGDATA 0x124
#define BCM_PCIE2_H2D_DOORBELL_0 0x140
#define BCM_PCIE2_H2D_MAILBOX_DATA 0x144
#define BCM_PCIE2_POWER_CONTROL 0x1E8 // written by forcePowerLite()
#define BCM_PCIE2_MAILBOXINT 0xC30 // device -> host status, write-1-to-clear
#define BCM_PCIE2_MAILBOXMASK 0xC34

/* GCI core registers (ChipCoreID 6). */
#define BCM_GCI_INDEX 0x040
#define BCM_GCI_STATUS 0x204 // bit 6 must read 0, see checkHardware()
#define BCM_GCI_STATUS_FAIL (1U << 6)
#define BCM_GCI_STATUS_VALUE (0x00000000U & ~BCM_GCI_STATUS_FAIL)
#define BCM_GCI_CHIPCTRL 0xE64

/*
 * OTP core (ChipCoreID 8, backplane 0x18011000).
 *
 * The chip's one-time-programmable fuse array is exposed as a plain register
 * window and read by AppleBCMWLANChipBackplane::copyRegisters16() as a run of
 * 16-bit loads. Two regions are copied out (both live in the SAME 0x400-byte
 * image, they only differ in where they start):
 *
 *   kBCOM4378ChipUserOTP = { 0x120, 0x2e0 } -> OSData [busIface+0x470],
 *       published as the "OTP" property AND fed to
 *       AppleBCMWLANBusInterfacePCIe::parseOTP() as a Broadcom CIS tuple
 *       stream. THIS is the one that has to contain something sensible.
 *   kBCOM4378ChipOTP     = { 0x000, 0x400 } -> OSData [busIface+0x480],
 *       published as the "ChipOTP" property only, never parsed.
 *
 * AppleBCMWLANBusInterface::parseOTPData (@0xfffffff0094488e8) walks the
 * stream as {u8 tag, u8 len, u8 data[len]}: tag 0x00 is one byte of padding,
 * tag 0xFF ends the stream, anything else is handed to the parseOTPTuple
 * callback. Truncated tuples are a hard error.
 */
#define BCM_OTP_IMAGE_SIZE 0x400
#define BCM_OTP_CIS_OFFSET 0x120 // == kBCOM4378ChipUserOTP.offset

/* Broadcom/PCMCIA CIS tuple tags. */
#define CIS_TPL_VERS_1 0x15
#define CIS_TPL_MANFID 0x20
#define CIS_TPL_FUNCID 0x21
#define CIS_TPL_END 0xFF

/*
 * Synthetic CIS content. Nothing here is copied from Apple; it is the minimum
 * a BCM4378 OTP has to say for the driver's identification path to work.
 *
 * The tuple that matters is CISTPL_VERS_1 (0x15). Its handler chain is
 * AppleBCMWLANBusInterface::parseOTPTuple (@0xfffffff009448b5c, requires
 * len > 6, else "dropping invalid Version tuple") ->
 * AppleBCMWLANBusInterfacePCIe::parseVersion1Tuple (@0xfffffff0095c4660),
 * which skips the two version bytes and then splits the payload into up to
 * four NUL-terminated strings, storing them at busIface +0xd8 (ProductInfo0),
 * +0xe0 (ProductInfo1), +0xc8 (Manufacturer) and +0xd0 (Product) -- note the
 * PCIe override's slot order differs from the base class'. It stops at a 0xFF
 * byte or when it has consumed len-3 bytes.
 *
 * publishHWIdentifiers (inlined into attachPCIeBusGated) then requires ALL
 * FOUR of those pointers to be non-NULL, otherwise it bails with
 * kIOReturnBadArgument at source lines 4112..4115 -- which is exactly the
 * "publishHWIdentifiers@4112: Bad argument" we used to get with an all-zero
 * OTP. It merges the four strings into one dictionary with
 * AppleBCMWLANUtil::appendParsedKeyValuePairsToDictionary
 * (@0xfffffff00953f264), which parses SPACE-separated "key=value" pairs and
 * FAILS on anything malformed, and publishes it as "HWIdentifiers". The keys
 * it looks up afterwards are the single letters used by Apple's Wi-Fi
 * firmware naming scheme: "C" (chip, filled in by the driver itself from the
 * PCI device id), "P" (product/platform), "M" (module) and "m" (module
 * revision).
 */
static const uint8_t apple_bcm_wlan_otp_cis[] = {
    /* CISTPL_MANFID: manufacturer 0x02D0 (Broadcom), card id 0x4378. */
    CIS_TPL_MANFID, 0x04, 0xD0, 0x02, 0x78, 0x43,
    /* CISTPL_FUNCID: function 0x0C == network adapter, no sysinit. */
    CIS_TPL_FUNCID, 0x02, 0x0C, 0x00,
    /* CISTPL_VERS_1: major 8, minor 0, four strings, 0xFF terminator. */
    CIS_TPL_VERS_1, 0x1B, 0x08, 0x00,
    'P', '=', 'C', '0', '5', '1', '\0', // platform
    'M', '=', 'B', 'C', 'P', 'N', '\0', // module
    'V', '=', 'm', '\0', // vendor
    'm', '=', '1', '.', '0', '\0', // module revision
    CIS_TPL_END,
    /* End of the tuple stream. */
    CIS_TPL_END,
};

/* AI (wrapper) registers, identical for every wrapper. */
#define BCM_AI_IOCTRL 0x408
#define BCM_AI_RESETCTRL 0x800
#define BCM_AI_RESETSTATUS 0x804

/* One BAR0 window == one 4 KiB backplane aperture. */
#define APPLE_BCM_WLAN_WINDOW_SIZE 0x1000
#define APPLE_BCM_WLAN_NUM_WINDOWS \
    (APPLE_BCM_WLAN_DEVICE_BAR0_SIZE / APPLE_BCM_WLAN_WINDOW_SIZE)

struct AppleBCMWLANDeviceState {
    PCIDevice parent_obj;
    AppleBCMWLANState *root;

    MemoryRegion bar0, bar2;

    ApplePCIEPort *port;
    MemoryRegion *dma_mr;
    AddressSpace *dma_as;

    /*
     * ChipCommon shadow registers. The remaining ChipCommon space is left
     * unimplemented on purpose so `-d unimp` keeps reporting what the driver
     * touches next.
     */
    uint32_t cc_gpioout;
    uint32_t cc_gpioouten;
    uint32_t cc_gpiocontrol;
    uint32_t cc_gci_indirect_addr;
    uint32_t cc_gci_chipctrl;

    /* GCI core shadow registers. */
    uint32_t gci_index;
    uint32_t gci_chipctrl;

    /* PCIe2 core shadow registers. */
    uint32_t pcie_configaddr;
    uint32_t pcie_h2d_doorbell_0;
    uint32_t pcie_h2d_mailbox_data;
    uint32_t pcie_power_control;
    uint32_t pcie_mailboxint;
    uint32_t pcie_mailboxmask;

    /* Per-wrapper AI registers, indexed by (backplane_addr >> 12) & 0x3F. */
    uint32_t wrapper_ioctrl[BCM_BACKPLANE_NUM_WRAPPERS];
    uint32_t wrapper_resetctrl[BCM_BACKPLANE_NUM_WRAPPERS];

    /* OTP fuse array as seen through ChipCoreID 8, see apple_bcm_wlan_otp_cis. */
    uint8_t otp[BCM_OTP_IMAGE_SIZE];
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
 * PCI config-space access to the Broadcom-proprietary registers.
 *
 * The window registers and the SPROM control register are made writable in
 * realize() by opening up their wmask, so pci_default_{read,write}_config()
 * already implements them as exact-read-back 32-bit scratch storage at any
 * access width. These overrides exist only to trace them; every other offset
 * (PM @0x40, MSI @0x50, Express @0xD0, AER @0x100, the BARs, ...) is left
 * entirely to the parent implementation.
 */
static bool apple_bcm_wlan_cfg_is_broadcom_reg(uint32_t addr, int len)
{
    static const uint32_t regs[] = {
        BCM_PCI_CFG_BAR0_WINDOW1, BCM_PCI_CFG_BAR0_WINDOW4,
        BCM_PCI_CFG_BAR0_WINDOW5, BCM_PCI_CFG_BAR0_WINDOW0,
        BCM_PCI_CFG_SPROM_CONTROL,
    };
    size_t i;

    for (i = 0; i < ARRAY_SIZE(regs); i++) {
        if (addr < regs[i] + 4 && regs[i] < addr + len) {
            return true;
        }
    }
    return false;
}

static uint32_t apple_bcm_wlan_device_config_read(PCIDevice *dev,
                                                  uint32_t addr, int len)
{
    uint32_t val;

    val = pci_default_read_config(dev, addr, len);

    if (apple_bcm_wlan_cfg_is_broadcom_reg(addr, len)) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: CFG READ @ 0x%02x value: 0x%x len %d\n", __func__,
                      addr, val, len);
    }

    return val;
}

static void apple_bcm_wlan_device_config_write(PCIDevice *dev, uint32_t addr,
                                               uint32_t val, int len)
{
    if (apple_bcm_wlan_cfg_is_broadcom_reg(addr, len)) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: CFG WRITE @ 0x%02x value: 0x%x len %d\n", __func__,
                      addr, val, len);
    }

    pci_default_write_config(dev, addr, val, len);
}

/*
 * BAR0 window table (kBCOM4378ChipBackplaneWindows @0xfffffff00730d21c).
 *
 * Eight 4 KiB apertures; an access at BAR0 offset `index * 0x1000 + off` is
 * dispatched to backplane address `base + off`. Four windows take their base
 * from a config register (remappable), four are pinned to a fixed core. The
 * last two entries are never used by the driver but are modelled the same way
 * the table describes them.
 */
typedef struct AppleBCMWLANBackplaneWindow {
    const char *name;
    uint8_t cfg_reg; // config register steering the window, 0 when fixed
    uint64_t fixed_base; // backplane base when cfg_reg == 0
} AppleBCMWLANBackplaneWindow;

static const AppleBCMWLANBackplaneWindow
    apple_bcm_wlan_backplane_windows[APPLE_BCM_WLAN_NUM_WINDOWS] = {
        { "core@cfg0x80", BCM_PCI_CFG_BAR0_WINDOW0, 0 },
        { "wrapper@cfg0x70", BCM_PCI_CFG_BAR0_WINDOW1, 0 },
        { "pcie2", 0, BCM_BACKPLANE_PCIE2_BASE },
        { "chipcommon", 0, BCM_BACKPLANE_CHIPCOMMON_BASE },
        { "core@cfg0x74", BCM_PCI_CFG_BAR0_WINDOW4, 0 },
        { "wrapper@cfg0x78", BCM_PCI_CFG_BAR0_WINDOW5, 0 },
        { "pcie2(unused)", 0, BCM_BACKPLANE_PCIE2_BASE },
        { "chipcommon(unused)", 0, BCM_BACKPLANE_CHIPCOMMON_BASE },
    };

static uint64_t apple_bcm_wlan_window_base(AppleBCMWLANDeviceState *s,
                                           unsigned index)
{
    const AppleBCMWLANBackplaneWindow *win =
        &apple_bcm_wlan_backplane_windows[index];

    if (win->cfg_reg == 0) {
        return win->fixed_base;
    }
    return pci_get_long(PCI_DEVICE(s)->config + win->cfg_reg);
}

/*
 * Backplane register reads.
 *
 * CRITICAL: never return 0xFFFFFFFF. AppleBCMWLANChipBackplane::readRegister32
 * treats an all-ones result as a dead chip, returns 0xE3FF830A and its
 * forcePowerLite() caller turns that into a kernel panic. Unimplemented
 * registers therefore read as 0.
 */
static uint32_t apple_bcm_wlan_backplane_read(AppleBCMWLANDeviceState *s,
                                              uint64_t addr)
{
    uint64_t off;

    if (addr - BCM_BACKPLANE_CHIPCOMMON_BASE < BCM_BACKPLANE_CORE_SIZE) {
        off = addr - BCM_BACKPLANE_CHIPCOMMON_BASE;
        switch (off) {
        case BCM_CHIPCOMMON_CHIPID:
            return BCM4378_CHIPID_VALUE;
        case BCM_CHIPCOMMON_CAPABILITIES:
            /* SROM escape hatch, see BCM_CHIPCOMMON_CAPABILITIES_VALUE. */
            return BCM_CHIPCOMMON_CAPABILITIES_VALUE;
        case BCM_CHIPCOMMON_GPIOOUT:
            return s->cc_gpioout;
        case BCM_CHIPCOMMON_GPIOOUTEN:
            return s->cc_gpioouten;
        case BCM_CHIPCOMMON_GPIOCONTROL:
            return s->cc_gpiocontrol;
        case BCM_CHIPCOMMON_GCI_INDIRECT_ADDR:
            return s->cc_gci_indirect_addr;
        case BCM_CHIPCOMMON_GCI_CHIPCTRL:
            return s->cc_gci_chipctrl;
        default:
            break;
        }
    } else if (addr - BCM_BACKPLANE_PCIE2_BASE < BCM_BACKPLANE_CORE_SIZE) {
        off = addr - BCM_BACKPLANE_PCIE2_BASE;
        switch (off) {
        case BCM_PCIE2_CONFIGADDR:
            return s->pcie_configaddr;
        case BCM_PCIE2_CONFIGDATA:
            /*
             * Indirect access to the PCIe core's own config space is not
             * modelled. Read as 0 rather than echoing back what was written:
             * the bring-up sequence writes 0xFFFFFFFF here (to clear sticky
             * error bits) and reading that value back would be fatal.
             */
            return 0;
        case BCM_PCIE2_H2D_DOORBELL_0:
            return s->pcie_h2d_doorbell_0;
        case BCM_PCIE2_H2D_MAILBOX_DATA:
            return s->pcie_h2d_mailbox_data;
        case BCM_PCIE2_POWER_CONTROL:
            return s->pcie_power_control;
        case BCM_PCIE2_MAILBOXINT:
            return s->pcie_mailboxint;
        case BCM_PCIE2_MAILBOXMASK:
            return s->pcie_mailboxmask;
        default:
            break;
        }
    } else if (addr - BCM_BACKPLANE_GCI_BASE < BCM_BACKPLANE_CORE_SIZE) {
        off = addr - BCM_BACKPLANE_GCI_BASE;
        switch (off) {
        case BCM_GCI_INDEX:
            /* checkHardware() writes 4 here and requires 4 back. */
            return s->gci_index;
        case BCM_GCI_STATUS:
            /* Bit 6 set == "GCI not ready"; checkHardware() fails 0xE00002CA. */
            return BCM_GCI_STATUS_VALUE;
        case BCM_GCI_CHIPCTRL:
            return s->gci_chipctrl;
        default:
            break;
        }
    } else if (addr - BCM_BACKPLANE_OTP_BASE < BCM_BACKPLANE_CORE_SIZE) {
        /*
         * OTP fuse array. copyRegisters16() reads it as consecutive 16-bit
         * loads; the BAR0 region only implements 32-bit accesses, so QEMU
         * hands us the containing aligned word and extracts the half itself.
         * Anything past the 0x400-byte image reads as 0.
         */
        off = addr - BCM_BACKPLANE_OTP_BASE;
        if (off + 4 <= BCM_OTP_IMAGE_SIZE) {
            return ldl_le_p(s->otp + off);
        }
        return 0;
    } else if (addr >= BCM_BACKPLANE_WRAPPER_BASE &&
               addr < BCM_BACKPLANE_WRAPPER_END) {
        unsigned index = (addr - BCM_BACKPLANE_WRAPPER_BASE) / 0x1000;

        off = addr & 0xFFF;
        switch (off) {
        case BCM_AI_IOCTRL:
            return s->wrapper_ioctrl[index];
        case BCM_AI_RESETCTRL:
            return s->wrapper_resetctrl[index];
        case BCM_AI_RESETSTATUS:
            /*
             * loadChipImage() asserts reset (RESETCTRL bit0 = 1) and then
             * polls RESETSTATUS for bit0 to go clear within a second. There is
             * no core behind the wrapper to actually reset, so report the
             * reset as always already complete; a real chip completes this in
             * microseconds anyway.
             */
            return 0;
        default:
            break;
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "%s: UNIMP backplane READ @ 0x%" PRIx64 " -> 0\n", __func__,
                  addr);
    return 0;
}

static void apple_bcm_wlan_backplane_write(AppleBCMWLANDeviceState *s,
                                           uint64_t addr, uint32_t data)
{
    uint64_t off;

    if (addr - BCM_BACKPLANE_CHIPCOMMON_BASE < BCM_BACKPLANE_CORE_SIZE) {
        off = addr - BCM_BACKPLANE_CHIPCOMMON_BASE;
        switch (off) {
        case BCM_CHIPCOMMON_GPIOOUT:
            s->cc_gpioout = data;
            return;
        case BCM_CHIPCOMMON_GPIOOUTEN:
            s->cc_gpioouten = data;
            return;
        case BCM_CHIPCOMMON_GPIOCONTROL:
            s->cc_gpiocontrol = data;
            return;
        case BCM_CHIPCOMMON_GCI_INDIRECT_ADDR:
            s->cc_gci_indirect_addr = data;
            return;
        case BCM_CHIPCOMMON_GCI_CHIPCTRL:
            s->cc_gci_chipctrl = data;
            return;
        default:
            break;
        }
    } else if (addr - BCM_BACKPLANE_PCIE2_BASE < BCM_BACKPLANE_CORE_SIZE) {
        off = addr - BCM_BACKPLANE_PCIE2_BASE;
        switch (off) {
        case BCM_PCIE2_CONFIGADDR:
            s->pcie_configaddr = data;
            return;
        case BCM_PCIE2_CONFIGDATA:
            /* Swallowed; see the read side. */
            return;
        case BCM_PCIE2_H2D_DOORBELL_0:
            /*
             * Host -> device doorbell. The written value is a microsecond
             * timestamp, not a command, so it carries no information: any
             * write means "the host queued work on the msgbuf rings". Phase 2
             * will consume the rings from here.
             */
            s->pcie_h2d_doorbell_0 = data;
            qemu_log_mask(LOG_UNIMP,
                          "%s: H2D doorbell 0 rung (value 0x%x ignored)\n",
                          __func__, data);
            return;
        case BCM_PCIE2_H2D_MAILBOX_DATA:
            s->pcie_h2d_mailbox_data = data;
            return;
        case BCM_PCIE2_POWER_CONTROL:
            s->pcie_power_control = data;
            return;
        case BCM_PCIE2_MAILBOXINT:
            /*
             * Device -> host status, write-1-to-clear. The bits must stay
             * asserted until the host acknowledges them, which is exactly what
             * the driver's ISR does: v = read(INT); write(INT, v).
             */
            s->pcie_mailboxint &= ~data;
            return;
        case BCM_PCIE2_MAILBOXMASK:
            s->pcie_mailboxmask = data;
            return;
        default:
            break;
        }
    } else if (addr - BCM_BACKPLANE_GCI_BASE < BCM_BACKPLANE_CORE_SIZE) {
        off = addr - BCM_BACKPLANE_GCI_BASE;
        switch (off) {
        case BCM_GCI_INDEX:
            s->gci_index = data;
            return;
        case BCM_GCI_CHIPCTRL:
            /* prepareHardware() does a (v & ~3) | 1 read-modify-write. */
            s->gci_chipctrl = data;
            return;
        default:
            break;
        }
    } else if (addr >= BCM_BACKPLANE_WRAPPER_BASE &&
               addr < BCM_BACKPLANE_WRAPPER_END) {
        unsigned index = (addr - BCM_BACKPLANE_WRAPPER_BASE) / 0x1000;

        off = addr & 0xFFF;
        switch (off) {
        case BCM_AI_IOCTRL:
            /* loadChipImage() writes 0x23 and requires 0x23 back. */
            s->wrapper_ioctrl[index] = data;
            return;
        case BCM_AI_RESETCTRL:
            s->wrapper_resetctrl[index] = data;
            return;
        default:
            break;
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "%s: UNIMP backplane WRITE @ 0x%" PRIx64 " value 0x%x\n",
                  __func__, addr, data);
}

static uint64_t apple_bcm_wlan_device_bar0_read(void *opaque, hwaddr addr,
                                                unsigned size)
{
    AppleBCMWLANDeviceState *s = opaque;
    unsigned index = addr / APPLE_BCM_WLAN_WINDOW_SIZE;
    uint64_t backplane_addr =
        apple_bcm_wlan_window_base(s, index) + (addr % APPLE_BCM_WLAN_WINDOW_SIZE);
    uint32_t val;

    val = apple_bcm_wlan_backplane_read(s, backplane_addr);

    qemu_log_mask(LOG_UNIMP,
                  "%s: READ @ 0x" HWADDR_FMT_plx " (window %u %s, backplane "
                  "0x%" PRIx64 ") value: 0x%x size %u\n",
                  __func__, addr, index,
                  apple_bcm_wlan_backplane_windows[index].name, backplane_addr,
                  val, size);
    return val;
}

static void apple_bcm_wlan_device_bar0_write(void *opaque, hwaddr addr,
                                             uint64_t data, unsigned size)
{
    AppleBCMWLANDeviceState *s = opaque;
    unsigned index = addr / APPLE_BCM_WLAN_WINDOW_SIZE;
    uint64_t backplane_addr =
        apple_bcm_wlan_window_base(s, index) + (addr % APPLE_BCM_WLAN_WINDOW_SIZE);

    qemu_log_mask(LOG_UNIMP,
                  "%s: WRITE @ 0x" HWADDR_FMT_plx " (window %u %s, backplane "
                  "0x%" PRIx64 ") value: 0x%" PRIx64 " size %u\n",
                  __func__, addr, index,
                  apple_bcm_wlan_backplane_windows[index].name, backplane_addr,
                  data, size);

    apple_bcm_wlan_backplane_write(s, backplane_addr, (uint32_t)data);
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

    /*
     * Open up the Broadcom-proprietary config registers so they behave as
     * plain 32-bit read/write scratch storage: pci_default_write_config()
     * only lets a byte through where wmask is set, and validateWindow()
     * requires the window base it just wrote to read back unchanged. Done
     * after the capabilities are installed so it cannot silently make a
     * capability register writable if one ever moves on top of them.
     */
    pci_set_long(dev->wmask + BCM_PCI_CFG_BAR0_WINDOW1, 0xFFFFFFFF);
    pci_set_long(dev->wmask + BCM_PCI_CFG_BAR0_WINDOW4, 0xFFFFFFFF);
    pci_set_long(dev->wmask + BCM_PCI_CFG_BAR0_WINDOW5, 0xFFFFFFFF);
    pci_set_long(dev->wmask + BCM_PCI_CFG_BAR0_WINDOW0, 0xFFFFFFFF);
    pci_set_long(dev->wmask + BCM_PCI_CFG_SPROM_CONTROL, 0xFFFFFFFF);

    /* T8030 combo chip links at 5GT; mirror the baseband link-cap fill. */
    if (s->port->maximum_link_speed == 2) {
        pcie_cap_fill_link_ep_usp(dev, QEMU_PCI_EXP_LNK_X1,
                                  QEMU_PCI_EXP_LNK_8GT);
    } else if (s->port->maximum_link_speed == 1) {
        pcie_cap_fill_link_ep_usp(dev, QEMU_PCI_EXP_LNK_X2,
                                  QEMU_PCI_EXP_LNK_5GT);
    }

    /*
     * No BAR-visibility workaround (unlike baseband.c, which pins aliases of
     * its BARs at a hardcoded system-memory address): apcie now maps the PCI
     * memory space into system memory according to the apcie node's "ranges",
     * so the BARs are reachable wherever iOS' IOPCIConfigurator assigns them.
     */
    pci_register_bar(dev, 0, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar0);
    pci_register_bar(dev, 2, PCI_BASE_ADDRESS_SPACE_MEMORY, &s->bar2);
}

static void apple_bcm_wlan_device_qdev_reset_hold(Object *obj, ResetType type)
{
    AppleBCMWLANDeviceState *s = APPLE_BCM_WLAN_DEVICE(obj);
    PCIDevice *dev = PCI_DEVICE(obj);

    pci_set_word(dev->config + PCI_COMMAND,
                 PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER);

    /* Backplane window / SPROM control registers live in config space. */
    pci_set_long(dev->config + BCM_PCI_CFG_BAR0_WINDOW1, 0);
    pci_set_long(dev->config + BCM_PCI_CFG_BAR0_WINDOW4, 0);
    pci_set_long(dev->config + BCM_PCI_CFG_BAR0_WINDOW5, 0);
    pci_set_long(dev->config + BCM_PCI_CFG_BAR0_WINDOW0, 0);
    pci_set_long(dev->config + BCM_PCI_CFG_SPROM_CONTROL, 0);

    s->cc_gpioout = 0;
    s->cc_gpioouten = 0;
    s->cc_gpiocontrol = 0;
    s->cc_gci_indirect_addr = 0;
    s->cc_gci_chipctrl = 0;

    s->gci_index = 0;
    s->gci_chipctrl = 0;

    s->pcie_configaddr = 0;
    s->pcie_h2d_doorbell_0 = 0;
    s->pcie_h2d_mailbox_data = 0;
    s->pcie_power_control = 0;
    s->pcie_mailboxint = 0;
    s->pcie_mailboxmask = 0;

    memset(s->wrapper_ioctrl, 0, sizeof(s->wrapper_ioctrl));
    memset(s->wrapper_resetctrl, 0, sizeof(s->wrapper_resetctrl));

    /* Unprogrammed fuses read as 0; the CIS sits at kBCOM4378ChipUserOTP. */
    QEMU_BUILD_BUG_ON(BCM_OTP_CIS_OFFSET + sizeof(apple_bcm_wlan_otp_cis) >
                      BCM_OTP_IMAGE_SIZE);
    memset(s->otp, 0, sizeof(s->otp));
    memcpy(s->otp + BCM_OTP_CIS_OFFSET, apple_bcm_wlan_otp_cis,
           sizeof(apple_bcm_wlan_otp_cis));
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
    c->config_read = apple_bcm_wlan_device_config_read;
    c->config_write = apple_bcm_wlan_device_config_write;
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
