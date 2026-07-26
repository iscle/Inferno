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

/*
 * The only MAILBOXINT bit we may ever raise.
 *
 * AppleBCMWLANBusInterfacePCIe's ISR reads MAILBOXINT, writes the value straight
 * back (write-1-to-clear) and then dispatches on it. It ignores everything
 * outside 0x00010100, and the 0x100 ("D2H mailbox data") arm of that dispatch
 * NULL-dereferences in this build, so 0x00010000 -- "the device wrote to a D2H
 * ring" -- is the one bit that is both understood and safe.
 */
#define BCM_PCIE2_MAILBOXINT_D2H_DB0 0x00010000U

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
 * FAILS on anything malformed, and publishes it as "HWIdentifiers".
 *
 * Two of the strings are ALSO published verbatim, by getModuleInfo
 * (@0xfffffff0095c4a60): "ChipInfo" = the Manufacturer string (+0xc8) and
 * "ModuleInfo" = the Product string (+0xd0). AppleBCMWLANCore::generateFileName
 * (@0xfffffff009466cd0) turns those two into the firmware path:
 *
 *     <upper-case keys joined by '_'> "__" <lower-case keys joined by '_'>
 *
 * for each of them (AppleBCMWLANCore::copyKeys @0xfffffff00946779c selects by
 * key case), joined with '/', with '=' rendered as '-'; if ChipInfo has no
 * "C=" key the driver prepends "C=<chip number>".
 *
 * ModuleInfo must NOT carry a "P=" key: generateFileName takes the "P=" key as
 * a signal that the platform is already named and then skips the branch that
 * appends the device tree's "module-instance" to the SHORT name buffers -- the
 * ones the firmware/CLM/TxCap files use. With "P=" present you get
 * "C-4378__s-B1/.trx" and a bare ".clmb"/".txcb"; without it the driver builds
 * the platform name itself. So
 *
 *     ChipInfo   = "s=B1"
 *     ModuleInfo = "M=GODF V=m m=4.3"
 *
 * asks for exactly what the guest's own /usr/share/firmware/wifi contains for
 * this board:
 *
 *     C-4378__s-B1/moana.trx  .clmb  .txcb
 *     C-4378__s-B1/P-moana_M-GODF_V-m__m-4.3.txt   (NVRAM)
 *
 * ("moana" is the DT module-instance of the n104ap WLAN node.) We deliberately
 * name real files: the guest downloads ITS OWN firmware into our BAR2 and we
 * merely swallow the writes; we neither ship nor execute it.
 */
static const uint8_t apple_bcm_wlan_otp_cis[] = {
    /* CISTPL_MANFID: manufacturer 0x02D0 (Broadcom), card id 0x4378. */
    CIS_TPL_MANFID, 0x04, 0xD0, 0x02, 0x78, 0x43,
    /* CISTPL_FUNCID: function 0x0C == network adapter, no sysinit. */
    CIS_TPL_FUNCID, 0x02, 0x0C, 0x00,
    /*
     * CISTPL_VERS_1: major 8, minor 0, then the four strings in the slot
     * order the PCIe override uses: ProductInfo0, ProductInfo1, Manufacturer
     * (== ChipInfo), Product (== ModuleInfo). Terminated by 0xFF.
     */
    CIS_TPL_VERS_1, 0x23, 0x08, 0x00,
    'V', '=', 'm', '\0', // ProductInfo0: module vendor (Murata)
    'm', '=', '4', '.', '3', '\0', // ProductInfo1: module revision
    's', '=', 'B', '1', '\0', // Manufacturer -> "ChipInfo": chip stepping
    // Product -> "ModuleInfo": module, vendor, module revision
    'M', '=', 'G', 'O', 'D', 'F', ' ', 'V', '=', 'm', ' ', 'm', '=', '4', '.',
    '3', '\0',
    CIS_TPL_END,
    /* End of the tuple stream. */
    CIS_TPL_END,
};

/* AI (wrapper) registers, identical for every wrapper. */
#define BCM_AI_IOCTRL 0x408
#define BCM_AI_RESETCTRL 0x800
#define BCM_AI_RESETSTATUS 0x804

/*
 * The ARM core's own wrapper (kBCOM4378ChipWrappers id 2, "ARMMaster"). This
 * is the one loadChipImage() drives to take the CPU in and out of reset around
 * the firmware download.
 */
#define BCM_BACKPLANE_ARM_WRAPPER_BASE 0x18130000ULL
#define BCM_BACKPLANE_ARM_WRAPPER_INDEX \
    ((BCM_BACKPLANE_ARM_WRAPPER_BASE - BCM_BACKPLANE_WRAPPER_BASE) / 0x1000)

/*
 * Chip memories (kBCOM4378ChipMemories @0xfffffff00730d1c8). BAR2 maps them
 * identity-style, so a "dongle address" is simply a BAR2 offset.
 *
 * ChipMemoryID 4 is the firmware RAM: { 0x352000, 0x1CE000 }. loadChipImage()
 * blits the firmware at its base, a block of random bytes and the NVRAM near
 * its top, then writes a token into its LAST word and releases the ARM core.
 * It then polls that same word until it reads back as neither 0xFFFFFFFF nor
 * the token -- i.e. until the firmware has replaced it with a pointer to its
 * pciedev_shared_t. There is no magic value to match; the only thing checked
 * is that the pointer lands inside the firmware RAM, is 4-byte aligned, and
 * leaves room for the 0x78-byte structure.
 */
#define BCM_MEM_FW_RAM_BASE 0x352000
#define BCM_MEM_FW_RAM_SIZE 0x1CE000
#define BCM_MEM_FW_RAM_END (BCM_MEM_FW_RAM_BASE + BCM_MEM_FW_RAM_SIZE)
#define BCM_SHARED_INFO_PTR_ADDR (BCM_MEM_FW_RAM_END - 4)

/*
 * Where we put the structures the (absent) firmware would have published. This
 * firmware image occupies roughly the first 1.3 MiB of chip RAM and the NVRAM
 * sits at the very top, so the middle of the region is free; nothing ever
 * reads the downloaded image back, so even an overlap would be harmless.
 */
#define BCM_SHARED_INFO_ADDR (BCM_MEM_FW_RAM_BASE + 0x1C0000)
#define BCM_RING_INFO_ADDR (BCM_SHARED_INFO_ADDR + 0x80)
#define BCM_H2D_MB_DATA_ADDR (BCM_SHARED_INFO_ADDR + 0x100)
#define BCM_D2H_MB_DATA_ADDR (BCM_SHARED_INFO_ADDR + 0x110)
/*
 * The ring-descriptor array. One 16-byte descriptor per ring, and the host
 * fills in *every* ring it might ever create: the two common H2D rings, the
 * three common D2H rings, and one per TX flow ring. With 40 flow rings that is
 * 45 descriptors, so reserve a comfortable 4 KiB.
 */
#define BCM_RINGMEM_ADDR (BCM_SHARED_INFO_ADDR + 0x1000)
#define BCM_RINGMEM_MAX_RINGS 128

/*
 * pciedev_shared_t. Only four fields are ever READ by the driver; everything
 * else in the structure is filled in by the host once it accepts us, so we
 * leave the rest zero.
 *
 *   0x00 flags
 *   0x04 trap_addr        (only dereferenced from handleFWTrap())
 *   0x30 rings_info_ptr
 *   0x50 flags2
 *
 * flags:
 *   [7:0]  msgbuf protocol version, must be 5..7 (@0xfffffff0095c93d0)
 *   b16    host writes the ring indices by DMA. MANDATORY: without it the
 *          driver bails with "Driver only supports FW with bi-directional
 *          ring index DMA" (@0xfffffff0095ca4b0).
 *   b29    "no out-of-band device wake"
 *   b30    "in-band device wake supported"
 *          b29 set together with b30 clear is fatal; and with b30 clear the
 *          driver falls back to an out-of-band device-wake GPIO which an
 *          emulated endpoint does not have, so b30 has to be set.
 *
 * flags2 is NOT validated -- it is a bag of feature bits. Leave it zero: the
 * one trap is that bit 4 without bit 2 panics ("btLogNoMaxQIncrease is set but
 * BT logging isn't supported", @0xfffffff0095cbfc0).
 */
#define BCM_SHARED_SIZE 0x78
#define BCM_SHARED_FLAGS_OFF 0x00
#define BCM_SHARED_RINGS_INFO_PTR_OFF 0x30
#define BCM_SHARED_FLAGS2_OFF 0x50

#define BCM_SHARED_VERSION 6
#define BCM_SHARED_FLAG_DMA_INDEX (1U << 16)
#define BCM_SHARED_FLAG_INBAND_DEVICE_WAKE (1U << 30)
#define BCM_SHARED_FLAGS                              \
    (BCM_SHARED_VERSION | BCM_SHARED_FLAG_DMA_INDEX | \
     BCM_SHARED_FLAG_INBAND_DEVICE_WAKE)
#define BCM_SHARED_FLAGS2 0x00000000

/*
 * ring_info_t. Again only a few fields come from us: the ring-memory pointer
 * and the two u16 counts at 0x34/0x36. 0x14..0x30 are the four 64-bit host
 * DMA addresses of the index arrays, which the HOST writes, and 0x38/0x3A
 * (max_completion_rings / max_rxbufpost) are never read at all.
 */
#define BCM_RING_INFO_SIZE 0x3C
#define BCM_RING_INFO_RINGMEM_PTR_OFF 0x00
#define BCM_RING_INFO_MAX_TX_FLOWRINGS_OFF 0x34
#define BCM_RING_INFO_MAX_SUBMISSION_QUEUES_OFF 0x36

/*
 * ring_info_t continued: the four host DMA addresses of the ring-index arrays
 * and the ring counts. The host writes 0x14..0x33; we only ever read them.
 */
#define BCM_RING_INFO_H2D_W_IDX_HOSTADDR_OFF 0x14
#define BCM_RING_INFO_H2D_R_IDX_HOSTADDR_OFF 0x1C
#define BCM_RING_INFO_D2H_W_IDX_HOSTADDR_OFF 0x24
#define BCM_RING_INFO_D2H_R_IDX_HOSTADDR_OFF 0x2C
#define BCM_RING_INFO_MAX_COMPLETION_RINGS_OFF 0x38

/*
 * ring_mem_t, one per ring, written by the host into the array at
 * ring_info.ringmem_ptr. AppleBCMWLANBusInterfacePCIe::fillRingEndpointMemory
 * (@0xfffffff0095cf208 for submission rings, @0xfffffff0095d1a24 for completion
 * rings) writes exactly these fields.
 */
#define BCM_RINGMEM_ENTRY_SIZE 0x10
#define BCM_RINGMEM_IDX_OFF 0x00 // u16, the ring's own id
#define BCM_RINGMEM_TYPE_OFF 0x02 // u8
#define BCM_RINGMEM_MAX_ITEM_OFF 0x04 // u16
#define BCM_RINGMEM_LEN_ITEMS_OFF 0x06 // u16
#define BCM_RINGMEM_BASE_ADDR_OFF 0x08 // u32 low, u32 high

/*
 * Ring types, from the ring objects' getRingType(). Standard msgbuf numbering:
 * the two H2D common rings are the control submit ring and the RX-post ring,
 * the three D2H common rings are the control/TX/RX completion rings, and TX
 * flow rings get their own type.
 */
#define BCM_RING_TYPE_H2D_CTRL_SUBMIT 0
#define BCM_RING_TYPE_D2H_CTRL_COMPLETE 1
#define BCM_RING_TYPE_H2D_RXPOST_SUBMIT 2
#define BCM_RING_TYPE_D2H_RX_COMPLETE 3
#define BCM_RING_TYPE_D2H_TX_COMPLETE 4
#define BCM_RING_TYPE_H2D_TXFLOW 5

/*
 * msgbuf message types, shared by both directions. Confirmed against
 * AppleBCMWLANBusInterfacePCIe::fillControlSubmitRing (@0xfffffff0095cd6e0),
 * which emits 0x1B/0x1C/0x23/0x25/0x26/0x29/0x2B/0x2C and the ioctl request,
 * and ::drainControlCompleteRing (@0xfffffff0095cf70c), whose two jump tables
 * cover 0x01..0x14 and 0x1D..0x2E.
 */
#define BCM_MSGBUF_TYPE_IOCTLPTR_REQ 0x09
#define BCM_MSGBUF_TYPE_IOCTLPTR_REQ_ACK 0x0A
#define BCM_MSGBUF_TYPE_IOCTLRESP_BUF_POST 0x0B
#define BCM_MSGBUF_TYPE_IOCTL_CMPLT 0x0C
#define BCM_MSGBUF_TYPE_EVENT_BUF_POST 0x0D
#define BCM_MSGBUF_TYPE_WL_EVENT 0x0E
#define BCM_MSGBUF_TYPE_H2D_RING_CREATE 0x1B
#define BCM_MSGBUF_TYPE_D2H_RING_CREATE 0x1C
#define BCM_MSGBUF_TYPE_H2D_RING_CREATE_CMPLT 0x1D
#define BCM_MSGBUF_TYPE_D2H_RING_CREATE_CMPLT 0x1E

/*
 * Common message header, 8 bytes, at the front of every ring item:
 *
 *   0x00 u8  msgtype
 *   0x01 u8  ifidx
 *   0x02 u8  flags -- bit 7 is the ring's phase bit
 *   0x03 u8  epoch
 *   0x04 u32 request_id
 *
 * The phase bit alternates every time the producer wraps the ring, so the
 * consumer can tell a fresh item from a stale one even before the index
 * arrives. AppleBCMWLANPCIeCompletionRing::initWithOptions (@0xfffffff0095ebe88)
 * starts a D2H ring's expected phase at 1 and ::requestRingDrain
 * (@0xfffffff0095ec10c) flips it on every wrap, so our first message must carry
 * bit 7 SET. A mismatch is only a logged fault, not a hard error.
 */
#define BCM_MSGBUF_HDR_MSGTYPE_OFF 0x00
#define BCM_MSGBUF_HDR_IFIDX_OFF 0x01
#define BCM_MSGBUF_HDR_FLAGS_OFF 0x02
#define BCM_MSGBUF_HDR_REQUEST_ID_OFF 0x04
#define BCM_MSGBUF_FLAG_PHASE 0x80

/*
 * ioctl_req_msg_t (H2D control submit, 40 bytes). Built at
 * 0xfffffff0095ce9c0..0xfffffff0095cea94.
 */
#define BCM_IOCTL_REQ_CMD_OFF 0x08 // u32
#define BCM_IOCTL_REQ_TRANS_ID_OFF 0x0C // u16
#define BCM_IOCTL_REQ_INPUT_LEN_OFF 0x0E // u16
#define BCM_IOCTL_REQ_OUTPUT_LEN_OFF 0x10 // u16
#define BCM_IOCTL_REQ_BUF_ADDR_OFF 0x18 // u32 low, u32 high

/*
 * ioctl_resp_evt_buf_post_msg_t (H2D control submit, 40 bytes). Built by
 * AppleBCMWLANBusInterfacePCIe::submitControlBufferMsg (@0xfffffff0095d9c80),
 * which serves message types 0x0B, 0x0D and 0x25.
 */
#define BCM_BUF_POST_HOST_BUF_LEN_OFF 0x08 // u16
#define BCM_BUF_POST_HOST_BUF_ADDR_OFF 0x10 // u32 low, u32 high

/*
 * ioctl_comp_resp_msg_t (D2H control complete, 24 bytes). Parsed at
 * 0xfffffff0095cfdf8: request_id identifies the response buffer the host posted
 * earlier, resp_len is how much of it we filled, and status is the BCME code.
 */
#define BCM_IOCTL_CMPLT_STATUS_OFF 0x08 // s16
#define BCM_IOCTL_CMPLT_FLOW_RING_ID_OFF 0x0A // u16
#define BCM_IOCTL_CMPLT_RESP_LEN_OFF 0x0C // u16
#define BCM_IOCTL_CMPLT_TRANS_ID_OFF 0x0E // u16
#define BCM_IOCTL_CMPLT_CMD_OFF 0x10 // u32

/* Ring item sizes we produce. */
#define BCM_H2D_CTRL_ITEM_SIZE 40
#define BCM_D2H_CTRL_ITEM_SIZE 24

/*
 * WLC command numbers and BCME status codes.
 *
 * Everything we do not implement answers BCME_UNSUPPORTED; the driver maps a
 * BCME code -N onto 0xE3FF81NN and explicitly tolerates "unsupported" for
 * optional features, so a blanket -23 is the correct default rather than a
 * generic failure.
 */
#define WLC_UP 2
#define WLC_DOWN 3
#define WLC_GET_VERSION 1
#define WLC_SET_RADIO 38
#define WLC_SET_REGULATORY 47
#define WLC_GET_COUNTRY 83
#define WLC_SET_COUNTRY 84
#define WLC_GET_VALID_CHANNELS 217
#define WLC_GET_COUNTRY_LIST 261
#define WLC_GET_VAR 262
#define WLC_SET_VAR 263

#define BCME_OK 0
#define BCME_ERROR (-1)
#define BCME_UNSUPPORTED (-23)
#define BCME_BUFTOOSHORT (-24)

/* How many host ioctl-response / event buffers we keep queued. */
#define BCM_MAX_POSTED_BUFS 256

/*
 * Ring geometry, mirroring brcmfmac's msgbuf defaults. The driver panics
 * ("maxNbrOfDynamicSubmissionRings <= maxNbrOfTxFlowRings",
 * @0xfffffff0095cbfa0) unless max_submission_queues is STRICTLY GREATER than
 * max_tx_flowrings: the submission-queue array has to hold the two common H2D
 * rings on top of the per-flow TX rings.
 */
#define BCM_MAX_TX_FLOWRINGS 40
#define BCM_MAX_SUBMISSION_QUEUES (BCM_MAX_TX_FLOWRINGS + 2)

/*
 * One msgbuf ring, as described by its ring_mem_t descriptor plus the index
 * slot the host allocated for it.
 *
 * The producer's index lives in host DMA memory (a u32 per ring, taken from
 * the arrays whose addresses ring_info carries); so does the consumer's. For an
 * H2D ring the host owns the write index and we own the read index; for a D2H
 * ring it is the other way round.
 */
typedef struct AppleBCMWLANRing {
    bool valid;
    unsigned slot; // index into the ringmem descriptor array
    uint16_t id; // index into the H2D or D2H index arrays
    uint8_t type;
    uint16_t max_item;
    uint16_t len_items;
    uint64_t base_addr; // host DMA address of the item array
    uint32_t index; // our own end: read index (H2D) / write index (D2H)
    uint8_t phase; // D2H only, see BCM_MSGBUF_FLAG_PHASE
} AppleBCMWLANRing;

/* A host buffer posted to us for an ioctl response or an event. */
typedef struct AppleBCMWLANPostedBuf {
    uint32_t request_id;
    uint64_t addr;
    uint16_t len;
} AppleBCMWLANPostedBuf;

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

    /*
     * msgbuf state. Everything here is (re)discovered from the ring_info the
     * host filled in, on the first doorbell after the firmware handshake.
     */
    bool rings_discovered;
    uint64_t h2d_w_idx_addr, h2d_r_idx_addr;
    uint64_t d2h_w_idx_addr, d2h_r_idx_addr;
    AppleBCMWLANRing h2d_ctrl;
    AppleBCMWLANRing d2h_ctrl;

    /* Host buffers posted for ioctl responses and for events. */
    AppleBCMWLANPostedBuf ioctl_resp_bufs[BCM_MAX_POSTED_BUFS];
    unsigned ioctl_resp_head, ioctl_resp_count;
    AppleBCMWLANPostedBuf event_bufs[BCM_MAX_POSTED_BUFS];
    unsigned event_head, event_count;
};

struct AppleBCMWLANState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    AppleBCMWLANDeviceState *device;

    PCIBus *pci_bus;
};

/*
 * DMA helpers.
 *
 * The endpoint sits behind the apcie DART, so host addresses handed to us over
 * msgbuf are IOVAs and must go through the port's own AddressSpace -- never
 * cpu_physical_memory_*().
 */
static bool apple_bcm_wlan_dma_read(AppleBCMWLANDeviceState *s, uint64_t offset,
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

static bool apple_bcm_wlan_dma_write(AppleBCMWLANDeviceState *s,
                                     uint64_t offset, uint64_t size,
                                     uint8_t *buf)
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
static void apple_bcm_wlan_publish_shared_info(AppleBCMWLANDeviceState *s);
static void apple_bcm_wlan_h2d_doorbell(AppleBCMWLANDeviceState *s);

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
             * Host -> device doorbell. AppleBCMWLANBusInterfacePCIe::hitDoorbell
             * (@0xfffffff0095d96f4) writes a microsecond timestamp here, so the
             * value carries no information: any write means "the host queued
             * work on the H2D rings".
             */
            s->pcie_h2d_doorbell_0 = data;
            apple_bcm_wlan_h2d_doorbell(s);
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
            /*
             * Taking the ARM core out of reset is the last step of the
             * firmware download: loadChipImage() ends with
             * IOCTRL = 3, RESETCTRL = 0, IOCTRL = 1. Only the final write
             * counts -- the earlier IOCTRL = 0x21 has bit 5 set (still
             * clocking up), and IOCTRL = 3 arrives while reset is still
             * asserted.
             */
            if (index == BCM_BACKPLANE_ARM_WRAPPER_INDEX &&
                (data & 0x21) == 1 && (s->wrapper_resetctrl[index] & 1) == 0) {
                apple_bcm_wlan_publish_shared_info(s);
            }
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

/*
 * Stand in for the firmware's "I am up" announcement.
 *
 * loadChipImage() releases the ARM core and then polls the last word of chip
 * RAM for ~4.8 s, waiting for it to become something other than 0xFFFFFFFF or
 * the token it wrote there itself (@0xfffffff0095c8eec). A real chip gets
 * there by running the firmware we just swallowed; we synthesize the outcome
 * instead -- we never execute a single instruction of Apple's image.
 *
 * The driver then bounds-checks the pointer (inside ChipMemoryID 4, 4-byte
 * aligned, at least 0x78 bytes of room), reads the shared structure and
 * follows rings_info_ptr, so all three structures have to sit in chip RAM.
 * BAR2 is a plain RAM region mapped identity-style onto dongle addresses, so
 * we can simply build them in place.
 */
static void apple_bcm_wlan_publish_shared_info(AppleBCMWLANDeviceState *s)
{
    uint8_t *ram = memory_region_get_ram_ptr(&s->bar2);

    QEMU_BUILD_BUG_ON(BCM_SHARED_INFO_ADDR % 4 != 0);
    QEMU_BUILD_BUG_ON(BCM_SHARED_INFO_ADDR < BCM_MEM_FW_RAM_BASE);
    QEMU_BUILD_BUG_ON(BCM_SHARED_INFO_ADDR + BCM_SHARED_SIZE >
                      BCM_MEM_FW_RAM_END);
    QEMU_BUILD_BUG_ON(BCM_RINGMEM_ADDR +
                          BCM_RINGMEM_MAX_RINGS * BCM_RINGMEM_ENTRY_SIZE >
                      BCM_MEM_FW_RAM_END);
    QEMU_BUILD_BUG_ON(BCM_MEM_FW_RAM_END > APPLE_BCM_WLAN_DEVICE_BAR2_SIZE);

    memset(ram + BCM_SHARED_INFO_ADDR, 0, BCM_SHARED_SIZE);
    stl_le_p(ram + BCM_SHARED_INFO_ADDR + BCM_SHARED_FLAGS_OFF,
             BCM_SHARED_FLAGS);
    stl_le_p(ram + BCM_SHARED_INFO_ADDR + BCM_SHARED_FLAGS2_OFF,
             BCM_SHARED_FLAGS2);
    stl_le_p(ram + BCM_SHARED_INFO_ADDR + BCM_SHARED_RINGS_INFO_PTR_OFF,
             BCM_RING_INFO_ADDR);

    memset(ram + BCM_RING_INFO_ADDR, 0, BCM_RING_INFO_SIZE);
    stl_le_p(ram + BCM_RING_INFO_ADDR + BCM_RING_INFO_RINGMEM_PTR_OFF,
             BCM_RINGMEM_ADDR);
    stw_le_p(ram + BCM_RING_INFO_ADDR + BCM_RING_INFO_MAX_TX_FLOWRINGS_OFF,
             BCM_MAX_TX_FLOWRINGS);
    stw_le_p(ram + BCM_RING_INFO_ADDR + BCM_RING_INFO_MAX_SUBMISSION_QUEUES_OFF,
             BCM_MAX_SUBMISSION_QUEUES);

    memset(ram + BCM_RINGMEM_ADDR,
           0, BCM_RINGMEM_MAX_RINGS * BCM_RINGMEM_ENTRY_SIZE);

    /* A fresh firmware means fresh rings. */
    s->rings_discovered = false;
    memset(&s->h2d_ctrl, 0, sizeof(s->h2d_ctrl));
    memset(&s->d2h_ctrl, 0, sizeof(s->d2h_ctrl));
    s->ioctl_resp_head = s->ioctl_resp_count = 0;
    s->event_head = s->event_count = 0;

    /* Last: hand the driver the pointer it is spinning on. */
    stl_le_p(ram + BCM_SHARED_INFO_PTR_ADDR, BCM_SHARED_INFO_ADDR);

    qemu_log_mask(LOG_UNIMP,
                  "%s: firmware released, published pciedev_shared_t @ 0x%x\n",
                  __func__, BCM_SHARED_INFO_ADDR);
}

/*
 * ============================ msgbuf ring engine ============================
 *
 * Three memories are in play and it is easy to mix them up:
 *
 *   - the ring *descriptors* (ring_info_t and the ring_mem_t array) live in
 *     chip RAM, i.e. inside BAR2, and the host writes them over MMIO. We read
 *     them straight out of the backing RAM.
 *   - the ring *items* live in host memory, at IOVAs the host puts in the
 *     descriptors. Reached by DMA through the DART.
 *   - the ring *indices* also live in host memory, as four arrays of u32 (one
 *     entry per ring, 4-byte stride -- see AppleBCMWLANPCIeCompletionRing::
 *     initWithOptions @0xfffffff0095ebf50, which computes each ring's index
 *     pointer as arraybase + slot * 4). Also reached by DMA.
 */

static uint8_t *apple_bcm_wlan_tcm(AppleBCMWLANDeviceState *s, uint32_t addr,
                                   uint32_t len)
{
    if (addr < BCM_MEM_FW_RAM_BASE || addr + len > BCM_MEM_FW_RAM_END) {
        return NULL;
    }
    return memory_region_get_ram_ptr(&s->bar2) + addr;
}

static bool apple_bcm_wlan_read_index(AppleBCMWLANDeviceState *s,
                                      uint64_t array, unsigned slot,
                                      uint32_t *out)
{
    uint8_t buf[4];

    if (array == 0) {
        return false;
    }
    if (!apple_bcm_wlan_dma_read(s, array + slot * 4, sizeof(buf), buf)) {
        return false;
    }
    *out = ldl_le_p(buf);
    return true;
}

static void apple_bcm_wlan_write_index(AppleBCMWLANDeviceState *s,
                                       uint64_t array, unsigned slot,
                                       uint32_t val)
{
    uint8_t buf[4];

    if (array == 0) {
        return;
    }
    stl_le_p(buf, val);
    apple_bcm_wlan_dma_write(s, array + slot * 4, sizeof(buf), buf);
}

/* Pick the ring_mem_t entry out of chip RAM and sanity-check it. */
static bool apple_bcm_wlan_load_ring(AppleBCMWLANDeviceState *s,
                                     uint32_t ringmem_ptr, unsigned slot,
                                     AppleBCMWLANRing *ring)
{
    const uint8_t *e =
        apple_bcm_wlan_tcm(s, ringmem_ptr + slot * BCM_RINGMEM_ENTRY_SIZE,
                           BCM_RINGMEM_ENTRY_SIZE);

    if (e == NULL) {
        return false;
    }

    ring->slot = slot;
    ring->id = lduw_le_p(e + BCM_RINGMEM_IDX_OFF);
    ring->type = e[BCM_RINGMEM_TYPE_OFF];
    ring->max_item = lduw_le_p(e + BCM_RINGMEM_MAX_ITEM_OFF);
    ring->len_items = lduw_le_p(e + BCM_RINGMEM_LEN_ITEMS_OFF);
    ring->base_addr = ldl_le_p(e + BCM_RINGMEM_BASE_ADDR_OFF) |
                      ((uint64_t)ldl_le_p(e + BCM_RINGMEM_BASE_ADDR_OFF + 4)
                       << 32);
    ring->valid = ring->base_addr != 0 && ring->max_item != 0 &&
                  ring->len_items != 0;
    return ring->valid;
}

/*
 * Learn the ring layout from what the host wrote into ring_info.
 *
 * The two rings we need are found by ring TYPE rather than by a hardcoded slot,
 * because the slot numbering is the host's business.
 *
 * Note the two numbering spaces. The descriptor's position in the ringmem array
 * is one thing (iOS lays out: 0 H2D control submit, 1 H2D RX post, 2 D2H control
 * complete, 3 D2H TX complete, 4 D2H RX complete, then the TX flow rings), and
 * the ring's own id -- which is what indexes the H2D and D2H index arrays, and
 * restarts from 0 for the D2H set -- is another.
 */
static void apple_bcm_wlan_discover_rings(AppleBCMWLANDeviceState *s)
{
    const uint8_t *ri = apple_bcm_wlan_tcm(s, BCM_RING_INFO_ADDR,
                                           BCM_RING_INFO_SIZE);
    uint32_t ringmem_ptr;
    unsigned nrings, slot;

    if (ri == NULL) {
        return;
    }

    ringmem_ptr = ldl_le_p(ri + BCM_RING_INFO_RINGMEM_PTR_OFF);
    s->h2d_w_idx_addr = ldq_le_p(ri + BCM_RING_INFO_H2D_W_IDX_HOSTADDR_OFF);
    s->h2d_r_idx_addr = ldq_le_p(ri + BCM_RING_INFO_H2D_R_IDX_HOSTADDR_OFF);
    s->d2h_w_idx_addr = ldq_le_p(ri + BCM_RING_INFO_D2H_W_IDX_HOSTADDR_OFF);
    s->d2h_r_idx_addr = ldq_le_p(ri + BCM_RING_INFO_D2H_R_IDX_HOSTADDR_OFF);

    nrings = lduw_le_p(ri + BCM_RING_INFO_MAX_SUBMISSION_QUEUES_OFF) +
             lduw_le_p(ri + BCM_RING_INFO_MAX_COMPLETION_RINGS_OFF);
    if (nrings == 0 || nrings > BCM_RINGMEM_MAX_RINGS) {
        nrings = BCM_RINGMEM_MAX_RINGS;
    }

    qemu_log_mask(LOG_UNIMP,
                  "%s: ring_info ringmem 0x%x h2d_w 0x%" PRIx64 " h2d_r 0x%"
                  PRIx64 " d2h_w 0x%" PRIx64 " d2h_r 0x%" PRIx64
                  " flowrings %u subq %u compl %u\n",
                  __func__, ringmem_ptr, s->h2d_w_idx_addr, s->h2d_r_idx_addr,
                  s->d2h_w_idx_addr, s->d2h_r_idx_addr,
                  lduw_le_p(ri + BCM_RING_INFO_MAX_TX_FLOWRINGS_OFF),
                  lduw_le_p(ri + BCM_RING_INFO_MAX_SUBMISSION_QUEUES_OFF),
                  lduw_le_p(ri + BCM_RING_INFO_MAX_COMPLETION_RINGS_OFF));

    for (slot = 0; slot < nrings; slot++) {
        AppleBCMWLANRing ring = { 0 };

        if (!apple_bcm_wlan_load_ring(s, ringmem_ptr, slot, &ring)) {
            continue;
        }

        qemu_log_mask(LOG_UNIMP,
                      "%s: ring slot %u id %u type %u max_item %u len_items %u "
                      "base 0x%" PRIx64 "\n",
                      __func__, slot, ring.id, ring.type, ring.max_item,
                      ring.len_items, ring.base_addr);

        switch (ring.type) {
        case BCM_RING_TYPE_H2D_CTRL_SUBMIT:
            s->h2d_ctrl = ring;
            break;
        case BCM_RING_TYPE_D2H_CTRL_COMPLETE:
            s->d2h_ctrl = ring;
            /*
             * The host expects the first item it ever reads out of a D2H ring
             * to carry phase 1 (initWithOptions stores 1 at ring+0x80).
             */
            s->d2h_ctrl.phase = 1;
            break;
        default:
            break;
        }
    }

    s->rings_discovered = s->h2d_ctrl.valid && s->d2h_ctrl.valid;
    if (!s->rings_discovered) {
        qemu_log_mask(LOG_UNIMP,
                      "%s: control rings not (yet) described: h2d %d d2h %d\n",
                      __func__, s->h2d_ctrl.valid, s->d2h_ctrl.valid);
    }
}

/* Tell the host a D2H ring grew: sticky MAILBOXINT bit plus the single MSI. */
static void apple_bcm_wlan_signal_d2h(AppleBCMWLANDeviceState *s)
{
    PCIDevice *pci_dev = PCI_DEVICE(s);

    s->pcie_mailboxint |= BCM_PCIE2_MAILBOXINT_D2H_DB0;

    if ((s->pcie_mailboxint & s->pcie_mailboxmask) == 0) {
        return;
    }

    if (msi_enabled(pci_dev)) {
        msi_notify(pci_dev, 0);
    } else {
        pci_set_irq(pci_dev, 1);
    }
}

/*
 * Append one item to the D2H control completion ring.
 *
 * The write index is ours; the host reads it out of the d2h_w index array. The
 * phase bit flips on every wrap, matching ::requestRingDrain.
 */
static void apple_bcm_wlan_d2h_ctrl_post(AppleBCMWLANDeviceState *s,
                                         uint8_t *item)
{
    AppleBCMWLANRing *ring = &s->d2h_ctrl;
    uint32_t read_index;

    if (!ring->valid || ring->len_items < BCM_D2H_CTRL_ITEM_SIZE) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: no D2H control ring\n", __func__);
        return;
    }

    /* Refuse to overrun the host: leave one slot free as the full marker. */
    if (apple_bcm_wlan_read_index(s, s->d2h_r_idx_addr, ring->id,
                                  &read_index) &&
        (ring->index + 1) % ring->max_item == read_index % ring->max_item) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: D2H control ring full\n", __func__);
        return;
    }

    if (ring->phase) {
        item[BCM_MSGBUF_HDR_FLAGS_OFF] |= BCM_MSGBUF_FLAG_PHASE;
    }

    apple_bcm_wlan_dma_write(s, ring->base_addr + ring->index * ring->len_items,
                             BCM_D2H_CTRL_ITEM_SIZE, item);

    ring->index++;
    if (ring->index >= ring->max_item) {
        ring->index = 0;
        ring->phase ^= 1;
    }

    apple_bcm_wlan_write_index(s, s->d2h_w_idx_addr, ring->id, ring->index);
    apple_bcm_wlan_signal_d2h(s);
}

/*
 * The ioctl dispatcher.
 *
 * `in` is the request payload the host DMA'd in (for GET_VAR/SET_VAR: the
 * NUL-terminated iovar name followed by its value), `out` is the buffer we may
 * fill with a response. Returns a BCME status; BCME_UNSUPPORTED is the correct
 * answer for anything we do not model, and the driver is written to cope with
 * it for every command but one.
 */
typedef int (*AppleBCMWLANCommandFn)(AppleBCMWLANDeviceState *s,
                                     const void *arg, const uint8_t *in,
                                     uint16_t inlen, uint8_t *out,
                                     uint16_t outmax, uint16_t *outlen);

/* Succeed with an empty reply. The natural answer to a configuration write. */
static int apple_bcm_wlan_cmd_ok(AppleBCMWLANDeviceState *s, const void *arg,
                                 const uint8_t *in, uint16_t inlen,
                                 uint8_t *out, uint16_t outmax,
                                 uint16_t *outlen)
{
    return BCME_OK;
}

/* Succeed with a constant 32-bit reply taken from the command table. */
static int apple_bcm_wlan_cmd_u32(AppleBCMWLANDeviceState *s, const void *arg,
                                  const uint8_t *in, uint16_t inlen,
                                  uint8_t *out, uint16_t outmax,
                                  uint16_t *outlen)
{
    if (outmax < sizeof(uint32_t)) {
        return BCME_BUFTOOSHORT;
    }
    stl_le_p(out, *(const uint32_t *)arg);
    *outlen = sizeof(uint32_t);
    return BCME_OK;
}

/* Succeed with a constant NUL-terminated string from the command table. */
static int apple_bcm_wlan_cmd_string(AppleBCMWLANDeviceState *s,
                                     const void *arg, const uint8_t *in,
                                     uint16_t inlen, uint8_t *out,
                                     uint16_t outmax, uint16_t *outlen)
{
    size_t len = strlen(arg) + 1;

    if (len > outmax) {
        return BCME_BUFTOOSHORT;
    }
    memcpy(out, arg, len);
    *outlen = len;
    return BCME_OK;
}

/*
 * The firmware version banner.
 *
 * This is the one reply AppleBCMWLANCore::setupFirmware refuses to proceed
 * without ("iovar get version command failed"); it logs the string verbatim as
 * "Firmware Version:". Real Broadcom firmware answers with this exact shape --
 * an interface tag, a build timestamp, a dotted version and an FWID.
 */
#define APPLE_BCM_WLAN_FW_VERSION \
    "wl0: Jan  1 2020 00:00:00 version 18.20.244.5 FWID 01-deadbeef"

/*
 * CLM (regulatory database) version banner, in the multi-line "key: value"
 * form real firmware returns. setupFirmware logs it and, on failure, treats
 * the error as the outcome of the whole regulatory download.
 */
#define APPLE_BCM_WLAN_CLM_VERSION           \
    "API: 12.2\nData: 9.10.39\nCompiler: 1.29.4\n" \
    "ClmImport: 1.36.3\nCustomization: v1\nCreation: 2020-01-01 00:00:00\n"

/* Same idea for the TX-power-cap database. */
#define APPLE_BCM_WLAN_TXCAP_VERSION \
    "TxCap API: 1.0\nData: 1.0.0\nCreation: 2020-01-01 00:00:00\n"

/*
 * Chip capability list.
 *
 * AppleBCMWLANCore::processChipCaps (@0xfffffff009472460) reads this into a
 * 1 KiB buffer and then just runs ::findWord over it for each of the 55 names
 * in its allCaps table (@0xfffffff0079acee0), setting or clearing a feature
 * bit per hit -- nothing here is mandatory, and a name we leave out simply
 * leaves its feature off. Only the *command* has to succeed; failing it aborts
 * bring-up with "iovar get cap command failed".
 *
 * So this is deliberately a conservative, plausible BCM4378 list: the basics
 * plus dual band and management-frame protection, and none of the exotic
 * features (RSDB, time sync, scan core, 802.11ax) whose bits would send the
 * driver down paths this model does not implement.
 */
#define APPLE_BCM_WLAN_CAPABILITIES                                     \
    "ap sta wme 802.11d 802.11h 802.11n dualband ampdu ampdu_tx "       \
    "ampdu_rx amsdurx amsdutx wep tkip aes wpa wpa2 psk mfp"

static const uint32_t apple_bcm_wlan_zero = 0;
/*
 * Scan "home away time", in milliseconds: how long the radio may spend off the
 * home channel during a scan while associated.
 *
 * AppleBCMWLANScanManager::initDefaultScanParametersFromChip
 * (@0xfffffff009506024) asks for it with a CommandRxExpected of {min 4, max 4},
 * i.e. it insists on exactly a u32, stores it verbatim at scanMgr+0x8bc and
 * logs it ("Setting Scan Home away time to %u"). It is the LAST thing
 * AppleBCMWLANCore::setupDriver does, and the only unanswered command left in
 * that path -- failing it gives "setupDriver@5161: Error: Failure to get
 * default Home Away Time" and then "loadAndSetup@4963: setupDriver fail".
 * Nothing validates the value, so use Broadcom's usual 100 ms default.
 */
static const uint32_t apple_bcm_wlan_scan_home_away_time = 100;
/*
 * WLC ioctl interface version. AppleBCMWLANCore::updateFWAPIVerFromHW
 * (@0xfffffff0094750e0) pre-seeds its 4-byte result buffer with 1 and does not
 * validate what comes back, so 1 is the value the driver would have assumed
 * anyway -- but the *command* must succeed or setupFirmware gives up.
 */
static const uint32_t apple_bcm_wlan_ioctl_version = 1;

/*
 * The regulatory domains we claim to know about.
 *
 * "X0" is Broadcom's worldwide/generic domain; the rest are ordinary ISO
 * country codes. Each entry occupies a fixed 4-byte slot.
 */
#define APPLE_BCM_WLAN_CCODE_SIZE 4
static const char *const apple_bcm_wlan_country_codes[] = {
    "X0", "US", "CA", "GB", "DE", "FR", "JP", "AU", "CN", "KR",
};

/*
 * WLC_GET_COUNTRY_LIST, wl_country_list_t. AppleBCMWLANCore::
 * handleGetCountryListAsyncCallBack (@0xfffffff0094def1c) reads the count at
 * +0x0C (clamped to 256) and then copies that many 4-byte codes from +0x10.
 */
static int apple_bcm_wlan_cmd_country_list(AppleBCMWLANDeviceState *s,
                                           const void *arg, const uint8_t *in,
                                           uint16_t inlen, uint8_t *out,
                                           uint16_t outmax, uint16_t *outlen)
{
    size_t count = ARRAY_SIZE(apple_bcm_wlan_country_codes);
    size_t len = 0x10 + count * APPLE_BCM_WLAN_CCODE_SIZE;
    size_t i;

    if (len > outmax) {
        return BCME_BUFTOOSHORT;
    }

    memset(out, 0, len);
    stl_le_p(out, len); // buflen
    stl_le_p(out + 0x0C, count);
    for (i = 0; i < count; i++) {
        strncpy((char *)out + 0x10 + i * APPLE_BCM_WLAN_CCODE_SIZE,
                apple_bcm_wlan_country_codes[i], APPLE_BCM_WLAN_CCODE_SIZE);
    }
    *outlen = len;
    return BCME_OK;
}

/*
 * WLC_GET_COUNTRY. The full reply is a wl_country_t -- abbreviation, regulatory
 * revision, underlying country code -- but the driver also asks for it with a
 * 4-byte buffer when all it wants is the abbreviation, so answer whichever
 * fits rather than failing the short form.
 */
static int apple_bcm_wlan_cmd_country(AppleBCMWLANDeviceState *s,
                                      const void *arg, const uint8_t *in,
                                      uint16_t inlen, uint8_t *out,
                                      uint16_t outmax, uint16_t *outlen)
{
    size_t len = outmax >= 12 ? 12 : APPLE_BCM_WLAN_CCODE_SIZE;

    if (outmax < len) {
        return BCME_BUFTOOSHORT;
    }
    memset(out, 0, len);
    strncpy((char *)out, arg, APPLE_BCM_WLAN_CCODE_SIZE);
    if (len == 12) {
        strncpy((char *)out + 8, arg, APPLE_BCM_WLAN_CCODE_SIZE);
    }
    *outlen = len;
    return BCME_OK;
}

/*
 * The channels we claim to support: the 2.4 GHz band plus the usual
 * non-DFS 5 GHz channels. Reported as a wl_uint32_list_t -- a count followed
 * by that many 32-bit entries.
 */
static const uint8_t apple_bcm_wlan_channels_2g[] = { 1, 2, 3, 4,  5, 6,
                                                      7, 8, 9, 10, 11 };
static const uint8_t apple_bcm_wlan_channels_5g[] = { 36,  40,  44,  48, 149,
                                                      153, 157, 161, 165 };

/*
 * chanspec_t, the D11AC encoding used by every part from the 4350 onwards:
 * the channel number in the low byte, the bandwidth and (for wide channels)
 * the position of the control sub-band in the middle, and the band on top.
 * A plain 20 MHz channel needs no sub-band.
 */
#define BCM_CHANSPEC_CHAN_MASK 0x00FF
#define BCM_CHANSPEC_BW_20 0x1000
#define BCM_CHANSPEC_BAND_2G 0x0000
#define BCM_CHANSPEC_BAND_5G 0xC000

#define BCM_CHANSPEC_2G(ch) (BCM_CHANSPEC_BAND_2G | BCM_CHANSPEC_BW_20 | (ch))
#define BCM_CHANSPEC_5G(ch) (BCM_CHANSPEC_BAND_5G | BCM_CHANSPEC_BW_20 | (ch))

/* The channel the fake access point beacons on. */
#define APPLE_BCM_WLAN_AP_CHANNEL 6

static int apple_bcm_wlan_cmd_u32_list(AppleBCMWLANDeviceState *s,
                                       const uint32_t *list, size_t count,
                                       uint8_t *out, uint16_t outmax,
                                       uint16_t *outlen)
{
    size_t i;

    if (outmax < sizeof(uint32_t) * (count + 1)) {
        return BCME_BUFTOOSHORT;
    }
    stl_le_p(out, count);
    for (i = 0; i < count; i++) {
        stl_le_p(out + sizeof(uint32_t) * (i + 1), list[i]);
    }
    *outlen = sizeof(uint32_t) * (count + 1);
    return BCME_OK;
}

/* WLC_GET_VALID_CHANNELS: plain channel numbers. */
static int apple_bcm_wlan_cmd_channels(AppleBCMWLANDeviceState *s,
                                       const void *arg, const uint8_t *in,
                                       uint16_t inlen, uint8_t *out,
                                       uint16_t outmax, uint16_t *outlen)
{
    uint32_t list[ARRAY_SIZE(apple_bcm_wlan_channels_2g) +
                  ARRAY_SIZE(apple_bcm_wlan_channels_5g)];
    size_t n = 0, i;

    for (i = 0; i < ARRAY_SIZE(apple_bcm_wlan_channels_2g); i++) {
        list[n++] = apple_bcm_wlan_channels_2g[i];
    }
    for (i = 0; i < ARRAY_SIZE(apple_bcm_wlan_channels_5g); i++) {
        list[n++] = apple_bcm_wlan_channels_5g[i];
    }
    return apple_bcm_wlan_cmd_u32_list(s, list, n, out, outmax, outlen);
}

/*
 * GET_VAR "chanspecs": the same set, but as chanspecs.
 *
 * AppleBCMWLANCore::handleGetChanSpecs (@0xfffffff0094ded5c) reads the count
 * at +0x00, clamps it to 110 and copies the LOW 16 BITS of each following u32
 * into its own table at core+0x1c5e, with the count at core+0x1c5c. Without
 * this the driver's channel table stays empty ("cannot get chanspecs"), which
 * leaves it with nothing to scan.
 */
static int apple_bcm_wlan_cmd_chanspecs(AppleBCMWLANDeviceState *s,
                                        const void *arg, const uint8_t *in,
                                        uint16_t inlen, uint8_t *out,
                                        uint16_t outmax, uint16_t *outlen)
{
    uint32_t list[ARRAY_SIZE(apple_bcm_wlan_channels_2g) +
                  ARRAY_SIZE(apple_bcm_wlan_channels_5g)];
    size_t n = 0, i;

    for (i = 0; i < ARRAY_SIZE(apple_bcm_wlan_channels_2g); i++) {
        list[n++] = BCM_CHANSPEC_2G(apple_bcm_wlan_channels_2g[i]);
    }
    for (i = 0; i < ARRAY_SIZE(apple_bcm_wlan_channels_5g); i++) {
        list[n++] = BCM_CHANSPEC_5G(apple_bcm_wlan_channels_5g[i]);
    }
    return apple_bcm_wlan_cmd_u32_list(s, list, n, out, outmax, outlen);
}

/*
 * Everything the emulated firmware answers.
 *
 * Anything absent from this table gets BCME_UNSUPPORTED, which the driver maps
 * to 0xE3FF8117 and tolerates for every optional feature -- so this only has to
 * grow when a failure actually blocks bring-up. `iovar` is NULL for a plain
 * WLC command and the iovar name for GET_VAR/SET_VAR; `arg` is passed to the
 * handler.
 */
static const struct {
    uint32_t cmd;
    const char *iovar;
    AppleBCMWLANCommandFn fn;
    const void *arg;
} apple_bcm_wlan_commands[] = {
    { WLC_GET_VAR, "ver", apple_bcm_wlan_cmd_string,
      APPLE_BCM_WLAN_FW_VERSION },
    /*
     * Regulatory (CLM) and TX-power-cap blob download. setupFirmware pushes the
     * .clmb / .txcb files it loaded from /usr/share/firmware/wifi down in
     * chunks, reads back a status word and then asks for the resulting
     * database version; any of those failing is reported as "Download clmb
     * failed". We swallow the blobs and describe a plausible database.
     */
    { WLC_SET_VAR, "clmload", apple_bcm_wlan_cmd_ok },
    { WLC_GET_VAR, "clmload_status", apple_bcm_wlan_cmd_u32,
      &apple_bcm_wlan_zero },
    { WLC_GET_VAR, "clmver", apple_bcm_wlan_cmd_string,
      APPLE_BCM_WLAN_CLM_VERSION },
    { WLC_SET_VAR, "txcapload", apple_bcm_wlan_cmd_ok },
    { WLC_GET_VAR, "txcapload_status", apple_bcm_wlan_cmd_u32,
      &apple_bcm_wlan_zero },
    { WLC_GET_VAR, "txcapver", apple_bcm_wlan_cmd_string,
      APPLE_BCM_WLAN_TXCAP_VERSION },
    { WLC_GET_VERSION, NULL, apple_bcm_wlan_cmd_u32,
      &apple_bcm_wlan_ioctl_version },
    { WLC_GET_VAR, "cap", apple_bcm_wlan_cmd_string,
      APPLE_BCM_WLAN_CAPABILITIES },
    /*
     * Minimum power consumption. The driver reads it, turns it off and
     * requires the read to have worked ("Iovar failure getting MPC value").
     * Report it already off.
     */
    { WLC_GET_VAR, "mpc", apple_bcm_wlan_cmd_u32, &apple_bcm_wlan_zero },
    { WLC_GET_VAR, "scan_home_away_time", apple_bcm_wlan_cmd_u32,
      &apple_bcm_wlan_scan_home_away_time },
    { WLC_GET_COUNTRY_LIST, NULL, apple_bcm_wlan_cmd_country_list },
    { WLC_GET_COUNTRY, NULL, apple_bcm_wlan_cmd_country, "US" },
    { WLC_SET_COUNTRY, NULL, apple_bcm_wlan_cmd_ok },
    { WLC_UP, NULL, apple_bcm_wlan_cmd_ok },
    { WLC_DOWN, NULL, apple_bcm_wlan_cmd_ok },
    { WLC_SET_REGULATORY, NULL, apple_bcm_wlan_cmd_ok },
    { WLC_SET_RADIO, NULL, apple_bcm_wlan_cmd_ok },
    { WLC_GET_VALID_CHANNELS, NULL, apple_bcm_wlan_cmd_channels },
    { WLC_GET_VAR, "chanspecs", apple_bcm_wlan_cmd_chanspecs },
};

static int apple_bcm_wlan_ioctl(AppleBCMWLANDeviceState *s, uint8_t ifidx,
                                uint32_t cmd, const uint8_t *in, uint16_t inlen,
                                uint8_t *out, uint16_t outmax,
                                uint16_t *outlen)
{
    const char *iovar = NULL;
    size_t i;
    int status = BCME_UNSUPPORTED;

    *outlen = 0;

    if (cmd == WLC_GET_VAR || cmd == WLC_SET_VAR) {
        /*
         * A var request is the NUL-terminated iovar name followed by its value.
         * The caller over-allocated `in` by one byte and zeroed it, but a
         * malformed request with no terminator is still refused outright.
         */
        uint16_t n;

        for (n = 0; n < inlen; n++) {
            if (in[n] == '\0') {
                iovar = (const char *)in;
                break;
            }
        }
    }

    for (i = 0; i < ARRAY_SIZE(apple_bcm_wlan_commands); i++) {
        if (apple_bcm_wlan_commands[i].cmd != cmd) {
            continue;
        }
        if (apple_bcm_wlan_commands[i].iovar != NULL &&
            (iovar == NULL ||
             strcmp(apple_bcm_wlan_commands[i].iovar, iovar) != 0)) {
            continue;
        }
        status = apple_bcm_wlan_commands[i].fn(s, apple_bcm_wlan_commands[i].arg,
                                               in, inlen, out, outmax, outlen);
        break;
    }

    /*
     * A configuration write we do not model is still a write that "took": the
     * emulated firmware has no state to disagree with. Accepting them by
     * default is both what real firmware does and what keeps bring-up moving,
     * whereas a *read* of something we cannot invent must stay unsupported so
     * the driver falls back to its own default instead of believing a zero.
     */
    if (status == BCME_UNSUPPORTED && cmd == WLC_SET_VAR && iovar != NULL) {
        status = BCME_OK;
    }

    /*
     * Pad an iovar reply by the length of the iovar name.
     *
     * AppleBCMWLANCommand::complete (@0xfffffff00958f9bc) does, for GET_VAR
     * with a non-empty name and a result buffer of at most 0x7fc bytes:
     *
     *     actual = min(resp_len, packet_len - 16);
     *     if (actual > strlen(name) + 1)
     *             actual -= strlen(name) + 1;
     *     memcpy(rx.buf, payload, min(actual, rx.cap));
     *
     * i.e. it expects real firmware to report a length that still counts the
     * echoed request. It does NOT skip those bytes when copying, so the value
     * must start at offset 0 and the reported length must be that many bytes
     * LONGER. Without this every iovar reply arrives short by exactly the
     * length of its name -- which is why the firmware banner used to be logged
     * as "FWID 01-deadb".
     */
    if (status == BCME_OK && cmd == WLC_GET_VAR && iovar != NULL &&
        *outlen != 0) {
        uint16_t pad = strlen(iovar) + 1;

        if (*outlen + pad <= outmax && outmax - pad <= 0x7FC) {
            memset(out + *outlen, 0, pad);
            *outlen += pad;
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "%s: ifidx %u cmd %u (%s) iovar '%s' inlen %u outmax %u -> "
                  "%d (%u bytes)\n",
                  __func__, ifidx, cmd,
                  cmd == WLC_GET_VAR ? "GET_VAR" :
                  cmd == WLC_SET_VAR ? "SET_VAR" :
                                       "ioctl",
                  iovar ? iovar : "", inlen, outmax, status, *outlen);
    return status;
}

/* Handle one MSGBUF_TYPE_IOCTLPTR_REQ from the H2D control submit ring. */
static void apple_bcm_wlan_handle_ioctl_req(AppleBCMWLANDeviceState *s,
                                            const uint8_t *msg)
{
    uint8_t ack[BCM_D2H_CTRL_ITEM_SIZE];
    uint8_t cmplt[BCM_D2H_CTRL_ITEM_SIZE];
    g_autofree uint8_t *in = NULL;
    g_autofree uint8_t *out = NULL;
    AppleBCMWLANPostedBuf resp;
    uint32_t request_id = ldl_le_p(msg + BCM_MSGBUF_HDR_REQUEST_ID_OFF);
    uint8_t ifidx = msg[BCM_MSGBUF_HDR_IFIDX_OFF];
    uint32_t cmd = ldl_le_p(msg + BCM_IOCTL_REQ_CMD_OFF);
    uint16_t trans_id = lduw_le_p(msg + BCM_IOCTL_REQ_TRANS_ID_OFF);
    uint16_t inlen = lduw_le_p(msg + BCM_IOCTL_REQ_INPUT_LEN_OFF);
    uint16_t outlen_req = lduw_le_p(msg + BCM_IOCTL_REQ_OUTPUT_LEN_OFF);
    uint64_t in_addr = ldl_le_p(msg + BCM_IOCTL_REQ_BUF_ADDR_OFF) |
                       ((uint64_t)ldl_le_p(msg + BCM_IOCTL_REQ_BUF_ADDR_OFF + 4)
                        << 32);
    uint16_t resp_len = 0;
    uint16_t outmax;
    int status;

    /*
     * Acknowledge the request first. The ACK releases the host's request
     * buffer and moves the command from the "to send" to the "in flight"
     * queue; the completion below then finishes it.
     */
    memset(ack, 0, sizeof(ack));
    ack[BCM_MSGBUF_HDR_MSGTYPE_OFF] = BCM_MSGBUF_TYPE_IOCTLPTR_REQ_ACK;
    ack[BCM_MSGBUF_HDR_IFIDX_OFF] = ifidx;
    stl_le_p(ack + BCM_MSGBUF_HDR_REQUEST_ID_OFF, request_id);
    apple_bcm_wlan_d2h_ctrl_post(s, ack);

    if (s->ioctl_resp_count == 0) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: no ioctl response buffer posted, dropping cmd %u\n",
                      __func__, cmd);
        return;
    }
    resp = s->ioctl_resp_bufs[s->ioctl_resp_head];
    s->ioctl_resp_head = (s->ioctl_resp_head + 1) % BCM_MAX_POSTED_BUFS;
    s->ioctl_resp_count--;

    if (inlen != 0 && in_addr != 0) {
        in = g_malloc0(inlen + 1u);
        if (!apple_bcm_wlan_dma_read(s, in_addr, inlen, in)) {
            inlen = 0;
        }
    }

    outmax = MIN(outlen_req, resp.len);
    out = g_malloc0(outmax + 1u);

    status = apple_bcm_wlan_ioctl(s, ifidx, cmd, in, inlen, out, outmax,
                                  &resp_len);
    if (status != BCME_OK) {
        resp_len = 0;
    }
    if (resp_len != 0) {
        apple_bcm_wlan_dma_write(s, resp.addr, resp_len, out);
    }

    memset(cmplt, 0, sizeof(cmplt));
    cmplt[BCM_MSGBUF_HDR_MSGTYPE_OFF] = BCM_MSGBUF_TYPE_IOCTL_CMPLT;
    cmplt[BCM_MSGBUF_HDR_IFIDX_OFF] = ifidx;
    /* The completion is matched to the *response buffer*, not to the request. */
    stl_le_p(cmplt + BCM_MSGBUF_HDR_REQUEST_ID_OFF, resp.request_id);
    stw_le_p(cmplt + BCM_IOCTL_CMPLT_STATUS_OFF, (uint16_t)(int16_t)status);
    stw_le_p(cmplt + BCM_IOCTL_CMPLT_RESP_LEN_OFF, resp_len);
    stw_le_p(cmplt + BCM_IOCTL_CMPLT_TRANS_ID_OFF, trans_id);
    stl_le_p(cmplt + BCM_IOCTL_CMPLT_CMD_OFF, cmd);
    apple_bcm_wlan_d2h_ctrl_post(s, cmplt);
}

/*
 * Acknowledge a dynamic ring creation.
 *
 * The driver tracks the request by the common header's request_id (the
 * completion handlers at 0xfffffff0095cf9a8 and 0xfffffff0095d01f0 look it up
 * in a hash table and then require the u16 status at +0x08 to be zero), so an
 * echo of the request id plus a zero status is a complete answer. Nothing else
 * is needed: these rings carry firmware debug logs, which we never produce.
 */
static void apple_bcm_wlan_ack_ring_create(AppleBCMWLANDeviceState *s,
                                           const uint8_t *msg, uint8_t type)
{
    uint8_t cmplt[BCM_D2H_CTRL_ITEM_SIZE];

    memset(cmplt, 0, sizeof(cmplt));
    cmplt[BCM_MSGBUF_HDR_MSGTYPE_OFF] = type;
    cmplt[BCM_MSGBUF_HDR_IFIDX_OFF] = msg[BCM_MSGBUF_HDR_IFIDX_OFF];
    stl_le_p(cmplt + BCM_MSGBUF_HDR_REQUEST_ID_OFF,
             ldl_le_p(msg + BCM_MSGBUF_HDR_REQUEST_ID_OFF));
    apple_bcm_wlan_d2h_ctrl_post(s, cmplt);
}

static void apple_bcm_wlan_post_buf(AppleBCMWLANPostedBuf *pool, unsigned head,
                                    unsigned *count, const uint8_t *msg)
{
    AppleBCMWLANPostedBuf *slot;

    if (*count >= BCM_MAX_POSTED_BUFS) {
        return;
    }
    slot = &pool[(head + *count) % BCM_MAX_POSTED_BUFS];
    slot->request_id = ldl_le_p(msg + BCM_MSGBUF_HDR_REQUEST_ID_OFF);
    slot->len = lduw_le_p(msg + BCM_BUF_POST_HOST_BUF_LEN_OFF);
    slot->addr = ldl_le_p(msg + BCM_BUF_POST_HOST_BUF_ADDR_OFF) |
                 ((uint64_t)ldl_le_p(msg + BCM_BUF_POST_HOST_BUF_ADDR_OFF + 4)
                  << 32);
    (*count)++;
}

/* Drain everything the host queued on the H2D control submit ring. */
static void apple_bcm_wlan_h2d_doorbell(AppleBCMWLANDeviceState *s)
{
    AppleBCMWLANRing *ring = &s->h2d_ctrl;
    uint8_t msg[BCM_H2D_CTRL_ITEM_SIZE];
    uint32_t write_index;
    unsigned guard;

    if (!s->rings_discovered) {
        apple_bcm_wlan_discover_rings(s);
        if (!s->rings_discovered) {
            return;
        }
    }

    if (!apple_bcm_wlan_read_index(s, s->h2d_w_idx_addr, ring->id,
                                   &write_index)) {
        return;
    }
    if (write_index >= ring->max_item) {
        qemu_log_mask(LOG_GUEST_ERROR, "%s: bogus H2D write index %u\n",
                      __func__, write_index);
        return;
    }

    for (guard = 0; ring->index != write_index && guard < ring->max_item;
         guard++) {
        if (!apple_bcm_wlan_dma_read(s,
                                     ring->base_addr +
                                         ring->index * ring->len_items,
                                     MIN(ring->len_items, sizeof(msg)), msg)) {
            break;
        }

        switch (msg[BCM_MSGBUF_HDR_MSGTYPE_OFF]) {
        case BCM_MSGBUF_TYPE_IOCTLRESP_BUF_POST:
            apple_bcm_wlan_post_buf(s->ioctl_resp_bufs, s->ioctl_resp_head,
                                    &s->ioctl_resp_count, msg);
            break;
        case BCM_MSGBUF_TYPE_EVENT_BUF_POST:
            apple_bcm_wlan_post_buf(s->event_bufs, s->event_head,
                                    &s->event_count, msg);
            break;
        case BCM_MSGBUF_TYPE_IOCTLPTR_REQ:
            apple_bcm_wlan_handle_ioctl_req(s, msg);
            break;
        case BCM_MSGBUF_TYPE_H2D_RING_CREATE:
            apple_bcm_wlan_ack_ring_create(
                s, msg, BCM_MSGBUF_TYPE_H2D_RING_CREATE_CMPLT);
            break;
        case BCM_MSGBUF_TYPE_D2H_RING_CREATE:
            apple_bcm_wlan_ack_ring_create(
                s, msg, BCM_MSGBUF_TYPE_D2H_RING_CREATE_CMPLT);
            break;
        default:
            qemu_log_mask(LOG_UNIMP, "%s: UNIMP H2D control message type 0x%x\n",
                          __func__, msg[BCM_MSGBUF_HDR_MSGTYPE_OFF]);
            break;
        }

        ring->index = (ring->index + 1) % ring->max_item;
    }

    /* Hand the consumed space back to the host. */
    apple_bcm_wlan_write_index(s, s->h2d_r_idx_addr, ring->id, ring->index);
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

    s->rings_discovered = false;
    s->h2d_w_idx_addr = 0;
    s->h2d_r_idx_addr = 0;
    s->d2h_w_idx_addr = 0;
    s->d2h_r_idx_addr = 0;
    memset(&s->h2d_ctrl, 0, sizeof(s->h2d_ctrl));
    memset(&s->d2h_ctrl, 0, sizeof(s->d2h_ctrl));
    s->ioctl_resp_head = s->ioctl_resp_count = 0;
    s->event_head = s->event_count = 0;

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
