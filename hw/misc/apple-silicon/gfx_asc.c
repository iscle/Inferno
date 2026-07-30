/*
 * Apple GFX ASC (gfx-asc) — the RTKit coprocessor wrapper in front of the
 * G12P GPU.
 *
 * This models only the ASC wrapper, i.e. the first link of the chain
 *
 *   gfx-asc (iop,ascwrap-v2)  ->  iop-gfx-nub (iop-nub,rtbuddy-v2)
 *     ->  AGXFirmwareKextG12PRTBuddy  ->  AGXG12P  ->  sgx (gpu,t8030)
 *
 * The GPU itself (the `sgx` node's MMIO) is deliberately NOT backed by any
 * device: the point of this stage is to measure *where* the driver stops and
 * *which* register it wanted, so unmodelled accesses must fault and be logged
 * rather than be answered with invented values.
 *
 * The device tree describes gfx-asc exactly like sio/ans — compatible
 * "iop,ascwrap-v2", reg = { rtkit_base, rtkit_size, ascv2_core_base,
 * ascv2_core_size } — so this follows hw/dma/apple_sio.c.
 *
 * Copyright (c) 2026 Inferno host-restore contributors.
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

#include "qemu/osdep.h"
#include "hw/misc/apple-silicon/gfx_asc.h"
#include "hw/misc/apple-silicon/a7iop/rtkit.h"
#include "migration/vmstate.h"
#include "qapi/error.h"
#include "system/address-spaces.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "util/mlib.h"

/*
 * Every endpoint message is reported: the whole value of the first boot is the
 * transcript of what the firmware kext asks for.
 */
static void apple_gfx_asc_handle_endpoint(void *opaque, uint8_t ep,
                                          uint64_t msg)
{
    (void)opaque;

    info_report("gfx-asc: UNIMPLEMENTED user endpoint message ep=0x%X "
                "msg=0x%" PRIX64,
                ep, msg);
}

static void ascv2_core_reg_write(void *opaque, hwaddr addr, uint64_t data,
                                 unsigned size)
{
    (void)opaque;

    qemu_log_mask(LOG_UNIMP,
                  "gfx-asc: UNIMPLEMENTED ascv2-core write @ 0x" HWADDR_FMT_plx
                  " value 0x%" PRIX64 " size %u\n",
                  addr, data, size);
}

static uint64_t ascv2_core_reg_read(void *opaque, hwaddr addr, unsigned size)
{
    (void)opaque;

    qemu_log_mask(LOG_UNIMP,
                  "gfx-asc: UNIMPLEMENTED ascv2-core read @ 0x" HWADDR_FMT_plx
                  " size %u\n",
                  addr, size);

    return 0;
}

static const MemoryRegionOps ascv2_core_reg_ops = {
    .write = ascv2_core_reg_write,
    .read = ascv2_core_reg_read,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 8,
    .valid.min_access_size = 4,
    .valid.max_access_size = 8,
    .valid.unaligned = false,
};

/*
 * microPPL -- the GPU firmware's half of the handoff page.
 *
 * The handoff page is a two-sided contract, and XNU's PPL is the other side of
 * it: PPL carries the GPU's own address translator (uat_init, uat_map,
 * uat_unmap, uat_get_leaf_table, uat_ioctl) and treats the GPU firmware as a
 * peer that owns the same structure. Both halves are legible from PPL's code:
 *
 *   +0x00  u64  0x4B1D000000000002   written by uat_init  (the AP's magic)
 *   +0x08  u64  0x4B1D000000000002   written by the firmware -- microPPL
 *   +0x10  u8   AP wants the lock    +0x11  u8  firmware holds the lock
 *   +0x14  u32  handshake word       +0x18  u32 = -1
 *   +0x20 + ctx * 0x18, 65 contexts: u32 flush_state, then two u64s
 *   +0x638 u8   set by uat_init when +0x08 did not hold the magic
 *
 * Only three of those are ours, and all three are satisfied by doing nothing
 * beyond publishing the magic:
 *
 *   +0x08  uat_unmap _panics_ unless it reads the magic ("Invalid microPPL
 *          magic value found in the handoff region"). This is the one thing
 *          the firmware must actively do.
 *   +0x11  uat_init's lock acquire is `[+0x10] = 1; if ([+0x11] == 0) -> got
 *          it`, with a back-off loop on [+0x14] otherwise. A firmware that
 *          never contends leaves [+0x11] at 0 and the AP always wins
 *          immediately, which is correct rather than merely convenient: there
 *          is no concurrent firmware here to serialise against.
 *   flush_state  uat_perform_unmap records &handoff.ctx[id].flush_state when
 *          it retires a firmware-writeable cached PTE, and update_mappings
 *          then panics if it reads 1 ("Attempted to call unmap before
 *          completing a TTBR%d cache flush"); ioctl_remove_ctx_id likewise
 *          requires 0. Both are the "flush already finished" case, which is
 *          what a coprocessor with no cache of its own genuinely is.
 *
 * Note that publishing the magic *enables* those last two checks: uat_init
 * sets +0x638 when the magic is missing, and update_mappings skips the
 * flush-completion check whenever +0x638 is set. So this is not just quieting
 * one panic, it opts into the real protocol.
 */
#define GFX_HANDOFF_MICROPPL_MAGIC_OFF (0x8)
#define GFX_HANDOFF_MICROPPL_MAGIC (0x4B1D000000000002ULL)

static void apple_gfx_asc_start(void *opaque)
{
    AppleGFXASCState *s;
    MemTxResult res;

    s = APPLE_GFX_ASC(opaque);

    if (s->handoff_base == 0) {
        /* No carveout published, i.e. the `gpu` machine property is off. */
        return;
    }

    res = MEMTX_OK;
    address_space_stq_le(&address_space_memory,
                         s->handoff_base + GFX_HANDOFF_MICROPPL_MAGIC_OFF,
                         GFX_HANDOFF_MICROPPL_MAGIC, MEMTXATTRS_UNSPECIFIED,
                         &res);
    if (res != MEMTX_OK) {
        error_report("gfx-asc: failed to publish the microPPL magic at 0x%" PRIX64,
                     s->handoff_base + GFX_HANDOFF_MICROPPL_MAGIC_OFF);
        return;
    }

    info_report("gfx-asc: microPPL up, magic 0x%" PRIX64 " at 0x%" PRIX64,
                GFX_HANDOFF_MICROPPL_MAGIC,
                s->handoff_base + GFX_HANDOFF_MICROPPL_MAGIC_OFF);
}

static const AppleRTKitOps apple_gfx_asc_rtkit_ops = {
    .start = apple_gfx_asc_start,
};

void apple_gfx_asc_set_handoff_base(AppleGFXASCState *s, uint64_t base)
{
    s->handoff_base = base;
}

static void apple_gfx_asc_realize(DeviceState *dev, Error **errp)
{
    AppleGFXASCClass *gc;

    gc = APPLE_GFX_ASC_GET_CLASS(dev);

    if (gc->parent_realize != NULL) {
        gc->parent_realize(dev, errp);
    }

    info_report("gfx-asc: realized");
}

static const VMStateDescription vmstate_apple_gfx_asc = {
    .name = TYPE_APPLE_GFX_ASC,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields =
        (const VMStateField[]){
            VMSTATE_APPLE_RTKIT(parent_obj, AppleGFXASCState),
            VMSTATE_UINT64(handoff_base, AppleGFXASCState),
            VMSTATE_END_OF_LIST(),
        },
};

static void apple_gfx_asc_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc;
    AppleGFXASCClass *gc;

    (void)data;

    dc = DEVICE_CLASS(klass);
    gc = APPLE_GFX_ASC_CLASS(klass);

    device_class_set_parent_realize(dc, apple_gfx_asc_realize,
                                    &gc->parent_realize);
    dc->desc = "Apple GFX ASC (G12P RTKit coprocessor wrapper)";
    dc->user_creatable = false;
    dc->vmsd = &vmstate_apple_gfx_asc;
}

static const TypeInfo apple_gfx_asc_info = {
    .name = TYPE_APPLE_GFX_ASC,
    .parent = TYPE_APPLE_RTKIT,
    .instance_size = sizeof(AppleGFXASCState),
    .class_size = sizeof(AppleGFXASCClass),
    .class_init = apple_gfx_asc_class_init,
};

static void apple_gfx_asc_register_types(void)
{
    type_register_static(&apple_gfx_asc_info);
}

type_init(apple_gfx_asc_register_types);

SysBusDevice *apple_gfx_asc_from_node(AppleDTNode *node,
                                      AppleA7IOPVersion version)
{
    DeviceState *dev;
    AppleGFXASCState *s;
    SysBusDevice *sbd;
    AppleRTKit *rtk;
    AppleDTNode *child;
    AppleDTProp *prop;
    uint64_t *reg;

    dev = qdev_new(TYPE_APPLE_GFX_ASC);
    s = APPLE_GFX_ASC(dev);
    sbd = SYS_BUS_DEVICE(dev);
    rtk = APPLE_RTKIT(dev);

    dev->id = g_strdup("gfx-asc");

    child = apple_dt_get_node(node, "iop-gfx-nub");
    assert_nonnull(child);

    prop = apple_dt_get_prop(node, "reg");
    assert_nonnull(prop);
    assert_true(prop->len >= sizeof(uint64_t) * 4);

    reg = (uint64_t *)prop->data;

    apple_rtkit_init(rtk, s, "GFX", reg[1], version, &apple_gfx_asc_rtkit_ops);
    apple_rtkit_register_user_ep(rtk, 0, s, apple_gfx_asc_handle_endpoint);

    memory_region_init_io(&s->ascv2_iomem, OBJECT(dev), &ascv2_core_reg_ops, s,
                          TYPE_APPLE_GFX_ASC ".ascv2-core-reg", reg[3]);
    sysbus_init_mmio(sbd, &s->ascv2_iomem);

    /*
     * iBoot loads the `gfxf` RTKit image into the carveout published on the nub
     * as region-base/region-size and hands the coprocessor over already
     * running, so the nub must not try to load firmware itself. Same contract
     * as sio/ans.
     */
    apple_dt_set_prop_u32(child, "pre-loaded", 1);
    apple_dt_set_prop_u32(child, "running", 1);

    info_report("gfx-asc: created from DT, rtkit_mmio_size=0x%" PRIX64
                " ascv2_core_size=0x%" PRIX64,
                reg[1], reg[3]);

    return sbd;
}
