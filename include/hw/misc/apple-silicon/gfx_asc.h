/*
 * Apple GFX ASC (gfx-asc) — the RTKit coprocessor wrapper in front of the
 * G12P GPU.
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

#ifndef HW_MISC_APPLE_SILICON_GFX_ASC_H
#define HW_MISC_APPLE_SILICON_GFX_ASC_H

#include "qemu/osdep.h"
#include "hw/arm/apple-silicon/dt.h"
#include "hw/misc/apple-silicon/a7iop/rtkit.h"
#include "hw/sysbus.h"

#define TYPE_APPLE_GFX_ASC "apple-gfx-asc"
OBJECT_DECLARE_TYPE(AppleGFXASCState, AppleGFXASCClass, APPLE_GFX_ASC)

struct AppleGFXASCClass {
    /*< private >*/
    AppleRTKitClass base_class;

    /*< public >*/
    DeviceRealize parent_realize;
};

struct AppleGFXASCState {
    /*< private >*/
    AppleRTKit parent_obj;

    /*< public >*/
    MemoryRegion ascv2_iomem;
};

SysBusDevice *apple_gfx_asc_from_node(AppleDTNode *node,
                                      AppleA7IOPVersion version);

#endif /* HW_MISC_APPLE_SILICON_GFX_ASC_H */
