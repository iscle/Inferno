/*
 * ChefKiss Kernel Patches.
 *
 * Copyright (c) 2025-2026 Visual Ehrmanntraut (VisualEhrmanntraut).
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

#ifndef HW_ARM_APPLE_SILICON_KERNEL_PATCHES_H
#define HW_ARM_APPLE_SILICON_KERNEL_PATCHES_H

#include "hw/arm/apple-silicon/boot.h"

/*
 * @root_snapshot_name: when non-NULL the device tree carries a
 * `root-snapshot-name`, so the kernel can root from the APFS snapshot and the
 * snapshot-disabling patch is skipped. Pass NULL to keep the old behaviour of
 * forcing a live-filesystem root.
 */
void ck_patch_kernel(MachoHeader64 *hdr, const char *root_snapshot_name);

#endif /* HW_ARM_APPLE_SILICON_KERNEL_PATCHES_H */
