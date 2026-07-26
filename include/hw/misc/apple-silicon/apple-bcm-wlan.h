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
 */

#ifndef HW_MISC_APPLE_SILICON_APPLE_BCM_WLAN_H
#define HW_MISC_APPLE_SILICON_APPLE_BCM_WLAN_H

#include "hw/arm/apple-silicon/dt.h"
#include "hw/pci-host/apcie.h"
#include "hw/pci/pci.h"
#include "hw/sysbus.h"

/*
 * `node` is the PCIe endpoint node (arm-io/apcie/pci-bridge2/wlan); `mac_node`
 * is the platform node (arm-io/wlan), which is where iOS takes the interface's
 * MAC address from and therefore where the emulated NIC has to take its own.
 */
SysBusDevice *apple_bcm_wlan_create(AppleDTNode *node, AppleDTNode *mac_node,
                                    PCIBus *pci_bus, ApplePCIEPort *port);

#endif /* HW_MISC_APPLE_SILICON_APPLE_BCM_WLAN_H */
