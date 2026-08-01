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
 */

#ifndef HW_MISC_APPLE_SILICON_APPLE_BCM_BT_H
#define HW_MISC_APPLE_SILICON_APPLE_BCM_BT_H

#include "hw/arm/apple-silicon/dt.h"
#include "hw/pci-host/apcie.h"
#include "hw/pci/pci.h"
#include "hw/sysbus.h"

/*
 * `node` is the PCIe endpoint node (arm-io/apcie/pci-bridge2/bluetooth-pcie);
 * `bt_node` is the platform node (arm-io/bluetooth), which is where iOS takes
 * the controller's BD_ADDR from.
 *
 * The endpoint is function 1 of the same PCI device as Wi-Fi -- both halves of
 * the BCM4378 combo part sit behind apcie pci-bridge2.
 */
SysBusDevice *apple_bcm_bt_create(AppleDTNode *node, AppleDTNode *bt_node,
                                  PCIBus *pci_bus, ApplePCIEPort *port);

#endif /* HW_MISC_APPLE_SILICON_APPLE_BCM_BT_H */
