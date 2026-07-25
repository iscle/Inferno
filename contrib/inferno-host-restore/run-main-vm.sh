#!/usr/bin/env bash
#
# run-main-vm.sh — launch the Inferno main VM (t8030 / iPhone 11) wired for a
# host-direct restore.
#
# This is the same QEMU invocation the manual documents, except the USB is left
# on the default UNIX-socket transport (/tmp/InfernoUSBRemote) so that a host
# process — inferno-usbd, from this directory — can play the USB *host* role
# that the companion VM used to play.
#
# Ordering matters: the USB host (inferno-usbd) must be listening on the socket
# BEFORE this VM starts, exactly as the companion VM had to be started first.
# Use inferno-restore.sh to get the ordering right automatically.
#
# Copyright (c) 2026 Inferno host-restore contributors.
# SPDX-License-Identifier: AGPL-3.0-or-later

set -euo pipefail

# --- configuration ---------------------------------------------------------
# Root of the "InfernoData" working folder (disks, tickets, extracted Restore).
DATA_DIR="${INFERNO_DATA:-$HOME/InfernoData}"
# Inferno build directory containing qemu-system-aarch64.
BUILD_DIR="${INFERNO_BUILD:-$HOME/Inferno/build}"
# USB transport. Defaults to the UNIX socket the host USB stack listens on.
USB_CONN_TYPE="${INFERNO_USB_CONN_TYPE:-unix}"
USB_CONN_ADDR="${INFERNO_USB_CONN_ADDR:-/tmp/InfernoUSBRemote}"
USB_CONN_PORT="${INFERNO_USB_CONN_PORT:-}"

# Device-specific asset names for iPhone 11 (n104ap / iPhone12,1), iOS 14.0b5.
BOARD="n104"
TRUSTCACHE="Restore/Firmware/038-44135-124.dmg.trustcache"
KERNEL="Restore/kernelcache.research.iphone12b"
DTB="Restore/Firmware/all_flash/DeviceTree.${BOARD}ap.im4p"
SEP_FW="sep-firmware.${BOARD}.RELEASE.new.img4"
SEP_ROM="AppleSEPROM-Cebu-B1"
TICKET="root_ticket.der"
# Erase RAM disk (the smaller of the two small dmgs) — needed for --erase restore.
RAMDISK="${INFERNO_RAMDISK:-Restore/038-44135-124.dmg}"

# Whether to attach the restore RAM disk (-initrd). Set RESTORE=0 for a normal boot.
RESTORE="${RESTORE:-1}"

QEMU="$BUILD_DIR/qemu-system-aarch64"

# --- sanity checks ---------------------------------------------------------
cd "$DATA_DIR"
[ -x "$QEMU" ] || { echo "error: $QEMU not found or not executable" >&2; exit 1; }
for f in "$TRUSTCACHE" "$KERNEL" "$DTB" "$SEP_FW" "$SEP_ROM" "$TICKET" \
         root firmware syscfg ctrl_bits nvram effaceable panic_log \
         sep_nvram sep_ssc; do
    [ -e "$f" ] || { echo "error: missing required file: $f" >&2; exit 1; }
done

# --- assemble the machine option -------------------------------------------
MACHINE="t8030"
# Optional boot mode: "exit_recovery" sets auto-boot=true in nvram and boots the
# installed OS from the restored disk (use for a normal boot after a restore);
# "enter_recovery" forces recovery. Leave unset for the machine default.
[ -n "${INFERNO_BOOT_MODE:-}" ] && MACHINE+=",boot-mode=$INFERNO_BOOT_MODE"
MACHINE+=",trustcache=$TRUSTCACHE"
MACHINE+=",ticket=$TICKET"
MACHINE+=",sep-fw=$SEP_FW"
MACHINE+=",sep-rom=$SEP_ROM"
MACHINE+=",kaslr-off=true"
case "$USB_CONN_TYPE" in
    unix)
        # default path is used when addr is omitted; pass it explicitly for clarity
        MACHINE+=",usb-conn-type=unix,usb-conn-addr=$USB_CONN_ADDR"
        ;;
    ipv4|ipv6)
        [ -n "$USB_CONN_PORT" ] || { echo "error: INFERNO_USB_CONN_PORT required for $USB_CONN_TYPE" >&2; exit 1; }
        MACHINE+=",usb-conn-type=$USB_CONN_TYPE,usb-conn-addr=$USB_CONN_ADDR,usb-conn-port=$USB_CONN_PORT"
        ;;
    *) echo "error: bad INFERNO_USB_CONN_TYPE: $USB_CONN_TYPE" >&2; exit 1;;
esac

APPEND="tlto_us=-1 mtxspin=-1 agm-genuine=1 agm-authentic=1 agm-trusted=1 serial=3 wdt=-1 -vm_compressor_wk_sw"

CMD=( "$QEMU"
    -M "$MACHINE"
    -kernel "$KERNEL"
    -dtb "$DTB"
    -append "$APPEND"
    -smp 7 -m "${INFERNO_RAM:-4G}"
    -serial mon:stdio
    -drive file=sep_nvram,if=pflash,format=raw
    -drive file=sep_ssc,if=pflash,format=raw
    -drive file=root,format=raw,if=none,id=root
        -device nvme-ns,drive=root,bus=nvme-bus.0,nsid=1,nstype=1,logical_block_size=4096,physical_block_size=4096
    -drive file=firmware,format=raw,if=none,id=firmware
        -device nvme-ns,drive=firmware,bus=nvme-bus.0,nsid=2,nstype=2,logical_block_size=4096,physical_block_size=4096
    -drive file=syscfg,format=raw,if=none,id=syscfg
        -device nvme-ns,drive=syscfg,bus=nvme-bus.0,nsid=3,nstype=3,logical_block_size=4096,physical_block_size=4096
    -drive file=ctrl_bits,format=raw,if=none,id=ctrl_bits
        -device nvme-ns,drive=ctrl_bits,bus=nvme-bus.0,nsid=4,nstype=4,logical_block_size=4096,physical_block_size=4096
    -drive file=nvram,if=none,format=raw,id=nvram
        -device apple-nvram,drive=nvram,bus=nvme-bus.0,nsid=5,nstype=5,id=nvram,logical_block_size=4096,physical_block_size=4096
    -drive file=effaceable,format=raw,if=none,id=effaceable
        -device nvme-ns,drive=effaceable,bus=nvme-bus.0,nsid=6,nstype=6,logical_block_size=4096,physical_block_size=4096
    -drive file=panic_log,format=raw,if=none,id=panic_log
        -device nvme-ns,drive=panic_log,bus=nvme-bus.0,nsid=7,nstype=8,logical_block_size=4096,physical_block_size=4096
)

# Display backend. cocoa by default (a native window). Set INFERNO_DISPLAY=none
# for headless, or INFERNO_DISPLAY=vnc to serve VNC on 127.0.0.1:5900 (connect
# with e.g. `open vnc://127.0.0.1:5900`) — useful when the VM is launched from a
# context without window-server access (a background/ssh session).
case "${INFERNO_DISPLAY:-cocoa}" in
    none) CMD+=( -display none );;
    vnc)
        # Set INFERNO_VNC_PASSWORD to require a password (recommended: macOS
        # Screen Sharing prompts for one regardless). The password itself is
        # applied after start via the monitor: `set_password vnc <pw>`.
        vnc_opts="vnc=${INFERNO_VNC_ADDR:-127.0.0.1:0}"
        [ -n "${INFERNO_VNC_PASSWORD:-}" ] && vnc_opts+=",password=on"
        CMD+=( -display "$vnc_opts" )
        ;;
    *)    CMD+=( -display cocoa,zoom-to-fit=on,zoom-interpolation=on,show-cursor=on );;
esac

if [ "$RESTORE" = "1" ]; then
    [ -e "$RAMDISK" ] || { echo "error: RESTORE=1 but RAM disk missing: $RAMDISK" >&2; exit 1; }
    CMD+=( -initrd "$RAMDISK" )
fi

# Optional extra QEMU args (e.g. "-s -monitor unix:/tmp/qmon.sock,server,nowait"
# for a gdbstub + QMP/HMP monitor to inspect a hung guest).
if [ -n "${INFERNO_QEMU_EXTRA:-}" ]; then
    # shellcheck disable=SC2206
    CMD+=( ${INFERNO_QEMU_EXTRA} )
fi

echo "+ ${CMD[*]}" >&2
exec "${CMD[@]}"
