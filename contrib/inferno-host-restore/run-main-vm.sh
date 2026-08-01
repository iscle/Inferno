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

# Device-specific asset names. The defaults are for iPhone 11 (n104ap /
# iPhone12,1) on iOS 14.0b5; every one of them can be overridden from the
# environment so the same script can drive a different iOS build out of a
# different INFERNO_DATA directory (the asset file names are build-specific —
# the RAM disk / OS dmg numbers and the trust cache change with every build).
BOARD="${INFERNO_BOARD:-n104}"
TRUSTCACHE="${INFERNO_TRUSTCACHE:-Restore/Firmware/038-44135-124.dmg.trustcache}"
KERNEL="${INFERNO_KERNEL:-Restore/kernelcache.research.iphone12b}"
DTB="${INFERNO_DTB:-Restore/Firmware/all_flash/DeviceTree.${BOARD}ap.im4p}"
SEP_FW="${INFERNO_SEP_FW:-sep-firmware.${BOARD}.RELEASE.new.img4}"
SEP_ROM="${INFERNO_SEP_ROM:-AppleSEPROM-Cebu-B1}"
TICKET="${INFERNO_TICKET:-root_ticket.der}"
# Erase RAM disk (the smaller of the two small dmgs) — needed for --erase restore.
RAMDISK="${INFERNO_RAMDISK:-Restore/038-44135-124.dmg}"

# Whether to attach the restore RAM disk (-initrd). Set RESTORE=0 for a normal boot.
RESTORE="${RESTORE:-1}"

# Snapshot mode. "1" appends ,snapshot=on to *every* drive, including the two SEP
# pflash devices (sep_nvram, sep_ssc), so a run leaves the device images
# untouched.
#
# It has to be all-or-nothing. Snapshotting the NAND drives but leaving the SEP
# pflash writable desynchronises the SEP irrecoverably: SEPOS rewrites the
# SEP-xART Locker into sep_ssc while the matching xART records on the (discarded)
# root and effaceable images are thrown away, and the next boot dies in
#   SEP Panic: :sks /sks
# right after "D-key effaceable locker does not exist, attempt to init it
# implicitly". The SSC keeps four identical copies of each slot, all rewritten
# together, so there is no older copy to recover from and no way back short of a
# re-restore. Zeroing sep_ssc instead trades that for
#   SEP Panic: :sars/sars
# (anti-replay rollback). Hence the check below.
INFERNO_SNAPSHOT_DRIVES="${INFERNO_SNAPSHOT_DRIVES:-0}"

# Extra options appended to the root drive, e.g. ",cache=unsafe,aio=threads".
# Measured on a 26.5 first boot after an erase restore -- the one-time filesystem
# work is I/O bound, and those two options took the same boot from 28 minutes to
# 12. Left opt-in rather than default: cache=unsafe discards flush guarantees, so
# a host crash can corrupt the image. Fine for a disposable test image, not for
# one you care about.
INFERNO_ROOT_DRIVE_OPTS="${INFERNO_ROOT_DRIVE_OPTS:-}"

# Number of vCPUs (INFERNO_SMP). One vCPU is consumed by the emulated SEP --
# t8030_real_cpu_count() is `smp.cpus - (sep_fw != NULL)` -- so the guest gets
# INFERNO_SMP-1 application cores.
#
# Only 7 and 6 work. The A13 is 4 efficiency + 2 performance cores, and
# t8030_cpu_setup() trims the device tree's `cpus` children from the end while
# always creating both clusters; at 4 application cores or fewer the performance
# cluster is left empty and XNU's per-CPU-kind accounting overruns its
# allocation, panicking early in boot with
#   [recount_track_cpu_kind]: element modified after free
# Measured on iOS 26.5: 7 and 6 boot; 5, 4, 3 and 2 all panic that way.
#
# Reducing 7 -> 6 does not help throughput: the datamigrator's starvation ratio
# (runnable vs running, from the panic stackshot) got *worse*, 5.13x -> 7.21x.

# The emulator binary. Normally the one in the build directory; INFERNO_QEMU can
# point at a copy under a different name, which is handy when several VMs share
# the host and one of them is torn down with a name-matching `pkill`.
QEMU="${INFERNO_QEMU:-$BUILD_DIR/qemu-system-aarch64}"

# --- snapshot mode ---------------------------------------------------------
case "$INFERNO_SNAPSHOT_DRIVES" in
    0) SNAP="";;
    1) SNAP=",snapshot=on";;
    *) echo "error: INFERNO_SNAPSHOT_DRIVES must be 0 or 1" >&2; exit 1;;
esac
if [ "$INFERNO_SNAPSHOT_DRIVES" != "1" ] &&
   [ "${INFERNO_ROOT_DRIVE_OPTS#*snapshot=on}" != "$INFERNO_ROOT_DRIVE_OPTS" ]; then
    echo "error: snapshot=on in INFERNO_ROOT_DRIVE_OPTS without INFERNO_SNAPSHOT_DRIVES=1." >&2
    echo "       Snapshotting some drives but not the SEP pflash ones desynchronises" >&2
    echo "       the SEP and bricks the images; use INFERNO_SNAPSHOT_DRIVES=1 instead." >&2
    exit 1
fi

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
# iOS 16+ seals the system volume and a RELEASE kernel will not root from its
# live filesystem, so the kernel needs the name of the APFS snapshot to root
# from. Read it off the restored image with `diskutil apfs listSnapshots`.
[ -n "${INFERNO_ROOT_SNAPSHOT:-}" ] && MACHINE+=",root-snapshot-name=$INFERNO_ROOT_SNAPSHOT"
MACHINE+=",trustcache=$TRUSTCACHE"
MACHINE+=",ticket=$TICKET"
MACHINE+=",sep-fw=$SEP_FW"
MACHINE+=",sep-rom=$SEP_ROM"
MACHINE+=",kaslr-off=true"
# Divide guest time by this factor (see the t8030 "time-dilation" property).
# Guest-measured deadlines -- watchdogs, driver timeouts, the datamigrator's
# per-plugin budget -- then scale with how fast this emulator actually runs.
[ -n "${INFERNO_TIME_DILATION:-}" ] && MACHINE+=",time-dilation=$INFERNO_TIME_DILATION"
# Keep the `sgx` GPU node in the device tree so a real AGX driver attaches.
[ -n "${INFERNO_GPU:-}" ] && MACHINE+=",gpu=$INFERNO_GPU"
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
# Optional extra kernel boot-args (e.g. INFERNO_EXTRA_BOOTARGS="pcidebug=0xffffffff"
# to trace IOPCIFamily enumeration). Appended verbatim.
[ -n "${INFERNO_EXTRA_BOOTARGS:-}" ] && APPEND+=" $INFERNO_EXTRA_BOOTARGS"

CMD=( "$QEMU"
    -M "$MACHINE"
    -kernel "$KERNEL"
    -dtb "$DTB"
    -append "$APPEND"
    -smp "${INFERNO_SMP:-7}" -m "${INFERNO_RAM:-4G}"
    -serial mon:stdio
    -drive "file=sep_nvram,if=pflash,format=raw${SNAP}"
    -drive "file=sep_ssc,if=pflash,format=raw${SNAP}"
    -drive "file=root,format=raw,if=none,id=root${INFERNO_ROOT_DRIVE_OPTS:-}${SNAP}"
        -device nvme-ns,drive=root,bus=nvme-bus.0,nsid=1,nstype=1,logical_block_size=4096,physical_block_size=4096
    -drive "file=firmware,format=raw,if=none,id=firmware${SNAP}"
        -device nvme-ns,drive=firmware,bus=nvme-bus.0,nsid=2,nstype=2,logical_block_size=4096,physical_block_size=4096
    -drive "file=syscfg,format=raw,if=none,id=syscfg${SNAP}"
        -device nvme-ns,drive=syscfg,bus=nvme-bus.0,nsid=3,nstype=3,logical_block_size=4096,physical_block_size=4096
    -drive "file=ctrl_bits,format=raw,if=none,id=ctrl_bits${SNAP}"
        -device nvme-ns,drive=ctrl_bits,bus=nvme-bus.0,nsid=4,nstype=4,logical_block_size=4096,physical_block_size=4096
    -drive "file=nvram,if=none,format=raw,id=nvram${SNAP}"
        -device apple-nvram,drive=nvram,bus=nvme-bus.0,nsid=5,nstype=5,id=nvram,logical_block_size=4096,physical_block_size=4096
    -drive "file=effaceable,format=raw,if=none,id=effaceable${SNAP}"
        -device nvme-ns,drive=effaceable,bus=nvme-bus.0,nsid=6,nstype=6,logical_block_size=4096,physical_block_size=4096
    -drive "file=panic_log,format=raw,if=none,id=panic_log${SNAP}"
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

# Host networking for the emulated Broadcom Wi-Fi endpoint.
#
# The t8030 machine looks up a netdev with id "wlan0" and, when it finds one,
# attaches the BCM4378 endpoint's NIC to it; without one the emulated Wi-Fi
# still associates with the fake access point but carries no traffic. libslirp
# ("user") needs no privileges and gives the guest 10.0.2.15 with a gateway and
# DNS resolver at 10.0.2.2/10.0.2.3.
#
# Override with INFERNO_NETDEV (e.g. a vmnet/tap backend), or set it to "none"
# to leave the interface unconnected.
NETDEV="${INFERNO_NETDEV:-user,id=wlan0}"
[ "$NETDEV" != "none" ] && CMD+=( -netdev "$NETDEV" )

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
