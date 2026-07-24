#!/usr/bin/env bash
#
# inferno-restore.sh — drive a full host-direct restore of the Inferno main VM,
# with no companion VM.
#
# Startup order mirrors the companion setup ("companion must be started before
# the main VM"): the host USB stack must own the socket before the VM connects.
#
#   1. inferno-usbd   — owns /tmp/InfernoUSBRemote, plays the USB host controller
#   2. usbmuxd        — shim-backed; provides the usbmux socket for restore mode
#   3. main VM        — connects, enumerates, appears to the host tools
#   4. idevicerestore — runs the actual erase/restore
#
# Copyright (c) 2026 Inferno host-restore contributors.
# SPDX-License-Identifier: AGPL-3.0-or-later

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DATA_DIR="${INFERNO_DATA:-$HOME/InfernoData}"
PREFIX="${INFERNO_HOST_TOOLS:-$DATA_DIR/host-tools}"
IPSW="${INFERNO_IPSW:-$HOME/Downloads/iPhone11,8,iPhone12,1_14.0_18A5351d_Restore.ipsw}"
ECID="${INFERNO_ECID:-0x1122334455667788}"

USB_SOCK="/tmp/InfernoUSBRemote"
BROKER_SOCK="/tmp/inferno-usbd.sock"
MUX_SOCK="/tmp/inferno-usbmuxd.sock"

export PATH="$PREFIX/bin:$PREFIX/sbin:$PATH"
export INFERNO_USBD_SOCK="$BROKER_SOCK"
# Point both usbmuxd (server) and libusbmuxd clients (idevicerestore) at our mux.
export USBMUXD_SOCKET_ADDRESS="UNIX:$MUX_SOCK"

PIDS=()
cleanup() {
    echo "== shutting down =="
    for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null || true; done
    rm -f "$USB_SOCK" "$BROKER_SOCK" "$MUX_SOCK"
}
trap cleanup EXIT INT TERM

command -v inferno-usbd >/dev/null || { echo "inferno-usbd not found; run build-host-tools.sh first" >&2; exit 1; }
command -v usbmuxd >/dev/null || { echo "usbmuxd not found; run build-host-tools.sh first" >&2; exit 1; }
[ -f "$IPSW" ] || { echo "IPSW not found: $IPSW (set INFERNO_IPSW)" >&2; exit 1; }

USBD_LOG="${INFERNO_USBD_LOG:-/tmp/inferno-usbd.log}"
echo "== 1. inferno-usbd (log: $USBD_LOG) =="
inferno-usbd -s "$USB_SOCK" -b "$BROKER_SOCK" -v > "$USBD_LOG" 2>&1 &
PIDS+=($!)
sleep 1

echo "== 2. usbmuxd (shim-backed, foreground) =="
# -f foreground; -v verbose; -U <me> avoids dropping to a non-existent
# 'usbmux' user on macOS; -S makes the daemon LISTEN on our private socket
# (it does not read USBMUXD_SOCKET_ADDRESS — that's client-side); -P NONE
# skips the /var/run/usbmuxd.pid lockfile that is fatal to create here.
# (Do NOT use -z: it makes usbmuxd exit when no device is present.)
# -p disables the lockdownd preflight so a device is made visible/listable as
# soon as it's ACTIVE, instead of gating visibility on a preflight lockdownd
# query (which is flaky against the emulated device and made discovery
# nondeterministic).
usbmuxd -f ${INFERNO_MUXD_VERBOSE:--v} -U "$(whoami)" -S "$MUX_SOCK" -P NONE -p \
    ${INFERNO_MUXD_LOG:+> "$INFERNO_MUXD_LOG" 2>&1} &
PIDS+=($!)
sleep 1

echo "== 3. main VM (t8030) =="
INFERNO_DATA="$DATA_DIR" INFERNO_BUILD="${INFERNO_BUILD:-$HOME/Inferno/build}" \
    RESTORE=1 "$HERE/run-main-vm.sh" &
PIDS+=($!)

echo "== waiting for the emulated iPhone to enumerate (via inferno-usbd) =="
for i in $(seq 1 60); do
    grep -qa 'device enumerated' "$USBD_LOG" 2>/dev/null && break
    sleep 2
done
grep -qa 'device enumerated' "$USBD_LOG" 2>/dev/null \
    && echo "   $(grep -a 'device enumerated' "$USBD_LOG" | tail -1)" \
    || echo "   WARNING: device did not enumerate"

echo "== waiting for usbmuxd to attach the device =="
for i in $(seq 1 90); do
    if idevice_id -l 2>/dev/null | grep -q .; then
        echo "   usbmuxd reports device: $(idevice_id -l 2>/dev/null | tr '\n' ' ')"
        break
    fi
    sleep 1
done
# Let the device's restored daemon and the mux settle before querying it — the
# very first restored queries right after attach are flaky on the emulated USB.
sleep "${INFERNO_SETTLE:-8}"

echo "== 4. idevicerestore (erase) =="
echo "   NOTE: USB is experimental; if it stalls, see README 'Remaining work'."
# Single invocation: re-running idevicerestore against a device whose restored
# has already been partway driven confuses it. The wait above ensures the device
# is discoverable first. Never let a nonzero exit trip 'set -e'.
idevicerestore ${INFERNO_IREC_VERBOSE:-} --erase --restore-mode -i "$ECID" "$IPSW" -T "$DATA_DIR/root_ticket.der" 2>&1 \
    | tee "/tmp/idevicerestore.out"
echo "== idevicerestore exited ${PIPESTATUS[0]} =="

echo "== leaving VM + usbmuxd running (Ctrl-C to stop) =="
wait
