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

export PATH="$PREFIX/bin:$PATH"
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

echo "== 1. inferno-usbd =="
inferno-usbd -s "$USB_SOCK" -b "$BROKER_SOCK" -v &
PIDS+=($!)
sleep 1

echo "== 2. usbmuxd (shim-backed, foreground) =="
# -U "" keeps it from dropping privileges; -v verbose; -z prevents forking.
usbmuxd -v -z &
PIDS+=($!)
sleep 1

echo "== 3. main VM (t8030) =="
INFERNO_DATA="$DATA_DIR" INFERNO_BUILD="${INFERNO_BUILD:-$HOME/Inferno/build}" \
    RESTORE=1 "$HERE/run-main-vm.sh" &
PIDS+=($!)

echo "== waiting for the device to appear on the host USB stack =="
for i in $(seq 1 60); do
    if idevicerestore -h >/dev/null 2>&1 && irecovery -q >/dev/null 2>&1; then
        echo "   device visible in recovery/DFU"
        break
    fi
    sleep 2
done

echo "== 4. idevicerestore (erase) =="
echo "   NOTE: USB is experimental; if it stalls, see README 'Live bring-up'."
idevicerestore --erase --restore-mode -i "$ECID" "$IPSW" -T "$DATA_DIR/root_ticket.der"

echo "== restore command returned; leaving VM running (Ctrl-C to stop) =="
wait
