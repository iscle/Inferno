#!/usr/bin/env bash
#
# build-host-tools.sh — build the libimobiledevice restore stack on the host,
# linked against the libinferno-usb shim (this directory's libusb-1.0 drop-in)
# instead of the real libusb, so idevicerestore talks to the emulated iPhone
# through inferno-usbd. No companion VM.
#
# This is the host-side equivalent of the "iDevice Tool Setup" the manual
# describes for the companion VM — it just builds natively and points pkg-config
# at the shim.
#
# Copyright (c) 2026 Inferno host-restore contributors.
# SPDX-License-Identifier: AGPL-3.0-or-later

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PREFIX="${INFERNO_HOST_TOOLS:-$HOME/InfernoData/host-tools}"
SRC="$PREFIX/src"
JOBS="$(sysctl -n hw.ncpu 2>/dev/null || nproc)"

# Order matters — each depends on the previous.
REPOS=(
    "libplist            https://github.com/libimobiledevice/libplist"
    "libimobiledevice-glue https://github.com/libimobiledevice/libimobiledevice-glue"
    "libtatsu            https://github.com/libimobiledevice/libtatsu"
    "libusbmuxd          https://github.com/libimobiledevice/libusbmuxd"
    "usbmuxd             https://github.com/libimobiledevice/usbmuxd"
    "libimobiledevice    https://github.com/libimobiledevice/libimobiledevice"
    "libirecovery        https://github.com/libimobiledevice/libirecovery"
    "idevicerestore      https://github.com/libimobiledevice/idevicerestore"
)

export PKG_CONFIG_PATH="$PREFIX/lib/pkgconfig:${PKG_CONFIG_PATH:-}"
export CPPFLAGS="-I$PREFIX/include ${CPPFLAGS:-}"
export LDFLAGS="-L$PREFIX/lib -Wl,-rpath,$PREFIX/lib ${LDFLAGS:-}"
export PATH="$PREFIX/bin:$PATH"
# openssl/gnutls etc. from Homebrew
if command -v brew >/dev/null; then
    for p in openssl@3 libtasn1 libzip; do
        pfx="$(brew --prefix "$p" 2>/dev/null || true)"
        [ -n "$pfx" ] && export PKG_CONFIG_PATH="$pfx/lib/pkgconfig:$PKG_CONFIG_PATH"
    done
fi

echo "==> building the libusb shim + inferno-usbd"
make -C "$HERE/libusb-shim" install PREFIX="$PREFIX"

echo "==> shim self-test"
make -C "$HERE/libusb-shim" test

mkdir -p "$SRC"
cd "$SRC"

for entry in "${REPOS[@]}"; do
    name="${entry%% *}"; url="${entry##* }"
    echo "==> $name"
    [ -d "$name" ] || git clone --depth 1 "$url" "$name"
    pushd "$name" >/dev/null

    # idevicerestore needs the ChefKiss patch to restore against Inferno.
    if [ "$name" = "idevicerestore" ] && [ -f "$HOME/InfernoData/idevicerestore.patch" ]; then
        git apply --check "$HOME/InfernoData/idevicerestore.patch" 2>/dev/null \
            && git apply "$HOME/InfernoData/idevicerestore.patch" \
            && echo "    applied idevicerestore.patch" || echo "    (patch already applied or N/A)"
    fi

    [ -f configure ] || ./autogen.sh --prefix="$PREFIX" PKG_CONFIG_PATH="$PKG_CONFIG_PATH"
    ./configure --prefix="$PREFIX" || { echo "configure failed for $name"; exit 1; }
    make -j"$JOBS"
    make install
    popd >/dev/null
done

echo
echo "Host restore tools built into $PREFIX"
echo "  bin: $PREFIX/bin  (inferno-usbd, usbmuxd, idevicerestore, ideviceinfo, ...)"
echo "  libusb-1.0 is the shim: $PREFIX/lib/libusb-1.0.dylib"
echo
echo "Next: run  contrib/inferno-host-restore/inferno-restore.sh"
