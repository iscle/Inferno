# Inferno host-direct restore (no companion VM)

This branch adds a way to restore the Inferno main VM (t8030 / iPhone 11)
**directly from the host**, replacing the companion x86_64 Linux VM that the
manual otherwise requires.

## Why a companion VM exists upstream

The emulated iPhone's USB does not attach to the host's USB stack. Instead,
Inferno's `usb-tcp-host` (main VM) opens a socket and speaks a small
transaction protocol — `tcp_usb` (`hw/usb/tcp-usb.h`) — where it plays the USB
*device* (the iPhone). Upstream, the other end of that socket is
`usb-tcp-remote`, a USB *device model* plugged into a **second** QEMU
(`qemu-system-x86_64`) running Linux. That Linux guest's kernel + `libusb` +
`usbmuxd` + `idevicerestore` drive the restore. The companion VM exists purely
to provide a real USB host stack.

Key facts that make removing it possible (verified against the source):

- The main VM is the socket **client** (`hw/usb/hcd-tcp.c` `connect()`s); the
  companion is the **server** (`hw/usb/dev-tcp-remote.c` `listen()`/`accept()`s).
  Whoever accepts is the USB **host** and drives transactions.
- The emulator hardcodes **no** USB descriptors — the socket is a raw USB
  transport; all descriptors come from guest firmware. So the host side must do
  real enumeration.
- The whole restore (DFU → recovery → restore → normal) rides **one** socket,
  so exactly one host process can own it — the role the companion's kernel had.

## What this branch adds

```
[ idevicerestore ]   [ usbmuxd ]        unmodified libimobiledevice stack
        \                /
     libinferno-usb  (libusb-1.0 API drop-in)   <- links here instead of libusb
              |  broker IPC (/tmp/inferno-usbd.sock)
        [ inferno-usbd ]   owns the tcp_usb socket, is the USB host controller
              |  tcp_usb (/tmp/InfernoUSBRemote)
        [ Inferno main VM: emulated iPhone USB device ]
```

- **`inferno-usbd`** (`libusb-shim/inferno_usbd.c`) — a daemon that listens on
  the `tcp_usb` socket the companion used, performs USB enumeration (device +
  config descriptors, `SET_ADDRESS`), and multiplexes the single device to
  multiple local clients over a Unix control socket. It is the host-USB-stack
  replacement — a minimal USB host controller driver over `tcp_usb`.
- **`libinferno-usb`** (`libusb-shim/libusb_shim.c`) — a `libusb-1.0`
  API-compatible shared library. The unmodified `usbmuxd` and `libirecovery`
  link against it; every `libusb_*` call is routed to `inferno-usbd`. It uses
  the vendored upstream `libusb.h` so struct/enum ABI matches the tools. Hotplug
  is reported unsupported so `usbmuxd` uses its device-list polling fallback.
- **`build-host-tools.sh`** — fetches and builds the full stack (libplist,
  libimobiledevice-glue, libtatsu, libusbmuxd, usbmuxd, libimobiledevice,
  libirecovery, idevicerestore) against the shim, applying the ChefKiss
  `idevicerestore.patch`.
- **`run-main-vm.sh`** — launches the main VM with the exact machine options
  from the manual, USB left on the default UNIX socket.
- **`inferno-restore.sh`** — orchestrates the correct startup order and runs the
  erase restore.

## Build & run

```sh
# 1. build the shim + daemon + the whole restore stack
contrib/inferno-host-restore/build-host-tools.sh

# 2. do the file-setup from the manual (disks, tickets, SEP firmware) if not done
#    (this branch's helper leaves those in ~/InfernoData)

# 3. run the host-direct restore
contrib/inferno-host-restore/inferno-restore.sh
```

## Status

| Component | State |
|---|---|
| `tcp_usb` host transport + control/bulk transaction state machine | implemented, unit-tested |
| Enumeration (device + config descriptors, address assignment) | implemented, unit-tested |
| Broker fan-out to multiple shim clients | implemented, unit-tested |
| `libusb` shim: descriptors, sync control/bulk, async transfers, pollfd/event loop | implemented, unit-tested |
| End-to-end shim ⇄ broker ⇄ tcp_usb ⇄ fake device | **passing** (`make -C libusb-shim test`) |
| Full restore against a live booting VM | **needs live bring-up** — see below |

`make -C libusb-shim test` runs `selftest.c`, which stands up the broker and a
fake iPhone device (serving the usbmux interface descriptors) and drives the
*real* libusb API through the shim: enumeration, descriptor parsing, a control
transfer, and a bulk transfer all round-trip. This validates the whole
mechanism except the live VM.

## Live bring-up (the remaining work)

These need a booting VM to observe and tune; they are marked `LIVE-TUNE` in
`inferno_usbd.c`:

1. **Persistent bulk-IN / NAK pacing.** `usbmuxd` keeps a bulk-IN outstanding
   waiting for device data. Real libusb hardware retries NAKs transparently in
   the host controller. `inferno-usbd` currently issues each transaction
   synchronously; the NAK-retry/async-reissue path (so a pending IN doesn't
   head-of-line-block other traffic) is the main scheduler work to finish. The
   protocol already carries per-transaction `id`s to support this.
2. **Mode-change re-enumeration.** As the guest moves DFU → recovery → restore →
   normal it re-enumerates with new descriptors (and PID). Whether the guest
   drops the socket or resets in place determines the trigger; `inferno-usbd`
   re-enumerates on socket reconnect and on client `RESET`, but the exact signal
   needs to be confirmed live.
3. **`usbmuxd` socket path.** The orchestrator uses `USBMUXD_SOCKET_ADDRESS` so
   the shim-backed `usbmuxd` and `idevicerestore` share a private socket and
   never collide with macOS's own `usbmuxd`.

Per the upstream manual, USB in Inferno is itself still experimental
("USB is currently unstable"), so first-boot restores may need iteration
regardless of host vs. companion.

## Files

```
contrib/inferno-host-restore/
├── README.md                 this file
├── build-host-tools.sh       fetch + build the stack against the shim
├── run-main-vm.sh            launch the t8030 main VM (USB on the socket)
├── inferno-restore.sh        orchestrate inferno-usbd + usbmuxd + VM + restore
└── libusb-shim/
    ├── tcp_usb_proto.h       mirror of hw/usb/tcp-usb.h (wire protocol)
    ├── broker_proto.h        inferno-usbd <-> shim IPC
    ├── inferno_usbd.c        the broker / host USB controller daemon
    ├── libusb_shim.c         the libusb-1.0 shim
    ├── libusb.h              vendored upstream libusb header (ABI reference)
    ├── selftest.c            end-to-end test with a fake device
    ├── libusb-1.0.pc.in      pkg-config for the shim
    └── Makefile
```
