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
| Async completion + NAK-retry + USB reset (matches the emulated dwc controller) | **live-validated** |
| Enumeration (device + config descriptors, address assignment) | **live-validated** against real iPhone 11 firmware |
| Broker fan-out + reader-thread scheduler (concurrent IN/OUT, persistent bulk-IN) | **live-validated** |
| `libusb` shim: descriptors, sync + async transfers, pollfd/event loop, transfer-flag semantics | **live-validated** |
| Full stack (`usbmuxd`/`libirecovery`/`idevicerestore`) built against the shim | **working** (`build-host-tools.sh`) |
| Drive the real iOS restore over the host-direct path | **working** — see below |

End to end, with `inferno-restore.sh`: the emulated iPhone enumerates, the
shim-backed **`usbmuxd` attaches it** (`idevice_id -l` →
`00008030-1122334455667788`), and **`idevicerestore` drives the actual restore
protocol** — 25+ real restore steps (`find_filesystem_partitions`,
`verify_storage_for_update`, `load_sep_os`, …), all firmware components
personalized (LLB, iBoot, DeviceTree, RestoreSEP, SEP, …), RootTicket sent — with
`USB mux: N reads / 0 errors, M writes / 0 errors`. The USB bridge carries the
whole restore with zero transport errors.

`make -C libusb-shim test` additionally stands up the broker and a fake iPhone
(exercising `ASYNC` completion and a parked bulk-IN released by a bulk-OUT) and
round-trips the real libusb API through the shim without any VM.

### Where the current restore stops

`idevicerestore` reaches `Sending NORData now...` then reports `Unable to send
NORData`. This is the guest `restored` daemon rejecting the NOR/NAND firmware
write (note `NAND firmware file not exist: /usr/standalone/firmware/t302/…pak`),
**not** a transport problem — the mux shows 0 errors, and `restored` runs inside
the guest, so it would reject the same write whether the host or the companion
VM drove it. That final step is Inferno device-emulation / firmware territory,
independent of the host-vs-companion question this branch addresses.

## Remaining work

1. **NOR/NAND firmware write.** Chase `Unable to send NORData` on the guest /
   Inferno side (NAND controller `t302` firmware, `restored` NOR acceptance).
   This is orthogonal to the USB bridge.
2. **Mode-change re-enumeration.** If a restore path ever moves the device
   between USB modes mid-flight, `inferno-usbd` re-enumerates on socket
   reconnect; an in-place-reset trigger is a `LIVE-TUNE` item if needed.
3. **Debug logging.** `inferno-usbd -v` prints per-transaction `[txn]/[rdr]/
   [sub]/[cmp]/[cli]` traces and the shim honours `INFERNO_SHIM_DEBUG`; both are
   off by default and can be removed once the flow is fully settled.

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
