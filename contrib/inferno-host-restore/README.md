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

### Device-side bulk-IN parking (in progress)

The blocker to a *complete* restore is the throughput of the large NOR firmware
write. Root cause: the emulated dwc3 controller **NAKs an idle bulk-IN** rather
than holding it pending (the device model uses `USB_RET_NAK` on purpose — a
comment in `hw/usb/hcd-dwc3.c` notes that `USB_RET_ASYNC` there "causes DART
faults", i.e. stale IOMMU mappings on late completion). So the host has to
software-poll every idle RX-loop IN, and that polling competes with the bulk-OUT
data stream, starving the multi-MB NOR write until the guest `restored` times
out.

The clean fix, implemented here in **`hw/usb/hcd-tcp.c`** (the tcp_usb
device-side relay): when the device NAKs a bulk/interrupt IN, don't relay the
NAK over the socket — park the request and re-run it *locally* on a timer until
the guest queues data, then send one response. Each re-run is a fresh
transaction, so the guest's current DMA descriptors are used and the DART faults
that broke device-side ASYNC don't occur. `inferno-usbd`'s reader then reverts
to a simple demux (no socket NAK storm to pace).

Status: **data now delivers through the parked path** — the re-poll must be slow
(`USB_TCP_HOST_REPOLL_NS`, 2ms) because each re-run re-fires the endpoint's
`XFERNOTREADY` event and polling too fast starves the guest so it never queues
data. What still needs work: after the version response is delivered, usbmuxd
does not yet mark the device *active/listable*, and there is a one-time
ID1→ID2 re-attach from an initial bulk-OUT NAK. Until that's resolved the
device attaches but `idevicerestore` can't discover it.

Everything up to the restore itself — enumeration, mode/serial/config, and (on
the earlier broker-polling revision, tag `1e5e94b`) the full restore protocol
through firmware personalization and RootTicket with zero transport errors —
works over the host-direct path.

## Device image handling: the SEP pairing spans four images

The SEP's anti-replay state is not confined to one file. It is a pairing across

- `sep_ssc`   -- the emulated secure-storage IC (metadata slots, four identical
                 copies of each, all rewritten together),
- `sep_nvram` -- the SEP's own NVRAM,
- `effaceable` -- where the D-key locker lives,
- the xART records inside `root` (`/private/xarts/*.gl`).

**Advance any one of those without the others and the device is unrecoverable.**
The usual way to do it by accident is to run with `snapshot=on` on the NAND
drives while leaving the two `if=pflash` drives (`sep_nvram`, `sep_ssc`)
writable: SEPOS rewrites the SEP-xART Locker into `sep_ssc`, the matching xART
records on the discarded `root` and `effaceable` never persist, and the next boot
dies in

```
panic(...): SEP Panic: :sks /sks : ...
```

immediately after `D-key effaceable locker does not exist, attempt to init it
implicitly` -- where a healthy boot instead logs `Fetched SEP-xART Locker with
CRC: ...` and carries on. `run-main-vm.sh` now refuses that combination; use
`INFERNO_SNAPSHOT_DRIVES=1`, which snapshots all nine drives including both
pflash devices, or snapshot nothing.

If it has already happened, none of the obvious repairs work:

| attempted state | result |
|---|---|
| the desynced `sep_ssc` as-is | `SEP Panic: :sks /sks` |
| `sep_ssc` zeroed | `SEP Panic: :sars/sars` (anti-replay rollback) |
| another device's working `sep_ssc` | `:sks /sks` -- it is bound to *that* device's media |
| boot fully writable so SEPOS can repair | panics before it can write anything |

**A re-restore does not fix it either** -- the restore ramdisk boots far enough to
hit the same `sks` panic *before* USB enumeration, so `idevicerestore` never gets
a device to talk to. What works is going back to a freshly created image set:
truncate `root`, `nvram`, `sep_nvram`, `sep_ssc`, `effaceable`, `ctrl_bits`,
`panic_log` and `firmware` to zeroes of their original sizes while **preserving
`syscfg`** (it carries the device identity), then restore. The device then
enumerates and the restore proceeds normally.

## Debugging: the gdbstub is not usable on t8030

`-s` works, but attaching stops the VM, and stopping the VM violates the AP<->SEP
mailbox timeout. Reproduced two ways on an iOS 26.5 boot: attaching once gave

```
panic(...): AppleSEPManager panic for "AppleSEPKeyStore": sks request timeout
```

and attaching/detaching twice left the guest wedged spinning 100% of userspace
inside UIKitCore instead. Momentary inspection is fine; breakpoints, stepping and
anything else that keeps the VM stopped are not. Use PC sampling through the HMP
monitor (`info registers -a` gives PC plus PSTATE, hence EL) or emulator-side
counters instead.

## Remaining work

1. **usbmuxd device activation over the parked path** — get the mux version
   handshake to complete so the device becomes listable; then re-validate the
   NOR-write throughput the parking was meant to fix.
2. **Avoid the initial re-attach** — the first bulk-OUT NAKs before the device's
   mux OUT endpoint is ready; retry it longer (or park it device-side too).
3. **Debug logging.** `inferno-usbd -v` prints `[txn]/[rdr]/[sub]/[cmp]/[cli]`
   traces and the shim honours `INFERNO_SHIM_DEBUG`; both off by default.

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
