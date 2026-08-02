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

## Filesystem patches have to be applied *inside* the seal on iOS 16+

The manual's [Filesystem Patches](https://chefkiss.dev/guides/inferno/fs-patches/)
step — patch the dyld shared cache, and add `Disabled=true` to five
`LaunchDaemons` entries in `/System/Library/xpc/launchd.plist` — is written as
something you do to the restored image *after* the restore, by attaching `root`
and mounting the System volume.

That works on iOS 14, whose system volume is neither sealed nor snapshot-rooted.
From iOS 16/17 it does not: `restored` seals the system volume and creates a
`com.apple.os.update-<hash>` snapshot, and the kernel roots from *that snapshot*
(it refuses to root from the live filesystem of a sealed volume on a RELEASE
build — `run-main-vm.sh` has to be given `INFERNO_ROOT_SNAPSHOT` for this
reason). A host edit made afterwards lands on the live volume, which the running
guest never reads. Measured directly on 18.6.2: setting `Disabled=true` on
`com.apple.bluetoothd` post-restore still let it respawn 97 times.

Two halves, two different answers:

- **The dyld shared cache** is fine post-restore, because on cryptex-era iOS it
  is not on the sealed volume at all — it lives inside the OS cryptex,
  `Preboot:/cryptex1/current/os.dmg`, which is unsealed.
- **The launch service cache** is on the sealed volume, so it has to be patched
  before `seal_system_volume` runs.

### iOS 26: the patched cache must be re-hashed

On iOS 26 patching the dyld shared cache is not enough on its own. Each
shared-cache page is validated against a SHA-256 in the subcache's own embedded
code signature, and from iOS 26 the kernel *terminates* any process that faults
in a page whose bytes no longer match its stored hash:

```
launchd: (com.apple.backboardd [N]) exited with exit reason
    (namespace: 3 code: 0x2) - OS_REASON_CODESIGNING
```

InfernoFSPatcher rewrites the cache without re-signing it, so every daemon that
maps a patched page dies and respawns forever; backboardd survives only until it
first touches the patched CoreImage/QuartzCore pages — exactly when the UI would
come up — so the device sits with every process alive and a black screen. (A
non-patched cache produces zero such kills; the patched-but-not-re-hashed cache
produces ~150 per boot.)

The fix is to make the pages valid rather than fight the kernel: after running
InfernoFSPatcher, run

```
inferno-cache-rehash.py <cryptex>/System/Library/Caches/com.apple.dyld
```

which recomputes the CodeDirectory page hash for each page InfernoFSPatcher
touched (it reads the `.InfernoOriginalBytes` files the patcher leaves behind).
The kernel's AMFI/trust-cache bypass already accepts the resulting cdhash, so no
re-signing is needed. With this, iOS 26.5 boots to the lock screen. It is
idempotent and safe to re-run.

### The hook

`patches/idevicerestore-fs-patch-hook.patch` adds `INFERNO_FS_PATCH_CMD`, which
idevicerestore runs at the `SystemImageRootHash` data request that `restored`
raises **from inside** the `seal_system_volume` checkpoint. That instant is the
one the whole mechanism turns on:

```
Checkpoint started   id: 0x68B (seal_system_volume)
  -> SystemImageCanonicalMetadata request   (host sends the .mtree)
     ... 8 minutes of canonical-metadata work over the volume ...
  -> SystemImageRootHash request            <-- the hook fires here
     Unmounting filesystems
     Sealing System Volume
Checkpoint completed id: 0x68B (seal_system_volume) result=0
```

It is after the canonical-metadata pass (so the patch cannot be flagged by it),
it is before any sealing has happened, and `restored` is blocked waiting for the
host's reply, so nothing is racing with the edit. The gate is expressed with
*names*, not the numeric checkpoint id, because restored's ids move between
releases:

```
INFERNO_FS_PATCH_CMD=<cmd>           run "<cmd> <checkpoint>" at that point
INFERNO_FS_PATCH_CHECKPOINT=<name>   default seal_system_volume; empty disables
                                     the gate
INFERNO_FS_PATCH_DATATYPE=<type>     default SystemImageRootHash
```

`SystemImageRootHash` is also requested earlier, from `install_kernel_cache`;
the checkpoint gate is what tells the two apart. On a build that never seals —
iOS 14 — neither the checkpoint nor the data request ever appears, the hook never
fires, and the existing post-restore flow is untouched.

### The patcher

`inferno-preseal-fs-patch.py` is the command the hook runs. It deliberately does
**not** mount anything: at that moment the RAM disk has the container open and
the system volume mounted, and a second APFS driver writing into the same
container would either be reverted by the guest's next checkpoint or corrupt it.

Instead it treats the NAND image as bytes:

1. scan the image's allocated regions (via `SEEK_DATA`/`SEEK_HOLE`) for
   4096-aligned blocks starting with `bplist00` — APFS file data always begins on
   a block boundary;
2. recover each candidate's length from its binary-plist trailer, which is
   confirmed arithmetically (`offsetTableOffset + numObjects * offsetIntSize`
   must land exactly on the trailer), then parse it and keep the ones that are a
   launch service cache;
3. re-serialise with the five services disabled and pad the result back to
   *exactly* the original byte count, so **no APFS metadata changes at all** —
   no allocation, no inode update, no checkpoint. The guest's cached metadata
   stays valid, and the seal, computed afterwards from the on-disk content,
   covers the patched bytes.

The padding is inserted between the last object and the offset table and the
trailer's `offsetTableOffset` corrected, so every object offset is unchanged and
the result is an ordinary valid binary plist. It is re-parsed, cross-checked
against CoreFoundation via `plutil` (the parser launchd actually uses), written,
and read back before the hook returns. There is room: Apple's 2,045,714-byte
cache re-serialises to 1,066,374 bytes with the five `Disabled` keys added.

Because none of this needs an APFS driver, a loop device or root, it behaves the
same on macOS and Linux.

### Measured result: the hook lands, the byte-level edit does not survive SSV

Everything above works, and the seal is still happy afterwards. On 18.6.2:

```
16:26:04 fs-patch hook: SystemImageRootHash seen outside checkpoint
         seal_system_volume (active: 'install_root_hash'), not firing
16:27:34 Checkpoint started   id: 0x68B (seal_system_volume)
16:35:55 == running filesystem-patch hook before seal_system_volume
16:36:03 == filesystem-patch hook done                      (7.7 s)
16:36:04 Sealing System Volume (77)
16:38:37 Checkpoint completed id: 0x68B (seal_system_volume) result=0
16:38:39 Checkpoint completed id: 0x669 (create_system_snapshot) result=0
```

with, in the guest:

```
restored_external: AppleImage4 [DEBUG] trust evaluation succeeded for payload: msys
```

and the restored image reports `Sealed: Yes` (an unpatched restore that has been
edited afterwards reports `Sealed: Broken`), with the patched cache still on disk
at the same offset and the same 2,045,714 bytes.

**But the guest cannot read it.** Mounting the restored System volume on macOS,
every untouched file reads fine and only the patched one fails:

```
System/Library/xpc/launchd.plist                  FAIL [Errno 94] Bad message
System/Library/CoreServices/SystemVersion.plist   OK 573
usr/lib/dyld                                      OK 1264816
System/Library/LaunchDaemons/com.apple.locationd.plist  OK 1552
```

and the guest kernel says the same thing, then dies:

```
apfs_announce_hash_mismatch:147: disk1s1 Data hash mismatch for 16384 bytes at
  offset 0 ... expected 8dda2fa1..., got 1b92868d...
apfs_vnop_read:11565: disk1s1 ### ... retval 94 filesize 2045714 offset 0 ###
panic(cpu 5 ...): launchd_cache_loader[28] exited -- ...
  description: Failed to create SecStaticCodeRef
```

So `seal_system_volume` does **not** compute the per-extent data hashes. Those
come from Apple's ASR image and are written verbatim by `asr`; sealing only
builds the root over hashes that already exist. `filesize 2045714` in the
mismatch identifies the file exactly. A byte-level edit — at *any* point after
`asr`, whether before or after sealing — therefore leaves a file whose recorded
hash no longer matches, and APFS refuses to read it on both macOS and iOS.

What does update the hash is a write that goes *through* an APFS driver: writing
the same patched plist onto the restored volume via a read-write mount and
reading it back succeeds. That is not usable from the hook, because at that
moment the RAM disk still has the container open (and the only windows in which
it does not are the `asr` write itself and the sealing pass). So the remaining
routes are:

- patch through a read-write mount *after* the restore and then re-create the
  `com.apple.os.update-<hash>` snapshot so the guest roots from patched content
  (`fs_snapshot_create`, needs root — there is no CLI for a named APFS snapshot);
- or let the guest root from the live filesystem, which needs the
  `"Rooting from the live fs of a sealed volume is not allowed on a RELEASE
  build"` check in `apfs_vfsops.c` to be patched out kernel-side.

The hook itself is independent of which of those wins: `INFERNO_FS_PATCH_CMD` is
just "run this at the last mutable moment", so a different patcher can be dropped
in without touching C.

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

## Known defect: dart-apcie2 faults on a first boot with Wi-Fi attached

Reproducible on an iOS 14.0b5 first boot after an erase restore, twice in a row,
and only when a netdev is attached to the emulated BCM4378 endpoint:

```
panic(cpu 0 caller ...): "dart-apcie2 (0x...): DART(DART) error:
    SID 1 PTE invalid exception on read with DVA ..."
panic(cpu 0 caller ...): "dart-apcie2 (0x...): DART(DART) error:
    SID 1 TTBR invalid exception ..."
```

With `INFERNO_NETDEV=none` the same image boots with no panic and runs for at
least 21 minutes of guest time. So the emulated PCIe IOMMU is being handed a
stream ID whose page tables are not (or no longer) valid, on a path only the
Wi-Fi endpoint's DMA exercises. A device that is already through Setup does not
hit it -- it takes the first-boot Wi-Fi provisioning to provoke.

Not diagnosed further. Filed here because it is an emulation bug in the IOMMU
rather than a guest problem, and because it will bite anyone doing a fresh iOS 14
restore.

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

Two traps that have each cost measurement runs in this project:

- **Detach VMs into their own session, not just `nohup ... & disown`.** `disown`
  leaves the child in the caller's process group, so anything that kills that
  group (a harness timeout, for instance) reaps the VM too -- silently, mid-run.
  macOS has no `setsid(1)`; a two-line Python wrapper calling `os.setsid()`
  before `Popen` does the job.
- **If you add a counter that generated code writes to, its address must never
  move.** A `GArray` of counters reallocates as it grows, leaving every
  previously translated block writing into freed memory; the crash lands minutes
  later with nothing pointing at the cause. Use a fixed slab.
- **Dump instrumentation on `SIGUSR1`, not only from `atexit`.** A run that wedges
  or has to be `SIGKILL`ed takes every counter with it. This is the
  generalisable one: make the results harvestable at any moment, and a run that
  ends badly is still a run you can read.
- **Do not drive the monitor with `nc -U`.** `-monitor unix:...,server,nowait`
  serves one client at a time, and an `nc` that exits without closing cleanly
  leaves the chardev occupied: every later connection is accepted but never
  serviced, so `quit` silently never arrives and the VM looks hung when it is
  merely unreachable. Confirmed with `lsof -U` showing QEMU holding both the
  listener and a stale accepted FD. Use a client that closes its socket.

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
├── inferno-ramdisk-patcher.py  patch libimage4 inside the restore RAM disk
├── inferno-preseal-fs-patch.py the manual's launch-service patch, applied to
│                               the NAND image from inside seal_system_volume
├── ios18-n104.env            18.6.2 / n104ap asset names and knobs
├── ios26-n104.env            26.5 / n104ap asset names and knobs
├── patches/
│   ├── idevicerestore-cryptex1-local-ticket.patch
│   ├── idevicerestore-emulated-hardware-model.patch   N104DEV -> N104AP
│   ├── idevicerestore-fs-patch-hook.patch             INFERNO_FS_PATCH_CMD
│   └── usbmuxd-graceful-drain-close.patch
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
