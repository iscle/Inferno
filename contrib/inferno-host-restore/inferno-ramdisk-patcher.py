#!/usr/bin/env python3
"""inferno-ramdisk-patcher — patch binaries inside an iOS restore RAM disk.

Inferno already patches the kernelcache (hw/arm/apple-silicon/kernel_patches.c)
and the restored filesystem's dyld shared cache (InfernoFSPatcher). This is the
third member of that family: the restore RAM disk carries its own userland copies
of the trust machinery, and those are not covered by either.

Why it is needed
----------------
Restoring iOS 16+ reaches `seal_system_volume` and fails there:

    restored_external: AppleImage4 [INFO] certificate trust evaluation failed
    restored_external: AppleImage4 [ERROR] trust evaluation failed: 80
    img4_firmware_execute failed: 80
    CHECKPOINT FAILURE:(FAILURE:6) RESTORED:[0x068B] seal_system_volume

That is a *certificate chain* evaluation, not a missing manifest object, so no
amount of fixing up the ticket helps -- and no amount of better hardware
modelling either; it is pure software trust. The evaluation runs in
/usr/lib/libimage4.dylib inside the RAM disk, which the kernel img4 patch does
not reach. The step cannot simply be skipped: the code path that actually runs
consults no restore option (the `SealSystemVolumeDuringRestore` sites belong to a
different flow), and a sealed-but-unverified volume is in any case closer to what
the kernel expects at boot than an unsealed one.

So make the RAM disk's evaluator report success. Note that the kernel patch's
approach -- overwrite `Img4DecodePerformTrustEvaluationWithCallbacksInternal`'s
prologue with `mov w0,#0; ret` -- does **not** work here: that function fills in
output parameters the RAM disk's caller then dereferences, so short-circuiting it
makes restored_external die on SIGSEGV mid-seal (observed: "restored exited with
status 0x9 due to signal 9"). The kernel's caller happens to tolerate it; this one
does not.

Which patch, and why this one
-----------------------------
Three variants were measured. Only the first keeps the restore working:

  | patch                                   | install_splat | seal cert eval |
  |-----------------------------------------|---------------|----------------|
  | (none)                                  | passes        | fails, err 80  |
  | Internal prologue -> `mov w0,#0; ret`   | passes        | passes         |
  | verdict store `mov w8,#0x50` -> `#0`    | breaks        | n/a            |
  | caller test in `img4_firmware_execute`  | breaks        | n/a            |

The usual instinct -- let the function run so its out-parameters get filled, and
flip only the verdict afterwards -- is wrong here. The evaluation records failure
state that the *nonce roll* inside `_personalize_splat_ticket` later consumes, so
overwriting the verdict after the fact leaves `install_splat` operating on state
that genuinely failed, and it dies at the nonce preroll. Returning success before
anything is marked failed keeps that state coherent.

(Note also that the resulting death is not a crash: `restored` exits cleanly and
the monitor reboots, so launchd SIGKILLs it. `SIGNAL(9)` in restore-child-failures
means "the engine gave up", not "something faulted".)

No re-wrapping
--------------
`extract_im4p_payload()` returns a file verbatim as payload type "raw" when it
does not parse as IM4P, and `apple_boot_load_ramdisk()` accepts "rdsk" or "raw",
so the patched image can be handed straight to -initrd / INFERNO_RAMDISK. There is
nothing to re-sign or re-wrap.

Usage
-----
  inferno-ramdisk-patcher.py <ramdisk.dmg|.im4p> <output.dmg> [--img4 PATH]
  inferno-ramdisk-patcher.py --dry-run <ramdisk.dmg> <output.dmg>

Copyright (c) 2026 Inferno host-restore contributors.
SPDX-License-Identifier: AGPL-3.0-or-later
"""
import argparse
import os
import shutil
import struct
import subprocess
import sys
import tempfile

# (relative path, symbol-or-None, find, replace, description)
#
# find/replace are byte patterns, never hardcoded offsets, in the spirit of
# hw/arm/apple-silicon/kernel_patches.c. If `symbol` is given the pattern is
# anchored there (a bare function prologue is not unique -- this one occurs 17
# times); otherwise the pattern must be unique in the whole file. Either way the
# bytes are verified before anything is written.
PATCHES = [
    (
        "usr/lib/libimage4.dylib",
        "__Img4DecodePerformTrustEvaluationWithCallbacksInternal",
        # prologue:
        #   pacibsp ; sub sp, sp, #0x90
        bytes.fromhex("7f2303d5") + bytes.fromhex("ff4302d1"),
        #   mov w0, #0 ; ret
        bytes.fromhex("00008052") + bytes.fromhex("c0035fd6"),
        "return success from the RAM disk's img4 trust evaluation",
    ),
]


def run(*cmd, check=True):
    return subprocess.run(cmd, capture_output=True, text=True, check=check)


def segments(path):
    segs, cur = [], {}
    for line in run("otool", "-l", path).stdout.splitlines():
        line = line.strip()
        if line.startswith("segname"):
            cur["name"] = line.split()[1]
        elif line.startswith("vmaddr"):
            cur["vmaddr"] = int(line.split()[1], 16)
        elif line.startswith("vmsize"):
            cur["vmsize"] = int(line.split()[1], 16)
        elif line.startswith("fileoff"):
            cur["fileoff"] = int(line.split()[1])
        elif line.startswith("filesize"):
            cur["filesize"] = int(line.split()[1])
            if "name" in cur:
                segs.append(cur)
            cur = {}
    return segs


def vm_to_file(segs, addr):
    for s in segs:
        if s["filesize"] and s["vmaddr"] <= addr < s["vmaddr"] + s["vmsize"]:
            return s["fileoff"] + (addr - s["vmaddr"])
    return None


def symbol_addr(path, name):
    for line in run("nm", "-a", path, check=False).stdout.splitlines():
        parts = line.split()
        if len(parts) >= 3 and parts[2] == name:
            try:
                return int(parts[0], 16)
            except ValueError:
                pass
    return None


def unwrap(src, dst, img4):
    """IM4P -> raw payload. A file that is already raw is copied through."""
    with open(src, "rb") as f:
        head = f.read(4)
    if head[:1] == b"\x30":  # DER SEQUENCE, so probably IM4P
        r = run(img4, "-i", src, "-o", dst, check=False)
        if r.returncode == 0 and os.path.exists(dst):
            return True
        print(f"  img4 unwrap failed ({r.stderr.strip()}); copying verbatim")
    shutil.copyfile(src, dst)
    return False


def resign(path):
    """Re-sign a patched binary so its CodeDirectory covers the new bytes.

    Without this the kernel SIGKILLs the process the first time it executes a
    modified *page* -- which is why each earlier patch attempt died at a
    different place: the death tracked the patch's page, not its semantics.
    (`signal 9` is SIGKILL; a genuine fault would report 11.)

    These dylibs ship ad-hoc signed with 4K page hashes, so re-sign ad-hoc,
    preserve identifier/entitlements/flags, and force --pagesize 4096 -- codesign
    otherwise defaults to 16K and produces a differently-shaped CodeDirectory.
    """
    r = run("codesign", "-f", "-s", "-", "--pagesize", "4096",
            "--preserve-metadata=identifier,entitlements,flags", path,
            check=False)
    if r.returncode != 0:
        return f" (WARNING: re-sign failed: {r.stderr.strip()})"
    v = run("codesign", "-v", path, check=False)
    return " (re-signed)" if v.returncode == 0 else " (re-signed, verify failed)"


def patch_mounted(root, dry_run):
    applied, missing = [], []
    for rel, sym, find, repl, desc in PATCHES:
        path = os.path.join(root, rel)
        if not os.path.exists(path):
            missing.append(f"{rel} (absent)")
            continue
        with open(path, "rb") as f:
            data = bytearray(f.read())
        if sym:
            addr = symbol_addr(path, sym)
            if addr is None:
                missing.append(f"{rel}:{sym} (symbol not found)")
                continue
            off = vm_to_file(segments(path), addr)
            if off is None:
                missing.append(f"{rel}:{sym} (no file offset)")
                continue
            if bytes(data[off:off + len(repl)]) == repl:
                applied.append(f"{desc} (already patched)")
                continue
            if bytes(data[off:off + len(find)]) != find:
                missing.append(
                    f"{rel}:{sym} bytes at 0x{off:x} are "
                    f"{bytes(data[off:off + len(find)]).hex()}, expected "
                    f"{find.hex()}; refusing to patch"
                )
                continue
        else:
            if data.count(repl) == 1 and data.count(find) == 0:
                applied.append(f"{desc} (already patched)")
                continue
            hits = data.count(find)
            if hits != 1:
                missing.append(
                    f"{rel}: pattern found {hits} times (need exactly 1)")
                continue
            off = data.find(find)
        if dry_run:
            applied.append(f"{desc} (dry run, would patch at 0x{off:x})")
            continue
        data[off:off + len(find)] = repl
        with open(path, "wb") as f:
            f.write(bytes(data))
        note = resign(path)
        applied.append(f"{desc} at 0x{off:x}{note}")
    return applied, missing


def main() -> int:
    ap = argparse.ArgumentParser(
        description="Patch binaries inside an iOS restore RAM disk."
    )
    ap.add_argument("ramdisk", help="restore RAM disk (.dmg IM4P or raw)")
    ap.add_argument("output", help="patched raw image, for -initrd")
    ap.add_argument("--img4", default=os.environ.get("INFERNO_IMG4",
                        os.path.expanduser("~/InfernoData/img4lib/img4")),
                    help="path to img4lib's img4 tool")
    ap.add_argument("-n", "--dry-run", action="store_true",
                    help="report what would be patched, change nothing")
    args = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="inferno-rd-")
    raw = os.path.join(tmp, "ramdisk.raw")
    print(f"== unwrapping {args.ramdisk}")
    unwrap(args.ramdisk, raw, args.img4)
    shutil.move(raw, args.output)

    mnt = os.path.join(tmp, "mnt")
    os.makedirs(mnt, exist_ok=True)
    mode = "-readonly" if args.dry_run else "-readwrite"
    print(f"== attaching {mode}")
    r = run("hdiutil", "attach", mode, "-nobrowse", "-mountpoint", mnt,
            args.output, check=False)
    if r.returncode != 0:
        print(f"attach failed: {r.stderr.strip()}", file=sys.stderr)
        return 1
    try:
        applied, missing = patch_mounted(mnt, args.dry_run)
    finally:
        run("hdiutil", "detach", mnt, check=False)

    for a in applied:
        print(f"  applied: {a}")
    for m in missing:
        print(f"  SKIPPED: {m}")
    if not applied:
        print("no patches applied", file=sys.stderr)
        return 1
    print(f"== wrote {args.output}")
    print("   pass it as INFERNO_RAMDISK / -initrd; it loads as a \"raw\" payload,")
    print("   so there is nothing to re-wrap or re-sign.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
