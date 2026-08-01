#!/usr/bin/env python3
"""inferno-preseal-fs-patch — apply the guide's launch-service patch *inside* the
seal, by editing the guest's NAND image while the restore is still running.

Why this exists
---------------
The ChefKiss manual's "Filesystem Patches" step disables five launch services by
adding ``Disabled=true`` to their entries in
``/System/Library/xpc/launchd.plist``, after the restore has finished.

That works on iOS 14, whose system volume is neither sealed nor snapshot-rooted.
It does **not** work from iOS 16/17 onwards: the restore seals the system volume
and creates a ``com.apple.os.update-<hash>`` snapshot, and the kernel roots from
that snapshot (it refuses to root from the live filesystem of a sealed volume on
a RELEASE build). A host edit made after the restore lands on the live volume,
which the running guest never looks at -- verified directly: ``Disabled=true``
on ``com.apple.bluetoothd`` still let it respawn 97 times.

So the edit has to happen *before* ``seal_system_volume``. This script is the
part that does the editing; ``patches/idevicerestore-fs-patch-hook.patch`` is the
part that decides when to call it (at the ``SystemImageRootHash`` data request
raised from inside the seal checkpoint -- the last moment the host is given
control before the volume is sealed and snapshotted).

How it edits, and why that way
------------------------------
It does **not** mount the volume. At the moment it runs, the restore RAM disk has
the container open and the system volume mounted; a second APFS driver writing
into the same container would either be reverted by the guest's next checkpoint
or corrupt it outright.

Instead the edit is a *same-length, data-block-only* rewrite of the file:

  * scan the raw image for 4096-aligned blocks starting with ``bplist00``
    (APFS file data always starts on a block boundary) and recover each
    candidate's length from its binary-plist trailer;
  * keep the ones that parse as a launch service cache;
  * re-serialise with the five services disabled and pad the result back to
    *exactly* the original byte count, so no APFS metadata changes at all --
    no allocation, no inode update, no checkpoint. The guest's cached metadata
    stays valid, and the seal, computed afterwards from the on-disk content,
    covers the patched bytes.

The padding goes between the end of the object region and the offset table, and
the trailer's ``offsetTableOffset`` is fixed up accordingly. Every object offset
is unchanged, so the result is a plain valid binary plist -- it is re-parsed, and
cross-checked against CoreFoundation via ``plutil`` when that is available,
before anything is written.

Because nothing here needs an APFS driver, a loopback device or root, it behaves
identically on macOS and Linux.

KNOWN LIMITATION -- read this before trusting the result
--------------------------------------------------------
On a **sealed** system volume the byte-level edit is not enough, and this was
measured, not guessed. The restore seals happily over the patched image
("trust evaluation succeeded for payload: msys", `Sealed: Yes`), but every reader
that verifies SSV then rejects the file:

    apfs_announce_hash_mismatch:147: disk1s1 Data hash mismatch ... filesize 2045714
    apfs_vnop_read:11565: ... retval 94       (94 = EBADMSG)

`seal_system_volume` does not compute the per-extent data hashes: they come from
Apple's ASR image and `asr` writes them verbatim, so sealing only builds the root
over hashes that already exist. Any byte-level edit after `asr` -- before or
after the seal -- leaves a hash that no longer matches, and iOS panics in
`launchd_cache_loader` on the next boot.

What *does* refresh the hash is a write through an APFS driver (a read-write
mount), which is not available while the restore RAM disk has the container open.
See the README section "Measured result" for the two remaining routes.

So: this script is correct and useful on an **unsealed** volume (iOS 14/15), and
on a sealed one it is a demonstration of where the wall is. It warns when it is
about to write into a volume that looks sealed.

Usage
-----
  inferno-preseal-fs-patch.py [stage] [--image PATH] [--dry-run] [--verify]

``stage`` is ignored (the hook passes the checkpoint name); it is accepted so the
script can be used directly as ``INFERNO_FS_PATCH_CMD``.

Environment:
  INFERNO_FS_PATCH_IMAGE   raw NAND image to edit (default: $INFERNO_DATA/root)
  INFERNO_DATA             data directory, used to find ``root``
  INFERNO_FS_PATCH_SERVICES  comma-separated launchd labels to disable
                             (default: the five from the guide)

Copyright (c) 2026 Inferno host-restore contributors.
SPDX-License-Identifier: AGPL-3.0-or-later
"""
import argparse
import os
import plistlib
import re
import struct
import subprocess
import sys
import time

# The five launch services the manual tells you to disable. Overridable so the
# list can follow the guide without editing code.
DEFAULT_SERVICES = [
    "com.apple.voicemail.vmd",
    "com.apple.CommCenter",
    "com.apple.CommCenterMobileHelper",
    "com.apple.CommCenterRootHelper",
    "com.apple.locationd",
]

BLOCK = 4096
MAGIC = b"bplist00"
TRAILER_LEN = 32
# A launch service cache is a couple of MB; give the trailer search room without
# reading the whole volume for every stray bplist.
MAX_PLIST = 64 << 20

# Binary-plist trailer: 5 unused bytes, sortVersion, offsetIntSize,
# objectRefSize, numObjects (u64be), topObject (u64be), offsetTableOffset
# (u64be). For a file of this size numObjects and offsetTableOffset both fit in
# three bytes and topObject is 0, which makes the whole 32-byte trailer a very
# distinctive mostly-zero pattern to search for.
TRAILER_RE = re.compile(
    rb"\x00{6}[\x01-\x08]{2}\x00{5}.{3}\x00{8}\x00{5}.{3}", re.S)


def log(msg):
    sys.stdout.write("[preseal-fs-patch] %s\n" % msg)
    sys.stdout.flush()


# --- locating the launch service cache in the raw image ---------------------

def data_regions(fd, size):
    """[(start, end)] of the image's allocated regions, so holes are skipped.

    SEEK_DATA/SEEK_HOLE exist on both macOS and Linux. A freshly created NAND
    image is mostly hole, and skipping it turns a 34 GB scan into an 11 GB one.
    Falls back to one whole-file region where the platform lacks them.
    """
    if not hasattr(os, "SEEK_DATA"):
        return [(0, size)]
    regions, pos = [], 0
    while pos < size:
        try:
            start = os.lseek(fd, pos, os.SEEK_DATA)
        except OSError:
            break
        try:
            end = os.lseek(fd, start, os.SEEK_HOLE)
        except OSError:
            end = size
        if end <= start:
            break
        regions.append((start, min(end, size)))
        pos = end
    return regions or [(0, size)]


def plist_length_at(buf):
    """Total byte length of the binary plist starting at buf[0], or None.

    The trailer is found by pattern, then confirmed arithmetically: the offset
    table has numObjects entries of offsetIntSize bytes and ends where the
    trailer begins. That equation makes a false positive essentially impossible.
    """
    for m in TRAILER_RE.finditer(buf):
        tstart = m.start()
        if tstart < 8:
            continue
        (off_size, ref_size, n_objects, top, table_off) = struct.unpack(
            ">6xBBQQQ", buf[tstart:tstart + TRAILER_LEN])
        if not (1 <= off_size <= 8 and 1 <= ref_size <= 8):
            continue
        if top != 0 or n_objects == 0:
            continue
        if table_off < 8 or table_off >= tstart:
            continue
        if table_off + n_objects * off_size != tstart:
            continue
        return tstart + TRAILER_LEN
    return None


def is_launchd_cache(obj, services):
    if not isinstance(obj, dict):
        return False
    daemons = obj.get("LaunchDaemons")
    if not isinstance(daemons, dict) or len(daemons) < 32:
        return False
    labels = {e.get("Label") for e in daemons.values() if isinstance(e, dict)}
    return all(s in labels for s in services)


def find_caches(path, services, progress_every=4 << 30):
    """[(offset, length, parsed)] for every launch service cache in the image."""
    found, seen = [], set()
    size = os.path.getsize(path)
    with open(path, "rb") as f:
        regions = data_regions(f.fileno(), size)
        total = sum(e - s for s, e in regions)
        log("scanning %.1f GiB of allocated data in %s (%d region%s)"
            % (total / (1 << 30), path, len(regions),
               "" if len(regions) == 1 else "s"))
        scanned = 0
        next_report = progress_every
        t0 = time.time()
        chunk_size = 64 << 20
        for start, end in regions:
            # Align the read cursor down to a block so block-start detection is
            # exact regardless of where the region begins.
            pos = start - (start % BLOCK)
            while pos < end:
                want = min(chunk_size, end - pos)
                f.seek(pos)
                # Overlap by one block so a magic at a chunk edge is not missed.
                buf = f.read(want + BLOCK)
                if not buf:
                    break
                idx = 0
                while True:
                    idx = buf.find(MAGIC, idx)
                    if idx < 0:
                        break
                    abs_off = pos + idx
                    if abs_off % BLOCK == 0 and abs_off not in seen:
                        seen.add(abs_off)
                        hit = _try_cache(f, abs_off, size, services)
                        if hit:
                            found.append(hit)
                            log("  launch service cache at 0x%x, %d bytes, "
                                "%d LaunchDaemons"
                                % (hit[0], hit[1], len(hit[2]["LaunchDaemons"])))
                    idx += 1
                pos += want
                scanned += want
                if scanned >= next_report:
                    log("  ... %.0f%% (%.0fs, %d bplist candidates)"
                        % (100.0 * scanned / total, time.time() - t0, len(seen)))
                    next_report += progress_every
        log("scan done in %.0fs, %d block-aligned bplists, %d cache(s) found"
            % (time.time() - t0, len(seen), len(found)))
    return found


def _try_cache(f, off, size, services):
    """Cheap-to-expensive test of one block-aligned bplist candidate.

    An iOS system volume holds tens of thousands of binary plists, so the order
    matters: grow the read window only until a trailer resolves, reject on a
    plain substring test for the service labels before paying for a parse, and
    only then parse.
    """
    keep = f.tell()
    try:
        needles = [s.encode() for s in services]
        window = b""
        length = None
        for want in (256 << 10, 4 << 20, MAX_PLIST):
            want = min(want, size - off)
            if want <= len(window):
                break
            f.seek(off)
            window = f.read(want)
            length = plist_length_at(window)
            if length:
                break
        if not length:
            return None
        raw = window[:length]
        if not all(n in raw for n in needles):
            return None
        try:
            obj = plistlib.loads(raw)
        except Exception:
            return None
        if not is_launchd_cache(obj, services):
            return None
        return (off, length, obj)
    finally:
        f.seek(keep)


# --- building the same-length replacement ----------------------------------

def pad_to(data, target):
    """Grow a binary plist to exactly `target` bytes without changing content.

    Filler is inserted between the last object and the offset table; object
    offsets are all below that point so none of them move, and only the
    trailer's offsetTableOffset has to be corrected.
    """
    if len(data) > target:
        raise ValueError("patched plist is %d bytes, will not fit in %d"
                         % (len(data), target))
    pad = target - len(data)
    if pad == 0:
        return data
    (off_size, ref_size, n_objects, top, table_off) = struct.unpack(
        ">6xBBQQQ", data[-TRAILER_LEN:])
    body = data[:table_off]
    table = data[table_off:-TRAILER_LEN]
    trailer = struct.pack(">5xBBBQQQ", 0, off_size, ref_size,
                          n_objects, top, table_off + pad)
    out = body + b"\x00" * pad + table + trailer
    assert len(out) == target
    return out


def build_patched(raw, services):
    """(new_bytes, disabled_labels) — same length as `raw`."""
    obj = plistlib.loads(raw)
    daemons = obj["LaunchDaemons"]
    disabled, already = [], []
    for entry in daemons.values():
        if not isinstance(entry, dict):
            continue
        label = entry.get("Label")
        if label in services:
            if entry.get("Disabled") is True:
                already.append(label)
            else:
                entry["Disabled"] = True
                disabled.append(label)
    new = plistlib.dumps(obj, fmt=plistlib.FMT_BINARY)
    new = pad_to(new, len(raw))
    # Never write something we cannot read back.
    check = plistlib.loads(new)
    if check != obj:
        raise ValueError("re-parsed patched plist does not match")
    for entry in check["LaunchDaemons"].values():
        if isinstance(entry, dict) and entry.get("Label") in services:
            if entry.get("Disabled") is not True:
                raise ValueError("Disabled not set on %s" % entry.get("Label"))
    return new, disabled, already


def cf_check(data):
    """Second opinion from CoreFoundation's parser, which is what launchd uses.

    plistlib and CF are different implementations; the padded layout is only
    interesting if *CF* accepts it. Skipped where plutil does not exist.
    """
    try:
        import tempfile
        with tempfile.NamedTemporaryFile(suffix=".plist", delete=False) as tf:
            tf.write(data)
            name = tf.name
        try:
            r = subprocess.run(["plutil", "-convert", "xml1", "-o", "-", name],
                               capture_output=True)
            if r.returncode != 0:
                return "plutil rejected the padded plist: %s" % r.stderr.decode()[:200]
            if b"<key>Disabled</key>" not in r.stdout:
                return "plutil output has no Disabled key"
            return None
        finally:
            os.unlink(name)
    except FileNotFoundError:
        return None
    except Exception as e:  # pragma: no cover - best effort only
        return "plutil check skipped: %s" % e


# --- main ------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("stage", nargs="?", default="",
                    help="restore stage name passed by the hook (informational)")
    ap.add_argument("--image", default=None, help="raw NAND image (the 'root' file)")
    ap.add_argument("-n", "--dry-run", action="store_true",
                    help="find and build the patch, write nothing")
    ap.add_argument("--services", default=None,
                    help="comma-separated launchd labels to disable")
    a = ap.parse_args()

    image = (a.image or os.environ.get("INFERNO_FS_PATCH_IMAGE")
             or os.path.join(os.environ.get("INFERNO_DATA",
                                            os.path.expanduser("~/InfernoData")),
                             "root"))
    svc = a.services or os.environ.get("INFERNO_FS_PATCH_SERVICES")
    services = [s.strip() for s in svc.split(",")] if svc else list(DEFAULT_SERVICES)

    log("stage=%s image=%s" % (a.stage or "-", image))
    log("services: %s" % ", ".join(services))
    if not os.path.exists(image):
        log("ERROR: no such image: %s" % image)
        return 1

    caches = find_caches(image, services)
    if not caches:
        log("ERROR: no launch service cache containing all of %s was found."
            % ", ".join(services))
        log("       (a fragmented or not-yet-written volume would look like this)")
        return 1

    rc = 0
    for off, length, _ in caches:
        with open(image, "rb") as f:
            f.seek(off)
            raw = f.read(length)
        try:
            new, disabled, already = build_patched(raw, services)
        except Exception as e:
            log("ERROR: cannot build replacement for cache at 0x%x: %s" % (off, e))
            rc = 1
            continue
        problem = cf_check(new)
        if problem:
            log("ERROR: %s" % problem)
            rc = 1
            continue
        log("cache at 0x%x: disabling %s%s"
            % (off, ", ".join(disabled) or "(nothing new)",
               "; already disabled: " + ", ".join(already) if already else ""))
        if a.dry_run:
            log("  dry run, not writing")
            continue
        if disabled:
            log("  NOTE: on a sealed (SSV) volume this byte-level edit will not"
                " be readable —")
            log("        the per-extent hash was written by asr and sealing does"
                " not recompute it.")
            log("        See README, \"Measured result\". Fine on iOS 14/15.")
        if not disabled:
            log("  already patched, nothing to write")
            continue
        with open(image, "r+b") as f:
            f.seek(off)
            f.write(new)
            f.flush()
            os.fsync(f.fileno())
        with open(image, "rb") as f:
            f.seek(off)
            back = f.read(length)
        if back != new:
            log("  ERROR: read-back mismatch at 0x%x" % off)
            rc = 1
            continue
        obj = plistlib.loads(back)
        got = [e.get("Label") for e in obj["LaunchDaemons"].values()
               if isinstance(e, dict) and e.get("Disabled") is True
               and e.get("Label") in services]
        log("  wrote %d bytes in place; Disabled=true now set on: %s"
            % (length, ", ".join(sorted(got))))
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
