#!/usr/bin/env python3
"""inferno-cache-rehash — make an InfernoFSPatcher-edited dyld shared cache
validate again, so iOS does not kill every process that maps a patched page.

Why this exists
---------------
InfernoFSPatcher rewrites a few dozen bytes of the guest's dyld shared cache to
force software rendering (CoreImage/QuartzCore Metal off, etc.). It does this
*without* re-signing the cache and has always relied on the kernel's code-signing
bypass to let the modified pages load.

On iOS 26 that is no longer enough. Each shared-cache page is validated against a
SHA-256 in the subcache's own embedded code signature, and a page whose bytes no
longer match its stored hash is rejected -- from iOS 26 the kernel then
*terminates the faulting process*:

    launchd: (com.apple.backboardd [N]) exited with exit reason
        (namespace: 3 code: 0x2) - OS_REASON_CODESIGNING

Every daemon that maps a patched page dies and respawns forever; backboardd
survives only until it first touches the patched CoreImage/QuartzCore pages, i.e.
exactly when the UI would come up, so the device sits with every process alive
and nothing on screen. (Measured: a *non*-patched cache produces zero such kills;
the patched-but-not-rehashed cache produces ~150 per boot.)

The fix is not to fight the many kernel kill paths but to make the pages valid:
recompute the code-directory page hash for each page InfernoFSPatcher touched.
The kernel's AMFI/trust-cache bypass already accepts the resulting cdhash, so no
re-signing is needed -- only the per-page hashes have to match the bytes. With
this run after InfernoFSPatcher, iOS 26.5 boots to the lock screen.

What it does
------------
For every subcache with a sibling `.InfernoOriginalBytes` file (which
InfernoFSPatcher writes, recording exactly which offsets it changed), it locates
the embedded signature (CSMAGIC_EMBEDDED_SIGNATURE, 0xfade0cc0), finds the
CodeDirectory, and for each code-signing page that contains an edited byte,
recomputes SHA-256 over that page and writes it back into the hash slot. Only the
touched pages are rehashed, and each new hash is verified against the bytes
before the file is written.

Safe to re-run: a page whose hash already matches is left untouched.

Usage
-----
  inferno-cache-rehash.py <cache-dir>

where <cache-dir> is the directory holding dyld_shared_cache_arm64e* -- i.e.
`.../System/Library/Caches/com.apple.dyld` inside the mounted OS cryptex,
the same path passed to InfernoFSPatcher. Run it *after* InfernoFSPatcher.

Copyright (c) 2026 Inferno host-restore contributors.
SPDX-License-Identifier: AGPL-3.0-or-later
"""
import glob
import hashlib
import os
import struct
import sys

CSMAGIC_EMBEDDED_SIGNATURE = 0xFADE0CC0
CSSLOT_CODEDIRECTORY = 0
CD_HASHTYPE_SHA256 = 2

# CodeDirectory field offsets (big-endian), from xnu's cs_blobs.h.
CD_HASHOFFSET = 0x10   # uint32 hashOffset
CD_NSPECIAL = 0x18     # uint32 nSpecialSlots  (NOT the code slots -- easy to
CD_NCODE = 0x1C        # uint32 nCodeSlots      confuse; codeslots are at 0x1C)
CD_CODELIMIT = 0x20    # uint32 codeLimit
CD_HASHSIZE = 0x24     # uint8  hashSize
CD_HASHTYPE = 0x25     # uint8  hashType
CD_PAGESIZE = 0x27     # uint8  pageSize (log2)


def find_code_directory(d, cs):
    """Return the file offset of the primary CodeDirectory in the SuperBlob at
    file offset `cs`, or None."""
    magic, _length, count = struct.unpack_from(">III", d, cs)
    if magic != CSMAGIC_EMBEDDED_SIGNATURE:
        return None
    for i in range(count):
        slot_type, off = struct.unpack_from(">II", d, cs + 12 + i * 8)
        if slot_type == CSSLOT_CODEDIRECTORY:
            return cs + off
    return None


def rehash_subcache(path, edited_offsets):
    d = bytearray(open(path, "rb").read())
    cs = d.find(b"\xfa\xde\x0c\xc0")
    if cs < 0:
        print("  %s: no embedded signature, skipped" % os.path.basename(path))
        return 0
    cd = find_code_directory(d, cs)
    if cd is None:
        print("  %s: no CodeDirectory, skipped" % os.path.basename(path))
        return 0

    hash_off = struct.unpack_from(">I", d, cd + CD_HASHOFFSET)[0]
    n_code = struct.unpack_from(">I", d, cd + CD_NCODE)[0]
    code_limit = struct.unpack_from(">I", d, cd + CD_CODELIMIT)[0]
    hash_size = d[cd + CD_HASHSIZE]
    hash_type = d[cd + CD_HASHTYPE]
    page_shift = d[cd + CD_PAGESIZE]
    if hash_type != CD_HASHTYPE_SHA256 or hash_size != 32:
        raise SystemExit(
            "%s: unexpected hash type %d/size %d (only SHA-256 handled)"
            % (path, hash_type, hash_size))
    page_size = 1 << page_shift

    slots = {off >> page_shift for off in edited_offsets}
    updated = 0
    for slot in sorted(slots):
        if slot >= n_code:
            continue
        start = slot * page_size
        end = min(start + page_size, code_limit)
        want = hashlib.sha256(d[start:end]).digest()
        pos = cd + hash_off + slot * hash_size
        if d[pos:pos + hash_size] != want:
            d[pos:pos + hash_size] = want
            updated += 1

    if updated:
        open(path, "wb").write(d)
        # Read back and confirm every touched slot now matches.
        v = open(path, "rb").read()
        for slot in sorted(slots):
            if slot >= n_code:
                continue
            start = slot * page_size
            end = min(start + page_size, code_limit)
            pos = cd + hash_off + slot * hash_size
            if v[pos:pos + hash_size] != hashlib.sha256(v[start:end]).digest():
                raise SystemExit("%s: slot %d did not verify after write"
                                 % (path, slot))
    print("  %s: %d page hash(es) updated" % (os.path.basename(path), updated))
    return updated


def main():
    if len(sys.argv) != 2:
        sys.stderr.write("usage: %s <cache-dir>\n" % os.path.basename(sys.argv[0]))
        return 2
    cache_dir = sys.argv[1]
    marks = sorted(glob.glob(os.path.join(cache_dir, "*.InfernoOriginalBytes")))
    if not marks:
        sys.stderr.write(
            "no *.InfernoOriginalBytes files in %s -- run InfernoFSPatcher "
            "first\n" % cache_dir)
        return 1
    total = 0
    for mark in marks:
        sub = mark[:-len(".InfernoOriginalBytes")]
        if not os.path.exists(sub):
            continue
        offsets = []
        for line in open(mark):
            line = line.strip()
            if line:
                offsets.append(int(line.split(":")[0], 16))
        total += rehash_subcache(sub, offsets)
    print("total pages rehashed: %d" % total)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
