#!/usr/bin/env python3
"""Add Cryptex1 objects to an unsigned IM4M so install_splat can find their digests.

Background
----------
Apple's TSS refuses to personalise a forged ECID, so idevicerestore never gets a
Cryptex1 ticket and the restore dies at `install_splat` (user progress 95). The AP
kernel's img4 trust evaluation is already bypassed, so the ticket does not need a
real signature -- but it does need the right *contents*. Handing the device the
plain AP ticket produced exactly the structural failure that predicts:

    ramrod_ticket_copy_data_object_property_from_ticket_data:
        failed to get data property from Img4 manifest
    install_splat_block_invoke: Failed to get expected digest for Cryptex1,SystemOS

The manifest parsed fine; it simply had no Cryptex1 objects. So clone an existing
object entry six times, retag it, and drop in the digest the device itself asked
us to certify.

Sources (derived, not guessed):
  * name -> img4 4CC comes from libauthinstall.dylib's mapping table
  * digests come from the device's own DeviceGeneratedRequest (INFERNO_CRYPTEX_DUMP)
  * the private-tag encoding is verified by re-encoding a known tag and finding it
    at the expected offset in the input ticket

Usage
-----
  add_cryptex_objects.py <in_ticket.der> <captured-arguments.plist> <out.der>

The input ticket, the captured request and the output are local artefacts and
never belong in a commit, a log, or a report.
"""
import plistlib
import sys

NAME_TO_TAG = {
    "Cryptex1,SystemOS": "csos",
    "Cryptex1,SystemTrustCache": "trcs",
    "Cryptex1,SystemVolume": "cssy",
    "Cryptex1,AppOS": "caos",
    "Cryptex1,AppTrustCache": "trca",
    "Cryptex1,AppVolume": "casy",
}


def b128(n: int) -> bytes:
    out = [n & 0x7F]
    n >>= 7
    while n:
        out.append((n & 0x7F) | 0x80)
        n >>= 7
    return bytes(reversed(out))


def privtag(fourcc: str) -> bytes:
    """Private/constructed high-tag-number form, tag number == the 4CC."""
    return b"\xff" + b128(int.from_bytes(fourcc.encode(), "big"))


def read_len(buf: bytes, i: int):
    """Return (length, index_after_length)."""
    n = buf[i]
    if n < 0x80:
        return n, i + 1
    k = n & 0x7F
    return int.from_bytes(buf[i + 1:i + 1 + k], "big"), i + 1 + k


def write_len(n: int) -> bytes:
    if n < 0x80:
        return bytes([n])
    b = n.to_bytes((n.bit_length() + 7) // 8, "big")
    return bytes([0x80 | len(b)]) + b


def tlv(buf: bytes, i: int):
    """Parse one TLV at i; return (tag_bytes, content, next_index)."""
    j = i
    if buf[j] & 0x1F == 0x1F:            # high tag number
        j += 1
        while buf[j] & 0x80:
            j += 1
        j += 1
    else:
        j += 1
    tag = buf[i:j]
    ln, k = read_len(buf, j)
    return tag, buf[k:k + ln], k + ln


def main() -> int:
    if len(sys.argv) != 4:
        print(__doc__)
        return 1
    src, argsplist, out = sys.argv[1:4]
    data = open(src, "rb").read()
    args = plistlib.load(open(argsplist, "rb"))
    dgr = args["DeviceGeneratedRequest"]

    # outer SEQUENCE -> IA5String "IM4M", INTEGER, SET{ MANB }, ...
    otag, obody, _ = tlv(data, 0)
    i = 0
    _, _, i = tlv(obody, i)            # "IM4M"
    _, _, i = tlv(obody, i)            # version
    set_start = i
    stag, sbody, set_end = tlv(obody, i)

    # inside that SET: the private-tagged MANB
    mtag, mbody, _ = tlv(sbody, 0)
    # inside MANB: SEQUENCE { IA5String "MANB", SET{objects} }
    seqtag, seqbody, _ = tlv(mbody, 0)
    k = 0
    _, _, k = tlv(seqbody, k)          # "MANB"
    objs_tag, objs_body, _ = tlv(seqbody, k)

    # find a template object entry carrying a DGST
    template = None
    p = 0
    while p < len(objs_body):
        t, body, nxt = tlv(objs_body, p)
        if b"DGST" in body:
            template = (t, body, objs_body[p:nxt])
            break
        p = nxt
    if template is None:
        print("no template object with DGST found", file=sys.stderr)
        return 1
    ttag, tbody, traw = template

    # the private tag wraps a SEQUENCE; its first element is the 4CC IA5String
    _, tseq, _ = tlv(tbody, 0)
    _, name_bytes, _ = tlv(tseq, 0)
    tname = name_bytes.decode()
    told = privtag(tname)
    if not traw.startswith(told):
        print(f"template tag mismatch for {tname}", file=sys.stderr)
        return 1

    # its digest is the 48-byte OCTET STRING after "DGST"
    d_at = traw.find(b"DGST") + 4
    assert traw[d_at] == 0x04, "expected OCTET STRING after DGST"
    dlen, d_val = read_len(traw, d_at + 1)
    old_digest = traw[d_val:d_val + dlen]

    added, extra = [], b""
    for name, tag in NAME_TO_TAG.items():
        ent = dgr.get(name)
        if not isinstance(ent, dict) or "Digest" not in ent:
            print(f"  skip {name}: no Digest in captured request")
            continue
        digest = ent["Digest"]
        if len(digest) != dlen:
            print(f"  skip {name}: digest is {len(digest)}B, template is {dlen}B")
            continue
        new = traw.replace(told, privtag(tag), 1)
        new = new.replace(tname.encode(), tag.encode(), 1)
        new = new.replace(old_digest, digest, 1)
        extra += new
        added.append(f"{name}->{tag}")

    if not added:
        print("nothing added", file=sys.stderr)
        return 1

    # rebuild, innermost first
    new_objs = objs_tag + write_len(len(objs_body) + len(extra)) + objs_body + extra
    new_seqbody = seqbody[:k] + new_objs
    new_manb_seq = seqtag + write_len(len(new_seqbody)) + new_seqbody
    new_manb = mtag + write_len(len(new_manb_seq)) + new_manb_seq
    new_set = stag + write_len(len(new_manb)) + new_manb
    new_obody = obody[:set_start] + new_set + obody[set_end:]
    open(out, "wb").write(otag + write_len(len(new_obody)) + new_obody)

    print(f"wrote {out}; template={tname}; added: {', '.join(added)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
