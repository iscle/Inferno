#!/usr/bin/env python3
"""Synthesise a local Cryptex1 TSS response for the Inferno emulated device.

Why this exists
---------------
Apple's TSS will not personalise a forged ECID, so `tss_request_send()` returns
NULL, idevicerestore logs "Unable to fetch Cryptex1 ticket", and the device never
receives a FirmwareResponseData for the Cryptex1 updater -- restore then fails at
`install_splat` with restore-step-user-progress = 95.

It does not need to be genuine. The AP kernel's img4 trust evaluation is already
bypassed (`allow unsigned firmware in img4_firmware_evaluate` rewrites the
prologue of Img4DecodePerformTrustEvaluationWithCallbacksInternal to
`mov w0,#0; ret`, confirmed applied to this kernelcache). On the device side the
call is `_AMAuthInstallUpdaterCryptex1LocalPolicyStitchTicketData` -- it
*stitches* the returned ticket into the local policy, which is ASN.1 assembly,
not a signature check. So the response has to be well-formed, not authentic.

The response plist is handed to the device verbatim as FirmwareResponseData, so
its shape is the entire interface.

Nonces
------
The device imports `__img4_nonce_domain_cryptex1_boot`,
`_img4_nonce_domain_preroll_nonce` and `_img4_nonce_domain_roll_nonce`, and the
personalisation parameters include SepNonce/SepNonceSlotID. Those nonce domains
are SEP-backed anti-replay state, so a ticket whose nonce does not match what SEP
reports can be rejected even if the AP accepts it. Pass the captured request with
--request so nonce/identity fields the device generated are echoed back rather
than invented.

Usage
-----
  make_cryptex_response.py <ticket.der> <out.plist> [--request req.plist]
                           [--tag ResponseTagName]...

With no --tag the default "Cryptex1,Ticket" is used; pass the tag(s) read from
the captured arguments plist (DeviceGeneratedTags/ResponseTags) if they differ.

NOTE: the generated response and the ticket it wraps are local artefacts. They
never go into a commit, a log, or a report.
"""
import argparse
import plistlib
import sys

# Fields worth echoing back from the device-generated request, if present.
ECHO_KEYS = (
    "ApNonce", "ApNonceSlotID", "SepNonce", "SepNonceSlotID",
    "ApECID", "ApChipID", "ApBoardID", "ApProductionMode", "ApSecurityMode",
    "Cryptex1Nonce", "Cryptex1NonceSlotID",
)


def collect_echoes(req):
    """Pull nonce/identity fields out of a captured request, at any depth."""
    found = {}

    def walk(node):
        if isinstance(node, dict):
            for k, v in node.items():
                if k in ECHO_KEYS and k not in found:
                    found[k] = v
                walk(v)
        elif isinstance(node, list):
            for v in node:
                walk(v)

    walk(req)
    return found


def main() -> int:
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("ticket")
    ap.add_argument("out")
    ap.add_argument("--request", default=None,
                    help="captured TSS request or arguments plist")
    ap.add_argument("--tag", action="append", default=None,
                    help="response tag name (repeatable)")
    args = ap.parse_args()

    ticket = open(args.ticket, "rb").read()
    tags = args.tag or ["Cryptex1,Ticket"]

    resp = {"Status": {"code": 0, "message": "SUCCESS"}}

    if args.request:
        try:
            req = plistlib.load(open(args.request, "rb"))
        except Exception as exc:
            print(f"could not read {args.request}: {exc}", file=sys.stderr)
            return 1
        echoes = collect_echoes(req)
        for k, v in echoes.items():
            resp[k] = v
        print("echoed from request: " + (", ".join(sorted(echoes)) or "(none)"))

    for t in tags:
        resp[t] = ticket

    with open(args.out, "wb") as f:
        plistlib.dump(resp, f)
    print(f"wrote {args.out}; tags: {', '.join(tags)}; ticket {len(ticket)} bytes")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
