"""Probe conn.ident: does a part's flightID survive separation, and an FMRS jump?

That is the whole question. The identity scheme this service exists for records which
flightIDs belong to which booster BEFORE launch, then intersects after separation. It only
works if flightID is stable across both events. The docs say it is; nothing here has
measured it.

    python python/probe_ident.py            take a snapshot, compare with the last one
    python python/probe_ident.py --reset     forget the stored snapshots and start over
    python python/probe_ident.py --label X   name this snapshot (default: auto)

HOW TO USE IT, and the order matters:

    1. On the pad, before launch          python python/probe_ident.py --label prelaunch
    2. After separation, on the booster   python python/probe_ident.py --label separated
    3. After an FMRS jump to the booster  python python/probe_ident.py --label afterjump

Each run prints the intersection with EVERY earlier snapshot. Step 2 answers M1, step 3
answers M1b. A non-empty intersection with `prelaunch` is the whole result.

Snapshots go in python/probe_ident.json, which .gitignore excludes: it is one flight's
measurement, not a fixture. Nothing here writes to KSP - it is a pure observer, and that is
deliberate. A probe that moves what it observes measures itself.

IF THIS HANGS - ping answers and then a call never returns - do not wait on it. Run
`python python/diag_ident.py` instead: it tries the same calls one at a time with a deadline
on each, and the first one to time out names the cause.
"""

import argparse
import json
import os
import sys

STORE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "probe_ident.json")


def snapshot(conn, label):
    """Everything worth knowing about the active vessel's identity, right now."""
    sc = conn.space_center
    vessel = sc.active_vessel
    if vessel is None:
        raise SystemExit("no active vessel - go to the flight scene first")

    # The two calls under test. `vessel_flight_ids` is the set the intersection uses;
    # `vessel_ids` is the FMRS/KSP pair, printed because it is the OTHER thing that is
    # supposed to be stable and it costs one RPC to check.
    ids = conn.ident.vessel_flight_ids(vessel)
    persistent, guid = (conn.ident.vessel_ids(vessel).split("\t") + ["", ""])[:2]

    # Decouple stage per part, from stock kRPC, so a group can be named without this
    # service. -1 means "never decoupled from the vessel".
    parts = vessel.parts.all
    by_stage = {}
    if len(parts) == len(ids):
        # Index alignment is NOT relied on for the result - only for this readout, and
        # only when the two lists came back the same length. The intersection below uses
        # sets, which do not care.
        for part, fid in zip(parts, ids):
            by_stage.setdefault(str(part.decouple_stage), []).append(fid)
    else:
        print(f"  note: {len(parts)} parts but {len(ids)} flightIDs - "
              f"skipping the per-stage readout, the set is still valid")

    return {
        "label": label,
        "vessel": vessel.name,
        "situation": str(vessel.situation),
        "loaded": vessel.loaded,
        "packed": vessel.packed,
        "met": round(vessel.met, 1),
        "ut": round(sc.ut, 1),
        "persistent_id": persistent,
        "guid": guid,
        "n_parts": len(parts),
        "flight_ids": sorted(ids),
        "by_decouple_stage": {k: sorted(v) for k, v in sorted(by_stage.items())},
    }


def report(snap, earlier):
    print(f"\n=== {snap['label']} ===")
    print(f"  vessel        {snap['vessel']}  [{snap['situation']}]"
          f"  loaded={snap['loaded']} packed={snap['packed']}")
    print(f"  met           T+{snap['met']}   ut={snap['ut']}")
    print(f"  persistent_id {snap['persistent_id']}")
    print(f"  guid          {snap['guid']}")
    print(f"  parts         {snap['n_parts']}, {len(snap['flight_ids'])} flightID(s)")
    for stage, fids in snap["by_decouple_stage"].items():
        shown = ", ".join(fids[:6]) + (" ..." if len(fids) > 6 else "")
        print(f"    decouple_stage {stage:>3}  {len(fids):>3} part(s)  {shown}")

    if not earlier:
        print("\n  no earlier snapshot to compare with - run this again after separation.")
        return

    mine = set(snap["flight_ids"])
    print()
    for old in earlier:
        theirs = set(old["flight_ids"])
        both = mine & theirs
        smaller = min(len(mine), len(theirs))

        # Three outcomes, and the first version of this script called two of them the same
        # thing. Zero overlap between two POST-separation snapshots is the CORRECT answer:
        # they are disjoint pieces of one stack, and a shared part would be the anomaly. It
        # only means the invariant failed when one side is the pre-launch reference, which
        # contains every part there will ever be.
        reference = old["label"] == "prelaunch" or snap["label"] == "prelaunch"
        if both and len(both) == smaller:
            verdict = "SUBSET - same lineage, flightID held"
        elif both:
            verdict = f"PARTIAL - {len(both)}/{smaller} of the smaller set, look at this"
        elif reference:
            verdict = "!! NO OVERLAP with the pre-launch set - the invariant FAILED"
        else:
            verdict = "disjoint - expected between two different sub-vessels"

        print(f"  vs {old['label']:<12} {len(both):>3} shared / "
              f"{len(theirs)} then / {len(mine)} now   {verdict}")
        if both:
            # The kept ids are the answer. Showing a few makes it obvious they are the
            # same numbers rather than a coincidence of counts.
            sample = ", ".join(sorted(both)[:6]) + (" ..." if len(both) > 6 else "")
            print(f"     kept: {sample}")
        if old["persistent_id"] != snap["persistent_id"]:
            print(f"     persistent_id CHANGED  {old['persistent_id']} -> "
                  f"{snap['persistent_id']}   (expected across a separation)")
        if old["guid"] != snap["guid"]:
            print(f"     guid CHANGED           {old['guid']} -> {snap['guid']}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default=None, help="name for this snapshot")
    ap.add_argument("--reset", action="store_true", help="delete stored snapshots and exit")
    ap.add_argument("--address", default="127.0.0.1")
    args = ap.parse_args()

    if args.reset:
        if os.path.exists(STORE):
            os.remove(STORE)
            print(f"removed {STORE}")
        else:
            print("nothing stored")
        return 0

    try:
        import krpc
    except ImportError:
        raise SystemExit("pip install krpc")

    conn = krpc.connect(name="probe_ident", address=args.address)

    if not conn.bridge.has_plugin("Ident"):
        raise SystemExit(
            "conn.ident is not there. Build and deploy KRPC.Bridge.Ident, restart KSP, "
            "and check conn.bridge.plugins for the reason.")
    if not conn.ident.available:
        raise SystemExit("conn.ident.available is False - see conn.bridge.plugins")
    print(f"conn.ident.ping() -> {conn.ident.ping()}")

    stored = []
    if os.path.exists(STORE):
        with open(STORE, encoding="utf-8") as fh:
            stored = json.load(fh)

    label = args.label or f"snap{len(stored) + 1}"
    snap = snapshot(conn, label)
    report(snap, stored)

    stored.append(snap)
    with open(STORE, "w", encoding="utf-8") as fh:
        json.dump(stored, fh, indent=2)
    print(f"\n  saved to {STORE}  ({len(stored)} snapshot(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
