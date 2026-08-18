"""Probe the cameras: can two lateral boosters stream at the same time, and which is which?

Two questions, and only the second one is hard.

CAN BOTH STREAM AT ONCE? Yes, in principle - OCISLY enumerates every LOADED vessel, not
just the active one, so a dropped booster still in range has its camera listed like any
other. The condition is the word "loaded": a vessel out of range has no instantiated parts
and therefore no camera, and nothing on the bridge side can conjure one. This probe prints
the loaded/packed state of every vessel next to its cameras, so the answer is visible rather
than assumed.

WHICH IS WHICH? That is the real problem. Two boosters built from the same craft file carry
the same vessel name AND the same cameraName - the latter is a KSPField that comes from the
part config, not the instance. So on the wire both cameras look alike, and OCISLY tells them
apart only by the "@flightID" token it appends to each name.

Until conn.ident existed, that token was an opaque number: it said the two cameras were
different without saying which booster either belonged to. This probe closes that loop. It
reads every loaded vessel's flightID set from conn.ident and resolves each camera's token to
the vessel that owns the part. Exactly, not by name and not by guessing.

    python python/probe_cams.py                 look
    python python/probe_cams.py --rearm         also re-open every camera it found
    python python/probe_cams.py --rearm-token @1898164639     just that one

A NOTE ON COST, because it is easy to arm everything and then wonder about the framerate:
each armed camera costs three full scene renders per cycle. Two is a considered choice; six
is not.

Nothing here writes to KSP except --rearm, which is the one flag that does.
"""

import argparse
import sys

TOKEN_SEP = "@"


def token_of(name):
    """The '@flightID' identity token OCISLY appends, or None.

    Split on the LAST separator: a vessel or camera named with an '@' would otherwise
    steal the parse, and vessel names are user input.
    """
    if not name or TOKEN_SEP not in name:
        return None
    return TOKEN_SEP + name.rsplit(TOKEN_SEP, 1)[1]


def loaded_vessels(conn):
    """Every loaded vessel, with its flightID set. Unloaded ones are reported separately.

    ★ EVERY VESSEL GETS A KEY THAT IS NOT ITS NAME, and that is the whole lesson of this
    file. The first version grouped cameras by owner NAME - and on a two-booster flight the
    twins are both called "<craft> Probe", so it merged them and reported "2 distinct
    vessels" where there were three. It made exactly the mistake this probe exists to avoid,
    in the line right after resolving the owner correctly.

    persistentId from conn.ident is the key. It differs between twins, unlike the name, and
    it is what makes the display honest.
    """
    have_ident = hasattr(conn, "ident") and conn.ident.available
    loaded, unloaded = [], []
    for vessel in conn.space_center.vessels:
        try:
            entry = {
                "vessel": vessel,
                "name": vessel.name,
                "type": str(vessel.type),
                "situation": str(vessel.situation),
                "loaded": vessel.loaded,
                "packed": vessel.packed,
                "ids": set(),
                "key": None,
            }
        except RuntimeError:
            # A vessel destroyed between the list call and this read. Not an error.
            continue
        if not entry["loaded"]:
            unloaded.append(entry)
            continue
        if have_ident:
            try:
                entry["ids"] = set(conn.ident.vessel_flight_ids(vessel))
            except RuntimeError as exc:
                entry["ids"] = set()
                entry["note"] = f"conn.ident refused: {exc}"
            try:
                entry["key"] = conn.ident.vessel_ids(vessel).split("\t")[0]
            except RuntimeError:
                entry["key"] = None
        loaded.append(entry)

    # A label that tells twins apart on screen. Only the ambiguous ones get decorated, so a
    # single-booster flight reads exactly as before.
    counts = {}
    for entry in loaded:
        counts[entry["name"]] = counts.get(entry["name"], 0) + 1
    seen = {}
    for entry in loaded:
        name = entry["name"]
        if counts[name] > 1:
            seen[name] = seen.get(name, 0) + 1
            entry["label"] = f"{name}  #{seen[name]} ({len(entry['ids'])}p)"
        else:
            entry["label"] = name
        # Fall back to the label when persistentId is unavailable, never to the bare name:
        # the label at least carries the part count.
        if entry["key"] is None:
            entry["key"] = entry["label"]
    return loaded, unloaded, have_ident


def owner_of(token, loaded):
    """The loaded vessel whose parts include this token's flightID, or None.

    This is the whole point of the probe. The token is a part flightID; a vessel's flightID
    set is what conn.ident.vessel_flight_ids returns; membership is the answer. No name
    comparison anywhere, which is what makes it work on twins.
    """
    if not token:
        return None
    fid = token.lstrip(TOKEN_SEP)
    for entry in loaded:
        if fid in entry["ids"]:
            return entry
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rearm", action="store_true",
                    help="re-open every camera found (filters on the token, the only "
                         "string that matches both phases of OCISLY's rearm)")
    ap.add_argument("--rearm-token", default=None, metavar="@ID",
                    help="re-open just this one")
    ap.add_argument("--address", default="127.0.0.1")
    args = ap.parse_args()

    try:
        import krpc
    except ImportError:
        raise SystemExit("pip install krpc")

    conn = krpc.connect(name="probe_cams", address=args.address)

    if not hasattr(conn, "ocisly") or not conn.ocisly.available:
        raise SystemExit(
            "conn.ocisly is not available. Is OCISLY installed and the bridge deployed? "
            "Run check_bridge.py.")

    print(f"disambiguate_names = {conn.ocisly.disambiguate_names}"
          f"   auto_rearm = {conn.ocisly.auto_rearm}")
    if not conn.ocisly.disambiguate_names:
        print("  !! With this off, two identical cameras get no token and CANNOT be told")
        print("     apart. Turn it on: conn.ocisly.disambiguate_names = True")

    loaded, unloaded, have_ident = loaded_vessels(conn)
    if not have_ident:
        print("\n  note: conn.ident is absent, so cameras cannot be resolved to a vessel.")
        print("        Everything else below still works.")

    # ---- vessels -----------------------------------------------------------
    print(f"\nLoaded vessels ({len(loaded)})")
    twins = [e for e in loaded if e["label"] != e["name"]]
    for entry in loaded:
        print(f"  {entry['label']:<40} [{entry['situation']:<12}] "
              f"packed={str(entry['packed']):<5} {len(entry['ids']):>3} flightID(s)")
        if entry.get("note"):
            print(f"      {entry['note']}")
    if twins:
        print(f"  ** {len(twins)} vessels share a name. ** That is the twin-booster case, and")
        print("     the only thing that separates them is the flightID set - not the name,")
        print("     which is why the cameras below are grouped by persistentId.")
    if unloaded:
        print(f"\n  {len(unloaded)} vessel(s) NOT loaded - they have no camera at all:")
        for entry in unloaded[:8]:
            print(f"      {entry['name']}  [{entry['situation']}]")
        print("      A booster out of range cannot be filmed. physics_range is what keeps")
        print("      it loaded, and the autopilot is what sets it.")

    # ---- cameras -----------------------------------------------------------
    # hullcams is the richest list: "<vessel>.<cameraName>", and cameraName carries the
    # token once disambiguate_names has run. streaming carries the tracker's own name.
    hullcams = list(conn.ocisly.hullcams)
    streaming = list(conn.ocisly.streaming)
    remembered = list(conn.ocisly.remembered)
    streaming_tokens = {token_of(s) for s in streaming} - {None}

    print(f"\nHullcam parts ({len(hullcams)})")
    if not hullcams:
        print("  none. No loaded vessel carries a Hullcam part.")

    # Grouped by persistentId, NEVER by name - see loaded_vessels().
    by_vessel = {}
    for name in hullcams:
        token = token_of(name)
        owner = owner_of(token, loaded)
        key = owner["key"] if owner else "(unresolved)"
        by_vessel.setdefault(key, []).append((name, token, owner))
        live = "STREAMING" if token in streaming_tokens else "idle"
        kept = "remembered" if token in remembered else ""
        resolved = owner["label"] if owner else ("no owner found" if token else "NO TOKEN")
        print(f"  {name}")
        print(f"      token {token or '-':<14} -> {resolved:<36} {live} {kept}")

    # ---- the verdict -------------------------------------------------------
    print("\nVerdict")
    distinct = [k for k in by_vessel if k != "(unresolved)"]
    labels = {}
    for entry in loaded:
        labels[entry["key"]] = entry["label"]
    print(f"  cameras             {len(hullcams)}")
    print(f"  on distinct vessels {len(distinct)}")
    for key in distinct:
        cams = [t for _, t, _ in by_vessel[key] if t]
        print(f"      {labels.get(key, key):<40} {len(cams)} camera(s)")
    print(f"  streaming now       {len(streaming)}")

    if len(hullcams) >= 2 and len(distinct) >= 2:
        print("\n  ** CAMERAS ON MORE THAN ONE VESSEL, ALL RESOLVED BY TOKEN. **")
        print("  They can stream at once, and each is attributable to its own booster -")
        print("  including twins that share a name, which is the case the token exists for.")
        print("  Pin them in the overlay's proxy so the OBS assignment survives a restart:")
        for key in distinct:
            for _, token, _ in by_vessel[key]:
                if token:
                    print(f"      --alias {token}=<name you want>   # {labels.get(key, key)}")
    elif len(hullcams) >= 2 and len(distinct) < 2:
        print("\n  Two cameras, but they resolve to fewer than two vessels. Either they are")
        print("  on the same craft, or a token did not resolve - check the lines above.")
    elif len(hullcams) == 1:
        print("\n  One camera. Separate the boosters and run this again while both are loaded.")

    if len(hullcams) > 2:
        print(f"\n  {len(hullcams)} cameras. Each ARMED one costs three full scene renders")
        print("  per cycle, so arm what you will show and no more.")

    # ---- optional rearm ----------------------------------------------------
    if args.rearm or args.rearm_token:
        # The token is the only string that matches BOTH phases of OCISLY's rearm: the
        # open phase filters on "<vessel>.<cameraName>", the enable phase on the tracker's
        # own name. A filter on either half alone silently does half the job.
        targets = [args.rearm_token] if args.rearm_token else \
                  sorted({token_of(n) for n in hullcams} - {None})
        if not targets:
            print("\n  nothing to rearm: no camera carried a token.")
            return 0
        print(f"\nRearming {len(targets)} camera(s) - the HUD flashes, this is normal")
        for token in targets:
            conn.ocisly.rearm(token)
            print(f"  rearmed {token}")
        print(f"  streaming now: {len(conn.ocisly.streaming)}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
