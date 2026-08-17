"""Isolate WHY a conn.ident call hangs, one call at a time, with a deadline on each.

The symptom: conn.ident.ping() returns "pong", then vessel_flight_ids(vessel) never
returns - the client sits in select() until Ctrl-C. No error, no exception, no response.
And a second run's ping() works again, so kRPC's update loop is alive. So the call IS
dispatched and its response never comes.

That narrows it to one of a few places, and the ONLY way to tell them apart is to vary one
thing at a time. Each step below adds exactly one ingredient:

    1  bridge.ping                  control: the bridge answers at all
    2  ident.ping                   the Ident service answers - no parameters
    3  ident.available              a property on it
    4  space_center.active_vessel   control: SpaceCenter itself answers
    5  ident.vessel_ids(v)          FIRST cross-service parameter, small string back
    6  ident.part_flight_id(p)      a Part parameter instead of a Vessel
    7  ident.part_flight_ids([p])   a LIST of cross-service objects as a parameter
    8  ident.vessel_flight_ids(v)   the call that hung: list of strings back

READING THE RESULT - this is the whole point of the script:

    5 times out      Passing a class from another service does not survive marshalling at
                     runtime, even though the scanner accepts the signature. The design
                     needs the flat-table fallback: return a row per vessel and join on
                     the client side. Nothing below 5 will tell you more.
    5 ok, 6 times out  Vessel marshals, Part does not. Part's InternalPart is a lookup by
                     flightID rather than a stored reference, which is the obvious suspect.
    7 AND 8 time out, 5 and 6 ok
                     ★ This is what happened on 17/08, and the first reading of it was
                     wrong. 7 takes a list and 8 does not, so the list PARAMETER is not the
                     culprit. What 7 and 8 share is the RETURN type: both returned an
                     array typed as IList of string, while 5 and 6 return a plain string.
                     Returning `new string[n]` hangs; returning a List does not. 7 hung on
                     a one-element result, so it is the type and not the size.
    7 ok, 8 times out  Only the vessel-wide call fails, so suspect its body or the part
                     count rather than the shape of the return.
    all ok           It was transient - a paused game, a scene load, or a modal dialog.
                     Re-run probe_ident.py.

WHY A BAD RETURN TYPE HANGS RATHER THAN ERRORS, since that is the surprising part: the
failure is in encoding the RESPONSE, after the procedure body has already run. There is no
response to send, and the error path cannot send one either. From Python it is
indistinguishable from a dead server - which is exactly why this script exists.

Every step gets its OWN connection, closed after. A kRPC call abandoned mid-flight leaves
the socket with an unread response on it, and reusing that connection would make every
later step lie.

    python python/diag_ident.py
    python python/diag_ident.py --timeout 15      slower machine, or heavy scene
"""

import argparse
import sys
import threading

STEP_WIDTH = 30


def attempt(label, body, timeout, address):
    """Run one probe on a throwaway connection. Never blocks longer than timeout.

    Returns "ok", "timeout", "error" or "skip". The thread is a daemon: if the call really
    is wedged, the interpreter can still exit.
    """
    import krpc

    box = {}

    def run():
        conn = None
        try:
            conn = krpc.connect(name="diag_ident", address=address)
            box["value"] = body(conn)
        except BaseException as exc:            # noqa: BLE001 - reporting, not handling
            box["error"] = exc
        finally:
            if conn is not None:
                try:
                    conn.close()
                except BaseException:           # noqa: BLE001
                    pass

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    thread.join(timeout)

    if thread.is_alive():
        print(f"  {label:<{STEP_WIDTH}} TIMEOUT   no response in {timeout:.0f}s")
        return "timeout"
    if "error" in box:
        exc = box["error"]
        print(f"  {label:<{STEP_WIDTH}} ERROR     {type(exc).__name__}: {exc}")
        return "error"

    value = box.get("value")
    text = repr(value)
    if len(text) > 60:
        text = text[:57] + "..."
    print(f"  {label:<{STEP_WIDTH}} ok        {text}")
    return "ok"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--timeout", type=float, default=8.0,
                    help="seconds to wait per call (default 8)")
    ap.add_argument("--address", default="127.0.0.1")
    args = ap.parse_args()

    try:
        import krpc                              # noqa: F401 - checked here, used in attempt
    except ImportError:
        raise SystemExit("pip install krpc")

    def first_part(conn):
        parts = conn.space_center.active_vessel.parts.all
        if not parts:
            raise RuntimeError("active vessel has no parts")
        return parts[0]

    steps = [
        ("1 bridge.ping",
         lambda c: c.bridge.ping()),
        ("2 ident.ping",
         lambda c: c.ident.ping()),
        ("3 ident.available",
         lambda c: c.ident.available),
        ("4 space_center.active_vessel",
         lambda c: c.space_center.active_vessel.name),
        ("5 ident.vessel_ids(v)",
         lambda c: c.ident.vessel_ids(c.space_center.active_vessel)),
        ("6 ident.part_flight_id(p)",
         lambda c: c.ident.part_flight_id(first_part(c))),
        ("7 ident.part_flight_ids([p])",
         lambda c: c.ident.part_flight_ids([first_part(c)])),
        ("8 ident.vessel_flight_ids(v)",
         lambda c: len(c.ident.vessel_flight_ids(c.space_center.active_vessel))),
    ]

    print(f"Isolating conn.ident, {args.timeout:.0f}s deadline per call, "
          f"one connection each.\n")
    results = {}
    for label, body in steps:
        results[label] = attempt(label, body, args.timeout, args.address)

    # ---- verdict -----------------------------------------------------------
    print("\nVerdict")
    order = list(results)
    first_bad = next((k for k in order if results[k] != "ok"), None)

    if first_bad is None:
        # Deliberately does NOT claim a cause. This branch cannot know whether an
        # earlier failure was fixed in between or was never there, and the first version
        # of this script asserted "transient" - which was wrong the very first time it
        # printed, right after a rebuild that fixed a real bug.
        print("  Every call returned. Nothing to diagnose.")
        print("  If a call was failing before, whatever changed since is what fixed it;")
        print("  if nothing changed, it was transient - a paused game, a scene load or a")
        print("  modal dialog eats RPCs the same way.")
        print("  Next: python python/probe_ident.py --label prelaunch")
        return 0

    n = first_bad.split()[0]
    print(f"  First failure: {first_bad}  ({results[first_bad]})")
    guidance = {
        "1": "The bridge itself is unreachable. Is the kRPC server started, and is the\n"
             "  game in the flight scene rather than paused?",
        "2": "The Ident SERVICE is unreachable although the bridge answers. Check\n"
             "  conn.bridge.plugins and the KRPC.Bridge.Ident lines in KSP.log.",
        "3": "ping works but a property does not, which is strange enough to want the\n"
             "  KSP.log lines around the call.",
        "4": "SpaceCenter itself is not answering, so this is not about Ident at all.",
        "5": "★ Passing a class from another service does not survive marshalling at\n"
             "  runtime, even though kRPC's own scanner accepted the signature. That is the\n"
             "  finding that matters: the design falls back to a flat table - one row per\n"
             "  vessel, joined on the client side - and needs no cross-service parameter.",
        "6": "A Vessel parameter marshals but a Part does not. Part.InternalPart is a\n"
             "  lookup by flightID rather than a stored reference; suspect that first.",
        "7": "Cross-service parameters marshal fine - 5 and 6 proved it. Check whether 8\n"
             "  ALSO timed out: if it did, the list parameter is innocent and the shared\n"
             "  cause is the RETURN type. Returning an array typed as IList hangs; return a\n"
             "  List instead. If 8 passed, then it really is the list parameter.",
        "8": "Everything marshals, so the fault is inside VesselFlightIds or in the shape\n"
             "  of what it returns. Return a List, never an array typed as IList.",
    }
    print("  " + guidance.get(n, "unexpected step - report the table above."))

    if results.get("7 ident.part_flight_ids([p])") != "ok" and \
       results.get("8 ident.vessel_flight_ids(v)") != "ok" and \
       results.get("5 ident.vessel_ids(v)") == "ok":
        print()
        print("  ** BOTH 7 AND 8 FAILED WHILE 5 AND 6 PASSED. **")
        print("  8 takes no list, so the list parameter is not the cause. What 7 and 8 share")
        print("  is returning an IList; 5 and 6 return a plain string. Return a List rather")
        print("  than an array and rebuild.")

    print("\n  Then look at KSP.log for the seconds around the call:")
    print('      findstr /C:"kRPC" /C:"Exception" "<KSP>\\KSP.log" | more')
    print("  An exception raised while kRPC decodes a parameter never reaches the client:")
    print("  from Python it looks exactly like this - a request sent, no reply.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
