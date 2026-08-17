#!/usr/bin/env python3
"""Run the in-game acceptance checks for KRPC.Bridge and report pass/fail.

    python python/test_inflight.py            # everything that is safe to run
    python python/test_inflight.py --jump     # ... including the FMRS jump

A check whose precondition is missing is skipped, not failed, and says what it needed -
so an incomplete run still tells you something.

Run it after every deploy. It is the step nothing else substitutes for - the signature
scan in build.cmd proves the kRPC server will start, not that the mod works - and doing
it by hand in a REPL is slow enough that you stop repeating it after a change.

SAFETY. Nothing here recovers, deletes, reverts or stages. The only action that alters
the game is the FMRS jump, and it is behind --jump precisely because it reloads the
flight scene and takes 5 to 20 seconds. Everything else is a read or a setting the script
puts back.

Exit code is the number of failures, so it can gate a release script.
"""

import argparse
import sys
import time


# ---------------------------------------------------------------- reporting

PASS, FAIL, SKIP, INFO = "PASS", "FAIL", "SKIP", "  ->"
results = []


def record(status, title, detail=""):
    results.append((status, title, detail))
    mark = {PASS: "  ok  ", FAIL: " FAIL ", SKIP: " skip "}.get(status, "      ")
    print(f"[{mark}] {title}")
    if detail:
        for line in str(detail).splitlines():
            print(f"          {line}")


def check(title, fn, needs=None):
    """Run one check.

    `fn` returns (ok, detail), and ok may be None to mean "the precondition for this
    check is not set up" - which is a skip, not a failure. The distinction matters:
    a red FAIL for "you have not put a camera on air" trains you to ignore red, and
    then a real failure goes unnoticed too.
    """
    if needs is not None and not needs[0]:
        record(SKIP, title, needs[1])
        return None
    try:
        ok, detail = fn()
    except Exception as exc:
        record(FAIL, title, f"{type(exc).__name__}: {exc}")
        return False
    record(SKIP if ok is None else (PASS if ok else FAIL), title, detail)
    return ok


# ---------------------------------------------------------------- the checks

def run(conn, do_jump):
    sc = conn.space_center
    bridge = conn.bridge

    print("\n=== Core ===")

    check("bridge.ping() answers pong",
          lambda: (bridge.ping() == "pong", bridge.ping()))

    def plugins_listed():
        rows = [r.split("\t") for r in bridge.plugins]
        if not rows:
            return False, ("no plugin registered - this is the resolution-order bug: "
                           "Core resolved before the plugins had registered")
        detail = "\n".join(f"{r[0]:<8} {'ok' if r[1] == '1' else 'MISSING':<8} mod {r[3] or '-'}"
                           for r in rows)
        return len(rows) == 3, detail
    check("all three plugins registered", plugins_listed)

    def events_flowing():
        before = bridge.events_recorded
        bridge.mark("test.marker", "test_inflight")
        return bridge.events_recorded > before, f"{before} -> {bridge.events_recorded}"
    check("the event log records", events_flowing)

    def event_signal():
        evt = bridge.on_event("test.signal")
        try:
            with evt.condition:
                bridge.mark("test.signal", "wake up")
                got = evt.wait(timeout=5.0)
            return got is not False, "signal fired" if got is not False else "timed out"
        finally:
            evt.remove()
    check("bridge.on_event wakes on a marker", event_signal)

    # ------------------------------------------------------------- FMRS
    print("\n=== FMRS ===")
    fmrs_ok = conn.fmrs.available
    if not fmrs_ok:
        record(SKIP, "FMRS checks", conn.fmrs.diagnostics)
    else:
        check("armed is readable", lambda: (isinstance(conn.fmrs.armed, bool),
                                            f"armed={conn.fmrs.armed}"))

        def dropped_consistent():
            dropped = conn.fmrs.dropped_vessels
            saves = conn.fmrs.dropped_saves
            if not dropped:
                return None, "no dropped stage tracked - separate one first"
            missing = [v for v in dropped if v not in saves]
            if missing:
                return False, f"{len(missing)} stage(s) have no save name"
            return True, f"{len(dropped)} stage(s), {len(set(saves.values()))} batch(es)"
        dropped_present = check("dropped stages have a batch key", dropped_consistent)

        check("separation times parse as numbers",
              lambda: (all(_is_float(v) for v in conn.fmrs.separation_times.values()),
                       conn.fmrs.separation_times),
              needs=(dropped_present, "needs a dropped stage"))

        def state_readable():
            vid = next(iter(conn.fmrs.dropped_vessels))
            state = conn.fmrs.vessel_state(vid)
            known = {"NONE", "FLY", "LANDED", "DESTROYED", "RECOVERED"}
            return state in known, f"{vid[:8]} -> {state}"
        check("vessel_state returns a known name", state_readable,
              needs=(dropped_present, "needs a dropped stage"))

    # ------------------------------------------------------------- OCISLY
    print("\n=== OCISLY ===")
    if not conn.ocisly.available:
        record(SKIP, "OCISLY checks", conn.ocisly.diagnostics)
    else:
        def names_have_tokens():
            cams = conn.ocisly.hullcams
            if not cams:
                return None, "no Hullcam on any loaded vessel"
            tokenless = [c for c in cams if "@" not in c]
            if tokenless and conn.ocisly.disambiguate_names:
                return False, ("these carry no @flightID token, so they cannot be "
                               "recognised after a reload:\n" + "\n".join(tokenless))
            return True, "\n".join(cams)
        check("every Hullcam carries an @flightID token", names_have_tokens)

        def snapshot_tracks_streaming():
            streaming = conn.ocisly.streaming
            remembered = conn.ocisly.remembered
            if not streaming:
                return None, ("nothing is streaming - open a camera in OCISLY's window "
                              "and enable its stream, otherwise neither this check nor "
                              "the restore after a jump proves anything")
            # The snapshot is refreshed twice a second; give it a beat.
            time.sleep(1.0)
            remembered = conn.ocisly.remembered
            if not remembered:
                return False, ("streaming but the snapshot is empty - the restore would "
                               "have nothing to work with")
            return True, f"streaming {streaming}\nremembered {remembered}"
        check("the snapshot follows what is on air", snapshot_tracks_streaming)

    # ------------------------------------------------------------- MechJeb
    print("\n=== MechJeb ===")
    if not conn.mech_jeb.available:
        record(SKIP, "MechJeb checks", conn.mech_jeb.diagnostics)
    elif not conn.mech_jeb.on_vessel:
        record(SKIP, "MechJeb checks", "this craft carries no running MechJeb part")
    else:
        check("the installed build exposes its settings by name",
              lambda: (len(conn.mech_jeb.ascent_setting_names) > 0,
                       f"{len(conn.mech_jeb.ascent_setting_names)} settings, "
                       f"{len(conn.mech_jeb.ascent_flag_names)} flags"))

        def autostage_releases_pool():
            """THE member the MechJeb service exists for. Restores what it changed."""
            was = conn.mech_jeb.autostage
            try:
                conn.mech_jeb.autostage = False
                users = conn.mech_jeb.staging_users
                if users == -1:
                    return False, "the staging user pool could not be read at all"
                return users == 0, f"staging_users = {users} (0 expected)"
            finally:
                conn.mech_jeb.autostage = was
        check("autostage=False empties the staging pool", autostage_releases_pool)

        # --- 1.1.0: the whole of MechJeb, reached by name.
        #
        # These matter more than they look. Everything below is new reflection against a
        # mod that renames things, and the failure mode is not an exception - it is an
        # empty list, or a member that reads back a plausible zero. So each check asserts
        # something a broken resolve could not fake.

        def modules_resolve():
            names = conn.mech_jeb.modules
            if not names:
                return False, ("no module resolved - ComputerModule was not identified, so "
                               "every by-name accessor is dead")
            # Ascent and Staging have been on MechJebCore across every refactor so far.
            missing = [n for n in ("Ascent", "Staging") if n not in names]
            if missing:
                return False, f"expected {missing} among {len(names)} modules: {names}"
            return True, f"{len(names)} modules: {', '.join(names)}"
        modules_ok = check("MechJebCore's modules resolve by name", modules_resolve)

        def describe_is_populated():
            rows = conn.mech_jeb.describe_module("Staging")
            if not rows:
                return False, "describe_module('Staging') is empty - member walk found nothing"
            channels = {r.split("\t")[1] for r in rows if "\t" in r}
            unsupported = [r.split("\t")[0] for r in rows if "\tunsupported\t" in r]
            return "number" in channels and "flag" in channels, (
                f"{len(rows)} members, channels {sorted(channels)}"
                + (f"\nunsupported: {unsupported}" if unsupported else ""))
        check("describe_module classifies members", describe_is_populated,
              needs=(modules_ok, "needs the module walk"))

        def staging_knobs_round_trip():
            """The knobs 1.0.0 could not reach at all. Restores what it changed."""
            was = conn.mech_jeb.setting("Staging", "AutostageLimit")
            try:
                conn.mech_jeb.set_setting("Staging", "AutostageLimit", 3)
                got = conn.mech_jeb.setting("Staging", "AutostageLimit")
                return abs(got - 3) < 1e-6, f"AutostageLimit {was} -> set 3 -> read {got}"
            finally:
                conn.mech_jeb.set_setting("Staging", "AutostageLimit", was)
        check("a setting on a module other than Ascent round-trips", staging_knobs_round_trip,
              needs=(modules_ok, "needs the module walk"))

        def enum_channel_works():
            options = conn.mech_jeb.enum_options("Thrust", "Tmode")
            if not options:
                return None, "this MechJeb exposes no Tmode enum on Thrust"
            current = conn.mech_jeb.enum_value("Thrust", "Tmode")
            return current in options, f"Tmode = {current}, of {options}"
        check("enum settings read back as names", enum_channel_works,
              needs=(modules_ok, "needs the module walk"))

        def backing_fields_hidden():
            """_autostage next to Autostage: writing the field skips the pool registration."""
            names = conn.mech_jeb.ascent_flag_names
            leaked = [n for n in names if n.startswith("_")]
            return not leaked, ("these backing fields are still advertised and would "
                                f"silently do nothing: {leaked}" if leaked
                                else f"{len(names)} flags, none of them a backing field")
        check("MechJeb's public backing fields stay out of the name lists", backing_fields_hidden)

        def predictor_is_readable():
            if not conn.mech_jeb.landing_predicted:
                return None, ("no suicide-burn solution right now - the predictor only "
                              "solves on a descent towards a surface")
            lat = conn.mech_jeb.landing_latitude
            lon = conn.mech_jeb.landing_longitude
            return -90 <= lat <= 90 and -180 <= lon <= 360, (
                f"impact {lat:.4f}, {lon:.4f} | ignition in "
                f"{conn.mech_jeb.ignition_countdown:.1f}s | slope {conn.mech_jeb.landing_slope:.1f}deg")
        check("the landing predictor gives a sane impact point", predictor_is_readable)

        def planner_catalogue():
            ops = conn.mech_jeb.maneuver_operations
            if not ops:
                return False, ("no maneuver operation resolved - Operation or "
                               "GetAvailableOperations did not bind")
            refs = conn.mech_jeb.maneuver_time_references("OperationCircularize")
            return "OperationCircularize" in ops, (
                f"{len(ops)} operations; circularize accepts {refs}")
        check("the maneuver planner enumerates its operations", planner_catalogue)

    # ------------------------------------------------------------- the jump
    print("\n=== FMRS jump ===")
    if not do_jump:
        record(SKIP, "jump and restore", "pass --jump to run it (reloads the scene)")
        return
    if not fmrs_ok:
        record(SKIP, "jump and restore", "FMRS unavailable")
        return

    dropped = conn.fmrs.dropped_vessels
    if not dropped:
        record(SKIP, "jump and restore", "no dropped stage to jump to")
        return
    if conn.fmrs.switched_to_dropped:
        record(SKIP, "jump and restore",
               "already flying a dropped stage - jump_to_main() first")
        return

    vid, name = next(iter(dropped.items()))
    before_streaming = list(conn.ocisly.streaming) if conn.ocisly.available else []
    before_remembered = list(conn.ocisly.remembered) if conn.ocisly.available else []
    print(f"{INFO} jumping to {name}")
    print(f"{INFO} streaming before: {before_streaming or '(nothing)'}")

    def jump_arrives():
        conn.fmrs.jump_to_vessel(vid)
        return conn.fmrs.switched_to_dropped, f"now flying {sc.active_vessel.name}"
    arrived = check("jump_to_vessel arrives on the dropped stage", jump_arrives)

    if arrived:
        # The restore coroutine waits restore_delay then a repaint. Give it room.
        wait = conn.ocisly.restore_delay + 3.0 if conn.ocisly.available else 3.0
        print(f"{INFO} waiting {wait:.0f}s for the scene restore")
        time.sleep(wait)

        if conn.ocisly.available:
            def cameras_restored():
                now = conn.ocisly.streaming
                report = conn.ocisly.last_restore
                if not before_streaming:
                    return False, ("nothing was on air before the jump, so this proves "
                                   "nothing\n" + report)
                return bool(now), (f"before {before_streaming}\n"
                                   f"after  {now}\n"
                                   f"snapshot was {before_remembered}\n"
                                   f"{report}")
            check("cameras are back on air after the jump", cameras_restored)

        check("jump_to_main returns to the mission",
              lambda: (_jump_back(conn), f"now flying {sc.active_vessel.name}"))


def _jump_back(conn):
    conn.fmrs.jump_to_main()
    return not conn.fmrs.switched_to_dropped


def _is_float(text):
    try:
        float(text)
        return True
    except (TypeError, ValueError):
        return False


# ---------------------------------------------------------------- entry point

def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--jump", action="store_true",
                        help="also run the FMRS jump, which reloads the flight scene")
    args = parser.parse_args()

    # Imported here, not at module level, so --help works on a machine without krpc.
    try:
        import krpc
    except ImportError:
        sys.exit("krpc is not installed:  pip install 'krpc>=0.6,<0.7'")

    try:
        conn = krpc.connect(name="bridge-acceptance")
    except Exception as exc:
        sys.exit(f"cannot reach the kRPC server: {exc}\n"
                 "Is KSP running with the kRPC server started?")

    print(f"kRPC {conn.krpc.get_status().version}, scene {conn.krpc.game_scene}")
    if not hasattr(conn, "bridge"):
        sys.exit("the Bridge service is missing - is KRPC.Bridge.Core.dll in GameData?\n"
                 "If EVERY kRPC service is missing, some assembly has a signature kRPC\n"
                 "rejects and that takes the whole server down. Run build.cmd.")

    run(conn, args.jump)

    failures = [r for r in results if r[0] == FAIL]
    skipped = [r for r in results if r[0] == SKIP]
    print(f"\n{len(results) - len(failures) - len(skipped)} passed, "
          f"{len(failures)} failed, {len(skipped)} skipped")
    for _, title, detail in failures:
        print(f"  FAILED: {title}")
    if skipped and not failures:
        print("  nothing failed, but a skip is a hole, not a pass:")
        for _, title, _ in skipped:
            print(f"    untested: {title}")
    conn.close()
    return len(failures)


if __name__ == "__main__":
    raise SystemExit(main())
