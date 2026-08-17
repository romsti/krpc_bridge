"""Smoke test for KRPC.Bridge.

Run this the moment the DLLs are in GameData and KSP is in a flight scene. It answers,
in order: is kRPC alive, did the Core load, which plugins resolved and why not, and what
would a jump target look like right now.

    python python/check_bridge.py

Nothing here jumps, recovers or touches the flight. It only reads.
"""

from __future__ import annotations

import sys

try:
    import krpc
except ImportError:  # pragma: no cover
    sys.exit("krpc is not installed:  pip install 'krpc>=0.6,<0.7'")


def line(label: str, value: object) -> None:
    print(f"  {label:<24} {value}")


def _safe(read, default):
    """Read one member, tolerating the one that did not resolve in this mod build.

    Members are resolved individually on the C# side precisely so a rename costs you
    that member rather than the service. Swallowing the error here is what makes that
    design visible: the report still prints, minus one line.
    """
    try:
        return read()
    except (RuntimeError, ValueError):
        return default


def main() -> int:
    print("Connecting to kRPC...")
    try:
        conn = krpc.connect(name="bridge-check")
    except Exception as exc:
        sys.exit(f"cannot reach the kRPC server: {exc}\n"
                 "Is KSP running with the kRPC server started?")

    status = conn.krpc.get_status()
    print("\nkRPC")
    line("server version", status.version)
    line("client version", krpc.__version__)
    line("game scene", conn.krpc.game_scene)

    # 1. Did the Core load? Note that a malformed service signature in ANY loaded
    #    assembly disables the whole kRPC server, so getting this far already proves
    #    every signature in the install is one kRPC accepts.
    print("\nCore")
    if not hasattr(conn, "bridge"):
        print("  Bridge service NOT FOUND.")
        print("  - is KRPC.Bridge.Core.dll in GameData/KRPC.Bridge/ ?")
        print("  - search KSP.log for '[KRPC.Bridge]' and for 'kRPC Service Error'")
        print("  - if EVERY kRPC service is missing, some assembly has a signature")
        print("    kRPC rejects; that takes the whole server down. Run build.cmd,")
        print("    whose scan step names it.")
        return 1
    line("bridge.ping()", conn.bridge.ping())
    line("core version", conn.bridge.version)
    line("services", ", ".join(
        s for s in ("bridge", "fmrs", "ocisly", "mech_jeb") if hasattr(conn, s)))
    line("events recorded", conn.bridge.events_recorded)

    print("\nPlugins")
    for row in conn.bridge.plugins:
        name, ok, version, mod_version, report = (row.split("\t") + [""] * 5)[:5]
        state = "ok" if ok == "1" else "MISSING"
        print(f"  {name:<10} {state:<8} plugin {version:<10} mod {mod_version or '-':<12}")
        if ok != "1":
            print(f"      {report}")

    if not hasattr(conn, "fmrs"):
        print("\n  The FMRS plugin assembly did not load at all. If KRPC.Bridge.Fmrs.dll")
        print("  is present, check KSP.log: KSP skips a plugin whose KSPAssemblyDependency")
        print("  on KRPC.Bridge.Core cannot be satisfied, and says so.")
        return 1

    # 2. FMRS.
    print("\nFMRS")
    line("available", conn.fmrs.available)
    if conn.fmrs.available:
        line("active in scene", conn.fmrs.active)
        line("jump in progress", conn.fmrs.jump_in_progress)

        # The three flags that decide whether a booster ever appears below. Reading
        # them here turns "dropped_vessels is empty" from a mystery into an answer.
        for label, read in (("armed", lambda: conn.fmrs.armed),
                            ("has launched", lambda: conn.fmrs.has_launched),
                            ("flying a dropped stage", lambda: conn.fmrs.switched_to_dropped),
                            ("auto-recover", lambda: conn.fmrs.auto_recover),
                            ("track parachutes", lambda: conn.fmrs.track_parachutes),
                            ("control uncontrollable", lambda: conn.fmrs.control_uncontrollable)):
            try:
                line(label, read())
            except RuntimeError as exc:
                # Names one member, not the whole service: that is the point of
                # resolving them separately.
                line(label, f"unreadable: {exc}")

        try:
            line("main vessel id", conn.fmrs.main_vessel_id or "(none yet)")
            dropped = conn.fmrs.dropped_vessels
            line("dropped stages", len(dropped))
            if not dropped:
                print("      (none - separate a stage with FMRS armed, then re-run)")
            else:
                # Group by FMRS save file. One save per STAGING EVENT, so stages
                # sharing a save came off together: that is the batch. Several ids
                # being listed at once does NOT mean they were dropped together.
                saves = conn.fmrs.dropped_saves
                pids = conn.fmrs.dropped_persistent_ids
                times = _safe(lambda: conn.fmrs.separation_times, {})
                kerbals = _safe(lambda: conn.fmrs.kerbals_aboard, {})

                batches: dict[str, list[str]] = {}
                for vid in dropped:
                    batches.setdefault(saves.get(vid, "?"), []).append(vid)
                line("batches (= staging events)", len(batches))

                for save, members in sorted(
                        batches.items(), key=lambda kv: float(times.get(kv[0], 0) or 0)):
                    tag = "SAME DROP" if len(members) > 1 else "alone"
                    when = times.get(save)
                    stamp = f"  ut={float(when):.1f}" if when else "  ut=?"
                    print(f"      save '{save}'{stamp}  -> {len(members)} stage(s)  [{tag}]")
                    for vid in sorted(members):
                        pid = pids.get(vid)
                        pid_text = f"persistent_id={pid}" if pid else "GONE (destroyed or recovered)"
                        aboard = [k for k, v in kerbals.items() if v == vid]
                        state = _safe(lambda: conn.fmrs.vessel_state(vid), "?")
                        print(f"          {vid}  {dropped[vid]}  [{state}]")
                        print(f"          {'':36}{pid_text}")
                        if aboard:
                            print(f"          {'':36}crew: {', '.join(aboard)}")

                unresolved = [v for v in dropped if v not in pids]
                if unresolved:
                    print(f"      {len(unresolved)} stage(s) have no persistent_id - "
                          "KSP no longer has them in FlightGlobals at all (destroyed or "
                          "recovered), so retrying will not help.")

            ledger = _safe(lambda: conn.fmrs.recovery_report(), [])
            if ledger:
                line("recovery ledger", f"{len(ledger)} row(s)")
                for row in ledger[:12]:
                    category, key, value = (row.split("\t") + ["", "", ""])[:3]
                    print(f"      {category:<14} {key[:34]:<34} {value}")
                if len(ledger) > 12:
                    print(f"      ... and {len(ledger) - 12} more")
        except RuntimeError as exc:
            line("state", f"unreadable: {exc}")
    else:
        print("      FMRS not resolved. What the bridge found:")
        print(f"      {conn.fmrs.diagnostics}")

    # 3. OCISLY.
    print("\nOCISLY")
    line("available", conn.ocisly.available)
    if conn.ocisly.available:
        try:
            line("window open", conn.ocisly.window_open)
            hullcams = conn.ocisly.hullcams
            line("hullcams found", len(hullcams))
            for key in hullcams:
                print(f"      {key}")
            tracked = conn.ocisly.cameras
            line("tracked by OCISLY", len(tracked))
            for cid, name in tracked.items():
                print(f"      id={cid}  {name}")
            streaming = conn.ocisly.streaming
            line("streaming now", len(streaming))
            for name in streaming:
                print(f"      {name}")
            line("auto-rearm", conn.ocisly.auto_rearm)
            line("unique naming", conn.ocisly.disambiguate_names)
            remembered = conn.ocisly.remembered
            line("will restore", ", ".join(remembered) or "(nothing remembered yet)")
            line("last scene restore", conn.ocisly.last_restore)
            line("auto-hide HUD", conn.ocisly.hide_ui_on_scene_load)
        except RuntimeError as exc:
            line("state", f"unreadable: {exc}")
    else:
        print("      OCISLY not resolved. What the bridge found:")
        print(f"      {conn.ocisly.diagnostics}")
        print("      (send me this line — it names the exact member that is missing)")

    # 4. MechJeb.
    print("\nMechJeb")
    if not hasattr(conn, "mech_jeb"):
        print("      service missing -- rebuild and reinstall the DLL")
    else:
        line("mech_jeb.ping()", conn.mech_jeb.ping())
        line("available", conn.mech_jeb.available)
        if conn.mech_jeb.available:
            line("on this vessel", conn.mech_jeb.on_vessel)
            if conn.mech_jeb.on_vessel:
                for label, read in (("ascent engaged", lambda: conn.mech_jeb.ascent_enabled),
                                    ("ascent path", lambda: conn.mech_jeb.ascent_path),
                                    ("autostage", lambda: conn.mech_jeb.autostage),
                                    ("staging users", lambda: conn.mech_jeb.staging_users)):
                    try:
                        line(label, read())
                    except RuntimeError as exc:
                        # Names one member, not the whole service: that is the point
                        # of resolving them separately.
                        line(label, f"unreadable: {exc}")
                # The knob list comes from the live assembly, so it is the ground
                # truth for what this MechJeb build calls things.
                try:
                    names = conn.mech_jeb.ascent_setting_names
                    line("ascent settings", len(names))
                    wanted = ("desiredorbitaltitude", "desiredinclination",
                              "turnstartaltitude", "turnendaltitude", "turnshapeexponent")
                    for entry in names:
                        if entry.split(":")[0].strip().lower() in wanted:
                            print(f"      {entry}")
                    line("ascent flags", len(conn.mech_jeb.ascent_flag_names))
                except RuntimeError as exc:
                    line("ascent settings", f"unreadable: {exc}")
            else:
                print("      MechJeb is installed but this craft carries no running "
                      "MechJeb part (AR202 case, or a pod with MechJeb embedded).")
        else:
            print("      MechJeb not resolved. What the bridge found:")
            print(f"      {conn.mech_jeb.diagnostics}")
            print("      (send me this line - it names the exact member that moved)")

    print("\nActive vessel")
    # Outside flight there is no active vessel, and kRPC returns None rather than
    # raising - so the naive read fails with "'NoneType' object has no attribute
    # 'name'", which reads like a bug in the bridge rather than the normal state of
    # the space centre.
    vessel = _safe(lambda: conn.space_center.active_vessel, None)
    if vessel is None:
        line("state", f"none - not in flight (scene is {conn.krpc.game_scene})")
    else:
        try:
            line("name", vessel.name)
            line("situation", vessel.situation)
            line("met", f"{vessel.met:.1f} s")
            line("parts", len(vessel.parts.all))
        except Exception as exc:
            line("state", f"unreadable: {exc}")

    conn.close()
    print("\nOK.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
