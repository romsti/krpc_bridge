r"""Smoke test for KRPC.Bridge.

Run this the moment the DLLs are in GameData and KSP is in a flight scene. It answers,
in order: is kRPC alive, did the Core load, which plugins resolved and why not, and what
would a jump target look like right now.

    python python/check_bridge.py
    python python/check_bridge.py --ksp "D:\Games\KSP_1.12.5"   also compare with the installed manifest.json

Nothing here jumps, recovers or touches the flight. It only reads.

The "Build" section reads conn.bridge.manifest_v1(): the commit every loaded KRPC.Bridge
assembly was built from, whether the tree was clean, and the versioned layouts this build
knows. With --ksp, it also reads GameData/KRPC.Bridge/manifest.json, written by build.cmd and
shipped in the release zip, and checks that the commit that RUNS is the commit that was
INSTALLED, and that every file on disk still has the sha256 the manifest gives. Exit code 1 if
not.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import math

try:
    from dynamics_v3 import decode_snapshot
except ImportError:
    decode_snapshot = None

try:
    import krpc
except ImportError:  # pragma: no cover
    sys.exit("krpc is not installed:  pip install 'krpc>=0.7,<0.8'")


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


def _norm3(state: dict, prefix: str):
    vals = [state.get(prefix + axis, float("nan")) for axis in ("x", "y", "z")]
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in vals):
        return None
    return math.sqrt(sum(float(v) * float(v) for v in vals))


def _parse_manifest(rows):
    """conn.bridge.manifest_v1() as (header dict, list of row dicts)."""
    if len(rows) < 2:
        return {}, []
    header = dict(p.split("=", 1) for p in rows[0].split("\t") if "=" in p)
    columns = rows[1].split("\t")
    return header, [dict(zip(columns, r.split("\t"))) for r in rows[2:]]


def _sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def check_build(conn, ksp_root):
    """The Build section. Returns False when what runs is not one clean, deployed build."""
    print("\nBuild")
    if not hasattr(conn.bridge, "manifest_v1"):
        print("  manifest_v1 missing: this Core predates the build stamp (lot 2).")
        return True
    header, entries = _parse_manifest(conn.bridge.manifest_v1())
    ok = True
    line("commit (loaded)", header.get("git_sha", "?"))
    line("one clean build", "yes" if header.get("consistent") == "1" else "NO")
    if header.get("consistent") != "1":
        ok = False
        for e in entries:
            if not e.get("layout"):
                print(f"      {e.get('assembly'):<26} {e.get('git_sha') or 'no stamp':<42} "
                      f"dirty={e.get('git_dirty') or '?'}")
    assemblies = sorted({e.get("assembly") for e in entries})
    line("assemblies", f"{len(assemblies)}: " + ", ".join(a.replace("KRPC.Bridge.", "")
                                                        for a in assemblies))
    layouts = [e for e in entries if e.get("layout")]
    line("versioned layouts", ", ".join(
        f"{e['layout']} {e.get('protocol')}" + (f".{e['schema']}" if e.get("schema") else "")
        for e in layouts))
    missing = [e["layout"] for e in layouts if "?" in (e.get("protocol"), e.get("schema"))]
    if missing:
        ok = False
        line("UNRESOLVED layouts", ", ".join(missing))

    if not ksp_root:
        line("deployed manifest", "not compared (pass --ksp <KSP root>)")
        return ok
    moddir = os.path.join(ksp_root, "GameData", "KRPC.Bridge")
    path = os.path.join(moddir, "manifest.json")
    try:
        with open(path, encoding="utf-8") as handle:
            deployed = json.load(handle)
    except (OSError, ValueError) as exc:
        line("deployed manifest", f"UNREADABLE: {path} ({exc})")
        return False
    line("commit (deployed)", f"{deployed.get('git_sha')} "
                              f"({'clean' if not deployed.get('git_dirty') else 'DIRTY'})")
    if deployed.get("git_sha") != header.get("git_sha"):
        ok = False
        line("RUNS = DEPLOYED", "NO - KSP loaded other DLLs than the deployed ones "
                                "(deployed while KSP ran?) - restart KSP")
    else:
        line("runs = deployed", "yes")
    bad = []
    for entry in deployed.get("files", []):
        full = os.path.join(moddir, entry["path"].replace("/", os.sep))
        if not os.path.isfile(full):
            # Shipped in the release, not installed in the flight instances.
            if not entry["path"].startswith("Plugins/KRPC.Bridge.MechJeb."):
                bad.append(entry["path"] + " missing")
        elif _sha256(full) != entry["sha256"]:
            bad.append(entry["path"] + " changed")
    line("files vs manifest", "all sha256 match" if not bad else "; ".join(bad))
    return ok and not bad


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Smoke test for KRPC.Bridge (read only).")
    parser.add_argument("--ksp", default="",
                        help="KSP root, to compare what runs with the deployed manifest.json")
    args = parser.parse_args(argv)
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
        s for s in ("bridge", "discovery", "control", "telemetry", "actuators", "fmrs", "ident",
                    "ocisly", "mech_jeb", "trajectories", "stock", "stock_delta_v", "tower")
        if hasattr(conn, s)))
    line("events recorded", conn.bridge.events_recorded)

    build_ok = check_build(conn, args.ksp)

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

    # 2. Control and Telemetry (refactor R3; conn.actuators before). Read-only here: commands
    # belong to an explicitly armed GNC probe.
    print("\nControl and Telemetry")
    if conn.krpc.game_scene != conn.krpc.GameScene.flight:
        # Control and Telemetry only answer in the flight scene: stop here, the Build section above already
        # says whether the deployed bridge is the one that runs.
        print("      needs the flight scene (now %s): put a vessel on the pad and run again"
              % conn.krpc.game_scene)
        conn.close()
        return 0 if build_ok else 1
    ctl = getattr(conn, "control", None) or getattr(conn, "actuators", None)
    tel = getattr(conn, "telemetry", None) or getattr(conn, "actuators", None)
    if ctl is None or tel is None:
        print("      service missing -- rebuild and reinstall the DLLs")
    else:
        line("control.ping() / telemetry.ping()", "%s / %s" % (ctl.ping(), tel.ping()))
        line("engines", len(tel.engine_sample()) // 9)
        line("gimbals", len(tel.gimbal_sample()) // 8)
        line("direct thrust directions",
             len(tel.thrust_direction_sample()) // 5
             if hasattr(tel, "thrust_direction_sample")
             else "not deployed")
        line("independent gimbal command",
             hasattr(ctl, "lease_gimbal") and
             hasattr(ctl, "release_gimbal"))
        if hasattr(ctl, "control_snapshot_v2"):
            snapshot = ctl.control_snapshot_v2()
            if len(snapshot) < 14:
                line("atomic protocol v2", "INVALID header")
            else:
                expected = (14 + int(snapshot[8]) * int(snapshot[9])
                            + int(snapshot[10]) * int(snapshot[11])
                            + int(snapshot[12]) * int(snapshot[13]))
                line("atomic protocol v2", int(snapshot[0]))
                line("physics/topology tick",
                     f"{int(snapshot[1])} / {int(snapshot[5])}")
                line("per-nozzle geometry", int(snapshot[12]))
                line("snapshot shape",
                     "ok" if len(snapshot) == expected
                     else f"INVALID {len(snapshot)} != {expected}")
                status_v2 = ctl.control_status_v2()
                line("exclusive owner active", bool(status_v2[3]))
        else:
            line("atomic protocol v2", "not deployed")

        if hasattr(tel, "dynamics_snapshot_v3"):
            dyn = tel.dynamics_snapshot_v3()
            if not dyn:
                line("dynamics protocol v3", "waiting for first FixedUpdate")
            elif len(dyn) < 22:
                line("dynamics protocol v3", "INVALID header")
            else:
                # Schema 3 (refactor R3) ends with the one-value clock section.
                expected = (22 + int(dyn[12])
                            + int(dyn[13]) * int(dyn[14])
                            + int(dyn[15]) * int(dyn[16])
                            + int(dyn[17]) * int(dyn[18])
                            + (1 if int(dyn[1]) >= 3 else 0))
                line("dynamics protocol v3", f"{int(dyn[0])}.{int(dyn[1])}")
                line("dynamics tick", int(dyn[2]))
                line("dynamics capabilities", hex(int(dyn[11])))
                line("dynamics history", f"{int(dyn[20])}/{int(dyn[19])}")
                line("dynamics shape",
                     "ok" if len(dyn) == expected
                     else f"INVALID {len(dyn)} != {expected}")
                hist = tel.dynamics_frames_v3(max(0, int(dyn[2]) - 4), 8)
                line("dynamics history RPC",
                     f"{int(hist[2])} frame(s)" if len(hist) >= 6 else "INVALID")

                if int(dyn[1]) >= 2 and decode_snapshot is not None:
                    try:
                        decoded = decode_snapshot(dyn)
                        st = decoded["state"]
                        line("dynamics capability names",
                             ", ".join(sorted(decoded["capabilities"])))
                        for label, prefix, unit in (
                            ("engine force realized", "engine_force_body_n_", "N"),
                            ("engine torque realized", "engine_torque_body_nm_", "N.m"),
                            ("aero force live", "aero_force_body_n_", "N"),
                            ("aero torque live", "aero_torque_body_nm_", "N.m"),
                            ("external force residual",
                             "external_force_residual_world_n_", "N"),
                            ("unexplained non-aero",
                             "unexplained_non_aero_force_world_n_", "N"),
                        ):
                            norm = _norm3(st, prefix)
                            line(label, f"{norm:.3f} {unit}" if norm is not None else "NA")
                        q = st.get("dynamic_pressure_pa")
                        aoa = st.get("aoa_deg")
                        beta = st.get("sideslip_deg")
                        line("aero q / AoA / beta",
                             (f"{q:.1f} Pa / {aoa:.3f} deg / {beta:.3f} deg"
                              if all(isinstance(v, (int, float)) and math.isfinite(v)
                                     for v in (q, aoa, beta)) else "NA"))
                    except Exception as exc:
                        line("dynamics v3.2 decode", f"ERROR: {exc}")
        else:
            line("dynamics protocol v3", "not deployed")

    # 3. FMRS.
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

    # 5. Trajectories. The service is Flight-only, so outside flight kRPC does not
    # offer it at all and hasattr is False - that is normal, not a missing DLL.
    print("\nTrajectories")
    if not hasattr(conn, "trajectories"):
        print("      service not offered in this scene (Flight only), or the DLL did not load")
    else:
        tr = conn.trajectories
        line("trajectories.ping()", tr.ping())
        # available() is a PROCEDURE on this service - parentheses - because the
        # previous single-DLL bridge had it that way and existing clients call it so.
        line("available()", tr.available())
        if tr.available():
            line("mod version", tr.mod_version)
            for label, read in (("api version", lambda: tr.api_version),
                                ("always update", lambda: tr.always_update),
                                ("has impact", lambda: tr.has_impact()),
                                ("time till impact", lambda: tr.get_time_till_impact()),
                                ("has target", lambda: tr.has_target()),
                                ("retrograde entry", lambda: tr.retrograde_entry),
                                ("profile angles deg", lambda: tr.get_descent_profile_angles())):
                try:
                    line(label, read())
                except RuntimeError as exc:
                    line(label, f"unreadable: {exc}")
            if _safe(lambda: tr.has_impact(), False):
                lat, lon, alt = tr.get_impact_geo()
                line("impact", f"lat {lat:.4f}  lon {lon:.4f}  alt {alt:.0f} m")
        else:
            print("      Trajectories not resolved. What the bridge found:")
            print(f"      {tr.diagnostics}")

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
    if not build_ok:
        print("\nBUILD CHECK FAILED (see Build above).")
        return 1
    print("\nOK.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
