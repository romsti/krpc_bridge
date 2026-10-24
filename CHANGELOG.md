# Changelog

All notable changes to KRPC.Bridge. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2.0.0] — 2026-10-24

The first release since 1.0.0, and a breaking one: **it needs kRPC 0.7.0 and no longer loads on
kRPC 0.6.x.** kRPC 0.7 changed some of the shapes the bridge reads (SpaceCenter's `Wrench` is now a
struct, a few text members of `Module` are gone), so the assemblies are built against 0.7.0.
KSP 1.12.x as before. Of the 1.1.0 that was versioned in August but never released, everything is
included.

**Upgrading.** Delete the old `GameData/KRPC.Bridge` folder before you unzip this one: the DLLs are
not the same set (`Control`, `Telemetry`, `Model`, `Stock`, `Ident`, `Tower` are new), and two
assemblies that declare the same kRPC service make kRPC refuse the duplicate, which takes the whole
server down. The booster catch tower is no longer inside `KRPC.Bridge`: it is the optional
`TowerCatch` package (see below).

### Added

**New services** (twelve services and 307 members in all; `available` and `diagnostics` on each, as before)

- `conn.control` — the half that writes: per-engine independent throttle and per-gimbal leases (short,
  restored automatically), atomic control frames (`acquire_control`, `apply_control_frame`), and
  **AttitudeV1**, an attitude controller that runs on the physics step. Every option of AttitudeV1 is
  off until a client switches it on, and a client reads what the build supports first
  (`attitude_options_v1`, `attitude_capabilities_v1`).
- `conn.telemetry` — the half that only observes: samples of every engine and gimbal in one call,
  dynamics frames at the physics rate (V3, Ext, and for tracked vessels that are not active), the aerodynamic
  actuators and the aerodynamic wrench, the contact of a vessel with the ground by leg (`contact_v1`).
- `conn.stock` — delta-v simulation and stage table, staging from the frame loop, the aerodynamic
  force without the launch clamps, landing-leg deployment, the fields and events of part modules in
  one call.
- `conn.trajectories` — the Trajectories mod's atmospheric impact prediction, its landing target and
  its descent profile.
- `conn.ident` — the identifiers kRPC holds but does not expose (part flightID, vessel persistentId).
- `conn.discovery` — read-only introspection of the assemblies and types the game has loaded.
- `conn.tower` and `conn.tower_site` — booster catch towers: put one on the ground from a craft, list
  and read the towers of a game from the Space Center, ground tests (place and hold a vessel, anchor,
  hinge probes).

**`conn.bridge`**

- `manifest_v1` says which build is loaded: every `KRPC.Bridge*` assembly, its services, the versioned
  layouts it knows and the commit it was built from.
- A game clock (`clock_v1`), a vessel registry (`vessels_v1`), bit and layout registries (`bits_v1`,
  `layout_v1`) and an event bus V2 (`poll_events_v2`: one poll per client tick, every record stamped
  with the master tick of the game clock, by persistentId).
- Errors of `conn.control` and `conn.telemetry` start with a code, `"[CODE] text"`: branch on the code.

**MechJeb** — any module by name (`describe_module`, `setting`, `flag`, `enum_value`, ...), the landing
predictor, and the maneuver planner (the nodes are ordinary KSP maneuver nodes, read back with stock
kRPC), where 1.0.0 reached the ascent autopilot only.

**FMRS** — `dropped_stages_v1`, a table that binds each FMRS entry to its staging and separation, and
`ready_v1`, a readiness probe.

**Build and distribution**

- `build.cmd` runs the tests that need no game, builds, and validates every kRPC signature with
  kRPC's own scanner. Every DLL carries the commit it was built from, and each package has a
  `manifest.json` (sha256 of every file, the commit, clean or dirty).
- `tools/package.py` builds the two release zips, `KRPC.Bridge-x.y.z.zip` and `TowerCatch-x.y.z.zip`.
- CKAN and KSP-AVC records for both packages (`distribution/`).

**Trajectories — `conn.trajectories`** (`KRPC.Bridge.Trajectories.dll`)

- The mod's atmospheric impact prediction over kRPC: `available()`, `has_impact()`,
  `get_impact_geo()` (latitude, longitude, terrain altitude), `get_impact_position()`
  (the mod's raw body-relative vector), `get_time_till_impact()` — these five keep the
  names, shapes and failure behaviour of the previous single-DLL `KRPCTrajectories`
  bridge, byte for byte, so existing clients need no change. `available()` is therefore a
  procedure, not a property.
- New: `get_end_time()`, `get_impact_velocity()`, `update_trajectory()`, `always_update`;
  the landing target — `has_target()`, `set_target()`, `clear_target()`, `get_target()`,
  `get_planned_direction()`, `get_corrected_direction()`; the descent profile —
  `get_descent_profile_angles()/modes()/grades()`, `set_descent_profile()`,
  `reset_descent_profile(aoa_deg)`, `retrograde_entry`, `prograde_entry`. Angles are
  degrees on the wire and converted to the radians `Trajectories.API` speaks.
- Every member of `Trajectories.API` is resolved once at load; one that the installed
  version lacks raises `Trajectories.API.<name> absent (vX)` and is listed in
  `diagnostics`, never a `NullReferenceException`. `AlwaysUpdate` is switched on at every
  flight-scene start so the prediction is live with the mod's window closed.
- Verified against Trajectories 2.4.5.4 (members read from the installed DLL by
  reflection). Reached by reflection only; the mod is GPL-3.0 and nothing of it is linked.

### Changed

- **The booster catch tower is a package of its own, `TowerCatch`.** Its ModuleManager patch
  (`Patches/TowerCatch.cfg`) and its craft (`Crafts/Tour Catch v5.craft`) no longer travel inside
  `GameData/KRPC.Bridge`; the services that place and read a tower stay in `KRPC.Bridge.Tower.dll`.
  The `TG.*` part names did not change, so a tower already in a save loads as before. Never keep two
  copies of the patch. `conn.tower_site.tower_package_v1` tells which folder holds it.
- The tower craft lets its arms open wider, for larger boosters.
- Documentation (`docs/API.md`) rewritten for all of the above.

### Removed

- Support for kRPC 0.6.x.

## [1.1.0] — 2026-08-08

Versioned in August but never published as a separate download: everything below is part of 2.0.0.

MechJeb goes from one module to all of them. 1.0.0 could reach `AscentSettings` and
nothing else, because the by-name settings mechanism was wired to that single type; this
release points it at any module and adds the maneuver planner. Forty-one new procedures,
none of which names a MechJeb class — the previous kRPC-to-MechJeb bridge died binding
about forty-eight type names, and the same failure applies at member granularity, just
more slowly.

FMRS and OCISLY are unchanged.

### Added

**MechJeb — any module, by name**

- `modules`, `describe_module()` — what this MechJeb has, read from the live assembly.
  `describe_module` reports each member's name, channel, type, whether it is writable and
  whether writing it touches the player's saved configuration. It is the member that turns
  a MechJeb rename into a one-string script edit.
- `setting()` / `set_setting()`, `flag()` / `set_flag()`, `enum_value()` /
  `set_enum_value()` / `enum_options()`, `list_value()` / `set_list_value()`,
  `text_value()` — five channels, because MechJeb's settings are not all doubles. Choices
  are carried as their names, so a script says `"KEEP_SURFACE"` rather than `2`, and the
  legal values come from the installed build.
- `engage()`, `disengage()`, `module_enabled()`, `module_users()` — with the differences
  handled: self-pinned modules refuse to be disengaged rather than silently breaking the
  rest of MechJeb, and settings bags say they are not autopilots instead of appearing to
  work.

What that unlocks immediately, all previously unreachable: the staging controller's
`AutostageLimit`, `HotStaging`, `DropSolids` and the three fairing-jettison criteria; the
thrust limiters for dynamic pressure, acceleration, flameout and overheating; the target
controller's latitude and longitude.

**MechJeb — the landing predictor**

- `landing_predicted`, `landing_latitude`, `landing_longitude`, `ignition_countdown`,
  `landing_countdown`, `landing_delta_v`, `landing_slope`. MechJeb's suicide-burn solver
  runs whenever you are in flight and republishes about once a second, so these are field
  reads and safe to stream. **Stock kRPC has no impact prediction of any kind**, and this
  one propagates through the atmosphere rather than guessing ballistically.

**MechJeb — the maneuver planner**

- `maneuver_operations`, `maneuver_operation_name()`, `describe_maneuver()`,
  `maneuver_parameter()` / `set_maneuver_parameter()`, `maneuver_flag()` /
  `set_maneuver_flag()`, `maneuver_time_references()` / `set_maneuver_time_reference()`,
  `create_maneuver_nodes()`, `maneuver_warning`.
- The nodes are ordinary KSP maneuver nodes: MechJeb computes and places, and you read the
  result back with **stock kRPC's `vessel.control.nodes`**. That is what keeps the whole
  planner to eleven procedures — MechJeb's transfer maths returns `Vector3d` and C#
  `ValueTuple`, neither of which is a legal kRPC type, and a malformed signature takes the
  entire kRPC server down. Nothing of the sort crosses this boundary.
- Interplanetary transfers are excluded and say so: that operation's solver is built by
  MechJeb's GUI and cannot be driven headlessly.

**MechJeb — the three modules whose entry point is a method**

- `land_at_target()`, `land_untargeted()`, `stop_landing()`, `execute_node()`,
  `execute_all_nodes()`, `abort_node()`, `smart_ass_engage()`. Enabling those modules
  without calling the method leaves them running with nothing to do.

### Fixed

- `ascent_setting_names` and `ascent_flag_names` filter out MechJeb's public backing
  fields more thoroughly. The leading-underscore rule caught `_autostage` and missed
  `AscentTypeInteger` and the four `DisplayModule` pairs; a field is now hidden when a
  *writable* property covers it, including through a `Config`, `Internal` or `Integer`
  suffix. Fields under a read-only property are kept, because there they are the only way
  to write the setting.

## [1.0.0] — 2026-08-07

First public release. The mod was previously a single unreleased `KRPC.Bridge.dll`; this
version splits it into a Core plus three plugins and adds a large amount of FMRS surface
that was verified against the mod's source but never exposed.

### Added

**FMRS**

- `recovery_report()` and `recovered_funds()` — read FMRS's recovery ledger. The only place
  the outcome of a booster recovery is stated as a number rather than inferred by diffing
  the funds counter. Covers funds, science, contracts, kerbals lost and buildings destroyed.
- `recover_current()` — recover the dropped stage you are flying, through FMRS's own
  settlement path.
- `revert_to_launch()` — FMRS's Revert To Launch, which still works after a jump when KSP's
  own revert is gone.
- `delete_dropped()` and `delete_all_dropped()` — stop tracking a stage and delete its save.
- `separation_times` — UT per FMRS save file, so a stage's separation time comes from what
  FMRS recorded rather than from when a poll happened to notice it.
- `kerbals_aboard` — which dropped stage each kerbal is on.
- `set_vessel_state()` — repair a stage wrongly marked `RECOVERED` so a landing series can
  continue.
- `launched_at`, `enabled`, `kick_to_main` — mission clock, plugin state, and advance
  warning that FMRS is about to drag the game back into the main mission.
- `track_parachutes`, `control_uncontrollable`, `auto_cut_off`, `screen_messages`,
  `window_hidden` — the session settings that decide whether a booster is tracked at all,
  plus two for recording.

**Core (`conn.bridge`)**

- An ordered, id-numbered event log: `poll_events()`, `on_event()`, `mark()`,
  `events_recorded`. Lets a script observe an *instant* — a separation, a part dying —
  rather than sampling a value and hoping. Hooks ten stock GameEvents plus
  `fmrs.dropped`, `fmrs.forgotten`, `fmrs.on_dropped`, `fmrs.on_main` and
  `ocisly.rearmed`.
- `describe_type()` — list any loaded mod type's members from Python, without a rebuild.
- `plugins`, `available_plugins`, `has_plugin()` — what loaded and, when it did not, why.
- `hide_ui()`, `show_ui()`, `ui_visible` — fires the GameEvents F2 fires, so mod windows go
  away too.
- Background jobs (`await_job` and friends) for work too expensive to run inside a single
  RPC. Scaffolding for plugins you write; nothing shipped here uses it.

**Build**

- `build/scan` runs kRPC's own `Scanner.GetServices` against the built DLLs, catching a
  malformed signature — which would disable the entire kRPC server in game — in about a
  second. `build.cmd` runs it on every build.
- `build/verify` now globs `src/**/*.cs` instead of listing files, so a new plugin cannot
  be left unchecked by omission.

### Changed

- **Split into Core plus plugins.** One `KRPC.Bridge.dll` becomes `KRPC.Bridge.Core.dll` at
  the root of the mod folder, with `Fmrs`, `Ocisly` and `MechJeb` under `Plugins/`. Load
  order is declared with `KSPAssembly`/`KSPAssemblyDependency`, so a plugin whose Core is
  missing is skipped by KSP with a log line rather than throwing on first use.
- **FMRS now works outside flight.** The service bound only to the flight `KSPAddon`, but
  FMRS keeps its state on a base class shared by four per-scene subclasses. Resolving
  against the base makes the tracked stages and the recovery ledger readable from the space
  centre — which is where you want them.
- `MechJeb.ascent_setting_names`, `ascent_flag_names` and `core_members` are cached. They
  are reflection walks, a client can put a stream on any property, and a streamed property
  re-runs every physics tick.
- HUD control moved from the FMRS-era `Bridge` service to the Core; the *automatic* hide
  after a scene load stays on `conn.ocisly`, because it has to happen after the cameras are
  restored and only that coroutine knows when they are.
- Every XML docstring rewritten against verified FMRS 1.2.9.6 and kRPC 0.6.0 behaviour, and
  checked against the tags the kRPC Python client actually parses.

### Fixed

- `jump_to_vessel` bound the FMRS overload by exact parameter types. FMRS declares three,
  two of which take two arguments — `(Guid, bool)` and `(Guid, string)` — so binding by
  name or arity can select the wrong one.
- `ocisly.rearm()` now raises the game HUD for the half second it needs and puts it back.
  OCISLY refuses to open a camera while the UI is hidden and refuses *silently*, so with
  `hide_ui_on_scene_load` on — the configuration for a recorded flight — a manual rearm
  armed nothing and reported success. It also counts cameras the mod actually took, rather
  than calls that did not throw.
- `jump_to_main()` returns immediately when you are already on the main mission. FMRS's own
  entry point early-returns there, so nothing reloaded and the wait ran to its 90-second
  timeout before throwing — on a call the guard-rail advice tells you to make defensively.
- `mech_jeb.ascent_setting_names` and `ascent_flag_names` no longer advertise MechJeb's
  public backing fields. `_autostage` sits next to `Autostage` and writing it skips the
  staging registration, so the list offered two names of which one silently did nothing.
- `fmrs.main_vessel_id` returns an empty string rather than the zero GUID when no flight is
  armed. The zero GUID is truthy in Python.

### Documentation

- `docs/API.md`, `docs/SCRIPTING.md`, `docs/ARCHITECTURE.md`, all in English.
- `LICENSE` (MIT) and `NOTICE` recording that the mod links against kRPC (LGPL v3) using
  the player's own copy, and reaches FMRS, MechJeb, OCISLY and HullcamVDS purely by
  reflection with nothing redistributed.
