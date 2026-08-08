# Changelog

All notable changes to KRPC.Bridge. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.1.0] — 2026-08-08

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
