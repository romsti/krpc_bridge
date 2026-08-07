# Changelog

All notable changes to KRPC.Bridge. Format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/); versions follow
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
