# API reference

Every remote procedure KRPC.Bridge adds, by service. Names are given in their Python
form; kRPC converts C# `PascalCase` to `snake_case` on the wire, so `DroppedVessels`
becomes `conn.fmrs.dropped_vessels`.

`help(conn.fmrs)` in a Python REPL gives you the same text as this file, generated from
the XML shipped next to each DLL. This document exists so you can read it without
launching the game.

**Contents**

- [Conventions](#conventions)
- [`conn.bridge`](#connbridge) — Core: what loaded, events, jobs, HUD, reflection probe
- [`conn.fmrs`](#connfmrs) — dropped stages, jumps, recovery ledger
- [`conn.ocisly`](#connocisly) — camera streams across a scene reload
- [`conn.mech_jeb`](#connmech_jeb) — ascent, staging, every module by name, the landing
  predictor and the maneuver planner

---

## Conventions

**Check `available` first.** Every plugin exposes it. When it is `False`, almost every
other member of that service raises `RuntimeError`, and `diagnostics` says which
reflective lookup came back empty. This is the difference between a clear "FMRS is not
installed" and a mystery twenty minutes into a flight.

The exceptions are the ones that are meant to answer rather than fail: `ping()` always
returns `"pong"`, and `fmrs.active`, `fmrs.jump_in_progress` and `mech_jeb.on_vessel`
return `False`. The last two are worth remembering — a script that expected an exception
gets a quiet `False` instead.

```python
if not conn.fmrs.available:
    raise SystemExit(conn.fmrs.diagnostics)
```

**`available` is not enough on its own, and this catches people.** It says the mod is
installed and resolved. It does not say the mod is reachable *here*.

- **FMRS** lives only in flight, the space centre, the tracking station and the main menu.
  In the VAB or SPH, `available` is still `True` and everything except `available`,
  `diagnostics`, `ping()`, `jump_in_progress` and the seven session settings raises
  `RuntimeError: FMRS is not active in this scene`. `fmrs.active` is the guard.
- **MechJeb** needs a running MechJeb part on the current craft. When `on_vessel` is
  `False`, every member below it raises; the three cached `*_names` lists still answer.
- **OCISLY** needs a flight scene: without one, `rearm()` raises because the mod's GUI
  object does not exist.

So the full guard is two lines, not one:

```python
if not conn.fmrs.available:
    raise SystemExit(conn.fmrs.diagnostics)
if not conn.fmrs.active:
    raise SystemExit("FMRS is not reachable in this scene — are you in the editor?")
```

**Exceptions.** A C# `InvalidOperationException` arrives in Python as `RuntimeError`;
`ArgumentException` arrives as `ValueError`. The message is always preserved. Note that
kRPC's own `RPCError` will *not* catch these — use `RuntimeError`.

**Packed rows.** Some members return a list of tab-separated strings rather than a list
of objects. That is deliberate, not laziness: a kRPC object handle costs one extra round
trip per property you then read, so forty records with six fields each would be 240 RPCs
as objects and one as packed strings. Split on `\t`.

**Dictionaries are string-to-string.** kRPC requires every value in a dictionary to be
one type. Where a value is really a number it is returned as a decimal string; parse it
with `float()`.

**Streaming.** Any member with a return value can be streamed, including a procedure
with arguments — they are bound once at stream creation. A streamed member then re-runs
**every physics tick, forever**, so two categories are worth avoiding.

*Side effects.* `bridge.hide_ui()`, `bridge.show_ui()`, `ocisly.name_cameras()`,
`mech_jeb.release_staging()` and `fmrs.set_vessel_state()` all return a value and all
change something. Streaming one means doing that thing sixty times a second.

*Expensive reads.* Three tiers, and the middle one is larger than it looks.

- **Never stream.** `fmrs.recovery_report()` and `fmrs.recovered_funds()` re-read a file
  from disk with their default argument; `bridge.describe_type()` reflects over a whole
  type.
- **Prefer not to stream.** Every FMRS property, and every MechJeb property other than the
  three cached `*_names` lists, does a live-object lookup on *each* read — a scene-wide
  `FindObjectOfType` for FMRS, two or three reflection hops for MechJeb. On top of that,
  `fmrs.dropped_persistent_ids` walks every vessel in the game state and builds a dict,
  `fmrs.separation_times` does one reflective call per distinct save file, and
  `ocisly.hullcams` walks every loaded vessel's parts looking for camera modules. Poll
  these at the rate you actually need instead.
- **Cheap.** Everything on `conn.bridge` except `describe_type()`, and MechJeb's
  `ascent_setting_names`, `ascent_flag_names` and `core_members`, which are computed once
  and cached for exactly this reason.

---

## `conn.bridge`

The Core service. Available whenever the mod is installed, in every game scene, whether
or not any other mod is present.

### What is loaded

| Member | Type | Notes |
|---|---|---|
| `ping()` | `str` | `"pong"` once the Core is up, `"loading"` before that. |
| `version` | `str` | Version of `KRPC.Bridge.Core.dll`. |
| `plugins` | `list[str]` | One row per plugin: `name⇥available⇥plugin_version⇥mod_version⇥report`. `available` is `"1"` or `"0"`. |
| `available_plugins` | `list[str]` | Just the names of the plugins whose target mod resolved. |
| `has_plugin(name)` | `bool` | Registered *and* resolved. |
| `ticks` | `int` | Physics ticks since the Core started. Frozen means the game is paused or loading, not that your script is slow. |
| `pending_main_thread_work` | `int` | Should sit at 0. A rising number means a plugin is posting faster than one tick can drain. |

```python
for row in conn.bridge.plugins:
    name, ok, version, mod_version, report = row.split("\t")
    print(f"{name:8} {'ok' if ok == '1' else 'MISSING':8} {report}")
```

### Events

kRPC streams sample a value once per update: they report what something *is*, never that
something *happened*. A part destroyed between two samples leaves no trace. The event log
fixes that by recording on the C# side, in order, with ids.

| Member | Type | Notes |
|---|---|---|
| `poll_events(since_id)` | `list[str]` | Rows `id⇥kind⇥ut⇥realtime⇥vessel_id⇥detail`, oldest first. Pass 0 first, then the last id you saw. |
| `on_event(kinds="")` | *event* | A kRPC event that fires when a matching record is written. Comma-separated substrings; empty or `"*"` matches everything. |
| `events_recorded` | `int` | Total this session, including records aged out of the 512-entry ring. Compare against your last id to detect that you polled too slowly. |
| `mark(kind, detail)` | — | Publish your own marker, to align a flight log with your script's phases. |

**The two timestamps.** `ut` is absolute in-game universal time in seconds — *not* mission
elapsed time, so subtract `fmrs.launched_at` if you want T+. `realtime` is seconds since
the KSP **process** started: useful for measuring how long something took in the real
world, meaningless across a restart and not a wall clock. Both are decimal strings with
three digits after the point; parse with `float()`.

**Event kinds, and what `detail` and `vessel_id` hold in each.** `detail` is free text
whose format depends on the kind, so a parser needs this table.

| Kind | `vessel_id` | `detail` |
|---|---|---|
| `vessel.destroy` | the vessel | its name |
| `vessel.change` | the new vessel | its name |
| `vessel.recovered` | the vessel | its name |
| `vessel.situation` | the vessel | `"<from> -> <to>"`, e.g. `"FLYING -> LANDED"` |
| `vessel.soi` | the vessel | `"<from> -> <to>"` body names |
| `stage.activate` | the vessel | the stage number |
| `part.die` | the vessel | the part's **internal** name (`probeCoreOcto`), not its title |
| `part.crash` | the vessel | same, internal name |
| `part.splashdown` | the vessel | the part's internal name |
| `scene.load` | *empty* | the `GameScenes` name, e.g. `"FLIGHT"` |
| `fmrs.dropped` | the dropped stage | `"<vessel name> \| save=<save name>"` |
| `fmrs.forgotten` | the stage | *empty* |
| `fmrs.on_dropped` | *empty* | which stage was switched to |
| `fmrs.on_main` | *empty* | |
| `ocisly.rearmed` | *empty* | `"opened=N streaming=M"` |
| whatever you pass to `mark()` | *empty* | whatever you pass |

So pulling the save name out of a separation — the obvious thing to want — is
`detail.split(" | save=")[1]`.

Block rather than poll:

```python
last = 0
evt = conn.bridge.on_event("fmrs.dropped")
with evt.condition:
    evt.wait()                       # returns the instant a stage separates
for row in conn.bridge.poll_events(last):
    eid, kind, ut, rt, vessel_id, detail = row.split("\t")
    last = int(eid)
```

The event carries no payload and two occurrences inside one tick collapse into one — read
`poll_events` afterwards for what actually happened. That pairing is the point: the signal
gives you latency, the log gives you completeness.

### HUD

| Member | Type | Notes |
|---|---|---|
| `hide_ui()` | `bool` | As if F2 had been pressed. `False` if already hidden. Flight only. |
| `show_ui()` | `bool` | `False` if already visible. Flight only. |
| `ui_visible` | `bool` | |

These fire `GameEvents.onHideUI` / `onShowUI` rather than blanking the stock canvases, so
mod windows disappear too — FMRS's, OCISLY's, MechJeb's, the toolbar. For a recorded
flight that is the whole difference.

### Reflection probe

| Member | Type | Notes |
|---|---|---|
| `describe_type(assembly_fragment, type_name)` | `list[str]` | Rows `kind⇥name⇥signature` for every member, public or not. The **first** row is always `assembly⇥<name>⇥<version>`. On failure the whole result is a single `error⇥…` row rather than an exception — test for it, or you will treat `error` as a member. |

The tool for writing a new plugin, and for diagnosing an old one after a mod update.

```python
for line in conn.bridge.describe_type("FMRSContinued", "FMRS.FMRS_Core"):
    print(line)
```

### Background jobs

For work too expensive to do inside a single RPC. An RPC body runs inside Unity's
`FixedUpdate`, so an RPC that spends 300 ms computing drops the game to 3 FPS for that
frame. A job moves the work to a worker thread operating on a plain-data snapshot.

| Member | Type | Notes |
|---|---|---|
| `get_job_state(id)` | `str` | `pending`, `running`, `done`, `failed`, `cancelled`. An **unknown id also reports `failed`**, with an empty `get_job_error()` — so does one whose result has been swept. |
| `get_job_progress(id)` | `float` | 0 to 1, as last reported by the worker. |
| `get_job_result(id)` | `list[float]` | Raises while still running. |
| `get_job_error(id)` | `str` | Empty when the job did not throw — including when there is no such job. |
| `await_job(id, timeout_seconds=60)` | `list[float]` | Blocks via a kRPC continuation, so the game keeps full framerate. The timeout is **real** time, and it is clamped: 0, a negative, NaN or infinity all become 1 second, and anything above 3600 becomes 3600. There is no "wait forever". |
| `cancel_job(id)` | — | Cooperative; the worker decides when to notice. |

Finished jobs are kept for **300 seconds** and then discarded, so collect a result before
then. After that the id is indistinguishable from one that never existed.

No shipped plugin starts a job — this is scaffolding for ones you write.
`src/Plugins/Template/` shows the pattern, and is deliberately not built into the release:
it declares a kRPC service of its own, so shipping it would put a fifth service in every
user's install.

---

## `conn.fmrs`

Verified against **FMRS Continued 1.2.9.6**.

FMRS records a save file at every staging event and lets you fly a dropped stage
afterwards, then return to the main mission where you left it.

### Three things to know first

1. **A jump reloads the flight scene.** `jump_to_vessel` blocks 5 to 20 seconds and every
   kRPC handle you held beforehand — vessels, parts, modules — is dead afterwards. Remove
   your streams first, then rebuild everything from a fresh `space_center.active_vessel`.

2. **`switched_to_dropped` is the guard rail.** FMRS hooks `onGameSceneLoadRequested`, and
   if a scene change is requested while that flag is true it force-loads the main mission
   a few dozen frames later — dragging the game back into flight in the middle of whatever
   your script was doing. Always `jump_to_main()` before reverting, recovering or
   launching anything else.

3. **`armed` must be true before launch.** FMRS reads it exactly once, at the top of its
   launch routine. Set it between the craft arriving on the pad and the first staging.

### State

| Member | Type | Notes |
|---|---|---|
| `available` | `bool` | Installed and the jump API resolved. |
| `diagnostics` | `str` | Member-by-member resolution report. |
| `active` | `bool` | An FMRS object exists in this scene. Briefly false during a jump, and **always false in the VAB and SPH** — where every member below raises. Never raises itself. |
| `ping()` | `str` | `"pong"`. |
| `armed` | `bool` r/w | Will FMRS capture this flight's stages. Sticky — FMRS persists it. Writing also writes FMRS's save file. |
| `enabled` | `bool` r/w | The plugin is live for this flight. **Setting it true does not start FMRS**: it writes a field and attaches no handlers, so nothing is captured. Useful for turning FMRS *off* mid-flight. Prefer `armed`. |
| `switched_to_dropped` | `bool` | Currently flying a dropped stage rather than the main mission. |
| `has_launched` | `bool` | FMRS saw this flight launch, which closes the arming window. |
| `kick_to_main` | `bool` r/w | FMRS has queued a forced return to the main mission. Finding this true in the space centre means you are about to be interrupted. Writing it also writes FMRS's save file, like `armed`. |
| `launched_at` | `float` | UT of the main mission's launch, or 0. Subtract from `space_center.ut` for FMRS's mission clock. |
| `jump_in_progress` | `bool` | True for about a second after a jump is requested. **False does not mean the jump finished.** |

### Settings

All session-scoped statics: FMRS copies its difficulty settings into them once, from the
space-centre scene, so changing the difficulty setting mid-flight does nothing while
writing these takes effect immediately. They do not persist — set them each session.

| Member | Type | Notes |
|---|---|---|
| `auto_recover` | `bool` r/w | FMRS banks a landed stage when you *leave* it — see below. Not needed for replays. |
| `track_parachutes` | `bool` r/w | Track a stage that has parachutes but no probe core. |
| `control_uncontrollable` | `bool` r/w | Let you fly a stage FMRS considers uncontrollable. |
| `auto_cut_off` | `bool` r/w | Cut the engines of a stage FMRS stops simulating. |
| `screen_messages` | `bool` r/w | FMRS's screen messages. Turn off for a recording. |
| `window_hidden` | `bool` r/w | Hide FMRS's own window only, unlike `bridge.hide_ui()`. Flight only, and **any stock UI event overwrites it** — F2, `bridge.hide_ui()`, or any mod firing those events. Set it last. |

The first two are the answer to "why is my booster not in the list" — **unless you also
have StageRecovery installed**, in which case FMRS defers parachute-only stages to it by
default and `track_parachutes` alone will not bring them back. That setting is not exposed
here yet; the symptom is an empty `dropped_vessels` with every knob apparently correct.

One more case `track_parachutes` does not explain: a **crewed** stage is tracked whatever
the setting says.

**When auto-recovery actually fires**, because the obvious guess is wrong. FMRS has no
landing handler and never simulates an unloaded stage, so a booster can sit landed
indefinitely with `auto_recover` on and nothing happens. Recovery is triggered only at the
four moments you *leave* a dropped stage:

- `jump_to_vessel(other, save_landed=True)` — jumping away to a different stage
- `jump_to_main()`
- the stock **Recover Vessel** button
- a scene change while `switched_to_dropped` is true

and only on Kerbin, and only if the stage is landed or splashed.

**So it does not block a replay.** Re-jumping to the stage you are already flying passes
`save_landed=False`, which never reaches that path. And a `RECOVERED` stage is still
jumpable: `jump_to_vessel` does not read the state and the separation save is never
deleted. What recovery costs you is FMRS's own jump button for that stage, and a second
payout. `delete_dropped()` is the only thing that genuinely makes a stage unreachable.

### Tracked stages

| Member | Type | Notes |
|---|---|---|
| `dropped_vessels` | `dict[str,str]` | vessel id → vessel name. The keys are what `jump_to_vessel` takes. |
| `dropped_saves` | `dict[str,str]` | vessel id → FMRS save name. **The batch key** — see below. |
| `separation_times` | `dict[str,str]` | save name → UT, as a decimal string. Join on the save name for a separation time per stage. **A save may be missing** — FMRS drops unreadable entries — so absence means unknown, not t=0. Use `.get(name)` and handle `None`. |
| `dropped_persistent_ids` | `dict[str,str]` | vessel id → KSP `persistentId`. **The bridge to kRPC's own Vessel objects.** Absent means KSP no longer has that vessel at all — destroyed or recovered — not that it is out of physics range, so retrying will not help. Walks every vessel in the game state: the most expensive member here. |
| `kerbals_aboard` | `dict[str,str]` | kerbal name → vessel id. Is anyone on that booster. |
| `main_vessel_id` | `str` | Empty string when FMRS has not armed a flight. |
| `vessel_state(vessel_id)` | `str` | `NONE`, `FLY`, `LANDED`, `DESTROYED`, `RECOVERED`. An **untracked id also returns `NONE`**, indistinguishable from a real one — check `dropped_vessels` first if the difference matters. |
| `set_vessel_state(vessel_id, state)` | `bool` | Overwrite it. `False` if FMRS is not tracking that id; `ValueError` if the state name is not one of the five. Un-banks a stage so FMRS will pay for it again, and restores its jump button in FMRS's own window. |

**Why `dropped_saves` is the batch key.** FMRS writes one save per *staging event*, not one
per vessel. Two stages that came off together carry the same value here; stages from
different events carry different ones. Several ids being present at once does *not* mean
they were dropped together — a launcher that sheds side boosters and then a core has both
sets listed simultaneously. Grouping on when a poll first noticed an id is a guess about
timing; grouping on this is what FMRS actually recorded.

**What `dropped_persistent_ids` is, and is not.** It is KSP's real `persistentId` for each
dropped stage, which survives a scene reload and lines up with `KSP.log` and the `.sfs`.

It is **not** a way to find the matching kRPC `Vessel`. `SpaceCenter.Vessel` exposes no
identifier at all — no `id`, no `uid`, no `persistent_id` — so there is nothing on that side
to compare it against. Reach a dropped stage by passing its FMRS id to `jump_to_vessel`
instead; the Guid *is* the handle. Its real uses are presence (an absent entry means KSP no
longer has that vessel at all — destroyed or recovered — not that it is out of physics
range) and correlation with the game's own records.

### Jumping

| Member | Notes |
|---|---|
| `jump_to_vessel(vessel_id, save_landed=True)` | Fly a dropped stage. Blocks until the target is active and unpacked. `ValueError` on an id FMRS is not tracking or a malformed GUID; `RuntimeError` if a jump is already running, or after a 90-second real-time timeout. |
| `jump_to_main()` | Back to the main mission, resumed where it was left. **Returns immediately and does nothing** when you are already on the main mission, so it is safe to call defensively. `RuntimeError` if FMRS never recorded a main vessel — that is, the flight was not armed at launch. |
| `revert_to_launch()` | FMRS's own Revert To Launch. Works after a jump, when KSP's revert is gone. Discards the flight's tracked stages. `RuntimeError` if `has_launched` is false. Does **not** check the career's no-revert rule, unlike FMRS's own button. |

`save_landed=True` writes the current state into the main-mission save before leaving. Keep
it true when leaving the main mission; with `False` you would later resume the main mission
from an older state.

**Re-jumping to the stage you are already flying is valid** — that is FMRS's "return to
separation", and it reloads the save frozen at the moment of separation. A landing can be
replayed from bit-identical initial conditions without re-flying the ascent. Pass
`save_landed=False` there: there is no main mission to preserve.

```python
vid = next(iter(conn.fmrs.dropped_vessels))
conn.fmrs.jump_to_vessel(vid)          # blocks 5-20 s
vessel = conn.space_center.active_vessel   # everything from before is dead
```

### Recovery and cleanup

| Member | Notes |
|---|---|
| `recover_current(force=True)` | Recover the dropped stage you are flying: refunds parts, credits science, completes contracts, records it in the ledger. `RuntimeError` unless `switched_to_dropped`. **Does not return you to the main mission** — call `jump_to_main()` yourself. Settles *every* loaded stage of the current sub-save, not just the one you are on. Freezes the game for a save-write plus two save-reads. |
| `delete_dropped(vessel_id)` | Stop tracking one stage, making it unjumpable. The red X in FMRS's window. The `.sfs` file is **not** deleted — the index entry is, and that is what makes it unreachable. `ValueError` on an untracked id. Not undoable from here. |
| `delete_all_dropped()` | All of them, leaving FMRS running. Same caveat: the files stay, the index empties. |
| `reset()` | FMRS's own close routine — forget everything and close the plugin for this flight. FMRS's bookkeeping is shared by every savegame of the install, so this is how you clean up after an interrupted run. **Flight scene only**: elsewhere it throws part-applied, having already disabled FMRS and dropped the tracked stages. |
| `recovery_report(reread=True)` | Rows `category⇥key⇥value`. `RuntimeError` if the ledger cannot be read. |
| `recovered_funds(reread=True)` | `float` — the `fund` rows summed, which is the number most campaigns want. |

The recovery ledger is the only place the outcome of a booster recovery is stated as a
number rather than inferred by diffing the funds counter. FMRS keeps it on disk so the main
mission — resumed from a save written *before* the recovery happened — can have it applied
on return. Read it after a recovery and before going back.

**It is cumulative for the mission, not per recovery.** FMRS never clears it on settlement,
only at prelaunch, so the third booster's report contains all three. What you want per
booster is the difference between two reads. And because the ledger is re-applied a couple
of seconds into every flight-scene start where you are not on a dropped stage, returning to
the main mission twice in one flight credits everything a second time. That is FMRS's
behaviour, not the bridge's, but it changes what the number means.

| category | key | value |
|---|---|---|
| `fund` | `add` | funds refunded |
| `science` | subject id | data amount recovered |
| `science_sent` | subject id | science credited |
| `contract` | `complete` | contract id |
| `kerbal` | `kill` | reputation lost |
| `building` | `destroyed` | building name |
| `message` | a heading | text FMRS would have shown |
| `warning` | `FMRS Info:` | a scene-change warning |

---

## `conn.ocisly`

OCISLY streams Hullcam VDS cameras out of the game. It subscribes to no scene GameEvent
and uses no `DontDestroyOnLoad`, so any flight scene reload — an FMRS jump, a quickload, a
revert — silently drops every camera it was tracking. This service brings them back.

**By default it is automatic.** `auto_rearm` and `disambiguate_names` are on, so it works
even when the jump was made by hand from FMRS's window with no script connected. Most
scripts never need to call `rearm()` at all.

| Member | Type | Notes |
|---|---|---|
| `available` | `bool` | |
| `diagnostics` | `str` | The mod with the least stable symbol names of the three — quote this line in a bug report. |
| `ping()` | `str` | |
| `window_open` | `bool` r/w | OCISLY's in-game window. Necessary for opening a camera, but **not sufficient**: the mod also gates on the game HUD being up, which this does not report. `rearm()` handles that itself. |
| `hullcams` | `list[str]` | Every Hullcam on every loaded vessel, as `vessel.camera`. Includes a dropped booster still in physics range. Walks every loaded vessel's parts — do not stream it. |
| `cameras` | `dict[str,str]` | OCISLY's internal id → camera name. The ids change on every reload; key on the name. A camera OCISLY has not painted yet appears as the literal `"(not painted yet)"`. |
| `streaming` | `list[str]` | Names currently on air. |
| `remembered` | `list[str]` | What will be restored on the next scene load: the `@flightID` identity tokens of the cameras last seen on air, refreshed every 30 physics ticks (0.6 s at the default timestep). |
| `last_restore` | `str` | What the last automatic restore did. |
| `auto_rearm` | `bool` r/w | Default on. |
| `restore_delay` | `float` r/w | Seconds to let a reloaded scene settle. Default 2. Dominates how long the HUD stays up after a jump. |
| `hide_ui_on_scene_load` | `bool` r/w | Default off. |
| `hide_ui_delay` | `float` r/w | Default 1. |
| `disambiguate_names` | `bool` r/w | Default on. |
| `name_cameras()` | `int` | Apply the naming now; returns how many were renamed. 0 means either nothing needed it, or Hullcam's `cameraName` field did not resolve so nothing *could* be renamed — `diagnostics` tells you which. |
| `rearm(filter="")` | — | Re-open and re-enable matching cameras. Blocks ~0.5 s. **Raises the HUD for the duration and puts it back**, because OCISLY refuses to open a camera while the game UI is hidden and refuses silently. Expect a half-second HUD flash on the recording. Also forces `window_open` on, and leaves it on. |

These five settings — `auto_rearm`, `disambiguate_names`, `hide_ui_on_scene_load`,
`restore_delay`, `hide_ui_delay` — are session-scoped and reset when KSP restarts, like
FMRS's. The two delays raise `ValueError` on NaN, infinity or a negative, and are **silently
clamped to 60** above that.

**Filter on the `@flightID` token, not on the name.** `rearm` matches comma-separated
substrings case-insensitively against `vessel.camera`, and *neither half of that name is
stable across a separation*. The vessel half obviously changes. The camera half changes
too: the ordinal in `Aerocam DN 2` is only added when several cameras share a name **on
the same vessel**, so separating the stack leaves each one alone and the next naming pass
drops it back to `Aerocam DN`. Only the `@1898164639` suffix survives, because
`part.flightID` is persistent — which is the entire reason it is appended. The automatic
restore filters on exactly that.

**Why `hide_ui_on_scene_load` lives here and not on `bridge`.** Order is the whole
difficulty: OCISLY refuses to open a camera while its UI is hidden, and only fills in a
camera's name during a repaint. The cameras must be renamed, opened and repainted *before*
the HUD goes away. Hiding first silently produces cameras that never stream, with no error
anywhere. That sequencing is one coroutine, and this is the service that owns it.

**Why `name_cameras()` exists.** A Hullcam's `cameraName` is a `KSPField` with
`isPersistant = false`, so it comes from the part config, not the instance: two cameras of
the same part type on one vessel report an identical name. Anything downstream then has to
invent a tie-break from frame arrival order, which can swap between flights and exchange
two feeds mid-broadcast. This appends an ordinal derived from `part.flightID` — persistent,
and it survives an FMRS jump — plus the flightID itself after an `@` so a returning camera
is recognisable with certainty. Idempotent, and it never touches a save file.

---

## `conn.mech_jeb`

Note the Python name: kRPC snake-cases `MechJeb` to `mech_jeb`.

### Two things it is for

**Fly the ascent under script control.** Set the target orbit and turn, then engage.

**Fly the ascent with MechJeb while deciding the staging yourself.** MechJeb stages when
the current stage has no active engines left. On a launcher whose side boosters and core
sit in the same stage, that stays false for as long as the core burns — so the side
boosters are never dropped. No setting fixes it; the criterion is the wrong question for
that vehicle shape. Setting `autostage = False` removes the ascent autopilot from the
staging controller's user pool and *only* that one: it stays registered with attitude and
thrust, so MechJeb keeps flying while the staging decision comes back to you.

| Member | Type | Notes |
|---|---|---|
| `available` | `bool` | Mod installed and resolved. |
| `diagnostics` | `str` | |
| `on_vessel` | `bool` | This craft carries a running MechJeb part. Distinct from `available`, and **when it is `False` every member below raises `RuntimeError`** — the three cached `*_names` lists still answer. Never raises itself. |
| `ping()` | `str` | |
| `ascent_enabled` | `bool` r/w | Engage / disengage. **Does not round-trip**: the setter adds or removes *your* handle, the getter reads the module's state, so if the player also engaged from the GUI you can set `False` and read back `True`. See below. |
| `disengage_ascent()` | — | Stop outright, whoever asked. |
| `ascent_path` | `str` r/w | Case-insensitive on write; an unknown value raises **with the list this MechJeb offers**. Read it or provoke it rather than hard-coding: current builds say `"CLASSIC"` and `"PSG"`, older ones said `"PVG"`. Set it before engaging and never during — the autopilot handle is resolved through this value. |
| `autostage` | `bool` r/w | **Set before engaging, and it does not go back.** See below. |
| `staging_users` | `int` | How many users the staging controller has. 0 is what `autostage = False` should produce, so this is how you *check* it took. Non-zero is not proof of failure, though — MechJeb's own windows and other autopilots join the same pool. `-1` means the pool could not be read. |
| `release_staging()` | `str` | Remove the autopilot from the staging pool by hand. A fallback if `autostage` has been renamed. Returns one of four English sentences; success is not machine-distinguishable from "there was nothing to release". |
| `ascent_setting_names` | `list[str]` | `Name : Type` for every numeric setting **this installed MechJeb has**. Cached. Names beginning with `_` are filtered out — see below. |
| `ascent_flag_names` | `list[str]` | Every boolean setting. Cached, same filter. |
| `ascent_setting(name)` | `float` | Case-insensitive. `ValueError` if this MechJeb has no such setting — the message names the list property to read — or if the name resolves to something non-numeric. |
| `set_ascent_setting(name, value)` | — | Same lookup and same `ValueError`s. `RuntimeError` if the setting turns out to be read-only. Takes effect immediately, but see the ordering note below. |
| `ascent_flag(name)` | `bool` | The boolean counterpart. `ValueError` if the name is unknown *or* is not a boolean, so a numeric setting cannot be read through this by mistake. |
| `set_ascent_flag(name, value)` | — | Same. `RuntimeError` if read-only. |
| `core_members` | `list[str]` | Public members found on `MechJebCore`, for when a rename breaks a lookup. Rows are `field <Name> : <Type>` or `prop  <Name> : <Type>`. Cached. |

**Order matters, and `autostage` is one-way.** Set `autostage` and the ascent settings
*before* `ascent_enabled = True`: the autopilot reads `autostage` as it is enabled, to decide
whether to register with the staging controller at all.

Setting it back to `True` afterwards will **not** re-register. That is a defect in MechJeb,
not here — its setter only re-registers under a condition that MechJeb itself never makes
true, so that branch is unreachable. To genuinely hand staging back, cycle `ascent_enabled`
off and on. `staging_users` is how you tell.

**Underscore names are hidden on purpose.** MechJeb declares the backing field of several
settings as public next to the property that wraps it — `_autostage` sits beside
`Autostage`. Writing the field skips the property's side effect, which for this one is the
staging registration, so `set_ascent_flag("_autostage", False)` would change the flag and
leave MechJeb staging anyway: the exact silent failure this service exists to prevent. Two
names, one real. The lists only offer the real one.

**Engaging composes with the GUI.** `ascent_enabled = True` adds this bridge to the
autopilot's user pool, which is the path MechJeb's own window uses. The module is enabled
while it has at least one user, so if you engaged from a script and the player also clicked
Engage, setting it back to `False` withdraws only your request and leaves theirs standing.
`disengage_ascent()` is "stop, whoever asked".

**Settings are addressed by name on purpose.** `ascent_setting_names` is generated from the
live assembly, so it is right for the build actually in your GameData. A MechJeb that
renames a knob costs a string in your script, not a rebuilt DLL — which is exactly the
failure that stopped the previous kRPC-to-MechJeb bridge from loading against 2.15.

Units are MechJeb's own — metres, m/s, degrees. Its `EditableDoubleMult` knobs (altitudes,
mostly) store the SI value, so 100 km is `100000`, not `100`.

```python
mj = conn.mech_jeb
print("\n".join(mj.ascent_setting_names))       # what this build actually has

mj.ascent_path = "CLASSIC"
mj.set_ascent_setting("DesiredOrbitAltitude", 120000)
mj.set_ascent_setting("TurnStartAltitude", 2000)
mj.autostage = False                             # before engaging
mj.ascent_enabled = True
assert mj.staging_users == 0                     # check it took
```

---

### Any module, by name

The members above cover the ascent path, which is what this service was built for. Beyond
it, MechJeb has twenty other modules and roughly two hundred settings, and they are all
reachable through five accessors rather than two hundred procedures.

Nothing here is hard-coded. `modules` is read from the MechJeb in your GameData, and
`describe_module()` reports each member's name, how to reach it, its type, whether it can
be written and whether writing it touches the player's saved configuration. A MechJeb
release that renames a setting costs you a string, not a mod update — which is the whole
reason the previous bridge to MechJeb is dead and this one is not.

| Member | Type | Notes |
|---|---|---|
| `modules` | `list[str]` | The modules MechJebCore publishes by short name: `Ascent`, `AscentSettings`, `Attitude`, `Landing`, `Node`, `Staging`, `Target`, `Thrust`, `Hoverslam`, `SmartASS`, `StageStats`, `Warp` and the rest. |
| `describe_module(module)` | `list[str]` | Rows `name⇥channel⇥type⇥rw⇥persistence`. **Read this first.** |
| `module_enabled(module)` | `bool` | Whether MechJeb is running it right now. |
| `module_users(module)` | `int` | How many things want it running. `-1` if the pool could not be read. |
| `engage(module)` | `int` | Ask for a module. Returns the resulting user count. |
| `disengage(module)` | `int` | Withdraw your request. Returns the remaining user count. |
| `setting(module, name)` | `float` | Read a number. |
| `set_setting(module, name, value)` | — | Write a number. SI units: 100 km is `100000`. |
| `flag(module, name)` | `bool` | Read a boolean. |
| `set_flag(module, name, value)` | — | Write a boolean. |
| `enum_value(module, name)` | `str` | Read a multiple-choice setting **by name** — `"KEEP_SURFACE"`, not `2`. |
| `set_enum_value(module, name, value)` | — | Write one. Case-insensitive; an unknown value raises with the list this build offers. |
| `enum_options(module, name)` | `list[str]` | What that choice accepts. Empty if the member is not a choice. |
| `list_value(module, name)` | `str` | Read an integer list, in MechJeb's own text form. |
| `set_list_value(module, name, value)` | — | Write one. Both `"1,2,3"` and `"1-3"` are accepted. |
| `text_value(module, name)` | `str` | Read anything as text — the status strings, or a member whose type has no channel. |

**The channel column tells you which accessor to use.** `number` → `setting`, `flag` →
`flag`, `enum` → `enum_value`, `list` → `list_value`, `text` → read-only via `text_value`,
`unsupported` → the member exists but its type cannot cross kRPC; it is listed rather than
hidden so you know it is there.

**Watch the persistence column.** `persistent:GLOBAL` means writing that setting changes
the player's MechJeb configuration for **every vessel in every save on that install**, not
just this flight. Most ascent settings are global. That is MechJeb's design and the bridge
does not block it, but it is worth knowing before a script tunes something.

**Engaging is not uniform, and the differences are handled for you.** Most modules run
while at least one user wants them. Some pin themselves — `Thrust` holds the throttle
limiters, `Target` is what everything else aims at, `StageStats` is the delta-v simulation,
`Hoverslam` is the landing predictor — so `engage` on those is a no-op and `disengage` is
refused rather than silently breaking the rest of MechJeb. `AscentSettings` and `Settings`
are settings bags with no autopilot behind them and cannot be engaged at all; the ascent
autopilot is `Ascent`.

Three modules need more than a pool entry and have their own procedures below. And in a
career save MechJeb may disable a module again a frame later if the part or tech is not
researched, without an error — which is why `engage` returns the user count instead of
nothing, and why `module_enabled` is worth checking after.

```python
mj = conn.mech_jeb
print("\n".join(mj.describe_module("Staging")))
# AutostageLimit          number  EditableInt         rw  persistent:GLOBAL
# DropSolids              flag    Boolean             rw  persistent:GLOBAL
# FairingMaxDynamicPressure number EditableDoubleMult rw  persistent:GLOBAL
# HotStaging              flag    Boolean             rw  persistent:GLOBAL

mj.set_setting("Staging", "AutostageLimit", 3)      # stage, but never past stage 3
mj.set_flag("Staging", "DropSolids", True)          # drop solid boosters mid-burn
mj.set_setting("Staging", "FairingMinAltitude", 55000)
```

### The landing predictor

MechJeb runs a suicide-burn solver whenever you are in flight, republishing about once a
second. Nothing has to be engaged and reading it costs a field access, so these are safe to
stream.

| Member | Type | Notes |
|---|---|---|
| `landing_predicted` | `bool` | Whether there is a solution at all. |
| `landing_latitude` | `float` | Predicted impact latitude, degrees. `NaN` with no solution. |
| `landing_longitude` | `float` | Predicted impact longitude, degrees. `NaN` with no solution. |
| `ignition_countdown` | `float` | Seconds until the burn must start. `NaN` with no solution. |
| `landing_countdown` | `float` | Seconds until touchdown. `NaN` with no solution. |
| `landing_delta_v` | `float` | Delta-v the burn needs, m/s. `NaN` with no solution. |
| `landing_slope` | `float` | Terrain slope at the predicted site, degrees. `NaN` with no solution. |

**Stock kRPC has no impact prediction of any kind**, and this one propagates through the
atmosphere rather than guessing ballistically. It is the number a boostback burn exists to
null out, and an independent check on your own descent solver. Test `landing_predicted`, or
`math.isnan()`, before using any of the others.

### Landing, nodes and SmartASS

Three modules do nothing useful when merely enabled, because their real entry point is a
method.

| Member | Notes |
|---|---|
| `land_at_target()` | Start the landing autopilot on the current target site. **Deletes every maneuver node on the vessel** — MechJeb does that itself. Competes with any descent guidance of your own. |
| `land_untargeted()` | Same, with no target: come down wherever the trajectory leads. |
| `stop_landing()` | The correct way out. Withdrawing from the pool leaves it enabled with a step still set. |
| `execute_node()` | Execute the next maneuver node. |
| `execute_all_nodes()` | Execute every node in turn. |
| `abort_node()` | Stop executing. |
| `smart_ass_engage()` | Push SmartASS's mode and target to the attitude controller. Setting its members does nothing without this — and with `autoDisableSmartASS` on, which is the default, SmartASS stands down whenever another autopilot takes attitude. |

### The maneuver planner

MechJeb can compute a Hohmann transfer, a plane match, an intercept, a moon return, a
resonant orbit and a dozen other burns. This exposes all of them, and the design is worth a
sentence because it decides how you use the result.

**The nodes are ordinary KSP maneuver nodes.** MechJeb computes the burn and places it; the
bridge then gets out of the way. You read it back with **stock kRPC's
`vessel.control.nodes`** — prograde, normal, radial, UT — and execute or delete it with
tooling you already have. No vector, tuple or MechJeb type crosses this boundary, which is
also why the whole planner needs only eleven procedures.

| Member | Type | Notes |
|---|---|---|
| `maneuver_operations` | `list[str]` | Operations by **class** name: `OperationCircularize`, `OperationApoapsis`, `OperationGeneric` (Hohmann transfer), `OperationPlane`, `OperationLambert`, `OperationMoonReturn`… |
| `maneuver_operation_name(op)` | `str` | The label MechJeb shows, in the game's language. Display only. |
| `describe_maneuver(op)` | `list[str]` | Its parameters, same format as `describe_module`. Includes the burn-time parameters. |
| `maneuver_parameter(op, name)` | `float` | Read a numeric parameter. |
| `set_maneuver_parameter(op, name, v)` | — | Write one. SI units: a 200 km apoapsis is `200000`. |
| `maneuver_flag(op, name)` | `bool` | Read a boolean parameter. |
| `set_maneuver_flag(op, name, v)` | — | Write one. |
| `maneuver_time_references(op)` | `list[str]` | When this operation will accept the burn: `"APOAPSIS"`, `"CLOSEST_APPROACH"`, `"X_FROM_NOW"`… Empty means it computes its own timing. |
| `set_maneuver_time_reference(op, ref)` | — | Choose one. Rejected, with the list it accepts, if the operation does not allow it. |
| `create_maneuver_nodes(op, append=True)` | `int` | **The one that acts.** Plans and places; returns how many nodes it made. |
| `maneuver_warning` | `str` | A caveat from the last plan, or empty. |

**Class names, not labels.** MechJeb's operation labels are translated, one is hardcoded
English while the rest are not, and one has a stray quotation mark in the English file. A
script keyed on a label breaks on a French install.

**There is no absolute-UT burn time.** For a specific moment, select `"X_FROM_NOW"` and set
`LeadTime` to the number of seconds from now.

**The burn-time selection is shared with MechJeb's own window.** It is one object per
operation *type*, not per instance, so setting it also changes what the player sees in the
Maneuver Planner — and a player changing it there changes what your script gets. MechJeb's
design, not a choice made here.

**`append=True` chains plans.** With nodes already present, the operation is planned from
the end of the last one rather than from now — so circularising after a change of apoapsis
circularises at the *new* apoapsis, which is what MechJeb's own window does.

**Failure is explained, warnings are not raised.** An impossible burn — no target, target
in a different sphere of influence, no ascending node with it, an apoapsis below the
surface — throws with MechJeb's own message. But three operations plan a perfectly good
burn and still have something to say, so that goes to `maneuver_warning` instead of
rejecting a usable plan.

**Interplanetary transfers are not supported.** `OperationAdvancedTransfer`'s solver is
created by MechJeb's GUI, so driving it headlessly returns "Started computation" forever
rather than a plan. It is the one thing here that has to be done another way.

```python
mj = conn.mech_jeb
vessel = conn.space_center.active_vessel

conn.space_center.target_vessel = station
conn.space_center.wait()                      # MechJeb sees the target next tick, not now

mj.set_maneuver_time_reference("OperationPlane", "REL_HIGHEST_AD")
mj.create_maneuver_nodes("OperationPlane")    # match planes

mj.set_maneuver_parameter("OperationGeneric", "LagTime", 0)
mj.create_maneuver_nodes("OperationGeneric")  # then a Hohmann transfer, from the last node

for node in vessel.control.nodes:             # ordinary kRPC nodes from here on
    print(f"{node.ut:.0f}  {node.delta_v:.1f} m/s")

if mj.maneuver_warning:
    print("MechJeb says:", mj.maneuver_warning)
```
