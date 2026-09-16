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
- [`conn.actuators`](#connactuators) — per-engine thrust leases and bulk gimbal state
- [`conn.fmrs`](#connfmrs) — dropped stages, jumps, recovery ledger
- [`conn.ocisly`](#connocisly) — camera streams across a scene reload
- [`conn.mech_jeb`](#connmech_jeb) — ascent, staging, every module by name, the landing
  predictor and the maneuver planner
- [`conn.trajectories`](#conntrajectories) — Trajectories' impact prediction, landing
  target and descent profile

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

## `conn.discovery`

Read-only introspection of the assemblies and types loaded by KSP. This service never
invokes arbitrary methods and never writes fields or properties. Bulk records are packed
as tab-separated strings and returned as real `List` instances, not arrays.

| Member | Returns | Meaning |
|---|---|---|
| `list_assemblies(filter)` | `list[str]` | Loaded assemblies matching comma-separated name fragments. Rows are `name`, version and full name. Empty or `"*"` matches all. |
| `search_types(assembly_fragment, query, limit)` | `list[str]` | Full type names containing the query. The limit defaults to 50 when non-positive and is capped at 500. |
| `describe_member(assembly_fragment, type_name, member_name)` | `list[str]` | Exact members or overloads as nine fields: kind, declaring type, visibility, static/instance, value type, parameters, accessors, metadata token and attributes. |
| `type_hierarchy(assembly_fragment, type_name)` | `list[str]` | Base chain followed by implemented interfaces. Unknown types return an empty list. |
| `find_part_modules(query)` | `list[str]` | Module types instantiated on the active vessel, with assembly, count and sample part titles. Empty outside a loaded flight. |
| `runtime_type_fingerprint(assembly_fragment, type_name)` | `str` | Stable member count and SHA-256 used to compare the loaded type with an offline index. |

These calls establish that a member exists and what is loaded. They do not establish that
the member is on the active behavior path or that using it improves a flight.

---

## `conn.ident`

Stable KSP identities that kRPC's public `Vessel` and `Part` objects hold internally but
do not expose. The service is available in flight scenes when SpaceCenter resolved.

| Member | Returns | Meaning |
|---|---|---|
| `available` | `bool` | Whether the kRPC SpaceCenter service resolved. |
| `ping()` | `str` | Returns `"pong"`. |
| `part_flight_id(part)` | `str` | The part's KSP `flightID` as an exact decimal string. |
| `part_flight_ids(parts)` | `list[str]` | One `flightID` per input part, preserving input order. |
| `vessel_flight_ids(vessel)` | `list[str]` | All `flightID` values on a loaded vessel; an unloaded vessel returns an empty list. |
| `vessel_ids(vessel)` | `str` | Tab-separated KSP `persistentId` and kRPC vessel Guid. |

Use part `flightID` to address `conn.actuators`, to disambiguate otherwise identical side
boosters, or to join OCISLY camera names. Use `persistentId` to join FMRS records to live
kRPC vessel objects.

---

## `conn.actuators`

Stock KSP actuator access used by the GNC S7 prototype. Reads are grouped so one call
samples every engine or every gimbal on the active vessel. Commands use short leases:
if the Python client stops renewing, the plugin restores the engine's previous fields
within at most one second.

| Member | Returns | Meaning |
|---|---|---|
| `available` | `bool` | Always `True` when the service loaded. |
| `ping()` | `str` | Returns `"pong"`. |
| `engine_sample()` | `list[float]` | Rows of 9 values: part `flight_id`, engine ordinal, ignited, realized throttle, realized thrust, max thrust, thrust limit, independent mode, independent percentage. |
| `gimbal_sample()` | `list[float]` | Rows of 8 values: part `flight_id`, gimbal ordinal, locked, limiter, range, measured local X/Y/Z actuation in degrees. During a lease, response-speed limiting can make this lag the target. |
| `thrust_direction_sample()` | `list[float]` | Rows of 5 values: part `flight_id`, engine ordinal, then unit direction X/Y/Z in the vessel frame (right, forward, bottom). Reads each current `ModuleEngines.thrustTransforms`, so it remains valid after a revert even when stock kRPC's cached `Thruster` wrapper is stale. |
| `thrust_position_sample()` | `list[float]` | Rows of 5 values: part `flight_id`, engine ordinal, then the thrust-weighted lever X/Y/Z from the current centre of mass in the vessel frame. Prefer the per-transform v2 rows for exact multi-nozzle geometry. |
| `lease_independent_throttle(flight_id, engine_ordinal, percentage, lease_seconds=0.25)` | `bool` | Gives one engine an absolute independent throttle for 0.05–1 s. Renew to continue. |
| `release_independent_throttle(flight_id, engine_ordinal)` | `bool` | Restores the fields saved when that engine's first lease began. |
| `lease_gimbal(flight_id, gimbal_ordinal, x_degrees, y_degrees, lease_seconds=0.25)` | `bool` | Directly commands one gimbal's local X/Y target for 0.05–1 s. Refuses a locked gimbal, clamps each direction to its installed KSP limits and preserves the configured response speed. Renew to continue. |
| `release_gimbal(flight_id, gimbal_ordinal)` | `bool` | Restores the rotations and active state saved when that gimbal's first lease began. |
| `release_all()` | `int` | Restores every active throttle and gimbal lease and returns the number restored. |
| `protocol_version` | `int` | `2` for the atomic snapshot/control-frame protocol below. |
| `control_snapshot_v2()` | `list[float]` | One coherent, timestamped snapshot containing all engines, gimbals and individual thrust transforms. |
| `acquire_control(owner, lease_seconds=1)` | `str` | Acquires exclusive v2 engine/gimbal authority for 0.1–5 s and returns an opaque token. Existing legacy leases are restored during this explicit handover. |
| `renew_control(token, lease_seconds=1)` | `bool` | Renews exclusive authority. It does not extend an actuator command: frames still need to be refreshed. |
| `apply_control_frame(token, sequence, apply_tick, valid_until_tick, lease_seconds, engine_commands, gimbal_commands)` | `int` | Validates a complete frame and queues it for one `FixedUpdate`; returns its scheduled tick. |
| `control_status_v2()` | `list[float]` | Ownership, queue, acknowledgement and result state described below. |
| `release_control(token)` | `int` | Cancels the pending frame, restores all v2 actuator leases and releases authority. |

`thrust_limit` and independent throttle are different controls. The former is a ceiling
already exposed by stock kRPC's `Engine.thrust_limit`; the latter makes one engine stop
following the vessel's main throttle. S7 uses the second mechanism for differential
thrust.

`lease_gimbal()` does not merely write `ModuleGimbal.actuationLocal`: KSP would overwrite
that field and the nozzle transforms in `ModuleGimbal.FixedUpdate`. The lease temporarily
disables that stock calculation, applies the same installed-KSP transform formula with an
absolute X/Y command, advances toward it with the installed response-speed rule, and
reapplies it each physics tick. Expiry, explicit release, scene change and add-on
destruction all restore the captured actuation, rotations and stock active state.
This is actuator authority only; it is not evidence that a guidance law improves flight.

### Atomic protocol v2

`control_snapshot_v2()` has a 14-double header:

| Offset | Meaning |
|---:|---|
| 0–7 | protocol version, physics tick, UT, fixed delta time, vessel persistent id, topology generation, last applied sequence, last result code |
| 8–9 | engine row count and stride (`18`) |
| 10–11 | gimbal row count and stride (`19`) |
| 12–13 | thrust-transform row count and stride (`10`) |

The three row sections immediately follow the header, in that order.

An engine row is: `flight_id, ordinal, ignited, requested_throttle,
current_throttle, final_thrust, max_thrust, thrust_percentage,
independent_throttle, independent_percentage, finite_response,
acceleration_speed, deceleration_speed, requested_mass_flow,
propellant_requirement_met, real_isp, thrust_transform_count, leased`.

A gimbal row is: `flight_id, ordinal, locked, stock_active, limiter, range,
range_x_negative, range_x_positive, range_y_negative, range_y_positive,
finite_response, response_speed, actual_x, actual_y, actual_z,
transform_count, leased, target_x, target_y`.

A thrust-transform row preserves the geometry that an engine-level average loses:
`flight_id, engine_ordinal, transform_index, multiplier, lever_x, lever_y,
lever_z, direction_x, direction_y, direction_z`. Lever and direction use the vessel
frame (right, forward, bottom); the lever is measured from the current centre of mass.
Unavailable transform geometry is represented by six `NaN` values, never plausible zeros.

`apply_control_frame()` accepts engine rows of `flight_id, ordinal, percentage` and
gimbal rows of `flight_id, ordinal, x_degrees, y_degrees`. All identifiers must be exact
integers carried as doubles and no actuator may appear twice. `sequence` is strictly
increasing for one authority token. Use `apply_tick=0` for the next observed physics tick
and `valid_until_tick=0` for the same tick; explicitly scheduled frames are bounded to 250
ticks ahead. The method only queues after every row has resolved and validated. The add-on
then applies the whole frame from one `FixedUpdate` callback.

`control_status_v2()` returns 11 doubles: `protocol, tick, topology_generation,
owner_active, owner_ttl_seconds, pending_sequence, pending_tick,
last_accepted_sequence, last_applied_sequence, result, last_applied_tick`.

Result codes are `0` none, `1` queued, `2` applied, `-1` frame expired, `-2` authority
lost/expired, `-3` topology or active vessel changed, `-4` unexpected application failure,
and `-5` a queued frame was superseded. A topology change invalidates the token rather
than applying commands addressed to stale part/module ordinals. While v2 authority is
active, legacy per-actuator lease procedures reject commands so two control paths cannot
silently fight.

The part id is returned as a double in the flat samples because all KSP `uint` flight ids
are exactly representable by a double. Pass it back as a decimal string to command or
release a lease.

### Dynamics protocol v3 (PDG2)

`dynamics_snapshot_v3()` is a **read-only FixedUpdate snapshot** designed for the new PDG2
planner. Unlike `control_snapshot_v2()`, it is captured automatically at the end of every
`ActuatorsWatcher.FixedUpdate`, after any queued actuator frame and leased-gimbal update
for that callback. The next frames therefore show the physical response to a known
`last_applied_sequence` / `last_applied_tick` pair.

The bridge keeps the latest 300 frames (about six seconds at 50 Hz). Use
`dynamics_frames_v3(since_tick, max_frames)` to retrieve the ring without relying on the
Python/RPC polling cadence. `max_frames` is clamped to 1–128. A topology or active-vessel
change clears the ring rather than mixing incompatible actuator row layouts.

The v3 snapshot starts with a 22-double header:

| Offset | Meaning |
|---:|---|
| 0–1 | protocol (`3`) and schema (`1`) |
| 2–6 | physics tick, UT, fixed delta time, vessel persistent id, topology generation |
| 7–10 | last accepted sequence, last applied sequence, last applied tick, actuator result |
| 11 | capability bit mask; absent quantities are `NaN`, never plausible zeros |
| 12 | state stride (`67`) |
| 13–14 | engine count / stride (`20`) |
| 15–16 | gimbal count / stride (`19`) |
| 17–18 | thrust-transform count / stride (`10`) |
| 19–20 | history capacity and number of frames present before this capture |
| 21 | capture phase (`0` = pre-integration state sampled from the FixedUpdate callback) |

The 67-value state block immediately follows the header. Its fields, in order, are:

```text
position_world_xyz
orbital_velocity_world_xyz
surface_velocity_world_xyz
surface_acceleration_derived_xyz
orbital_acceleration_derived_xyz
rotation_world_xyzw
angular_velocity_world_xyz
angular_acceleration_derived_xyz
mass_tonnes
com_world_xyz
moi_native_xyz
gravity_world_xyz
atm_density
static_pressure_kpa
dynamic_pressure_kpa
mach
altitude_asl_m
radar_altitude_m
latitude_deg
longitude_deg
global_throttle
aero_force_raw_xyz
aero_torque_raw_xyz
drag_vector_raw_xyz
lift_vector_raw_xyz
surface_speed_ms
orbital_speed_ms
gee_force_raw
situation
mission_time_s
packed
surface_velocity_reference_xyz
aoa_raw
sideslip_raw
```

Important unit rule: **KSP native force/mass units are kept where the stock module exposes
them**. `mass_tonnes` is therefore tonnes and engine `final_thrust` / `max_thrust` are kN,
which preserve the useful `kN / tonne = m/s²` relationship without a silent factor of
1000. The aerodynamic/attitude fields obtained reflectively are not assumed present: the
capability mask says which blocks resolved in this KSP build, and unresolved entries stay
`NaN`. In particular, do not use `aoa_raw`, `sideslip_raw`, `aero_force_raw` or `aero_torque_raw`
in a flight-critical model until the live probe has confirmed the installed build's field,
frame and units. The `_raw` suffix is deliberate.

The engine rows are:

```text
flight_id, ordinal, ignited, requested_throttle, current_throttle,
final_thrust_kn, max_thrust_kn, thrust_percentage,
independent_throttle, independent_percentage,
finite_response, acceleration_speed, deceleration_speed,
requested_mass_flow, propellant_requirement_met, real_isp_s,
thrust_transform_count, leased, flameout, min_thrust_kn
```

The gimbal and thrust-transform row schemas are the same as protocol v2. This duplication
is intentional: PDG2 can consume state and actuator realization from **one physics-tick
frame**, instead of joining independently sampled RPCs later.

`dynamics_frames_v3()` returns a six-value history header:

```text
protocol, schema, frame_count, oldest_tick, newest_tick, dropped_before
```

followed by `frame_length, frame_payload` for each returned v3 frame. `dropped_before=1`
means the requested `since_tick` predates the oldest frame still in the ring.

`dynamics_status_v3()` returns:

```text
protocol, schema, latest_tick, history_count, history_capacity,
capability_mask, latest_applied_sequence, latest_applied_tick, actuator_result
```

`python/dynamics_v3.py` is the canonical dependency-free decoder for both RPCs. Keep the
PDG2 code behind that decoder instead of hard-coding offsets in multiple places.

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

---

## `conn.trajectories`

Trajectories predicts where the active vessel hits the ground by integrating its descent
through the stock aerodynamics — the drag cubes, at the vessel's *actual* attitude or at
the descent profile you set. Stock kRPC has no impact prediction of any kind, and MechJeb's
landing predictor only runs while its landing module is engaged; this one runs whenever you
are in flight. It is a **witness, not an autopilot**: the prediction ignores thrust and
answers "where do I land if I cut the engines now".

**The first five members are frozen.** `available()`, `has_impact()`, `get_impact_geo()`,
`get_impact_position()` and `get_time_till_impact()` keep the names, shapes and failure
behaviour of the previous single-DLL bridge, because existing clients pin them. Note that
`available()` is a *procedure* here — call it with parentheses — where every other
service has a property of that name.

| Member | Type | Notes |
|---|---|---|
| `available()` | `bool` | Mod installed and its API resolved. A procedure, not a property. |
| `diagnostics` | `str` | What resolution found, member by member. |
| `ping()` | `str` | |
| `mod_version` | `str` | The Trajectories assembly version, e.g. `"2.4.5.4"`; empty when absent. |
| `api_version` | `str` | What the mod's own API says, e.g. `"2.4.5"`. Raises when absent. |
| `has_impact()` | `bool` | A prediction exists for the active vessel. Never raises. |
| `get_impact_geo()` | `list[float]` | `[lat_deg, lon_deg, terrain_alt_m]` on the active vessel's main body. **Raises** `RuntimeError` when there is no impact — guard with `has_impact()`. |
| `get_impact_position()` | `list[float]` | The mod's raw vector `[x, y, z]` in metres: relative to the main body's centre, world axes, rotated to where that surface point is *now*. Add the body's position for a world position. **Raises** when there is no impact. |
| `get_time_till_impact()` | `float` | Seconds to impact, or `-1` if none. Never raises. |
| `get_end_time()` | `float` | Universal time of the impact, or `-1`. |
| `get_impact_velocity()` | `list[float]` | `[x, y, z]` m/s, world axes. **Empty list** if no impact. |
| `update_trajectory()` | — | Ask for a recompute now. Lands on a later tick; does not block. |
| `always_update` | `bool` r/w | Integrate even with the mod's window closed. The bridge turns it on at every flight-scene start so a headless script gets a live prediction. |
| `has_target()` | `bool` | A landing target is set. |
| `set_target(lat, lon, alt)` | — | Degrees on the active vessel's main body; pass `float('nan')` as the altitude to put it on the ground there. |
| `clear_target()` | — | |
| `get_target()` | `list[float]` | `[lat_deg, lon_deg, alt_m]`, or an empty list. |
| `get_planned_direction()` | `list[float]` | The navball marker the mod draws for "fly this way to the target", `[x, y, z]` world axes. Empty list without a target. |
| `get_corrected_direction()` | `list[float]` | Its second marker: the correction to bring the predicted impact onto the target. Empty list without a target. |
| `get_descent_profile_angles()` | `list[float]` | Four angles in **degrees**: entry, high altitude, low altitude, final approach. |
| `get_descent_profile_modes()` | `list[bool]` | Per node: `True` = the angle is an angle of attack, `False` = measured from the horizon. |
| `get_descent_profile_grades()` | `list[bool]` | Per node: `True` = retrograde. |
| `set_descent_profile(angles, modes, grades)` | — | All three lists of four, same order. `ValueError` on any other length. Degrees in. |
| `reset_descent_profile(aoa_deg)` | — | All four nodes to one angle of attack. `0` = prograde, `180` = tail first; anything beyond 90 in magnitude is retrograde. |
| `retrograde_entry` | `bool` r/w | All four nodes retrograde. Writing `True` resets the profile to retrograde at 0° AoA. |
| `prograde_entry` | `bool` r/w | All four nodes prograde. Writing `True` resets to prograde at 0° AoA. |

**Two conventions for "nothing to report", on purpose.** The two frozen vector members
raise, as they always have, so the guard is `has_impact()`. Every *new* vector member
returns an empty list and the two time members return `-1`, so a control loop can poll
them without exceptions. A member the installed Trajectories does not have raises
`RuntimeError: Trajectories.API.<name> absent (v2.4.5.4)` — never a `NullReferenceException`
— and `diagnostics` lists every such member at once.

**Angles are degrees here, radians in the mod.** `Trajectories.API` takes and returns
radians for `ResetDescentProfile` and `DescentProfileAngles`; this service converts, so
`reset_descent_profile(180)` is tail first. The mod's GUI shows degrees too.

**Set the descent profile before you need the number.** The mod integrates the attitude it
is told about, and by default that is the vessel's current one. A booster still pointed
for its boostback is predicted as if it would fall nose-first all the way down, which is
the wrong drag by a wide margin. One line at boot fixes it:

```python
tr = conn.trajectories
if tr.available():
    tr.reset_descent_profile(180)         # tail first, zero AoA, all four nodes
    # or, with a trim: tr.set_descent_profile([180, 180, 175, 170], [True]*4, [True]*4)

# later, in the loop
if tr.has_impact():
    lat, lon, alt = tr.get_impact_geo()
    t = tr.get_time_till_impact()
```

**It has a cadence of its own.** The mod recomputes on its timer, not on each read, so the
prediction can be one or two seconds old while the vessel is sweeping its impact point at
hundreds of metres per second. `update_trajectory()` asks for a fresh one but the answer
still arrives on a later tick. Read `get_time_till_impact()` with the position and you
can tell two consecutive reads of the same computation apart from two computations.

**Frames.** `get_impact_geo()` is the member to use: latitude and longitude are what
`body.surface_position(lat, lon, frame)` on stock kRPC turns into any frame you like.
`get_impact_position()` is the mod's own vector — body-relative, world axes, already
rotated to the current position of that surface point — kept for clients that were already
doing that sum themselves.

**Why some of `Trajectories.API` is not here.** `GetSpaceOrbit()` returns a stock `Orbit`
that kRPC's own `vessel.orbit` already gives you; the three split version numbers are
folded into `api_version`.
