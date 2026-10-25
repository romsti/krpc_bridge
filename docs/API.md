# API reference

Every remote procedure KRPC.Bridge adds, by service. Names are given in their Python
form; kRPC converts C# `PascalCase` to `snake_case` on the wire, so `DroppedVessels`
becomes `conn.fmrs.dropped_vessels`.

`help(conn.fmrs)` in a Python REPL gives you the same text as this file, generated from
the XML shipped next to each DLL. This document exists so you can read it without
launching the game.

**Contents**

- [Conventions](#conventions)
- [`conn.bridge`](#connbridge) — Core: what loaded and which build (`manifest_v1`), events,
  jobs, HUD, reflection probe
- [`conn.discovery`](#conndiscovery) — read-only introspection of the running game
- [`conn.ident`](#connident) — part flightID, vessel persistentId and Guid
- [`conn.control`](#conncontrol) — what flies: per-engine thrust and per-gimbal leases, the
  atomic protocol v2, AttitudeV1 and its lateral force (refactor R3: the commanding half of
  the former `conn.actuators`)
- [`conn.telemetry`](#conntelemetry) — what observes: dynamics frames (V3, Ext, tracked
  vessels), aero actuators, aerodynamic wrenches, actuator samples (refactor R3: the
  observing half of the former `conn.actuators`)
- [`conn.stock`](#connstock) — stock utilities (`conn.stock_delta_v` before R3): delta-v
  simulation and stage table,
  staging from the frame loop, clamp-free aero force, landing-leg deployment
- [`conn.fmrs`](#connfmrs) — dropped stages, jumps, recovery ledger
- [`conn.ocisly`](#connocisly) — camera streams across a scene reload
- [`conn.trajectories`](#conntrajectories) — Trajectories' impact prediction, landing
  target and descent profile
- [`conn.tower`](#conntower) — catch tower ground tests: place and hold a vessel, ground
  anchor, hinge and joint probes, exhaust damage switch, hinge lock at rest
- [`conn.tower_site`](#conntower_site) — the catch towers of a game, from the Space Center:
  list, state (loaded or saved), put one on the ground from a craft, take one away, fly to one;
  in flight, a tower's bubble (loaded first, unpacked once loaded)

---

## Conventions

**Check `available` first.** Every plugin exposes it. When it is `False`, almost every
other member of that service raises `RuntimeError`, and `diagnostics` says which
reflective lookup came back empty. This is the difference between a clear "FMRS is not
installed" and a mystery twenty minutes into a flight.

The exceptions are the ones that are meant to answer rather than fail: `ping()` always
returns `"pong"`, and `fmrs.active` and `fmrs.jump_in_progress` return `False`. The last
two are worth remembering — a script that expected an exception gets a quiet `False` instead.

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

**Typed errors** (refactor R1, `conn.control` and `conn.telemetry`). A refusal says why in a code at the start of its
message, `"[CODE] text"`: branch on the code, never on the text. The exception type and the text
after the code are unchanged (a client that matched the text still does). A message without a
code is an invalid argument (a value outside its documented range, a malformed list).

| Code | Meaning | What the client does |
|---|---|---|
| `UNSUPPORTED_MODE` | a mode, option or bit this build does not support (refused, never degraded) | fall back to a mode the build has (read the capabilities first) |
| `TOKEN_EXPIRED` | the token was valid and its lease ran out (wall clock) | re-arm / re-acquire |
| `TOKEN_INVALID` | the token was never valid, was released, or was replaced by a newer arming | use the current token |
| `NOT_LOADED` | no loaded vessel to act on (no active vessel, unloaded, packed) | wait for the vessel |
| `TRANSIENT` | a state that clears by itself (topology changing, gimbal transforms not ready) | retry next tick |
| `BUSY` | another owner holds it (v2 control, another arming, the vessel limit) | release the other owner, or wait |
| `NOT_FOUND` | the named part, module, vessel or layout does not exist, or the vessel is not tracked | re-read the vessel |

```python
import re
def bridge_error(exc):
    m = re.match(r"\[([A-Z_]+)\] ", str(exc))
    return m.group(1) if m else ""
```

### Service names (refactor R3)

R3 split the `Actuators` DLL by what it does and named every service in English. Procedure
names, arguments and returns did not change; only the service a procedure belongs to did:

| Before R3 | Since R3 | What moved |
|---|---|---|
| `conn.actuators` | `conn.control` | leases, `protocol_version`, `control_*`, `acquire_control`, `renew_control`, `apply_control_frame`, `release_control`, every `attitude_*` member |
| `conn.actuators` | `conn.telemetry` | `dynamics_*`, `tracked_*`, `track_vessel_v3`, `untrack_vessel_v3`, `aero_actuator_*`, `simulate_aerodynamic_wrench_batch_v1`, the four `*_sample()` |
| `conn.stock_delta_v` | `conn.stock` | every member (R3, second step) |

`available` and `ping()` exist on both new services. A client written for both generations
resolves a member through an alias table of its own: the old
name first, the new one when the loaded DLL has it. The DLLs answer only to the new names.

### One tick in every frame (refactor R3)

Every frame now ends with a **clock** section (appended, so every field before it keeps its
offset): `master_tick`, the Core game clock's tick of the physics step the frame is of
(`conn.bridge.clock_v1`, one tick per step, the same in every DLL). The counters the frames
always published (`physics_tick`, the pump tick; `attitude_tick`; `pump_physics_tick`) are
unchanged, byte for byte; join frames of different transports on `master_tick`.

| Layout | Schema before | Schema since R3 | Clock section |
|---|---:|---:|---|
| `dynamics_v3` (V3, tracked V3) | 2 | 3 | `master_tick` |
| `dynamics_ext_v1` (Ext, tracked Ext) | 1 | 2 | `master_tick` |
| `aero_actuator_v1` | 1 | 2 | `master_tick`, `vessel_persistent_id` (the vessel of the rows, which the header never named) |
| `attitude_v1` | 3 | 4 | `master_tick` |
| `attitude_translation_frames_v1` | 1 | 2 | `master_tick` |

The history wrappers (`*_frames_*`) carry the frame's schema in their header, so they say the new
one. The **status** replies did not change and keep their schema (`dynamics_status_v3` 2,
`dynamics_ext_status_v1` 1, `aero_actuator_status_v1` 1, `attitude_status_v1` 3). A decoder reads
the clock section by name (`layout_v1(name)`, section `clock`); a decoder written for the older
schema refuses the new one, as it should.

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
and `fmrs.set_vessel_state()` all return a value and all
change something. Streaming one means doing that thing sixty times a second.

*Expensive reads.* Three tiers, and the middle one is larger than it looks.

- **Never stream.** `fmrs.recovery_report()` and `fmrs.recovered_funds()` re-read a file
  from disk with their default argument; `bridge.describe_type()` reflects over a whole
  type.
- **Prefer not to stream.** Every FMRS property does a live-object lookup on *each* read — a
  scene-wide `FindObjectOfType`. On top of that,
  `fmrs.dropped_persistent_ids` walks every vessel in the game state and builds a dict,
  `fmrs.separation_times` does one reflective call per distinct save file, and
  `ocisly.hullcams` walks every loaded vessel's parts looking for camera modules. Poll
  these at the rate you actually need instead.
- **Cheap.** Everything on `conn.bridge` except `describe_type()`.

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

### Which build is loaded (`manifest_v1`)

| Member | Type | Notes |
|---|---|---|
| `manifest_v1()` | `list[str]` | Every loaded `KRPC.Bridge*` assembly, its kRPC services and the versioned layouts this build knows, with the commit it was built from. Read once per session (cached: assemblies never unload). Protocol 1, schema 1. |

Every DLL carries a build stamp written by `build.cmd` (`tools/build_stamp.py`): the git commit,
whether the tree was clean, and the commit's date (`AssemblyInformationalVersion`
`"<mod version>+<sha>"`, `AssemblyMetadata` `KRPC.Bridge.GitSha`, `GitDirty`, `SourceDate`).
The date is the commit's, not the build's, so two builds of one commit are identical bit for
bit. `manifest_v1()` reads those stamps back from the assemblies the game actually loaded.

Row 0 is a header of tab-separated `key=value` pairs:

| Key | Meaning |
|---|---|
| `protocol`, `schema` | 1 and 1. Columns are only ever appended: map them by name, from row 1. |
| `n` | Number of rows after the column row. |
| `assemblies` | Number of `KRPC.Bridge*` assemblies loaded. |
| `git_sha` | The one commit every assembly was built from; `mixed` when they differ (an old DLL among new ones); empty when none carries a stamp. |
| `git_dirty` | `1` when any assembly was built from a dirty tree or carries no stamp. |
| `consistent` | `1` when one clean build is loaded: same commit everywhere, nothing dirty. |

Row 1 names the columns; then one row per entry:

| Column | Meaning |
|---|---|
| `assembly` | Assembly name, e.g. `KRPC.Bridge.Actuators`. |
| `service` | kRPC service name, e.g. `Actuators`. |
| `layout` | Empty on a service row. On a layout row, the name of a versioned wire layout, e.g. `attitude_v1`, `dynamics_v3`, `dropped_stages_v1`. |
| `protocol`, `schema` | The protocol and schema numbers this build knows for that layout, read from the constants that define them; `?` when the constant is missing, empty when the layout has no schema number. `attitude_translation_v1` gives the newest return layout (1, 2 or 3 depending on the mode). |
| `git_sha`, `git_dirty`, `source_date` | The stamp of that assembly (`git_dirty` `"0"` or `"1"`, empty without a stamp). |
| `informational_version`, `assembly_version` | As in the assembly. |

The layouts known in this build: `control_v2`, `inactive_status_v1`, `dynamics_v3`,
`dynamics_tracked_v3`, `dynamics_ext_v1`, `aero_actuator_v1`, `aero_wrench_batch_v1`,
`attitude_v1`, `attitude_translation_frames_v1`, `attitude_translation_v1`,
`translation_evidence`, `translation_free`, `attitude_options_v1`, `attitude_capabilities_v1`,
`attitude_gimbal_pivots_v1`, `attitude_effectors_v1`, `attitude_translation_v2`, `surface_model_v1` (Actuators; lot 3, 3.1), `surface_kappa_v1` (Actuators), `attitude_surfaces_roll_v1` (Actuators), `contact_v1` (Telemetry, lot 3), `dropped_stages_v1` (FMRS) and `manifest_v1`,
`clock_v1`, `vessels_v1`, `bits_v1`, `layout_v1`, `events_v2` (Bridge). `build.cmd` runs this same code on the built DLLs right after the signature scan,
and fails when a layout names a constant that does not exist.

```python
rows = conn.bridge.manifest_v1()
header = dict(p.split("=", 1) for p in rows[0].split("\t"))
columns = rows[1].split("\t")
entries = [dict(zip(columns, r.split("\t"))) for r in rows[2:]]
if header["consistent"] != "1":
    print("mixed or dirty bridge build:", header["git_sha"])
```

The same commit is in `GameData/KRPC.Bridge/manifest.json`, written by the build next to the
DLLs (sha256 of every file): comparing the two proves that what runs is what was deployed.
`python/check_bridge.py` prints both.

### Game clock, vessel registry, bit registry (Core v2: `clock_v1`, `vessels_v1`, `bits_v1`)

Read-only, every scene. All three are text in the `manifest_v1` style: row 0 is tab-separated
`key=value` pairs (always `protocol` first), row 1 names the columns (appended only: map them by
name), then one tab-separated row per entry.

| Member | Type | Notes |
|---|---|---|
| `clock_v1()` | `list[str]` | The game clock. Header: `protocol` (1), `tick`, `fixed_time`, `ut`, `dt`, `warp_rate`, `real_time`, `hooked`, `early_opens`, `core_opens`, `n`. Columns: `name`, `value`, `last_tick`. |
| `vessels_v1()` | `list[str]` | The vessel registry. Header: `protocol` (1), `tick`, `active`, `n`. Columns below. |
| `bit_sets_v1()` | `list[str]` | One row per declared bit set: `name⇥owner⇥bits`, sorted. |
| `bits_v1(set)` | `list[str]` | The bits of one set. Header: `protocol` (1), `set`, `owner`, `n`. Columns: `bit`, `name`, `kind`, `note`. Unknown set: `ValueError` naming the known ones. |

**The clock.** One master `tick` per physics step. A step opens at the first of: a
`TimingManager` callback at stage `ObscenelyEarly` (before Precalc, Earlyish and every part's
FixedUpdate), or the Core's own `FixedUpdate`. `fixed_time`, `ut` (NaN where there is no
Planetarium: main menu, editors), `dt` (`TimeWarp.fixedDeltaTime`), `warp_rate` and `real_time`
(`Time.realtimeSinceStartup`) are the values frozen when the step opened. `hooked` is 1 while the
`ObscenelyEarly` callback is registered; `early_opens` and `core_opens` count the steps each path
opened. The rows are the tick counters the frames have always carried, hosted by the clock
with exactly their old increments: `core.fixed_updates` (= `ticks`), `actuators.pump_tick`
(the `physics_tick` of the V3, Ext, AeroActuator and control frames) and `attitude.tick` (the
AttitudeV1 and translation frames); `last_tick` is the master tick of their last increment.
That is the table that maps the old counters onto the one tick; the frames themselves are
unchanged.

**The vessel registry.** One row per vessel the mod has fingerprinted or seen in a topology
GameEvent (at most 64; cleared at a scene change):

| Column | Meaning |
|---|---|
| `persistent_id`, `active`, `loaded`, `packed`, `name` | Identity and state, refreshed at the call. |
| `event_generation`, `last_event_tick` | +1 at every topology or packing GameEvent naming the vessel (`onVesselWasModified`, `onVesselPartCountChanged`, `onVesselCreate`, `onVesselGoOnRails`/`OffRails`, `onVesselChange`, `onPartDie`, `onPartDeCoupleNewVesselComplete`, `onVesselsUndocking`), and the master tick of the last one. |
| `fingerprint`, `fingerprint_tick` | The topology fingerprint (hex) every Actuators transport compares to decide that its actuator rows still mean the same modules, and when it was last computed. |
| `walks`, `memo_hits` | Fingerprints computed, and served from the memo. The memo lives within one WINDOW of the bridge's own code (the Actuators pump, the AttitudeV1 Earlyish tick), where no game code runs between two reads: it returns exactly what a new walk would. |
| `cross_window_checks`, `unevented_changes`, `evented_changes` | Walks that another window of the same step had already done, and how often the value had changed since, with or without a topology event in between. `unevented_changes` stays at 0 if and only if the events are enough to keep one fingerprint per vessel per STEP: the evidence to collect before arming that. |

**The bit registry.** Every mask a layout publishes, with the meaning of each bit, declared by the
plugin that owns it from its own constants. `kind` is `blocking` (forbids every channel it
applies to), `blocking_roll`, `blocking_pitch_yaw`, `informational`, `capability`, `reason` or
`option`. `bit_sets_v1()` lists the sets the loaded plugins declare.

```python
rows = conn.bridge.bits_v1("attitude_v1.motifs")
names = {int(r.split("\t")[0]): r.split("\t")[1] for r in rows[2:]}
```

### Schema descriptors (`layouts_v1`, `layout_v1`)

Every frame, return, status reply and text table the bridge publishes is described here, so a
client GENERATES its decoders instead of copying offsets (refactor R1). The plugin that owns a
layout declares it at load from the constants that version it, and a decoder written from the
descriptors fails on a single value that does not fit.

| Member | Type | Notes |
|---|---|---|
| `layouts_v1()` | `list[str]` | One row per layout: `name⇥service⇥protocol⇥schema⇥kind⇥fields`, sorted. `kind`: `frame` (one physics step, in a ring), `history` (the six-value header + length-prefixed frames of every `*_frames_*` RPC), `return`, `status`, `block` (appended to another layout), `text` (tab-separated, `key=value` header, column row). |
| `layout_v1(name)` | `list[str]` | The descriptor of one layout (format below). Unknown name: `ValueError` listing the known ones. |

Row 0 is `key=value` pairs: `protocol` (1, this format), `layout`, `service`, `layout_protocol`,
`layout_schema` (the numbers the layout's own header carries), `kind`, `sections`, `fields`,
`note`. Then, per section, one line `#section⇥name⇥count⇥stride⇥prefixed⇥frame`, followed by
its fields, one line each: `section⇥offset⇥name⇥unit`.

| Section column | Meaning |
|---|---|
| `count` | `1` (read once), an integer, `*` (rows until the end of the list), or `@field`: the value of that field of an earlier section |
| `stride` | values per row: an integer or `@field`. A stride larger than the declared fields means a newer build appended fields: decode the declared ones, skip the rest |
| `prefixed`, `frame` | `1`: each repetition is `length, values...` of the layout named in `frame` (empty in `history_v1`: the RPC says which) |
| `unit` | the SI unit (`N`, `N.m`, `m/s2`, `rad/s`, `kg.m2`, `Pa`, `s`...), the native unit a legacy field keeps (`kN`, `t`, `kPa`, `deg`), or empty (a code, a count, a flag, a direction cosine) |

**Rules** (enforced by review, the layout check and the manifest): fields are only ever
appended (`layout_schema` + 1); removing, renaming or moving one is a new layout or
`layout_protocol` + 1; a layout never depends on a mode (a field the mode does not fill is NaN);
every new layout is SI.

The layouts of this build: `dynamics_v3`, `dynamics_status_v3`, `dynamics_tracked_v3`,
`dynamics_reflection_check_v1`, `history_v1`, `control_snapshot_v2`, `control_v2`,
`inactive_status_v1`, `dynamics_ext_v1`, `dynamics_ext_status_v1`, `aero_actuator_v1`,
`aero_actuator_status_v1`, `aero_wrench_batch_v1`, `attitude_v1`, `attitude_status_v1`,
`attitude_order_probe_v1`, `attitude_options_v1`, `attitude_capabilities_v1`,
`attitude_gimbal_pivots_v1`, `attitude_effectors_v1`, `attitude_effectors_config_v1`,
`surface_model_v1`, `attitude_surfaces_v1` (lot 3, 3.1), `surface_kappa_v1`, `attitude_surfaces_roll_v1`, `attitude_translation_frames_v1`, `attitude_translation_v1` (legacy, mode-dependent),
`attitude_translation_v2`, `translation_evidence`, `translation_free`, `attitude_translation_config_v1`,
`engine_sample`, `gimbal_sample`, `thrust_direction_sample`, `thrust_position_sample`
(Actuators); `contact_v1` (Telemetry, lot 3); `dropped_stages_v1` (FMRS); `manifest_v1`,
`clock_v1`, `vessels_v1`, `bits_v1`, `bit_sets_v1`, `layout_v1`, `layouts_v1`, `events_v2`
(Bridge, lot 3).

```python
def decoder(conn, name):
    rows = conn.bridge.layout_v1(name)
    sections, current = [], None
    for row in rows[1:]:
        cells = row.split("\t")
        if cells[0] == "#section":
            current = {"name": cells[1], "count": cells[2], "stride": cells[3],
                       "prefixed": cells[4] == "1", "frame": cells[5], "fields": []}
            sections.append(current)
        else:
            current["fields"].append(cells[2])
    return sections
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

### Event bus V2 (`poll_events_v2`, refactor lot 3)

The same log, built for a flight loop that reads it in **one call per tick**: every record is
stamped with the **master tick** of the game clock and the UT, names its vessels and part by
**persistentId**, and the bus adds the events the first log never had (audit C, R6).

| Member | Type | Notes |
|---|---|---|
| `poll_events_v2(since_seq, max_rows)` | `list[str]` | Every record with `seq > since_seq`, oldest first, at most `max_rows` (0 or less: all). Text table, layout `events_v2` (protocol 1, schema 1). Read-only. |

Row 0 is tab-separated `key=value` pairs, row 1 names the columns (appended only: map them by
name), then one row per record.

| Header key | Meaning |
|---|---|
| `protocol`, `schema` | 1 and 1. |
| `since` | The `since_seq` asked (a negative one reads as 0). |
| `first_seq`, `last_seq` | What the ring holds now (1024 records; `first_seq` = `last_seq` + 1 when empty). |
| `lost` | Records after `since` already overwritten: you polled too slowly, they are gone. |
| `truncated` | 1 when `max_rows` cut the answer: poll again from the last `seq`. |
| `tick`, `ut` | The master tick and the UT when the call ran. |
| `capacity`, `n` | 1024, and the number of rows after the column row. |

| Column | Meaning |
|---|---|
| `seq` | Sequence number: from 1, never repeated within a game session. A `since_seq` greater than `last_seq` means the game restarted: start again from 0. |
| `tick` | The master tick of the last physics step opened when the event fired (`clock_v1`; the `master_tick` of the frames). An event fired between two steps (in `Update` or `LateUpdate`, a scene load) carries the step before it; the contact transitions carry the step whose pump saw them. |
| `ut` | Universal time when the event was recorded (s; 0 before the Planetarium exists). |
| `kind` | The event (tables below). |
| `vessel_pid`, `vessel_id` | The vessel's persistentId (0 when none); its Guid where the first log gave one (the first log's kinds and the plugins' records), empty otherwise. |
| `other_pid`, `part_pid` | A second persistentId the kind names, and the part concerned (0 when none). |
| `value` | A number the kind documents, SI, `NaN` when none. |
| `detail` | Free text, the same as the first log's for its kinds. |

**Every record of `poll_events` is here too**, same `kind` and `detail`, with `vessel_pid`,
`part_pid` and a `value` the first log has no column for: `vessel.situation` (`value`: the new
situation code, `Vessel.Situations`, flags: 1 LANDED, 2 SPLASHED, 4 PRELAUNCH, 8 FLYING, 16 SUB_ORBITAL,
32 ORBITING, 64 ESCAPING, 128 DOCKED), `stage.activate` (the stage number, `vessel_pid` the active
vessel), `part.die`, `part.crash` and `part.splashdown` (`part_pid`), `scene.load` (the
`GameScenes` code), and `fmrs.*`, `ocisly.*` and `mark()` records (`vessel_pid` 0: they name their
vessel by Guid). The first log itself is unchanged, byte for byte, and its ids are its own.

The kinds only V2 has:

| Kind | `vessel_pid` | `other_pid` | `part_pid` | `value` | From |
|---|---|---|---|---|---|
| `vessel.decouple` | the original vessel | the new vessel | — | — | `onPartDeCoupleNewVesselComplete` (end of `Part.decouple`): the record that binds a separated booster's persistentId to its tick |
| `vessel.undock` | the old vessel | the new vessel | — | — | `onVesselsUndocking` |
| `part.joint_break` | the child part's vessel | the parent PART | the child part | break force, N | `onPartJointBreak` (`PartJoint.OnJointBreak`; KSP's kN × 1000) |
| `vessel.part_count` | the vessel | — | — | its part count | `onVesselPartCountChanged` |
| `vessel.ease_start`, `vessel.ease_stop` | the vessel | — | — | — | `onPhysicsEaseStart`/`Stop`: unpacking, gravity ramped from 0, the active vessel's commands LOCKED in between |
| `vessel.off_rails`, `vessel.on_rails` | the vessel | — | — | — | `onVesselGoOffRails`/`OnRails` |
| `vessel.create`, `vessel.load`, `vessel.unload` | the vessel | — | — | — | `onVesselCreate`, `onVesselLoaded`, `onVesselUnloaded` (physics range) |
| `warp.rate` | the active vessel | — | — | `TimeWarp.CurrentRate` | `onTimeWarpRateChanged` |
| `contact.first` | the vessel | — | the first part in contact | `verticalSpeed` (m/s, up +) | `contact_v1`'s watcher: the vessel's first contact (an episode starts); `detail` `source=<bits> episode=<n>` |
| `contact.leg` | the vessel | — | the leg's part | `verticalSpeed` | a wheel's first ground in the episode; `detail` `ordinal=<k> leg` or `wheel` |
| `contact.all_legs` | the vessel | — | — | `verticalSpeed` | every deployed leg grounded |
| `contact.landed` | the vessel | — | — | `verticalSpeed` | `Vessel.Landed` rises |
| `contact.situation_landed` | the vessel | — | — | `verticalSpeed` | the situation becomes LANDED or SPLASHED; `detail` the situation |
| `contact.collision` | the vessel | — | — | `verticalSpeed` | a part starts colliding with something (another vessel: a tower's arms, debris) |
| `contact.lost` | the vessel | — | — | `verticalSpeed` | no more contact (a bounce, a liftoff) |

The V2-only kinds fire no `on_event` signal (the signals stay the first log's). The hooks are
lambdas (a static method group throws inside KSP's `EventData.Add`), each in its own
`try`, and are removed when the Core is destroyed.

```python
since = 0
def poll(conn):
    global since
    rows = conn.bridge.poll_events_v2(since, 0)
    head = dict(p.split("=", 1) for p in rows[0].split("\t"))
    if int(head["last_seq"]) < since:         # the game restarted
        since = 0
    columns = rows[1].split("\t")
    records = [dict(zip(columns, r.split("\t"))) for r in rows[2:]]
    if records:
        since = int(records[-1]["seq"])
    return head, records                        # head["lost"] > 0: some were overwritten
```

### HUD

| Member | Type | Notes |
|---|---|---|
| `hide_ui()` | `bool` | As if F2 had been pressed. `False` if already hidden. Flight only. |
| `show_ui()` | `bool` | `False` if already visible. Flight only. |
| `ui_visible` | `bool` | |

These fire `GameEvents.onHideUI` / `onShowUI` rather than blanking the stock canvases, so
mod windows disappear too — FMRS's, OCISLY's, the toolbar. For a recorded
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

Use part `flightID` to address `conn.control`, to disambiguate otherwise identical side
boosters, or to join OCISLY camera names. Use `persistentId` to join FMRS records to live
kRPC vessel objects.

---

## `conn.control`

What flies (refactor R3, `KRPC.Bridge.Control.dll`): stock KSP actuator authority used by the
GNC, the atomic protocol v2, and the AttitudeV1 loop. Commands use short leases: if the Python
client stops renewing, the plugin restores the engine's previous fields within at most one
second. Until R3 these members were on `conn.actuators`, with the same names and arguments;
the observing half moved to [`conn.telemetry`](#conntelemetry) (see
[Service names](#service-names-refactor-r3)).

| Member | Returns | Meaning |
|---|---|---|
| `available` | `bool` | Always `True` when the service loaded. |
| `ping()` | `str` | Returns `"pong"`. |
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
| `acquire_control_for_v1(owner, lease_seconds, persistent_id)` | `str` | Refactor R3 (state per vessel): `acquire_control` for the vessel the client names; refused with `[NOT_LOADED]` unless `persistent_id` (decimal string) is the loaded active vessel, the only one the v2 protocol drives. |
| `attitude_protocol_version` | `int` | `1`, the protocol of the AttitudeV1 frames and status. |
| `attitude_arm_v1(persistent_id, owner, lease_seconds=1, mode=0, axes_mask=3)` | `str` | Arms the AttitudeV1 loop for one loaded vessel (active or not), [below](#attitudev1-physics-rate-attitude-loop-shadow-and-armed-roll): `mode=0` shadow, `mode=1` armed on the channels of `axes_mask` (1 roll, 2 pitch/yaw ; pitch/yaw refused if another actuator owner is present). Returns a token. |
| `attitude_reference_v1(token, sequence, frame, dir_x, dir_y, dir_z, roll_mode, omega_ff_x, omega_ff_y, omega_ff_z, ut_stamp, ut_valid_until, omega_max)` | `int` | One reference per GNC tick (the AutoPilot's own). `1` accepted, `0` ignored. Renews the lease. Never allows a write. |
| `attitude_reference_v2(token, sequence, frame, dir_x, dir_y, dir_z, roll_mode, omega_ff_x, omega_ff_y, omega_ff_z, ut_stamp, ut_valid_until, omega_max_x, omega_max_y, omega_max_z, write_mask)` | `int` | V1 plus the AutoPilot's limit per body axis (per-axis ω guard) and the write permission of this reference (`1` roll, `2` pitch/yaw; with option 16 also `4` surfaces out of the law and `8` pitch/yaw holds the reference, with option 64 also `16` pitch/yaw only while the gimbals lead). Returns 0 ignored, else 1 + 2 (pitch/yaw written at the last physics step) + 4 (roll written). |
| `attitude_arm_v2(persistent_id, owner, lease_seconds=1, mode=0, axes_mask=3, inactive_options=0)` | `str` | born closed: `attitude_arm_v1` plus the non-active vessel's own authority (option 1 pitch/yaw, 3 + lateral force), [below](#non-active-vessel-d18). |
| `attitude_inactive_status_v1()` | `list[float]` | the per-vessel authorities held on non-active vessels. Read-only. |
| `attitude_gimbal_pivots_v1(token, mode)` | `list[float]` | born closed (0 at every arming): 1 maps a gimbal module whose pivots are not parallel (AR-1 Cluster) per pivot, as stock does; 2 also gives the law its exact authority, [below](#gimbal-modules-with-several-pivots-d42). |
| `attitude_effectors_v1(token, mode)` / `attitude_effectors_read_v1(persistent_id)` | `list[float]` | Refactor R1, born closed (0 at every arming): the effector registry, a model status per allocation column and a computed authority per family, [below](#effector-registry-refactor-r1-born-closed). |
| `attitude_surfaces_v1(token, mode)` / `surface_model_v1(persistent_id)` / `surface_kappa_v1(persistent_id)` | `list[float]` | Lot 3 (3.1), born closed (0 at every arming): SurfaceModelV1, the exact replica of the stock control surfaces, in shadow (1) or as an actuator of the law (2); (mode flag 8): its physical slope kappa, [below](#surfacemodelv1-control-surfaces-as-a-modeled-actuator-lot-3-31-born-closed). |
| `attitude_surfaces_roll_v1(token, ut_until)` | `list[float]` | born closed (no window before the first call, every arming closes it): the client's window for the roll excitation that proves κ on roll, and its state, [below](#roll-excitation-of-κ-d176-a-born-closed). |
| `attitude_translation_v2(token, sequence, frame, ax, ay, az, ut_stamp, ut_valid_until, mode)` | `list[float]` | Refactor R1: `attitude_translation_v1` with a return in one layout whatever the mode (46 values, NaN where the mode does not fill), [below](#lateral-force-differential-thrust-and-common-gimbal-translation). |
| `attitude_renew_v1(token, lease_seconds=1)` / `attitude_release_v1(token)` | `bool` | Renews (0.2–10 s) or drops an armed vessel. |
| `attitude_status_v1()` | `list[float]` | Hook state and one row per armed vessel. Streamable. |
| `attitude_frames_v1(persistent_id, since_tick=0, max_frames=64)` | `list[float]` | AttitudeV1 frames of one armed vessel, v3 history layout. |
| `attitude_order_probe_v1(frames=50)` / `attitude_order_probe_read_v1()` | `bool` / `list[float]` | T-sol-0 order probe: which stage runs first in a physics step. Read-only. |
| `gimbal_authority_v1(persistent_id)` | `list[float]` | the gimbals' columns and authority per kN of thrust, without thrust, pivots included, [below](#gimbal-authority-per-kn-d237-k4). `""`: the active vessel. Read-only. |
| `attitude_reference_v3(token, sequence, frame, dir_x, dir_y, dir_z, roll_mode, up_x, up_y, up_z, omega_ff_x, omega_ff_y, omega_ff_z, ut_stamp, ut_valid_until, omega_max_x, omega_max_y, omega_max_z, write_mask)` | `int` | born closed: `attitude_reference_v2` plus a roll mode and an up vector; `roll_mode=1` (refused without option 1024) points the roll, the roof toward the up vector, [below](#roll-pointed-d252-k6); `roll_mode=2` is its shadow (the roll error published, the law mode 0's). `roll_mode=0` is exactly V2. |
| `attitude_reference_ring_v1(token, sequence, frame, samples)` | `list[float]` | born closed (refused without option 2048): a REFERENCE RING of 2 to 64 dated samples (the nose, a lateral acceleration, the main throttle), interpolated at every physics step inside its span and served instead of the standing reference's direction, the request's acceleration and a ramp order's throttle, [below](#reference-ring-d252-k7). |
| `attitude_throttle_v1(token, sequence, kind, frame, parameters, ut_stamp, ut_valid_until)` | `list[float]` | born closed (refused without option 512): an order of the MAIN THROTTLE decided by the loop at every physics step and written at its fly-by-wire (ramp, pulse, cut when s = 0, ignition when h <= h_ign(v), the flip's calage), [below](#main-throttle-at-the-physics-step-d252-k5). A sequence not newer reads the state without changing the order. |
| `attitude_allocation_v1(token)` / `attitude_allocation_frames_v1(persistent_id, since_tick=0, max_frames=64)` | `list[float]` | (step 1), born closed (option 4096): the ENGINE-BY-ENGINE allocation in shadow -- the throttle of each engine, the gimbals and the RCS in one problem with the force and torque rows, strict priorities, the commands it would write (it writes none), the residuals and the lateral authority with the attitude kept and in function of the thrust, [below](#engine-by-engine-allocation-in-shadow-d264-step-1); with option 8192 (step 2) it WRITES, [below](#engine-by-engine-allocation-writing-d264-step-2). |

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

`control_status_v2()` returns 12 doubles (layout `control_v2`, schema 2): `protocol, tick,
topology_generation, owner_active, owner_ttl_seconds, pending_sequence, pending_tick,
last_accepted_sequence, last_applied_sequence, result, last_applied_tick,
vessel_persistent_id`. The last one (refactor R3, appended) is the persistentId of the vessel
the v2 authority drives, NaN without an owner; a client reads the first eleven by position or
every field by name (`layout_v1("control_v2")`), never the length.

Result codes are `0` none, `1` queued, `2` applied, `-1` frame expired, `-2` authority
lost/expired, `-3` topology or active vessel changed, `-4` unexpected application failure,
and `-5` a queued frame was superseded. A topology change invalidates the token rather
than applying commands addressed to stale part/module ordinals. While v2 authority is
active, legacy per-actuator lease procedures reject commands so two control paths cannot
silently fight.

The part id is returned as a double in the flat samples because all KSP `uint` flight ids
are exactly representable by a double. Pass it back as a decimal string to command or
release a lease.

### AttitudeV1 (physics-rate attitude loop, shadow and armed roll)

`AttitudeV1` is the attitude loop. In mode `0` (step M2, **shadow**) it
computes and publishes and writes **no actuator**. Mode `1` (step M3, **armed**) is accepted
for the **roll only** (`axes_mask=1`): the loop writes `ctrlState.roll` after the kRPC
AutoPilot, which stays engaged and keeps pitch and yaw ([below](#armed-roll-m3)). Pitch/yaw
armed (`axes_mask` bit 2 with `mode=1`) is refused by this build.

**Placement and cadence.** A `TimingManager` callback at stage `Earlyish`, added when the
first vessel is armed and removed with the last one (or at scene exit). A witness counts its
calls; if the watcher `FixedUpdate` sees no `Earlyish` call for three physics steps, it runs
the attitude tick itself and every such frame says so (stage `2`, motif bit 11). Where
`Earlyish` falls relative to `ModuleEngines`, `ModuleGimbal`, `FlightInputHandler` and
`Planetarium` inside one step is measured by the order probe below, not assumed.

**Per armed vessel** (by `persistentId`, active or not, at most two), each physics step:
- ω = `Vessel.angularVelocity` (body), I = diag(`Vessel.MOI`) × 1000, ω̇ = Δω / Δt on
  consecutive ticks, else the filters restart;
- realized torque M = engines (Σ r × F per nozzle, `finalThrust`) + RCS
  (`thrustForces`) + reaction wheels (−`inputVector`);
- columns B: two per gimbal module (N·m/deg, the DynamicsExtV1 derivation at the realized
  deflection) and the ctrlState channel, three commands (pitch, roll, yaw) = RCS (the stock
  `ModuleRCS.FixedUpdate` thrust law evaluated per unit command, positive and negative
  separately) + reaction wheels (−torque × authority) + control surfaces
  (`GetPotentialTorque` magnitudes, stock sign; approximate, flagged);
- the law of `AttitudeMath.cs`: same 2nd-order low-pass H(z) (18 rad/s, ζ 0.7) on ω̇ and
  M; square-root reference toward the target direction, saturated at `omega_max`, plus
  the feed-forward rate, minus the pseudo-control hedge; ν = K_ω (ω_r − ω) bounded by the
  current authority α; torque-form INDI M_d = M_f + I (ν − ω̇_f); bounded weighted least
  squares (active set, warm start) of the increment that brings M to M_d; the unrealizable
  part feeds the hedge. Roll is rate damping only. The client's Python mirror
  replays golden vectors from `build/tests` exactly.

**Reference.** `attitude_reference_v1` carries what the client gives the kRPC AutoPilot:
a kRPC `ReferenceFrame` and a direction in it (converted to world with
`ReferenceFrame.DirectionToWorldSpace` at every tick, as the AutoPilot does), roll mode
`0` (rate damping, `target_roll` NaN), a feed-forward rate in the same frame, the UT of the
guidance state it came from, a validity UT, and the AutoPilot's `max_angular_velocity`
(0 = its default, 1 rad/s). `sequence` must increase. A reference past `ut_valid_until` is
not used (motif bit 2). An accepted reference renews the real-time lease; an expired lease
disarms the vessel (the client re-arms). `attitude_reference_v2` adds the AutoPilot's
`max_angular_velocity` per body axis (`omega_max_x` pitch, `omega_max_y` roll,
`omega_max_z` yaw; 0 = its default) for the per-axis ω guard, and `write_mask`, the write
permission of that reference (`1` = the roll may be written while it is valid, `0` =
shadow). The law's own rate limit stays `max(omega_max_x, omega_max_z)`, as the V1 client
sent, so the computed law is the one of M2. A V1 reference never allows a write.

**Frame** (`attitude_frames_v1`, v3 history layout, 1000-frame ring per vessel): a
24-double header, a 110-double state block (schema 3; 100 in schema 2, 79 in schema 1), then one row of 16
doubles per actuator column.

| Header offset | Meaning |
|---:|---|
| 0–1 | protocol (`1`), schema (`2`; `1` before M3) |
| 2 | attitude tick (+1 per attitude step; the ring key) |
| 3–4 | UT, fixed delta time |
| 5–6 | vessel persistent id, topology generation (of this vessel) |
| 7 | stage: `1` Earlyish, `2` watcher fallback |
| 8–9 | Actuators pump physics tick seen, `Time.fixedTime` (to join with v3) |
| 10–11 | mode (`0` shadow, `1` armed), axes mask |
| 12 | motif bits (below) |
| 13–15 | consumed reference sequence, its `ut_stamp`, transport delay UT − `ut_stamp` (s) |
| 16 | cost of the tick, microseconds (measurement + law + allocation) |
| 17–18 | allocation iterations, converged |
| 19 | actuator column count n |
| 20–22 | state stride (`100`), row count (n), row stride (`16`) |
| 23 | capability bits: 0 engine torque, 1 RCS torque, 2 wheel torque, 3 gimbal columns, 4 RCS columns, 5 wheel columns, 6 surface columns, 7 reference, 8 law ran, 9 a gimbal module mapped per pivot, 10 a mapped module's exact authority used (mode 2), 11 / 12 / 13 option 1 / 4 / 2 of `attitude_options_v1` active ([options](#options-of-an-arming-refactor-lot-1-d124-d125-motif-15)), 16 the RCS torque in flight taken off the allocation target this step (option 32), 17 a gimbal column without effect regularized toward neutral this step (option 128), 18 gimbal columns at the thrust of the spool's horizon this step (option 128), 19 the free authority of the lateral force computed this step (option 256), 20 the main throttle decided this step, written at its fly-by-wire (option 512), 21 the roll error of the reference's roll target in the frame's error y this step (option 1024, in the law under roll mode 1, published only under roll mode 2), 22 the reference ring served a channel this step (option 2048), 23 the engine-by-engine allocation solved in shadow this step (option 4096), 24 the engine-by-engine allocation wrote this step (option 8192), 25 the RCS nozzles modelled this step by ModuleRCS's combined rule (option 16384), 26 the law's roll held this step, written under the RCS dead zone (option 16384), 27 the law's pitch/yaw held this step (option 16384) |

State block (body frame: x right = pitch, y nose = roll, z bottom = yaw):

| Offsets | Field |
|---:|---|
| 0–2 | pointing error rotation vector e (rad) |
| 3–5, 6–8 | ω (rad/s), measured ω̇ (rad/s²) |
| 9–11 | filtered ω̇_f |
| 12–14 | authority α per axis (rad/s²) |
| 15–17, 18–20, 21–23 | square-root rate, reference rate ω_r, hedge rate ω_pch |
| 24–26 | pseudo-control ν (rad/s²) |
| 27–29 | inertia diagonal (kg·m²) |
| 30–32, 33–35 | realized torque M, filtered M_f (N·m) |
| 36–38, 39–41 | ΔM_d = I (ν − ω̇_f), wanted torque M_d = M_f + ΔM_d |
| 42–44, 45–47 | allocation target M_d − M, allocated B·Δu |
| 48–50 | unrealizable part ν_h (rad/s²) |
| 51–53, 54–56 | INDI residual Δω̇_f − I⁻¹ΔM_f, and its first-order filter (0.2 s) |
| 57–59 | target direction in the body frame |
| 60–62 | served ctrlState (pitch, roll, yaw) read from the vessel |
| 63–65 | feed-forward rate, body frame |
| 66 | ω_max used |
| 67–68 | saturated fraction, active bounds |
| 69 | guards (M2, vessel-wide): 1 residual > 0.5 α for 1 s, 2 \|ω\| > 1.5 ω_max, 4 > 50 % saturated for 1 s |
| 70–72, 73–75, 76–78 | RCS, wheel, engine torque (N·m) |
| 79–81 | schema 2: unrealized share of the authority \|ν_h,a\| / α_a per axis (law) |
| 82–84 | ω_max per axis used by the per-axis ω guard (rad/s) |
| 85 | per-axis guards: bit 3a residual, 3a+1 ω, 3a+2 saturation, body axis a (0 x, 1 y, 2 z) |
| 86 | gimbal-roll coupling ρ = Σ \|B_y,gimbal\|·span / \|B_y\| of the ctrlState roll column (+∞ written as-is when that column is null) |
| 87 | write state: `0` shadow, `1` armed and writing the roll this step, `2` armed and fallen back |
| 88 | roll command of this step, `clamp(u0 + (M_d − M)_y / B_yy, −1, 1)` (NaN if not computable), written only when 87 = 1 |
| 89 | fly-by-wire record: attitude tick of the step it belongs to (the PREVIOUS one; NaN if none) |
| 90–92 | ctrlState pitch, roll, yaw just before our callback in that step (AutoPilot + pilot + SAS) |
| 93 | roll written by our callback in that step (NaN if not written) |
| 94–96 | fly-by-wire chain: our index, kRPC PilotAddon index (−1 absent), length |
| 97–98 | chain reorders, roll writes (cumulative) |
| 99 | write permission of the current reference (`write_mask`) |
| 100 | schema 3 (M4): pitch/yaw write state, `0` shadow, `1` writing, `2` armed and fallen back |
| 101–102 | fly-by-wire: ctrlState pitch and yaw written by our callback in the previous step (NaN if not) |
| 103–104 | gimbals leased by the loop this step, actuator authority held (0/1) |
| 105 | contact (Landed or Splashed outside PRELAUNCH, or an impossible demand) |
| 106 | reference age, UT − `ut_stamp` (s) |
| 107–109 | ω guard limit per axis actually used (1.5 × the relative limit) |

Quantities of the law are NaN when it did not run; the filtered ones (9–14, 33–35, 51–56,
82–85) exist whenever the tick is consecutive, even without a reference.

Actuator rows: `kind (1 gimbal, 3 ctrlState, 4 differential-thrust motif), flight_id,
ordinal, axis, u0, du, u_min, u_max, active_set (0 free, -1 lower, 1 upper, 2 fixed),
b_pos_xyz, b_neg_xyz, priority`. Kind-4 rows exist only while the lateral-force layer puts
its two motif columns in the allocation ([below](#lateral-force-differential-thrust-and-common-gimbal-translation));
they come after the three ctrlState rows, so the frame keeps schema 3 and its strides.
Gimbal rows: axis 0 = δx, 1 = δy, u in degrees, `b_pos` = `b_neg` = the column (N·m/deg).
ctrlState rows: axis 0 pitch, 1 roll, 2 yaw, u in [−1, 1], `b_pos` the torque per unit of a
positive command, `b_neg` per unit of a negative one; the least squares uses their mean.
`u0` is the realized deflection or the served ctrlState; `u0 + du` is what the loop WOULD
command.

Motif bits: 0 lease expired, 1 no reference, 2 reference stale, 3 physics warp (Δt ≠ 0.02 or
rate ≠ 1), 4 tick gap (filters restarted), 5 unloaded or packed, 6 topology changed,
7 residual guard, 8 ω guard, 9 saturation guard, 10 exception, 11 Earlyish dead (watcher
fallback), 12 frame conversion failed, 13 RCS model approximate (precision mode or
`fullThrust`), 14 control-surface columns used (approximate), 15 allocation not converged.
Bits 7–9 (M2, vessel-wide) are kept for comparison and no longer decide anything. Schema 2
(M3): 16 per-axis residual guard, 17 per-axis ω guard, 18 per-axis saturation guard — on
the axes of the arm mask, in shadow too (what WOULD make an armed loop fall back); in mode
1 only: 19 roll channel unmodeled (ρ > 0.25 or no roll column), 20 fly-by-wire order not
verified, 21 our callback moved back to the end of the chain (informational), 22 the
reference does not allow the write, 23 fallback held after a guard (2 s of game time),
24 armed but not written this step (summary), 25 the previous step's decision never reached
the fly-by-wire chain. Schema 3 (M4), pitch/yaw: 26 reference older than 0.3 s, 27 contact
(also stops the roll), 28 actuator authority unavailable (another owner, not the active
vessel), 29 pitch/yaw armed but not written this step, 30 the reference does not allow
pitch/yaw, 31 pitch/yaw fallback held after a guard of its axes. Bits 32-36 exist only with
option 1 of `attitude_options_v1` ([options](#options-of-an-arming-refactor-lot-1-d124-d125-motif-15)),
which also makes 13, 14 and 20 informational (option 4: 15).

`attitude_status_v1()` returns `protocol, schema, count, stride (25), earlyish_registered,
earlyish_calls, fallback_ticks, attitude_tick, ut`, then per vessel `persistent_id, mode,
axes_mask, lease_remaining_s, loaded, last_motif, sequence, consumed_sequence,
reference_age_s, latest_tick, history_count, computed_ticks, exceptions, cost_last_us,
cost_max_us, accepted_references`, and (schema 2) `write_mask, order_verified, writes,
fly_by_wire_calls, chain_reorders, fallback_steps`, and (schema 3) `pitch_yaw_writes,
gimbal_authority, pitch_yaw_fallback_steps`.

#### Armed roll (M3)

**Where the write happens.** One physics step runs, in this order (T-sol-0 probe, 49/50
steps on the pad): `Precalc > Earlyish > Normal > fly-by-wire > watcher >
FlightIntegrator > Late > BetterLateThanNever`. The law runs at `Earlyish`; the
AutoPilot writes `ctrlState` later, inside `FlightInputHandler.FixedUpdate` →
`Vessel.FeedInputFeed` → `Vessel.OnFlyByWire`, where kRPC's `PilotAddon.Fly` registered
its callback once per vessel (it ADDS the AutoPilot's output to the pilot state that
`FlightInputHandler` reset at the start of the step). A write at `Earlyish` would be
overwritten, so in mode 1 the loop appends its own callback to `Vessel.OnFlyByWire` and
keeps it LAST: every tick it compares the chain object with the one it verified (a
reference comparison; `Combine`/`Remove` always make a new object) and, when it changed,
re-reads the invocation list, moves itself back to the end if needed (motif 21) and
records its index and kRPC's (state 94–96; kRPC's entry is recognised as a delegate
whose method belongs to `KRPC.SpaceCenter.PilotAddon`). The write is allowed only if
kRPC's callback is present and runs before ours (else motif 20). Our callback records the
`ctrlState` it receives (the AutoPilot's output, state 90–92 of the next frame), then writes
`ctrlState.roll` with the value decided at this step's `Earlyish`, once; nothing it does can
throw into the chain.

**What is written.** `u = clamp(u0 + (M_d − M)_y / B_yy, −1, 1)`: the roll allocation
target on the ctrlState roll channel alone, the only actuator written. `u0` is the served
roll read at `Earlyish` (the loop's own previous write while it writes). `B_yy` is the
pure-roll RCS column; flight B measured the realized RCS roll torque at 2–7 % of it,
because stock `ModuleRCS` projects the lever on the plane of the TOTAL rotation input
(pitch and yaw commands shrink the roll lever) and ignores |roll| < 0.05. The column is an
upper bound of the effectiveness: INDI converges monotonically, slowly, without overshoot.

**When it is not written.** Any bit of the blocking set, bits 0–6, 10–20, 22 and 23
(not 7–9, 21, 24, 25). In particular: a stale reference (the safety of
M3: the client stops sending, the AutoPilot is back within the reference's validity), no
write permission in the reference, a per-axis guard on the roll (and 2 s after it clears),
the watcher-fallback stage, a changed topology, physics warp, and **thrust**: stock
`ModuleGimbal` mixes `ctrlState.roll` into the engine deflections, an effectiveness the
ctrlState column does not contain; flight B: ρ = 0 in flip and coast, median 1.2–1.5 and
up to 7 in boostback and descent, so the armed roll writes in the unpowered phases only.
When the loop does not write, nothing is restored: the AutoPilot's own roll, computed
every step, passes through untouched.

**Per-axis guards** (`AttitudeMath`, gains `guard_*`): residual |r_f,a| > 0.5 α_a for 1 s;
ω |ω_a| > 1.5 ω_max,a for 0.5 s, with the AutoPilot's limit on that axis; saturation
|ν_h,a| / α_a > 0.9 for 1 s, in the torque space (the M2 guard counted columns at a bound:
12 of 15 are gimbal columns). Replayed on flight B, they leave no trigger where the
AutoPilot held its attitude (the M2 guards fired 303 frames in boostback and 118 in
descent) and none on the roll; the remaining ones are the flip's RCS-versus-aero stall
(pitch) and the separation tumble (yaw).

**Lease and validity.** The lease (real time, 0.2–10 s) only keeps the vessel armed; the
reference's validity (game time) decides the writes. The M3 client arms with a 10 s lease,
above the measured holes of the Python loop (6.6 s at the flip entry, 1.3 s at the
boostback-to-coast transition), so the loop is not dropped and re-armed there, and sends
references valid 1 s of game time (the transport delay reached p99 0.46 s and ran past
0.5 s in 34 short bursts of the flight B coast): the AutoPilot takes the roll back 1 s
after the client stops.

#### Armed pitch/yaw (M4)

**What is written.** The gimbals are LEASED at the allocation's deflections (`u0 + du` of
the gimbal columns, degrees), under an actuator authority the loop takes from inside the
plugin (the v2 owner `AttitudeV1:<persistentId>`, on the active vessel only). A leased
gimbal leaves the stock calculation, so `ctrlState` no longer moves it: the gimbal
columns of the model are exactly what is commanded (validated in shadow on three
flights: engine torque = columns × realized deflection, r = 1.000, slope 0.996–1.008 in
pitch and yaw). `ctrlState.pitch/yaw`, now RCS only (columns validated, slope
0.95–1.03), are written by the fly-by-wire callback after the AutoPilot, like the roll;
when the loop owns the gimbals the roll command is the allocation's too. The lease ramp
is the stock one (QW5): the realized deflection follows at `gimbalResponseSpeed`.

**When it is not written.** Each channel falls back on its own. Pitch/yaw needs a
direction younger than 0.3 s (motif 26; the roll keeps the 1 s of M3), the permission
bit 2, no guard on the x and z axes (and 2 s after), no contact (27: `Landed` or
`Splashed` outside `PRELAUNCH`, or an unrealizable share above 5 × α on an armed axis,
which the barge touchdown drives to 1e4–1e5), and the authority (28). On fallback the
gimbals are handed back WITHOUT a jump: `actuationLocal` and the public
`ModuleGimbal.oldActuationLocal` are set to the deflection the lease reached, so the
stock Lerp resumes from there toward the AutoPilot's command (a legacy lease restores
the rotation of BEFORE the lease, in one step). While the loop holds the authority,
`acquire_control` and the legacy lease procedures are refused; arming pitch/yaw is
refused when another owner (v2 control, legacy lease, the other armed vessel) is present.

**Guard on ω (all axes).** The limit is relative: 1.5 × the largest AutoPilot limit on
that axis over the last second, or the law's previous |ω_r| if larger. The AutoPilot cap
moves down near the target while the vessel still turns (flight B, 500–100 m: |ω_x| /
ω_max up to 2.8 with the AutoPilot at 1.1°).

**Write state for the guidance.** `attitude_reference_v2` returns the write state of the
last physics step (bit 2 pitch/yaw, bit 4 roll): the descent law D switches its attitude
time constant on it (step D3) without another RPC.

**Order probe (T-sol-0).** `attitude_order_probe_v1(frames)` stamps, for the next `frames`
physics steps (1–300), every call of the `Precalc`, `Earlyish`, `Normal`,
`FlightIntegrator`, `Late` and `BetterLateThanNever` stages, of the Actuators watcher and of
the active vessel's `OnFlyByWire` chain (read-only). `attitude_order_probe_read_v1()`
returns `version (1), armed, row_count, stride (13)` then `frame, call_index, stage (1
precalc, 2 earlyish, 3 normal, 4 integrator, 5 late, 6 better-late, 7 watcher, 8
fly-by-wire), fixed_time, ut, pump_tick, render_frame, served_pitch, first_gimbal_x,
first_engine_thrust, omega_x, first_rcs_thrust_sum, real_time`. A value that changes
between two stages of the same step names the module that ran in between.

#### Lateral force: differential thrust and common-gimbal translation

Chantier 19 (suite). At hover (main throttle ~0.1) the attitude has only the gimbals, and
every gimbal torque also pushes the vessel sideways the wrong way (the unstable zero of a
translation through the body): the precise final phase hunts. This layer uses the engines
**separately from the body**: a COMMON gimbal deflection (the same body tilt on every
nozzle) makes a lateral force T sin δ in ~0.12 s, and its torque is cancelled in advance
by **differential thrust** at constant total thrust. It is a pre-allocation in front of
the unchanged law (`AttitudeMath.cs`, its golden vectors and its Python mirror do not
move): two extra columns and shifted bounds for the weighted least squares.

**Born closed.** Nothing runs until `attitude_translation_v1` is called for an armed
vessel. A vessel that is not armed (mode 1) on pitch/yaw, a pitch/yaw channel without
permission (`write_mask` bit 2), a direction older than 0.3 s, physics warp, the watcher
fallback stage, a vessel that is not the active one, or a request past its validity (once
wound down) leave the tick exactly as `c03d5de`: same columns (2 per gimbal + 3), same
bounds, same writes, no ring frame.

```
attitude_translation_v1(token, sequence, frame, ax, ay, az, ut_stamp, ut_valid_until, mode)
    -> list of 17 doubles (25 while the stored mode has bit 4: modes 5 and 7; 32 with bit 16 or 32)
attitude_translation_v2(token, sequence, frame, ax, ay, az, ut_stamp, ut_valid_until, mode)
    -> list of 46 doubles, whatever the mode and the options (refactor R1)
```

`attitude_translation_v2` takes the same request and has the same effect; only its return
differs: ONE layout (`attitude_translation_v2`, protocol 1, schema 1, names from
`conn.bridge.layout_v1("attitude_translation_v2")`): the 32 values of layout 3 (a value the
stored mode does not fill is `NaN`; offset 4 still says which part it fills, 1, 2 or 3), then
the evidence block `protocol (1), schema (1), n (11)` and its 11 values (`NaN` without option 2
of `attitude_options_v1`). The return of `attitude_translation_v1`, whose length depends on the
mode, is unchanged.

- `(ax, ay, az)`: the TOTAL horizontal acceleration wanted from the engines (nose × thrust
  plus translation), m/s², in `frame` (the frame of the V2 reference; converted with
  `ReferenceFrame.DirectionToWorldSpace` of its unit vector times its norm, then to the
  body frame). Its vertical part is ignored.
- `sequence` must increase (an older or equal one is not stored: `accepted` = 0, the
  state is still returned). `ut_valid_until >= ut_stamp`, finite values, else an error.
- `mode`: `0` stop (wind down now), `1` differential thrust as two motif columns of the
  attitude allocation, `3` differential + translation by common gimbal deflection. `2`
  alone is refused with an error: the translation needs the differential to cancel its
  torque. Bit 4, "computed levers" (born closed, [below](#computed-levers-bit-4-modes-5-and-7)):
  `5` = `1` and `7` = `3` with a computed lever criterion, the RCS translation as a lever, a
  joint allocation and the torque-free authority; `4` and `6` are refused like `2`. A DLL
  older than this one refuses `5` and `7` (`ArgumentOutOfRangeException`): a client falls
  back on `1` / `3`. A request does not renew the arming lease.

Return (the values of the LAST physics step; the RPC runs between steps):

| Offset | Meaning |
|---:|---|
| 0 | accepted (1: stored, 0: sequence not newer) |
| 1 | lateral authority available now, m/s² (0 when unavailable: state not 1/2, or the pitch/yaw channel did not write): (ΣT/m) · sin(range) · min(frac_gimbal, frac_diff · diff authority / full common torque) |
| 2 | differential torque authority, N·m: the torque the motifs guarantee in ANY pitch/yaw direction with the symmetric throttle box (inradius; e.g. barge at hover 90 kN·m, about 180 on one axis alone). 0 when unavailable |
| 3 | realized common gimbal fraction: max over gimbals of \|realized common deflection\| / directional range (lease-ramp model, after the last step) |
| 4 | layout version of this return (`1`; `2` while the stored mode has bit 4, eight values appended below) |
| 5 | state: `0` idle (nothing added), `1` motif columns, `2` motifs + common deflection, `3` winding down, `4` requested but unavailable this step |
| 6 | reason bits (below) |
| 7 | wanted lateral acceleration \|gap\|, m/s², before the bounds |
| 8 | lateral acceleration commanded by the common deflection, m/s² (after the bounds) |
| 9 | lateral acceleration of the realized common deflection (model), m/s² |
| 10 | torque of the full common deflection (all gimbals at their smallest directional range), N·m (barge at hover ~38 kN·m) |
| 11 | pitch/yaw torque of the realized common deflection that the feed-forward cancels, N·m |
| 12 | engines in the motifs |
| 13 | main throttle read at `Earlyish` (the previous fly-by-wire's) |
| 14–15 | attitude tick and UT of these values |
| 16 | throttle writes (cumulative) |
| 17 | *(layout 2)* lateral authority of the differential + common tilt alone, m/s²: the formula of offset 1 in modes 1/3, without the 0.25 m lever threshold (0 when unavailable) |
| 18 | *(layout 2)* lateral authority with every lever, m/s² (= offset 1) |
| 19–20 | *(layout 2)* `ctrlState.X` and `ctrlState.Y` this step writes (the joint solution × the realized share of the common deflection; 0 when nothing is written) |
| 21 | *(layout 2)* RCS translation force capacity, N: the smallest along-axis force of the four commands X = ±1, Y = ±1 (full command; the translation uses at most `frac_rcs` of it) |
| 22 | *(layout 2)* motif lever, m: the smaller of the two motifs' largest levers (max \|z − z̄\|, max \|x − x̄\|), the value the lever criterion tests |
| 23 | *(layout 2)* pitch/yaw torque left after the feed-forward, N·m (realized tilt + RCS written + feed-forward amplitudes) |
| 24 | *(layout 2)* `frac_rcs` (0.5) |

Reason bits: 0 no request, 1 not armed on pitch/yaw, 2 pitch/yaw not permitted (write
mask, reference older than 0.3 s, warp, stage), 3 the pitch/yaw channel fell back this step
(nothing written; everything handed back), 4 stale request, 5 stop requested (mode 0),
6 no differential authority (fewer than two engines, or a motif lever below 0.25 m: the big
booster's centroids sit 0.13 m from the axis), 7 no throttle margin (main throttle below
0.005 or above 0.995), 8 no translation geometry (no thrusting gimbal, singular tilt matrix,
no mass, frame conversion failed), 9 nose more than 60° from the local vertical,
10 translation capped at `frac_gimbal`, 11 capped by what `frac_diff` of the throttle
margin can cancel, 12 feed-forward clamped (realized common torque above the throttle box),
13 the last throttle decision never reached the fly-by-wire chain (throttles given back at
once), 14 wind-down ended by time, 15 not the active vessel. Bit 4 only: 16 no torque-free
lateral authority (the joint allocation reaches a numerically null one: no translation this
step, the motifs stay in the attitude allocation), 17 translation capped at `frac_rcs` of
`ctrlState.X/Y`, 18 RCS model approximate (precision mode or `fullThrust` on a module with a
translation column). Bit 16 only: 19 the law's RCS pitch/yaw left out (combined rotation
nonlinear). With bit 4, 6 means a DEGENERATE geometry only (fewer than two engines,
a motif lever at or below the criterion, or singular motif columns), 10/11/17 are set only
when the force target is missed by more than 0.1 % and that bound holds.

**Differential thrust (modes 1 and 3).** Engines that may be driven: ignited, operational,
not flamed out, throttle not locked (solids), thrust limiter above 0, and not in an
independent throttle set by someone else. For each: the nozzle centroid (thrust-transform
multipliers as weights), its torque per newton r × dir (body), and the slope
s = dT/d(throttle command) from `ModuleEngines.UpdateThrottle` and
`RequiredPropellantMass`: thrust = K · `currentThrottle`, `currentThrottle` =
Lerp(`throttleMin`, 1, command · `thrustPercentage`%) at steady state, so
s = K (1 − `throttleMin`) `thrustPercentage`%, K = `finalThrust` / `currentThrottle`
(`maxThrust` when not thrusting). Two MOTIFS rather than one column per engine:
motif_pitch,i = (z_i − z̄)/max|z − z̄|, motif_yaw,i = (x_i − x̄)/max|x − x̄|, (x̄, z̄) the
slope-weighted mean, so Σ s_i motif_i = 0 (total thrust conserved). Engine i is commanded
`independentThrottlePercentage` = 100 · (main + d_p motif_p,i + d_y motif_y,i), which is the
stock `requestedThrottle` of the main throttle when d = 0 (the limiter and `throttleMin`
apply the same way). Column of motif k: Σ_i motif_k,i s_i (r_i × dir_i), N·m per unit
amplitude (barge: 1.8 MN·m). Box: every point of |d_p| ≤ h_p, |d_y| ≤ h_y keeps every
engine within [0, 1] around the main throttle (margin min(main, 1 − main)); the feed-forward
is reserved first, the same attitude share r is left to both motifs. Realized amplitudes:
slope-weighted least squares of the engines' realized commands (from `currentThrottle`), so
the law's u0 is realized, as for the gimbals. Priority 0.5 (gimbals 1, ctrlState 3: the
"1 against 2" of the bench without changing any other ratio); tunable.

**Translation (mode 3).** Each step: gap = horiz(demand) − (ΣT/m) · horiz(nose), horiz
= projection orthogonal to the local vertical (CoM − body centre); the body lateral
acceleration (x, z) whose horizontal projection is the gap (exact 2×2; refused beyond 60°
of tilt); the body tilt φ = K⁻¹ m a with K = Σ_g (force columns × rotation axes) of every
usable gimbal (the same derivation as the torque columns, at the realized deflection); φ
bounded at `frac_gimbal` (0.8) of every gimbal's directional range AND so that the
differential needs at most `frac_diff` (0.8) of every engine's throttle margin to cancel its
torque; converted into each gimbal's own x/y axes (δx = φ·axisX, δy = φ·axisY).

**Feed-forward and the law.** The realized common deflection is modelled with the lease's
own ramp (Lerp by `gimbalResponseSpeed` · Δt per step, QW5): the feed-forward amplitudes
cancel the pitch/yaw torque of the realized common deflection AFTER this step's ramp (in
advance of the gimbal lag, not of the engine lag). The law then allocates the ATTITUDE
part only: gimbal column u0 = realized − realized common of the previous step, bounds
shifted by the commanded common (common + attitude stays within the range); motif column
u0 = realized amplitude − previous feed-forward, bounds [−h − d_ff, h − d_ff]. Commands:
gimbal = common + u0 + du (leased at `Earlyish`, as M4); motif = d_ff + u0 + du, turned into
per-engine offsets. The throttles are written in the fly-by-wire callback, after the kRPC
AutoPilot served THIS step's main throttle: throttle_i = main + κ · offset_i, κ ∈ [0, 1]
scaling the offsets so every engine stays within [0, 1] around that main throttle (a cut
main throttle cuts them all; zero sum and direction kept). Because the shifted bounds are
asymmetric, the law's authority α (max of |bounds| × column) grows with the common offset
and the motifs: the law itself is unchanged.

**Validity, wind-down, leases.** The request is live while `ut <= ut_valid_until` (game
time) and mode ≠ 0. Stale or stopped: the common target goes to 0 at once, the realized
common decays at the gimbal ramp, the feed-forward keeps cancelling it, the motifs lose
their attitude share (bounds [0, 0]: the law compensates with the gimbals in the same
step); when the realized common is back under 0.01° after at least one such step, or after
1 s of game time, the throttle leases are given back and the layer is idle. The throttles
use the Actuators lease (`independentThrottle`/`independentThrottlePercentage`, original
values captured at the first write and restored): renewed at every write for 0.25 s of
real time, restored when the pitch/yaw channel falls back (the authority is handed back
with every lease), at disarming, lease expiry, topology change, scene change, a refused
step (state 4), a missed fly-by-wire write, or the end of the wind-down. Any fallback of
the pitch/yaw channel also resets the layer's offsets (the next step starts from the
realized deflections).

`attitude_translation_config_v1(token, frac_gimbal, frac_diff, diff_priority)` sets the
tunables of one armed vessel (reset at every arming): `frac_gimbal` and `frac_diff` in
[0, 0.95] (default 0.8), `diff_priority` in [0.1, 10] (default 0.5). Returns `frac_gimbal,
frac_diff, diff_priority, min_lever_m (0.25), min_throttle_margin (0.005),
min_projection_det (0.25 = cos² 60°), wind_down_done_deg (0.01), wind_down_max_s (1),
throttle_lease_s (0.25)`. Unchanged by bit 4: `frac_rcs` (0.5) is fixed, reset at every
arming and published at return offset 24; `min_lever_m` is the criterion of modes 1/3 only.

**Journal.** The AttitudeV1 frames keep schema 3 and their strides: while the motifs are
in the allocation they carry two more rows (kind 4, axis 0 pitch motif, 2 yaw motif; u in
throttle amplitude, `b_pos` = `b_neg` = the column), and the gimbal rows then carry the
ATTITUDE part (u0, bounds shifted by the common offset). The layer's own record is a
separate ring, `attitude_translation_frames_v1(persistent_id, since_tick, max_frames)` (v3
history layout, protocol `1`, schema `1`, one frame per tick once a request was received
since the arming): a 56-double header, then 6 doubles per gimbal module and 10 per motif
engine.

| Header offset | Meaning |
|---:|---|
| 0–4 | protocol (`1`), schema (`1`), attitude tick, UT, persistent id |
| 5–6 | state, reason bits |
| 7–10 | mode, sequence, request age (UT − `ut_stamp`), validity left (`ut_valid_until` − UT) |
| 11–13 | ΣT (N), mass (kg), main throttle |
| 14–16, 17–19 | demand and local vertical in the body frame |
| 20–21, 22–23 | wanted, commanded lateral acceleration (body x, z; m/s²) |
| 24–26 | body tilt φ (x, z; deg), bound scale applied |
| 27–29 | lateral authority (m/s²), differential authority, full common torque (N·m) |
| 30–31 | common fraction commanded, realized |
| 32–34 | torque of the realized common deflection (N·m, body) |
| 35–36, 37–38 | feed-forward, realized motif amplitudes (pitch, yaw) |
| 39–40 | box half widths |
| 41–42 | motif command written this step (NaN if the channel did not write) |
| 43 | attitude share r |
| 44–46 | fly-by-wire record of the PREVIOUS step: its attitude tick, the main throttle it saw, κ |
| 47 | throttle leases held |
| 48 | wind-down elapsed (s, NaN outside) |
| 49–50 | lateral acceleration of the realized common deflection (body x, z) |
| 51–52 | smallest directional gimbal range (deg), symmetric motif half width |
| 53–55 | gimbal module count, motif engine count, throttle writes |

Gimbal rows: `flight_id, ordinal, common_x, common_y` (commanded, deg) `,
realized_common_x, realized_common_y` (model after this step). Engine rows: `flight_id,
slope (N per unit throttle), motif_pitch, motif_yaw, centroid_x, centroid_z (body, m),
realized command, offset decided this step (NaN if not written), percentage written by the
previous fly-by-wire (NaN if none), leased (0/1)`.

**Limits.** The feed-forward anticipates the gimbal ramp, not the engine throttle response
(`useEngineResponseTime`): the INDI loop absorbs that lag, as it does for the gimbals. The
gap does not include the lateral force of the gimbals' ATTITUDE part (the trim of a
disturbance): the outer guidance loop sees it. The throttle box assumes every motif engine
shares the main throttle. Measured only on the pure-maths bench (build/tests): never flown.

#### Computed levers (bit 4, modes 5 and 7)

Precision of the heavy booster (flight, 28/09). Its 8 AR-1
Cluster parts have their nozzle centroids 0.13 m from the axis: the fixed 0.25 m lever
threshold refused the whole layer (state 4, bit 6, every step), and even accepted, the
differential alone cancels the torque of only ~0.035 m/s² of common tilt at throttle 0.08.
But its 8 RCS modules sit 37 m ABOVE the CoM while the nozzles are 10.8 m below: a lateral
force from the RCS and a common tilt in the same direction have opposite torques. **Born
closed**: modes 0, 1 and 3 are unchanged bit for bit (golden vectors unchanged); nothing
below runs without bit 4.

- **Lever criterion.** A motif is refused only for a degenerate geometry: its largest lever
  at or below `1e-5 × max |r|` (r: the engines' nozzle centroids from the CoM, x, y and z;
  float3 levers rotated in float are known to about 1.2e-7 |r|, so 1e-5 |r| is ~80 times
  their rounding), fewer than two engines, or singular motif columns (inradius 0). The heavy
  booster: 0.133 m against 1.1e-4 m, accepted. While winding down, the criterion of the mode
  that engaged the layer is kept (a stop after mode 7 winds down like mode 7).
- **RCS translation lever.** Per RCS module (the conditions of the rotation columns, plus
  `enableX` for `ctrlState.X` and `enableY` for `ctrlState.Y`): `ModuleRCS.Update` makes
  `inputLin = ReferenceTransform.rotation × (X, Z or throttle, Y)`, `FixedUpdate` gives nozzle i
  the fraction max(v·inputLin, 0) (plus the rotation's, the sum bounded at 1) and the force
  −v · power · fraction at its position (v = thruster up, forward with `useZaxis`). So
  `ctrlState.X` acts along body x, `ctrlState.Y` along body z; `ctrlState.X = +1` pushes along
  −v of the nozzles whose v·x > 0. Columns: force and torque (about the CoM, body) per unit of
  +1 and per unit of a negative command, their mean (as the rotation columns). Heavy booster:
  X = +1 gives −39.3 kN along x and +1.464 MN·m about z (37.25 m above the CoM).
- **Joint allocation** (`JointTranslation`, the law's own bounded least squares
  `AttitudeMath.SolveBoxLeastSquares`, unchanged, its work space preallocated). Variables:
  φx, φz (common body tilt, deg: force K φ, torque Kt φ, an earlier change included), rX, rY (`ctrlState.X`,
  `.Y`), dP, dY (motif amplitudes: torque only). Rows: Fx, Fz with target m·(ax, az) (the gap
  of mode 3), τx, τz with target 0, weighted 1e3 at the gimbals' lever L = σ(Kt)/σ(K) (the
  torque comes first: a conflict is shared in the ratio of the squared weights, 1e-6), τy
  (roll) weight 0 (no lever of the translation can cancel roll; a lexicographic or even
  force-weighted roll row bent the tilt step to step on the heavy booster model, the law's
  roll channel takes it as in mode 3), then one regularization row per variable, 1e-3 of its
  own effect at full span (force/m, torque/(m L)); the motifs' is multiplied by
  1 / `diff_priority` (2): the law favours the differential for the attitude, so the
  translation spares it and balances with the RCS first. Bounds: |φ| ≤ `frac_gimbal` × the
  smallest gimbal range, |r| ≤ `frac_rcs` (0.5: a nozzle is shared by rotation and
  translation, fraction sum bounded at 1; half leaves the attitude's own RCS command linear
  up to 0.5), |d| ≤ `frac_diff` × the symmetric motif half width. Then the whole solution is
  scaled so that the worst usable gimbal stays within `frac_gimbal` of its directional range
  (the tilt box is on body axes, a gimbal's on its own).
- **Motif columns (modes 5 and 7).** Each engine's torque per newton is taken along the
  engines' COMMON thrust direction (slope-weighted mean of their nozzle directions,
  `CommonDirectionTorques`), not along its own nozzles. A zero-sum motif keeps only its
  geometric lever from a shared direction; an engine's own tilt adds L × its gimbal deflection
  per newton (0.41 m per degree on the heavy stack, against its 0.133 m motif lever), i.e. the
  torque of that gimbal, which the law re-decides every step. Read in the motif columns it tied
  the differential to the gimbals' attitude part (third ground check, 28/09: throttles swinging
  0 / 0.16 every step, the law's modelled gimbal torque +2 303 kN·m against −1 255 realized,
  2 033 kN·m of total torque at −e, engines not reversed). Bench, the stack in the clamps with
  the measured open-loop drift: 2 097 kN·m before, the drift's 442 after (421 with the columns
  at rest); in flight (PF, 1 MN·m of steady torque held by the gimbals) throttle jumps 0.146 →
  0, attitude excursion 0.038° → 0.019°.
- **Feed-forward (mode 7).** The gimbals ramp, the RCS does not: the RCS written this step is
  the solution × κ, κ ∈ [−1, 1] the share of the commanded common deflection the gimbals
  realize after this step's ramp (the realized projected on the commanded; 1 when no tilt is
  commanded). Negative through a reversal: the RCS keeps balancing the old tilt the gimbals
  still hold (bounded at 0 before: reversal transient 254 kN·m on the bench, 11 signed). The differential then cancels the pitch/yaw torque of the realized tilt AND of
  the RCS written (`CancelTorque`, `BoxHalfWidths` as in mode 3); its part of the solution
  only sized the others. The law allocates the attitude part as in mode 3.
- **Write.** `ctrlState.X` and `.Y` are written by the fly-by-wire callback (after the kRPC
  AutoPilot) only in mode 7 with a live request, a vessel with an RCS translation column, and
  the pitch/yaw channel writing in that same callback; otherwise they are not touched (the
  pilot's or kRPC's). The step after our last write, X or Y still holding exactly what we
  wrote (a `ctrlState` nobody refreshed, e.g. a non-active vessel) are set to 0 once; also at
  unhooking. Fallback, guards, stale request: no write from that step on.
- **Pitch/yaw fallback** (second ground check, 28/09). A fallback hands the gimbals to the
  stock Lerp while they still hold the common tilt. Before, the realized common dropped to 0
  at once: the next writing step counted the tilt still held as the law's ATTITUDE part
  (u0 = realized − realized common) and kept it (in the launch clamps ω̇ stays 0: the INDI law
  only sees torque changes; bench, the stack of the check, one-step fallback every 12 steps at
  −e: tilt −1.61° for −0.33° modelled, 1 529 kN·m of total torque, κ 0.65). Once a bit-4 step
  has committed a common deflection, a release makes the realized common follow the Lerp, ×
  (1 − f) per physics step (f the module's ramp factor; `ReleasedCommonFollows`, once per
  step); the law writing with the layer off ends it at 0 (its u0 is then the whole deflection,
  as without bit 4). Same bench: 65 kN·m, κ 0.94.
- **Authority** (offset 1 in modes 5 and 7): the largest lateral acceleration WITHOUT torque
  the same allocation reaches, the smallest over the body directions +x, −x, +z, −z (each
  solved with a target 10 × the force upper bound Σ \|force column\| × span along the
  direction; the component reached along it, after the gimbal scale). Numerically null
  (below 1e-6 of that bound): 0, reason 16, no translation that step. Mode 5 publishes it
  and runs as mode 1 (no tilt, no RCS).
- **Ring.** Same layout (schema 1). In modes 5/7: offset 27 is the authority above, 22–23 and
  49–50 the commanded / realized lateral acceleration of the tilt AND the RCS, 32–34 the torque
  of the realized tilt AND the RCS written, 26 the gimbal-fraction scale of the joint solution.

Pure-maths bench (`build/tests`, heavy booster model: centroids of a recorded flight, nozzles
10.83 m below the CoM, 2.5 MN at throttle 0.08, 260 t, 2°, RCS ring as above): authority
0.0348 m/s² for the differential + tilt, 0.3355 for the RCS + tilt, **0.3441 for all three**
at throttle 0.08 (0.1304 / 0.3355 / 0.4660 at 0.3); barge and medium (no RCS translation):
the same as modes 1/3 (0.2739). Closed loop through the unchanged law: 0.2 m/s² asked,
0.193 delivered, attitude within 0.001°, RCS X/Y 0.28, differential throttles 0.059–0.101.
Never flown: a ground check first.

#### Honest translation (bits 16 and 32, modes 21/23, 37/39, 53/55)

Chantier « translation du gros sans effet sur le CdM » (a recorded flight and
`_172225`, tower capture; `_142810`, pad). Measured in flight, the body's lateral force
(engines + external residual + aero, `dynamics_v3`) along the announced realized translation:
**−0.12×** (183740), 0.43× (172225), 0.94× (142810); medium and super hope 0.56–1.05×. The
gimbals hold −0.17 / 0.62 / 1.04 of the common tilt (realized deflection projected on the
realized common: INDI's attitude part −1.17× the common on 183740). Cause, in the capture
only: the AutoPilot aligns the ROLL (`ctrlState.roll` ±1, mean |roll| 0.92 on 183740; the law
writes pitch/yaw only), and `ModuleRCS` combines (pitch, roll, yaw) into ONE rotation vector:
`rot = inputRot × unit(ProjectOnPlane(r, inputRot))`. On the heavy booster's ring (37 m above
the CoM) the slightest pitch/yaw turns the lever and the ring answers with its WHOLE torque:
roll served ≥ 0.9, pitch/yaw 0.05–0.3: 1.73–1.76 MN·m realized for 0.36 modelled (×4.8;
roll ~0: 0.215 for 0.207). The law's RCS becomes a bang-bang (RCS ±1.45 MN·m step to step,
saturation 0.83, pitch/yaw fallback 22 %), the X/Y translation share is clipped on the nozzles
the rotation fills, the common tilt's torque is left unbalanced and INDI cancels the tilt with
the gimbals. **Born closed**: modes 0–7 are unchanged bit for bit; bits 16/32 are refused
alone, with 1/3, or with 8/64.

- **Bit 16, shared RCS.** (1) The RCS translation columns count only the nozzles the
  translation drives LINEARLY over its span: nozzle i with ρᵢ + `frac_rcs`·fᵢ ≤ 1, ρᵢ its
  rotation fraction under the served (pitch, roll, yaw) by `ModuleRCS`'s combined rule, held
  0.5 s (linear decay), fᵢ = \|v·axis\|; each channel then keeps the weaker of its + and −
  columns. Exact for the nozzles counted, the others can only add. (2) The law's RCS pitch /
  yaw column is left out (bounds [0, 0]: it writes 0; reason 19) while `ModuleRCS` answers a
  0.1 command, at the served point, with more than 2× what the column predicts. (3) While the
  law does not write the roll and someone else commands it (\|ctrlState.roll\| held 0.5 s above
  0.05), the gimbal and motif columns carry no roll (B roll entries 0): the allocation no
  longer fights, with the gimbals, the roll the AutoPilot commands with the RCS; without a
  commanded roll the law keeps damping it with its gimbals. Calm (no roll commanded, small
  pitch/yaw), bit 16 = bit 4 (bench: bit for bit).
- **Bit 32, measured authority.** At each step following a step the law wrote with this layer,
  the realized gimbal deflection (u0 before the shift) is projected on the realized common of
  that step; forgetting means (1 s of steps with a common) of Σ u·c and Σ c² give the measured
  share. Offsets 1 and 18 publish the model's authority × the share bounded to [0, 1] once the
  mean of Σ c² reaches (0.05°)² per gimbal column (before: × 1). The layer itself runs on the
  model's authority (it keeps translating, so the share keeps being measured). Mono-vol: reset
  at every arming.
- **Return layout 3** (`3` at offset 4, while the stored mode has bit 16 or 32): layout 2 and

| Offset | Meaning |
|---:|---|
| 25 | the model's authority, m/s² (offset 1 before the bit-32 factor) |
| 26 | measured share of the common the gimbals hold (NaN before any evidence) |
| 27 | its evidence, deg² (forgetting mean of Σ c² over the gimbal columns) |
| 28 | RCS translation capacity without the headroom, N (offset 21 is with it under bit 16) |
| 29 | bit 16: weaker / stronger sign kept, the smaller of X and Y (1 when symmetric) |
| 30 | bit 16: the largest held rotation fraction of a nozzle |
| 31 | bit 16: 1 the law's RCS pitch left out, 2 yaw, 4 the gimbal/motif roll rows dropped |

Pure-maths bench (`build/tests`, heavy booster model at throttle 0.08, the RCS of the plant at
nozzle level with `ModuleRCS`'s combined rule): roll 1 + pitch 0.1 answers 1.470 MN·m for 0.147
(×10). Closed loop, 0.2 m/s² asked: calm, bit 4 and bit 16 deliver 0.193 (gimbals hold 0.95);
the AutoPilot's roll at +1, bit 4 delivers 0.059 for 0.203 announced (gimbals hold 0.30), bit
16 delivers 0.193 for 0.200 (0.94); roll ±1 every 0.3 s, bit 4 0.046 for 0.198 (0.18), bit 16
announces 0.064 and delivers 0.053 (0.55). Never flown.

#### Non-active vessel

In a double flight (two boosters, one `FlightGlobals.ActiveVessel`) the actuator authority
the pitch/yaw channel takes (the v2 owner, `TryHoldForAttitude`) exists on the ACTIVE vessel
only: on the other booster every step fell back with motif 28 (`gimbal_authority`) and the
lateral force said reason 15 (a recorded flight_b1`: 1052 steps with the
permission, 0 pitch/yaw written; its fly-by-wire chain ran at every step and its roll was
written 1875 times). **Born closed**:

```
attitude_arm_v2(persistent_id, owner, lease_seconds=1, mode=0, axes_mask=3, inactive_options=0) -> str
attitude_inactive_status_v1() -> list[float]
```

`inactive_options` 0 is exactly `attitude_arm_v1`. Bit 1 (needs mode 1 with axes bit 2):
while the vessel is loaded, unpacked and NOT the active vessel, its pitch/yaw channel leases
its gimbals (and, with bit 2, the lateral force its independent throttles) under its OWN
per-vessel authority. Those leases live apart from the active vessel's: `RestoreAll`, the v2
owner and the "another owner is present" check neither see nor touch them, so the active
vessel's path is unchanged. They follow the same rules (QW5 ramp per physics step, gimbals
handed back without a jump, original throttle fields restored, real-time expiry, 1 s
authority renewed at every writing step) and the whole authority is handed back when the
vessel becomes the active one (before any frame of the active path), is unloaded or packed,
or the scene changes. With bit 1, a vessel loaded and not active at arming skips the "another
owner is present" check. Options 2 and above 3 are refused.

`attitude_inactive_status_v1()` returns `version (1), count, stride (6)`, then per authority
`persistent_id, gimbals leased, throttles leased, real-time lease left (s), vessel is the
active one now (0/1, handed back at the next step), held (0/1)`. In the frames, `gimbal_authority`
(state 104) reads 1 while either authority is held.

KSP (decompiled `Assembly-CSharp`): `ModuleGimbal.FixedUpdate` reads its own
`vessel.ctrlState` and returns while `gimbalActive` is false; `ModuleEngines.UpdateThrottle`
reads `independentThrottlePercentage`, else its own `vessel.ctrlState.mainThrottle`;
`Vessel.FeedInputFeed` calls `OnFlyByWire` then `propagateControlUpdate`. kRPC 0.6.0
`PilotAddon` hooks every vessel's `OnFlyByWire`, and serves a non-active vessel's throttle in
that `FlightCtrlState`. None tests the active vessel. To verify in game: a ground check with the client.

#### Gimbal modules with several pivots

A `ModuleGimbal` may rotate several `gimbalTransforms` (pivots). Stock
(`ModuleGimbal.FixedUpdate` + `GimbalRotation`, decompiled `Assembly-CSharp`) computes the
deflection PER PIVOT: the vessel-frame command is brought into each pivot's own frame
(`t.InverseTransformDirection(ReferenceTransform.TransformDirection(PYR))`), so every nozzle
tilts the same way in the world. The lease and the AttitudeV1 columns apply ONE `(x, y)` to
every pivot in the pivot's own frame. That is the same thing when the module has one pivot,
or parallel pivots (every booster flown but the heavy one), and not when the pivots are
turned about the thrust axis: Near Future's AR-1 Cluster (`nflv-engine-ar1c-1`, six
`B_AR1C_Gimbal` pivots, one per nozzle, turned to its azimuth) then tilts its six nozzles in
six directions, a swirl: roll torque `sum r T` per degree, no pitch/yaw. Measured on the heavy
booster (a recorded flight, 8 such parts): published columns 350-600 N.m/deg of
pitch/yaw and 1.014e5 N.m/deg of roll per part at 4372 kN (the swirl model gives 1.0136e5;
`build/tests/PivotTests.cs`), against 8.26e5 N.m/deg of pitch/yaw for a coherent tilt;
realized engine torque under the lease ~250 N.m against 1.4 MN.m of RCS. **Born closed**:

```
attitude_gimbal_pivots_v1(token, mode) -> list[float]
```

Reset to mode 0 at every arming. Mode 0: V1, unchanged. Mode 1: a module whose pivots are not
all parallel to pivot 0 (unit axes within 1e-6 of `1 - cos`) is mapped per pivot
(`GimbalPivots.cs`): its `(x, y)` is a rotation in pivot 0's frame at rest, `w = x X0 + y Y0`,
and pivot `i` turns by `(w . X_i, w . Y_i)`, each component clamped to the module box (ranges
times limiter) as stock clamps each one. The columns are the chain rule of the pivots'
(`AttitudeGeometry.GimbalColumnsPivots`), the lease applies the same mapping
(`Actuation.ApplyGimbalPivots`, and the bumpless return writes each pivot's own reached
rotation into `oldActuationLocal[i]`), the lateral force uses pivot 0's axes and the pivots'
force columns, and a module that stock drives is read at pivot 0's rotation
(`oldActuationLocal[0]` with a response speed). Modules with one pivot, or parallel pivots,
take the V1 path in both modes. Frames of a step with at least one mapped module carry
capability bit 9. Returns `accepted (1), version (1), mode, gimbal modules, modules with two
pivots or more, modules mapped in mode 1, most pivots in one module, attitude tick` (counts
`NaN` while the vessel is not loaded). To verify in game: a ground check with the client.

**Saturation (ground check of 27/09).** The module box is `+-range` on each of `x`, `y`, so a
mapped module can be commanded to `|w|` up to `range sqrt 2` (2.83 deg), while every pivot
component stays free only up to `|w| = range` (the pivots' 12 axes are 30 deg apart). Beyond,
components clamp at the box as stock clamps them, and the published columns are the exact local
derivative: a clamped component counts nothing. In the clamps the law drove the heavy booster's
modules to their box corner: columns 70 890 and 55 500 N.m/deg for 76 460 at rest, exactly
`sqrt(31)/6` and `sqrt(19)/6` (2 and 4 components clamped), and `sum B.u0` was 0.57 of the
realized torque -- the clamp, reproduced to 2 % by `PivotTests` (realized 1.32 against 1.34
MN.m). INDI allocates increments on those local columns, which is right; the law's AUTHORITY
(`alpha = sum |B| span / I`) reads the module's columns with every component free
(`AttitudeInput.BAuthority`), otherwise it fell to 0.38 of the free value at a 45 deg module's
corner (under half the torque realized there) and the `nu` bound, the sqrt reference and the
guards would read a collapsing authority at saturation. The free authority is about 1.1 x the
torque the clamps realize at the corner. Mode 0, and a step with no mapped module: `alpha` from
`B`, bit for bit.

**translation and exact authority.** The translation by the engines solves a TOTAL
deflection from rest (tilt matrix `K phi = m a`, feed-forward of the common deflection's torque):
on the local columns a clamped component counted nothing, so the tilt matrix jumped to 4/6 as
soon as `|w|` passed the range along a pivot axis (0.378 at the 0.8 box's diagonal, where the
feed-forward cancelled 0.40 of the torque). For a mapped module the translation now reads the
FREE columns for the tilt matrix and the torque of a tilt, and the wrench the module REALIZES for
the common deflection (`GimbalPivots.Realized`: each pivot component clamped, holding its bound)
for the feed-forward and the commanded/realized accelerations. Every V1 module: unchanged. The
heavy booster never gets there: its 8 clusters share one axis, the differential has no lever
(0.133 m < 0.25 m) and the translation is refused (state 4, bit 6). **Mode 2** (born closed) is
mode 1 plus the EXACT authority: the largest torque the module realizes over the law's span box
(`GimbalPivots.ExactAuthority`, vertices of the clamp lines' arrangement: corners, edge crossings,
crossings; the corners alone miss it -- 5/6 with X0 on the axis), fed to the law as the free
columns scaled per axis; frames then carry capability bit 10. On the heavy booster's engines the
free authority is 1.159 x the exact one.

#### Options of an arming (refactor lot 1: motif 15)

**Born closed**: every arming resets the options to 0, and options 0 are exactly the behaviour
without this call (same blocking sets, columns, writes, frames and returns, bit for bit).

```
attitude_options_v1(token, options) -> list[float]
attitude_capabilities_v1() -> list[float]
attitude_layout_v1(name) -> list[str]
```

`attitude_layout_v1(name)` returns the field names of a layout, in order: `translation_evidence`
as before, and since refactor R1 every layout of `conn.bridge.layouts_v1()` (sections
concatenated; `conn.bridge.layout_v1(name)` gives sections, strides and units); an unknown name
is refused with `[NOT_FOUND]`.

`attitude_options_v1` returns `accepted (1), version (1), options now, supported options (32767; 16383 before an earlier change, 8191 before an earlier change, 4095 before an earlier change, 2047 before an earlier change, 1023 before an earlier change, 511 before an earlier change, 255 before an earlier change, 127 before an earlier change, 63 before an earlier change, 31 before an earlier change, 15 before an earlier change, 7 before an earlier change),
surfaces answering a ctrlState axis, of which neutralizable, axes they answer (bit 0 pitch, 1
roll, 2 yaw), attitude tick` (surface counts `NaN` while the vessel is not loaded); other bits
are refused. `attitude_capabilities_v1()` needs no token: `version (1), supported options,
evidence block protocol (1), its schema (1), its field count (11), surface neutralizer
available (1/0)`. A DLL without these procedures has none of the options.

- **Bit 1, per-family blocking.** a recorded flight: four grid fins with
  authority raised motif 14 at every tick, and 14 was in both blocking sets: 0 writes on 7 829
  frames, no gimbal per engine and no translation by the engines in the descent. With bit 1 a
  model motif sanctions the family it is about, never a whole channel:
  - 13 (RCS precision mode, `fullThrust`) and 20 (the kRPC AutoPilot's callback is not before
    ours) **freeze** the ctrlState columns: `B = 0`, bounds at the served `u0`, not written; the
    leased gimbals, which do not go through fly-by-wire, are allocated alone and written;
  - 14: the control surfaces leave `B` (`GetPotentialTorque` multiplies the torque by the
    lever, component by component: not a column). In every physics step where the loop writes
    ctrlState pitch/yaw (or wrote, at the last fly-by-wire, pitch/yaw or a roll the surfaces
    answer: whether a surface's FixedUpdate reads this step's fly-by-wire or the previous one is
    not measured, both are covered; a roll alone never starts it, but once the pitch/yaw channel
    has, a roll the loop keeps writing keeps them neutral, as the witness flights flew), each
    `ModuleControlSurface` that answers a ctrlState axis (enabled, range and authority limiter
    not 0, axis not ignored) is **held neutral for that step only**: its protected
    `deflectionDirection` (1 from the constructor, never persisted, not in the part menu, not
    exposed by kRPC) is set to 0 at Earlyish and put back at BetterLateThanNever of the same
    step (also at the next tick, `Drop`, scene change, the tick's exception). Its target
    deflection is then 0 and it returns to neutral at its actuator speed (grid fin KRE: 40°/s).
    Not `ignorePitch/Yaw/Roll` nor `authorityLimiter`: they are KSPFields, saved by a quicksave,
    shown in the menu and written by the client through kRPC (`ControlSurface.pitch_enabled`),
    and kRPC runs its RPCs inside the Earlyish..BetterLateThanNever window, so a restore could
    undo a client write. The price: every axis of the surface, the AutoPilot's roll too when
    the loop writes pitch/yaw but not the roll. A deployed surface (`deploy`) cannot be held
    neutral: it freezes the ctrlState axes it answers;
  - a roll written without that neutralization while a surface answers the roll is held back
    (motif 19 + 36): its roll effectiveness is not in the column;
  - 13, 14 and 20 stay in the frames, informational; the per-axis guards (16-18), fed by the
    measurement, stay the net. Frames carry capability bit 11 and the motif bits below.
- **Bit 2, evidence aging.** Bit 32 of the translation published the model's authority
  × the measured share, and the share moved only on a step following a translated step: a share
  measured at 0 stayed 0 while the client, reading 0, stopped asking for the translation that
  alone could measure it again (an absorbing state, 3-5 s in the heavy booster's precision
  phase). With bit 2 the forgetting means forget in GAME time whatever happens (a step without a
  sample is a zero contribution, `e^(-elapsed/τ)` on both means; after a regular step a sample
  is exactly the former update) and the published factor relaxes toward the model with the age
  `A` of the last sample: `f = 1 - (1 - f_measured) e^(-A/τ)`, τ = 1 s (the forgetting time).
  With the model above 0 the published authority is 0 only on a step whose fresh sample says 0;
  one step without a sample later it is ≥ `(1 - e^(-dt/τ))` × the model, half the model after
  τ ln 2 = 0.69 s, 95 % after 3τ = 3 s. Property test (`build/tests/FamilyEvidenceTests.cs`, fake
  game clock, the client asking only while the published authority is > 0, gimbals that never
  hold the common): never more than 2 steps at 0; without bit 2, stuck at 0 for good. Frames
  carry capability bit 13, and the translation return carries the evidence block below.
- **Bit 4, the non-converged iterate is served (motif 15).** The WLS iterate left when its 20
  iterations run out is feasible (the box holds by construction) and no worse than the warm
  start (primal active set on a convex QP: every step lowers the cost), and the hedge and the
  guards already read it. With bit 4, 15 is informational and that iterate is written, instead
  of a one-step fallback that hands the gimbals to the stock Lerp and back (7 and 45 such steps
  in the heavy booster's last two descents). Frames carry capability bit 12.
- **Bit 8, the gimbals held at the measured contact.** Six flights of the super hope
  (07/10-08/10): the client wrote pitch/yaw off 2 m above the contact and the kRPC AutoPilot took
  the gimbals back with its integrators wound up (its own command, which the loop overwrote at
  every step, stuck at -1 in pitch): gimbals 0.2-0.3° → 6-14° (stop 10.5°), |ω| ≤ 0.3 → 1.8-3.8
  °/s in the last 2.4 s. The client now keeps pitch/yaw until the contact it measures (its cut);
  the loop stops at the physics step of the contact (motif 27). With bit 8, at the MEASURED
  contact (Landed outside PRELAUNCH, or Splashed: not the impossible demand, which a flight can
  produce) and while the client's write mask still allows pitch/yaw, the gimbals the loop leased at
  its last write stay **leased at that deflection** (motif 39, write state fallback,
  `gimbals_leased` > 0) instead of going back to the stock Lerp toward the AutoPilot's ctrlState.
  Held, not recentred: that deflection balanced the torque about the centre of mass at the
  contact step, so holding it changes nothing while the legs take the load, and a bounce resumes
  the law from its own last output. The hold writes no ctrlState (the AutoPilot keeps the pitch/yaw
  RCS and the roll), gives the differential throttles of the lateral force back to the main
  throttle at its first step (the client's cut acts at once), and only continues a lease: the
  client's write-off, a reference older than 0.3 s, the lease, the authority, a topology change or
  an unloaded vessel hand the gimbals back as before; the contact's own consequences (axis guards
  and their hold, a step not converged or approximate, the fly-by-wire order) do not.
- **Bit 16, the coast write bits.** The loop owns pitch/yaw in the COAST (step M5 of the
  the project's plan), and the coast is not the descent: the control surfaces' authority grows with
  q (6 MN·m on pitch above 60 kPa on the super hope), and the Python loop is heavy there (the
  reference's age passed 0.3 s in 10-34 % of the coast steps of five flights, in about 110 runs a
  coast). Without bit 16 a `write_mask` above 3 is refused exactly as before. With it the
  `write_mask` of `attitude_reference_v2` may also carry, **per reference**:
  - `4`, **surfaces out**: no control surface is engaged into the law for this reference
    (SurfaceModelV1 sees none of the loop's axes as owned, sanction bits 15-17); out, every
    surface is stock, and while the loop writes pitch/yaw option 1 holds it neutral. The
    loop flies the coast on RCS and wheels alone; the client stops setting the bit in the descent,
    where the surfaces engage one written step later. Motif 41.
  - `8`, **hold the reference**: the pitch/yaw channel keeps this reference up to its own validity
    (`ut_valid_until`; motif 2 still ends it) instead of falling back past 0.3 s (motif 26): a
    fallback would hand the gimbals and ctrlState to a kRPC AutoPilot whose integrators wound up
    while the loop wrote over it, and that AutoPilot holds the same stale target anyway.
    Motif 40 on every step the hold keeps the channel.
  Both bits are read through the options of the tick: withdrawn, bit 16 masks them out of the
  stored mask. Nothing else changes: blocking sets, write rules, neutralization, roll gate.
- **Bit 32, the RCS torque in flight.** Stock KSP fires an RCS module from `inputRot`, which
  `ModuleRCS.Update` -- the RENDER frame, not the physics step -- fills from `ctrlState` (each
  component zero below the module's `EPSILON`, 0.05 by default, proportional above): a command the
  loop writes at the fly-by-wire of physics step k fires from step k + 1 at the earliest, one step
  after the wheels and the leased gimbals, later when two physics steps share a frame. The INDI
  increment (target `M_d - M` from the served command) took the measured torque for that of the
  served command when it was that of the command before: with a dominant RCS column
  `u_k = u_{k-1} - u_{k-2} + c`, poles at +-60 deg on the unit circle, a SIX-step cycle (8.3 Hz) held
  at the +-1 bounds. a recorded flight (the loop flying the coast on RCS and wheels):
  `-1, -1, 0, +1, +1, 0` at +-697 kN.m for a demand of 40-60 kN.m, served at +-1 on 65 % of the
  steps, 1.56 t of propellant in the coast (the wheels at 0 from mid-coast: the battery drained by
  the cycle, back at the first step of thrust). With bit 32, at every Earlyish, per RCS module: the
  torque of its columns (the WLS's own, signed) at the command its next Update will make of the
  served `ctrlState` (enable flags, its `EPSILON`), minus their torque at the command it fired at the
  previous step (its private `inputRot`, read at that step's Earlyish: exact at any frame rate), is
  taken off the allocation target (`AttitudeInput.TorqueFree`, the free torque of lot 3; the law's
  maths is unchanged): the RCS lands on the torque the law wants when it fires, and the model's
  error stays in the increment (integral action kept). Nothing for a module whose model is
  approximate (precision mode, `fullThrust`), at the first step of an arming, after a topology change
  or a tick gap, at the Pump's fallback stage, or when `inputRot` cannot be read; frames carry
  capability bit 16 on the steps that used it. The in-flight torque is published implicitly:
  `torque_desired - torque - torque_alloc_target`. Nothing changes where the served and fired
  commands agree (both in the dead zone: the descent, where the gimbals hold everything).
- **Bit 64, the boostback write bit.** The loop owns pitch/yaw in the BOOSTBACK (step M6 of the
  the project's plan) while its gimbals are its actuator. Offline closed loop of the law flying four
  boostbacks of the super hope (08-09/10; the client): at full thrust it holds them
  as the tuned AutoPilot does and points better, on gimbals alone; in the throttle taper that ends
  them the gimbals' authority falls below the RCS's (about 5 % of full thrust) while the law keeps
  allocating them first (priority 1 against 3): they run to their stops (up to 39 % of the
  gimbal-samples), and their torque comes with its lateral force, torque / lever (15-35 kN, four times
  the RCS's for the same torque), in the last seconds before the cut. Without bit 64 a `write_mask`
  carrying 16 is refused exactly as before. With it the `write_mask` may also carry, **per reference**:
  - `16`, **gimbals lead**: the pitch/yaw channel writes only while its gimbals LEAD -- on body x AND
    z, the authority of the gimbal columns, `sum_j |B_aj| span_j` (the law's own alpha rule, a mapped
    module's unclamped columns), is above zero and at least that of the three ctrlState columns
    (`AttitudeWrite.GimbalsLead`); motif 42 while they do not. Once the channel has written under the
    bit and they stop leading, it does not write again while the references keep the bit (the latch,
    motif 43; a reference without the bit, or a new arming, clears it): the AutoPilot keeps the end of
    the boostback, and the client resets it as at any handback. Before the thrust arrives the gimbals
    do not lead either: the channel waits for them.
  Read through the options of the tick: withdrawn, bit 64 masks it out of the stored mask. Nothing
  else changes: blocking sets, write rules, neutralization, roll gate, contact hold.
- **Bit 128, the gimbals without thrust to neutral and their columns at the coming thrust.**
  Measured (a study of 14/10, 82 flights): the super hope's gimbals sit at a stop 33 % of its coast
  under the loop; at the heavy booster's descent ignition they are at 78-100 % of their travel and the thrust
  brings a 1.0-3.7 MN.m kick for 0.4-0.9 s, |omega| times 2 to 4. Two causes, two parts:
  - **the law** (`AttitudeMath`, `AttitudeInput.TrimNullColumns` = the gimbal columns): a column whose own
    effect at full span is below `EffectFloor` (no thrust) had the floor's regularization weight, about
    1e-12: its KKT multiplier never passed the solver's tolerance, so the warm working set kept a dead
    gimbal at the bound it reached under thrust. With the bit such a gimbal column gets the zero-span weight
    `sqrt(gamma)`: decoupled from every torque row, the WLS puts it exactly at `u_pref`, neutral, within its
    box. Columns with an effect are untouched (bit for bit). Golden vectors of the law with it:
    `build/tests/golden/attitude_v1_golden_trim.json` (the client's mirror replays them; the historic
    `attitude_v1_golden.json` does not change);
  - **the columns** (`AttitudeV1GimbalTrim.cs`, `GimbalSpool`): each engine of a gimbal's part is taken at
    the thrust its stock spool (`ModuleEngines.UpdateThrottle`: a Lerp of `engineAccelerationSpeed` or
    `engineDecelerationSpeed` per second toward `requestedThrottle`, or at once without
    `useEngineResponseTime` or with the alternate law) reaches over the gimbal's own response time (the time
    constant of its stock Lerp, `-1 / ln(1 - gimbalResponseSpeed dt)` steps; one step for an instant
    gimbal): finalThrust per unit of currentThrottle while it thrusts, else `MaxThrustOutputAtm` at the
    part's pressure without the limiter; never below finalThrust (a spool down keeps the realized columns,
    true over modelled effectiveness in (0, 1]). The gimbal columns (a mapped module's free and per-pivot
    columns too) are computed at those thrusts, and the torque that spool adds with the nozzles where they
    are, `sum r x F` at the expected thrust minus at finalThrust (the misalignment of the thrust line
    included), is taken off the allocation target (`AttitudeInput.TorqueFree`): the gimbals go to the trim
    of the coming thrust instead of running to their stops on the small columns of a spool start. The
    lateral-force geometry keeps finalThrust. Bit 16 of `write_mask` (option 64) reads these columns: under
    bit 128 the gimbals lead from the spool start.
  Frames carry capability bits 17 (a null gimbal column regularized) and 18 (gimbal columns at the coming
  thrust). Bench (`build/tests/GimbalTrimTests.cs`): KSP's spool step by step, the dead gimbal held at its
  bound without the bit and at neutral with it; a closed loop of a saturating burn, a coast and an ignition
  (heavy booster's numbers): the gimbals' torque at the ignition 0.92 MN.m without, 0.018 with, |omega|
  0.0024 against 0.00011 rad/s. When the AutoPilot owns the coast (gimbals parked at their stop by its
  saturated ctrlState) and the law takes the ignition, the spool's free torque commands them back from the
  first step (0.83 against 0.92 MN.m): the full gain needs the law to own the seconds before the ignition.
- **Bit 256, the free authority of the lateral force.** Measured on eight pad landings of the heavy booster
  (the last six seconds of each): the authority of mode bit 32 (the model's times the
  measured share) was 0 on 24-33 % of the translating steps, in three or four holes of 0.5-1 s, while the gimbals
  realized the commanded common (realized / commanded 0.96-1.02). The measured share projects the TOTAL gimbal
  deflection on the realized common; the law's attitude part (its largest column at 74-104 % of a 2-degree span on
  that booster, against a common of about a tenth of a degree) projects at -1.0 to -1.5 times the common whenever the
  nose turns toward the demand (the gimbals push the tail the other way): the share goes negative, the published
  authority to 0, and the client's final phase switched law at every hole (4 to 6 times in 6 s). With the bit, every
  planned step with the computed levers (mode bit 4, the joint allocation prepared) also solves the joint allocation
  ALONG the gap the nose does not give (this step's `WantX`, `WantZ`, unit), on a scratch copy (the step's own
  allocation, its outputs and the law's input are not touched), twice:
  - with the model's bounds (`along_model`);
  - with what the attitude leaves (`along_free`): each usable gimbal column's directional ranges for the common are
    the smaller of its range and the room its REALIZED attitude part (u0 before the shift minus the realized common of
    the previous written step, the law's own) leaves in its box divided by `frac_gimbal` (so the common keeps within
    both `frac_gimbal` of the range and that room; the range itself, to the bit, while the attitude leaves the room);
    each motif amplitude's bounds keep its realized attitude part (realized amplitude minus the previous feed-forward)
    plus the translation's within the symmetric half width, 0 always inside; the RCS columns are the joint's own (mode
    bit 16: the nozzles' headroom after the served rotation).

  The free authority is the model's (offset 25 of layout 3, before the bit-32 factor) times `along_free / along_model`
  bounded to [0, 1] (`FreeAuthorityRatio`): the model's when the attitude leaves the room, lower when the attitude takes
  the range the translation needs, never a share the attitude's own deflection turns negative. An attitude part OPPOSED
  to the gap frees range on that side. No gap this step (no request, mode bit 2 off, the nose gives it all): the ratio
  is 1 (`source` 2); a failed solve or a model that reaches nothing along the gap: 1 (`source` 3). Published at offset 3
  of the free block while the layer may act (as offset 1), else 0. Nothing else changes: the layer, the law, every frame
  but capability bit 19, every other return value. Pure maths in `AttitudeTranslation.cs` (`JointTranslation.CopyInputs`,
  `AuthorityAlong`, the variables' `Lower` / `Upper`, -1 / 1 for every other caller; `FreeGimbalRanges`,
  `FreeDiffBounds`, `FreeRatio`, `FreeAuthorityRatio`), golden vectors `build/tests/golden/attitude_v1_golden_free.json`
  (the client's mirror replays them bit for bit). Bench (`build/tests`, the heavy
  booster model at throttle 0.08): the hole -- the attitude part at -1.3 times the common on every gimbal: measured
  factor 0, free ratio 1; no attitude part: `along_free == along_model` in 40 directions; the attitude part along the
  gap up to the stops: the ratio falls monotonically to 0.

| Motif bit | Name (bit 1 only, informational) |
|---:|---|
| 32, 33, 34 | ctrlState column of body x (pitch), y (roll), z (yaw) frozen this step |
| 35 | control surfaces held neutral this step |
| 36 | roll not written: a surface answers the roll and is not held neutral |
| 39 | (bit 8 only) pitch/yaw gimbals held leased at the measured contact |
| 40 | (bit 16, `write_mask` 8 only) pitch/yaw kept on a reference older than 0.3 s |
| 41 | (bit 16, `write_mask` 4 only) the client's write mask holds the surfaces out of the law |
| 42 | (bit 64, `write_mask` 16 only) pitch/yaw not written: the gimbals do not lead the ctrlState channel |
| 43 | (bit 64, `write_mask` 16 only) pitch/yaw not written: the gimbals stopped leading after a write (latched) |

**Evidence block** (bit 2). The `attitude_translation_v1` return of the mode's layout is padded
with `NaN` to 32 values (layout 3's length), then `protocol (1), schema (1), n` and `n` values,
whatever the mode. Fields are only ever appended; map them by name with
`attitude_layout_v1("translation_evidence")`, never by a copied index:

| Field | Meaning |
|---|---|
| `model_authority_ms2` | the model's lateral authority this step (NaN: the layer did not compute it) |
| `measured_share` | Σ u·c / Σ c² once evidence (NaN: none) |
| `evidence_deg2` | Σ c² mean, aged to this step |
| `evidence_min_deg2` | the evidence threshold, (0.05°)² per gimbal column |
| `evidence_age_s` | game seconds since the last sample (NaN: never) |
| `factor_unaged` | the factor before an earlier change |
| `factor` | the aged factor, applied to the model's authority (with mode bit 32) |
| `servable_authority_ms2` | = offset 1: model × factor while the layer may act, else 0 |
| `aging_tau_s` | τ of the forgetting and of the aging |
| `attitude_tick`, `ut` | the step these values belong to |

**Free block** (bit 256). The `attitude_translation_v1` return of the mode's layout is padded with `NaN` to 32
values, then the evidence block (always there under bit 256; its values `NaN` without bit 2), then `protocol (1),
schema (1), n` and `n` values. Fields are only ever appended; map them by name with
`attitude_layout_v1("translation_free")`:

| Field | Meaning |
|---|---|
| `free_authority_ms2` | model x ratio while the layer may act (as offset 1), else 0; NaN when not computed |
| `ratio` | `along_free / along_model` in [0, 1]; 1 without a gap or after a failed solve; NaN when not computed |
| `model_authority_ms2` | the model's authority this step (offset 25 of layout 3) |
| `along_model_ms2`, `along_free_ms2` | the joint allocation's reach along the gap, with the model's bounds, then with what the attitude leaves |
| `direction_x`, `direction_z` | the gap's unit direction, body (NaN: none) |
| `source` | 0 not computed (no planned step with the computed levers), 1 along the gap, 2 no gap (ratio 1), 3 a solve failed (ratio 1) |
| `gimbal_attitude_max` | the largest realized attitude part of a usable gimbal column over its directional range |
| `diff_attitude_max` | the larger realized attitude part of the two motifs over the symmetric half width |
| `attitude_tick`, `ut` | the step these values belong to |

KSP (decompiled `Assembly-CSharp`): `ModuleControlSurface.FixedUpdate` calls
`FixedCtrlSurfaceUpdate`, which reads `vessel.ctrlState` pitch/yaw/roll through
`ignorePitch/Yaw/Roll` and multiplies the clamped command by `deflectionDirection` (and again
after the deploy angle), then `MoveTowards(deflection, action, actuatorSpeed·dt)`; the
constructor sets `deflectionDirection = 1` and nothing else writes it. kRPC 0.6.0 runs its RPCs
in `KRPC.Addon.FixedUpdate`. To verify in game: the client`
(client repository).

#### Main throttle at the physics step

```text
attitude_throttle_v1(token, sequence, kind, frame, parameters, ut_stamp, ut_valid_until) -> list[float]
    # layout attitude_throttle_v1 (protocol 1, schema 1, 39 values)
```

**Born closed**: refused with `[UNSUPPORTED_MODE]` unless option 512 of `attitude_options_v1` is set; without it
the loop's fly-by-wire link never touches `ctrlState.mainThrottle`. Every thrust was decided and written by the
client: 0.26-0.70 s from the order to the thrust, a cadence guard of 0.4-1.3 s at the descent ignition
, the flip's throttle of calage computed on a state 0.2-0.7 s old (the cross velocity of the flip
14-40 m/s on 11 of 42 replayed separations). With the option the client arms an ORDER; at every Earlyish the loop
decides the main throttle of the step (after the law) and its fly-by-wire link, last in the chain after kRPC's
control inputs and AutoPilot, writes it into `ctrlState.mainThrottle` (`ModuleEngines.UpdateThrottle` reads
`mainThrottle x thrustPercentage / 100`, decompiled).

| kind | parameters | the throttle |
|---|---|---|
| 0 stop | none | nothing written: the client's throttle |
| 1 ramp | `u0, slope (1/s)` | `clamp01(u0 + slope (ut - ut_stamp))` |
| 2 pulse | `u, steps` | `u` for `steps` physics steps, then the client's |
| 3 cut when s = 0 | `u_before, u_after, a_x, a_y, a_z, c, velocity (0 surface, 1 orbital)` | `u_before` while `s = a . V + c > 0` (`a` in `frame`, `V` the vessel's `srf_velocity` or `obt_velocity`), `u_after` from the first step `s <= 0`, latched |
| 4 ignition when h <= h_ign(v) | `u_before, u_after, h_ref, c0, c1, c2` | `u_before` while `h > c0 + c1 v + c2 v^2` (`h = altitude - h_ref`, `v` the vertical speed), then `u_after`, latched |
| 5 the flip's calage | `b_x, b_y, b_z, h_x, h_y, h_z, slope, rcs_x, rcs_z, verdict` | see below |

Every kind but 0 ends with an earlier change's thrust guard, `threshold, chain steps`: an episode of thrust starts at the
first step written above 0; past `(chain + 1)` physics steps (plus `-ln(1 - threshold) / engineAccelerationSpeed`
for a spooling engine) a realized thrust of the main-throttle engines below `threshold x` the episode's smallest
throttle `x` their full thrust (for the guard, every IGNITED main-throttle engine counts, one flamed out for want of
propellant included: kRPC's `max_thrust` over the active engines, the client's own guard) trips the guard, latched for
the order (state 3: the client's throttle). The order
is valid until `ut_valid_until` (game time) and while the arming lives; a renewal of the same kind (a newer
`sequence`) keeps its runtime (episode, guard, a fired trigger), another kind or a stop starts afresh.

**The flip's calage (kind 5)** is the client at the
physics step, on the loop's own quantities: the law's pointing error, pseudo-command and inertia; the external
torque `I w_dot_f - M_f` of its filters (the measure an earlier change checked against kRPC's aerodynamic torque, slope
0.99-1.005); the RCS + wheels authority of the order (`NaN`: the loop's ctrlState columns); the gimbals'
authority at full main throttle (each gimbal module's columns at the full thrust of its part's engines, the law's
alpha rule over the span; finalThrust per unit of currentThrottle while thrusting, `MaxThrustOutputAtm` at the
part's pressure otherwise, limiter excluded, times `thrustPercentage`). In order: an unreadable RCS authority or
torque decides nothing (motif 11); the nose not on the side of the boostback direction `b` or its horizontal
part against the burn axis `h` (both in `frame`): 0 (motif 7); then an earlier change's branch `calage` (the aerodynamic torque
along the error opposes the correction more than the RCS + wheels can) or an earlier change's branch `saturation` on the law's
REAL pseudo-command (`max over x, z of (|I nu - M| - R) / R` beyond the law's guard ratio, 0.9): the smallest
throttle whose gimbals cover the whole aerodynamic torque, clamped to 1 (motifs 4, 5; otherwise 1 nul, 2 aide,
3 sous_rcs: 0; 6 sans_cardan: nothing); then an earlier change's guard of a push: the required throttle above 1 (motif 8), or
above the cost cap `slope x mass / (full thrust x lost fraction)` with the lost fraction
`(1 - max(0, n.b)) + max(0, up.(n - (n.b) b))` on the loop's own nose and local vertical (motif 9), or an
unreadable slope (motif 10): 0. A step without the law's decision (tick gap, filter restart) holds the last value
for at most `chain + 1` steps (state 2), then 0 (motif 12). `verdict` (an earlier change's, -1 not judged, 0 the AutoPilot, 1
the law) is published: the calage pushes under the law and under the AutoPilot alike.

**Where it may write**: armed (`mode=1`) with its fly-by-wire link on the vessel, at Earlyish (not in the pump's
fallback), out of warp; otherwise the order is blocked (state 5) and the client's throttle flies. **The client's
throttle is always the fallback**: the loop writes `ctrlState` per step, after kRPC's control inputs wrote the
client's last throttle into it; whenever it does not write (an order ended, the guard, a blocked step, the option
withdrawn), the client's throttle flies at once. **At the expiry of the arming's lease** while the loop wrote the
throttle at the last step, a lone link at the end of the vessel's fly-by-wire chain writes 0 until the throttle
kRPC serves (the client's) changes, a new arming of the vessel or the scene's end: kRPC keeps a disconnected
client's throttle, the loop gives it back at zero.

The reply: `accepted (1, or 0 for a sequence not newer), protocol, schema, kind, sequence` (the order now), then
`decide_kind, decide_sequence, state` (0 idle, 1 writing, 2 held, 3 guard, 4 ended, 5 blocked), `motif` (0 none,
1 nul, 2 aide, 3 sous_rcs, 4 calage, 5 saturation, 6 sans_cardan, 7 hemisphere, 8 hors_autorite, 9 cout,
10 sans_cout, 11 sans_mesure, 12 sans_loi, 13 garde_poussee, 14 schedule, 15 pulse, 16 avant, 17 declenche,
18 anneau: a ramp order took the reference ring's throttle),
`decided` (the last Earlyish's value, `NaN` none), `written, client_throttle` (the last fly-by-wire's, at
`fly_by_wire_tick`), `decide_tick, decide_ut, fly_by_wire_tick, guard_tripped, episode_on, episode_min,
episode_seen, fired, writes, held, unrealized, required, cost_cap, fraction_lost, m_par_nm, rcs_par_nm,
gimbal_authority_x_nm, gimbal_authority_z_nm, torque_ext_x_nm, torque_ext_z_nm, full_thrust_n, thrust_now_n,
mass_kg, dot_target, verdict, missed, attitude_tick`. Law frames carry capability bit 20 on the steps that decided
a value. The pure part is `Model/MainThrottle.cs`; its golden vectors
(`build/tests/golden/attitude_v1_golden_throttle.json`) are replayed by the client's mirror.

#### Roll pointed

```text
attitude_reference_v3(token, sequence, frame, dir_x, dir_y, dir_z, roll_mode, up_x, up_y, up_z,
                      omega_ff_x, omega_ff_y, omega_ff_z, ut_stamp, ut_valid_until,
                      omega_max_x, omega_max_y, omega_max_z, write_mask) -> int
```

**Born closed**: `roll_mode=1` is refused with `[UNSUPPORTED_MODE]` unless option 1024 of `attitude_options_v1` is
set; `roll_mode=0` is exactly `attitude_reference_v2` (the up vector ignored), and every V1/V2 reference clears a
roll target. The law's roll was rate damping only (`w_s = 0` on body y): to align the roll (the tower's pins at a
catch) the client handed the roll to the kRPC AutoPilot (`set_direction_and_up`), which aligned it from 14 to 39 deg
at RCS +-1 while the lateral force fell to 0.28-0.40 of its authority (measured on the client's captures). With `roll_mode=1` the
up vector (in `frame`, need not be unit nor across the nose) is carried to the body at every step; the roll error is
the rotation about the nose that brings the roof (body -z) onto the up vector's part across the nose,
`atan2(-u_x, -u_z)` (a positive rate about +y reduces a positive error); it goes into the law's error (published in
the frame's error y) and the roll follows the square-root reference of the other axes on its own authority and its
own AutoPilot limit (`omega_max_y`, else the scalar limit): `w_r,y = sat(sign(e) min(K_lin |e|, sqrt(2 eta alpha_y
|e|))) + w_ff - w_pch`, `nu_y = K_w,roll (w_r,y - w_y)`, served by the same allocation as the translation's columns
and written by the roll channel (write-mask bit 1, its guards and coupling rules). A frame that no longer converts or
an up vector along the nose (less than `AttitudeMath.RollTargetMinAcross`, 1e-3 of it, across the nose: the body
conversion is float, ~1e-7) leaves the roll damped for the step. Law frames carry capability bit 21 on the steps with
the roll target. Pure part `AttitudeMath.RollError` and `AttitudeInput.RollTargetValid` (born false: the law bit for
bit); golden vectors `build/tests/golden/attitude_v1_golden_roll.json`. The roll profile brakes on the modelled
roll authority: an RCS roll column overestimated (the client's M3: x15-40 on the super hope) slows the
convergence and overshoots (closed loop of the tests: 22 deg for x20); measure the column before arming.

**Roll mode 2, the shadow** (option 1024 too): the same roll error, carried and computed the same way, goes into the
frame's error y (capability bit 21) while the law stays exactly mode 0's (`RollTargetValid` false: `AttitudeMath`
ignores `Error[1]` then). The client flies the roll with the AutoPilot as before and reads, frame by frame, the error
the law would have pointed on, against the AutoPilot's own roll error, and the roll's realized response under the
AutoPilot's commands (the frame's measured and modelled roll acceleration): the in-flight check of the geometry and of
the roll column before mode 1 is armed.

#### Reference ring

```text
attitude_reference_ring_v1(token, sequence, frame, samples) -> list[float]
    # layout attitude_reference_ring_v1 (protocol 1, schema 1, 20 values)
```

**Born closed**: refused with `[UNSUPPORTED_MODE]` unless option 2048 of `attitude_options_v1` is set; without it
nothing of the ring runs. The client's reference reached the loop 60 ms late at the median, 80 ms at p95, up to
0.2 s (the study of 14/10), and the loop held it until the next one: the law followed a staircase, always late. A
guidance that knows where its reference goes can send it ahead, dated: `samples` is a flat list of 2 to 64 samples
`[ut, dir_x, dir_y, dir_z, a_x, a_y, a_z, u]` in `frame`, `ut` strictly increasing. Three channels, each finite in
every sample or `NaN` in every one: the wanted nose (any length but not null, normalized when stored, two
consecutive samples less than 90 deg apart), the lateral acceleration of the translation request (m/s^2), the
main throttle (0-1). Any other list is refused with its reason.

At every Earlyish whose UT lies in `[ut_first, ut_last]`, the segment `ut_k <= ut <= ut_k+1` (the first such)
gives `f = (ut - ut_k) / (ut_k+1 - ut_k)` and every channel the ring has: the nose by normalized linear
interpolation (nlerp), the rest linearly. Each consumer takes its channel only where it acts without a ring:

* the law's reference takes the ring's nose (in the ring's frame) instead of the standing reference's, while that
  reference is valid, and then serves **no omega_ff** (the ring is a position, never a rate: an earlier change);
* the lateral force takes the ring's acceleration instead of the live request's (mode, validity: the request's);
* a live ramp order of `attitude_throttle_v1` (kind 1) decides the ring's throttle instead of `u0 + slope (t -
  t0)`, motif 18, under every rule of K5 (validity, an earlier change's guard, the fallback, the zero at the lease's expiry).

Outside the ring's span nothing is served: the loop is the loop without a ring. **The ring renews nothing**: neither
the arming's lease, nor the standing reference's validity, nor its age (the pitch/yaw write rule of 0.3 s, an earlier change's
hold). A `sequence` not newer reads (nothing changes); an empty `samples` clears the ring; arming the vessel or
withdrawing option 2048 clears it too.

The reply: `accepted (1, or 0 for a sequence not newer), protocol, schema, sequence, count, channels` (1 direction,
2 lateral, 4 throttle), `ut_first, ut_last` (the ring now), then the ring's value at the last Earlyish: `served`
(channels), `served_ut, segment, fraction, dir_x, dir_y, dir_z, a_x, a_y, a_z, u` (`NaN` where not served), and
`attitude_tick`. Law frames carry capability bit 22 on the steps the ring served a channel. The pure part is
`Model/ReferenceRing.cs`; its golden vectors (`build/tests/golden/attitude_v1_golden_ring.json`) are replayed by
the client's mirror.

#### Engine-by-engine allocation in shadow (step 1)

```text
attitude_allocation_v1(token) -> list[float]
    # block_protocol (1), block_schema (2; 1 before step 2), block_n, then the fields of layout attitude_allocation
attitude_allocation_frames_v1(persistent_id, since_tick=0, max_frames=64) -> list[float]
    # history_v1 of layout attitude_allocation_frames_v1 (protocol 1, schema 2), one frame per physics step
```

**Born closed, and a shadow**: nothing runs without option 4096 of `attitude_options_v1`; with it the loop computes
and publishes, every physics step, what an allocation of EVERY actuator would command, and writes nothing (every
other answer, every frame and the game state identical). The law's allocation
solves torques only; the lateral force is a layer in front of it (`attitude_translation_v1`) whose share collapses when
the gimbals serve the attitude. Here one problem holds them all:

* **columns**: the throttle command of every engine the differential may drive (ignited, operational, not a solid, not
  someone else's independent throttle; slope `K (1 - throttleMin) tp`, force along its realized thrust direction, torque
  at its nozzle centroid), the two axes of every usable gimbal module (the law's torque columns, the translation
  layer's force columns), the ctrlState pitch, roll and yaw (the law's columns: RCS and wheels), the RCS translation
  ctrlState X and Y;
* **rows** (body): `Fx, Fy` (axial)`, Fz, Mx` (pitch)`, My` (roll)`, Mz` (yaw);
* **the variable is each actuator's REALIZED change at a common horizon** `H`: KSP Lerps every actuator toward its
  command (the gimbal and the bridge's lease ramp by `gimbalResponseSpeed dt`, the engine spool by
  `engineAccelerationSpeed dt` up and `engineDecelerationSpeed dt` down, at once without the response time or with the
  alternate law); held `H` steps, a command `c` realizes `phi (c - u0)`, `phi = 1 - (1 - f)^H` of its side. The box of
  each variable is exact, every row linear, and the wrench rows hold on the realized state (a common tilt and the
  differential that cancels it balanced where they ARE). `H` is the largest time constant of the actuators present
  (the slowest realizes `1 - 1/e`); RCS and wheels at `phi = 1`; the command is `u0 + x / phi`;
* **strict priorities**: (1) the axial force D asks (every engine at the served main throttle, realized at `H`), (2)
  pitch and yaw (the law's `M_d - M`), (3) the lateral force of the live translation request (the gap the nose does
  not give, as the translation layer computes it), only while one is live, (4) roll. A joint solve first (every row
  nearly an equality); when it meets every row it is the answer (`stages` 0, the usual case); otherwise a cascade in
  which each earlier level is held, nearly as an equality (weight 1e9 in shares of authority), at what it achieved;
* each stage is a box-constrained regularized least squares solved by a primal active set through the 6 x 6 matrix
  `diag(q) + A_F D^-2 A_F^T` (Woodbury): `O(36 n)` per iteration. Regularization only chooses among exact solutions
  (the joint translation's `SqrtGamma`, the law's priorities: gimbals 1, ctrlState 3, the throttles the translation's
  differential priority); preferred point: throttles at the main throttle, gimbals at neutral, ctrlState at 0;
* **authority**, for the solver: the largest lateral acceleration along body `+x, -x, +z, -z` with the axial force and
  the pitch/yaw torque held as equalities, roll free, at steady state: `free_*` from the realized state with the law's
  torque target (what the attitude leaves), `model_*` from the trim (throttles at a main-throttle level, gimbals at
  neutral, no torque wanted) at the current main throttle (`model_now_ms2`, at `model_now_throttle`) and at `k / 4`,
  the gimbal columns scaled by that level's thrust. One authority solve every second step, round robin (`free_age_steps`,
  `model_age_steps`: the oldest entry's age).

The block (`state`: 0 not computed, 1 solved, 2 the law did not run: no torque target, authorities only, 3 no
actuator; `reasons`: `bits_v1("attitude_allocation.reasons")` -- 1 no law target, 2 no actuator, 4 lateral demand
with the nose too far from the vertical, 8 not converged, 16 RCS columns approximate, 32/64/128/256 axial / pitch-yaw /
lateral / roll limited, 512 an RCS command inside ModuleRCS's dead zone (EPSILON), 1024 a jitter on the 6 x 6, 2048 no
live lateral demand): the cascade (`stages`, `iterations`, `converged`), the sizes, the horizon, main throttle,
thrust, mass, the lateral demand (`want_*`) and the realized lateral acceleration now, per row the wanted realized
increment (`target_*`, `NaN` for a row out), what the allocation achieves (`achieved_*`) and the row's authority at
full span (`authority_*`), the lateral force and the pitch/yaw torque by family (gimbals, throttles, RCS), the
throttle commands' range, the largest gimbal share of its range, the largest ctrlState command, the RCS channels in
the dead zone, the free and model authorities, and `cost_us` (the snapshot and the solves of this step). Frames:
the same fields after `protocol, schema, attitude_tick, ut, persistent_id`, then per engine `flight_id,
command_realized, command, realized_change, phi, slope_n`, per gimbal module `flight_id, ordinal, realized_x_deg,
realized_y_deg, command_x_deg, command_y_deg, phi, force_per_deg_n`, the ctrlState served and commanded (pitch, roll,
yaw, X, Y), the master tick. Law frames carry capability bit 23 on the steps it solved. The pure part is
`Model/EngineAllocation.cs` (`build/tests/EngineAllocationTests.cs`).

#### Engine-by-engine allocation, writing (step 2)

**Born closed**: option 8192 of `attitude_options_v1` (implies 4096; supported options 16383). On every physics step
where the pitch/yaw channel writes and the allocation's own solve of THIS step is sound, the allocation above WRITES in
place of the law's allocation and of the lateral-force layer:

* at Earlyish, every gimbal module of the allocation leased at its command (the loop's own authority and pivots, as the
  law's write); the ctrlState pitch / roll / yaw through the law's own fly-by-wire path (frozen axes of option 1 not
  written, the roll only when the loop writes it); the layer does not write (translation reason 20
  `served_by_allocation`; its return and published authority unchanged);
* at the fly-by-wire, after option 512's main throttle and before the layer: the throttle of every engine it drives,
  independent under the loop's lease, at `main + kappa (c - main)` (`main` the main throttle served this step, `kappa`
  keeping every engine within [0, 1]), and ctrlState `X` / `Y`.

Its solve when writing is the shadow's (horizon `H`) plus: the ctrlState axes the loop may not write held where they
are (option 1's frozen axes; the roll column and row when the loop does not write the roll), and ModuleRCS's DEAD ZONE
in the command -- an RCS-only channel (X / Y always, the rotation channels when no wheel or surface shares their
column) starts from what the RCS realizes of its served command (`0` inside `EPSILON`), and a command strictly inside
the zone is snapped to `0` (below `EPSILON / 2`) or `+-EPSILON`, held, and the allocation solved again so the other
actuators take the rest (`dead_zone_snapped`, allocation reason 4096). Every second step with a live demand it also
computes the free authority ALONG the demand (`free_demand_ms2`: the largest lateral acceleration along the demand's
direction with the axial force and the law's pitch/yaw torque held, plus the realized one along it), which the
client's final phase reads in place of the attitude-only authority while the allocation writes.

**Fallback without a jump**: a solve not solved, not converged, axial- or pitch/yaw-limited (a saturation), not of
this step, a lease refused at its last write, a gimbal the law drives missing from it, or an exception: the law and the
layer write that step, from the realized state. What they do not write themselves is handed back by decay (the
allocation's throttle offsets and `X` / `Y` written `exp(-1/H)` smaller each step, given back below `TargetTolerance`;
at once when the layer writes them). After a fallback the allocation writes again after `resume_steps` = ceil(`H`)
sound solves in a row. Releasing the authority, the contact hold, a missed fly-by-wire, option 8192 set or taken off:
every lease of its own given back at once.

The block's schema 2 appends after `ut` (the frames' schema 2, after the fields in their header): `write_state` (0 off
-- the option off or the channel not writing --, 1 written, 2 fallback), `fallback_reasons`
(`bits_v1("attitude_allocation.fallbacks")`: 1 not_solved, 2 not_converged, 4 saturated, 8 stale, 16 lease, 32
gimbal_missing, 64 exception, 128 resuming), `write_streak`, `resume_steps`, `write_count`, `fallback_count`,
`dead_zone_snapped`, `free_demand_ms2`, `free_demand_age_steps`, `fbw_kappa` (the last fly-by-wire's), `lease_failures`,
`decay_steps`, `missed_writes` (`write_state` 0 and `NaN` without option 8192). Law frames carry capability bit 24 on
the steps it wrote. The rule is `EngineAllocation.AllocationWriteRule` (pure; `build/tests/EngineAllocationWriteTests.cs`,
golden vectors `build/tests/golden/attitude_v1_golden_allocation.json`); its tests exercise the
write, a forced saturation, the decay and the resume.

#### The RCS as ModuleRCS fires it

**Born closed**: option 16384 of `attitude_options_v1` (supported options 32767). Without it nothing below runs and every
frame, column and command is the one of before, bit for bit; the law (`AttitudeMath.Step`), its B, its alpha and its
allocation are never touched, with or without it.

`ModuleRCS` (KSP 1.12.5, decompiled; `ModuleRCSFX` inherits both methods) fires every nozzle of a module on ONE rotation
vector, `inputRot = rotation x (pitch, roll, yaw)` (each component zero unless enabled and below `EPSILON`), along
`inputRot x unit(ProjectOnPlane(r, inputRot))`, bounded at 1, nothing at a fraction `<= EPSILON` (`CalculateThrust`):
a nozzle fires at `|inputRot|` times its alignment with the tangent of the rotation about the commanded axis through
the CoM. The torque of a combined command is NOT the sum of the per-axis columns the law (and an earlier change's in-flight torque)
use. On the heavy booster (ring of RCS ~23 radii above the CoM, coasts of five flights) a pure roll gives its column
(0.2486 rad/s^2 measured and predicted), pitch/yaw alone theirs, but a roll beside pitch/yaw above the dead zone gives
0.07 of its column and the pitch/yaw 1.5-1.7 times theirs: the law's pitch/yaw chattered at the period of two steps
whenever the roll commanded, its roll overshot.

With the option, at every step: the nozzles of every RCS module of the cache are read (the conditions of
`AddRcsColumns`; not in precision mode, which the rule does not carry); option 32's in-flight torque is the combined
rule's (`Torque(next) - Torque(fired)` per module); and at the roll's write the ctrlState command is the one, of the
law's command, the same with its roll held and the same with its pitch/yaw held (the latter only when the loop writes
pitch/yaw this step), whose torque by the combined rule (plus the linear rest of the ctrlState columns: wheels) is
nearest, in the law's own metric (`sum_a (Wv_a / I_a x error_a)^2`), the torque the law's INDI expects: the served
command's by the combined rule plus the increment through its columns. The cost of holding a part again is counted for
every step it has already waited (`1 + steps held in a row`: no axis starves); the law's command on a tie, then pitch/yaw
held. A held part is written just under the RCS dead zone (the smallest `EPSILON` of the modules that enable it, less
1e-5): the RCS ignores it, the wheels and the other ctrlState users still get it (capability bit 26: the roll held, 27:
the pitch/yaw held). Capability bit 25: the nozzles were modelled. `RcsCombined.cs` (pure; golden vectors
`build/tests/golden/attitude_v1_golden_rcs.json`), `AttitudeV1RcsCombined.cs`. A combination the RCS answers smoothly is
kept; one that jumps (the heavy booster's) is not. On a vessel whose roll is a wheel's and whose RCS cannot roll but
sprays pitch/yaw when a roll is combined (a radial quad) the RCS is kept for pitch/yaw and the
wheel gets the roll under the dead zone only: the option is for vessels whose RCS holds the roll.

#### Gimbal authority per kN

```text
gimbal_authority_v1(persistent_id) -> list[float]     # layout gimbal_authority_v1 (protocol 1, schema 1)
```

Every gimbal column is proportional to `finalThrust`: without thrust kRPC (`ModuleGimbal.GetPotentialTorque`,
`Engine.AvailableTorque`) and the law's own columns read nothing. The client's throttle of calage at the
flip needs the gimbals' authority BEFORE the thrust; it regressed 24 frames of the V3 ring
(25 RPCs per flip) and refused a module with several pivots, so the heavy booster served a profile. This
read gives it in one call, from the geometry AttitudeV1 already computes per pivot. Read-only; the
vessel need not be armed nor active.

Per gimbal module (a `ModuleGimbal` and the `ModuleEngines` of its part, AttitudeV1's discovery):
- one kN split over the part's engines by their `maxThrust` (equal shares when none has one), each engine's
  share over its nozzles by the stock multipliers;
- each nozzle under pivot `i` carried to the pivot's REST pose: from its world rotation now `R` to
  `Q = parent.rotation x initRots[i]` (what `ModuleGimbal` turns by `initRots[i] x` its two rotations), the
  nozzle's direction becomes `(Q R^-1) d` and its position `p + (Q R^-1)(n - p)`, `p` the pivot;
- per pivot, `d(r x F)/d(theta)` about its two rest axes (`xMult` right and `yMult` up or forward, as the lease
  rotates), the nozzle's offset from the pivot included (the formula of the attitude columns);
- a module whose pivots are not parallel (the AR-1 Cluster) is chained through its per-pivot rows, every
  component free: its free columns, what the law's alpha reads under `attitude_gimbal_pivots_v1` mode 1; its
  exact authority is an earlier change's largest realized torque over its box (`GimbalPivots.ExactAuthority`); its pivot rows
  are published. Any other module applies one `(x, y)` to every pivot in the pivot's own frame (the V1 path,
  every booster but the heavy one) and its exact authority is the alpha rule, `|column_x| span_x + |column_y|
  span_y` (a box is realized at its corners).

Header (21): `protocol, schema, persistent_id, found, ut, module_count, module_stride (31), pivot_count,
pivot_stride (12), max_thrust_kn` (summed `maxThrust` of EVERY engine of the vessel: the normalization of the
an earlier change), `gimbal_max_thrust_kn, authority_per_kn_x/y/z, exact_authority_per_kn_x/y/z` (sum over
modules of the module's authority per kN times its part's `maxThrust`, over `max_thrust_kn`: times the vessel's
available thrust in kN, the authority at full thrust), `com_from_reference_m_x/y/z` (the centre of mass from the
reference transform, body), `rest_pose` (1: every nozzle was carried to rest).

Module row (31): `part_flight_id, part_persistent_id, ordinal` (among the part's gimbal modules), `pivot_count,
mapped, usable` (enabled, active, not locked), `limiter` (0..1), `x_min_deg, x_max_deg, y_min_deg, y_max_deg`
(ranges times the limiter: the law's box), `max_thrust_kn` (the part's), `response_speed` (`gimbalResponseSpeed`,
NaN without it), `column_x_per_kn_x/y/z, column_y_per_kn_x/y/z` (N.m per degree per kN of the part, body frame,
about the current centre of mass), `authority_per_kn_x/y/z, exact_authority_per_kn_x/y/z, pivot_centroid_m_x/y/z`
(the pivots' mean position from the centre of mass, body), `rest_pose, engine_count, pivot_rows`.

Pivot row (12, mapped modules only): `module` (its module row), `pivot, map_xx, map_xy, map_yx, map_yy` (pivot
`i` turns by `(xx x + xy y, yx x + yy y)` for the module command `(x, y)`),
`column_x_per_kn_x/y/z, column_y_per_kn_x/y/z` (per degree of the pivot's own rotations, body, per kN of the
part).

The columns are those of the rest pose: under a deflection the law's columns differ by the cosine of it (under
1.7 % at the super hope's 10.5 deg) and turn with the nozzle. To verify in game: a ground check with the client (against the law's columns at a throttle in the clamps).

#### Effector registry (refactor R1, born closed)

```text
attitude_effectors_v1(token, mode) -> list[float]
attitude_effectors_read_v1(persistent_id) -> list[float]
```

**Born closed**: every arming resets it to 0, and with 0 every frame, return and write is the one
of the build before. `mode` 1 arms it for this arming, 0
disarms it; other modes are refused. The reply is `accepted (1), table protocol (1), mode now,
attitude tick`.

The registry gives every column of the allocation a **model status** and sanctions a family of
actuators only on its own columns, never the vessel (Simplex/RTA per actuator, audit C §3.3,
§4.2):

| Status | Meaning | Action |
|---|---|---|
| `0` exact | the column models what the actuator does: gimbals, RCS and reaction wheels outside the cases below, the differential motifs. RCS in **precision mode without `useLever`** is exact too: KSP multiplies each nozzle's thrust by `precisionFactor` after the same clamp, so the column is scaled by it | allocated |
| `1` measured | a scale measured in this flight (reserved; SurfaceModelV1's scale is a guard, not a measured column) | allocated |
| `2` approximate | an upper bound of the real effectiveness on its own axis: RCS in precision mode **with** `useLever` (thrust divided by the lever distance when above 1) | allocated (`action` 2, bounded), under the per-axis residual guard |
| `3` unmodeled | no model, or not a bound: control surfaces (`GetPotentialTorque` is a torque times a lever), RCS with `fullThrust`, and the ctrlState write path while the fly-by-wire order is not verified | the effector is neutralized while the loop writes an axis it answers (surfaces: `deflectionDirection = 0` for the physics step, as option 1); when it cannot be, **only the ctrlState axes it answers** are frozen (B = 0, bounds at the served u0, not written); all three for the write path |

The registry implies the mechanics of option 1 (frames carry capability bits 11 and 14, the
motifs 13, 14 and 20 are informational, the surfaces leave B and are neutralized), with these
decisions instead of option 1's "13 or 20 freezes the three ctrlState axes".

**Authority per family**, computed every step from the columns, never a threshold:
`A_f,a = Σ_j |B_a,j| · (u_max,j − u_min,j)/2` over the family's columns (0 for a frozen one), in
N·m on body axis `a`, and `α_f,a = A_f,a / I_a` in rad/s². The surfaces' is published from
`GetPotentialTorque` (dynamic pressure and Mach dependent, status unmodeled), for comparison only.

`attitude_effectors_read_v1(persistent_id)` returns the table of the last physics step, in one
layout whatever the mode (`attitude_effectors_v1`, protocol 1, schema 1): NaN and no column rows
while the registry is off or the vessel is not armed.

| Part | Fields |
|---|---|
| header (10) | `protocol`, `schema`, `enabled`, `attitude_tick`, `family_count` (5), `family_stride` (12), `column_count`, `column_stride` (10), `frozen_axes` (bit 0 pitch, 1 roll, 2 yaw), `surfaces_neutralized` |
| family row (12), families 0 `gimbal`, 1 `rcs`, 2 `wheel`, 3 `surface`, 4 `diff_motif` | `family`, `status` (−1 absent), `modules`, `scale` (the RCS precision factor applied, else 1), `authority_x/y/z_nm`, `alpha_x/y/z_rad_s2`, `columns`, `frozen_columns` |
| column row (10), one per allocation column | `column`, `kind` (1 gimbal, 3 ctrlState, 4 differential motif), `family`, `axis`, `status`, `action` (0 allocated, 1 frozen, 2 bounded), `b_x`, `b_y`, `b_z` (N·m per unit), `half_span` |


#### SurfaceModelV1: control surfaces as a modeled actuator (lot 3, 3.1, born closed)

```text
attitude_surfaces_v1(token, mode) -> list[float]
surface_model_v1(persistent_id) -> list[float]
```

**Born closed**: every arming resets it to 0, its response-lag evidence to none and its scale
guard to k = 1 with no evidence (mono-flight: nothing carries over to the next arming, let alone
the next flight); with 0 every frame, return and write is the one of the build before. Other modes are refused (`[UNSUPPORTED_MODE]`). The reply is
`accepted (1), protocol (1), mode now, surfaces answering a ctrlState axis, of which modeled,
attitude tick` (NaN counts while the vessel is not loaded).

**Units, the mode flag 4 (born closed).** The replica works in KSP's own force unit, the
**kilonewton**: the module's `Qlift`/`Qdrag` are in Pa, but `liftForce`/`dragForce` go to the
part's rigidbody, whose mass is in tonnes, as engine thrust (`finalThrust`, kN) does. Every
other torque of the law is converted to N.m (engines, RCS, wheels: × 1000). Up to `ad35d85` the
surfaces' realized torque, sub-columns, free torque, authority and residual input entered the
law — and the publication — in kN.m, **1000 times too small**: in the first armed flight
(a recorded flight) the measured angular acceleration is RCS + wheel + 870-1030 ×
the replica's fin torque, and the loop's roll, written in the coast once the fins were engaged,
fell into a saturated 3 Hz cycle as soon as the true-to-modeled ratio of the ctrlState column
passed 2 (q ≈ 7 kPa). Mode 1 or 2 **ORed with 4** (5 shadow, 6 armed) multiplies them by 1000 where
they enter the law (`SurfaceModel.LawTorqueScale`) and in the publication (forces in N, torques,
columns, free torque and authority in N.m); `mode` (reply and header) carries the flag. The
replica, its identity and its scale guard are unit-free and unchanged. Without the flag every
answer is the build before's, bit for bit (compared with the build before). A client must not arm mode 2 without it.

**The replica.** For every `ModuleControlSurface` of the vessel that is EXACTLY that class, not a
rotor blade (`displaceVelocity`), without node lift, with its protected members found, the plugin
reproduces what the module's own `FixedUpdate` does (KSP 1.12.5, decompiled): the command law of
`FixedCtrlSurfaceUpdate` (projection of `R_ref (pitch, 0, yaw)` on the hinge, sign flip when the
CoM is below the surface, the roll heuristic, `deflectionDirection`, `ctrlSurfaceRange`,
`authorityLimiter`, the deploy term, the clamps), the actuator (`MoveTowards`, or `Lerp` with
`useExponentialSpeed`, at `actuatorSpeed`), and the forces (`GetLiftVector` on the fixed part
`1 − ctrlSurfaceArea` and on the moving part turned by the deflection about the hinge,
`GetDragVector` likewise, `perpendicularOnly`; a surface with mission adjusters is not modeled). The module's
`deflectionLiftCoeff` and `ctrlSurfaceRange` are read live: KRE's `ModuleDeployableAero` sets them
to 0.0001 while the fins are folded.

**When.** At the end of every physics step (`BetterLateThanNever`, after every part's
`FixedUpdate`, before the physics moves anything) the plugin reads each module's own values of
the step (`deflection`, `action`, `nVel`, `Qlift`, `Qdrag`, `liftForce`, `dragForce`; Mach as the
module read it, kept at `Earlyish` because FlightIntegrator rewrites `part.machNumber` after the
parts) and computes:

- the **identity** of the replica: `(|L_model − L_real| + |D_model − D_real|) / (|L_real| + |D_real|)`,
  judged above 1 N; a surface off by more than 1e-3 for 5 judged steps is sanctioned for the
  arming (a mod or a patch that does not follow the stock module). The replica's moving part is
  turned or not (`rotated`) as the module has it: the module's own test
  `ctrlSurface.localRotation != neutral`, evaluated by Unity's own operator (`Quaternion.Dot`,
  a float32 method, then `> 0.999999f`; not turned below about 0.16°). An inline copy of that
  test, compared without the float rounding of `Dot`'s return, disagreed with the module in the
  half-ulp band above the threshold (3 one-step identities of 0.19-0.37 in flight
  a recorded flight);
- the **command law**: the module's `action` against the replica's target from this step's
  ctrlState and from the previous step's, and whether this step's surface answered this step's
  fly-by-wire (open point: the parts' `FixedUpdate` against `FlightInputHandler.FixedUpdate`). A
  surface held neutral answers nothing, so with the loop armed the fly-by-wire callback also notes
  each module's `liftForce`: equal to this step's value, the module ran before the callback;
  equal to the previous one while the force moved, after it;
- per surface, body frame: the realized torque about the CoM (lift at `CoLOffset`, drag at
  `CoPOffset`), the exact **column** dM/dδ (central differences ±0.1° of the same replica) and the
  **command gain** dδ_target/d(pitch, roll, yaw);
- the **scale guard** per body axis: `k = Σ ΔM_real ΔM_pred / Σ ΔM_pred²` (forgetting 0.995 per
  step), ΔM_pred the model's torque increment over the step (predicted deflection minus the
  previous one, at this step's flow), ΔM_real the realized torque minus the model at the previous
  deflection; each model torque in the state the module has at that deflection (the previous
  capture's `rotated`, and the module's test on the rotation it would set for the predicted one:
  an earlier change — always turned, a fin resting under the threshold added −column × δ to every ΔM_real; a
  surface the step leaves at the same float deflection keeps its state). **A step counts on an
  axis only when it is predicted to the replica's precision**: its uncertainty U, the sum
  over the surfaces of the replica's measured mismatch at the realized state (realized torque
  minus the model at the realized deflection and `rotated`), of |column| × the deflection's
  resolution (the module's float `action` against the replica's target, plus one float spacing
  of the deflections), and, for a surface that moves within Unity's **rotation band**, of its
  whole turned-state jump (the model turned minus not turned), must be below 1e-3 (the identity
  tolerance) of |ΔM_pred|. The rotation band is computed from Unity's test (`Dot > 0.999999f`) and
  the float spacing below 1 (2⁻²⁴): the exact flip is at 0.1607°, and four float roundings of
  unit quaternions (the stored neutral, `AngleAxis`, the product, the transform's own storage of
  `localRotation`) put it anywhere in **0.1399-0.1791°** — there the module's turned state is not
  a function of δ, and the bare rotation `AngleAxis(δ) * neutral` the guard can test need not
  answer as the rotation the transform stored (a recorded flight, k_y
  1.000 → 0.503 on the roll axis's only counted step, two fins at ±0.157°; every move of k above
  0.003 in that flight fell on steps with a fin inside the band). No excitation share, no floor:
  a weakly excited axis simply gathers no evidence (k = 1, `scale_steps` 0). The increments and U
  are rotated to the body frame in double. After 25 counted steps, k outside [0.5, 2] trips the
  axis for the arming; with 25 counted steps and no trip the axis is **proven** (k = 1
  without evidence is not a proof; the armed mode engages nothing on an axis before). It is a
  **guard**, never an adaptation: B is not scaled by k. The column
  stays the turned replica's (a secant over the usual increments): under the threshold the
  module's force does not move with δ, and steps by about column × 0.16° (0.5 % of a 35° fin's
  authority) when it is crossed. Diagnostic (KSP.log only, the first three of an arming): a
  surface captured inside the band whose bare rotation tests otherwise than the stored one
  (`SurfaceModelV1: part … at … deg: the stored rotation tests …`).

**Mode 1 (shadow)**: all of this is published; nothing enters the law, nothing is written. The
law's residual is published next to the residual with the surfaces' realized torque taken off
(same filter, same increment form).

**Mode 2 (armed)** implies the family mechanics of `attitude_options_v1` bit 1. A surface is
**engaged** in a step when the same-step response is established (25 answers at least, 20 times
more than every other outcome), no scale axis is tripped, its replica holds (no strike in the
last capture), it was captured in the previous step and is not shielded, and **every body
axis its column contributes to is proven** in this arming: the axes whose column component (last
capture) is above 1e-3 (the identity tolerance) of the column's norm — whatever the command, a
surface's torque increment lies along its column, and below that share it is within the
replica's own precision. A surface is engaged as a whole (`deflectionDirection` is per module and
it answers every ctrlState axis at once), so an axis without proof keeps out every surface that
would act on it — held neutral exactly as with bit 1 — while the others are engaged: a grid fin
off the roll axis (its lift across the radius) has a roll share and waits for the roll proof; a
flap whose lift stays along the radius (hinge across it) has none. The guard counts its steps
whether the surfaces are engaged or not (shadow, the slews of a held surface back to neutral,
the steps the AutoPilot flies them): the proof is gathered in the same arming before the
engagement, and a held surface at rest adds none. An engaged surface
leaves the neutralization and the roll gate of bit 1; its sub-columns (column × gain, one per
ctrlState axis) are ADDED to the ctrlState columns of the allocation (the surfaces read the same
ctrlState as the RCS and the wheels: they are sub-columns of that variable, with their own status,
guard and publication), its realized torque joins the law's M, and the torque it adds anyway
while slewing toward the target of the served command (M_free) is taken off the allocation
target (`AttitudeInput.UseTorqueFree`, born false). Every other surface is held neutral exactly as
with bit 1. Frames carry capability bit 15 (`surface_model`) and the informational motifs 37
(`surfaces_engaged`) and 38 (`surface_sanction`: a family condition holds surfaces back, armed
an axis not proven yet included); the effector registry, when armed, gives the engaged surfaces
the status exact and their static authority.

`surface_model_v1(persistent_id)` returns the last capture, one layout whatever the mode
(`surface_model_v1`, protocol 1, schema 1): NaN and no rows while the model is off or the vessel
not armed. SI units except the authority limiter (%) — with the mode flag 4 only: without it the
forces (`*_n_*`) are KSP's kN and the torques, columns and authority (`*_nm_*`, `column_nm_rad_*`)
kN.m.

| Part | Fields |
|---|---|
| header (50) | `protocol`, `schema`, `enabled`, `mode`, `attitude_tick`, `ut`, `physics_dt`, `vessel_persistent_id`, `surface_count`, `row_stride` (48), `modeled`, `engaged`, `neutralized`, `lag_same`, `lag_previous`, `lag_neither`, `lag_established`, `identity_max`, `identity_judged`, `sanctions` (bits 1/2/4 scale x/y/z, 8 lag, 16 identity, 32 not armed, 64/128/256 scale x/y/z not proven — fewer than 25 counted steps, armed only), `scale_k_x/y/z`, `scale_steps_x/y/z`, `torque_realized_nm_x/y/z`, `torque_free_nm_x/y/z`, `residual_law_rad_s2_x/y/z`, `residual_surfaces_rad_s2_x/y/z`, `residual_surfaces_f_rad_s2_x/y/z`, `authority_nm_x/y/z` (static, per unit of command), `ctrl_pitch`, `ctrl_roll`, `ctrl_yaw` (the captured step's), `earlyish_tick`, `probe_same`, `probe_previous` |
| row (48), one per control surface | `part_persistent_id`, `module_index`, `status` (0 exact, 3 unmodeled), `reasons` (bits 1 subclass, 2 rotor, 4 adjusters, 8 reflection, 16 no transform, 32 node lift, 64 shielded, 128 identity, 256 not captured), `axes`, `engaged`, `neutralized`, `shielded`, `deflection_rad`, `deflection_previous_rad`, `action_rad`, `target_same_rad`, `target_previous_rad`, `target_nominal_rad`, `predicted_same_rad`, `predicted_previous_rad`, `lag_outcome` (−1 not told, 0 same step, 1 previous step, 2 neither), `rate_rad_s`, `range_rad`, `authority_limiter_pct`, `deflection_direction`, `q_lift_pa`, `q_drag_pa`, `mach`, `rotated`, `force_realized_n_x/y/z`, `force_model_n_x/y/z`, `identity`, `torque_realized_nm_x/y/z`, `column_nm_rad_x/y/z`, `gain_pitch_rad`, `gain_roll_rad`, `gain_yaw_rad`, `identity_strikes`, `mach_source` (1 Earlyish, 2 after FlightIntegrator), `lift_coeff`, `area`, `drag_model`, `exponential`, `deploy_term_rad` |
| clock (1) | `master_tick` |

Validation, in three steps: the identity below 1e-3 (offline, then every flight in
shadow); the command law on the ground (impulses of ctrlState, `action` against `target_same`,
the slope at `actuatorSpeed`, which step answered); the column in shadow in flight (ΔM over
consecutive steps against the column, the scale guard at 1). To verify in game:
a ground check with the client.

**The physical slope κ, the mode flag 8 (born closed).** The identity and the scale guard
compare the replica with the module *in the same unit*: they could not see (k = 1.000 and
an identity of 2.6e-7 in the armed flight that took kN.m for N.m). κ asks the vessel itself:
per body axis, the slope of what the **measured** angular acceleration attributes to the
surfaces' deflections over the torque the law is given for them — 1 when the law's unit, levers
and frames are right, 1000 for an earlier change. Mode 1 or 2 ORed with 8 (with or without 4: 9, 10, 13, 14)
turns it on; `mode` carries the flag; 8 with mode 0 is refused. Per counted physics step, aligned
with the INDI (the same raw ω̇ and the same H(z) as the law, the same realized torques):

```text
y   = I (ω̇(t) − ω̇(t−1)) + Δ(ω_m × I ω_m) − ΔM_engines+RCS+wheels   (N.m, the law's realized torques)
x_δ = law scale × Σ over the modeled surfaces [T_i(t) − T_i(δ(t−1), flow(t))]   (law's unit)
x_n = 1000 × [ΔT_aero − Σ (the same deflection increments)]                     (N.m)
H(y) = κ H(x_δ) + c H(x_n) + e        (one regression per axis, H the law's low-pass, zero state)
```

`T_i` is a modeled surface's realized torque (its `liftForce` at the part's `CoLOffset`,
`dragForce` at `CoPOffset`), `T_i(δ(t−1), flow(t))` the replica at the previous deflection and
this step's flow (what the law's columns predict for the move), `T_aero` the whole vessel's
aerodynamic torque as the game applied it, read at the capture like kRPC's
`Flight.aerodynamic_torque` (every part's cube drag `−dragVectorDir × dragScalar` at its CoP, body
lift, every lifting surface's forces, the rigidbody angular drag), all body frame, double.
**Why these regressors** (shadow a recorded flight, 50 Hz frames, the captures
and the kRPC aerodynamic torque of `dynamics_v3`): the fins' torque alone, in levels, has a slope
of 2.5 (x) / 3.5 (z) on I ω̇ − M — the hull's aerodynamic torque is correlated 0.80-0.87 with
it (both follow q and the angle of attack, and the AutoPilot's deflections follow them); with
the hull's torque as a regressor the fins' slope is 0.89 / 0.975 and the hull's 1.01-1.02. In
filtered increments the fins' total torque alone still gives 1.9-2.0 (2.2-4.1 while the RCS
maneuvers in the coast); with the hull, 1.025 ± 0.036 (x), 0.973 ± 0.025 (y), 1.005 ± 0.007 (z).
The fins' own flow-driven increment is the hull's twin (folded fins, range 1e-4°, gave slopes of
−5e5), hence the deflection-driven regressor for κ and everything else aerodynamic as one
nuisance with a free slope `c` (published: the readout's own scale, ≈ 1).

**The gyroscopic torque.** The vessel obeys Euler's equation, I ω̇ = M − ω × I ω: in the
FLIP of the shadow a recorded flight (yaw rate up to 1.18 rad/s, roll rate
±0.05 rad/s) the pitch term (I_z − I_y) ω_y ω_z was 138 kN.m rms; regressing the measured I ω̇
on the realized RCS + wheel torque and the aerodynamic readout leaves 131 kN.m rms on x without
it and 3.6 kN.m with it (slope −0.998). Its increments followed the AutoPilot's pitch answers,
which the fins' deflections follow too: without the term κ_x read −0.69 after 4 batches and the
axis was declared out of tolerance for the whole arming. `y` adds the increment of the
gyroscopic torque at the step's mid rate ω_m = (ω(t) + ω(t−1)) / 2 (the rate the measured ω̇
spans; the flight's best fit), diagonal inertia, the law's own ω and I; a step needs the rates of
three consecutive ticks. Replayed offline on that flight's 50 Hz trace, κ_x is 1.000 ± 0.001.
The rms of its filtered increment is published (`rms_gyro_nm`).

What counts: a step whose ω̇ is valid (consecutive ticks, no warp, no topology change) and whose
capture of the previous step brought both increments, **once no engine torque has been in the
filter's memory for a batch** (the engines' realized torque is 10-100 times the fins' and its
error follows the same commands: its free slope was 1.06-1.13 in that flight), and, on an axis,
only while the modeled surfaces' static authority there is at least 1e-3 (the identity
tolerance) of the loop's whole authority on it (folded fins are 1e-8 of the RCS). The RCS and
the wheels stay at their realized torque.

Uncertainty and verdict, all computed: batches of `batch_steps` = ⌈ln(1000) / (ζ ω_c dt)⌉
counted steps (28 with the plan R2 gains: the time H(z) takes to forget an input to 1e-3);
κ's standard error is the **delete-one-batch jackknife** (exact from the batch sums; a batch that
alone carries the estimate makes its own deletion move κ); `tolerance` = t × SE, t Student's at
1 − 0.0005 with batches − 1 degrees of freedom (a decision is wrong once in a thousand). An axis
is **proven** while `[κ − t SE, κ + t SE]` lies inside the proof band [0.707, 1.414] (the scale
guard's band [0.5, 2] narrowed by a gain margin of √2, published as `band_min`/`band_max`
since schema 2). **Out of tolerance** — sticky for the arming, so that a real defect seen
once cannot hide behind later evidence — is judged on its own interval `[out_low, out_high]` =
κ ± t(ν) SE, entirely outside the band, with ν (`out_dof`) the jackknife's own degrees of freedom
: its Satterthwaite (Bell-McCaffrey) count for one regressor and independent batches,
ν = (Σ p/(1−p))² / (Σ p²(1−2p)/(1−p)⁴ + (Σ p²/(1−p)²)²), p each batch's share of the deflection
regressor partialled out of the nuisance — batches − 1 for equal shares, down to 1 as one batch
carries everything. In a recorded flight the trip rested on 1.3 effective batches (the grid fins'
deployment slew) judged with 3 degrees of freedom; with unequal weights the t(batches − 1) rule
is wrong 2-6 times more often than 1e-3 (Monte Carlo), t(ν) is not. A proof is re-judged at every
batch and engages only while it holds, inside the margin of it keeps t(batches − 1)
(with ν, the super hope's weakly excited roll — 2.7 effective batches over a whole flight — would
never be proven). Since ν ≤ batches − 1 the out interval contains the proof interval: an axis is
never both. Both need 3 batches. A
strict `|κ − 1| ≤ t SE` is published (`consistent_with_one`) as a diagnostic only: with 10⁴
steps it would reject the flight's own 1-3 % physics residue. Reset at every arming, whenever
the model is switched on with the flag or its unit flag changes (a shadow → armed change keeps
it). Armed with the flag, a surface is engaged only when, besides an earlier change, **every axis its column
contributes to has κ proven** (`sanctions` bits 512/1024/2048 while not, 4096/8192/16384 out of
tolerance — an engaged surface then returns to the neutralization, for the arming). Armed, the
surfaces not engaged are held neutral and give no deflection: **prove κ in shadow first** (mode
13), then arm (14). A guard, never an adaptation: nothing is scaled by κ. Cost: one pass over the
vessel's parts per capture (published, `aero_cost_us_*`), nothing allocated per step.

**The leak gate of the counted steps, the mode flag 16 (born closed).** The measured
side subtracts the engines', RCS' and wheels' torque at its realized value and multiplies by the
inertia: a proportional error of either (ε on the realized torque, δ on the inertia) is collinear
with whatever moves together with the surfaces' deflections and reads as a bias, not as noise:
κ̂ = κ (1 + δ) + (δ + ε) β, β = Σ d m / Σ d² over the counted steps (d the filtered deflection
regressor, m the filtered increment of the non-surface realized torque; an earlier change's formula, where β
was the column ratio r of a roll command). A command excites the surfaces AND the RCS and wheels;
where the surfaces are weak — the flip, the low q of the coast — |β| is large: (in the
emulator, x/z of the coast alone out of tolerance with 1 % of inertia error), (the roll proven
at the flip on steps the surfaces barely carry). Mode 1 or 2 ORed with 8 **and** 16 (25, 26, 29, 30;
16 without 8 is refused) counts a step on an axis only while `|m| ≤ LeakMax |d|`, `LeakMax` =
1 / GainMargin² = 1/2 — the surfaces carry at least two thirds of the excited increment, an earlier change's
threshold — so that `|β| ≤ 1/2` by construction. Computed on the **realized** increments (the RCS
dead zone, the wheels' lag, every module's actual share), not on the columns: the super hope's
RCS + wheel roll column is 122 kN.m per unit while a command inside the RCS dead zone moves the
wheels' 30 kN.m only. `d` is compared in physical N.m (×1000 without the unit flag 4): the gate
must not depend on the very unit κ judges, or an earlier change's unit would refuse nearly every step and delay
its own detection. Replayed on ten flights of the super hope (07/10-09/10, the emulator), the
errors on four of them: RCS +10 % moves κ_x by 0.009-0.066 without the gate, by 0.001-0.006 with
it; wheels +10 % moves κ_y by 0.006-0.036 without it, 0.002-0.008 with it; x and z are still proven in the flip on all ten flights,
the roll on eight; every mutant (×1.5, ×0.6, ×2.5, ×0.4) stays out of tolerance or
unproven. The inertia's own error is multiplicative (κ (1 + δ)) and is no leak: κ_y stays at
0.92-0.98 with the gate. Changing the flag starts a fresh estimator. The roll excitation of an earlier change
counts as counted only the roll the gate let through. Published per axis since schema 3, flag or
not: `leak_gate`, `leak_refused` (steps the gate refused), `leak_beta` (β of the counted steps,
physical).

**The physical inertia of the measured side, the mode flag 32 (born closed).** κ_y of the
super hope stayed at 0.967-0.981 with the leak gate (flights of 09/10 19h48 and 23h04, 10/10 09h29
and 12h11) while κ_x and κ_z read 0.998-1.001, and the nuisance slope of y — the game's own
aerodynamic torque, not the replica — read the same 0.968-0.979: a factor common to every roll
regressor, the left-hand side's. The inertia there is `Vessel.MOI` (`VesselPrecalculate.
CalculatePhysicsStats`, KSP 1.12.5, decompiled): the rotated rigidbody tensors plus
m (|r|² E − r rᵀ) with r = `rb.position` − CoM, the part's **origin**, whereas the physics has the
mass at `rb.worldCenterOfMass` = origin + R `CoMOffset` (`FlightIntegrator.UpdateMassStats` sets
`rb.centerOfMass = part.CoMOffset` at every step; the CoM itself is summed from
`worldCenterOfMass`). The super hope's four KRE titanium grid fins (0.7 t, `CoMOffset` 0.6 m along
the radius: 1.81 → 2.41 m) and four structural wings (0.1 t, `CoMOffset` (−0.7, −0.6, 0):
1.90 → 2.60 m) leave 8 349 kg.m² of roll inertia out of `Vessel.MOI` (6 234 on pitch and yaw,
0.07 % of theirs): I_y is served 2.6 % low at the flip, 3.3 % in the coast. Identified on the same
four flights, one at a time, from the realized torques alone (I ω̇ + ω × Iω regressed on the
wheels', RCS', engines' and the aerodynamic readout's torques over the flip and the boostback):
I_true / I_served 1.022-1.027 on roll, 0.998-1.009 on pitch and yaw. Mode 1 or 2 ORed with 8
**and** 32 (41, 42, 45, 46, 57, 58, 61, 62; 32 without 8 is refused) multiplies the measured side
by `Vessel.MOI` plus those parallel-axis terms, computed at every attitude tick of κ from the
vessel's parts (`SurfaceKappa.AddInertiaOffset`: m [(|c|² − c_a²) − (|o|² − o_a²)], c and o the
mass centre and the origin about the CoM in the reference frame, `rb.mass` as the stock sum). The
law's own inertia is untouched. Changing the flag starts a fresh estimator. Published per axis
since schema 4, flag or not: `inertia_physical` (1 with the flag), `inertia_offset_kg_m2` (the
terms `Vessel.MOI` leaves out on the axis).

`surface_kappa_v1(persistent_id)` (`surface_kappa_v1`, protocol 1, schema 4; NaN while the flag
is off; schema 3, d190-dba3ad0: 30 values per axis; schema 2, d171-d188: 27 values per axis; schema 1, up to 6015f9c: 22 values per axis and
`band_min`/`band_max` = [0.5, 2], the scale band, while the decision used the proof band — an earlier change):

| Part | Fields |
|---|---|
| header (26) | `protocol`, `schema`, `enabled`, `mode`, `attitude_tick` (of the last step), `ut`, `vessel_persistent_id`, `batch_steps` (doubled at each merge of the 512-batch store), `batch_steps_base`, `batch_capacity`, `false_alarm` (1e-3), `band_min`, `band_max` (the proof band, [0.707, 1.414]), `steps_seen`, `steps_counted`, `restarts` (filter restarts), `engine_quiet_steps`, `aero_parts`, `aero_lifting_modules`, `aero_cost_us_last`, `aero_cost_us_max`, `axes_now` (bits: the surfaces' authority counts on that axis this step), `proven_axes`, `out_axes`, `axis_count` (3), `axis_stride` |
| axes (32), one per body axis | `axis`, `state` (0 not proven, 1 proven, 2 out of tolerance), `proven`, `out`, `consistent_with_one`, `kappa`, `kappa_se`, `t_quantile`, `tolerance`, `kappa_low`, `kappa_high`, `z_from_one`, `nuisance_slope`, `nuisance_se`, `regressors`, `steps`, `batches`, `effective_batches` (Kish, on the deflection regressor), `rms_y_nm`, `rms_fins_nm` (the law's unit), `rms_nuisance_nm`, `rms_residual_nm`, then (schema 2) `out_dof` (ν), `out_t_quantile`, `out_low`, `out_high` (the out-of-tolerance interval), `rms_gyro_nm` (the filtered gyroscopic increment `y` includes), then (schema 3) `leak_gate` (1 with the mode flag 16), `leak_refused` (steps the gate refused), `leak_beta` (Σ d m / Σ d² of the counted steps, physical), then (schema 4) `inertia_physical` (1 with the mode flag 32), `inertia_offset_kg_m2` (what `Vessel.MOI` leaves out on the axis, kg.m²) |
| clock (1) | `master_tick` |

### Engagement only on the axes the loop owns

Armed, a surface is engaged only when the loop **owns every axis it needs**: each body axis its
column contributes to (`SurfaceModel.ContributedAxes`, an earlier change's) and each ctrlState axis its
command answers (`CommandedAxes`: the linear coefficients of its command law, pitch, roll, yaw,
above 1e-3 of their norm) — bit 0 pitch x, 1 roll y, 2 yaw z for both. An axis is owned when the
loop's own fly-by-wire link **wrote it at the previous physics step** and it is still in the arm
mask and the client's write mask now (`AttitudeWrite.LoopOwnedAxes`; none while that mask
carries bit 4 under option 16, `AttitudeWrite.SurfaceOwnedAxes`); `sanctions` bits
32768/65536/131072 say which are not (armed only). The previous step's writes are the only ones
known in time: the engagement feeds the law (its columns, its M, M_free), and this step's write
decision comes after the law. A surface is so never engaged before the loop has written its axes
— one written step of proof, during which option 1 holds it neutral if the loop writes pitch/yaw
— and a write the loop refuses at the same step (law not computed, an axis guard, contact,
authority) costs that one physics step at most: the surface moves at most `actuatorSpeed × dt`
toward the other regulator's command, then it is out from the next step.

Why (a recorded flight): armed with κ proven, the four grid fins of the super hope
were engaged through the whole coast while the kRPC AutoPilot wrote pitch/yaw and the loop only
the roll. The engagement lifted the roll gate of option 1 (the loop wrote the roll in the coast)
and the law counted as its own actuator a torque the AutoPilot commanded. Out, a surface is stock
again — it follows the served ctrlState, the roll gate and the neutralization of option 1 apply —
so the coast is the shadow's. In the descent, where the loop writes the three axes, the fins
engage one step after the loop's first write.

At an engagement or a disengagement the law's M changes **definition** (the surface's realized
torque, which acted all along, enters or leaves it): `AttitudeLawState.ShiftTorque` moves the
torque filter's memory and the previous filtered torque by the change (H(z) has a unit DC gain), so
the allocation target M_d − M and the INDI residual take no step; the published residual's filter
(the surfaces outside M) moves by the opposite change. Without it the target would jump by minus
the surfaces' torque (0.04-0.25 MN.m for the super hope's fins at the PDI of the shadow flights)
and relax over H(z)'s time.

### Roll excitation of κ (born closed)

Every grid fin of the super hope has a roll column at 12 % of its norm, so armed with the κ flag a
fin is engaged only once κ is proven on roll too. Roll is the axis the trajectory excites least:
in the coast the AutoPilot damps the roll rate (`target_roll` NaN) and its roll command stays at
1e-5..1e-4. a recorded flight never proved it (κ_y 0.908 ± 0.086, all from the
flip), `_191445` did only because its flip excited the roll twice as much.

`attitude_surfaces_roll_v1(token, ut_until)` opens (or with a past time closes) the window until
the game time `ut_until` (finite, else refused). While it is open, the model is in shadow with the
κ flag (mode 13), the loop owns its fly-by-wire link (roll armed) with the per-family rules (option
1), it does not write the roll nor hold the surfaces neutral this step, roll is neither proven nor
any axis out of tolerance, the estimator counted roll this step, and the leak ratio
`|B_y| / |F_y|` (the wheels' and RCS' roll torque per unit of `ctrlState.roll` over the surfaces',
physical N.m) is at most `leak_max` = 1/GainMargin² = 1/2, the loop adds to the AutoPilot's
`ctrlState.roll`, in its fly-by-wire link, a square wave ±A switching at the start and the middle of
every κ batch (zero mean per batch). It stops at once (that step) when any of these fails; an
engine torque stops the estimator's counting, hence the excitation.

Computed, nothing tuned (`Model/SurfaceKappaRoll.cs`): the **leak gate** keeps the bias a
proportional error of the non-surface torque (ε) or of the inertia (δ) makes on κ —
κ̂ = κ (1 + δ) + (δ + ε) r, collinear with the regressor, not noise — within the inertia's own
error; the **plan** is the information still missing for the roll interval to fit in the proof
band with the quantile in force (`D_now max(ρ² − 1, GainMargin² − 1)`, ρ = t SE / margin), spread
over `horizon_batches` N_h = the smallest K with t(K − 1) ≤ GainMargin · t(511) (11) and held N_h
batches; the **bound** is the slowest surface's travel in one physics step (`2 |∂target/∂roll| A ≤
actuatorSpeed dt`: every switch answered within the step) and the command headroom. Reset with the
estimator, at every arming.

Reply (`attitude_surfaces_roll_v1`, protocol 1, schema 1, 26 values): `accepted`, `protocol`,
`schema`, `state` (0 off, 1 waiting, 2 exciting, 3 roll proven, 4 out of tolerance), `reasons`
(bits: 1 no window, 2 not shadow with κ, 4 loop without its link, 8 option 1 off, 16 the loop writes
the roll or holds the surfaces, 32 roll proven, 64 an axis out of tolerance, 128 roll not counted,
256 no surface answers the roll, 512 exponential actuator, 1024 leak ratio above `leak_max`),
`until`, `ut`, `attitude_tick` (of the last decision), `command` (added at that step),
`amplitude`, `amplitude_torque_nm` (the law's unit), `plan_torque_nm` (NaN: the bound),
`horizon_batches`, `batch_information` (one batch of a unit square wave through H(z)),
`surface_per_unit_nm` (F_y, the law's unit), `non_surface_per_unit_nm` (B_y, N.m), `leak_ratio`,
`leak_max`, `steps`, `episodes`, `plans`, `writes`, `last_written`, `window_calls`, `batch_seconds`
and `plan_seconds` (the estimator's batch and N_h of them, game seconds: the client closes its window
a plan, a batch, the window's lifetime and the client's ignition guard before the predicted ignition).

---


## `conn.telemetry`

What observes (refactor R3, `KRPC.Bridge.Telemetry.dll`): the physics-rate state of the vessels
and of their actuators, read-only towards the game. Its captures run inside Control's pump (one
physics driver, `ControlWatcher`), at the point of the step where they always ran. Until R3
these members were on `conn.actuators`, with the same names and arguments.

| Member | Returns | Meaning |
|---|---|---|
| `available` | `bool` | Always `True` when the service loaded. |
| `ping()` | `str` | Returns `"pong"`. |
| `engine_sample()` | `list[float]` | Rows of 9 values: part `flight_id`, engine ordinal, ignited, realized throttle, realized thrust, max thrust, thrust limit, independent mode, independent percentage. |
| `gimbal_sample()` | `list[float]` | Rows of 8 values: part `flight_id`, gimbal ordinal, locked, limiter, range, measured local X/Y/Z actuation in degrees. During a lease, response-speed limiting can make this lag the target. |
| `thrust_direction_sample()` | `list[float]` | Rows of 5 values: part `flight_id`, engine ordinal, then unit direction X/Y/Z in the vessel frame (right, forward, bottom). Reads each current `ModuleEngines.thrustTransforms`, so it remains valid after a revert even when stock kRPC's cached `Thruster` wrapper is stale. |
| `thrust_position_sample()` | `list[float]` | Rows of 5 values: part `flight_id`, engine ordinal, then the thrust-weighted lever X/Y/Z from the current centre of mass in the vessel frame. Prefer the per-transform v2 rows for exact multi-nozzle geometry. |
| `dynamics_protocol_version` | `int` | `3`, the protocol of the dynamics frames below. |
| `dynamics_snapshot_v3()`, `dynamics_frames_v3(since_tick=0, max_frames=64)`, `dynamics_status_v3()` | `list[float]` | Read-only FixedUpdate dynamics of the ACTIVE vessel, [below](#dynamics-protocol-v3). |
| `aero_actuator_snapshot_v1()`, `aero_actuator_frames_v1(since_tick=0, max_frames=64)`, `aero_actuator_status_v1()` | `list[float]` | Same-FixedUpdate state of the active vessel's aero surfaces and deployment animations (header protocol, schema, tick, UT, row count, stride `9`; rows `part_flight_id, module_kind, module_ordinal, deploy_bool, deploy_fraction, authority_limiter, deploy_angle_deg, animation_open_hint, enabled_hint`). The realized deflection angle is in DynamicsExtV1. |
| `dynamics_ext_enable_v1(enabled, lease_seconds=5)` | `list[float]` | Arms (or disarms) the DynamicsExtV1 capture for 0.5–60 s of real time; returns its status. Born closed. |
| `dynamics_ext_snapshot_v1()`, `dynamics_ext_frames_v1(since_tick=0, max_frames=64)`, `dynamics_ext_status_v1()` | `list[float]` | Same-tick supplemental frames of the active vessel, [below](#dynamicsextv1-same-tick-supplement). |
| `track_vessel_v3(persistent_id, lease_seconds=5)` / `untrack_vessel_v3(persistent_id)` | `bool` | Starts/renews (0.5–60 s) or stops the capture of a loaded vessel by `persistentId` (decimal string), active or not. At most four. |
| `tracked_vessels_v3()` | `list[float]` | The tracked-vessel registry. |
| `tracked_dynamics_snapshot_v3(persistent_id)`, `tracked_dynamics_frames_v3(persistent_id, since_tick=0, max_frames=64)`, `tracked_dynamics_ext_frames_v1(persistent_id, since_tick=0, max_frames=64)` | `list[float]` | v3 / DynamicsExtV1 frames of one tracked vessel, [below](#tracked-vessels-by-persistentid). |
| `simulate_aerodynamic_wrench_batch_v1(states, non_rotating_frame=True, persistent_id="")` | `list[float]` | Up to 32 `Flight.simulate_aerodynamic_wrench_at` evaluations in one call, [below](#batched-aerodynamic-wrench). Read-only. |
| `dynamics_reflection_check_v1()` | `list[float]` | QW6 identity check: the active vessel's v3 frame and aero-actuator rows captured through the legacy and the cached reflection readers, compared bit for bit, [below](#reflection-cache-qw6). Read-only. |
| `contact_v1(persistent_id)` | `list[float]` | The contact of a vessel with the ground by leg and by part, and the master tick of each first contact (refactor lot 3), [below](#contact-by-leg-contact_v1-refactor-lot-3). `""`: the active vessel. Read-only. |
| `legs_geometry_v1(persistent_id)` | `list[float]` | where the feet of the legs are and what they take, one row per wheel module in the order of `contact_v1`, [below](#legs-geometry-legs_geometry_v1-d237-k1). `""`: the active vessel. Read-only. |

### Contact by leg (`contact_v1`, refactor lot 3)

`Landed` says THAT a vessel touched, at a rendered frame, after the fact. `contact_v1` says WHEN,
to the physics step, per leg: the master tick of the first contact of each leg and of the vessel,
of every leg on the ground, of `Vessel.Landed` and of the LANDED situation, latched by a watcher
that runs at **every physics step** inside the actuator pump, right after the V3 capture. A
client that reads at its own rate (50 Hz at best, with holes) still learns the exact step: the
ticks are the `master_tick` of the V3 frames, so the two join on it.

**Which vessels.** The active vessel at every step. Another loaded vessel from the first
`contact_v1` that names it, and for 3000 master ticks (60 s at 50 Hz) after each one: a double
flight reads its non-active booster from the start. At most 8 vessels; nothing survives the
flight scene. A packed vessel is not stepped (its latches wait). A vessel that is not loaded
answers `found` 0 and no row: this procedure never throws for that (a malformed id is a
`ValueError`).

**Definitions.** The vessel is **in contact** when a wheel is grounded (`WheelState.grounded`,
the wheel collider's ground hit) or a part has `Part.GroundContact` or `PermanentGroundContact`
(the flags `Vessel.checkLanded` reads). A collision with another vessel alone (a tower's arms,
debris) is not a contact: it has its own latch, `collision_tick`. An **episode** starts at a step
in contact after a step without: the per-leg first ticks and `all_legs_tick` belong to the current
episode and restart at the next one (a bounce); `first_episode_tick` keeps the very first. Every
other `*_tick` is the LAST rising edge seen. `-1`: none seen by this watch (a watch started on the
pad sees no rising edge until it has flown).

**Timing.** Unity orders the pump and the wheels' own `FixedUpdate` freely: a wheel grounded by
the physics of step N is seen at step N or N+1; `Part.GroundContact` set by a collision callback
after the physics of step N is seen at N+1. One step of jitter, the same for every leg.
`Vessel.Landed` is set inside the wheels' `FixedUpdate` (`ModuleWheelBase.LandingDetectionUpdate`
→ `Vessel.checkLanded`); the **situation** changes in `Vessel.LateUpdate` (once per rendered
frame), and that is what kRPC's `vessel.situation` reports.

Layout `contact_v1` (protocol 1, schema 1; `conn.bridge.layout_v1("contact_v1")`): a header of
45 doubles, then `wheel_count` rows of `wheel_stride` doubles, one per wheel module
(`ModuleWheelBase`: legs and wheels), in part order. Live fields are read when the call runs;
latched fields come from the watcher's last step (`last_step_tick`).

| Header field | Meaning |
|---|---|
| `protocol`, `schema`, `persistent_id`, `found` | 1, 1, the vessel, 1 when it is loaded. |
| `tick`, `ut` | Master tick and UT of the call. |
| `watched`, `watch_start_tick`, `last_step_tick` | 1 once the watcher has stepped it; its first and last step. |
| `started_in_contact`, `contact_now` | The contact at the watch's first step and at its last (NaN before the first). |
| `landed`, `splashed`, `situation` | `Vessel.Landed`, `Splashed`, the situation code (`Vessel.Situations` flags: 1 LANDED, 2 SPLASHED, 4 PRELAUNCH, 8 FLYING...), now. |
| `parts_ground_contact`, `parts_permanent_contact`, `parts_colliding` | Parts with `GroundContact` or `PermanentGroundContact`, with `PermanentGroundContact`, with a current collision, now. |
| `wheels_grounded`, `legs`, `legs_deployed` | Now. A leg is a `ModuleWheelBase` of type `LEG`; deployed when `ModuleWheelDeployment.position` is within 0.01 of `deployedPosition` (KSP's own test), or when it has no deployment. |
| `episodes`, `first_episode_tick` | Rising edges of the contact seen, and the first. |
| `airborne_since_tick`, `lost_tick` | The last time the vessel left the ground (or the watch start when it began airborne), and the last falling edge. |
| `first_contact_tick`, `first_contact_ut` | The current episode's first contact. |
| `first_contact_source` | Bits (`bits_v1("contact_v1.sources")`): 1 a wheel grounded, 2 a part without a wheel in ground contact (the body), 4 a wheel part's own `GroundContact` flag. |
| `first_contact_part_pid` | The part that touched first (a wheel's part when a wheel did). |
| `first_contact_vertical_speed_ms`, `first_contact_horizontal_speed_ms`, `first_contact_tilt_rad` | `Vessel.verticalSpeed` (up +), `horizontalSrfSpeed`, and the angle between the nose and the local vertical, at that step. |
| `all_legs_tick` | Every deployed leg grounded, in the current episode. |
| `landed_tick`, `situation_landed_tick`, `collision_tick` | Last rising edge of `Vessel.Landed`, of the situation LANDED or SPLASHED, of any part colliding. |
| `height_from_terrain_m`, `terrain_normal_body_x/y/z`, `terrain_tilt_rad` | KSP's own per-step raycast (`Vessel.UpdatePosVel` → `GetHeightFromTerrain`): from the vessel transform down to layer 15 (Local Scenery) only, `-1` without a hit — over a barge (parts, layer 0) it is the sea floor or nothing; the normal in the body frame and its angle to the local vertical (NaN without a hit). |
| `tilt_rad`, `vertical_speed_ms`, `horizontal_speed_ms` | Now. |
| `wheel_count`, `wheel_stride` | The rows that follow (stride 25). |

| Wheel row field | Meaning |
|---|---|
| `part_persistent_id`, `part_flight_id`, `module_ordinal` | The wheel's part and its rank among the part's wheel modules. |
| `wheel_type` | 0 FREE, 1 MOTORIZED, 2 LEG. |
| `deploy_position`, `deployed_position`, `deployed` | `ModuleWheelDeployment.position`, `deployedPosition` (NaN without a deployment module), and the verdict. |
| `grounded`, `module_grounded`, `part_ground_contact` | `WheelState.grounded` (NaN before the controller is initialised), `ModuleWheelBase.isGrounded` (what its last `FixedUpdate` copied), `Part.GroundContact`. |
| `downforce_n`, `suspension_force_n` | `WheelState.downforce` (the suspension force plus extra, times the cosine of the contact angle, floored at the module's minimum) and `hit.force` (the suspension force alone), KSP's kN × 1000. On level ground at rest, the legs' `suspension_force_n` should add up to the weight. |
| `suspension_compression`, `contact_depth_m`, `compression_rate_ms` | `contactDepth / suspension travel` (a fraction), `contactDepth`, and `WheelState.contactSpeed` = d`contactDepth`/dt: the rate of compression, NOT the touchdown speed (use the vessel's vertical speed for that). |
| `normalized_load` | `WheelState.normalizedLoad`. |
| `hit_normal_body_x/y/z`, `hit_layer` | The ground hit's normal in the body frame and its collider's layer (15 terrain, 0 a part: a barge, a tower), NaN and -1 when not grounded. |
| `first_grounded_tick`, `first_grounded_ut`, `first_grounded_compression_rate_ms` | The wheel's first ground in the current episode. |
| `grounded_transitions`, `last_change_tick` | Edges of `grounded` since the watch started, and the last one. |

The transitions are also published on the event bus V2 (`contact.first`, `contact.leg`,
`contact.all_legs`, `contact.landed`, `contact.situation_landed`, `contact.collision`,
`contact.lost`): a client that polls `poll_events_v2` once per tick learns them without calling
`contact_v1` every tick.

**Cost.** Per watched vessel and physics step: one pass over its parts (three flags and a
collision count each) and over its wheels, no allocation (the wheel list is rescanned into
preallocated lists when the vessel object or its part count changes, and every 250 steps). The
call itself builds one list of 45 + 25 per wheel doubles.

```python
c = conn.telemetry.contact_v1(pid)
names = [row.split("\t")[2] for row in conn.bridge.layout_v1("contact_v1")
         if row.startswith("header\t")]
head = dict(zip(names, c[:45]))
if head["first_contact_tick"] >= 0:
    print("first contact at tick", int(head["first_contact_tick"]))
```

### Legs geometry (`legs_geometry_v1`)

the client's datum (the height of the centre of mass above the feet: D's last metres, the contact cut) comes
from a model of the craft, wrong by 0.45-0.88 m on the super hope (a study of 14/10), hence a zone C of
3-5 s. The legs' colliders give it. Read-only; layout `legs_geometry_v1` (protocol 1, schema 1).

KSP (Assembly-CSharp 1.12.5, decompiled): `ModuleWheelBase.Wheel` is a `KSPWheelController` (null before its
setup) whose `wheelCollider` (`VehiclePhysics.VPWheelCollider`) carries `center, radius, suspensionDistance,
suspensionAnchor, springRate, damperRate` and a `cachedTransform`; `ModuleWheelSuspension.OnWheelSetup` copies
its `suspensionDistance x rescaleFactor` and `targetPosition` into the collider's `suspensionDistance` and
`suspensionAnchor`. `VPWheelCollider.GetContactDepth(point, travel)` = `travel + radius - (depth of the point
below the transform's origin + suspensionDistance (1 - suspensionAnchor))` with `travel = suspensionDistance`:
0 for a point `radius + suspensionDistance x suspensionAnchor` below the origin along `-up`, the leg just
touching at full extension. `KSPWheelController.WheelCenter` = `cachedTransform.TransformPoint(center + up
suspensionDistance (1 - suspensionAnchor)) + WheelDown x contactDepth`, `WheelDown` = `-cachedTransform.up`,
`WheelRadius` = the collider's radius. The submodules (`ModuleWheelSuspension`, `ModuleWheelDamage`,
`ModuleWheelDeployment`) name their `ModuleWheelBase` by `baseModuleIndex` (its index among the part's modules).

Header (14): `protocol, schema, persistent_id, found, ut, wheel_count, wheel_stride (42), legs, legs_deployed,
wheels_grounded, foot_extended_below_com_m` (the deepest deployed leg's foot at full extension below the centre
of mass along the nose, `-y` body), `wheel_lowest_below_com_m` (the same with the wheel's lowest point now),
`hit_below_com_m` (the mean live contact point of the grounded wheels), `height_from_terrain_m`.

Wheel row (42), in the order of `contact_v1`: `part_persistent_id, part_flight_id, module_ordinal, wheel_type,
deployed, deploy_position, collider_ready, collider_origin_m_x/y/z` (body, from the centre of mass),
`down_x/y/z, collider_center_local_m_x/y/z, radius_m, suspension_distance_m, suspension_anchor, spring_rate,
damper_rate, foot_extended_m_x/y/z` (origin `- up (radius + suspension_distance x suspension_anchor)`),
`wheel_lowest_now_m_x/y/z` (`WheelCenter + WheelDown x WheelRadius`), `grounded, hit_point_m_x/y/z`
(`WheelState.hit.point`), `contact_depth_m, suspension_distance_config_m, spring_ratio, damper_ratio,
maximum_load, target_position` (the suspension module's KSPFields), `impact_tolerance, stress_tolerance, damaged`
(the damage module's), `part_crash_tolerance_ms, module_radius_m`. NaN for what the part has not (no collider
yet, no suspension or damage module, not grounded).

Which foot a touchdown datum reads is for the ground check: standing on its
legs, the live contact point must sit on `wheel_lowest_now`, and `foot_extended` contact depth below it if
`contactDepth` is the compression from full extension.

### Dynamics protocol v3

`dynamics_snapshot_v3()` is a **read-only FixedUpdate snapshot** designed for a descent-guidance client and
for the common BB/flip/coast system-identification path. Unlike `control_snapshot_v2()`,
it is captured automatically at the end of every `ControlWatcher.FixedUpdate` (the actuator pump), after
any queued actuator frame and leased-gimbal update for that callback. The next frames
therefore show the physical response to a known `last_applied_sequence` /
`last_applied_tick` pair.

The bridge keeps the latest 1000 frames (about twenty seconds at 50 Hz). Use
`dynamics_frames_v3(since_tick, max_frames)` to retrieve the ring without relying on the
Python/RPC polling cadence. `max_frames` is clamped to 1–128. A topology or active-vessel
change clears the ring rather than mixing incompatible actuator row layouts.

This stream always follows `FlightGlobals.ActiveVessel`. With two boosters in flight, the
process flying the non-active one must use a [tracked vessel](#tracked-vessels-by-persistentid)
instead, or it reads the other booster.

#### Wire compatibility

The protocol remains **3**. Schema **2** is an append-only extension of schema 1:

- the 22-value header is unchanged;
- the first 67 state values are byte-for-byte the schema-1 layout;
- engine/gimbal/thrust-transform row formats are unchanged;
- schema 2 appends 49 canonical SI dynamics values and raises state stride to `116`.

`python/dynamics_v3.py` decodes schemas 1, 2 and 3, so historical V3.1 logs remain usable. Schema **3** (refactor R3) appends, after the last thrust-transform
row, the one-value clock section (`master_tick`); the header, the state and the rows are
schema 2's.

The snapshot starts with a 22-double header:

| Offset | Meaning |
|---:|---|
| 0–1 | protocol (`3`) and schema (`3` since refactor R3: the frame then ends with the [clock section](#one-tick-in-every-frame-refactor-r3); `2` before) |
| 2–6 | physics tick, UT, fixed delta time, vessel persistent id, topology generation |
| 7–10 | last accepted sequence, last applied sequence, last applied tick, actuator result |
| 11 | capability bit mask; absent quantities are `NaN`, never plausible zeros |
| 12 | state stride (`116` in schema 2; `67` in schema 1) |
| 13–14 | engine count / stride (`20`) |
| 15–16 | gimbal count / stride (`19`) |
| 17–18 | thrust-transform count / stride (`10`) |
| 19–20 | history capacity and number of frames present before this capture |
| 21 | capture phase (`0` = pre-integration state sampled from the FixedUpdate callback) |

#### Schema-1 prefix: offsets 0–66 of the state block

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

These values preserve the V3.1 native-KSP contract. `mass_tonnes` is tonnes and engine
`final_thrust_kn` / `max_thrust_kn` are kN, retaining the convenient
`kN / tonne = m/s²` relationship. The `_raw` aerodynamic fields remain best-effort
reflection probes and should not be used by new flight-critical code.

**Two names in this prefix do not mean what they say** (verified 2026-09-19 against the
decompiled KSP 1.12.5 `VesselPrecalculate.CalculatePhysicsStats` and three flights). The
names are kept for wire and log compatibility; read them as follows:

| Field | Actual frame and unit |
|---|---|
| `angular_velocity_world_xyz` | **Vessel BODY frame** (right, forward, bottom), rad/s. KSP's `Vessel.angularVelocity` is the mass-weighted mean of `Inverse(ReferenceTransform.rotation) * rb.angularVelocity`. On three flights the published quaternion's derivative matches it with a 0.2–0.5 % median residual in the body frame, against more than 100 % in the world frame. Rotate by `rotation_world_xyzw` to get Unity world. |
| `angular_acceleration_derived_xyz` | Tick difference of the above, so body frame too (rad/s²). Because ω×ω = 0 it equals the world angular acceleration expressed in the body frame. |
| `moi_native_xyz` | **Diagonal** of the inertia tensor about the CoM, body frame, in **tonne·m²** (built from `rb.mass`, in tonnes). kRPC's `Vessel.moment_of_inertia` is exactly this × 1000 (kg·m²); `Vessel.inertia_tensor` is the full tensor × 1000 in the same frame. |

Beware of the obvious cross-check: kRPC's `vessel.angular_velocity(vessel.reference_frame)`
is **always zero**, because it returns `frame.Rotation⁻¹·(ω_root − ω_frame)` and the vessel
frame rotates with the vessel. Use `vessel.angular_velocity(body.non_rotating_reference_frame)`
and bring it to the vessel frame with `vessel.rotation(body.non_rotating_reference_frame)`.

#### Schema-2 extension: offsets 67–115 of the state block

All forces/torques in this block are canonical SI quantities. The body frame is the
vessel frame used elsewhere by the bridge: **X right, Y forward, Z bottom**.

| State offset(s) | Field | Unit / notes |
|---:|---|---|
| 67–69 | `engine_force_body` | N |
| 70–72 | `engine_force_world` | N |
| 73–75 | `engine_torque_body` | N·m about current CoM |
| 76–78 | `engine_torque_world` | N·m about current CoM |
| 79–81 | `aero_force_body` | N, official kRPC live `Flight.AerodynamicForce` |
| 82–84 | `aero_force_world` | N |
| 85–87 | `aero_torque_body` | N·m, official kRPC live `Flight.AerodynamicTorque` |
| 88–90 | `aero_torque_world` | N·m |
| 91–93 | `aero_lift_body` | N |
| 94–96 | `aero_drag_body` | N |
| 97–99 | `aero_side_force_body` | N, reconstructed exactly as total − lift − drag |
| 100 | `dynamic_pressure_pa` | Pa |
| 101 | `static_pressure_pa` | Pa |
| 102 | `atmosphere_density_kg_m3` | kg/m³ |
| 103 | `speed_of_sound_ms` | m/s |
| 104 | `true_air_speed_ms` | m/s |
| 105 | `aoa_deg` | degrees |
| 106 | `sideslip_deg` | degrees |
| 107–109 | `external_force_residual_world` | N |
| 110–112 | `external_force_residual_body` | N |
| 113–115 | `unexplained_non_aero_force_world` | N |

The realized engine wrench is computed from each live `ModuleEngines.finalThrust` and its
current `thrustTransforms`, distributing total engine thrust by the stock transform
multipliers. KSP thrust transforms point along the exhaust/nozzle direction; the realized
force exported here is the reaction force applied to the vessel, i.e. `-transform.forward`. The per-nozzle force and current CoM give
`r × F` for the realized engine torque. This uses the same live nozzle geometry exposed by
the actuator snapshot, including engines that have more than one thrust transform.

The translational residual is deliberately named **external force residual**, not
`aero_force`:

```text
F_external = m * (a_orbital_derived - g) - F_engine
```

It is valid only after two consecutive physics ticks provide the inertial velocity
derivative. It can contain aerodynamics, RCS/contact/joint forces, and measurement/frame
error. When official live aerodynamic force is also available the bridge additionally
publishes:

```text
F_unexplained_non_aero = F_external - F_aero_live
```

That last signal is a diagnostic for missing force providers or a frame/sign/model error;
it is **not** assumed to be zero in all KSP situations.

A torque residual is intentionally not published yet. With changing mass distribution a
naive `I*omega_dot + omega×I*omega` residual can be misleading unless `dI/dt` and the
other torque providers are accounted for.

#### Capability bits

The legacy V3.1 bits 0–21 are unchanged. Schema 2 adds:

```text
22 realized_engine_wrench
23 krpc_live_aero_force
24 krpc_live_aero_torque
25 krpc_live_aero_components
26 krpc_aero_angles
27 krpc_aero_thermo
28 external_force_residual
29 unexplained_non_aero_force
30 realized_engine_force_on_vessel
```

A capability bit is the authority; a `NaN` is never silently converted to zero. In
particular, live aerodynamic torque may be unavailable with an aerodynamic overhaul that
does not expose the corresponding live quantity.

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
is intentional: a client can consume state and actuator realization from **one physics-tick
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

`python/dynamics_v3.py` is the canonical dependency-free decoder for both RPCs. Keep BB,
coast and guidance behind that decoder instead of hard-coding offsets in multiple places.
It also decodes every transport below, and `canonical_rotation_state()` returns the body
angular rates and the inertia in kg·m² under names that say so.

### DynamicsExtV1 (same-tick supplement)

Quantities the v3 frame does not carry: realized control-surface angles, RCS and
reaction-wheel wrench, thrust available at the current pressure, realized mass flow,
the per-actuator control-effectiveness matrix B, INDI residuals computed **in shadow**,
and the first response tick of each actuator after a v2 frame. It is captured in the
**same** `Pump` call and physics tick as the v3 frame, after it, from the finished v3
state array, then kept in its own 1000-frame ring. Join the two by
`(physics_tick, vessel_persistent_id)`.

It is a separate transport on purpose. A guidance client decodes `dynamics_snapshot_v3()` in its flight
loop, and the decoder rejects any unknown schema, so appending to v3 would break a flight
whose Python side is older than the DLL. With DynamicsExtV1 disarmed, the v3 frames are
byte-identical to earlier builds.

**Born closed.** Nothing is captured until `dynamics_ext_enable_v1(True, lease_seconds)`.
The lease is 0.5–60 s of real time. When it runs out, capture stops and the per-vessel
memory is cleared. `dynamics_ext_enable_v1(False, ...)` disarms at once. Nothing in this
transport writes game state or commands an actuator.

The frame is a 19-double header, a 52-double state block, then five row sections:

| Header offset | Meaning |
|---:|---|
| 0–1 | protocol (`1`), schema (`2` since refactor R3: the frame ends with the one-value clock section, `master_tick`; `1` before) |
| 2–6 | physics tick, UT, fixed delta time, vessel persistent id, topology generation |
| 7 | capability bit mask |
| 8 | state stride (`52`) |
| 9–10 | engine rows / stride (`9`) |
| 11–12 | control-surface rows / stride (`13`) |
| 13–14 | RCS rows / stride (`16`) |
| 15–16 | B rows / stride (`9`) |
| 17–18 | response rows / stride (`12`) |

State block. Body frame = right, forward, bottom:

| Offsets | Field | Unit / source |
|---:|---|---|
| 0–2 | `omega_body_rad_s` | v3 state 19–21 under its true name |
| 3–5 | `omega_dot_body_rad_s2` | v3 state 22–24 under its true name |
| 6–8 | `moi_body_kg_m2` | v3 `moi_native` × 1000 |
| 9–11, 12–14 | `rcs_force_body_n`, `rcs_force_world_n` | Σ −(`useZaxis` ? forward : up) × `ModuleRCS.thrustForces[i]` × 1000; modules marked `isJustForShow` apply nothing and add nothing |
| 15–17 | `rcs_torque_body_nm` | Σ r × F about the current CoM |
| 18–20 | `reaction_wheel_torque_body_nm` | Σ −`ModuleReactionWheel.inputVector` × 1000 over wheels that are enabled, `operational` and `Active`, i.e. what `ActiveUpdate` passes to `AddTorque` |
| 21–23 | `control_torque_body_nm` | realized engine torque (v3 73–75) + RCS + wheels |
| 24–27 | engine totals | Σ available thrust (kN), Σ max thrust (kN), Σ realized mass flow (kg/s), Σ requested mass flow (kg/s) |
| 28–30 | `indi_delta_omega_dot_rad_s2` | Δω̇ₖ = ω̇ₖ − ω̇ₖ₋₁ |
| 31–33 | `indi_pred_b_rad_s2` | I⁻¹·(B·Δuₖ + ΔM_RCS + ΔM_wheels): gimbal and thrust columns at the current tick, RCS and wheels as measured increments |
| 34–36 | `indi_pred_m_rad_s2` | I⁻¹·ΔM_control: change of the realized control torque |
| 37–39 | `indi_pred_aero_rad_s2` | I⁻¹·ΔM_aero: kRPC live aerodynamic torque |
| 40–42 | `indi_residual_b_rad_s2` | Δω̇ₖ − pred_bₖ₋₁ |
| 43–45 | `indi_residual_m_rad_s2` | Δω̇ₖ − pred_mₖ₋₁ |
| 46–48 | `indi_residual_b_filtered_rad_s2` | first-order H(z), τ = 0.1 s, on residual_b. The same filter on both terms equals the filter on their difference |
| 49–51 | `indi_filter_tau_s`, `indi_lag_ticks` (`1`), `indi_valid_streak` | |

The residual is r = ω̇_f(t+1) − ω̇_f(t) − I⁻¹·B_M·Δu(t), in shadow.
The lag is one tick: the actuator state read in FixedUpdate k acts during the physics step
that follows it. Ordering between MonoBehaviours is not guaranteed, so every ingredient is
published per tick, and offline tools can re-align at another lag. Diagonal inertia only;
the gyroscopic term ω × Iω is not subtracted. `pred_b − pred_m` isolates the error of the B
linearization against the geometric torque change of the real nozzles. A consistent sign
error in B shows up there.

Engine rows: `flight_id, ordinal, available_thrust_kn, max_thrust_kn,
thrust_per_throttle_kn, realized_mass_flow_kg_s, requested_mass_flow_kg_s,
isp_at_pressure_s, static_pressure_atm`.
- Available and max thrust are the stock `ModuleEngines.MaxThrustOutputAtm(true,
  useThrustLimiter, part.staticPressureAtm, 310, part.atmDensity)`: the dV app's own
  function, with no side effect. `atmTemp` is unused by the stock body.
- `thrust_per_throttle_kn` is `finalThrust / currentThrottle` above 1 % throttle. It is the
  live throttle-to-thrust gain, including propellant starvation.
- Realized mass flow is `MassFlow() × propellantReqMet / 100 / fixedDeltaTime`, while
  ignited and thrusting, else 0. Requested mass flow is `requestedMassFlow` as last
  computed by the engine.

Control-surface rows: `flight_id, kind, ordinal, deflection_deg, action_deg,
ctrl_surface_range_deg, authority_limiter_pct, deploy, deploy_angle_deg,
current_deploy_angle_deg, actuator_speed_deg_s, use_exponential_speed, ignore_mask`.
- `kind` is 1 for `ModuleAeroSurface` (airbrake) and 2 for any other `ModuleControlSurface`.
- `deflection` is the applied angle, rate-limited toward `action` and written to the
  surface transform in the same update. Both are protected KSP fields, read by reflection.
- `ignore_mask` is pitch = 1, yaw = 2, roll = 4.

RCS rows: `flight_id, ordinal, module_enabled, rcs_enabled, rcs_active, just_for_show,
nozzle_count, thrust_sum_kn, thruster_power_kn, realized_isp_s, force_body_xyz_n,
torque_body_xyz_nm`.

B rows: `flight_id, ordinal, kind, axis, b_body_xyz, u, du`.
- `kind` 1 is a gimbal, with columns ∂M/∂δx (axis 0) and ∂M/∂δy (axis 1) in N·m per
  degree, and u in degrees.
- `kind` 2 is an engine's thrust, with ∂M/∂T in N·m per kN, and u = `finalThrust` in kN.
- The gimbal derivative follows the stock (and lease) transform formula
  `initRots[i] * AngleAxis(δx, xMult·right) * AngleAxis(δy, yMult·(flipYZ ? forward : up))`.
  A change of δx is a world rotation about `parent.rotation * initRots[i] * xMult·right`,
  and a change of δy is one about the second axis after δx. Every thrust transform that
  `IsChildOf` the gimbal transform turns with it: direction and lever. The realized δ is
  `actuationLocal`, or zero for a locked stock gimbal.

Response rows (active vessel only, after a v2 frame applied while armed): `kind (1 engine,
2 gimbal), flight_id, ordinal, sequence, apply_tick, first_response_tick, baseline_x,
command_x, realized_x, baseline_y, command_y, realized_y`.
- Baselines are sampled before the frame's commands are written.
- The response tick is the first captured tick whose realized value (engine
  `currentThrottle`, gimbal `actuationLocal`) moved by more than 1e-4 from its baseline.
- A response tick equal to `apply_tick` means the change was already visible in the same
  FixedUpdate. `NaN` means no response yet.

Capability bits:

```text
0 rotational_state_body      5 engine_availability      10 indi_prediction_m
1 moi_kg_m2                  6 control_surfaces         11 indi_prediction_aero
2 rcs_realized               7 b_matrix                 12 indi_residual_b
3 reaction_wheel_realized    8 indi_delta_omega_dot     13 indi_residual_m
4 control_torque             9 indi_prediction_b        14 response_tracking
```

`dynamics_ext_frames_v1()` uses the same six-value history header and length prefixes as
`dynamics_frames_v3()`. `dynamics_ext_status_v1()` returns `protocol, schema, armed,
lease_remaining_s, latest_tick, history_count, history_capacity, capability_mask`.

### Tracked vessels by persistentId

`track_vessel_v3(persistent_id, lease_seconds)` makes the bridge capture one loaded vessel
by identity in every FixedUpdate, whether it is active or not, in exactly the v3 frame
format. That vessel gets its own ring, finite differences and topology generation.
- Header fields 7–10 carry the v2 controller's sequence data only when that vessel is the
  one under v2 control; otherwise they are 0.
- An unloaded vessel yields no frame, and the channel is cleared.
- The registration is a real-time lease the client renews.
- At most four vessels are tracked.
- The active-vessel stream is unaffected.

Get the persistentId of a kRPC `Vessel` handle from `conn.ident.vessel_ids(vessel)`, which
returns `persistentId<TAB>Guid`. `tracked_vessels_v3()` returns `protocol, count, stride (6)`
then rows `persistent_id, loaded, lease_remaining_s, topology_generation, latest_tick,
history_count`. The tracked snapshot and frame calls raise when the vessel is not tracked.
While DynamicsExtV1 is armed, tracked vessels also get their ext frames (without response
rows).

### Batched aerodynamic wrench

`simulate_aerodynamic_wrench_batch_v1(states, non_rotating_frame=True, persistent_id="")`
runs the installed kRPC `Flight.simulate_aerodynamic_wrench_at` for up to 32 hypothetical
states in one call.
- Each state is 14 doubles: position xyz (m), velocity xyz (m/s), rotation xyzw,
  angular velocity xyz (rad/s), ut.
- The frame is the main body's non-rotating frame, which kRPC recommends for prediction,
  or its rotating frame.
- The reply is `protocol (1), count, stride (7), frame code`, then per state
  `ok, force_xyz_n, torque_xyz_nm` in the same frame. A failed or non-finite state gives
  `ok = 0` and NaNs.
- The kRPC member is bound in a separate method, so a kRPC build without it fails this call
  only.

Each state walks every part's drag cubes on the Unity thread; keep the batches small.

### Reflection cache (QW6)

The v3 capture and `AeroActuatorV1` read a few KSP members by reflection every physics
tick. Until this build, the v3 reader built the key `AssemblyQualifiedName + "|" + name` on
every access (about thirty per tick) and the aero reader called `GetField` /
`GetProperty` / `GetIndexParameters` for every module at every tick. Both now resolve a
member once per (runtime type, name), with the same resolution order and binding flags,
and bind property getters to a delegate once. A Unity `Quaternion` is read directly instead
of by four reflective reads. The values are the same objects, so the frames are the same
bytes.

`dynamics_reflection_check_v1()` proves it on the real vessel: it captures the active
vessel's v3 frame and aero rows twice in one call, through the legacy readers (kept
verbatim) and through the cache, into scratch buffers (no ring is touched), and returns
`protocol (1), v3_length, v3_mismatches, first_v3_mismatch (-1 if none), aero_length,
aero_mismatches, first_aero_mismatch, legacy_capture_us, cached_capture_us` (mean of five
alternated captures each). Expect zero mismatches. `build/tests` runs the same comparison
on synthetic types, with copies of the legacy readers, at every build.

## `conn.stock`

A separate assembly (`Plugins/KRPC.Bridge.Stock.dll`, no dependency on Core, like Tower) for
KSP's stock utilities. Until refactor R3 it was `Plugins/KRPC.Bridge.StockDeltaV.dll` and the
service `conn.stock_delta_v`; the members kept their names and arguments.

It started with KSP's stock delta-v simulation, `VesselDeltaV`, which kRPC's
`Vessel.stages` reads every number from. kRPC refuses with *"Delta-v has not been
calculated for this vessel yet"* until it is ready; on 01/10/2026 two ascents out of
three never got it, from the pad to orbit.

It is now the home of small "stock" utilities that do not need the Core: the stage table read in the game (`stages_v1`), staging from the frame loop
(`stage_in_frame_v1`, `stage_in_frame_v2`) and the locks that hold it (`staging_locks_v1`), the aerodynamic force without the launch clamps (`aero_force_v1`) and
the deployment of the landing legs (`legs_deployment_v1`) and the fields and events of part modules (`module_states_v1` and its three siblings). Flight scene only.

| Member | Type | Notes |
|---|---|---|
| `state_v1(vessel)` | `list[float]` | 1 = true, 0 = false, NaN = unreadable: [0] has a `VesselDeltaV`, [1] `IsReady`, [2] `DoStockSimulation`, [3] `SimulationRunning`, [4] `UpdateFlightScene`, [5] `DeltaVGlobals.DoStockSimulations`, [6] `GameSettings.DELTAV_CALCULATIONS_ENABLED`, [7] operating stage count, [8] active vessel, [9] loaded, [10] packed, [11] `calcsDirty` (private), [12] a simulation coroutine is held (private), [13] `FlightGlobals.ready`. Read only. |
| `recalculate_v1(vessel)` | `bool` | Enables the simulation if the settings allow and it is off, then `SetCalcsDirty(true, true)`. False without a `VesselDeltaV`. |
| `unstick_v1(vessel)` | `bool` | Clears a simulation left marked as running (private `simulation` handle and `updateFlightScene`), then marks it dirty. Only for one that stays "running" and never becomes ready. |
| `aero_force_v1(vessel, body, position, velocity, rotation, reference_frame, without_clamps)` | `list[float]` | The force of `Flight.simulate_aerodynamic_force_at` (same arguments, same frame), without the parts carrying a `LaunchClamp` module when `without_clamps`: [Fx, Fy, Fz] in N, clamp parts left out, parts summed. With the clamps it must equal kRPC's; the ascent checks that before trusting the clamp-free drag (the clamps stay on the ground at lift-off: on 03/10/2026 the drag in flight was 13-35 % below the pad measurement). kRPC's own `StockAerodynamics.SimPartAeroForce` by reflection; FAR refused. Read only. |
| `stages_v1(vessel)` | `list[float]` | The stage table kRPC's `Vessel.stages` would give, read from the vessel at each call: seven numbers per activation stage, stages in increasing order, `[number, start mass (kg), end mass (kg), vacuum thrust (N), vacuum Isp (s), sea-level thrust (N), sea-level Isp (s)]`. Empty while the simulation is not ready. Why: kRPC's object store hands back an existing `Stage` equal to a new one (same vessel id and number), and that object keeps the Unity vessel it was built with; after a revert to launch that reloads the vessel with the same id (FMRS), kRPC's `stages` refuses with "Delta-v has not been calculated" for as long as the server runs (02/10/2026: 60 s of refusals while KSP said ready). Same stage numbers and fields as kRPC (`ActivationStageNumbers`, `GetStage`). Read only. |
| `stage_in_frame_v1()` | `int` | Activates the next stage of the active vessel at the next **frame** (`Update`, where the space bar does it), not in the physics loop where kRPC runs every call. Returns the count of such stagings done before this request: wait for `stages_activated_in_frame_v1()` to exceed it. Raises `RuntimeError` with no active vessel or when staging is locked. Why: a radial decoupler pushes the separated part with a coroutine that waits two frames; on 02/10/2026 the side boosters of a three-core launcher staged by kRPC left nose towards the core and hit it, staged with the space bar they left nose outwards. |
| `stages_activated_in_frame_v1()` | `int` | The count of stagings done by `stage_in_frame_v1()` since the flight scene started. |
| `stage_in_frame_v2()` | `int` | `stage_in_frame_v1()` that does not stop at the lock KSP sets while its window does not have the focus (`flightDriver_ApplicationFocus`, every control type: `FlightDriver.OnApplicationFocus`, KSP 1.12.5 decompiled). Any other lock holding the staging (the Alt+L stage lock, the pause, the hidden stage stack...) still raises `RuntimeError` "Staging is locked by: id=0xMASK, ...". Same return value, same frame-loop staging, counted by `stages_activated_in_frame_v1()`. Why: `StageManager.ActivateNextStage` checks no lock and kRPC's `Control.activate_next_stage` only the Alt+L lock, but V1 refused on the focus lock: on 16/10 and 20/10/2026 ascents launched from a window beside KSP never left the pad (30 refusals in 6.1 s) or kept their spent side boosters. |
| `staging_locks_v1()` | `list[str]` | The input locks holding the staging now, each as `"id=0xMASK"` (the ids of `InputLockManager.lockStack` whose mask includes `ControlTypes.STAGING`), the focus lock included; empty when nothing holds it. A client can say before a launch that the game window has lost the focus. Read only, never raises (`"(ExceptionType)"` when the stack cannot be read). |
| `legs_deployment_v1(vessel)` | `list[float]` | Six numbers per landing leg (a part whose `ModuleWheelBase` is of type LEG, kRPC's definition), in part order: `[animLength (s, clip at speed 1), extendDurationFactor, retractDurationFactor, position (0..1, now), deployedPosition, state]`, state 0 retracted, 1 deploying, 2 deployed, 3 retracting, 4 other (inoperable or unknown). A fixed leg (no `ModuleWheelDeployment`) gives `[0, 0, 0, 1, 1, 2]`. Empty list: no landing leg. A deployment from `position` lasts `animLength × |deployedPosition − position| / |extendDurationFactor|` seconds of game time (the animation runs on physics time). Why: the client asks for its legs at the start of the powered descent, at 45-160 kPa, and would rather ask as late as the deployment allows. Read only. |
| `module_states_v1(modules, with_values)` | `list[str]` | The events and the visible fields of ANY number of part modules in ONE call (`modules`: kRPC `SpaceCenter.Module` objects). What kRPC 0.6.0's `Module.events` and `Module.fields` gave, and what 0.7.0 (which removed them) gives only through `event_list` and `field_list` at one RPC for the list and one to three per member; the same answer on both, read from KSP's `PartModule`. One record per module, in the order of `modules`: `module name, nEvents, nEvents x event gui_name, nFields, nFields x (gui_name, value)`. Events: those that are **visible and active**, in the module's order. Fields: the **visible** ones, in the module's order, a repeated `gui_name` kept (0.6's dictionary raised on it; the client does). A count of `"-1"` stands for a section the game could not read and is followed by ONE string, the message (the other section is still given: a field whose value throws spoils the fields of that module only); a module whose object is gone gives an empty name and both sections `"-1"`. Without `with_values` no value is read and each is the empty string (enough to ask whether a module has a field of that name, and cheaper). Refused with `[MODULE_ACCESS]` when this kRPC's `Module` cannot be opened (neither its `InternalModule` property nor its `module` field): the client then goes back to the lists. Why: the client's ship preparation (every module of every part, for the "Fuel Flow" field) went from under a second to 3 s on the heavy booster (a recorded flight) with the list path, and the opening of the grid fins reads and clicks a module several times a second. Read only. |
| `module_trigger_event_v1(module, gui_name)` | — | Clicks the **first** event of the module that is visible and active and has that `gui_name`: 0.6.0's `Module.trigger_event`. `RuntimeError` ("Sequence contains no matching element") when there is none. |
| `module_field_v1(module, gui_name)` | `str` | The value of the first **visible** field of the module with that `gui_name` (invariant culture): 0.6.0's `Module.get_field`. `RuntimeError` when there is none. Read only. |
| `module_field_by_id_v1(module, id)` | `str` | The value of the first visible field of the module whose identifier (`name`) is `id`, else of the first one of that identifier even if it is not shown (TweakScale's `currentScale`): 0.6.0's `Module.get_field_by_id`. `RuntimeError` when there is none. Read only. |
| `state_v2(persistent_id)`, `recalculate_v2(persistent_id)`, `unstick_v2(persistent_id)`, `stages_v2(persistent_id)`, `legs_deployment_v2(persistent_id)` | as V1 | Refactor R3 (state per vessel): the V1 procedure on the vessel named by its `persistentId` (decimal string), looked up in `FlightGlobals.Vessels` at each call instead of a kRPC `Vessel` object (which kRPC's object store can serve stale after a revert or an FMRS jump). Same answer, same refusals; `[NOT_FOUND]` when the game has no such vessel. |
| `aero_force_v2(persistent_id, body, position, velocity, rotation, reference_frame, without_clamps)` | `list[float]` | `aero_force_v1` on the vessel named by its `persistentId`, as above. |
| `ping()` | `str` | Returns `"pong"`. |

Nothing here touches the vessel or its flight except `stage_in_frame_v1()` and `stage_in_frame_v2()`, which stage like
the space bar, and `module_trigger_event_v1()`, which clicks a button of a part's menu; the rest reads KSP's own delta-v
bookkeeping, the legs or the part modules, or restarts the simulation.

**Part modules.** kRPC 0.7.0 retired the text members of `SpaceCenter.Module` (`fields`, `events`, `has_field`,
`get_field`, `get_field_by_id`, `trigger_event`, ...); the lists that remain (`field_list`, `event_list`) cost one RPC
for the list and one to three per member read. The four `module_*` procedures give 0.6.0's answers for any number of
modules in one call, from KSP's `PartModule`, with the rules of kRPC 0.6.0's `Module.cs` (decompiled): a field or an event
is *visible* when `guiActive` (`guiActiveEditor` in the editor); an event is listed and clicked only when it is also
`active`; the first match wins. They take the kRPC `Module` objects the client already holds, not an identifier: kRPC
keeps the `PartModule` private (`InternalModule`, by reflection here, in 0.6.0 and in 0.7.0 where it is resolved on the
part by name, occurrence and persistent id), and a persistent id plus an index would cost the client two RPCs per module
to rediscover. They compile against kRPC 0.6 and 0.7 from the same source (nothing here depends on `KRPC_07`). A
byproduct: kRPC 0.7's `PartField` and `PartEvent` find themselves again by NAME at every read (decompiled), so through
the lists two entries of the same identifier would both read as the first one; here every entry is its own.


## `conn.tower`

Ground-test tools for the booster catch tower (Terminal Guidance). A separate assembly
(`KRPC.Bridge.Tower.dll`) with no compile reference to the Core, like `conn.stock`: if it
fails to load, KSP skips it and every other `conn.*` service is untouched. **It is part of the
optional `TowerCatch` package** (`GameData/TowerCatch/Plugins/`), not of `KRPC.Bridge`: without that
package `conn.tower` and `conn.tower_site` do not exist (`hasattr(conn, "tower")` is `False`). It
declares a `KSPAssemblyDependency` on the Core, so KSP loads it after it. Flight scene only.
**A test tool, not a flight feature**: `place_v1` teleports a vessel.

| Member | Returns | Notes |
|---|---|---|
| `ping()` | `str` | Returns `"pong"`. |
| `place_v1(vessel, position, rotation, reference_frame, hold_seconds)` | `bool` | Teleports a loaded, unpacked vessel so that its ROOT part has the pose `position` [x, y, z] and `rotation` (quaternion [x, y, z, w]) in `reference_frame`, then holds it there, velocity and spin zero, until `release_v1()` or for `hold_seconds` of game time (clamped to 0.1–120 s). Every rigid body, robotic parts' moving bodies included, moves by the same rigid transform, so the vessel keeps its shape. The pose is kept relative to the body and re-applied every physics tick: use a body frame. Raises on a missing vessel or frame, on a malformed position or rotation, and on a vessel that is not loaded or is packed. True when the vessel is held. |
| `release_v1()` | — | Stops the hold: physics takes over. |
| `holding` | `bool` | Property. Whether a vessel is held by `place_v1`. The hold also ends by itself when its time runs out or the vessel dies or unloads. |
| `is_anchored(vessel)` | `bool` | Whether this catch tower (root part `TG.TowerBase`) is anchored to the ground right now (see below). False while it is packed or not landed, and before its first unpacked tick. |
| `hinge_drive_v1(part)` | `list[float]` | Read only. What the joint of a robotic hinge (`ModuleRoboticServoHinge`, e.g. `TG.TowerHinge`) really has, rather than the module's fields: [0] positionSpring, [1] positionDamper, [2] maximumForce of the joint's angular X drive; [3] motorOutput, [4] servoMotorLimit (%), [5] servoMotorSize (%), [6] hingeDamping (%), [7] driveTargetAngle, [8] targetAngle, [9] traverseVelocity (deg/s); [10] low and [11] high angular X limit of the joint (deg); [12] has enough resources, [13] motor engaged, [14] locked (1 or 0); [15] currentAngle (deg). NaN where a value cannot be read. Raises when the part has no hinge module. **[15] is the module's `currentAngle` FIELD, which `ModuleRoboticServoHinge` writes only in `UpdatePAWUI`, i.e. while the part action window is open (decompiled): stale otherwise.** The angle of the hinge is kRPC's `RoboticHinge.current_angle` (`modelInitialAngle + currentTransformAngle()`), or `angle_deg` of `hinge_lock_status_v1`. |
| `exhaust_damage_v1(vessel, enabled)` | `int` | Turns the exhaust damage of every engine module of the vessel on or off, and returns how many modules changed. `ModuleEngines.EngineExhaustDamage` raycasts along each nozzle up to `exhaustDamageMaxRange` (10 m by default) and adds the WHOLE thrust of that nozzle as a force on any part it hits: a booster hovering over the tower arms pushes them with ~110 kN per engine at idle, against hinge motors that hold 12 kN·m. Not persistent: the next load of the vessel has the engines' own setting back. |
| `joints_v1(part)` | `list[str]` | Read only. Every physics joint touching a loaded, unpacked part — those on its own game object and those of other parts whose connected body is this part — one string per joint: `owner\|connected\|type\|role\|xMotion,yMotion,zMotion\|angX,angY,angZ\|breakForce\|breakTorque`. A part is named `partName#flightID`, a body without a part `go:<name>`, no body at all `world`; role is `attach` (the part's own attach joint), `servo` (a robotic part's servo joint) or `other`; the motions are `-` for a joint that is not a `ConfigurableJoint`. |
| `hinge_lock_at_rest_v1(part, max_rate_deg_s, timeout_s)` | `list[float]` | Locks a robotic hinge (`ModuleRoboticServoHinge`) AT REST: a watcher reads its angle every physics step and, at the first step whose INSTANTANEOUS rate is at or under `max_rate_deg_s`, locks it, bypassing `IsMoving` (see below). Gives up after `timeout_s` seconds of game time. A hinge locked already is not touched (state 3). Arming the same part again replaces its watcher (the old one ends cancelled). Returns the status (layout `hinge_lock_at_rest_v1`, below). Both numbers come from the client, finite and > 0 (`ValueError` otherwise); no default, no clamp. Raises `[NOT_FOUND]` on a part without a hinge module (or a KSP without the members the lock needs) and `[NOT_LOADED]` when the part's vessel is not loaded or is packed. A hinge at rest when armed locks on the second physics step: arm only when the lock is wanted now. |
| `hinge_lock_status_v1(part)` | `list[float]` | Read only. The status of the watcher of this part (layout `hinge_lock_at_rest_v1`); state 0 and NaN fields when there is none in this scene. Raises `[NOT_FOUND]` on a part without a hinge module. |
| `hinge_lock_cancel_v1(part)` | `list[float]` | Cancels the watcher of this part if it is armed (state 5) and returns its status. A hinge it already locked stays locked. |
| `place_v2(persistent_id, position, rotation, reference_frame, hold_seconds)` | `bool` | `place_v1` of the vessel of that persistentId (decimal string, first field of `conn.ident.vessel_ids`). Refactor R3. |
| `is_anchored_v2(persistent_id)` | `bool` | `is_anchored` of the vessel of that persistentId. Refactor R3. |
| `exhaust_damage_v2(persistent_id, enabled)` | `int` | `exhaust_damage_v1` of the vessel of that persistentId. Refactor R3. |

**By persistentId (refactor R3, 2.7).** The `_v2` vessel procedures answer exactly as the
procedure they call, for the vessel of that persistentId whichever vessel is active: a process
without a kRPC handle of the vessel (an observer, a tool) names it the way every other bridge
service does. The lookup is the `conn.stock` one (no Core here), over `FlightGlobals.Vessels`:
`ArgumentException` on a persistentId that is not a decimal uint, `[NOT_FOUND]` when the game
has no such vessel; then the `_v1` checks (loaded, unpacked) apply. The part procedures keep
their part handle, which already names one part of one vessel.

**Hinge lock at rest.** KSP refuses to lock a hinge that moves: `BaseServo.EngageServoLock`
does nothing when `IsMoving()`, and `ModuleRoboticServoHinge.IsMoving` compares to 0.049 deg/s a
MOVING AVERAGE of the absolute rate over the last 20 physics steps
(`BaseServo.CalculateAverageRateOfMovement`; all decompiled from KSP 1.12.5). A catch tower arm
that vibrated on the spot (a recorded flight: ±0.08–0.17° at the hinge, period
1.15–1.2 s, undamped) kept that average near 0.3 deg/s even at its turning points: the lock was
refused for 18 s and the capture with it. The instantaneous rate goes through zero twice a
period. The watcher, stepped from `TowerAddon.FixedUpdate`, takes the hinge's angle as kRPC's
`current_angle` does (`modelInitialAngle + currentTransformAngle()`), its rate between two
DIFFERENT physics steps (`Time.fixedTime`; the first step after arming only records), and on the
first step at or under the threshold runs exactly the two instructions of `EngageServoLock` that
follow its test: `Fields["servoIsLocked"].SetValue(true, module)` (which fires `OnValueModified`,
to which the module subscribed `ModifyLocked`, as in the stock path) then `ModifyLocked(null)`
(protected, by reflection): lock angle recorded, attach nodes updated, `CycleAllAutoStrut`,
`onRoboticPartLockChanged`. Order of the tests on each step: a hinge locked by someone else ends
the watcher (state 3); a slow step locks, even past the time limit; then the time limit. The
watcher ends lost (state 6) when its vessel unloads, packs or dies, and every watcher is
forgotten at the end of the flight scene. Log lines `[KRPC.Bridge.Tower] hinge lock at rest ...`
say every arming, lock (angle, rate, KSP's average), expiry and loss.

| Index | Field | Unit | Meaning |
|---|---|---|---|
| 0 | `protocol` | | 1 |
| 1 | `state` | | 0 none (never armed in this scene), 1 armed, 2 locked by the watcher, 3 already locked (when armed, or by someone else while armed: nothing done), 4 expired, 5 cancelled (by the client or a new arming), 6 lost (vessel unloaded, packed or destroyed, part or module gone), 7 failed (the lock was applied and the hinge did not say locked) |
| 2 | `elapsed_s` | s | game time since arming, frozen at the end |
| 3 | `lock_angle_deg` | deg | the angle of the step it locked at (NaN otherwise) |
| 4 | `lock_rate_deg_s` | deg/s | the instantaneous rate of that step (NaN otherwise) |
| 5 | `max_rate_deg_s` | deg/s | as armed |
| 6 | `timeout_s` | s | as armed |
| 7 | `steps` | | physics steps watched |
| 8 | `min_rate_deg_s` | deg/s | the slowest instantaneous rate seen (NaN before two steps) |
| 9 | `ksp_average_rate_deg_s` | deg/s | `BaseServo.transformRateOfMotion` now, the average `IsMoving` compares to 0.049 |
| 10 | `locked` | | the hinge's `servoIsLocked` now (1 or 0; NaN when its vessel is not loaded) |
| 11 | `angle_deg` | deg | the hinge's angle now (the last one watched when its vessel is not loaded) |
| 12 | `turning_points` | | reversals of direction seen |

The layout is versioned by `HingeRestWatch.Protocol` and `Schema` (1, 1) and declared at load
into the Core's layout registry by reflection (`conn.bridge.layout_v1("hinge_lock_at_rest_v1")`,
kind `status`, one section `status`), since this DLL has no Core reference; without a Core, or
with one older than R1, it is not declared and one log line says so. Its row in
`BridgeManifest.Known` (refactor R3) puts it in `manifest_v1`, read from the constants of the
loaded Tower DLL like every other row. Pure tests:
`build/tests/tower` (the sinusoid and the arm's own trace of 132656: locked at the first turning
point, where IsMoving's average says 0.3 deg/s; the time limit, the order of the tests, the
descriptor against the real `LayoutRegistry`).

**Anchoring, with no call needed.** Every loaded, unpacked, landed (or pre-launch) vessel whose
root part is `TG.TowerBase` gets one unbreakable `FixedJoint` from its root rigid body to the
world, on its FIRST unpacked physics tick. It is released when the tower packs, is no longer
landed, unloads, or at the end of the scene. Why: stock launch clamps hold a tall vessel on
soft joints, and Kerbal Joint Reinforcement skips its ground joints when the vessel unpacks far
from the floating origin; the tower rocked by 1–4.5° (29/09/2026). A world joint keeps its
world anchor, so every anchor is destroyed in the floating-origin shift event itself and made
again on the next tick. The tower is never straightened: it keeps the pose it was saved in, and
its tilt against the local vertical is logged at every anchoring
(`[KRPC.Bridge.Tower] anchored <name> to the ground, tilt <deg> deg`). A tower saved tilted is
re-posed by the client.

## `conn.tower_site`

The catch towers of a game, from the Space Center: list them, read their state, put one on the
ground from a craft of the game, take one away, fly to one — with no Kerbal Konstructs and no
restart of KSP. A second service of `KRPC.Bridge.Tower.dll` (in the `TowerCatch` package; no compile
reference to the Core, like `conn.tower`), in the Space Center, Flight and Tracking Station scenes. A tower is a vessel whose
ROOT part is `TG.TowerBase` (the anchor's rule). Vessels are named by their `persistentId` as a
decimal string (the first field of `conn.ident.vessel_ids`); no kRPC `SpaceCenter` type appears in
any signature of this service.

| Member | Returns | Notes |
|---|---|---|
| `save_folder_v1()` | `str` | The folder of the game being played under `saves/` (`HighLogic.SaveFolder`). |
| `towers_v1()` | `list[str]` | Every tower of the game, loaded or not (layout `towers_v1`, below). Read only. |
| `tower_status_v1(persistent_id)` | `list[str]` | One tower's state (layout `tower_status_v1`, below): read in the game when it is loaded (`source` `loaded`), in its saved state otherwise (`saved`). Read only. |
| `tower_spawn_v1(craft_name, latitude, longitude, heading_deg, vessel_name)` | `str` | **Space Center only.** Puts a tower on the ground from the craft `saves/<game>/Ships/VAB/<craft_name>.craft` (then `SPH`, then the `TowerCatch` package's `GameData/TowerCatch/Crafts`, then the bridge's old `GameData/KRPC.Bridge/Crafts`: see `craft_part_names_v1`), whose root must be `TG.TowerBase`, and saves the game. Returns the new tower's persistentId. `vessel_name` empty: the craft's ship name. Conventions and method below. |
| `tower_remove_v1(persistent_id)` | `bool` | **Space Center only.** Takes a tower (only a tower) out of the game the way the Tracking Station deletes a vessel, and saves the game. |
| `tower_focus_v1(persistent_id)` | `bool` | **Space Center or Tracking Station.** Saves the game and asks for the flight scene on that tower, as the Tracking Station's Fly button does. True when the scene change is requested; the scene loads afterwards. Only a tower. |
| `tower_bubble_v1(persistent_id, range_m)` | `list[float]` | **Flight only.** The tower's BUBBLE: loaded out to `range_m` while it stays packed, unpacked by KSP once its load has finished (below). One call: the game does both steps. A bubble already held keeps the larger range. Returns the status (layout `tower_bubble_v1`). |
| `tower_bubble_status_v1(persistent_id)` | `list[float]` | **Flight only.** The status of the tower's bubble (state 0 when none was posed in this flight scene). Read only. |
| `tower_bubble_release_v1(persistent_id)` | `list[float]` | **Flight only.** Releases the bubble, whoever posed it: the ranges the tower had before the pose written back. Nothing held: nothing done. Returns the status. |
| `tower_ground_v1(persistent_id)` | `list[float]` | **Flight only.** The tower's GROUND JOINTS: what Kerbal Joint Reinforcement fixes to the world only within 500 m of the floating origin, fixed at any distance and made again at every origin shift; never an arm (below). Made now if the tower is loaded, unpacked and landed, at its first such physics step otherwise. Already posed: nothing changes. Returns the status (layout `tower_ground_v1`). |
| `tower_ground_status_v1(persistent_id)` | `list[float]` | **Flight only.** The status of the tower's ground joints (state 0 when none were posed in this flight scene). Read only. |
| `tower_ground_release_v1(persistent_id)` | `list[float]` | **Flight only.** Releases the ground joints, whoever posed them: every joint destroyed. Nothing posed: nothing done. Returns the status. |
| `tower_ground_parts_v1(persistent_id)` | `list[str]` | **Flight only.** The parts of the last selection (layout `tower_ground_parts_v1`, below): every part KJR's rule picks and every hinge, with its physics mass and whether a joint holds it. Read only. |
| `craft_part_names_v1(craft_name)` | `list[str]` | The distinct part names of a craft, in the order of its PART nodes: Ships/VAB, then Ships/SPH of the game, then the crafts of the `TowerCatch` package (`GameData/TowerCatch/Crafts`), then the ones the bridge shipped before the package existed (`GameData/KRPC.Bridge/Crafts`, kept as a fallback for the transition). The game's own copy wins. What an installation must have loaded to spawn it. Read only. `NOT_FOUND`: no such craft. |
| `tower_package_v1(craft_name)` | `list[str]` | Where the tower's patch and craft are in `GameData`: four rows, a path relative to `GameData`, a tab, then 1 when the file is there, else 0, always in this order: `TowerCatch/Patches/TowerCatch.cfg`, `KRPC.Bridge/Patches/TowerCatch.cfg` (the patch the bridge carried before the package), `TowerCatch/Crafts/<craft_name>.craft`, `KRPC.Bridge/Crafts/<craft_name>.craft`. Exactly one patch is right: none and the parts do not exist (KSP removes the towers of a save at load), both and every part is made twice. The game's `Ships` folders are not looked at. Read only, any scene. `ValueError`: a craft name that is not a bare file name. |
| `parts_loaded_v1(part_names)` | `list[str]` | Which of these parts KSP has loaded (`PartLoader.getPartInfoByName`): one row per name, in order, the name, a tab, then 1 or 0. Read only. |
| `expansions_v1(names)` | `list[str]` | Which of these expansions are installed (`ExpansionsLoader.IsExpansionInstalled`): one row per name, the name, a tab, then 1 or 0 (`MakingHistory`, `Serenity`). Read only. |
| `ping()` | `str` | Returns `"pong"`. |

**Refusals.** A refusal starts with a code (`RuntimeError` in Python); a malformed argument is a
`ValueError` without a code.

| Code | Meaning |
|---|---|
| `WRONG_SCENE` | the procedure does not run in this scene (spawn and remove: Space Center; focus: Space Center or Tracking Station; the three bubble procedures and the four ground joint procedures: Flight) |
| `NOT_FOUND` | no vessel of that persistentId; no such craft (every path tried is named); a craft KSP could not load |
| `NOT_A_TOWER` | the vessel's or the craft's root part is not `TG.TowerBase` |
| `OCCUPIED` | a landed, splashed or pre-launch vessel of the body stands closer to the point than the two extents (below) |
| `SAVE_DISABLED` | the game's FLIGHT parameter `CanAutoSave` is off: `GamePersistence.SaveGame` would not write `persistent.sfs`; nothing is done |
| `SAVE_FAILED` | the save returned nothing: the change is in the game, not in `persistent.sfs` (spawn, remove), or no flight was started (focus) |
| `REFUSED` | terrain under the footprint below sea level; a root part whose origin would be under the terrain (KSP lifts such a landed vessel when it loads it, `Vessel.getCorrectedLandedAltitude`) |
| `UNSUPPORTED` | the home body does not rotate (no surface orbit to save) |
| `TRANSIENT` | no game, no `FlightGlobals` or no terrain yet (a scene loading) |

**The list of vessels.** `FlightGlobals.Vessels`, in every scene: `Game.Start` loads every saved
vessel as a `Vessel` object in a scene with a planetarium (Flight, Space Center, Tracking Station:
`flightState.Load()`), and `GamePersistence.SaveGame` rebuilds the game's `flightState` from
`FlightGlobals.Vessels` there (`Game.Updated`: `flightState = new FlightState()`, one
`BackupVessel` per vessel; KSP 1.12.5, decompiled). `FlightGlobals.Vessels` is what the next save
writes; `flightState.protoVessels` is only what the last one read. The root part is
`rootPart.partInfo.name` when the vessel's parts exist,
`protoVessel.protoPartSnapshots[rootIndex].partName` otherwise.

**Status, loaded or saved.** Loaded: every `ModuleRoboticServoHinge` of the vessel, its angle as
kRPC's `RoboticHinge.current_angle` (`modelInitialAngle + currentTransformAngle()`, the same
reflective read as `hinge_lock_status_v1`), `targetAngle`, `servoIsLocked`, `servoMotorIsEngaged`;
the `ElectricCharge` of every part; the tilt TowerAnchor logs (the root's `up` against the radial
of its position); `anchored` from TowerAnchor (Flight only). Saved: the same fields from the
`ProtoPartModuleSnapshot` (`targetAngle`, `servoIsLocked`, `servoMotorIsEngaged` as written), the
angle from what `BaseServo.OnSave` wrote of its moving transform (`servoTransformRotation` =
`movingPartObject.transform.localRotation`): its Euler angle about the module's `mainAxis`, wrapped
to (-180, 180] as `currentTransformAngle` does, plus the prefab module's `modelInitialAngle`; the
`ElectricCharge` of the `ProtoPartResourceSnapshot`s; the tilt from the saved `rot` and the vertical
of the saved latitude and longitude.

**Spawn: the two conventions.** `latitude` and `longitude` are those of the ROOT part, the mast's
foot, where Kerbal Konstructs puts the spawn point (the client takes the capture
point and puts the spawn point `d_axe` metres opposite the cap): the client computes the foot.
`heading_deg` is the azimuth of the craft's +X axis (0 north, 90 east, clockwise seen from above),
for the towers of the craft generator the direction of the arms, mast to booster. At 90
the tower has exactly the orientation of the two towers Kerbal Konstructs launched from a spawn
point of yaw 0; any other heading turns that orientation about the local vertical by
(`heading_deg` − 90) clockwise. Proof, in the pure tests: the `rot` this computes at heading 90 for
the latitude and longitude of `Tour Catch Sud` and `Tour Catch Nord` is within 0.00126 and 0.00123
degrees of the `rot` KSP saved for them, which is their own tilt (0.0013 deg); their saved root +X
points west and +Z south, i.e. the craft's +X east (the root is turned 180 deg about +Y in the
craft).

**Spawn: the rotation.** `rot` is the root part's rotation relative to the body
(`Vessel.srfRelRotation = Inverse(bodyTransform.rotation) * vesselTransform.rotation`;
`ProtoVessel.Load` sets `transform.rotation = bodyTransform.rotation * rot`), in the body's local
frame, the one of `CelestialBody.GetRelSurfaceNVector` (the vertical of (lat, lon) is
`(cos lat cos lon, sin lat, cos lat sin lon)`). The craft frame's +X goes to the horizontal of
azimuth `heading_deg`, +Y to the vertical, +Z to the horizontal of azimuth `heading_deg` − 90; the
root's `rot` is that rotation times the root's rotation in the craft. Computed from latitude,
longitude and heading only (`TowerSiteGeometry`, pure): no floating origin, no rotating frame, no
time.

**Spawn: the altitude.** Nothing arbitrary. The craft is loaded (`ShipConstruct.LoadShip`, at the
craft file's positions, +Y up) and each collider gives a footprint by KSP's own rule for a launch
(`ShipConstruction.PutShipToGround`: every enabled collider not on layer 21): its world bounds in
the craft frame, relative to the root's origin. Then the same `OnPutToGround` message KSP sends at a
launch: a launch clamp sizes its tower to the craft's lowest point (its saved `height`) and may
raise that lowest point to its own level, which then counts as the bottom of every collider under
it. The root's altitude is the largest, over the footprints, of max(terrain under the footprint) −
the footprint's bottom − ρ²/2(R + terrain): every collider at or above the HIGHEST terrain under its
own footprint, the one that sets the altitude exactly on it, no margin (ρ²/2R, 0.3 mm at 20 m on
Kerbin, is how much higher a point ρ metres from the root axis stands above the sphere than the
root's tangent plane). The terrain is the PQS's analytic height (`pqsController.GetSurfaceHeight` −
`Radius`, the height `Vessel.getCorrectedLandedAltitude` checks a landed vessel against), sampled on
one grid over the union of the footprints with a spacing of at most 0.5 m (a resolution, never added
to the altitude), plus the four corners of every footprint and the root's own point. The error the
spacing leaves between samples is computed from the samples (the largest second difference of the
grid, κh²/8 per axis) and logged.

**Spawn: occupancy.** The craft's extent is its farthest footprint corner from the root,
horizontally. Every landed, splashed or pre-launch vessel of the body has its own: the farthest of
its parts' saved origins from its root, across the local vertical (part origins: a lower bound of
its colliders). Refused (`OCCUPIED`) when the surface distance between the two roots is smaller
than the sum. Kerbal Konstructs statics and KSC buildings are not vessels and are not checked.

**Spawn: the vessel node.** It reproduces the VESSEL nodes KSP saved for the two towers Kerbal
Konstructs launched (`persistent.sfs`, compared field by field), and KSP itself writes it to the
save (`Game.AddVessel` builds the `ProtoVessel` and its `Vessel` the way `Game.Start` does for every
saved vessel of the scene; `GamePersistence.SaveGame` then writes it like any unloaded vessel:
`BackupVessel` keeps the `ProtoVessel` and refreshes its orbit and discovery).

| Field | Spawned tower | Why |
|---|---|---|
| `sit`, `landed`, `splashed` | `LANDED`, `True`, `False` | As both towers. `PRELAUNCH` would not change the clamps (both give `StartState.Landed`, which is what `LaunchClamp.OnStart` tests before setting its ground joint) but it skips the test below and forces a ground positioning at the first unpack (`Vessel.GoOffRails`). |
| `PQSMin`, `PQSMax` | the home body's `pqsController.minLevel`, `maxLevel` (the terrain preset: 2, 10 on a test install) | As both towers. A landed vessel whose saved levels equal the body's is NOT ground-positioned when it unpacks (`skipGroundPositioning` set true in `GoOffRails`): no vertical move, no rotation onto the ground normal (`Vessel.CheckGroundCollision` turns a vessel more than 1 deg off it). The computed pose is the one that loads. |
| `skipGroundPositioning`, `skipGroundPositioningForDroppedPart`, `vesselSpawning` | `False` | As both towers (`vesselSpawning` true would force the ground positioning). |
| `lat`, `lon`, `alt` | the arguments (longitude in (−180, 180]) and the computed altitude, of the root part | `Vessel.latitude/longitude/altitude` are those of `vesselTransform`, the root part. |
| `rot` | computed (above) | |
| `hgt` | altitude − terrain at the root | `heightFromTerrain`, used only by the ground positioning, which is skipped. |
| `nrm` | the local vertical in the root's frame | `terrainNormal`, idem (the towers have the measured normal, 0.05 deg from it). |
| `CoM` | Σ (mass + resources) × (part origin + `CoMOffset`), root frame | `Vessel.localCoM`; recomputed when the tower unpacks. |
| `type` | the craft's `vesselType` (`Base`) | As both towers; `Base` when the craft has none. |
| `landedAt`, `displaylandedAt`, `launchedFrom` | empty | The towers say `Tour_Catch` / `Tour_Nord`, their Kerbal Konstructs site. KSP recovers at every launch any vessel whose `landedAt` CONTAINS the launch site's name (`ShipConstruction.FindVesselsLandedAt`): an empty one contains none. |
| `persistentId` | `FlightGlobals.GetUniquepersistentId()` | Checked again by `ProtoVessel`'s constructor. |
| `pid` | a new Guid | |
| `ref`, `ctrl` | the probe core's `flightID`, `True` | As `AssembleForLaunch` picks it: the root when it commands, else the first crewable part, else the first command part (the towers: `probeCoreOcto.v2`). |
| `stg`, `prst`, `met`, `lct`, `lastUT`, `distanceTraveled`, `GroupOverride`, `Override*` | 0 (no staged part), `False`, 0, now, now, 0, 0, the craft's | |
| `ORBIT` | the orbit of the CoM held on the rotating surface now (`Cross(zUpAngularVelocity, −pos)`, KSP's rotating-frame velocity) | KSP recomputes a landed vessel's orbit from its position when its `OrbitDriver` starts (`TrackRigidbody`, mode `IDLE`). |
| `DISCOVERY`, `ACTIONGROUPS` | owned, size C, infinite lifetime; empty | As both towers (`state = -1`, `size = 2`). |
| parts: `uid`, `mid`, `launchID`, `flag` | one `ShipConstruction.GetUniqueFlightID` each, one mission and one `launchID` (the game's, incremented) for all, the craft's flag | As `AssembleForLaunch`. |
| parts: `persistentId`, `cid`, `position`, `rotation`, modules, resources | as `ShipConstruct.LoadShip` and `ProtoPartSnapshot` write them | `orgPos`/`orgRot` about the root; the launch clamps' `height` from `OnPutToGround` as at a launch (the towers: 0.0150088 m). |
| parts: `mass`, `temp`, `tempExt`, `tempExtUnexp` | `UpdateMass()` (prefab + modules); the air's temperature at the altitude (`FlightGlobals.getExternalTemperature`) | What a part would get in flight; `Part` sets that temperature itself for a part saved without one. |

The construct's part objects are destroyed after the snapshot (`DestroyImmediate`, as KSP disposes
of a loaded construct), the part persistentIds its load registered released first. The log line
`[KRPC.Bridge.Tower] spawned <name> persistentId <id> from <craft>: ...` gives every computed number:
latitude, longitude, heading, altitude, terrain at the root, `hgt`, terrain range under the
footprint, samples, step, sampling error bound, governing collider and point, lowest point before
and after `OnPutToGround`, extent, nearest landed vessel, `rot` and its tilt, `nrm`, `CoM`, mass,
parts, colliders, root, type, `ref`, mission, launch, PQS levels, and the save.

**Remove** fires `onVesselTerminated`, sends any crew back to the roster, destroys the vessel's
object (`Vessel.OnDestroy` takes it out of `FlightGlobals`) and saves: the path of
`SpaceTracking.OnVesselDeleteConfirm`. Log: `[KRPC.Bridge.Tower] removed ...`.

**Focus** saves `persistent`, then `FlightDriver.StartAndFocusVessel("persistent", index)`, the
path of `SpaceTracking.FlyVessel`. The index is the tower's in the `flightState` the save just
wrote (the order of `FlightGlobals.Vessels`, which the Fly button passes); both are logged
(`[KRPC.Bridge.Tower] focus ...`).

**Layout `towers_v1`** (kind `text`): row 0 the header, row 1 the columns, then one row per tower.

| Header key | Meaning |
|---|---|
| `protocol`, `schema` | 1, 1 (`TowerSiteFormat.Protocol`, `Schema`) |
| `n` | towers |
| `scene` | `HighLogic.LoadedScene` |

| Column | Meaning |
|---|---|
| `persistent_id` | decimal |
| `name` | the vessel's name (tabs and line breaks replaced by spaces) |
| `body` | the body's name |
| `lat_deg`, `lon_deg`, `alt_m` | the root part's position: the live one when loaded, the saved one otherwise |
| `situation` | `Vessel.Situations` (`LANDED` ...) |
| `loaded` | 1 or 0 |
| `packed` | 1 or 0; empty when not loaded |

**Layout `tower_status_v1`** (kind `text`): row 0 the header (the tower), row 1 the columns, then
one row per hinge, in part order.

| Header key | Meaning |
|---|---|
| `protocol`, `schema` | 1, 1 |
| `persistent_id`, `name` | the tower |
| `source` | `loaded` (read in the game) or `saved` (read in its `ProtoVessel`) |
| `situation`, `loaded` | |
| `packed`, `anchored` | 1 or 0; empty when not loaded; `anchored` only in flight (TowerAnchor's ground joint) |
| `tilt_deg` | the root's +Y against the local vertical |
| `ec_amount`, `ec_max` | `ElectricCharge` of every part; empty when the tower has none |
| `n` | hinges |

| Column | Meaning |
|---|---|
| `part_persistent_id`, `part_flight_id` | the hinge's part |
| `angle_deg` | as `RoboticHinge.current_angle` (above); empty when unreadable |
| `target_angle_deg` | `targetAngle` |
| `locked` | `servoIsLocked`, 1 or 0 |
| `motor_engaged` | `servoMotorIsEngaged`, 1 or 0 |

Unknown values are empty, never zeros. Both layouts are versioned by `TowerSiteFormat.Protocol` and
`Schema`, declared at load into the Core's layout registry by reflection with the hinge lock's
(`conn.bridge.layout_v1("towers_v1")`), and have their rows in `BridgeManifest.Known`. Pure tests:
`build/tests/tower` (`TowerSiteTests.cs`: the rotation against the two saved towers, the heading
convention, the altitude over flat, sloped and uneven terrain, the clamps' floor, the curvature
term, the sampling bound, the extents, the refusals, both tables and their descriptors).

**The bubble: load first, unpack once loaded.** kRPC's `Vessel.physics_range = R` sets, in
every situation, load = unpack = 0.9 R and pack = unload = R (`VesselPhysicsRangeAddon.Apply`,
kRPC 0.6.0 decompiled): a tower beyond its stock load range (2.25 km landed) is LOADED AND UNPACKED
in the same frame (`Vessel.Update` loads it, `OrbitPhysicsManager.LateUpdate` unpacks it right
after). The ground test of 13/10/2026 (a test install, without KJR, 22 unpacks out of 22) found that such a
tower sags: rails 0.22 m lower over its foot than a good tower (38.59 m against 38.81 m), 2.5 cm
further from the mast, its locked hinges scattered by 1.4 deg either way, arms swinging up to 0.31 m
sideways in 3 s; a tower loaded earlier (packed) and unpacked later is good. What KSP 1.12.5 does
(decompiled): `Vessel.Load` instantiates the parts and only logs `landed - waiting for ground
contact to resume physics...`; `Vessel.GoOffRails` unpacks a loaded vessel once its physics hold is
over (75 frames after `Vessel.Awake`: long over for a vessel of the scene) and, landed, once
`CheckGroundCollision` has found the ground (skipped for a tower saved with the body's PQS levels):
it never asks whether the parts have started. A part builds itself over three frames (`Part.Start`,
a coroutine): its modules' `OnStart` (a servo's moving rigid body, its joint not yet connected), its
rigid body (`isKinematic = packed`), then `ModulesBeforePartAttachJoint` (the servo joint connected,
its lock angle read from the current angle) and `CreateAttachJoint`, then `started = true`. Unpacked
before that, every body is dynamic from its creation and the `FlightIntegrator` pulls it down (it
integrates every rigid body of an unpacked vessel from the root) before its joints exist; loaded
first, every joint is made between kinematic bodies at the saved pose.

`tower_bubble_v1` does the two steps, from `TowerAddon.FixedUpdate` (`TowerBubbles`,
`TowerBubbleWatch`):

1. **Loading** (state 1): in every situation load = max(0.9 R, the vessel's own) and unload =
   max(R, own); pack and unpack the vessel's own (landed: 350 / 200 m): KSP loads the tower and
   keeps it packed.
2. **Settling** (state 2, loaded and packed): ready when every part has `Part.started`, every
   `BaseServo` has `ServoInitComplete`, and the position of every rigid body (a part's own, a
   servo's moving one) against the root, in the root's frame, has not moved by more than the
   tolerance over **3 consecutive physics steps** with all of that true: three is the longest joint
   work KSP still defers once a part has started (`Part.SetJoints`: three `WaitForFixedUpdate`, then
   `ResetJoints`; the autostruts are secured one `WaitForFixedUpdate` after `UpdateAutoStrut`). The
   tolerance is 4 x eps32 x (the largest absolute world coordinate of the two looks + the tower's
   extent), eps32 = 2^-23: a root-relative position is the difference of two float32 world
   positions, so two looks at a still tower differ by at most 2 sqrt(3) ulps (1.6 mm 3.3 km from the
   origin). One look per physics step (`Time.fixedTime`): the RPC and the `FixedUpdate` of the same
   step count once.
3. **Unpacking** (state 3): pack = max(R, own) and unpack = max(0.9 R, own) as well (the values of
   `physics_range`), and KSP unpacks the tower on its own path (`GoOffRails`, its ground check
   included); then **unpacked** (state 4), held so: an active vessel switch packs every vessel and
   KSP unpacks it again (its parts stay built); an unload (beyond R) goes back to step 1.

A tower found unpacked BEFORE its load was finished (KSP's own 200 m, another writer) is held anyway
and flagged `early_unpack`; one already unpacked when the bubble is posed is held, not flagged. The
ranges are written again at every physics step; a range changed by someone else since the last
write (kRPC's `physics_range` on the same tower, which its addon re-applies at every `FixedUpdate`)
is flagged `overwritten` and logged once: do not combine the two. No range is ever made shorter than
the vessel's own. A bubble is not owned by the client that posed it (as `physics_range`): any client
releases it by persistentId, and the end of the flight scene forgets every bubble and writes the
original ranges back on the vessels still there (the next scene rebuilds them with the stock ranges).

Log, every step with the game time: `[KRPC.Bridge.Tower] bubble posed on <tower> (<id>): range ...`,
`... loaded, packed: n parts ...`, `... load finished: n/n parts started, k/k servos ready, pose still
for 3 physics steps (shift, tolerance, largest); unpack ranges written ...`, `... unpacked by KSP ...`,
`... released on ...: original ranges written back ...`; warnings `UNPACKED BEFORE ITS LOAD WAS
FINISHED`, `ranges ... changed by another writer`, `lost`.

**Layout `tower_bubble_v1`** (kind `status`, one section `status`, 23 values, protocol 1, schema 1,
`TowerBubbleWatch.Protocol`, `Schema`):

| Offset | Field | Meaning |
|---|---|---|
| 0 | `protocol` | 1 |
| 1 | `state` | 0 none, 1 loading, 2 settling, 3 unpacking, 4 unpacked, 5 released, 6 lost |
| 2 | `range_m` | R held |
| 3 | `elapsed_s` | game time since the pose, frozen at the end |
| 4, 5 | `loaded`, `packed` | now: 1 or 0, NaN when unknown (`packed` when not loaded) |
| 6, 7 | `parts`, `parts_started` | at the last step |
| 8, 9 | `servos`, `servos_ready` | `BaseServo` modules, with `ServoInitComplete` |
| 10, 11 | `still_ticks`, `settle_ticks` | consecutive still steps; 3 |
| 12, 13 | `shift_m`, `tolerance_m` | the largest move of a rigid body against the root since the step before; its tolerance |
| 14 | `settle_shift_max_m` | the largest such move with every part started |
| 15, 16, 17 | `loaded_after_s`, `ready_after_s`, `unpacked_after_s` | since the pose (first time), NaN until then |
| 18 | `early_unpack` | 1: unpacked before its load was finished |
| 19 | `overwritten` | 1: another writer changed its ranges |
| 20, 21 | `loads`, `unpacks` | counts |
| 22 | `distance_m` | to the active vessel (transforms, as `OrbitPhysicsManager`) |

Declared at load into the Core's layout registry with the others, row in `BridgeManifest.Known`.
Pure tests: `build/tests/tower/TowerBubbleTests.cs` (the order of the steps, every part and servo
needed, a move above the tolerance resets the count, float32 noise of a tower 3.2 km from the
origin under it, one look per physics step, the case flagged early, a repack, an unload, a
loss, the release, the status, the descriptor).

**The ground joints: the tower fixed to the world at any distance from the floating origin.**
Ground tests of 16/10 and 19/10/2026 (a test install, the north tower NOT active, unpacked by the bubble,
its arms opened, locked, closed): 3.2 km from the active vessel the hinges slip while the arm's tip
stays still - at `maxMotorOutput` 20000 a sustained vibration of 0.13-0.21 deg (period 1.2 s) and a
failed close, at 80000 the hinge runs against its own motor to its stop in 0.38 s; 400 m from the
active vessel the same tower closes in 4.8-5.0 s (20000) and 3.00 s (80000), no second move, no
vibration. The difference is Kerbal Joint Reinforcement (KJR Continued, decompiled:
`KJRManager.HandleWorldSpaceJoints`, `KJRGroundJointModule`). For a landed vessel with launch clamps
it adds one `FixedJoint` to the world (`connectedBody` null, `breakForce` and `breakTorque` infinite,
on the part's own game object) to each of its 3 heaviest parts (`Part.physicsMass`, not a clamp,
`physicalSignificance` not `NONE`; a pick's symmetry counterparts added) - only if the vessel is
within 500 m of the floating origin when its physics starts (`Vessel ... is N m from origin, skipping
ground joint creation` otherwise), and it destroys them at the first floating origin shift for good.
On the v5 tower those three parts are the root `TG.TowerBase` (160 t, which `TowerAnchor` fixes
anyway) and the two arm hinges `TG.TowerHinge`: 8.4 t each at 20000 (0.4 t of part plus the motor,
`BaseServo.GetModuleMass` = `motorizedMassPerKN` 0.0004 x `driveUnit`), 32.4 t at 80000, above the
8.0-8.04 t of the mast sections. (`FlightIntegrator.UpdateMassStats` gives half of a servo part's
`physicsMass` to each of its two rigid bodies, and kRPC's `Part.mass` reads `rb.mass` alone: the
4.2 t read in game at 20000 is half of the 8.4 t KJR sorts by.) The joint holds `Part.rb`, a
hinge's FIXED half; the arm hangs on its moving half (`Part.servoRb`). Near the origin the hinges'
fixed halves are held to the world; far away the hinge's reaction torque goes down the arm mount, the
carriage and the mast, and the hinge's angle (fixed half against moving half) moves while the arm
stays.

`tower_ground_v1` fixes, at any distance (`TowerGround`, `TowerGroundWatch`, from
`TowerAddon.FixedUpdate` and its `onFloatingOriginShift` handler):

* the parts KJR's rule picks, in its order (stable sort by `physicsMass`, ties in part order; no
  clamp, no physicsless part; symmetry counterparts added);
* the fixed half (`Part.rb`) of every hinge (`ModuleRoboticServoHinge`) whose parent is attached to
  one of its fixed nodes (not one of `BaseServo.servoAttachNodes`): with a light motor (the stock 1200:
  0.88 t) KJR's rule would pick two mast sections instead, and the hinges are still held;
* NEVER a "mobile" part: one attached to a servo node of any robotic servo, or below one; a servo whose
  parent holds it by a servo node; anything attached by a surface node to or from a servo (its side is
  not known). A KJR pick that is mobile is dropped (KJR would fix it). A part without a rigid body gets
  no joint.

On the v5 tower: the root and the two hinges, at 20000 as at 80000 - what KJR fixes at 400 m. One joint
per part, KJR's own (`FixedJoint` on `Part.rb`, world, anchor zero, axis up, unbreakable), made on the
first physics step where the tower is loaded, unpacked and landed (now if it is): the pose it keeps is
the pose it was unpacked in, as `TowerAnchor`.

**At a floating origin shift.** A joint to the world keeps its world anchor, and the shift moves every
body: `FloatingOrigin.setOffset` (decompiled) moves the celestial bodies, then every vessel
(`Vessel.SetPosition`; a landed vessel by the offset plus the Krakensbane frame offset, as its body and
terrain), then other objects, and only then fires `onFloatingOriginShift`. A joint left as it was would
pull its part back by the whole offset at the next physics step. In that event every joint is destroyed
at once (`DestroyImmediate`: `Object.Destroy` waits for the end of the frame, after the physics step)
and made again at once from the moved bodies (`Physics.SyncTransforms` first): every body and the
terrain moved by the same offset, so the new anchor is the old one relative to the ground. No physics
step runs without the joints, none with a stale anchor: no jolt, no ratchet.

**Life cycle.** Nothing is fixed while no client poses it. Posed, the joints are made at every unpack,
made again at every shift, and made again whenever one is found destroyed by someone else or the part
count changed; destroyed when the tower packs (an active vessel switch packs every vessel), unloads or
leaves the ground (made again at its next unpack), at `tower_ground_release_v1` (any client, by
persistentId) and at the end of the flight scene. They live next to `TowerAnchor`'s ground joint on the
root (two world joints on the root, as at 400 m with KJR). A make that fixes nothing is not tried again
until the part count changes or the tower unpacks again.

Log, with the game time: `[KRPC.Bridge.Tower] ground joints posed on <tower> (<id>): ... m from the
floating origin`, `... of <tower> made (unpacked): 3 joint(s), 176.80 t: TG.TowerBase 160.00 t (KJR),
TG.TowerHinge 8.40 t (KJR, hinge), ...; KJR's rule picks 3 (excluded 0), hinges 2/2 fixed (0 skipped);
3244 m from the floating origin (beyond KJR's 500 m)`, `... made again after k floating origin shift(s)
(the last of N m): 3 joint(s), n remade in all` (at the first shift, then at most one line per 10 s of game
time: with Krakensbane active the origin shifts at every physics step; the joints are made again on the
same parts without selecting again while the vessel keeps them), `... released (packed): n joint(s)
destroyed`, `... released on ...`; warnings `lost`, `destroyed at a floating origin shift and NOT made
again`.

**Layout `tower_ground_v1`** (kind `status`, one section `status`, 23 values, protocol 1, schema 1,
`TowerGroundWatch.Protocol`, `Schema`):

| Offset | Field | Meaning |
|---|---|---|
| 0 | `protocol` | 1 |
| 1 | `state` | 0 none, 1 waiting (not loaded, packed or not landed: nothing fixed now), 2 held, 3 released, 4 lost |
| 2 | `joints` | joints alive now |
| 3 | `parts_fixed` | parts of the last selection that are fixed (NaN before the first make) |
| 4, 5 | `kjr_parts`, `kjr_excluded` | parts KJR's rule picks (counterparts included); of those, not fixed (mobile, or no body) |
| 6, 7, 8 | `hinges`, `hinges_fixed`, `hinges_skipped` | hinges of the tower; fixed; skipped (mobile, side not known) |
| 9 | `mass_fixed_tonnes` | sum of the physics masses of the parts fixed |
| 10 | `makes` | makes: unpacks, after a destroyed joint or a part count change (not the shifts) |
| 11 | `remade_after_shift` | joints made again in a floating origin shift |
| 12 | `shifts` | floating origin shifts seen while posed |
| 13 | `broken` | joints found destroyed by someone else |
| 14 | `releases` | joints destroyed by the tower's state: packed, unloaded, off the ground |
| 15 | `elapsed_s` | game time since the pose, frozen at the end |
| 16 | `held_after_s` | since the pose, first time held; NaN until then |
| 17, 18 | `loaded`, `packed` | now: 1 or 0, NaN when unknown (`packed` when not loaded) |
| 19 | `origin_distance_m` | the vessel's distance to the floating origin now (the norm of `vessel.transform.position`, KJR's measure), NaN unloaded |
| 20, 21 | `frame_rotating`, `frame_speed_ms` | now: the frame rotates with the body (1 or 0); the Krakensbane frame speed. The world is the ground only when the frame rotates and does not move |
| 22 | `last_shift_m` | the move of the last shift seen |

**Layout `tower_ground_parts_v1`** (kind `text`, as `towers_v1`, protocol 1, schema 1,
`TowerGroundWatch.PartsProtocol`, `PartsSchema`): row 0 the header `protocol`, `schema`,
`persistent_id` (the tower), `name`, `state` (as the status), `n`; row 1 the columns
`part_persistent_id`, `part_flight_id`, `part_name`, `physics_mass_tonnes` (`Part.physicsMass`, what KJR
sorts by), `kjr` (1: picked by KJR's rule), `hinge` (1: a hinge), `mobile` (1: on a moving side, never
fixed), `held` (1: a joint holds it now); then one row per part KJR's rule picks and per hinge, the
fixed ones first. No row before the first make.

Both declared at load into the Core's layout registry with the others, rows in `BridgeManifest.Known`.
Pure tests: `build/tests/tower/TowerGroundTests.cs` (the v5 tower in the craft's part order with the
masses above: the root and the two hinges at 20000, 40000 and 80000; the hinge rule with a light motor;
a heavy part on an arm, an inverted hinge, a surface-attached hinge or part, a clamp, a physicsless part,
symmetry, a parent loop; made, packed, unloaded, off the ground, shifted while held and while waiting,
dropped, broken, a part count change, an empty make, released, lost; the status, the table, both
descriptors).

**Limits.** The analytic terrain is the one KSP checks a landed vessel against, not the terrain's
collider: the collider is the PQS mesh, flat between vertices tens of metres apart; on flat ground
(the tower sites, the KSC) the two agree within a millimetre (`Tour Catch Sud`: 64.743 m analytic 13 m
from its foot, 64.7425 m under its foot from its saved `alt` and `hgt`), on
uneven ground the tower may stand a little above or into the mesh (TowerAnchor holds it either way).
Statics (Kerbal Konstructs, KSC buildings) are neither terrain nor vessels: a tower put on one is
not placed on it. The modules of the craft's parts save themselves outside the flight scene and
without a vessel, as in the editor; a module that cannot fails the spawn (the construct is destroyed,
nothing is added).

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

### Dropped-stage table and readiness (`dropped_stages_v1`, `ready_v1`)

FMRS keeps four loose facts per dropped vessel (id, save, name, state). It does not keep
which staging dropped it, which vessel it came off, or when. The bridge sees the separation
itself, records those facts at the moment they happen, and joins them to FMRS's table when
asked. Both members are read-only.

| Member | Type | Notes |
|---|---|---|
| `dropped_stages_v1()` | `list[str]` | The table, protocol 1, schema 1: line 0 is a header of `key=value` pairs (tab-separated), line 1 the column names, then one tab-separated row per vessel FMRS has registered. **Raises** like the other members when FMRS is not usable in this scene, so an empty table always means "nothing registered". |
| `ready_v1()` | `dict[str,str]` | Can FMRS be used right now, and why not. **Never raises**: the probe to call first, after a scene change or a jump. |

**Header** (line 0): always starts with `protocol`, `schema`, `n` (the number of rows), then
`ut`, `scene`, `main_guid`, `main_pid`, `launched_at` (FMRS's), `has_launched`, `switched`,
`craft` and `launch_ut` (the launch the bridge saw), `stagings` (since that launch),
`pending` (separations FMRS has not registered yet), `tracker` (`1` when the bridge's
separation capture is hooked). Check `protocol` before reading anything else.

**Columns** (line 1; only ever appended, so map them by name). An unknown value is an
empty string, never a zero.

| Column | Meaning |
|---|---|
| `guid` | FMRS's id: what `jump_to_vessel` takes. |
| `persistent_id` | KSP's `persistentId` of the vessel. |
| `save` | The FMRS save the vessel belongs to. Rows with the same save came off together. |
| `stage` | The stage **activated** by the staging that released the vessel: the vessel's `currentStage` just **after** the staging, the number FMRS puts in `FMRS_save_N`, the `istg` of its decoupler in the `.craft`. |
| `stage_before` | The active vessel's `currentStage` just **before** that staging (usually `stage + 1`: what a pilot's staging log notes). |
| `stage_source` | `staging` (the bridge saw the staging and the separation), `save_name` (it did not; read from an `FMRS_save_N` name), or empty: a separation outside any staging has no stage, even when FMRS names its save `FMRS_save_N`. |
| `separation_ut` | UT of the separation, as the bridge saw it. |
| `save_ut` | UT at which FMRS registered the vessel and wrote its save (the bridge's observation, else the time FMRS recorded for the save). A row exists only once FMRS has registered the vessel, and FMRS writes the save in the same call. |
| `parent_persistent_id`, `parent_guid`, `parent_name` | The other vessel of the separation: the one that carried on. |
| `name` | The vessel's name as FMRS recorded it. |
| `craft` | The vessel's name at the launch of the mission the separation belongs to. |
| `state` | FMRS's state now: `FLY` (registered), `LANDED`, `DESTROYED`, `RECOVERED` or `NONE`. |
| `capture` | `event`: the bridge saw the separation and the registration. `fmrs`: FMRS's table alone (FMRS persists its table across sessions; a KSP restart loses the bridge's facts). |
| `kind` | `decouple` or `undock`, for an `event` row. |

Rows are ordered by separation, then save write, then `guid`. FMRS's table decides which
vessels appear: a revert empties it, and the rows go with it.

**`ready_v1()` keys**: `protocol`, `schema`, `ready` (`"1"`/`"0"`), `reason` (empty when
ready), `resolved`, `active`, `scene`, `scene_ok`, `jump_in_progress`, `armed`, `enabled`,
`has_launched`, `switched_to_dropped`, `main_known` (FMRS recorded a main vessel: an armed
launch), `main_guid`, `main_pid`, `launched_at`, `dropped` (registered vessels), `pending`,
`tracker`. `ready` = FMRS resolved, an FMRS object in this scene, a scene where FMRS runs and
kRPC answers (flight, space centre, tracking station), no jump starting.

**How the bridge captures it**, at no per-tick cost when nothing happens: static hooks on
`onStageActivate` (the activated stage; the active vessel's `currentStage` is still the one
before), `onPartDeCoupleNewVesselComplete` (original, new) and `onVesselsUndocking` (both
sides: id, `persistentId`, name), `onGameStateSaved` (FMRS's save write) and `onLaunch`. A
separation belongs to the latest staging at most 0.5 s of game time before it (the same
frame in stock KSP). From a separation, for at most 10 s of game time, a watch diffs FMRS's
table the tick after each save write (FMRS registers, saves, then may swap the main vessel,
all in one Update) and every fifth tick as a backstop, and binds each new entry to the
separation that created it, or to the one it was the original side of (FMRS's "main vessel
changed": the dropped side kept the root part). Entries FMRS held before the separation are
never bound to it. Each binding publishes a bus event `fmrs.stage_registered` (vessel id;
detail `stage=… | stage_before=… | save=… | pid=… | parent_pid=… | sep_ut=… | save_ut=… |
name=…`). The existing `fmrs.dropped` event is unchanged.

```python
lines = conn.fmrs.dropped_stages_v1()
head = dict(kv.split("=", 1) for kv in lines[0].split("\t"))
assert head["protocol"] == "1"
cols = lines[1].split("\t")
rows = [dict(zip(cols, l.split("\t"))) for l in lines[2:]]
core = [r["guid"] for r in rows if r["stage"] == "4"]
```

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

## `conn.trajectories`

Trajectories predicts where the active vessel hits the ground by integrating its descent
through the stock aerodynamics — the drag cubes, at the vessel's *actual* attitude or at
the descent profile you set. Stock kRPC has no impact prediction of any kind; this one runs whenever you
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
| `vessel_id_v1()` | `str` | Refactor R3 (state per vessel): the `persistentId` (decimal string) of the vessel the predictions are for, the ACTIVE vessel, the only one Trajectories follows; empty without one. Never raises. |
| `impact_geo_for_v1(persistent_id)` | `list[float]` | `get_impact_geo()` for the vessel the caller names: the impact when `persistent_id` is the vessel Trajectories computes for, an empty list when it has none; `[UNSUPPORTED_MODE]` for any other vessel, so the non-active booster of a double flight never takes the other one's impact for its own. |
| `get_impact_geo()` | `list[float]` | `[lat_deg, lon_deg, terrain_alt_m]` on the active vessel's main body. **Raises** `RuntimeError` when there is no impact — guard with `has_impact()`. |
| `get_impact_position()` | `list[float]` | The mod's raw vector `[x, y, z]` in metres: relative to the main body's centre, world axes, rotated to where that surface point is *now*. Add the body's position for a world position. **Raises** when there is no impact. |
| `get_time_till_impact()` | `float` | Seconds to impact, or `-1` if none. Never raises. |
| `get_end_time()` | `float` | Universal time of the impact, or `-1`. |
| `get_impact_velocity()` | `list[float]` | `[x, y, z]` m/s, world axes. **Empty list** if no impact. |
| `update_trajectory()` | — | Ask for a recompute now. Lands on a later tick; does not block. |
| `invalidate_aerodynamic_model_v1()` | `bool` | drop the mod's cached aerodynamic model, so the next computation rebuilds it from the vessel as it is now (the mod's internal `Trajectory.InvalidateAerodynamicModel()`). Call it once after changing the drag without changing the parts, such as opening grid fins: the mod rebuilds by itself only on a part count change or a 20 % change of its reference drag at 3000 m/s. `True` if the call reached the mod, `False` if the mod or that internal member is absent. Never raises. |
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
