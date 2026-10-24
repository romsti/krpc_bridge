# Architecture

Read this before adding a plugin, before changing the project layout, or before putting a
build into a game.

It describes the mod as it is (October 2026): a Core, ten plugins built and scanned
together, eleven DLLs that all say which commit they come from, and a manifest that gives the sha256 of
every file shipped. Since refactor R1 the Core also carries what every plugin
needs (Core v2: a game clock, a vessel registry, one lease, one frame recorder, the bit and
layout registries, frames and units, typed errors). Since refactor R3 what flies and what observes are separate DLLs: the old
Actuators is a pure Model, the Control that writes the actuators and the Telemetry that only
reads them, each service has an English name, and every frame carries the master tick of the
game clock. Lot 3 adds the event bus V2 (one poll per client tick, every record on the master
tick, by persistentId) and the contact by leg (ContactV1, latched at every physics step); its
line 3.1 makes the control surfaces a modeled actuator of AttitudeV1 (SurfaceModelV1, born
closed), and an earlier change checks that replica against the vessel's measured acceleration (the slope κ,
born closed). The first section is the five facts about kRPC and KSP that constrain everything
else; they have not changed since the first version of this file.

---

## Five facts

### 1. kRPC already runs RPC bodies on Unity's main thread

kRPC processes remote procedures during Unity's `FixedUpdate`. That is what the server's
"max time per update", "one RPC per update" and "adaptive rate control" settings bound.

**Consequence: an ordinary service method needs no dispatch at all.** When
`FmrsService.JumpToVessel` touches `FlightGlobals.ActiveVessel`, it is already on the main
thread. Wrapping an RPC body in `MainThread.Invoke` adds a queue hop and nothing else —
and would deadlock the whole game if `Invoke` did not detect the main thread and run
inline.

### 2. The corollary: a slow RPC freezes the game

If a procedure spends 300 ms integrating a trajectory, it spends 300 ms inside
`FixedUpdate`, and the frame rate drops to 3 for that frame. No server setting can
subdivide a single call.

The real asynchrony problem in a bridge plugin is therefore the *inverse* of the one people
expect. It is not about getting work onto the main thread; it is about getting expensive
work off it. That is what `Jobs` is for.

### 3. One malformed signature disables the entire kRPC server

kRPC scans every loaded assembly when the server starts. A single invalid signature — in
any DLL, yours or someone else's — makes `ServicesChecker.OK` false, which gates kRPC's
`FixedUpdate`. Every service dies, including stock `SpaceCenter`. The only symptom is a
popup and a line in `KSP.log`.

**Consequence: plugin isolation is a BUILD-TIME property.** There is no runtime mitigation
available: .NET does not unload assemblies, and kRPC scans everything already loaded. No
try/catch, no registry, no configuration flag can quarantine a broken plugin from inside
the game, because the damage is done before any of your code runs. See
[Keeping a bad signature out of GameData](#keeping-a-bad-signature-out-of-gamedata).

### 4. kRPC has events, but they are latches, not messages

An event rides the stream channel. `Trigger()` sets a flag the client's stream picks up on
the next update. So it tells a client **that** something happened, carries **no payload**,
and two triggers inside one tick **collapse into one**.

A kRPC *stream* is weaker still: it samples a value once per update, so it reports what
something IS, never that something HAPPENED. A part destroyed between two samples leaves no
trace.

**Consequence: the Core provides both.** `EventBus` is an ordered, id-numbered ring buffer
written on the C# side — nothing is lost between polls, and each record carries a
timestamp and a payload. `CoreService.OnEvent` returns a `KRPC.Service.Event` that fires
when a matching record is written. Block on the event for latency, read the log for
completeness.

### 5. Load order comes from assembly attributes, not from the folder tree

KSP loads every `*.dll` under `GameData` recursively; the directory layout is
organisational only. What actually orders loading is `[assembly: KSPAssembly]` and
`[assembly: KSPAssemblyDependency]`: KSP's `AssemblyLoader` reads them from every DLL,
builds a dependency graph, topologically sorts it, and loads in that order.

**Consequence: graceful degradation is free.** A plugin declaring a dependency on
`KRPC.Bridge.Core` is guaranteed Core's types are in the AppDomain before its own
`KSPAddon` runs — and if Core is missing or too old, KSP *skips* the plugin and logs it,
rather than loading it and throwing `TypeLoadException` on first use. Two plugins (Tower
and Stock) declare **no** dependency on the Core on purpose: they must load next to
any Core version, and they use nothing from it.

---

## Layout

### On disk

```
GameData/KRPC.Bridge/
├── KRPC.Bridge.Core.dll        + .xml     <- Core at the root (services Bridge, Discovery)
├── manifest.json                          <- what this build ships: sha256 of every file, commit
├── LICENSE, NOTICE, KRPC.Bridge.version
└── Plugins/
    ├── KRPC.Bridge.Model.dll        + .xml   no service          (pure: law, allocation, gates, geometry)
    ├── KRPC.Bridge.Control.dll      + .xml   service Control     (what flies: leases, v2, AttitudeV1)
    ├── KRPC.Bridge.Telemetry.dll    + .xml   service Telemetry   (what observes: V3, Ext, tracked, aero)
    ├── KRPC.Bridge.Fmrs.dll         + .xml   service FMRS        (FMRS Continued)
    ├── KRPC.Bridge.Ident.dll        + .xml   service Ident       (stock identifiers)
    ├── KRPC.Bridge.MechJeb.dll      + .xml   service MechJeb     (MechJeb 2)
    ├── KRPC.Bridge.Ocisly.dll       + .xml   service OCISLY      (OfCourseIStillLoveYou)
    ├── KRPC.Bridge.Stock.dll        + .xml   service Stock       (stock utilities, no Core)
    ├── KRPC.Bridge.Tower.dll        + .xml   services Tower, TowerSite (catch tower, no Core)
    └── KRPC.Bridge.Trajectories.dll + .xml   service Trajectories (Trajectories)
```

The `.xml` must accompany **each** DLL in its own folder — kRPC looks for a file of the same
name next to the assembly. A single `.xml` at the root would document the Core only.

The release zip ships all of it; `Template` is never built.

The booster catch tower's parts are a second package, with a zip of its own. They used to travel
inside `GameData/KRPC.Bridge` (`Patches/`, `Crafts/`); installing the bridge then installed the
patch, with no way to decline it. The services that place and read the tower (`conn.tower`,
`conn.tower_site`) stay in `KRPC.Bridge.Tower.dll`; the package is only what the tower is made of:

```
GameData/TowerCatch/
├── manifest.json                          <- sha256 of every file, the commit (tools/manifest.py)
├── LICENSE, NOTICE, TowerCatch.version
├── Patches/
│   └── TowerCatch.cfg                     <- ModuleManager patch: TG.* copies of parts of Modular Launch
│                                             Pads, HullCamera VDS and the two DLC (Breaking Ground,
│                                             Making History); text only, no model, no texture
└── Crafts/
    └── Tour Catch v5.craft                <- the tower tower_spawn_v1 puts on the ground
```

It needs ModuleManager, Modular Launch Pads, HullCamera VDS Continued, KRPC.Bridge and the two paid
DLC (which CKAN cannot install; `distribution/TowerCatch.netkan` says it in its description). The
part names (`TG.*`) did not change: a tower already in a save loads as before. Its sources are
`distribution/TowerCatch/`, which `build.cmd` copies to `dist/GameData/TowerCatch` as it is.

**The patch is in exactly one mod folder, always.** Two copies make every `TG.*` part twice
(ModuleManager's `+PART`); no copy makes KSP delete the towers of a save at load, for want of their
parts. `conn.tower_site.tower_package_v1`
tells a client which of `GameData/TowerCatch` and `GameData/KRPC.Bridge` holds the patch and the craft; and
`build.cmd deploy` refuses to copy over an install that still has the bridge's old patch. A craft is
searched in the game's `Ships/VAB` and `Ships/SPH`, then `GameData/TowerCatch/Crafts`, then
`GameData/KRPC.Bridge/Crafts` (the fallback of the transition, `TowerPackage.cs`).

### In the repo

```
Directory.Build.props        TFM, KSP paths, doc file, Deterministic.  Once, for every project.
Directory.Build.targets      KSP references, the build stamp, deploy of one project.
KRPC.Bridge.sln              Core + the ten plugins
build.cmd                    tests, stamp, build, signature scan, manifests (Windows)
src/
  Core/                      KRPC.Bridge.Core.dll
  Plugins/
    Model/  Control/  Telemetry/  Fmrs/  Ident/  MechJeb/  Ocisly/  Stock/  Tower/  Trajectories/
    Template/                copy this to start a plugin (never built)
build/
  stubs/Stubs.cs             stand-ins so sources type-check with no KSP
  verify/                    compiles every Core-dependent source against the stubs
  scan/                      kRPC's own signature scanner + the manifest check, on the real DLLs
  tests/                     pure tests without KSP: AttitudeV1 (golden file), fmrs/, manifest/,
                             core/ (Core v2, event log V2), tower/, contact/ (ContactV1's latch)
distribution/
  KRPC.Bridge.netkan, .version        the bridge's CKAN record and KSP-AVC file
  TowerCatch.netkan                   the tower package's CKAN record (dependencies: see above)
  TowerCatch/                         the tower package, exactly as it lands in GameData/TowerCatch
    Patches/  Crafts/  NOTICE  TowerCatch.version
tools/
  build_stamp.py             the commit, clean or dirty, compiled into every DLL
  manifest.py                writes dist/.../manifest.json, once for each package
  check_docs.py              docs/API.md against the C#, signature types, XML docs, versions
  package.py                 the two release zips (KRPC.Bridge-x.y.z.zip, TowerCatch-x.y.z.zip)
  find_ksp.py                finds a KSP install
python/check_bridge.py       smoke test against a running game
dist/GameData/               an exact mirror of the destination
```

### Referencing the Core without duplicating it

Two mechanisms answering two different questions. You need both.

**`ProjectReference` with `Private=false`** gives compile-time types and build ordering:

```xml
<ProjectReference Include="..\..\Core\KRPC.Bridge.Core.csproj">
  <Private>false</Private>
</ProjectReference>
```

`Private=false` is the whole answer. Without it, a second copy of `KRPC.Bridge.Core.dll`
lands inside `Plugins/`, KSP loads both, every type exists twice in the AppDomain, and the
error reads `cannot convert KRPC.Bridge.Core.MainThread to KRPC.Bridge.Core.MainThread`.

**`KSPAssemblyDependency`** gives *runtime* load ordering. MSBuild orders the build; the
attribute orders the load.

### Output routing

Each csproj sets its own `OutputPath` from `DistRoot` (Core at the root, plugins in
`Plugins/`), in the project body: `Directory.Build.targets` is imported too late to move the
real output. So `dist/GameData/` has exactly the shape of the destination, and deployment is
a mirror copy with no path logic. The references live in `.targets` rather than `.props`
because `.props` is imported before the project body and could not see `BridgeUseStubs`,
which the stub type-check, the tests and the scanner set to skip the KSP references.

---

## The Core

`KRPC.Bridge.Core.dll` owns the bootstrap `MonoBehaviour` and knows nothing about any
specific mod.

| | |
|---|---|
| `BridgeCore` | Bootstrap once per session (`Instantly`, `DontDestroyOnLoad`); every `FixedUpdate`: `Ticks++`, pump the main-thread queue (2 ms budget), sweep finished jobs every 256 ticks. Logs its build stamp at start. |
| `ModRegistry` | Where a plugin announces itself, and the reflection helpers (assemblies by historical names, compiled getters). |
| `EventBus` | 10 GameEvents into a 512-entry ring buffer, plus kRPC event signals. Used by FMRS and OCISLY. Since lot 3 also the event bus V2: a second, preallocated 1024-record ring (`EventLogV2`, pure) that copies every v1 record with its persistentIds and receives the events of R6 (decoupling, joint breaks, part count, physics easing, rails, warp, loading) and the contact transitions of Telemetry (`RecordV2`), on the master tick; `poll_events_v2` reads it from a sequence number. The v1 log, ids and signals are unchanged; the V2 hooks are lambdas kept in fields, removed in `BridgeCore.OnDestroy`. |
| `Wait` | Block an RPC on game state without freezing the game (kRPC continuations). |
| `Jobs` | Off-main-thread computation with a pollable result. |
| `MainThread` | The classic dispatcher, for the three cases that need one. |
| `Hud` | Hide and show the HUD the way F2 does. |
| `BridgeManifest` | What is loaded and which commit it comes from (`manifest_v1`). Pure: System and reflection only. |
| `GameClock` (+ `ClockCore`, pure) | Core v2: one master tick per physics step, UT, fixedTime, dt, warp rate and real time frozen when the step opens; hosts the legacy tick counters (`clock_v1`). |
| `VesselRegistry` | Core v2: vessels by persistentId, topology generation moved by GameEvents, the topology fingerprint memoized per window (`vessels_v1`). |
| `Lease` (pure) | Core v2: one lease (owner, token, wall guard with the legacy float arithmetic, optional physics-step budget). |
| `Recorder` (pure) | Core v2: the preallocated frame ring and its history reader. |
| `BitRegistry`, `LayoutRegistry` (pure), `CoreLayouts` | Core v2: the meaning of every published bit (`bits_v1`) and the schema descriptor of every published layout (`layout_v1`). |
| `Frames` (pure) | Core v2: body/world/kRPC frames, unit constants, the "SI in RPCs" rule. |
| `BridgeErrors`, `TokenMemory` (pure) | Core v2: typed error codes at the start of a refusal's message. |
| `BridgeLog` | One log prefix for the whole mod. |
| `CoreService` | `conn.bridge`. |
| `DiscoveryService` | `conn.discovery`: read-only introspection of the running game (assemblies, types, members, part modules), for tools that explore a running game. |

### Blocking without freezing: `Wait`

A `while (!ready)` loop or a `Thread.Sleep` inside an RPC hangs Unity, not just the client.
kRPC's own continuation is the answer: throw a `YieldException` and the server re-invokes
your delegate on a later update, letting the game run in between.

```csharp
Wait.Until (() => condition, timeoutSeconds, "what you are waiting for");
Wait.Settled (() => condition, consecutiveTicks, timeoutSeconds, "...");
Wait.Ticks (n);
```

Deadlines are in **real** time, deliberately: the in-game clock stops during a scene load,
which is exactly when a timeout must still be able to fire. A resumed continuation is not
re-checked against the game scene, and there is no client-side timeout: always bound the
chain.

### Getting work off the main thread: `Jobs`

Capture plain data on the main thread (never a `Vessel`, `Part` or `GameObject`), hand it to
`Jobs.Start`, compute on the snapshot alone, poll or `Wait.Until` from the RPC. **The worker
must not touch Unity or KSP**: off the main thread most of Unity's API hard-crashes the
process with no stack trace.

### Core v2 (refactor R1)

What every plugin needed and each rebuilt for itself (audit A §3.3). The pure parts (System
only) are tested by `build/tests/core`; nothing here changes a byte that flies.

**One clock.** `GameClock` opens a physics step at the first of: its `TimingManager` callback
at stage `ObscenelyEarly` (registered from the first FixedUpdate, and again whenever it stops
being called: `TimingManager.Instance` is protected, a plugin cannot see it), or the Core's
`FixedUpdate`. A step is recognised by `(Time.frameCount, Time.fixedTime)`, so it opens exactly
once whatever the order of those calls (limit: past about 72 h of one session the float
`fixedTime` can no longer separate two steps of one frame). The master `tick`, and the UT,
fixedTime, dt, warp rate and real time of the step, are frozen there. The frames keep the
counters they always published (renumbering them would change their bytes): `core.fixed_updates`
(`conn.bridge.ticks`), `actuators.pump_tick` and `attitude.tick` are `ClockCounter`s hosted by
the clock, with their old increments, and the clock records the master tick of each increment.
`clock_v1()` is that alias table. A new contract counts in master ticks. Since refactor R3 every
frame ring appends the master tick of its capture step (`GameClock.StepTick`) in a last section
`clock`, each touched layout one schema higher, the old fields unchanged (`docs/API.md`, « One
tick in every frame »).

**One vessel registry.** `VesselRegistry` keys vessels by persistentId (active, loaded, packed,
a generation moved by the topology GameEvents) and owns the topology fingerprint the Control and
Telemetry transports compare (moved there unchanged). The fingerprint is walked at most once per
vessel and per WINDOW: a window is one uninterrupted stretch of the bridge's own code (the actuator
pump, the AttitudeV1 Earlyish tick), opened and closed by its owner, in which no game code runs
between two reads, so the memo returns exactly what a new walk would. Between Earlyish and the
pump, part FixedUpdates and kRPC's RPCs run (a staging RPC splits a vessel): one fingerprint per
STEP needs the events to be enough, and the registry measures exactly that (`unevented_changes`
in `vessels_v1`). Zero over enough flights is the evidence to arm a per-step memo.

**One lease.** `Lease` is the lease of the v2 controller (and AttitudeV1's authority on the
active vessel), the per-engine and per-gimbal leases, the non-active authority, the
AttitudeV1 arming, the DynamicsExtV1 arming and the tracked-vessel registration. The wall guard
keeps the legacy float arithmetic (`now + seconds` stored as a float, expired when `now >=`),
each call site still samples the clock where it did, and `build/tests/core` compares it bit
for bit. A physics-step budget exists (`ExpiresAtTick`); every migrated lease leaves it unset,
so expiry is the wall's, as before. A contract that wants "N steps, wall as a guard" sets it.

**One recorder.** `Recorder` is every frame ring (V3 of the active and of each tracked vessel,
DynamicsExtV1, AeroActuatorV1, AttitudeV1, lateral force): `capacity` slots whose buffers are
reused, a writer that `Reserve`s a zeroed buffer (what `new double[n]` gave), fills it and
`Commit`s it, so a frame costs no allocation once the ring has wrapped. `ReadSince` returns the
legacy six-value header and length-prefixed frames; it finds the first frame by binary search
while ticks never decrease, and falls back to the legacy scan for good when one does.
`build/tests/core` compares it with a verbatim copy of the LinkedList rings on thousands of
random sequences.

**Bits and layouts.** `BitRegistry` holds the meaning of every published mask (motifs with their
blocking kind computed from the write rules' masks, capabilities, reasons, options), declared by
the owning plugin FROM ITS CONSTANTS (`bits_v1`). `LayoutRegistry` holds the schema descriptor of
every frame, return, status and text table (`layout_v1`): sections with counts and strides read
from named fields, length-prefixed frames, field names and units; the declaring plugin checks
each fixed section against the constant that sizes it. Layouts are append-only and never depend
on a mode.

**Frames and units.** `Frames` writes once the body frame (x right = pitch, y nose = roll,
z bottom = yaw; a positive ctrlState command gives a negative body torque), the world and kRPC
frames, and the unit constants. The rule for every RPC added from R1 on: SI in, SI out; a legacy
field that keeps a native unit (t, kN, kPa, deg) says so in its name.

**Typed errors.** A refusal starts with a code (`[UNSUPPORTED_MODE]`, `[TOKEN_EXPIRED]`,
`[TOKEN_INVALID]`, `[NOT_LOADED]`, `[TRANSIENT]`, `[BUSY]`, `[NOT_FOUND]`), the exception type and
the text after it unchanged; `TokenMemory` remembers why the last tokens stopped being valid.

---

## The plugins

Two families: **wrappers** of third-party mods, reached by reflection so a missing mod
degrades to `available = False`; and **our own flight services**, which drive or observe
the stock game for a flight-control client.

| DLL | Service (scene) | RPCs | Core | Hooks | Role |
|---|---|---|---|---|---|
| Model | none | 0 | no (System only) | none | the pure model: law and allocation, write rules, lateral force, gimbal pivots, effector registry (below) |
| Control | `Control` (Flight) | 37 | yes | `ControlAddon` (Instantly, registration), `ControlWatcher` (Flight, `FixedUpdate` = the actuator pump, `OnDestroy`), AttitudeV1 on `TimingManager` Earlyish + `OnFlyByWire` | what flies: leases, protocol v2, per-vessel authority, AttitudeV1 and its lateral force (below) |
| Telemetry | `Telemetry` (Flight) | 26 | yes (and Control) | `TelemetryAddon` (Instantly, registration and the pump's observer); no `FixedUpdate` of its own | what observes: V3, DynamicsExtV1, tracked vessels, AeroActuatorV1, aero wrench batch, samples, contact by leg (below) |
| Fmrs | `FMRS` (All) | 36 | yes (ModRegistry, EventBus) | `FmrsWatcher` (every 5 ticks), `DroppedStagesWatch` (bounded watch after a separation) | FMRS Continued: dropped stages, jumps, recovery ledger, the dropped-stage table |
| Ident | `Ident` (Flight) | 6 | yes | registration only | part `flightID`, vessel `persistentId` and `Guid` that kRPC holds but does not publish |
| MechJeb | `MechJeb` (All) | 58 | yes | registration only | MechJeb 2: ascent, staging, any module by name, landing predictor, maneuver planner |
| Ocisly | `OCISLY` (All) | 16 | yes (Hud, EventBus) | `AutoRestoreAddon` | camera streams across a scene reload |
| Trajectories | `Trajectories` (Flight) | 28 | yes (MainThread) | `TrajectoriesHeadless` | impact prediction, target, descent profile |
| Stock | `Stock` (Flight) | 15 | **no** | `FrameStaging` (Flight, `Update`) | stock utilities: delta-v simulation and stage table, staging from the frame loop, clamp-free aero force, landing-leg deployment; every procedure that takes a vessel also by persistentId (`_v2`) |
| Tower | `Tower` (Flight), `TowerSite` (Space Center, Flight, Tracking Station) | 14 + 13 | **no** | `TowerLayouts` (Instantly, Awake only: its layouts into the Core's registry, by reflection), `TowerAddon` (`FixedUpdate`: hold of `place_v1`, hinge locks at rest, tower bubbles), `TowerAnchor` (`FixedUpdate` + floating-origin shift) | catch tower: ground anchor of every `TG.TowerBase` vessel (always on), exhaust damage switch, place/hold, hinge and joint probes, hinge lock at rest; the towers of a game from the Space Center: list, state, spawn from a craft, remove, fly to; in flight, a tower's bubble, loaded first and unpacked once loaded (`TowerSite`) |

### Model, Control, Telemetry (the old Actuators, refactor R3)

About 15 800 lines, more than half of the mod, in three DLLs since refactor R3 (lot 2, line 2.3),
so that **what observes never redeploys what flies**:

```
KRPC.Bridge.Model      pure (System, plus Assembly-CSharp for its KSPAssembly only); no service
      ^
KRPC.Bridge.Control    service Control: writes the actuators; Core + Model
      ^
KRPC.Bridge.Telemetry  service Telemetry: reads them; Core + Model + Control (internals)
```

Control references nothing of the telemetry. The physics step has ONE driver,
`ControlWatcher.FixedUpdate` calling `Actuation.Pump()`; Telemetry has no `FixedUpdate`: at load
`TelemetryAddon` puts an instance of its `TelemetryObserver` into `Actuation.Observer`
(`IActuationObserver`), and the pump calls it at exactly the points where the single Actuators
DLL called DynamicsV3, DynamicsTracked and the response tracker, so a step runs in the order it
always did and no order between two MonoBehaviours is left to Unity. Without the Telemetry DLL
the pump runs alone (null observer). Cross-assembly internals are opened by
`InternalsVisibleTo` (`Friends.cs`: Model to Control and Telemetry, Control to Telemetry); KSP
loads Model, then Control, then Telemetry (`KSPAssemblyDependency`). A telemetry fix redeploys
Telemetry only; a change of Control's internal surface that Telemetry uses redeploys both.

Everything is static state (`Actuation` in Control), so it survives the creation and destruction
of `ControlWatcher` across scenes; its `OnDestroy` hands every lease back and resets the observer's
streams. Since refactor R1 every ring is a Core `Recorder`, every lease a Core `Lease`, the pump
and attitude ticks `ClockCounter`s of the game clock, the topology fingerprint the
`VesselRegistry`'s; the pump and the AttitudeV1 tick are registry windows. The masks of the
layouts are declared into the bit registry (`ControlBits.cs`, `TelemetryBits.cs`) and their
descriptors into the layout registry (`ControlLayouts.cs`, `TelemetryLayouts.cs`), each by the
DLL that owns them. The old wire names stay on the client side: a client written for both generations maps
`Actuators` to `Control` + `Telemetry` and `StockDeltaV` to `Stock`, procedure names unchanged (`docs/API.md`, « Service names »).

| Part | DLL | Files | What it does | Runs |
|---|---|---|---|---|
| Leases (historical) | Control | `ControlService.cs`, `Actuation.cs` | per-engine throttle and per-gimbal leases with real-time expiry, restored on expiry, on release and at scene exit | every tick while a lease exists |
| Atomic protocol v2 | Control | `ControlV2.cs`, `Actuation.cs` | one exclusive owner (token, TTL), whole command frames applied in one tick, `control_snapshot_v2`; `acquire_control_for_v1` names the vessel it means (refused unless it is the loaded active vessel), `control_status_v2` the vessel held | every tick |
| Non-active authority | Control | `Actuation.cs` | AttitudeV1's own per-vessel authority for the booster that is not the active vessel | every tick while held |
| DynamicsV3 | Telemetry | `DynamicsV3.cs` | the coherent state of the active vessel at the physics rate (header, state block, per-engine and per-nozzle rows), 1 000-frame ring | **every tick, always** |
| DynamicsExtV1 | Telemetry | `DynamicsExt.cs` | supplemental quantities of the same tick as V3, transported apart so the V3 schema stays byte-identical; under a real-time lease | only while enabled |
| Tracked vessels | Telemetry | `DynamicsTracked.cs` | V3 frames of other vessels, by `persistentId` (double flights) | only while tracking |
| AeroActuatorV1 | Telemetry | `AeroActuatorV1.cs` | aerodynamic-actuator observability of the same tick, kept apart from V3 | every tick, with V3 |
| Aero wrench batch | Telemetry | `AeroWrenchBatch.cs` | many simulated aerodynamic wrenches in one call | on call only |
| Reflection cache (QW6) | Telemetry | `ReflectionCache.cs` | compiled readers of private KSP fields, checked against the legacy readers by `dynamics_reflection_check_v1` | used by DynamicsV3 |
| ContactV1 (lot 3) | Telemetry | `ContactWatch.cs`, `ContactLatch.cs` (PURE), `ContactService.cs` | per vessel (the active one, and any other one `contact_v1` names, for 3000 master ticks after the call): the master tick of the first contact of each leg and of the vessel, all legs, `Vessel.Landed`, the LANDED situation, collisions, bounces; transitions also on the event bus V2 | every tick, in the pump after the V3 capture (the observer catches its exceptions) |
| AttitudeV1 | Control | `AttitudeV1*.cs`, `AttitudeService.cs`, `AttitudeGeometry.cs` (also the shared RCS and cross-product helpers V3 and Ext use) | the attitude loop at the physics rate: measures the rotational state and realized torques, builds the effectiveness columns, runs the law, writes the channels it is armed for | only once armed |
| Law and allocation | Model | `AttitudeMath.cs` (PURE) | square-root reference + pseudo-control hedging, torque-form INDI, bounded WLS allocation | — |
| Write rules | Model | `AttitudeWrite.cs` (PURE) | blocking and informational motifs, per-axis guards, roll command, fly-by-wire chain check, options of an arming | — |
| Lateral force | Model, Control | `AttitudeTranslation.cs` (PURE), `AttitudeV1Translation.cs` | differential thrust, common gimbal deflection, computed levers, honest translation with an aging evidence of the measured share | only when asked |
| Multi-pivot gimbals | Model | `GimbalPivots.cs` (PURE) | per-pivot mapping of gimbal modules with several non-parallel pivots | only in mode 1/2 |
| Families (lot 1) | Control | `AttitudeV1Families.cs` | per-family blocking: model motifs sanction the column, control surfaces held neutral (`deflectionDirection`, never persisted) while the loop writes pitch/yaw | only with option bit 1 |
| SurfaceModelV1 (lot 3, 3.1) | Model, Control | `SurfaceModel.cs` (PURE), `AttitudeV1Surfaces.cs` | the exact replica of each stock control surface (command law, actuator, `GetLiftVector`/`GetDragVector`), captured at `BetterLateThanNever` against the module's own values (identity, which step it answers, realized torque, exact column and gain, mono-flight scale guard that counts only the steps predicted to the replica's precision, a surface moving in Unity's rotation band excluded); armed, a surface is engaged only once every body axis its column contributes to is proven by that guard in the arming, and only while the loop OWNS every axis its column and its command reach (written by its own fly-by-wire link at the previous step, still permitted: an earlier change), and the engaged surfaces' sub-columns join the ctrlState columns, their torque M and their free slewing torque the allocation target; the law's torque filter moved at each engagement change, so the target takes no step (`surface_model_v1`) | only with `attitude_surfaces_v1` mode 1 or 2 |
| Physical slope κ | Model, Control | `SurfaceKappa.cs` (PURE), `AttitudeV1Surfaces.cs` | per body axis, the regression of the measured I Δω̇ plus the gyroscopic increment Δ(ω × I ω) minus the engines', RCS' and wheels' realized increments on the surfaces' deflection-driven torque increment (law's unit) and the rest of the vessel's aerodynamic increment (read as kRPC's `Flight.aerodynamic_torque`), through the law's H(z); batches of the filter's memory, jackknife standard error, Student at 1e-3; proven inside the proof band [0.707, 1.414], out of tolerance (sticky) outside it on the jackknife's own degrees of freedom; armed, an engagement condition (`surface_kappa_v1`, schema 4); with the mode flag 16, a step counts on an axis only while the surfaces carry 2/3 of the realized excited increment (the leak gate, \|β\| ≤ 1/2); with the mode flag 32, the measured side on the physical inertia (`Vessel.MOI` plus the parallel-axis terms its stock sum leaves out at the parts' `CoMOffset`, published per axis) | only with `attitude_surfaces_v1` mode ORed with 8 (16: the leak gate; 32: the physical inertia) |
| Roll excitation of κ | Model, Control | `SurfaceKappaRoll.cs` (PURE), `AttitudeV1Surfaces.cs` | in the coast window the client opens, a square wave added to the AutoPilot's `ctrlState.roll` by the loop's fly-by-wire link while roll is unproven: amplitude from the information missing for the proof band, spread over N_h batches (Student within the √2 gain margin), bounded by the surfaces' one-step travel; only while the surfaces carry two thirds of the excited roll torque (leak ratio ≤ 1/2: the collinear bias of inertia and wheel/RCS errors bounded); stops at the proof, an out of tolerance, the window's end (`attitude_surfaces_roll_v1`) | only with `attitude_surfaces_roll_v1` (window), shadow mode 13 |
| Engine allocation (step 1) | Model, Control | `EngineAllocation.cs` (PURE), `AttitudeV1Allocation.cs` | IN SHADOW: one allocation of every actuator (the throttle of each engine, the gimbal axes, the ctrlState rotation and X/Y) on the force and torque rows, strict priorities (D's axial force, pitch/yaw, the live lateral demand, roll), the realized change of each actuator at a common horizon (its stock Lerp), a joint solve then a cascade of box least squares through a 6 x 6 Woodbury system; publishes the commands, residuals and the lateral authority, writes nothing (`attitude_allocation_v1`, `attitude_allocation_frames_v1`) | only with option 4096 |
| Engine allocation writing (step 2) | Model, Control | `EngineAllocation.cs` (`AllocationWriteRule`, `SolveDeadZone`: PURE), `AttitudeV1Allocation.cs` | WRITES in place of the law's allocation and the lateral-force layer on every step the pitch/yaw channel writes and its solve is sound: gimbals leased at Earlyish, the throttle of every engine (independent, under the loop's lease) and ctrlState X / Y at the fly-by-wire, the ctrlState rotation through the law's path; the RCS dead zone in the command; the free authority along the live demand for the PF; a fallback (not solved, not converged, saturated, stale, lease, gimbal missing, exception) hands the step to the law and the layer from the realized state, its own throttle offsets and X / Y decaying at exp(-1/H) a step, the hand taken back after ceil(H) sound solves | only with option 8192 |
| RCS as ModuleRCS fires it | Model, Control | `RcsCombined.cs` (PURE), `AttitudeV1RcsCombined.cs` | the nozzles of every RCS module on ONE rotation vector (Update per component, FixedUpdate per nozzle, CalculateThrust threshold, fullThrust): option 32's in-flight torque by that rule; at the roll's write the ctrlState command the RCS answers (the law's, its roll held or its pitch/yaw held, nearest its INDI's expectation in its own WLS metric, a part's waiting counted), the part held written just under the RCS dead zone; the law untouched | only with option 16384 |
| Effector registry (R1) | Model, Control | `EffectorRegistry.cs` (PURE), `AttitudeV1Effectors.cs` | a model status per allocation column (exact, measured, approximate, unmodeled), a family sanctioned only on its own columns (a ctrlState axis frozen only for what is unmodeled on it; RCS precision mode scaled, exact), the authority of each family computed every step (`attitude_effectors_read_v1`); implies option 1's mechanics | only with `attitude_effectors_v1` mode 1 |

Born closed: AttitudeV1 does nothing until `attitude_arm_v1/v2`; every option, translation
mode, pivot mode, the effector registry, SurfaceModelV1, `DynamicsExtV1` and tracking starts off. Always on: the V3 and AeroActuator
capture, the reflection cache, the gimbal rate ramp (QW5).

The **pure** files (System only, the Model DLL) are compiled directly into
`build/tests/AttitudeTests.csproj` and tested without KSP; the same tests write
`build/tests/golden/attitude_v1_golden.json`, which the client's mirror of the law
replays operation by operation. The rest of Control and Telemetry (the loop, the geometry, the
Dynamics) is proven on the ground by the client's own ground checks, and in flight.

### FMRS

A wrapper of FMRS Continued by reflection (`FmrsApi`), plus, since lot 1, the dropped-stage
table: `DroppedStagesCapture` records each staging and each separation as GameEvents fire
(`onStageActivate`, `onPartDeCoupleNewVesselComplete`, `onVesselsUndocking`,
`onGameStateSaved`, `onLaunch`) and joins them to FMRS's own table (`dropped_stages_v1`,
`ready_v1`). The ledger and its wire format (`DroppedStages.cs`) are pure and tested by
`build/tests/fmrs`. The capture hooks GameEvents; FMRS's table is diffed only for 10 s of game
time after a separation and after a save write.

### Stock and Tower: no Core on purpose

Both declare no `KSPAssemblyDependency` on the Core and use nothing from it, so a Core
change never forces their redeployment and a broken Core never takes them down. Both are
nevertheless in the solution, built, scanned and stamped by `build.cmd` like the others.

`TowerAnchor` is **always on, without a flag**: every loaded, unpacked, landed vessel whose
root part is `TG.TowerBase` gets a `FixedJoint` to the world on its first unpacked tick
(stock clamps rocked the tower by 1–4.5°). Every capture flight depends on it.

Tower's first published layout, `hinge_lock_at_rest_v1` (the status of the hinge lock at rest),
still reaches `conn.bridge.layout_v1`: `TowerLayouts` (Instantly, Awake only) finds the Core's
`LayoutRegistry` by name among the loaded assemblies and declares it by reflection
(`TowerLayoutDeclaration`, pure, tested against the real registry in `build/tests/tower`). No
Core, or one older than R1: nothing declared, one log line, every RPC unchanged. The hinge
watchers (`HingeRestLocks`) are stepped from `TowerAddon.FixedUpdate`, hook no GameEvent, end
when their vessel unloads or packs and are forgotten in `TowerAddon.OnDestroy`.

`conn.tower_site` (`TowerSiteService`, `TowerSite`) is the same DLL's second service, the towers
of a game seen from the Space Center (and Flight, Tracking Station): no `SpaceCenter` type in its
signatures (persistentIds as decimal strings), so that code does not depend on kRPC's
`SpaceCenter` assembly, which the DLL still references for `conn.tower`. It lists the towers from
`FlightGlobals.Vessels` (in a scene with a planetarium KSP loads every saved vessel there, and the
save rebuilds the flight state from it) and puts one on the ground at the Space Center the way KSP
itself makes a saved vessel: the craft loaded as a `ShipConstruct`, its parts snapshotted, a LANDED
vessel node handed to `Game.AddVessel`, the construct destroyed, the game saved. The tower is loaded
at the next flight by the ordinary path of a landed vessel and anchored by `TowerAnchor`. Its pure
parts are `TowerSiteGeometry` (the rotation from latitude, longitude and heading; the altitude from
the colliders' footprints over the terrain) and `TowerSiteFormat` (the two text tables, declared
with the hinge lock's layout), tested by `build/tests/tower`. It never runs per frame, except the
tower bubbles of flight (`tower_bubble_v1`): `TowerBubbles`, stepped from
`TowerAddon.FixedUpdate` while one is held, writes the tower's own `VesselRanges` so that KSP loads it
packed first and unpacks it only once every part has started and its pose is still
(`TowerBubbleWatch`, pure, tested by `build/tests/tower`); forgotten, original ranges written back,
in `TowerAddon.OnDestroy`. And the ground joints of a tower (`tower_ground_v1`): `TowerGround`, stepped
from the same `FixedUpdate` while one is posed, fixes to the world the parts Kerbal Joint Reinforcement would
fix near the floating origin and the hinges' fixed halves, never an arm (`TowerGroundWatch`, pure, tested by
`build/tests/tower`), and makes them again inside `onFloatingOriginShift` (subscribed by `TowerAddon`);
destroyed in `TowerAddon.OnDestroy`.

### Versions, three kinds

| Kind | Where | Read by |
|---|---|---|
| Load-order version | `[assembly: KSPAssembly ("KRPC.Bridge.Core", 1, 0)]`, `KSPAssemblyDependency` in each plugin | KSP's `AssemblyLoader` |
| Wire layouts | `internal const` protocol and schema numbers per layout (`AttitudeV1.ProtocolVersion`, `DynamicsV3.SchemaVersion`, `DroppedStagesFormat.Schema`...), also in each layout's header | the client's decoders, and `manifest_v1` |
| Build | `AssemblyVersion` (the mod version, same in every DLL, checked by `check_docs.py`), and the build stamp (commit, clean, date) | `manifest_v1`, `manifest.json`, `KSP.log` |

Bump the minor of the Core's `KSPAssembly` when a plugin needs a new Core member, the major
only on a breaking change (and every plugin's dependency with it). A new versioned RPC gets
a row in `BridgeManifest.Known`, which names the constants that version it; `build/scan`
fails the build if a row names a constant that does not exist.

---

## Build and trace

### Build

`build.cmd "<KSP>"` runs, in order:

1. type-check of every Core-dependent source against the stubs (no KSP needed);
2. the pure tests: AttitudeV1 (and its golden file), the FMRS table, the manifest;
3. **the stamp**: `tools/build_stamp.py --write obj` records the commit, whether the tree is
   clean (any change git reports, untracked files included)
   and the commit's date, in `obj/BridgeStamp.g.cs` and `obj/BridgeStamp.json`. Taken once,
   after the tests (so a stale golden file shows as a dirty tree) and before the compile;
4. the solution, every project compiling the same stamp file (`-p:BridgeStampFile=...`):
   `AssemblyInformationalVersion "<mod version>+<sha>[.dirty]"`, `AssemblyMetadata`
   `KRPC.Bridge.GitSha`, `GitDirty`, `SourceDate`. LICENSE, NOTICE and the `.version` file are
   copied next to the DLLs, and `distribution/TowerCatch` (with LICENSE) into `dist/GameData/TowerCatch`;
5. kRPC's own signature scanner on every DLL of `dist/` (the nine must be there), then the
   Core's manifest code run on the same DLLs: every DLL stamped with one commit, every known
   layout resolved;
6. `tools/manifest.py` writes `dist/GameData/KRPC.Bridge/manifest.json` and
   `dist/GameData/TowerCatch/manifest.json`: the sha256 and size of every file, the commit,
   `git_dirty`, and `deliverable` (clean tree). It refuses a DLL that does not carry this
   build's commit, a file the build does not produce (a patch or a craft in the bridge's folder
   included), a shipped extra whose bytes differ from the repo's, and a tree that moved during the
   build.

**Builds are deterministic** (`<Deterministic>true</Deterministic>`, no PDB, no time in the
stamp): two builds of one commit give the same bytes, `manifest.json` included. That is what
lets a tree be proven to be what flies without flying it. A plain `dotnet build` of one
project stamps it through the same script, into its own `obj/`.

### Trace

- `conn.bridge.manifest_v1()` reads the stamps back from the assemblies the game loaded:
  `consistent=1` means one clean build is loaded, `git_sha=mixed` an old DLL among new ones.
- `manifest.json`, shipped next to the DLLs, gives the commit and the sha256 of every file.
- The Core logs `core starting (assembly 2.0.0.0, commit <sha>, <date>)` in `KSP.log`.
- `python/check_bridge.py` prints the manifest and compares it with `manifest.json`.

### CI

`.github/workflows/build.yml` runs on every branch and pull request: the stub type-check,
the pure tests (AttitudeV1, FMRS, manifest, Core v2, Tower, ContactV1, staging locks), the
scanner's compile, the unit tests of the build tools, and `check_docs.py`. `build.cmd verify`
runs the same steps locally. CI cannot run the signature scan or build real DLLs: that needs the
game's assemblies, which may not be redistributed.

---

## Keeping a bad signature out of GameData

Given fact 3, all robustness has to happen before deployment.

**1. Run kRPC's own scanner at build time.** `build/scan` loads the built DLLs and calls
`KRPC.Service.Scanner.Scanner.GetServices(errors)` — the same validation the server
performs, in about a second. Passing the error list makes it collect every bad signature
instead of throwing on the first. `build.cmd` runs it on all eleven DLLs and refuses to go on
if one is missing from `dist/` (which it empties first: a DLL renamed away must not linger).

**2. Type-check every source file against the stubs.** `build/verify` uses a glob, not a
list of files: forgetting to add a new plugin to a list is precisely the path by which an
unchecked source reaches `GameData`. Tower and Stock, which have no Core dependency,
are excluded from it and checked by the real build and the scan only.

**3. Know the signature rules.** Legal parameter and return types are: `bool`, `int`,
`long`, `uint`, `ulong`, `float`, `double`, `string`, `byte[]`, a `[KRPCClass]` type, a
`[KRPCEnum]` type with `int` backing, `KRPC.Service.Messages.IMessage`, and
`IList`/`IDictionary`/`HashSet`/`System.Tuple` of those. Dictionary keys are further
restricted to `int`, `long`, `uint`, `ulong`, `bool`, `string`. Return `List`, never an
array.

   Invalid: `Vector3`, `Quaternion`, `Guid`, `DateTime`, `object`, `byte`, `short`,
   `decimal`, `IEnumerable<T>`, `ISet<T>`, arrays other than `byte[]`, an enum without
   `[KRPCEnum]`, and C# `ValueTuple`, i.e. `(int, int)` syntax. `Nullable<T>` is illegal in
   kRPC 0.6.0.

   Identifiers must match `^[A-Z][A-Za-z0-9]*$`. Leave `Id` unset on `[KRPCService]`.

**4. `check_docs.py`** catches the cheap subset in CI: illegal types in signatures, XML doc
comments that would not parse, a member missing from `docs/API.md` or documented but gone,
and version numbers that disagree.

### XML documentation is part of the signature surface

kRPC reads the `.xml` next to each DLL and passes it to the client. Malformed XML, or a
`cref` that cannot be resolved, throws during scanning and therefore **also disables the
server**. The Python client understands only `<summary>`, `<param>`, `<returns>` and
`<remarks>` at the top level, and `<list>` items must wrap their content:
`<item><description>x</description></item>`.

---

## Performance

**An RPC costs about a millisecond round trip, and kRPC bounds how many it runs per
`FixedUpdate`.** Anything you would loop in Python must be looped in C# and returned flat:
the dynamics snapshots return one `list[float]` per tick, not objects.

**Do not return `[KRPCClass]` in volume.** Forty records with six fields each is 240 RPCs
as handles and 1 as packed rows. Nothing in the Core returns a `KRPCClass`.

**Resolve reflection once, at load**, and compile a delegate for anything read every tick
(`ModRegistry.StaticGetter<T>`, `InstanceGetter<T>`, Telemetry's `ReflectionCache`).

**Any property can be streamed, so keep every getter cheap**, and cache what cannot change
during a session (`plugins`, `manifest_v1`, MechJeb's name lists).

**What runs every tick.** Core: the game clock (one `ObscenelyEarly` callback, one check in
`FixedUpdate`), `Ticks++` and the queue pump. Control: lease upkeep and the pump, which calls
Telemetry's V3 and AeroActuator capture and, since lot 3, the contact watch (one pass over the
parts and wheels of each watched vessel, the active one at least); AttitudeV1 when armed, and its
SurfaceModelV1 capture at `BetterLateThanNever` when `attitude_surfaces_v1` is on (five replica
force evaluations per surface, compiled field readers, nothing allocated; with an earlier change's flag one
more per surface and one pass over the vessel's parts for its aerodynamic torque, its cost
published). Tower: the anchor check, the hinge locks at rest
while one is armed (one reflective angle read each, a boxed float), and a held tower bubble (its
ranges written, its parts and servos counted; while its load is watched, one root-relative position
per rigid body into a preallocated array). FMRS: its watcher every
fifth tick. Everything else runs on call.

**Allocations per physics step** (Mono's GC is not generational: every allocation ends up in a
full mark). Removed in R1: the AttitudeV1 and lateral-force frames and their LinkedList nodes,
the V3 frame, state array and row lists, the 42 name arrays of the V3 reflective reads and the
AeroActuatorV1 ones, `served`, the RCS, wheel and surface scratch arrays of the geometry, the
pump's two lists, TowerAnchor's set and list. Lot 3's contact watch allocates nothing per step
(preallocated latch arrays and wheel lists; `build/tests/contact` measures it) and the event
bus V2 reuses its 1024 record objects. Left: the V3 reflective vector reads
(`double[3]` each), the kRPC live-aero closures and tuples (about eleven per step), the
DynamicsExtV1 rows (born closed), the per-call RPC returns (`List<double>`, the wire needs one).

---

## Adding a plugin

1. Copy `src/Plugins/Template/` to `src/Plugins/<Name>/`.
2. Change `AssemblyName`, `RootNamespace`, and the names in `AssemblyInfo.cs` (version: the
   mod's, as in every other `AssemblyInfo.cs`).
3. `dotnet sln KRPC.Bridge.sln add src\Plugins\<Name>\KRPC.Bridge.<Name>.csproj`
4. Add the DLL to the list in `build.cmd` (step 4) and to `EXPECTED` in `tools/package.py`
   (which `tools/manifest.py` also reads).
5. Document every member in `docs/API.md`; give each versioned layout a row in
   `BridgeManifest.Known`, declare its descriptor into `LayoutRegistry` (checked against the
   constants that size it) and its masks into `BitRegistry`, at load. Use the Core v2 types
   (`Lease`, `Recorder`, `GameClock`, `VesselRegistry`) rather than a copy, SI units, and
   `BridgeErrors` codes for refusals.
6. `.\build.cmd verify`, then `.\build.cmd "<ksp>"`; `.\build.cmd deploy` copies the result into the game.

Never add a hard `<Reference>` to the mod you are wrapping. Resolve it out of
`AssemblyLoader.loadedAssemblies`: a hard reference means your plugin fails to load
entirely when the mod is absent; reflection means it loads and reports
`available = false`. Assembly names drift — FMRS ships as `FMRSContinued` while its folder
is still `FMRS` — so pass every historical name as a candidate. `conn.bridge.describe_type()`
and `conn.discovery` tell you what a mod's type really has, from a running game.

---

## Known limits

Recorded so that nobody mistakes them for design:

- Telemetry reads Control's internals (`InternalsVisibleTo`): a change of that internal surface
  redeploys both. Some duplications were left because merging them would change bytes: the
  gimbal and RCS effectiveness columns of AttitudeV1 against the V3 rows, the engine torque of
  AttitudeV1 against V3's realized wrench, and the aero of `AeroWrenchBatch` against
  `conn.stock.aero_force_v1` (different frames and inputs; `docs/API.md` says which to use).
- The frames carry the master tick (refactor R3) next to their three old tick counters (pump,
  attitude, fixedTime), which keep their values: renumbering them would change their bytes.
- The topology fingerprint is walked once per vessel per window, so twice per step when a
  vessel is armed (Earlyish and the pump): one per step waits for `unevented_changes = 0`
  (the flight of 06/10 14h47: zero in both windows, but no staging in it; not enough).
- Every lease still expires on the wall clock only; the physics-step budget of `Lease` is
  unused until a contract asks for it.
- Vessel identity: every service now accepts a `persistentId` where it means a vessel
  (refactor R3: Control v2, Stock and Tower `_v2`, Trajectories `vessel_id_v1` /
  `impact_geo_for_v1`, AeroActuator frames name their vessel), but `dynamics_snapshot_v3` and
  the v2 controller still follow the ACTIVE vessel; a non-active booster reads its own state
  through `track_vessel_v3`, and Trajectories predicts the active vessel only.
- SurfaceModelV1 assumes, for its armed mode, that a surface answers the ctrlState of the SAME
  step's fly-by-wire; it MEASURES it (the targets, and a fly-by-wire probe of `liftForce`) and
  does not engage until it is established. If KSP runs the parts' `FixedUpdate` before
  `FlightInputHandler.FixedUpdate`, the surfaces stay neutralized (a two-step model would be
  needed). Its surfaces share the ctrlState variable with the RCS and the wheels: separate
  sub-columns, not separate allocation variables (no stock way to command one surface alone).
  For the same reason a surface is engaged as a whole, never on one axis: it waits until every
  body axis its column contributes to is proven by the scale guard in the arming. The
  proof needs excitation: a surface held neutral gives evidence only while it slews back, so it
  comes from the steps the AutoPilot flies the surfaces in the same arming (a re-arming starts
  from nothing), and an axis nothing excites — the roll of a booster whose fins only answer
  pitch and yaw commands that cancel on roll — keeps every surface with a roll share neutral.
  an earlier change's κ needs the same excitation and more of it (batches, not steps): armed, the surfaces not
  engaged are held at rest, so κ is proven in shadow first and the mode then goes to armed (the
  evidence is kept); a surface disengaged by κ stays neutral and gathers no new evidence. κ
  reads the vessel's aerodynamic torque the way kRPC does (FAR would need its own readout) and
  leaves out every step with an engine torque in the filter's memory.
- `attitude_translation_v1` still returns layout 1, 2 or 3 depending on the mode;
  `attitude_translation_v2` is its mode-independent return.
- A few error messages of the old Actuators code (now Control and Telemetry) and of the Core
  are still in French: they are wire bytes clients may match. Identifiers, services, comments
  of the moved code and new code are in English.
