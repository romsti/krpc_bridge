# Scripting guide

How to drive KRPC.Bridge from a Python script. Assumes you already know kRPC; if you do
not, start with the [kRPC tutorial](https://krpc.github.io/krpc/tutorials.html) and come
back.

Everything here is plain `krpc` (`pip install "krpc>=0.6,<0.7"`) — this mod ships no
client library of its own, and there is nothing to import beyond `krpc` itself. The
bridge adds `conn.bridge`, `conn.fmrs`, `conn.ocisly` and `conn.mech_jeb` to the
connection you already have, and `help(conn.fmrs)` in a REPL gives you the reference.

**Contents**

- [Connect and check](#connect-and-check)
- [The one rule: handles die on a jump](#the-one-rule-handles-die-on-a-jump)
- [A complete booster recovery](#a-complete-booster-recovery)
- [Reacting to separation without polling](#reacting-to-separation-without-polling)
- [Grouping stages that came off together](#grouping-stages-that-came-off-together)
- [Replaying a landing](#replaying-a-landing)
- [Reading what a recovery earned](#reading-what-a-recovery-earned)
- [Ascent with MechJeb and manual staging](#ascent-with-mechjeb-and-manual-staging)
- [Driving the rest of MechJeb](#driving-the-rest-of-mechjeb)
- [Planning a burn](#planning-a-burn)
- [Cameras](#cameras)
- [Writing your own plugin](#writing-your-own-plugin)
- [When it does not work](#when-it-does-not-work)

---

## Connect and check

```python
import krpc

conn = krpc.connect(name="campaign")

for row in conn.bridge.plugins:
    name, ok, version, mod_version, report = row.split("\t")
    print(f"{name:8} {'ok' if ok == '1' else 'MISSING':8} {mod_version:10} {report}")
```

```
FMRS     ok       1.2.9.6    jump[guid,bool]=True jump[string]=True ...
OCISLY   ok       1.0.114.0  assembly=OfCourseIStillLoveYou ns=<global> ...
MechJeb  MISSING             no loaded assembly matches 'MechJeb' - mod not installed?
```

Write the guard once and mean it. Almost every member of an unavailable service raises
`RuntimeError`, and doing that check at the top turns a mid-flight failure into a one-line
exit. The exceptions are deliberate: `ping()` still answers, and `fmrs.active`,
`fmrs.jump_in_progress` and `mech_jeb.on_vessel` return `False` rather than throwing — so
those three will not tell you the mod is missing.

```python
def require(service, name):
    if not service.available:
        raise SystemExit(f"{name} unavailable: {service.diagnostics}")

require(conn.fmrs, "FMRS")
```

---

## The one rule: handles die on a jump

An FMRS jump reloads the entire flight scene. Every kRPC object you are holding — the
vessel, its parts, its modules, its control — refers to something that no longer exists,
and every stream you opened is reading from it.

```python
# WRONG - vessel is a dead handle after the jump
vessel = conn.space_center.active_vessel
altitude = conn.add_stream(getattr, vessel.flight(), "mean_altitude")
conn.fmrs.jump_to_vessel(vid)
print(altitude())            # raises, or worse, returns stale nonsense
```

```python
# RIGHT - tear down first, rebuild after
altitude.remove()
conn.fmrs.jump_to_vessel(vid)
vessel = conn.space_center.active_vessel
altitude = conn.add_stream(getattr, vessel.flight(), "mean_altitude")
```

A small helper is worth writing once:

```python
class Handles:
    """Streams that must not survive a scene reload."""
    def __init__(self):
        self._streams = []

    def stream(self, conn, *args):
        s = conn.add_stream(*args)
        self._streams.append(s)
        return s

    def drop(self):
        for s in self._streams:
            try:
                s.remove()
            except Exception:
                pass          # already dead if the scene went first
        self._streams.clear()
```

`jump_to_vessel` blocks for the whole reload — typically 5 to 20 seconds depending on part
count — and returns only once the target vessel is active, unpacked, and has been settled
for 25 physics ticks. You do not need to sleep afterwards.

---

## A complete booster recovery

The shape of a reusable-booster flight, start to finish.

```python
import krpc

conn = krpc.connect(name="booster")
sc, fmrs = conn.space_center, conn.fmrs

if not fmrs.available:
    raise SystemExit(fmrs.diagnostics)

# --- before launch -------------------------------------------------------
# armed is read exactly once, at the top of FMRS's launch routine, so it must
# be true before the first staging.
fmrs.armed = True

# If the booster has parachutes but no probe core, FMRS ignores it by default.
fmrs.track_parachutes = True

vessel = sc.active_vessel
vessel.control.activate_next_stage()          # launch

# --- fly the ascent, stage, whatever your mission needs -------------------
...

# --- wait for FMRS to register the dropped stage --------------------------
# Block on the event rather than polling: the tick FMRS writes its save file.
evt = conn.bridge.on_event("fmrs.dropped")
with evt.condition:
    evt.wait(timeout=600)
evt.remove()

dropped = fmrs.dropped_vessels
print("FMRS is tracking:", dropped)

# --- carry on flying the upper stage, then hand off -----------------------
...

booster_id, booster_name = next(iter(dropped.items()))
print(f"jumping to {booster_name}")
fmrs.jump_to_vessel(booster_id, save_landed=True)

# --- everything from before is dead ---------------------------------------
booster = sc.active_vessel
assert fmrs.switched_to_dropped

# fly the boostback, entry burn and landing on `booster` ...

# --- recover and go back --------------------------------------------------
fmrs.recover_current()
print(f"recovered for {fmrs.recovered_funds():.0f} funds")

fmrs.jump_to_main()                # ALWAYS before doing anything else
assert not fmrs.switched_to_dropped
```

That last assertion is not decoration. FMRS hooks `onGameSceneLoadRequested`: leaving the
flight scene while `switched_to_dropped` is true makes it force-load the main mission a
few dozen frames later, dragging the game back into flight in the middle of whatever your
script had moved on to. Revert, recover, launch — none of them are safe until you are back
on the main mission.

---

## Reacting to separation without polling

Separation is an instant, and an instant is the one thing a kRPC stream cannot report: a
stream samples a value once per update and tells you what something *is*. Polling
`dropped_vessels` in a Python loop costs a round trip per iteration and resolves the moment
to however long you sleep.

The bridge diffs FMRS's table on the C# side and publishes an event.

```python
last = 0
launched_at = conn.fmrs.launched_at          # ut is absolute; this makes it T+
evt = conn.bridge.on_event("fmrs.dropped,fmrs.forgotten,part.die")

while flying:
    with evt.condition:
        evt.wait(timeout=1.0)
    # Fall through whether the event fired or the timeout expired. poll_events
    # returns [] when there is nothing new, so the timeout costs one cheap call
    # and nothing else -- and a wakeup that races the poll is not missed.
    for row in conn.bridge.poll_events(last):
        eid, kind, ut, realtime, vessel_id, detail = row.split("\t")
        last = int(eid)
        met = float(ut) - launched_at
        if kind == "fmrs.dropped":
            name, _, save = detail.partition(" | save=")
            print(f"t+{met:.1f}  separated: {name}  save={save}  ({vessel_id})")
        elif kind == "part.die":
            print(f"t+{met:.1f}  lost part: {detail}")
```

**Do not branch on what `wait()` returns.** The kRPC Python client declares it as returning
`None` in every case, timeout included, so `if not evt.wait(timeout=1.0): continue` is
always true and the poll below it never runs. Wait, then poll unconditionally.

Use both together. The event gives you latency: it fires the tick something happens. The
log gives you completeness: it is an ordered, id-numbered record, so two events inside one
tick — which the signal would coalesce — both appear. Track the last id and nothing is
ever lost between polls.

`conn.bridge.events_recorded` is the total this session including records aged out of the
512-entry ring. If it has run ahead of your last id by more than 512, you polled too
slowly and dropped some.

`part.die` and `part.crash` are timestamped to the millisecond, which gives you the exact
instant of a landing failure instead of having to infer it afterwards from a telemetry
stream.

---

## Grouping stages that came off together

A launcher that sheds two side boosters and then a core has all three listed in
`dropped_vessels` at once. Which two were the pair?

Not "the two that appeared in the same poll" — that is a guess about your poll timing.
FMRS writes **one save per staging event**, so stages that separated together carry the
same value in `dropped_saves`.

```python
from collections import defaultdict

saves = conn.fmrs.dropped_saves        # vessel id -> save name
names = conn.fmrs.dropped_vessels      # vessel id -> vessel name
times = conn.fmrs.separation_times     # save name -> UT, as a string

batches = defaultdict(list)
for vessel_id, save in saves.items():
    batches[save].append(vessel_id)

for save, ids in sorted(batches.items(), key=lambda kv: float(times.get(kv[0], 0))):
    ut = float(times.get(save, 0))
    print(f"{save:22} t={ut:10.1f}  {[names[i] for i in ids]}")
```

```
FMRS_save_3            t=  187493.2  ['Falcon Booster', 'Falcon Booster']
FMRS_save_2            t=  187611.8  ['Falcon Core']
```

Note the two identical names. They are two different vessels built from the same craft
file, with the same part count and the same mass — nothing but the id tells them apart,
which is why `dropped_persistent_ids` exists:

```python
pids = conn.fmrs.dropped_persistent_ids      # FMRS id -> KSP persistentId
by_pid = {v.persistent_id: v for v in conn.space_center.vessels}

for vessel_id in batches["FMRS_save_3"]:
    pid = pids.get(vessel_id)
    if pid is None:
        continue                  # KSP no longer has this vessel at all -
                                  # destroyed or recovered. Retrying will not help.
    vessel = by_pid.get(int(pid))
    print(vessel.name, vessel.mass)
```

FMRS speaks in KSP `Guid`s and kRPC's `Vessel` exposes no Guid at all, so `persistentId` is
the only identity both sides can see. An unloaded stage is absent from that dictionary
rather than reported as zero — absent means "ask again", zero would look like an answer.

---

## Replaying a landing

Jumping to the stage you are **already flying** is FMRS's "return to separation": it reloads
the save frozen at the moment of separation. The same landing can be attempted repeatedly
from bit-identical initial conditions, without re-flying the ascent.

```python
for attempt, gain in enumerate([0.8, 1.0, 1.2, 1.5]):
    fmrs.jump_to_vessel(booster_id, save_landed=False)   # note: False
    booster = sc.active_vessel

    result = fly_the_landing(conn, booster, gain=gain)
    print(f"attempt {attempt}: gain={gain} -> {result}")

    if fmrs.vessel_state(booster_id) == "RECOVERED":
        # something recovered it anyway; put it back so the series can continue
        fmrs.set_vessel_state(booster_id, "LANDED")
```

`save_landed=False` is right here and `True` is wrong, for two reasons. There is no
main-mission state to preserve — and `True` is the argument that reaches FMRS's recovery
path. Passing it while the booster is already down on Kerbin banks the stage mid-series:
funds credited, state `RECOVERED`, and FMRS's own window stops offering it. The flight
itself would still work, but the accounting would be wrong and you would not notice.

**You do not need to touch `auto_recover` for this.** Re-jumping to the stage you are
already flying passes `False`, which never reaches the recovery path at all — the loop is
recovery-free by construction. FMRS has no landing handler; recovery only ever fires when
you *leave* a dropped stage.

---

## Reading what a recovery earned

```python
fmrs.recover_current()

for row in fmrs.recovery_report():
    category, key, value = row.split("\t")
    if category == "fund":
        print(f"refunded {float(value):.0f}")
    elif category == "science_sent":
        print(f"science: {value} from {key.split('@')[0]}")
    elif category == "kerbal":
        print(f"lost a kerbal, {value} reputation")
```

This is the only place the outcome of a booster recovery is stated as a number. FMRS keeps
the ledger on disk so the main mission — resumed from a save written *before* the recovery
happened — can have it applied on return; read it after recovering and before jumping back.

`recovered_funds()` sums the `fund` rows for you.

---

## Ascent with MechJeb and manual staging

The case that is hard to get any other way. MechJeb decides to stage when the current stage
has no active engines left. On a launcher whose side boosters and core share a stage, that
stays false while the core burns, so the boosters are never dropped.

```python
mj = conn.mech_jeb
if not mj.available:
    raise SystemExit(mj.diagnostics)
if not mj.on_vessel:
    raise SystemExit("no running MechJeb part on this craft - add an AR202")

# What does THIS build actually call things? Never trust a name from a changelog.
print("\n".join(mj.ascent_setting_names))

mj.ascent_path = "CLASSIC"
mj.set_ascent_setting("DesiredOrbitAltitude", 120000)    # SI: metres, not km
mj.set_ascent_setting("DesiredInclination", 0)
mj.set_ascent_setting("TurnStartAltitude", 2000)
mj.set_ascent_setting("TurnEndAltitude", 60000)

mj.autostage = False          # BEFORE engaging, not after
mj.ascent_enabled = True
assert mj.staging_users == 0, "MechJeb is still registered with staging"
# Non-zero is not automatically a failure -- MechJeb's own windows and other
# autopilots join the same pool. On a bare scripted launch it should be 0.
```

Now the staging decision is yours. A criterion that works for a shared-stage launcher is a
relative drop in total thrust:

```python
import time

vessel = conn.space_center.active_vessel
thrust = conn.add_stream(getattr, vessel, "thrust")

peak = 0.0
while vessel.available_thrust > 0:
    now = thrust()
    peak = max(peak, now)
    if peak > 0 and now < 0.6 * peak:      # boosters burnt out
        vessel.control.activate_next_stage()
        peak = 0.0
        time.sleep(1.0)                     # let the new stage light
    time.sleep(0.1)
```

**Order matters, and `staging_users` is how you verify it.** The autopilot reads
`autostage` as it is enabled, to decide whether to register with the staging controller at
all — flipping it afterwards leaves a window in which MechJeb may stage on its own. Zero
users means nothing is asking MechJeb to stage, which is the state you wanted; `-1` means
the pool could not be read, which is a different problem.

**And it is a one-way switch.** `mj.autostage = True` later in the flight will not give
staging back. MechJeb's setter re-registers only under a condition MechJeb itself never
makes true, so that path is dead code — the flag flips and nothing else happens. If you
need MechJeb staging again mid-flight, cycle the autopilot:

```python
mj.autostage = True
mj.ascent_enabled = False
mj.ascent_enabled = True          # registration runs here, not on the line above
assert mj.staging_users > 0
```

**If your side boosters are solids, none of this is necessary.** MechJeb's staging
controller has a `DropSolids` mode that skips the active-engine test for throttle-locked
engines, so it will drop solid boosters while the core burns. The problem this section
solves is specific to *liquid* side boosters sharing a stage with the core.

**Disengaging politely.** `mj.ascent_enabled = False` withdraws only your script's request.
If the player also clicked Engage in MechJeb's window, their request stands and the
autopilot keeps flying. `mj.disengage_ascent()` clears the pool outright — use that on a
cleanup path, where leaving an autopilot engaged would carry into the next flight.

---

## Driving the rest of MechJeb

The ascent members above are the ones with names of their own. Everything else in MechJeb —
twenty other modules, roughly two hundred settings — goes through five accessors instead,
and the first thing to call is the one that tells you what is there.

```python
mj = conn.mech_jeb
print(mj.modules)
# ['Ascent', 'AscentSettings', 'Attitude', 'Glueball', 'Guidance', 'Hoverslam',
#  'Landing', 'Node', 'RCS', 'Rcsbal', 'Rover', 'Settings', 'SmartASS', 'Solarpanel',
#  'Spinup', 'StageStats', 'Staging', 'Target', 'Thrust', 'Warp', 'AntennaControl']

for row in mj.describe_module("Staging"):
    print(row)
# AutostageLimit             number  EditableInt          rw  persistent:GLOBAL
# AutostagePostDelay         number  EditableDouble       rw  persistent:GLOBAL
# DropSolids                 flag    Boolean              rw  persistent:GLOBAL
# FairingMinAltitude         number  EditableDoubleMult   rw  persistent:GLOBAL
# HotStaging                 flag    Boolean              rw  persistent:GLOBAL
```

**The second column tells you which accessor to use** — `number` → `setting()`, `flag` →
`flag()`, `enum` → `enum_value()`, `list` → `list_value()`, `text` → read-only. A module
this MechJeb does not publish under a short name is still reachable by its class name:
`mj.describe_module("MechJebModuleRendezvousAutopilot")`.

**The last column is the one to read twice.** `persistent:GLOBAL` means the setting is saved
for **every vessel in every save on that install**. A script that tunes an ascent parameter
for one launcher has changed the default for all of them. The bridge does not block it —
that would make it useless for the main use case — but nothing tells the player either.

Two things you could not do at all before:

```python
# Stage, but never past stage 3. The middle ground between autostage on and off.
mj.set_setting("Staging", "AutostageLimit", 3)

# Cap the ascent by dynamic pressure instead of by throttle guesswork.
mj.set_flag("Thrust", "LimitDynamicPressure", True)
mj.set_setting("Thrust", "MaxDynamicPressure", 20000)
```

**Engaging is not uniform and the differences are handled for you.** `mj.engage("Node")`
adds you to a module's user pool; `mj.disengage("Node")` withdraws, and the module keeps
running if MechJeb's own window still wants it — which is why both return the remaining
user count rather than nothing. Modules that run themselves refuse to be disengaged, and
settings bags say they are not autopilots. Three modules need a method rather than the
pool: `land_at_target()`, `execute_node()`, `smart_ass_engage()`.

**The landing predictor costs nothing to read.** It runs whenever you are in flight,
republishing about once a second, so nothing has to be engaged:

```python
import math

if mj.landing_predicted:
    print(f"impact {mj.landing_latitude:.4f}, {mj.landing_longitude:.4f}")
    print(f"ignite in {mj.ignition_countdown:.1f}s, {mj.landing_delta_v:.0f} m/s")
    print(f"slope there: {mj.landing_slope:.1f} deg")
```

Every number is `NaN` when there is no solution, so test `landing_predicted` first rather
than trusting a zero. Stock kRPC has no impact prediction of any kind, and this one
propagates through the atmosphere — it is what a boostback burn exists to null out, and an
independent check on a descent solver of your own.

---

## Planning a burn

MechJeb can compute a Hohmann transfer, a plane match, an intercept, a moon return. The
design decision that matters is what you get back: **an ordinary KSP maneuver node**.
MechJeb computes and places it, and from there you are in stock kRPC.

```python
mj = conn.mech_jeb
vessel = conn.space_center.active_vessel

print(mj.maneuver_operations)
# ['OperationApoapsis', 'OperationCircularize', 'OperationCourseCorrection',
#  'OperationEccentricity', 'OperationGeneric', 'OperationInclination', ...]

# Raise apoapsis to 200 km at the next periapsis, then circularise there.
mj.set_maneuver_time_reference("OperationApoapsis", "PERIAPSIS")
mj.set_maneuver_parameter("OperationApoapsis", "NewApA", 200000)   # SI: metres
mj.create_maneuver_nodes("OperationApoapsis")

mj.set_maneuver_time_reference("OperationCircularize", "APOAPSIS")
mj.create_maneuver_nodes("OperationCircularize")   # append=True: plans from the last node

for node in vessel.control.nodes:                  # stock kRPC from here on
    print(f"t={node.ut:.0f}  {node.delta_v:.1f} m/s  prograde {node.prograde:.1f}")
```

**Operations are named by class, not by label.** MechJeb's labels are translated, so a
script keyed on `"circularize"` breaks on a French install. `maneuver_operation_name()`
gives the label if you need to display one.

**`describe_maneuver()` lists the parameters**, in the same format as `describe_module()`,
including the burn-time ones. And `maneuver_time_references()` says which burn times the
operation accepts — an unaccepted one is rejected with the list rather than ignored.

**There is no absolute-UT burn time.** For a specific moment, choose `"X_FROM_NOW"` and set
`LeadTime` to the seconds from now:

```python
mj.set_maneuver_time_reference("OperationCircularize", "X_FROM_NOW")
mj.set_maneuver_parameter("OperationCircularize", "LeadTime",
                          target_ut - conn.space_center.ut)
```

**Set a target a tick before you use it.** MechJeb syncs the target on its own
`FixedUpdate`, so a target set through stock kRPC is invisible to it in the same frame:

```python
conn.space_center.target_vessel = station
conn.space_center.wait()                     # let one tick pass
mj.set_maneuver_time_reference("OperationPlane", "REL_HIGHEST_AD")
mj.create_maneuver_nodes("OperationPlane")
```

**Failures explain themselves, warnings do not interrupt.** No target, a target in another
sphere of influence, an apoapsis below the surface — all raise with MechJeb's own message.
But three operations plan a perfectly good burn and still have a caveat, so that goes to
`maneuver_warning` instead of rejecting the plan:

```python
count = mj.create_maneuver_nodes("OperationSemiMajor")
if mj.maneuver_warning:
    print("MechJeb says:", mj.maneuver_warning)
```

**Two things are shared with MechJeb's own window.** The burn-time selection is one object
per operation type, so setting it changes what the player sees in the Maneuver Planner —
and vice versa. And parameters live on a single instance per operation, so two scripts
planning the same operation share them.

**Interplanetary transfers are not supported.** That operation's solver is built by
MechJeb's GUI and returns "Started computation" forever when driven headlessly. It is the
one thing here that has to be done another way.

---

## Cameras

If you use OCISLY, the short version is: **do nothing**. `auto_rearm` and
`disambiguate_names` are on by default, so the cameras come back by themselves after every
scene reload, including one you triggered by hand from FMRS's window.

```python
print(conn.ocisly.last_restore)
# rearm[filter=Hull 1@2451,Hull 2@2455 opened=2 streaming=2] hideUI[disabled] (2.6s)
```

Manual control, if you want it:

```python
oc = conn.ocisly
print(oc.streaming)               # ['Booster.Hull 1@2451', 'Booster.Hull 2@2455']

conn.fmrs.jump_to_vessel(vid)

oc.rearm("@2451,@2455")           # the tokens, not the names. Blocks ~0.5 s
print(oc.streaming)
```

**Filter on the `@flightID` token.** Neither half of `vessel.camera` survives a
separation. The vessel half obviously changes — but so does the camera half, because the
`1` in `Hull 1` is only there while several cameras share a name *on the same vessel*.
Split the stack and each is suddenly alone, so the ordinal disappears and `Hull 1` becomes
`Hull`. A filter written on the full name then matches nothing and arms zero cameras,
silently. `part.flightID` is persistent, so the token is the one thing that holds.

Every extra camera costs three full scene renders per cycle, so arm the ones actually on
air rather than passing `"*"`.

For a recorded flight:

```python
conn.fmrs.screen_messages = False        # FMRS is chatty at every separation
conn.fmrs.window_hidden = True           # hide FMRS's window only
oc.hide_ui_on_scene_load = True          # HUD off after every reload
oc.restore_delay = 1.2                   # tighter cut; too low and a camera is missed
```

`hide_ui_on_scene_load` lives on the OCISLY service rather than on `bridge` because the
ordering is the difficulty: OCISLY refuses to open a camera while the game UI is hidden, so
the cameras must be restored and repainted *before* the HUD goes away. Hiding first silently
produces cameras that never stream.

**Which is why `rearm()` raises the HUD itself, and puts it back.** Once
`hide_ui_on_scene_load` has done its job the scene is in exactly the state OCISLY refuses to
open a camera in — so a later manual `rearm()`, to catch a booster that separated after the
restore, would arm nothing and say nothing. It brings the HUD up for the half second it
needs and restores whatever state it found. On a recording that is a visible flash; arm your
cameras before you start rolling where you can.

For an immediate one-off, `conn.bridge.hide_ui()` and `conn.bridge.show_ui()`.

---

## Writing your own plugin

Copy `src/Plugins/Template/` and change three things: `AssemblyName`, `RootNamespace`, and
the names in `AssemblyInfo.cs`. Then:

```
dotnet sln KRPC.Bridge.sln add src\Plugins\MyMod\KRPC.Bridge.MyMod.csproj
```

Uncomment `<BridgeOutputSubdir>Plugins</BridgeOutputSubdir>` in your copy's csproj — the
template leaves it out, and stays out of the solution, so that it is never built into a
release. It declares a kRPC service of its own, and shipping that would put a stray
`conn.template` in every user's install.

The template shows the pattern: never add a hard `<Reference>` to the mod's DLL — resolve
it out of `AssemblyLoader.loadedAssemblies` instead, so your plugin still loads when the
mod is absent and simply reports `available = False`.

Start by asking the game what the mod really has:

```python
for line in conn.bridge.describe_type("SomeMod", "SomeMod.API"):
    print(line)
```

Ten seconds, no rebuild, and it is how every reflective lookup in this repo was found.

Four rules worth knowing before you write a signature:

1. **Legal types only.** `bool`, `int`, `long`, `uint`, `ulong`, `float`, `double`,
   `string`, `byte[]`, a `[KRPCClass]` type, a `[KRPCEnum]` with `int` backing,
   `KRPC.Service.Messages.IMessage`, or `IList`/`IDictionary`/`HashSet` of those.
   `System.Tuple` works; C# `ValueTuple` (`(int, int)`) does **not**. `Vector3`, `Guid`,
   `object`, `IEnumerable<T>` and `ISet<T>` are all invalid. Dictionary *keys* are further
   restricted to `int`, `long`, `uint`, `ulong`, `bool`, `string`.

2. **One bad signature kills the whole kRPC server** — every service, not just yours.
   `build.cmd` runs kRPC's own scanner against your DLLs before you deploy. Use it.

3. **Do not return `[KRPCClass]` in volume.** Each one is an object handle costing a round
   trip per property read. Pack records as tab-separated strings instead; see `CoreService`.

4. **Property getters get streamed.** A client can put a stream on any of them, and it will
   then re-run every physics tick. Keep them cheap, or cache — `MechJebService` caches its
   reflection walks for exactly this reason.

`docs/ARCHITECTURE.md` covers the rest: load order, the main-thread question, background
jobs, and why plugin isolation is a build-time property.

---

## When it does not work

| Symptom | Where to look |
|---|---|
| `conn.fmrs` does not exist | The DLL is not loaded, **or** one bad signature took the whole server down. `findstr /C:"[KRPC.Bridge]" "<KSP>\KSP.log"`, and search for `kRPC Service Error` too. |
| Every kRPC service is missing at once | That is the whole-server failure. Something in *any* loaded assembly has a signature kRPC rejects. Run `build.cmd <ksp>` — the scan step names it. |
| `available == False` | `conn.<service>.diagnostics` names the exact member that did not resolve. Usually a mod update renamed something. |
| `dropped_vessels` stays empty | `fmrs.armed` was false at launch, or the stage fails FMRS's filters. Try `fmrs.track_parachutes = True` and `fmrs.control_uncontrollable = True`. |
| An RPC never returns | Normal during a scene change — kRPC queues calls rather than failing. If it persists, the scene did not come back. `jump_to_vessel` gives up after 90 s with a clear message. |
| The game gets dragged back into flight | `switched_to_dropped` was true when something changed scene. `jump_to_main()` first. |
| Cameras do not come back | `conn.ocisly.last_restore` says what the restore did. If it says `nothing was on air`, the snapshot was empty — OCISLY was not streaming when the scene was torn down. |
| Two camera feeds swapped | `disambiguate_names` was off. Turn it on, or call `name_cameras()`. |
| MechJeb "engage" does nothing | `conn.mech_jeb.diagnostics`, and look for `WARNING: inherited` in the pool report. That means the bridge bound to `List`'s `Add` instead of `UserPool`'s, which adds to a list and enables nothing. |

Both `python/check_bridge.py` and `conn.bridge.plugins` are designed to be the first thing
you run. Start there.
