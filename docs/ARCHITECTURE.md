# Architecture

Read this before adding a plugin, or before changing the project layout.

Everything below follows from five facts about kRPC and KSP. They constrain the design far
more than any preference does, so they come first.

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
available, and this is worth being precise about: .NET does not unload assemblies, and
kRPC scans everything already loaded. No try/catch, no registry, no configuration flag can
quarantine a broken plugin from inside the game, because the damage is done before any of
your code runs.

With one DLL that was a nuisance. With a Core and three plugins it is four times the
surface, and it is the single largest risk in this architecture. See
[Keeping a bad signature out of GameData](#keeping-a-bad-signature-out-of-gamedata).

### 4. kRPC has events, but they are latches, not messages

*(This corrects an earlier design note in this repo, which said kRPC had no event channel
at all. It does: `KRPC.Service.Event`, in `KRPC.Core`.)*

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
rather than loading it and throwing `TypeLoadException` on first use.

---

## Layout

### On disk

```
GameData/KRPC.Bridge/
├── KRPC.Bridge.Core.dll        + .xml     <- Core at the root
├── LICENSE, NOTICE, KRPC.Bridge.version
└── Plugins/
    ├── KRPC.Bridge.Fmrs.dll    + .xml
    ├── KRPC.Bridge.Ocisly.dll  + .xml
    └── KRPC.Bridge.MechJeb.dll + .xml
```

The `.xml` must accompany **each** DLL in its own folder — kRPC looks for a file of the
same name next to the assembly. A single `.xml` at the root would document the Core only.

### In the repo

```
Directory.Build.props        TFM, KSP paths, doc file.  Once, for every project.
Directory.Build.targets      KSP references, output routing, deploy.
KRPC.Bridge.sln
build.cmd
src/
  Core/                      KRPC.Bridge.Core.dll
  Plugins/
    Template/                copy this to start a plugin
    Fmrs/  Ocisly/  MechJeb/
build/
  stubs/Stubs.cs             stand-ins so sources type-check with no KSP
  verify/                    compiles every source against the stubs
  scan/                      runs kRPC's own signature scanner
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
That is one of the least readable failures in KSP modding, and it costs nothing to avoid.

**`KSPAssemblyDependency`** gives *runtime* load ordering. MSBuild orders the build; the
attribute orders the load. Plugin-to-plugin references need both, for the same reasons.

### Output routing

`Directory.Build.targets` computes the output directory from one property each project
declares:

```xml
<BridgeOutputSubdir></BridgeOutputSubdir>          <!-- Core: the root -->
<BridgeOutputSubdir>Plugins</BridgeOutputSubdir>   <!-- a plugin -->
```

So `dist/GameData/` has exactly the shape of the destination, deployment is a mirror copy
with no path logic, and adding a plugin requires editing no deployment script. The rule
"Core at the root, plugins in `Plugins/`" is expressed once.

One MSBuild detail worth knowing: the references live in `.targets`, not `.props`. `.props`
is imported *before* the project body, so a condition written there cannot see a property
the project sets about itself. `build/verify/Verify.csproj` sets `BridgeUseStubs=true` and
must be able to skip the KSP references on that basis, including when built by hand with no
flags. `.targets` is imported after the body and sees it.

---

## The Core

`KRPC.Bridge.Core.dll` owns the one `MonoBehaviour` the mod has, and knows nothing about
any specific mod.

| | |
|---|---|
| `BridgeCore` | Bootstrap once per session; pump the main-thread queue every `FixedUpdate`. `DontDestroyOnLoad`, so it survives scene changes. |
| `ModRegistry` | Where a plugin announces itself, and where reflection helpers live. |
| `EventBus` | GameEvents to a 512-entry ring buffer, plus kRPC event signals. |
| `Wait` | Block an RPC on game state without freezing the game. |
| `Jobs` | Off-main-thread computation with a pollable result. |
| `MainThread` | The classic dispatcher, for the three cases that need one. |
| `Hud` | Hide and show the HUD the way F2 does. |
| `BridgeLog` | One log prefix for the whole mod. |
| `CoreService` | `conn.bridge`. |

### Blocking without freezing: `Wait`

The mechanism you will use most. A `while (!ready)` loop or a `Thread.Sleep` inside an RPC
hangs Unity, not just the client. kRPC's own continuation is the answer: throw a
`YieldException` and the server re-invokes your delegate on a later update, letting the
game run in between. The client sees one long call.

```csharp
Wait.Until (() => condition, timeoutSeconds, "what you are waiting for");
Wait.Settled (() => condition, consecutiveTicks, timeoutSeconds, "...");
Wait.Ticks (n);
```

`Settled` deserves a note: it requires the condition to hold for N ticks in a row. A vessel
is briefly `LANDED` mid-bounce and briefly "loaded" mid scene-swap. Requiring a run of true
readings is what makes a handoff reliable.

Deadlines are in **real** time (`Time.realtimeSinceStartup`), deliberately: the in-game
clock stops during a scene load, which is exactly when a timeout must still be able to
fire.

Two properties of yielded RPCs are worth knowing before you rely on one:

- **A resumed continuation is not re-checked against the game scene.** The `GameScene`
  guard applies only to the initial call. That is what lets `FmrsService.JumpToVessel`
  survive the reload it triggers — and it means a yielding RPC has to defend itself.
- **There is no client-side timeout.** A continuation that never stops yielding hangs the
  Python client forever. Always bound the chain with a deadline or a tick count.

### Getting work off the main thread: `Jobs`

1. On the main thread, capture what the computation needs as **plain data** — doubles,
   arrays. Never a `Vessel`, `Part`, `Orbit` or `GameObject`.
2. Hand the snapshot to `Jobs.Start`.
3. The worker computes on the snapshot alone.
4. Python polls, or the RPC uses `Wait.Until` so it looks synchronous without costing
   frame rate.

**The rule is absolute: the worker must not touch Unity or KSP.** Off the main thread most
of Unity's API does not raise a managed exception — it hard-crashes the process, with no
stack trace in `KSP.log`. A worker needing a live reading mid-computation must go back
through `MainThread.Invoke`, which costs a full tick.

### The dispatcher: `MainThread`

Only for the three cases where you are genuinely *not* already on the main thread: a
background job needing one live read, a callback a third-party mod hands you on its own
thread, or your own socket, timer or file watcher.

`Invoke` called *from* the main thread runs inline. Without that branch, any kRPC service
method calling it would block the main thread waiting on a queue only the main thread can
drain — deadlocking the entire game, not just the client.

---

## Performance

In rough order of impact.

**An RPC costs about a millisecond round trip, and kRPC bounds how many it runs per
`FixedUpdate`.** Anything you would loop in Python must be looped in C# and returned flat.
Reading one value per part on a 200-part vessel is 200 RPCs from Python and one from a
plugin. At 25 Hz the first is impossible and the second is free. This constraint shapes
every signature in the mod.

**Do not return `[KRPCClass]` in volume.** A `KRPCClass` return value is an object handle:
the client gets a reference and pays another round trip per property it then reads. Forty
records with six fields each is 240 RPCs as handles and 1 as tab-separated strings. Nothing
in the Core returns a `KRPCClass` — and packing has a second benefit, fewer exotic
signatures, which is less surface for fact 3.

**Resolve reflection once, at load.** Every plugin here does. For members a stream will
poll, go further and compile a delegate: a stream re-reads its value every `FixedUpdate`,
and at 60 Hz across a dozen streams the boxing and argument-array allocation of
`MethodInfo.Invoke` show up as GC pressure, which shows up as micro-freezes.
`ModRegistry.StaticGetter<T>` and `InstanceGetter<T>` exist for this.

**Any property can be streamed, so keep every getter cheap.** A client can put a stream on
any member with a return value, and it will then run every physics tick forever. A getter
that walks reflection or allocates a fresh `List` is a defect waiting for a user to find
it. `MechJebService` caches its three reflection walks for exactly this reason — the answer
cannot change during a session.

**Declare each service's `GameScene`.** It stops kRPC evaluating anything where it makes no
sense. The Core is `All` because asking which plugins loaded, from the space centre and
before any flight, is the normal way to open a session.

---

## Keeping a bad signature out of GameData

Given fact 3, all robustness has to happen before deployment. Four lines of defence, in
order of cost.

**1. Run kRPC's own scanner at build time.** `build/scan` loads the built DLLs and calls
`KRPC.Service.Scanner.Scanner.GetServices(errors)` — the same validation the server
performs, in about a second. Passing the error list makes it collect every bad signature
instead of throwing on the first, so a build that broke five reports five. This largely
replaces the manual discipline below, and `build.cmd` runs it automatically.

**2. Type-check every source file against the stubs.** `build/verify` uses a glob, not a
list of files. That is deliberate: forgetting to add a new plugin to a list is precisely
the path by which an unchecked source reaches `GameData`. A glob cannot be forgotten.

**3. Know the signature rules.** Legal parameter and return types are: `bool`, `int`,
`long`, `uint`, `ulong`, `float`, `double`, `string`, `byte[]`, a `[KRPCClass]` type, a
`[KRPCEnum]` type with `int` backing, `KRPC.Service.Messages.IMessage`, and
`IList`/`IDictionary`/`HashSet`/`System.Tuple` of those. Dictionary keys are further
restricted to `int`, `long`, `uint`, `ulong`, `bool`, `string`.

   Invalid: `Vector3`, `Quaternion`, `Guid`, `DateTime`, `object`, `byte`, `short`,
   `decimal`, `IEnumerable<T>`, `ISet<T>`, arrays other than `byte[]`, an enum without
   `[KRPCEnum]`, and — a modern footgun — C# `ValueTuple`, i.e. `(int, int)` syntax.
   kRPC detects tuples by type-name prefix, and `ValueTuple\`2` does not match.

   `Nullable<T>` is illegal in kRPC 0.6.0 and legal on `main`. Do not use it yet.

   Identifiers must match `^[A-Z][A-Za-z0-9]*$`: initial capital, no underscores. And leave
   `Id` unset on `[KRPCService]` — it is derived from the name, and hardcoding one risks
   colliding with a future stock service.

**4. Stage the deployment.** Deploy one new plugin at a time, confirm
`conn.bridge.ping()` returns `pong`, then add the rest. Ten minutes of discipline against
an evening spent working out which of four new plugins broke the server.

### XML documentation is part of the signature surface

kRPC reads the `.xml` next to each DLL and passes it to the client. Malformed XML, or a
`cref` that cannot be resolved, throws during scanning and therefore **also disables the
server**. Two things to watch:

- The Python client understands only `<summary>`, `<param>`, `<returns>` and `<remarks>` at
  the top level. `<example>`, `<exception>`, `<value>` and `<seealso>` are silently
  dropped. Inline, it understands `<see cref=>`, `<paramref>`, `<c>` and `<list>`.
- `<list>` items must wrap their content: `<item><description>x</description></item>`. A
  bare `<item>x</item>` raises `IndexError` in the client.

---

## Adding a plugin

1. Copy `src/Plugins/Template/` to `src/Plugins/<Name>/`.
2. Change `AssemblyName`, `RootNamespace`, and the names in `AssemblyInfo.cs`.
3. `dotnet sln KRPC.Bridge.sln add src\Plugins\<Name>\KRPC.Bridge.<Name>.csproj`
4. `.\build.cmd verify` — you now have a type-checked plugin with no KSP launch.
5. `.\build.cmd "<ksp>"` — the scan step validates your signatures.
6. Deploy it alone first.

Never add a hard `<Reference>` to the mod you are wrapping. Resolve it out of
`AssemblyLoader.loadedAssemblies`. A hard reference means your plugin fails to load
entirely when the mod is absent, and takes its kRPC service down with it; reflection means
it loads, reports `available = false`, and nothing else is affected.

Assembly names drift — FMRS ships as `FMRSContinued` while its folder is still `FMRS` —
so always pass every historical name as a candidate.

`conn.bridge.describe_type()` is how to find out what a mod's type really has, from a
running game, in ten seconds. Every reflective lookup in this repo was found that way
rather than trusted from a changelog.

### Versioning the Core

Bump the minor number of `[assembly: KSPAssembly("KRPC.Bridge.Core", 1, 0)]` when you add
members. Bump the major only on a breaking change, and bump the
`KSPAssemblyDependency` in every plugin at the same time. While only the minor moves, older
plugins keep loading.
