# KRPC.Bridge

**kRPC services for FMRS, OCISLY and MechJeb 2.** Three Kerbal Space Program mods that
have no scriptable interface at all, made drivable from Python.

[![Documentation](https://img.shields.io/badge/docs-romsti.github.io-deeppink)](https://romsti.github.io/krpc_bridge/)
[![KSP 1.12.x](https://img.shields.io/badge/KSP-1.12.x-blue)](https://www.kerbalspaceprogram.com/)
[![kRPC 0.6](https://img.shields.io/badge/kRPC-0.6.x-blue)](https://github.com/krpc/krpc)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

📖 **[Full documentation](https://romsti.github.io/krpc_bridge/)** — all 88 procedures,
a scripting guide, and search.

```python
import krpc
conn = krpc.connect()

conn.fmrs.armed = True             # capture this flight's stages
conn.fmrs.track_parachutes = True  # else a chute-only booster is ignored

# ... launch, fly, separate ...

vid = next(iter(conn.fmrs.dropped_vessels))
conn.fmrs.jump_to_vessel(vid)      # fly the booster down
booster = conn.space_center.active_vessel

# ... land it ...

conn.fmrs.recover_current()
print(conn.fmrs.recovered_funds()) # 14832.0
conn.fmrs.jump_to_main()           # back to the mission
```

The cameras come back on their own.

---

## Why this exists

kRPC exposes stock KSP beautifully and mod internals not at all. It wraps the game's own
objects, plus the `KSPField`, `KSPEvent` and `KSPAction` members of any `PartModule` — and
that is the whole of it. A mod whose functionality lives in a static class, a singleton or
a `ScenarioModule` is simply invisible.

All three mods here are exactly that shape:

- **FMRS** exposes no `KSPEvent`, no action group and no key binding. Its jump API is a
  public method on a `MonoBehaviour` that stock kRPC has no way to reach. And the jump is
  not a vessel switch — it is a full flight-scene reload, 5 to 20 seconds, in which
  everything is destroyed and rebuilt from a save file.
- **OCISLY** subscribes to no scene `GameEvent` and uses no `DontDestroyOnLoad`, so that
  reload silently drops every camera it was streaming — and nothing in the game brings
  them back.
- **MechJeb**'s ascent autopilot is reachable in principle, but the established bridge
  ([Genhis/KRPC.MechJeb](https://github.com/Genhis/KRPC.MechJeb)) targets 2.14.3 and no
  longer loads against 2.15.x, because it looks types and members up by exact name and
  2.15 renamed most of them.

This mod reaches all three by reflection, resolved once at load, and reports member by
member when something has moved.

## What you get

A hundred and twenty-nine remote procedures across four services. The full reference is in
[`docs/API.md`](docs/API.md); the highlights:

**`conn.fmrs`** — arm and disarm; list dropped stages with their separation timestamps,
their batch grouping and their KSP `persistentId`; jump to a stage and back; replay a
landing from its separation save; revert to launch after a jump, when KSP's own revert is
gone; recover a booster and read what it actually earned in funds, science and contracts;
delete a tracked stage.

**`conn.ocisly`** — restore camera streams across a scene reload, automatically and by
default, even when the jump was made by hand with no script running. Give duplicate
Hullcams names that are unique *and* stable across a reload, so two feeds can never swap.

**`conn.mech_jeb`** — fly the ascent, addressed by setting name so a MechJeb rename costs
a string in your script rather than a rebuilt DLL. And the reason this service exists:
take back the staging decision while MechJeb keeps flying, which is the only way to drop
side boosters that share a stage with the core.

**`conn.bridge`** — what loaded and why not; an ordered, id-numbered event log so a script
can observe an *instant* rather than sampling a value; a reflection probe for exploring any
loaded mod from Python.

## Install

Requires **KSP 1.12.x** and **[kRPC](https://github.com/krpc/krpc) 0.6.x**. FMRS, OCISLY
and MechJeb are each optional — install the ones you use, and the corresponding service
reports `available = False` for the others.

1. Download the latest release and unzip it so that `GameData/KRPC.Bridge/` lands in your
   KSP `GameData/`:

   ```
   GameData/KRPC.Bridge/
   ├── KRPC.Bridge.Core.dll        + .xml
   ├── LICENSE, NOTICE, KRPC.Bridge.version
   └── Plugins/
       ├── KRPC.Bridge.Fmrs.dll    + .xml
       ├── KRPC.Bridge.Ocisly.dll  + .xml
       └── KRPC.Bridge.MechJeb.dll + .xml
   ```

   Keep each `.xml` next to its `.dll`: kRPC reads it to build the Python docstrings, so
   without it `help(conn.fmrs)` is empty. Nothing goes in `GameData/kRPC/` — that folder
   belongs to kRPC and is overwritten on update.

2. Install the kRPC Python client, if you have not already. There is nothing else to
   install — this mod ships no client library of its own, it adds services to the
   connection kRPC already gives you.

   ```
   pip install "krpc>=0.6,<0.7"
   ```

3. Start KSP, start the kRPC server, then, from any REPL:

   ```python
   >>> import krpc
   >>> conn = krpc.connect()
   >>> conn.bridge.ping()
   'pong'
   >>> conn.bridge.available_plugins
   ['FMRS', 'OCISLY', 'MechJeb']
   ```

   `python/check_bridge.py`, in this repository, prints the same thing with a full
   diagnostic per plugin. It is not in the download: the zip holds the four assemblies
   and their `.xml`, and no Python at all.

**Upgrading from the single-DLL version:** delete the old
`GameData/KRPC.Bridge/Plugins/KRPC.Bridge.dll` by hand. With both present, two assemblies
declare the same service names, kRPC refuses the duplicate, and that takes the *entire*
kRPC server down — not just this mod.

## Build

Needs the [.NET SDK 8](https://dotnet.microsoft.com/download) and a KSP install with kRPC
in it. Nothing from the game is copied or redistributed.

| Command | Produces | When |
|---|---|---|
| `.\build.cmd verify` | nothing — just checks the C# compiles | after every code change. One second, no install needed. |
| `.\build.cmd` | the four DLLs and their `.xml` in `dist/GameData/` | before testing. Also validates every kRPC signature. |
| `.\build.cmd deploy` | the same, **and** copies into your GameData | to test in game. |
| `python tools/check_docs.py` | a report | docs against code, signature types, XML comments. |
| `python tools/package.py` | `dist/KRPC.Bridge-x.y.z.zip` | at release time. This is what people download. |
| `.\build.cmd forget` | forgets the remembered KSP path | if you move or change install. |

`build.cmd` only produces the DLLs. `tools/package.py` turns them into the release zip,
refusing to build one in which any `.dll` lacks its `.xml`, one of the four expected
assemblies is missing, or `LICENSE`/`NOTICE` would be left out.

Give the path explicitly the first time only if auto-detection fails:
`.\build.cmd "D:\Games\KSP_1.12.5"` — it is remembered afterwards.

**You do not normally have to give the path.** `tools/find_ksp.py` checks the `KSPROOT`
variable, a remembered `ksp.path`, every Steam library declared in `libraryfolders.vdf`
— which is what finds an install on a second drive — and then the usual locations. Give
the path once if all of that fails and it is remembered from then on.

### Why the build needs your KSP install

C# is statically typed. When `FmrsService.cs` writes `FlightGlobals.ActiveVessel`, the
compiler has to know that `FlightGlobals` exists, that it has a static property of that
name, and that the property returns a `Vessel` — which in turn has an `id` field of type
`Guid`, and so on. That whole description lives in `Assembly-CSharp.dll` inside your
install. Without it the build stops on a wall of `CS0246: type or namespace not found`.
Same for `[KRPCService]` and `YieldException`, which come from `KRPC.Core.dll`.

**What is read is not what is copied.** Those DLLs are referenced with `Private=False`,
so the compiler records only a *reference* — assembly name, type names — never their
code. At startup KSP resolves those references against the copies it has already loaded.
That is why the finished mod is a few tens of kilobytes rather than a hundred megabytes,
and why redistributing the game's assemblies would be as pointless as it is forbidden.

`build/stubs/Stubs.cs` is the exception that proves it: fake declarations with the same
signatures, enough for the compiler to check types and useless in game. That is what
`build.cmd verify` compiles, and why that one command needs no install at all. It catches
typos, wrong overloads and missing usings — not whether the mod loads.

### The four steps

The fourth is the one that matters:

1. **Type-check against stubs.** `build/verify` compiles every source file against
   `build/stubs/Stubs.cs`, so a typo costs a second rather than a KSP launch. No KSP
   install required.
2. **Locate KSP and check kRPC is in it.** Fails with a readable message rather than a
   wall of `CS0246`.
3. **Build the solution** into `dist/GameData/`, which mirrors the destination exactly, so
   deployment is a straight copy with no path logic.
4. **Validate every kRPC signature** by running kRPC's *own* scanner against the built
   DLLs.

Step 4 exists because of a fact worth internalising: kRPC scans every loaded assembly at
server start, and **one malformed signature anywhere disables the entire kRPC server** —
every service, not just the offending one. The only symptom is a popup and a line in
`KSP.log`; from Python it is indistinguishable from "the server is not running". There is
no runtime fix, because .NET does not unload assemblies and nothing can be quarantined once
KSP has started. Catching it before the DLL reaches `GameData` is the only option, and
`build/scan` does it in about a second by calling
`KRPC.Service.Scanner.Scanner.GetServices` directly — passing an error list, so it reports
*every* bad signature at once rather than stopping at the first.

## Documentation

| | |
|---|---|
| [`docs/API.md`](docs/API.md) | Every RPC, by service, with the reasoning behind the ones that are not obvious. |
| [`docs/SCRIPTING.md`](docs/SCRIPTING.md) | Driving it from Python: a complete booster recovery, replaying a landing, ascent with manual staging, and what goes wrong. |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Core and plugins, load order, threading, and why plugin isolation is a build-time property. Read before adding a plugin. |

**Read it as a site instead:** <https://romsti.github.io/krpc_bridge/> — same files, with a
sidebar, full-text search and a copy button on every example. Published from `main` on
every docs change. To preview a change before pushing:

```
pip install "mkdocs-material>=9.7,<10"
mkdocs serve
```

## Traps worth knowing

**A jump invalidates every handle.** `jump_to_vessel` reloads the whole flight scene.
Vessels, parts, modules and streams you held beforehand are all dead. Remove the streams
first, then rebuild from a fresh `space_center.active_vessel`.

**`switched_to_dropped` is a guard rail, not a status field.** FMRS hooks
`onGameSceneLoadRequested`, and a scene change requested while that flag is true makes it
force-load the main mission a few dozen frames later — dragging the game back into flight
mid-script. Always `jump_to_main()` before reverting, recovering or launching.

**`armed` must be true before launch.** FMRS reads it exactly once, at the top of its
launch routine.

**Auto-recovery does not fire on touchdown.** It fires when you *leave* a dropped stage —
jumping away with `save_landed=True`, `jump_to_main()`, the stock Recover button, or a
scene change. A landed booster sits there indefinitely otherwise. And a recovered stage is
still jumpable: only `delete_dropped()` makes one unreachable.

**Group stages on `dropped_saves`, not on when a poll noticed them.** FMRS writes one save
per staging event, so that value is what actually records which stages came off together.

**Filter cameras on the `@flightID` token, not on either half of the name.** Both halves
change across a separation. The vessel half obviously does. The camera half does too: the
ordinal in `Aerocam DN 2` is only appended while several cameras share a name on the *same*
vessel, so splitting the stack turns it back into `Aerocam DN`. `conn.ocisly.remembered`
gives you the tokens.

**Set MechJeb's `autostage` before engaging, and know it does not go back.** The autopilot
reads it as it is enabled, to decide whether to register with the staging controller at all.
Setting it true again later re-registers nothing — cycle `ascent_enabled` instead.

## Compatibility

Verified against **FMRS Continued 1.2.9.6**, **kRPC 0.6.0**, **KSP 1.12.5**.

Nothing is bound by hard reference, and every lookup is reported individually, so a mod
update that renames one member costs you that member rather than the service. When
something does break, `conn.<service>.diagnostics` names the exact lookup that failed and
`conn.bridge.describe_type()` tells you what the member is called now — usually a one-line
fix.

## Licence and credit

MIT. See [`LICENSE`](LICENSE), and [`NOTICE`](NOTICE) for attribution.
Source: https://github.com/romsti/krpc_bridge

This mod ships only its own assemblies. It links against kRPC (LGPL v3, Copyright
2015-2023 kRPC Org) using the copy already in your `GameData`, and reaches FMRS Continued
(MIT), MechJeb 2 (GPL-3.0), OCISLY and HullcamVDS Continued (GPL-3.0) purely by reflection
— no compile-time reference, no copied code, nothing redistributed. Thanks to their
authors and maintainers.
