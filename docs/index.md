# KRPC.Bridge

**kRPC services for FMRS, OCISLY and MechJeb 2.** Three Kerbal Space Program mods that
have no scriptable interface at all, made drivable from Python.

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

## Where to start

<div class="grid cards" markdown>

- **New here?** The [scripting guide](SCRIPTING.md) walks a complete booster recovery from
  arming to the recovery ledger, then the six things that go wrong.

- **Looking for a member?** The [API reference](API.md) has all 88 procedures by service,
  with units, return shapes and what each one raises. Use the search box.

- **Writing a plugin?** [Architecture](ARCHITECTURE.md) explains the Core, the load order
  and why plugin isolation is a build-time property. Read it before adding one.

</div>

## Why it exists

kRPC exposes stock KSP beautifully and mod internals not at all. It wraps the game's own
objects, plus the `KSPField`, `KSPEvent` and `KSPAction` members of any `PartModule` — and
that is the whole of it. A mod whose functionality lives in a static class, a singleton or
a `ScenarioModule` is simply invisible.

All three mods here are exactly that shape.

**FMRS** exposes no `KSPEvent`, no action group and no key binding. Its jump API is a public
method on a `MonoBehaviour` that stock kRPC has no way to reach. And the jump is not a
vessel switch — it is a full flight-scene reload, 5 to 20 seconds, in which everything is
destroyed and rebuilt from a save file.

**OCISLY** subscribes to no scene `GameEvent` and uses no `DontDestroyOnLoad`, so that
reload silently drops every camera it was streaming, and nothing in the game brings them
back.

**MechJeb**'s ascent autopilot is reachable in principle, but the established bridge targets
2.14.3 and no longer loads against 2.15.x, because it looks types and members up by exact
name and 2.15 renamed most of them.

This mod reaches all three by reflection, resolved once at load, and reports member by
member when something has moved. Each integration is a separate assembly and degrades on
its own: a mod you do not have reports `available = False` with a diagnostic naming exactly
what was looked for, and nothing else is affected.

## Install

Requires **KSP 1.12.x** and **[kRPC](https://github.com/krpc/krpc) 0.6.x**. FMRS, OCISLY and
MechJeb are each optional.

1. Download the [latest release](https://github.com/romsti/krpc_bridge/releases/latest) and
   unzip it so `GameData/KRPC.Bridge/` lands in your KSP `GameData/`. Keep each `.xml` next
   to its `.dll` — kRPC reads it to build the Python docstrings, so without it
   `help(conn.fmrs)` is empty.

2. Install the kRPC Python client. There is nothing else to install; this mod ships no
   client library of its own.

    ```
    pip install "krpc>=0.6,<0.7"
    ```

3. Start KSP, start the kRPC server, then from any REPL:

    ```python
    >>> import krpc
    >>> conn = krpc.connect()
    >>> conn.bridge.ping()
    'pong'
    >>> conn.bridge.available_plugins
    ['FMRS', 'OCISLY', 'MechJeb']
    ```

Verified against **FMRS Continued 1.2.9.6**, **kRPC 0.6.0**, **KSP 1.12.5**.
