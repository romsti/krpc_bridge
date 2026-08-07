#!/usr/bin/env python3
"""Locate the Kerbal Space Program install, so you never have to type its path.

    python tools/find_ksp.py            # print the path, or explain why it cannot
    python tools/find_ksp.py --check    # also require kRPC to be installed in it
    python tools/find_ksp.py --forget   # drop the remembered path

Why the build needs a path at all: KRPC.Bridge compiles against the game's own
assemblies - Assembly-CSharp.dll, the UnityEngine modules, and kRPC's two DLLs. None of
them may be redistributed (KSP Add-on Posting Rule 9), so they cannot be vendored into
this repo and have to come from a real install. There is no way around that. What there
IS a way around is typing the path every time.

Search order, first hit wins:

  1. the KSPROOT environment variable
  2. ksp.path at the repo root, remembered from a previous run (gitignored)
  3. Steam: the install path from the registry, then every library in
     libraryfolders.vdf - which is what finds an install on a second drive
  4. a short list of common locations, including the usual GOG and standalone ones

A hit is only accepted if it actually contains KSP_x64_Data/Managed/Assembly-CSharp.dll,
so a leftover empty folder is never mistaken for an install. Whatever is found gets
written to ksp.path, so the slow paths run once.
"""

import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REMEMBERED = os.path.join(ROOT, "ksp.path")


def tidy(path):
    """One separator, a capital drive letter, no trailing slash.

    Steam's registry value comes back with forward slashes and a lowercase drive, so
    joining it with anything produces "c:/program files (x86)/steam\\steamapps\\...".
    It works, and it looks like a bug every time you read it.
    """
    if not path:
        return path
    path = os.path.normpath(path)
    if len(path) > 1 and path[1] == ":":
        path = path[0].upper() + path[1:]
    return path


def is_ksp(path):
    """A real install, not an empty folder with the right name."""
    if not path:
        return False
    for data in ("KSP_x64_Data", "KSP_Data"):
        if os.path.isfile(os.path.join(path, data, "Managed", "Assembly-CSharp.dll")):
            return True
    return False


def has_krpc(path):
    return os.path.isfile(os.path.join(path, "GameData", "kRPC", "KRPC.Core.dll"))


def from_env():
    return tidy(os.environ.get("KSPROOT", "").strip('"').strip())


def from_remembered():
    if not os.path.isfile(REMEMBERED):
        return None
    return tidy(io.open(REMEMBERED, encoding="utf-8").read().strip())


def steam_root():
    """Steam's own install directory, from the registry on Windows."""
    if os.name != "nt":
        return None
    try:
        import winreg
    except ImportError:
        return None
    for hive, key in ((winreg.HKEY_CURRENT_USER, r"Software\Valve\Steam"),
                      (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\WOW6432Node\Valve\Steam"),
                      (winreg.HKEY_LOCAL_MACHINE, r"SOFTWARE\Valve\Steam")):
        for value in ("SteamPath", "InstallPath"):
            try:
                with winreg.OpenKey(hive, key) as handle:
                    found = winreg.QueryValueEx(handle, value)[0]
                if found and os.path.isdir(found):
                    return found
            except OSError:
                continue
    return None


def steam_libraries():
    """Every Steam library folder, including ones on other drives.

    This is the case a hardcoded default misses: Steam happily installs to D: or an
    external drive, and libraryfolders.vdf is the only record of where.
    """
    root = steam_root()
    if not root:
        return []
    libraries = [root]
    for name in ("libraryfolders.vdf", "steamapps/libraryfolders.vdf"):
        vdf = os.path.join(root, name.replace("/", os.sep))
        if not os.path.isfile(vdf):
            continue
        try:
            text = io.open(vdf, encoding="utf-8", errors="replace").read()
        except OSError:
            continue
        # Both the old ("1" "D:\\Games") and new ("path" "D:\\Games") layouts.
        for match in re.finditer(r'"(?:path|\d+)"\s+"([^"]+)"', text):
            candidate = match.group(1).replace("\\\\", "\\")
            if os.path.isdir(candidate) and candidate not in libraries:
                libraries.append(candidate)
    return libraries


def candidates():
    for library in steam_libraries():
        yield tidy(os.path.join(library, "steamapps", "common", "Kerbal Space Program"))

    for drive in ("C:", "D:", "E:", "F:"):
        yield drive + r"\Program Files (x86)\Steam\steamapps\common\Kerbal Space Program"
        yield drive + r"\Steam\steamapps\common\Kerbal Space Program"
        yield drive + r"\SteamLibrary\steamapps\common\Kerbal Space Program"
        yield drive + r"\Games\Kerbal Space Program"
        yield drive + r"\GOG Games\Kerbal Space Program"
        yield drive + r"\Program Files\Kerbal Space Program"
    # Versioned folder names, which is how most people keep more than one install.
    for drive in ("C:", "D:", "E:", "F:"):
        base = drive + "\\Games"
        if not os.path.isdir(base):
            continue
        try:
            for entry in sorted(os.listdir(base), reverse=True):
                if entry.lower().startswith("ksp") or "kerbal" in entry.lower():
                    yield os.path.join(base, entry)
        except OSError:
            pass


def remember(path):
    try:
        io.open(REMEMBERED, "w", encoding="utf-8", newline="").write(path + "\n")
    except OSError:
        pass


def find():
    """Return (path, how_it_was_found) or (None, None)."""
    for path, how in ((from_env(), "KSPROOT"),
                      (from_remembered(), "ksp.path")):
        if path and is_ksp(path):
            return path, how
        if path and how == "ksp.path":
            # A remembered path that no longer works is worse than none: it would send
            # the build at a folder the user has since moved or deleted.
            try:
                os.remove(REMEMBERED)
            except OSError:
                pass

    seen = set()
    for path in candidates():
        if path in seen:
            continue
        seen.add(path)
        if is_ksp(path):
            path = tidy(path)
            remember(path)
            return path, "searched"
    return None, None


def main():
    args = sys.argv[1:]

    if "--forget" in args:
        if os.path.isfile(REMEMBERED):
            os.remove(REMEMBERED)
            print("forgotten: " + REMEMBERED, file=sys.stderr)
        else:
            print("no remembered path", file=sys.stderr)
        return 0

    path, how = find()
    if not path:
        print(
            "KSP not found.\n"
            "\n"
            "Looked at: the KSPROOT variable, ksp.path at the root of the repo, every\n"
            "Steam library declared in libraryfolders.vdf, and the usual locations on\n"
            "drives C: to F:.\n"
            "\n"
            "Give the path once and it is remembered:\n"
            "    .\\build.cmd \"D:\\Games\\KSP_1.12.5\"\n"
            "\n"
            "The build needs a real install because it compiles against the game's\n"
            "assemblies, which may not be redistributed.",
            file=sys.stderr)
        return 1

    if "--check" in args and not has_krpc(path):
        print("KSP found (%s) but kRPC is not installed in it.\n"
              "Expected: %s"
              % (path, os.path.join(path, "GameData", "kRPC", "KRPC.Core.dll")),
              file=sys.stderr)
        return 2

    if "--quiet" not in args:
        print("KSP: %s   (%s)" % (path, how), file=sys.stderr)
    # stdout carries the path alone, so a caller can capture it.
    sys.stdout.write(path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
