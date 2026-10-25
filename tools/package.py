#!/usr/bin/env python3
"""Build the release zips from dist/.

    python tools/package.py [version]

Produces two zips, each holding a GameData/ folder the user unzips straight into their KSP install:

  * dist/KRPC.Bridge-<version>.zip   GameData/KRPC.Bridge: the assemblies, their .xml, LICENSE, NOTICE;
  * dist/TowerCatch-<version>.zip    GameData/TowerCatch: the whole booster catch tower, as a package of its own:
    its ModuleManager patch and craft (distribution/TowerCatch) and its plugin, Plugins/KRPC.Bridge.Tower.dll
    (the services that place and read the tower, and its .xml). It depends on the bridge. The tower's version
    is its own (distribution/TowerCatch/TowerCatch.version); `version` is the bridge's.

Run build.cmd first - this packages, it does not compile.

Rules it enforces, because they are easy to get wrong by hand and expensive to get wrong in public:

  * Every .dll must be accompanied by its .xml. kRPC reads the XML next to each assembly
    to build the Python docstrings; ship without it and help(conn.fmrs) is empty for
    everyone who installs the mod.

  * LICENSE and NOTICE must be inside each zip. The KSP Add-on Posting Rules require the
    full licence text in the download itself, not merely a licence name on the download
    page.

  * The bridge zip carries nothing of the tower (no patch, no craft, no assembly), and the tower zip carries
    nothing but the tower: the patch in two places makes every one of its parts twice in the game, and the
    tower assembly in two places declares the kRPC service Tower twice, which takes the whole server down.
"""

import io
import json
import os
import re
import sys
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIST = os.path.join(ROOT, "dist")
GAMEDATA = os.path.join(DIST, "GameData")

# The mod folders of GameData this repository builds.
BRIDGE = "KRPC.Bridge"
TOWER = "TowerCatch"
PACKAGES = (BRIDGE, TOWER)

MODDIR = os.path.join(GAMEDATA, BRIDGE)
TOWER_MODDIR = os.path.join(GAMEDATA, TOWER)

# Where the tower package's files are in the repo, as they are in its mod folder.
TOWER_SOURCE = "distribution/TowerCatch"

EXPECTED = [
    "KRPC.Bridge.Core.dll",
    "Plugins/KRPC.Bridge.Model.dll",
    "Plugins/KRPC.Bridge.Control.dll",
    "Plugins/KRPC.Bridge.Telemetry.dll",
    "Plugins/KRPC.Bridge.Fmrs.dll",
    "Plugins/KRPC.Bridge.Ocisly.dll",
    "Plugins/KRPC.Bridge.Trajectories.dll",
    "Plugins/KRPC.Bridge.Ident.dll",
    "Plugins/KRPC.Bridge.Stock.dll",
]

# The assemblies of the tower package (built, not in distribution/TowerCatch), as the bridge's EXPECTED.
TOWER_EXPECTED = [
    "Plugins/KRPC.Bridge.Tower.dll",
]

EXTRAS = [
    ("LICENSE", "LICENSE"),
    ("NOTICE", "NOTICE"),
    ("distribution/KRPC.Bridge.version", "KRPC.Bridge.version"),
]

# What the tower package must hold at least besides its assembly (the rest of distribution/TowerCatch goes in too).
TOWER_REQUIRED = [
    "Patches/TowerCatch.cfg",
    "Crafts/Tour Catch v5.craft",
    "NOTICE",
    "TowerCatch.version",
]


def version_from_changelog():
    """The topmost version heading in CHANGELOG.md."""
    path = os.path.join(ROOT, "CHANGELOG.md")
    if not os.path.exists(path):
        return None
    for line in io.open(path, encoding="utf-8"):
        match = re.match(r"^##\s*\[([0-9]+\.[0-9]+\.[0-9]+)\]", line)
        if match:
            return match.group(1)
    return None


def tower_version(root=ROOT):
    """MAJOR.MINOR.PATCH from the tower package's own .version file, or "0.0.0"."""
    try:
        with io.open(os.path.join(root, *TOWER_SOURCE.split("/"), "TowerCatch.version"),
                     encoding="utf-8") as handle:
            v = json.load(handle).get("VERSION", {})
        return "%d.%d.%d" % (int(v.get("MAJOR", 0)), int(v.get("MINOR", 0)), int(v.get("PATCH", 0)))
    except (OSError, ValueError, TypeError):
        return "0.0.0"


def tower_extras(root=ROOT):
    """[(source in the repo, path in the mod folder)] of the tower package: its source folder, and LICENSE."""
    source = os.path.join(root, *TOWER_SOURCE.split("/"))
    extras = [("LICENSE", "LICENSE")]
    for folder, _, names in os.walk(source):
        for name in sorted(names):
            full = os.path.join(folder, name)
            rel = os.path.relpath(full, source).replace(os.sep, "/")
            extras.append((TOWER_SOURCE + "/" + rel, rel))
    return extras


def write_zip(output, moddir, name):
    """Zip the package's mod folder, with GameData/<name>/ as the zip's root."""
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
        for folder, _, names in os.walk(moddir):
            for file_name in sorted(names):
                full = os.path.join(folder, file_name)
                # Store paths relative to dist/, so the zip root is GameData/.
                archive.write(full, os.path.relpath(full, DIST).replace(os.sep, "/"))
    print()
    with zipfile.ZipFile(output) as archive:
        for info in archive.infolist():
            print("  %-52s %7d" % (info.filename, info.file_size))
    print()
    print("wrote %s (%.0f kB)" % (os.path.relpath(output, ROOT), os.path.getsize(output) / 1024.0))


def package_bridge(version):
    if not os.path.isdir(MODDIR):
        print("nothing to package - %s does not exist.\nRun build.cmd first."
              % os.path.relpath(MODDIR, ROOT))
        return 2

    problems = []

    # Every DLL actually present must have its XML, not merely the four expected ones -
    # otherwise a plugin added later ships undocumented and nothing notices.
    shipped = []
    for folder, _, names in os.walk(MODDIR):
        for name in sorted(names):
            if not name.endswith(".dll"):
                continue
            full = os.path.join(folder, name)
            shipped.append(os.path.relpath(full, MODDIR).replace(os.sep, "/"))
            if not os.path.exists(full[:-4] + ".xml"):
                problems.append(
                    "missing %s - without it help(conn.<service>) is empty for every user"
                    % (os.path.relpath(full, MODDIR)[:-4] + ".xml"))

    for relative in EXPECTED:
        if relative not in shipped:
            problems.append("missing assembly: " + relative)

    # A DLL nobody meant to ship is a real hazard here: an extra assembly can declare a
    # kRPC service of its own, and the template does exactly that.
    for relative in shipped:
        if relative in TOWER_EXPECTED:
            problems.append(
                "%s in the bridge's folder: the tower's plugin belongs to the %s package "
                "(GameData/%s/Plugins); a build older than the move left it in dist/"
                % (relative, TOWER, TOWER))
        elif relative not in EXPECTED:
            problems.append(
                "unexpected assembly: %s - if this is intentional add it to EXPECTED, "
                "otherwise something is being shipped by accident" % relative)

    # The tower's patch and craft live in the tower package. In the bridge's folder they would be a second
    # copy of the patch next to the package's (ModuleManager would make every TG.* part twice).
    for folder in ("Patches", "Crafts"):
        if os.path.isdir(os.path.join(MODDIR, folder)):
            problems.append(
                "%s/ in the bridge's folder: the tower's patch and crafts belong to the %s package "
                "(distribution/TowerCatch); a build older than the split left it in dist/"
                % (folder, TOWER))

    for source, _ in EXTRAS:
        if not os.path.exists(os.path.join(ROOT, source)):
            problems.append("missing " + source)

    if problems:
        print("%d problem(s):" % len(problems))
        for problem in problems:
            print("  " + problem)
        return 1

    # Anything the build leaves behind that must not ship.
    strays = []
    for folder, _, names in os.walk(MODDIR):
        for name in names:
            if name.endswith((".pdb", ".deps.json", ".dev.json")):
                strays.append(os.path.relpath(os.path.join(folder, name), MODDIR))
    for stray in strays:
        os.remove(os.path.join(MODDIR, stray))
        print("removed " + stray)

    for source, target in EXTRAS:
        with io.open(os.path.join(ROOT, source), "rb") as handle:
            data = handle.read()
        with io.open(os.path.join(MODDIR, target), "wb") as handle:
            handle.write(data)

    write_zip(os.path.join(DIST, "%s-%s.zip" % (BRIDGE, version)), MODDIR, BRIDGE)
    return 0


def package_tower():
    version = tower_version()
    if not os.path.isdir(TOWER_MODDIR):
        print("nothing to package - %s does not exist.\nRun build.cmd first."
              % os.path.relpath(TOWER_MODDIR, ROOT))
        return 2

    problems = []
    shipped = []
    for folder, _, names in os.walk(TOWER_MODDIR):
        for name in sorted(names):
            if not name.endswith(".dll"):
                continue
            full = os.path.join(folder, name)
            shipped.append(os.path.relpath(full, TOWER_MODDIR).replace(os.sep, "/"))
            if not os.path.exists(full[:-4] + ".xml"):
                problems.append("missing %s - without it help(conn.tower) is empty for every user"
                                % (os.path.relpath(full, TOWER_MODDIR)[:-4] + ".xml"))
    for relative in TOWER_EXPECTED:
        if relative not in shipped:
            problems.append("missing assembly: " + relative)
    for relative in shipped:
        if relative not in TOWER_EXPECTED:
            problems.append("unexpected assembly in the tower package: %s - if this is intentional add it to "
                            "TOWER_EXPECTED" % relative)
    extras = tower_extras()
    targets = [target for _, target in extras]
    for source, _ in extras:
        if not os.path.exists(os.path.join(ROOT, source)):
            problems.append("missing " + source)
    for required in TOWER_REQUIRED:
        if required not in targets:
            problems.append("missing %s in %s (the package would ship without it)" % (required, TOWER_SOURCE))
    if version == "0.0.0":
        problems.append("%s/TowerCatch.version has no readable version" % TOWER_SOURCE)
    if problems:
        print("%d problem(s):" % len(problems))
        for problem in problems:
            print("  " + problem)
        return 1

    # Anything the build leaves behind that must not ship.
    for folder, _, names in os.walk(TOWER_MODDIR):
        for name in names:
            if name.endswith((".pdb", ".deps.json", ".dev.json")):
                os.remove(os.path.join(folder, name))
                print("removed " + os.path.relpath(os.path.join(folder, name), TOWER_MODDIR))

    for source, target in extras:
        destination = os.path.join(TOWER_MODDIR, target.replace("/", os.sep))
        if not os.path.isdir(os.path.dirname(destination)):
            os.makedirs(os.path.dirname(destination))
        with io.open(os.path.join(ROOT, source.replace("/", os.sep)), "rb") as handle:
            data = handle.read()
        with io.open(destination, "wb") as handle:
            handle.write(data)

    write_zip(os.path.join(DIST, "%s-%s.zip" % (TOWER, version)), TOWER_MODDIR, TOWER)
    return 0


def main():
    version = sys.argv[1] if len(sys.argv) > 1 else version_from_changelog()
    if not version:
        print("could not determine a version: pass one, or add a '## [x.y.z]' "
              "heading to CHANGELOG.md")
        return 2

    code = package_bridge(version)
    if code:
        return code
    return package_tower()


if __name__ == "__main__":
    sys.exit(main())
