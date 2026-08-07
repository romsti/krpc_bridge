#!/usr/bin/env python3
"""Build the release zip from dist/.

    python tools/package.py [version]

Produces  dist/KRPC.Bridge-<version>.zip  containing a GameData/ folder the user unzips
straight into their KSP install. Run build.cmd first - this packages, it does not compile.

Two rules it enforces, because both are easy to get wrong by hand and expensive to get
wrong in public:

  * Every .dll must be accompanied by its .xml. kRPC reads the XML next to each assembly
    to build the Python docstrings; ship without it and help(conn.fmrs) is empty for
    everyone who installs the mod.

  * LICENSE and NOTICE must be inside the zip. The KSP Add-on Posting Rules require the
    full licence text in the download itself, not merely a licence name on the download
    page.
"""

import io
import os
import re
import sys
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIST = os.path.join(ROOT, "dist")
GAMEDATA = os.path.join(DIST, "GameData")
MODDIR = os.path.join(GAMEDATA, "KRPC.Bridge")

EXPECTED = [
    "KRPC.Bridge.Core.dll",
    "Plugins/KRPC.Bridge.Fmrs.dll",
    "Plugins/KRPC.Bridge.Ocisly.dll",
    "Plugins/KRPC.Bridge.MechJeb.dll",
]

EXTRAS = [
    ("LICENSE", "LICENSE"),
    ("NOTICE", "NOTICE"),
    ("distribution/KRPC.Bridge.version", "KRPC.Bridge.version"),
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


def main():
    version = sys.argv[1] if len(sys.argv) > 1 else version_from_changelog()
    if not version:
        print("could not determine a version: pass one, or add a '## [x.y.z]' "
              "heading to CHANGELOG.md")
        return 2

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
        if relative not in EXPECTED:
            problems.append(
                "unexpected assembly: %s - if this is intentional add it to EXPECTED, "
                "otherwise something is being shipped by accident" % relative)

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

    output = os.path.join(DIST, "KRPC.Bridge-%s.zip" % version)
    with zipfile.ZipFile(output, "w", zipfile.ZIP_DEFLATED) as archive:
        for folder, _, names in os.walk(GAMEDATA):
            for name in sorted(names):
                full = os.path.join(folder, name)
                # Store paths relative to dist/, so the zip root is GameData/.
                archive.write(full, os.path.relpath(full, DIST).replace(os.sep, "/"))

    print()
    with zipfile.ZipFile(output) as archive:
        for info in archive.infolist():
            print("  %-52s %7d" % (info.filename, info.file_size))
    print()
    print("wrote %s (%.0f kB)" % (os.path.relpath(output, ROOT),
                                  os.path.getsize(output) / 1024.0))
    return 0


if __name__ == "__main__":
    sys.exit(main())
