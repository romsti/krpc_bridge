#!/usr/bin/env python3
"""Write manifest.json for a package of dist/GameData: what this build ships, file by file.

    python tools/manifest.py [--package KRPC.Bridge|TowerCatch] [--stamp obj/BridgeStamp.json] [--dist DIR]

build.cmd runs it last, once per package, after the signature scan. A package is a mod folder of
GameData: KRPC.Bridge (the DLLs) and TowerCatch (the whole booster catch tower: its ModuleManager patch
and craft, from distribution/TowerCatch, and its plugin KRPC.Bridge.Tower.dll).
Each manifest gives the sha256 and size of every file in the package's folder, the commit the build
was made from and whether the tree was clean. A manifest whose tree was not clean says
"deliverable": false: it is a build to test, not one to publish.

It also refuses to write a manifest that would lie:
  * a DLL that does not carry the commit of this build (a stale DLL left in dist/ by an old
    build, or one built outside build.cmd);
  * a file that the build does not produce (for KRPC.Bridge: anything besides the assemblies of
    tools/package.py EXPECTED and their .xml, LICENSE, NOTICE and the .version file; for TowerCatch:
    anything besides the files of distribution/TowerCatch, LICENSE and the assemblies of TOWER_EXPECTED
    and their .xml), or one of the copied files
    whose bytes differ from the repo's;
  * a commit or a dirty state that changed between the stamp and now (the tree moved during the
    build).

Deterministic: no time, sorted keys, LF line endings. Two builds of one commit give the same
manifest.json, byte for byte.
"""

import argparse
import hashlib
import io
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

import build_stamp  # noqa: E402
from package import BRIDGE, EXPECTED, PACKAGES, TOWER, TOWER_EXPECTED, TOWER_SOURCE, tower_version  # noqa: E402

MANIFEST_NAME = "manifest.json"
FORMAT = 1

# Files the build copies from the repo into KRPC.Bridge, by their path in the mod folder.
REPO_FILES = {
    "LICENSE": "LICENSE",
    "NOTICE": "NOTICE",
    "KRPC.Bridge.version": "distribution/KRPC.Bridge.version",
}


def sha256_of(path):
    digest = hashlib.sha256()
    with io.open(path, "rb") as handle:
        for block in iter(lambda: handle.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def list_files(moddir):
    """Every file under the mod folder, as sorted relative paths with '/', manifest excluded."""
    found = []
    for folder, _, names in os.walk(moddir):
        for name in names:
            rel = os.path.relpath(os.path.join(folder, name), moddir).replace(os.sep, "/")
            if rel != MANIFEST_NAME:
                found.append(rel)
    return sorted(found)


def tower_files(root=ROOT):
    """Every file of the tower package's source folder (distribution/TowerCatch), as sorted relative paths."""
    return list_files(os.path.join(root, *TOWER_SOURCE.split("/")))


def expected_sources(root=ROOT, package=BRIDGE):
    """{path in the mod folder: path in the repo or None (built)} for every file a build ships in `package`.

    KRPC.Bridge: the assemblies and their .xml (built), LICENSE, NOTICE and the .version file.
    TowerCatch: everything under distribution/TowerCatch (the patch, the craft, NOTICE, the .version file),
    the repo's LICENSE, and the tower's assembly and its .xml (built).
    """
    if package not in PACKAGES:
        raise ValueError("unknown package %r (known: %s)" % (package, ", ".join(PACKAGES)))
    expected = {}
    if package == TOWER:
        for rel in tower_files(root):
            expected[rel] = TOWER_SOURCE + "/" + rel
        expected["LICENSE"] = "LICENSE"
        for dll in TOWER_EXPECTED:
            expected[dll] = None
            expected[dll[:-4] + ".xml"] = None
        return expected
    for dll in EXPECTED:
        expected[dll] = None
        expected[dll[:-4] + ".xml"] = None
    for rel, src in REPO_FILES.items():
        expected[rel] = src
    return expected


def build(moddir, stamp, root=ROOT, package=BRIDGE):
    """The manifest dict, and the list of problems that forbid writing it."""
    problems = []
    expected = expected_sources(root, package)
    files = list_files(moddir)

    for rel in files:
        if rel not in expected:
            problems.append("%s: not produced by this build (stray file in dist/: delete it)" % rel)
    for rel in sorted(expected):
        if rel not in files:
            problems.append("%s: missing from dist/" % rel)

    sha = stamp.get("git_sha", "")
    entries = []
    for rel in files:
        full = os.path.join(moddir, rel.replace("/", os.sep))
        digest = sha256_of(full)
        entry = {"path": rel, "sha256": digest, "bytes": os.path.getsize(full)}
        source = expected.get(rel)
        if source:
            src_path = os.path.join(root, source.replace("/", os.sep))
            if not os.path.isfile(src_path) or sha256_of(src_path) != digest:
                problems.append("%s: differs from %s in the repo" % (rel, source))
        if rel.endswith(".dll"):
            with io.open(full, "rb") as handle:
                stamped = len(sha) == 40 and sha.encode("ascii") in handle.read()
            entry["stamped"] = stamped
            if not stamped:
                problems.append("%s: does not carry commit %s (stale DLL, or built outside build.cmd)"
                                % (rel, sha))
        entries.append(entry)

    deliverable = (not stamp.get("git_dirty", True)) and len(sha) == 40
    version = stamp.get("mod_version", "")
    informational = stamp.get("informational_version", "")
    if package == TOWER:
        # The package has its own version (distribution/TowerCatch/TowerCatch.version), not the bridge's.
        version = tower_version(root)
        informational = "%s+%s%s" % (version, sha, ".dirty" if stamp.get("git_dirty", True) else "")
    manifest = {
        "format": FORMAT,
        "name": package,
        "mod_version": version,
        "git_sha": sha,
        "git_dirty": bool(stamp.get("git_dirty", True)),
        "source_date": stamp.get("source_date", ""),
        "informational_version": informational,
        "deliverable": deliverable,
        "files": entries,
    }
    return manifest, problems


def render(manifest):
    return (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode("utf-8")


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--stamp", default=os.path.join(ROOT, "obj", "BridgeStamp.json"),
                        help="the stamp build.cmd compiled into the DLLs")
    parser.add_argument("--package", default=BRIDGE, choices=PACKAGES,
                        help="the mod folder of GameData this manifest describes (default: %s)" % BRIDGE)
    parser.add_argument("--dist", default=None,
                        help="the package's folder in dist (default: dist/GameData/<package>)")
    args = parser.parse_args(argv)
    if args.dist is None:
        args.dist = os.path.join(ROOT, "dist", "GameData", args.package)

    try:
        with io.open(args.stamp, encoding="utf-8") as handle:
            stamp = json.load(handle)
    except (OSError, ValueError) as exc:
        print("  cannot read the build stamp %s: %s" % (args.stamp, exc))
        return 1

    now = build_stamp.compute()
    if now["git_sha"] != stamp.get("git_sha") or now["git_dirty"] != stamp.get("git_dirty"):
        print("  the tree changed during the build: stamped %s%s, now %s%s - build again"
              % (stamp.get("git_sha"), " dirty" if stamp.get("git_dirty") else "",
                 now["git_sha"], " dirty" if now["git_dirty"] else ""))
        return 1

    manifest, problems = build(args.dist, stamp, package=args.package)
    if problems:
        print("  %d problem(s), manifest NOT written:" % len(problems))
        for problem in problems:
            print("    " + problem)
        return 1

    path = os.path.join(args.dist, MANIFEST_NAME)
    with io.open(path, "wb") as handle:
        handle.write(render(manifest))
    print("  wrote %s: %d files, commit %s, %s"
          % (os.path.relpath(path, ROOT), len(manifest["files"]), manifest["git_sha"],
             "deliverable" if manifest["deliverable"] else "NOT deliverable (dirty tree)"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
