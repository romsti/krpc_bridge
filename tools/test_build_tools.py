"""Unit tests of tools/build_stamp.py and tools/manifest.py, on temporary folders.

    python -m unittest discover -s tools -p "test_*.py"
"""

import io
import json
import os
import shutil
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import build_stamp  # noqa: E402
import manifest  # noqa: E402
import package  # noqa: E402
from package import EXPECTED, TOWER_EXPECTED  # noqa: E402

SHA = "0123456789abcdef0123456789abcdef01234567"


def write(path, data):
    parent = os.path.dirname(path)
    if not os.path.isdir(parent):
        os.makedirs(parent)
    with io.open(path, "wb") as handle:
        handle.write(data)


class StampTests(unittest.TestCase):

    def stamp(self, dirty=False):
        return {"git_sha": SHA, "git_dirty": dirty, "dirty_paths": [], "source_date":
                "2026-10-03T17:56:02Z", "mod_version": "1.1.0",
                "informational_version": "1.1.0+" + SHA + (".dirty" if dirty else "")}

    def test_render_is_deterministic_and_complete(self):
        a = build_stamp.render_cs(self.stamp())
        self.assertEqual(a, build_stamp.render_cs(self.stamp()))
        self.assertIn('AssemblyInformationalVersion ("1.1.0+%s")' % SHA, a)
        self.assertIn('AssemblyMetadata ("KRPC.Bridge.GitSha", "%s")' % SHA, a)
        self.assertIn('AssemblyMetadata ("KRPC.Bridge.GitDirty", "0")', a)
        self.assertIn('AssemblyMetadata ("KRPC.Bridge.SourceDate", "2026-10-03T17:56:02Z")', a)
        self.assertIn('"KRPC.Bridge.GitDirty", "1"', build_stamp.render_cs(self.stamp(True)))
        self.assertNotIn("\r", a)

    def test_dirty_paths_lists_what_git_reports(self):
        saved = build_stamp._git
        try:
            build_stamp._git = lambda args, cwd=None: ""
            self.assertEqual([], build_stamp.dirty_paths())
            build_stamp._git = lambda args, cwd=None: (" M build.cmd\n"
                                                       "R  a.cs -> b.cs\n")
            self.assertEqual(["build.cmd", "b.cs"], build_stamp.dirty_paths())
            build_stamp._git = lambda args, cwd=None: None
            self.assertIsNone(build_stamp.dirty_paths())
        finally:
            build_stamp._git = saved

    def test_without_git_the_stamp_is_unknown_and_dirty(self):
        saved = build_stamp._git
        try:
            build_stamp._git = lambda args, cwd=None: None
            s = build_stamp.compute()
            self.assertEqual(build_stamp.UNKNOWN_SHA, s["git_sha"])
            self.assertTrue(s["git_dirty"])
        finally:
            build_stamp._git = saved

    def test_write_only_when_changed(self):
        tmp = tempfile.mkdtemp(prefix="stamp_test_")
        try:
            build_stamp.write(tmp, self.stamp())
            cs = os.path.join(tmp, "BridgeStamp.g.cs")
            os.utime(cs, (1000000000, 1000000000))
            build_stamp.write(tmp, self.stamp())
            self.assertEqual(1000000000, int(os.path.getmtime(cs)), "unchanged stamp: file untouched")
            with io.open(os.path.join(tmp, "BridgeStamp.json"), encoding="utf-8") as h:
                self.assertEqual(SHA, json.load(h)["git_sha"])
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class ManifestTests(unittest.TestCase):

    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="manifest_test_")
        self.mod = os.path.join(self.root, "dist", "GameData", "KRPC.Bridge")
        for rel, src in manifest.REPO_FILES.items():
            write(os.path.join(self.root, src), ("repo " + rel).encode())
            write(os.path.join(self.mod, rel), ("repo " + rel).encode())
        for dll in EXPECTED:
            write(os.path.join(self.mod, dll), b"MZ " + dll.encode() + b" " + SHA.encode())
            write(os.path.join(self.mod, dll[:-4] + ".xml"), b"<doc/>")
        self.stamp = {"git_sha": SHA, "git_dirty": False, "source_date": "2026-10-03T17:56:02Z",
                      "mod_version": "1.1.0", "informational_version": "1.1.0+" + SHA}

    def tearDown(self):
        shutil.rmtree(self.root, ignore_errors=True)

    def test_clean_build(self):
        m, problems = manifest.build(self.mod, self.stamp, self.root)
        self.assertEqual([], problems)
        self.assertTrue(m["deliverable"])
        self.assertEqual(SHA, m["git_sha"])
        paths = [f["path"] for f in m["files"]]
        self.assertEqual(sorted(paths), paths)
        self.assertNotIn("manifest.json", paths)
        self.assertEqual(len(EXPECTED) * 2 + len(manifest.REPO_FILES), len(paths))
        self.assertTrue(all(f["stamped"] for f in m["files"] if f["path"].endswith(".dll")))
        self.assertEqual("KRPC.Bridge", m["name"])

    def test_the_bridge_ships_nothing_of_the_tower(self):
        """The tower's patch, craft and plugin belong to the TowerCatch package: in the bridge's folder they would
        be a second copy of the patch (every part twice) and of the plugin (the service Tower twice)."""
        write(os.path.join(self.mod, "Patches", "TowerCatch.cfg"), b"patch")
        write(os.path.join(self.mod, "Crafts", "Tour Catch v5.craft"), b"craft")
        write(os.path.join(self.mod, "Plugins", "KRPC.Bridge.Tower.dll"), b"MZ " + SHA.encode())
        _, problems = manifest.build(self.mod, self.stamp, self.root)
        text = "\n".join(problems)
        self.assertIn("Patches/TowerCatch.cfg: not produced by this build", text)
        self.assertIn("Crafts/Tour Catch v5.craft: not produced by this build", text)
        self.assertIn("Plugins/KRPC.Bridge.Tower.dll: not produced by this build", text)
        expected = "\n".join(manifest.expected_sources(self.root))
        self.assertNotIn("Patches/TowerCatch.cfg", expected)
        self.assertNotIn("Tower", expected)

    def test_the_bridge_ships_no_mechjeb(self):
        self.assertFalse([e for e in EXPECTED if "MechJeb" in e], "MechJeb is retired from the bridge")
        write(os.path.join(self.mod, "Plugins", "KRPC.Bridge.MechJeb.dll"), b"MZ " + SHA.encode())
        _, problems = manifest.build(self.mod, self.stamp, self.root)
        self.assertIn("Plugins/KRPC.Bridge.MechJeb.dll: not produced by this build", "\n".join(problems))

    def test_render_is_deterministic(self):
        a = manifest.render(manifest.build(self.mod, self.stamp, self.root)[0])
        b = manifest.render(manifest.build(self.mod, self.stamp, self.root)[0])
        self.assertEqual(a, b)
        self.assertNotIn(b"\r", a)

    def test_dirty_build_is_not_deliverable(self):
        dirty = dict(self.stamp, git_dirty=True)
        m, problems = manifest.build(self.mod, dirty, self.root)
        self.assertEqual([], problems)
        self.assertFalse(m["deliverable"])

    def test_stale_dll_stray_file_missing_file_and_edited_extra(self):
        write(os.path.join(self.mod, "Plugins", "KRPC.Bridge.Ident.dll"), b"MZ old build")
        write(os.path.join(self.mod, "Plugins", "KRPC.Bridge.dll"), b"MZ " + SHA.encode())
        os.remove(os.path.join(self.mod, "Plugins", "KRPC.Bridge.Stock.xml"))
        write(os.path.join(self.mod, "NOTICE"), b"edited notice")
        _, problems = manifest.build(self.mod, self.stamp, self.root)
        text = "\n".join(problems)
        self.assertIn("KRPC.Bridge.Ident.dll: does not carry commit", text)
        self.assertIn("Plugins/KRPC.Bridge.dll: not produced by this build", text)
        self.assertIn("Plugins/KRPC.Bridge.Stock.xml: missing from dist/", text)
        self.assertIn("NOTICE: differs from NOTICE in the repo", text)


class TowerPackageManifestTests(unittest.TestCase):
    """The TowerCatch package: the files of distribution/TowerCatch, the licence, and the tower's plugin (built)."""

    SOURCE = {
        "Patches/TowerCatch.cfg": b"patch",
        "Crafts/Tour Catch v5.craft": b"craft",
        "NOTICE": b"notice of the package",
        "TowerCatch.version": b'{"VERSION": {"MAJOR": 1, "MINOR": 2, "PATCH": 3}}',
    }

    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="manifest_tower_test_")
        self.mod = os.path.join(self.root, "dist", "GameData", "TowerCatch")
        for rel, data in self.SOURCE.items():
            write(os.path.join(self.root, "distribution", "TowerCatch", rel), data)
            write(os.path.join(self.mod, rel), data)
        write(os.path.join(self.root, "LICENSE"), b"mit")
        write(os.path.join(self.mod, "LICENSE"), b"mit")
        for dll in TOWER_EXPECTED:
            write(os.path.join(self.mod, dll), b"MZ " + dll.encode() + b" " + SHA.encode())
            write(os.path.join(self.mod, dll[:-4] + ".xml"), b"<doc/>")
        self.stamp = {"git_sha": SHA, "git_dirty": False, "source_date": "2026-10-03T17:56:02Z",
                      "mod_version": "1.1.0", "informational_version": "1.1.0+" + SHA}

    def tearDown(self):
        shutil.rmtree(self.root, ignore_errors=True)

    def build(self, stamp=None):
        return manifest.build(self.mod, stamp or self.stamp, self.root, package="TowerCatch")

    def test_clean_build(self):
        m, problems = self.build()
        self.assertEqual([], problems)
        self.assertTrue(m["deliverable"])
        self.assertEqual("TowerCatch", m["name"])
        built = [d for dll in TOWER_EXPECTED for d in (dll, dll[:-4] + ".xml")]
        self.assertEqual(sorted(list(self.SOURCE) + ["LICENSE"] + built), [f["path"] for f in m["files"]])
        self.assertEqual(["Plugins/KRPC.Bridge.Tower.dll"], [f["path"] for f in m["files"] if "stamped" in f],
                         "the tower's plugin is in the package, and carries the commit")
        self.assertTrue(all(f["stamped"] for f in m["files"] if "stamped" in f))

    def test_the_package_has_its_own_version(self):
        m, _ = self.build()
        self.assertEqual("1.2.3", m["mod_version"])
        self.assertEqual("1.2.3+" + SHA, m["informational_version"])
        dirty = dict(self.stamp, git_dirty=True)
        m, _ = self.build(dirty)
        self.assertEqual("1.2.3+" + SHA + ".dirty", m["informational_version"])
        self.assertFalse(m["deliverable"])

    def test_render_is_deterministic(self):
        a = manifest.render(self.build()[0])
        b = manifest.render(self.build()[0])
        self.assertEqual(a, b)
        self.assertNotIn(b"\r", a)

    def test_stray_missing_and_edited_files(self):
        write(os.path.join(self.mod, "Plugins", "KRPC.Bridge.Ident.dll"), b"MZ " + SHA.encode())
        os.remove(os.path.join(self.mod, "Plugins", "KRPC.Bridge.Tower.xml"))
        os.remove(os.path.join(self.mod, "NOTICE"))
        write(os.path.join(self.mod, "Patches", "TowerCatch.cfg"), b"patch edited after the copy")
        write(os.path.join(self.mod, "LICENSE"), b"another licence")
        _, problems = self.build()
        text = "\n".join(problems)
        self.assertIn("Plugins/KRPC.Bridge.Ident.dll: not produced by this build", text)
        self.assertIn("Plugins/KRPC.Bridge.Tower.xml: missing from dist/", text)
        self.assertIn("NOTICE: missing from dist/", text)
        self.assertIn("Patches/TowerCatch.cfg: differs from distribution/TowerCatch/Patches/TowerCatch.cfg", text)
        self.assertIn("LICENSE: differs from LICENSE in the repo", text)

    def test_an_unknown_package_is_refused(self):
        with self.assertRaises(ValueError):
            manifest.expected_sources(self.root, "Elsewhere")


class PackageToolTests(unittest.TestCase):
    """tools/package.py: the tower's files are listed from distribution/TowerCatch, and its version is its own."""

    def test_the_tower_plugin_is_the_packages_and_not_the_bridges(self):
        self.assertEqual(["Plugins/KRPC.Bridge.Tower.dll"], package.TOWER_EXPECTED)
        self.assertFalse([e for e in EXPECTED if "Tower" in e])

    def test_the_real_tower_source_has_what_the_package_requires(self):
        targets = [target for _, target in package.tower_extras()]
        for required in package.TOWER_REQUIRED:
            self.assertIn(required, targets)
        self.assertIn("LICENSE", targets)
        self.assertFalse([t for t in targets if t.endswith((".dll", ".xml"))],
                         "the plugin is built into the package, it is not a source file")

    def test_the_bridge_extras_carry_no_patch(self):
        self.assertFalse([t for _, t in package.EXTRAS if t.startswith(("Patches/", "Crafts/"))])

    def test_the_real_tower_version(self):
        self.assertRegex(package.tower_version(), r"^[0-9]+\.[0-9]+\.[0-9]+$")
        self.assertNotEqual("0.0.0", package.tower_version())

    def test_the_real_source_files_agree_with_the_netkan(self):
        with io.open(os.path.join(package.ROOT, "distribution", "TowerCatch.netkan"), encoding="utf-8") as h:
            netkan = json.load(h)
        self.assertEqual("TowerCatch", netkan["identifier"])
        depends = dict((d["name"], d) for d in netkan["depends"])
        for name in ("ModuleManager", "kRPC", "KRPC-Bridge", "ModularLaunchPads", "HullcamVDSContinued",
                     "MakingHistory-DLC", "BreakingGround-DLC"):
            self.assertIn(name, depends)
        # CKAN numbers kRPC "v0.7.0" (with the v), and compares the text before the digits first.
        self.assertEqual("v0.7.0", depends["kRPC"]["min_version"])
        self.assertEqual("2.1.0", depends["KRPC-Bridge"]["min_version"])
        self.assertEqual("TowerCatch", netkan["install"][0]["find"])
        with io.open(os.path.join(package.ROOT, "distribution", "TowerCatch", "TowerCatch.version"),
                     encoding="utf-8") as h:
            avc = json.load(h)
        self.assertEqual("TowerCatch", avc["NAME"])
        # The paid expansions CKAN cannot install are said in the description.
        self.assertIn("Breaking Ground", netkan["description"])
        self.assertIn("Making History", netkan["description"])


class NetkanTests(unittest.TestCase):
    """distribution/*.netkan, written to the spec of CKAN (Spec.md, "$kref" and "$vref"): the two zips of one GitHub
    release are told apart by their names, and each mod takes its version from its own zip's name."""

    KREF = "#/ckan/github/romsti/krpc_bridge/version_from_asset/"

    def load(self, name):
        with io.open(os.path.join(package.ROOT, "distribution", name), encoding="utf-8") as h:
            return json.load(h)

    def avc_version(self, rel):
        with io.open(os.path.join(package.ROOT, "distribution", rel), encoding="utf-8") as h:
            v = json.load(h)["VERSION"]
        return "%d.%d.%d" % (v["MAJOR"], v["MINOR"], v["PATCH"])

    def test_each_record_picks_its_own_zip_and_takes_its_version_from_the_name(self):
        import re
        zips = ["KRPC.Bridge-%s.zip" % self.avc_version("KRPC.Bridge.version"),
                "TowerCatch-%s.zip" % package.tower_version()]
        for netkan, own, version in (("KRPC.Bridge.netkan", zips[0], self.avc_version("KRPC.Bridge.version")),
                                     ("TowerCatch.netkan", zips[1], package.tower_version())):
            record = self.load(netkan)
            self.assertTrue(record["$kref"].startswith(self.KREF), netkan)
            pattern = re.compile(record["$kref"][len(self.KREF):].replace("(?<version>", "(?P<version>"))
            matches = [z for z in zips if pattern.search(z)]
            self.assertEqual([own], matches, "%s must match exactly one asset of the release" % netkan)
            self.assertEqual(version, pattern.search(own).group("version"))
            self.assertEqual("#/ckan/ksp-avc", record["$vref"])
            # asset_match is part of the $kref path, not an x_netkan_github field (that one knows only
            # use_source_archive and prereleases): the old field would be ignored and the release would have
            # two zips for one record.
            self.assertNotIn("x_netkan_github", record)

    def test_the_bridge_record_holds_nothing_of_mechjeb_or_the_tower(self):
        record = self.load("KRPC.Bridge.netkan")
        text = json.dumps(record)
        self.assertNotIn("MechJeb", text)
        self.assertEqual("KRPC-Bridge", record["identifier"])
        depends = dict((d["name"], d) for d in record["depends"])
        self.assertEqual("v0.7.0", depends["kRPC"]["min_version"])
        suggests = [d["name"] for d in record["suggests"]]
        self.assertIn("TowerCatch", suggests)
        self.assertNotIn("MechJeb2", suggests)
        # The old TowerCatch carried no plugin: next to this bridge the tower would have no service.
        self.assertIn({"name": "TowerCatch", "max_version": "1.0.0"}, record["conflicts"])

    def test_the_tower_record_needs_the_bridge_that_does_not_carry_the_plugin(self):
        record = self.load("TowerCatch.netkan")
        bridge = [d for d in record["depends"] if d["name"] == "KRPC-Bridge"][0]
        self.assertEqual(self.avc_version("KRPC.Bridge.version"), bridge["min_version"])


if __name__ == "__main__":
    unittest.main()
