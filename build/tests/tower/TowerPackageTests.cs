// -----------------------------------------------------------------------------------
// The tower package (GameData/TowerCatch, split from the bridge's folder): pure tests of TowerPackage, the
// search of a craft among the game's Ships folders, the package's Crafts folder and the bridge's old one, and
// the rows of tower_package_v1 that say where the patch and the craft are. See TowerTests.csproj; run by
// Program.Main.
//
// No file system is touched: "exists" is a set of paths, the folders are made up.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.IO;

namespace KRPC.Bridge.Tower
{
    public static partial class Program
    {
        static void TowerPackageTests ()
        {
            TestPackageFolders ();
            TestCraftSearchOrder ();
            TestCraftSearchTransition ();
            TestCraftSearchRefusesAnEscape ();
            TestPackageRows ();
        }

        static string FakeGameData ()
        {
            return Path.GetFullPath (Path.Combine (Path.Combine (Path.GetTempPath (), "ksp_fake_root"), "GameData"));
        }

        static string FakeShips ()
        {
            return Path.GetFullPath (Path.Combine (Path.Combine (Path.Combine (Path.GetTempPath (), "ksp_fake_root"), "saves"), "roston"));
        }

        static string P (params string[] parts)
        {
            return Path.GetFullPath (Path.Combine (parts));
        }

        static List<string> SearchDirs ()
        {
            var ships = Path.Combine (FakeShips (), "Ships");
            var dirs = new List<string> { Path.Combine (ships, "VAB"), Path.Combine (ships, "SPH") };
            dirs.AddRange (TowerPackage.CraftDirs (FakeGameData ()));
            return dirs;
        }

        static void TestPackageFolders ()
        {
            Equal ("TowerCatch", TowerPackage.Folder, "the package folder");
            Equal ("KRPC.Bridge", TowerPackage.LegacyFolder, "the bridge folder");
            Equal ("TowerCatch.cfg", TowerPackage.PatchFile, "the patch file");
            Equal (2, TowerPackage.Folders.Length, "two mod folders can hold the tower");
            Equal ("TowerCatch", TowerPackage.Folders [0], "the package comes first");
            var dirs = TowerPackage.CraftDirs (FakeGameData ());
            Equal (2, dirs.Count, "two Crafts folders");
            Equal (P (FakeGameData (), "TowerCatch", "Crafts"), P (dirs [0]), "the package's Crafts first");
            Equal (P (FakeGameData (), "KRPC.Bridge", "Crafts"), P (dirs [1]), "the bridge's Crafts second");
        }

        // The game's own copy wins, then the package's, then the bridge's.
        static void TestCraftSearchOrder ()
        {
            var dirs = SearchDirs ();
            var ships = Path.Combine (FakeShips (), "Ships");
            var vab = P (ships, "VAB", "Tour Catch v5.craft");
            var sph = P (ships, "SPH", "Tour Catch v5.craft");
            var package = P (FakeGameData (), "TowerCatch", "Crafts", "Tour Catch v5.craft");
            var legacy = P (FakeGameData (), "KRPC.Bridge", "Crafts", "Tour Catch v5.craft");

            var all = new HashSet<string> (StringComparer.OrdinalIgnoreCase) { vab, sph, package, legacy };
            var tried = new List<string> ();
            Equal (vab, TowerPackage.FindCraft (dirs, "Tour Catch v5", all.Contains, tried), "VAB beats everything");
            Equal (1, tried.Count, "it stops at the first copy");

            all.Remove (vab);
            Equal (sph, TowerPackage.FindCraft (dirs, "Tour Catch v5", all.Contains, null), "then SPH");
            all.Remove (sph);
            Equal (package, TowerPackage.FindCraft (dirs, "Tour Catch v5", all.Contains, null), "then the tower package (both folders present: the package wins)");
            all.Remove (package);
            Equal (legacy, TowerPackage.FindCraft (dirs, "Tour Catch v5", all.Contains, null), "then the bridge's old folder (the fallback)");
            all.Remove (legacy);
            tried = new List<string> ();
            Check (TowerPackage.FindCraft (dirs, "Tour Catch v5", all.Contains, tried) == null, "no craft anywhere: null");
            Equal (4, tried.Count, "every place is named in the refusal");
            Equal (package, tried [2], "the package's folder is one of them");
            Equal (legacy, tried [3], "the bridge's folder is the last");
        }

        // The two sides of the transition: a new bridge whose package is installed, a game whose package is not
        // installed yet, and a craft that only one folder holds.
        static void TestCraftSearchTransition ()
        {
            var dirs = SearchDirs ();
            var package = P (FakeGameData (), "TowerCatch", "Crafts", "Tour Catch v5.craft");
            var legacy = P (FakeGameData (), "KRPC.Bridge", "Crafts", "Tour Catch v5.craft");
            var onlyPackage = new HashSet<string> (StringComparer.OrdinalIgnoreCase) { package };
            var onlyLegacy = new HashSet<string> (StringComparer.OrdinalIgnoreCase) { legacy };
            Equal (package, TowerPackage.FindCraft (dirs, "Tour Catch v5", onlyPackage.Contains, null), "the package alone is found");
            Equal (legacy, TowerPackage.FindCraft (dirs, "Tour Catch v5", onlyLegacy.Contains, null), "the old bridge folder alone is found (an install not migrated yet)");
            Check (TowerPackage.FindCraft (dirs, "Tour Catch v4", onlyPackage.Contains, null) == null, "another craft's name finds nothing");
        }

        static void TestCraftSearchRefusesAnEscape ()
        {
            var dirs = SearchDirs ();
            Check (Throws (() => TowerPackage.FindCraft (dirs, "../Tour Catch v5", p => false, null)), "a name that leaves the folder is refused");
            Check (Throws (() => TowerPackage.FindCraft (dirs, "sub/Tour Catch v5", p => false, null)), "a name that goes into a sub-folder is refused");
        }

        // The patch: exactly one is right; the rows let a client see none and two.
        static void TestPackageRows ()
        {
            var gd = FakeGameData ();
            var patchNew = P (gd, "TowerCatch", "Patches", "TowerCatch.cfg");
            var patchOld = P (gd, "KRPC.Bridge", "Patches", "TowerCatch.cfg");
            var craftNew = P (gd, "TowerCatch", "Crafts", "Tour Catch v5.craft");
            var craftOld = P (gd, "KRPC.Bridge", "Crafts", "Tour Catch v5.craft");

            Func<HashSet<string>, List<string>> rows = present => TowerPackage.Rows (gd, "Tour Catch v5", present.Contains);
            var none = new HashSet<string> (StringComparer.OrdinalIgnoreCase);
            var r = rows (none);
            Equal (4, r.Count, "four places are probed");
            Equal ("TowerCatch/Patches/TowerCatch.cfg\t0", r [0], "the package's patch, absent");
            Equal ("KRPC.Bridge/Patches/TowerCatch.cfg\t0", r [1], "the bridge's patch, absent");
            Equal ("TowerCatch/Crafts/Tour Catch v5.craft\t0", r [2], "the package's craft, absent");
            Equal ("KRPC.Bridge/Crafts/Tour Catch v5.craft\t0", r [3], "the bridge's craft, absent");

            // The new layout.
            r = rows (new HashSet<string> (StringComparer.OrdinalIgnoreCase) { patchNew, craftNew });
            Equal ("TowerCatch/Patches/TowerCatch.cfg\t1", r [0], "the package's patch, there");
            Equal ("KRPC.Bridge/Patches/TowerCatch.cfg\t0", r [1], "the bridge carries no patch");
            Equal ("TowerCatch/Crafts/Tour Catch v5.craft\t1", r [2], "the package's craft, there");
            Equal ("KRPC.Bridge/Crafts/Tour Catch v5.craft\t0", r [3], "the bridge carries no craft");

            // The transition's danger: the patch twice. Nothing short-circuits, both are said.
            r = rows (new HashSet<string> (StringComparer.OrdinalIgnoreCase) { patchNew, patchOld, craftNew, craftOld });
            Equal ("TowerCatch/Patches/TowerCatch.cfg\t1", r [0], "double: the package's patch");
            Equal ("KRPC.Bridge/Patches/TowerCatch.cfg\t1", r [1], "double: the bridge's patch too");
            Equal ("KRPC.Bridge/Crafts/Tour Catch v5.craft\t1", r [3], "double: the bridge's craft too");

            // The old layout: only the bridge's folder.
            r = rows (new HashSet<string> (StringComparer.OrdinalIgnoreCase) { patchOld, craftOld });
            Equal ("TowerCatch/Patches/TowerCatch.cfg\t0", r [0], "old layout: no package");
            Equal ("KRPC.Bridge/Patches/TowerCatch.cfg\t1", r [1], "old layout: the bridge's patch");

            // Every row is "path<TAB>flag" for the client's decoder, with no tab in the path.
            foreach (var row in r) {
                var cut = row.Split ('\t');
                Check (cut.Length == 2 && (cut [1] == "0" || cut [1] == "1") && cut [0].IndexOf ('/') > 0, "row shape: " + row);
            }
        }
    }
}
