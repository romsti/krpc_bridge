using System;
using System.Collections.Generic;
using System.IO;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// Where the catch tower's patch and crafts live in GameData, and the search of a craft among the folders
    /// that can hold one. The tower is a package of its own, GameData/TowerCatch (Patches/TowerCatch.cfg and
    /// Crafts/Tour Catch v5.craft); until the split it travelled inside the bridge's folder,
    /// GameData/KRPC.Bridge, and that folder stays a fallback for the crafts, so a game whose bridge is new and
    /// whose tower package is not installed yet (or the other way round) still finds its craft.
    ///
    /// The patch is another matter: two copies of it make every TG.* part twice (ModuleManager's +PART), and no
    /// copy at all makes KSP delete the towers of a save at load. PackageFiles says which of the two folders
    /// holds each file, for a client to judge (the client's installation check says the double); an
    /// installer must never leave two.
    ///
    /// PURE: System only (build/tests/tower).
    /// </summary>
    internal static class TowerPackage
    {
        /// <summary>The package's mod folder: GameData/TowerCatch.</summary>
        internal const string Folder = "TowerCatch";

        /// <summary>The bridge's mod folder, which held the patch and the crafts before the split.</summary>
        internal const string LegacyFolder = "KRPC.Bridge";

        /// <summary>The patch's file name, under Patches/ of either folder.</summary>
        internal const string PatchFile = "TowerCatch.cfg";

        /// <summary>The mod folders that can hold the patch and the crafts, in the order the crafts are searched.</summary>
        internal static readonly string[] Folders = { Folder, LegacyFolder };

        /// <summary>The Crafts folders of the mod folders, under GameData, in the order of search.</summary>
        internal static List<string> CraftDirs (string gameData)
        {
            var dirs = new List<string> ();
            foreach (var folder in Folders)
                dirs.Add (Path.Combine (Path.Combine (gameData, folder), "Crafts"));
            return dirs;
        }

        /// <summary>
        /// The first &lt;craftName&gt;.craft found, looking in these folders in order (the game's own copy
        /// first, then the package's, then the bridge's), or null. Every path looked at goes to
        /// <paramref name="tried"/>. ArgumentException when a name resolves outside the folder it is looked up in
        /// (defence in depth: the name was checked to be a bare file name).
        /// </summary>
        internal static string FindCraft (IList<string> dirs, string craftName, Func<string, bool> exists, IList<string> tried)
        {
            foreach (var dir in dirs) {
                var path = Path.GetFullPath (Path.Combine (dir, craftName + ".craft"));
                if (!string.Equals (Path.GetDirectoryName (path), Path.GetFullPath (dir), StringComparison.OrdinalIgnoreCase))
                    throw new ArgumentException ("craftName resolves outside " + dir, "craftName");
                if (tried != null)
                    tried.Add (path);
                if (exists (path))
                    return path;
            }
            return null;
        }

        /// <summary>
        /// For each place the tower's files can be, one row, "path", a tab, then 1 when the file is there, else 0:
        /// the path is relative to GameData with '/', the patch of each mod folder (package first), then the craft
        /// of each. Everything is probed, nothing short-circuits: two 1s on the patch is the double that breaks
        /// the parts.
        /// </summary>
        internal static List<string> Rows (string gameData, string craftName, Func<string, bool> exists)
        {
            var rows = new List<string> ();
            foreach (var folder in Folders)
                rows.Add (Row (gameData, folder + "/Patches/" + PatchFile, exists));
            foreach (var folder in Folders)
                rows.Add (Row (gameData, folder + "/Crafts/" + TowerSiteFormat.Clean (craftName) + ".craft", exists));
            return rows;
        }

        static string Row (string gameData, string relative, Func<string, bool> exists)
        {
            var full = Path.GetFullPath (Path.Combine (gameData, relative.Replace ('/', Path.DirectorySeparatorChar)));
            return relative + "\t" + TowerSiteFormat.Flag (exists (full));
        }
    }
}
