using System.Reflection;

// This plugin's own identity. It ships in GameData/TowerCatch/Plugins, a folder of its own, so
// KSP's AssemblyLoader must be told to load it after the Core: without the dependency, nothing says
// that the Core's assembly is there when this one starts (the layouts it declares into the Core's
// registry, TowerLayouts). If the Core is absent or older than 1.3, KSP skips THIS assembly and says
// so in KSP.log instead of loading a tower that has no bridge. A name, not a reference: the assembly
// still compiles without the Core (see the csproj).
[assembly: KSPAssembly ("KRPC.Bridge.Tower", 1, 0)]
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Core", 1, 3)]

[assembly: AssemblyTitle ("KRPC.Bridge.Tower")]
[assembly: AssemblyDescription ("Ground-test tools for the booster catch tower: place a vessel at an exact pose and hold it still; the towers of a game from the Space Center (conn.tower_site).")]
[assembly: AssemblyVersion ("2.1.0.0")]
[assembly: AssemblyFileVersion ("2.1.0.0")]
