using System.Reflection;

// This plugin's own identity. No KSPAssemblyDependency on KRPC.Bridge.Core: this ground-test
// tool must load next to any Core version (see the csproj).
[assembly: KSPAssembly ("KRPC.Bridge.Tower", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Tower")]
[assembly: AssemblyDescription ("Ground-test tools for the booster catch tower: place a vessel at an exact pose and hold it still; the towers of a game from the Space Center (conn.tower_site).")]
[assembly: AssemblyVersion ("2.0.0.0")]
[assembly: AssemblyFileVersion ("2.0.0.0")]
