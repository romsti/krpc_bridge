using System.Reflection;

// This plugin's own identity.
[assembly: KSPAssembly ("KRPC.Bridge.Template", 1, 0)]

// The runtime half of the dependency, and the reason the architecture is safe.
//
// KSP's AssemblyLoader topologically sorts every DLL under GameData by these attributes
// before loading any of them. Declaring the dependency buys two things:
//
//   - Core is guaranteed loaded first, so this assembly's static state can use it
//     immediately, from Instantly-startup addons included.
//   - If Core is absent or older than 1.0, KSP skips THIS assembly and says so in
//     KSP.log, instead of loading it and throwing TypeLoadException on first touch.
//     A missing Core degrades to "the Template service is not there", which a script can
//     detect with conn.bridge.has_plugin(), rather than to a broken install.
//
// Check it landed:
//     findstr /C:"KRPC.Bridge" "D:\Games\KSP_1.12.5\KSP.log"
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Core", 1, 0)]

// Add one line per plugin this plugin depends on:
// [assembly: KSPAssemblyDependency ("KRPC.Bridge.Terrain", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Template")]
[assembly: AssemblyDescription ("Skeleton bridge plugin. Copy to start a new one.")]
[assembly: AssemblyVersion ("1.0.0.0")]
[assembly: AssemblyFileVersion ("1.0.0.0")]
