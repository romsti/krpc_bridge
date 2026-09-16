using System.Reflection;

// ---------------------------------------------------------------------------------
// Load order. KSP's AssemblyLoader reads these attributes from every DLL under
// GameData, builds a dependency graph, sorts it topologically and loads in that order -
// the folder layout has nothing to do with it.
//
// The dependency on the Core buys two things:
//   * Core's types are in the AppDomain before this assembly's first KSPAddon runs,
//     so registration cannot race,
//   * if the Core is missing or too old, KSP SKIPS this assembly and says so in the
//     log, rather than loading it and throwing TypeLoadException on first use.
//
// There is deliberately NO KSPAssemblyDependency on "Trajectories" itself: the mod is
// reached by reflection and is optional. Declaring it would make KSP skip this plugin
// when Trajectories is absent, and the service would vanish instead of answering
// available() = False.
// ---------------------------------------------------------------------------------
[assembly: KSPAssembly ("KRPC.Bridge.Trajectories", 1, 0)]
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Core", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Trajectories")]
[assembly: AssemblyDescription ("kRPC service for the Trajectories mod: atmospheric impact prediction, target and descent profile.")]
[assembly: AssemblyVersion ("1.1.0.0")]
[assembly: AssemblyFileVersion ("1.1.0.0")]
