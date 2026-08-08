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
//     log, rather than loading it and throwing TypeLoadException on first use. That is
//     the graceful degradation, and it is free.
// ---------------------------------------------------------------------------------
[assembly: KSPAssembly ("KRPC.Bridge.Fmrs", 1, 0)]
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Core", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Fmrs")]
[assembly: AssemblyDescription ("kRPC service for FMRS Continued: fly dropped stages, replay landings, read the recovery ledger.")]
[assembly: AssemblyVersion ("1.1.0.0")]
[assembly: AssemblyFileVersion ("1.1.0.0")]
