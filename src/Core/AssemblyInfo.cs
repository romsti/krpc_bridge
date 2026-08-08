using System.Reflection;

// ---------------------------------------------------------------------------------
// KSPAssembly is what makes the whole plugin architecture hold together.
//
// KSP's AssemblyLoader reads these attributes from every DLL under GameData, builds a
// dependency graph, topologically sorts it, and loads in that order. A plugin that
// declares [assembly: KSPAssemblyDependency("KRPC.Bridge.Core", 1, 0)] is therefore
// guaranteed that Core's types are already in the AppDomain when its own static
// constructors run.
//
// If Core is missing or too old, KSP SKIPS the dependent assembly and logs it, instead
// of loading it and blowing up with a TypeLoadException on first use. That is exactly
// the graceful degradation we want, and it is free.
//
// Bump the minor number when you add members. Bump the major only on a breaking change,
// and bump the dependency declared by every plugin at the same time.
// ---------------------------------------------------------------------------------
[assembly: KSPAssembly ("KRPC.Bridge.Core", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Core")]
[assembly: AssemblyDescription ("Shared runtime for the krpc_bridge plugins: main-thread dispatch, background jobs, GameEvents bus, mod discovery.")]
[assembly: AssemblyVersion ("1.1.0.0")]
[assembly: AssemblyFileVersion ("1.1.0.0")]
