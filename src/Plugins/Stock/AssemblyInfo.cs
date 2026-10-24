using System.Reflection;

// This plugin's own identity. No KSPAssemblyDependency on KRPC.Bridge.Core: it must load next to
// any Core version (see the csproj).
[assembly: KSPAssembly ("KRPC.Bridge.Stock", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Stock")]
[assembly: AssemblyDescription ("State of KSP's stock delta-v simulation for a vessel, and a way to restart it.")]
[assembly: AssemblyVersion ("2.0.0.0")]
[assembly: AssemblyFileVersion ("2.0.0.0")]
