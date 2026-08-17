using System.Reflection;

// This plugin's own identity.
[assembly: KSPAssembly ("KRPC.Bridge.Ident", 1, 0)]

// Core must load first: this assembly's addon registers with it from an Instantly-startup
// Awake. If Core is absent or older than 1.0, KSP skips THIS assembly and says so in
// KSP.log rather than loading it and throwing TypeLoadException on first touch.
//
// Check it landed:
//     findstr /C:"KRPC.Bridge" "<KSP>\KSP.log"
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Core", 1, 0)]

// No KSPAssemblyDependency on KRPC.SpaceCenter: it is not a KSPAssembly, it is a kRPC
// service assembly, and kRPC's own loader handles it. The compile-time <Reference> in the
// csproj is what ties them together, and the csproj explains why that is allowed here.

[assembly: AssemblyTitle ("KRPC.Bridge.Ident")]
[assembly: AssemblyDescription ("KSP identifiers that kRPC holds internally but does not publish: part flightID, vessel persistentId and Guid.")]
[assembly: AssemblyVersion ("1.1.0.0")]
[assembly: AssemblyFileVersion ("1.1.0.0")]
