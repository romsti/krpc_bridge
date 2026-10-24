using System.Reflection;

// Refactor R3: the pure model of what flies. No dependency: KSP may load it first, and Control
// and Telemetry declare a dependency on it (KSPAssemblyDependency), so it is in the AppDomain
// before their types are scanned.
[assembly: KSPAssembly ("KRPC.Bridge.Model", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Model")]
[assembly: AssemblyDescription ("The pure model of the bridge's flight loop: AttitudeV1 law and allocation, write rules, lateral force, gimbal pivots, effector registry. System only.")]
[assembly: AssemblyVersion ("2.0.0.0")]
[assembly: AssemblyFileVersion ("2.0.0.0")]
