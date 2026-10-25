using System.Reflection;

// Refactor R3: what flies (the former Actuators, its commanding half).
[assembly: KSPAssembly ("KRPC.Bridge.Control", 1, 0)]
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Core", 1, 3)]
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Model", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Control")]
[assembly: AssemblyDescription ("What flies: actuator leases, the atomic protocol v2, AttitudeV1 and its lateral force, the physics-step pump.")]
[assembly: AssemblyVersion ("2.1.0.0")]
[assembly: AssemblyFileVersion ("2.1.0.0")]
