using System.Reflection;

// Refactor R3: what observes (the former Actuators, its observing half).
[assembly: KSPAssembly ("KRPC.Bridge.Telemetry", 1, 0)]
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Core", 1, 3)]
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Model", 1, 0)]
[assembly: KSPAssemblyDependency ("KRPC.Bridge.Control", 1, 0)]

[assembly: AssemblyTitle ("KRPC.Bridge.Telemetry")]
[assembly: AssemblyDescription ("What observes: coherent physics-rate frames of the vessels and their actuators (DynamicsV3, DynamicsExtV1, AeroActuatorV1), tracked vessels, aerodynamic wrench batch, samples. Read-only.")]
[assembly: AssemblyVersion ("2.0.0.0")]
[assembly: AssemblyFileVersion ("2.0.0.0")]
