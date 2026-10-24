using System.Runtime.CompilerServices;

// Refactor R3: the pure model (law, allocation, write rules, lateral force, pivots, effector
// registry) is internal to the assemblies that fly and observe with it. Kept apart from
// AssemblyInfo.cs, which the replay harness does not compile, so the harness proves the same
// visibility as the game.
[assembly: InternalsVisibleTo ("KRPC.Bridge.Control")]
[assembly: InternalsVisibleTo ("KRPC.Bridge.Telemetry")]
