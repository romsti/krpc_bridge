using System.Runtime.CompilerServices;

// Refactor R3: what observes (Telemetry) reads the lease state, the frame commands and the
// vessel lookups of what flies, and registers itself as Actuation.Observer. Nothing in Control
// refers to Telemetry. Kept apart from AssemblyInfo.cs, which the replay harness does not
// compile, so the harness proves the same visibility as the game.
[assembly: InternalsVisibleTo ("KRPC.Bridge.Telemetry")]
