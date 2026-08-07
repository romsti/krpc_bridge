using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Service;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Template
{
    /// <summary>
    /// conn.template - the four shapes a bridge service ever needs.
    ///
    /// BEFORE YOU BUILD: a malformed signature anywhere in ANY loaded assembly does not
    /// disable that one service, it stops the entire kRPC server from starting, with a
    /// "kRPC Service Error" popup and nothing working at all. With one DLL that was a
    /// nuisance. With ten plugins it is ten times the exposure, and it is why
    /// build\verify exists and why you deploy a new plugin on its own before adding it
    /// to a working set.
    ///
    /// The rules a signature must obey:
    ///   - the class is static and carries [KRPCService],
    ///   - identifiers are letters and digits only and start upper case,
    ///   - parameter and return types are primitives (bool, int, long, uint, ulong,
    ///     float, double, string, byte[]), a [KRPCClass] or [KRPCEnum] type, or
    ///     IList/IDictionary/HashSet of those,
    ///   - default values are compile-time constants, or supplied via [KRPCDefaultValue].
    ///
    /// Vector3, Quaternion, Guid, DateTime, object, nullable value types and tuples are
    /// all invalid. Return a flat IList of doubles instead: it is also faster.
    /// </summary>
    [KRPCService (Name = "Template", GameScene = GameScene.Flight)]
    public static class TemplateService
    {
        /// <summary>Whether the wrapped mod is installed and resolved. Check this before anything else.</summary>
        [KRPCProperty]
        public static bool Available {
            get { return TemplateAddon.Available; }
        }

        /// <summary>Health check.</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        // ------------------------------------------------------------------
        // Shape 1: a plain read.
        //
        // Nothing to dispatch. kRPC executes RPC bodies on the Unity main thread inside
        // FixedUpdate, so touching Vessel, Part or any GameObject here is already safe.
        // Wrapping this in MainThread.Invoke would add a queue hop and buy nothing.
        // ------------------------------------------------------------------

        /// <summary>Universal time, as a trivial example of a direct main-thread read.</summary>
        [KRPCProperty]
        public static double Now {
            get {
                Require ();
                return Planetarium.GetUniversalTime ();
            }
        }

        // ------------------------------------------------------------------
        // Shape 2: bulk data in one call.
        //
        // The rule that shapes every plugin: one RPC costs roughly a millisecond of round
        // trip, and kRPC bounds how many it runs per FixedUpdate. Anything you would loop
        // over in Python must be looped over HERE and returned flat.
        //
        // Reading one number per part for a 200-part vessel is 200 RPCs from Python and
        // one from here. At a 25 Hz control rate the first is impossible and the second
        // is free.
        // ------------------------------------------------------------------

        /// <summary>
        /// Per-part sample of the active vessel, flattened: for each part, four doubles
        /// (index, temperature, max_temperature, skin_temperature). Length is 4 * n.
        ///
        /// Flat arrays of doubles rather than a list of objects, on purpose: no handles
        /// to release, no per-field round trip, and numpy reshapes it in one line.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> PartThermalSample ()
        {
            Require ();
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                return new double[0];

            var parts = vessel.parts;
            var output = new double[parts.Count * 4];
            for (int i = 0; i < parts.Count; i++) {
                var part = parts [i];
                output [i * 4 + 0] = i;
                output [i * 4 + 1] = part.temperature;
                output [i * 4 + 2] = part.maxTemp;
                output [i * 4 + 3] = part.skinTemperature;
            }
            return output;
        }

        // ------------------------------------------------------------------
        // Shape 3: waiting on game state.
        //
        // Never loop, never sleep. Both hang Unity, not the client, because this code IS
        // the game loop. Wait.Until throws a kRPC continuation: the server re-invokes the
        // call on a later tick and the game runs normally in between.
        // ------------------------------------------------------------------

        /// <summary>
        /// Block until the active vessel has been landed for a few consecutive ticks,
        /// then return the universal time at which it settled.
        ///
        /// Settled rather than Until because "landed" flickers during a bounce, and a
        /// single true reading is not touchdown.
        /// </summary>
        [KRPCProcedure]
        public static double AwaitLanded (float timeoutSeconds = 120f)
        {
            Require ();
            Wait.Settled (() => {
                var vessel = FlightGlobals.ActiveVessel;
                return vessel != null &&
                       (vessel.situation == Vessel.Situations.LANDED ||
                        vessel.situation == Vessel.Situations.SPLASHED);
            }, 10, timeoutSeconds, "the active vessel to settle on the ground");
            return Planetarium.GetUniversalTime ();
        }

        // ------------------------------------------------------------------
        // Shape 4: expensive computation.
        //
        // This body runs inside FixedUpdate. Spending 300 ms here drops the frame to
        // 3 FPS and no server setting can subdivide a single call. So: snapshot on the
        // main thread, compute on a worker, return an id immediately.
        //
        // The worker must not touch Unity or KSP. Off the main thread most of Unity's
        // API does not throw a managed exception, it takes the process down with no
        // stack trace in KSP.log. Snapshot into plain doubles first, always.
        // ------------------------------------------------------------------

        /// <summary>
        /// Start a long computation and return its job id at once.
        /// Poll with conn.bridge.get_job_state(id), collect with get_job_result(id), or
        /// call conn.bridge.await_job(id) to block without costing framerate.
        /// </summary>
        [KRPCProcedure]
        public static long StartExample (int steps)
        {
            Require ();

            // Snapshot: main thread, plain data only. Nothing below this line may hold a
            // reference to a Vessel, Part, Orbit or GameObject.
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                throw new InvalidOperationException ("no active vessel");
            double mass = vessel.totalMass;
            double ut = Planetarium.GetUniversalTime ();
            int count = Math.Max (1, Math.Min (steps, 100000));

            return Jobs.Start ("template.example", (progress, cancelled) => {
                var output = new double [count];
                for (int i = 0; i < count; i++) {
                    if (cancelled ())
                        break;
                    output [i] = ut + i * mass * 1e-6;
                    if ((i & 0x3FF) == 0)
                        progress ((double)i / count);
                }
                return output;
            });
        }

        static void Require ()
        {
            if (!TemplateAddon.Available)
                throw new InvalidOperationException (
                    "the Template plugin's target mod is not installed. " +
                    "Check conn.bridge.plugins before calling this.");
        }
    }
}
