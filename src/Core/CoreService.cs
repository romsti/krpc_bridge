using System.Collections.Generic;
using System.Text;
using KRPC.Service;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// conn.bridge - the Core's own kRPC service.
    ///
    /// Two deliberate design rules here, both worth copying in every plugin.
    ///
    /// 1. NO KRPCClass ANYWHERE IN CORE. Every signature uses primitives, strings and
    ///    collections of them. A KRPCClass return value is an object handle: the client
    ///    gets a reference and pays one more round trip per property it then reads.
    ///    Returning 40 event records as handles and reading 6 fields each is 240 RPCs.
    ///    Returning them as 40 packed strings is 1. Core is also the assembly that must
    ///    never break the server, and the fewer exotic signatures it declares, the
    ///    smaller that risk is.
    ///
    /// 2. Records are packed as tab-separated strings, documented per method. Python
    ///    splits them, which costs nothing, and the wire format stays one flat list.
    ///
    /// GameScene.All on purpose: asking which plugins loaded, from the space centre and
    /// before any flight, is the normal way to open a session.
    /// </summary>
    [KRPCService (Name = "Bridge", GameScene = GameScene.All)]
    public static class CoreService
    {
        /// <summary>Health check. Returns "pong" once the Core is up and its pump is running, "loading" before that.</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return BridgeCore.Ready ? "pong" : "loading";
        }

        // Cached because a client can stream any property, and a streamed property runs
        // every physics tick forever. Neither of these can change during a session.
        static string cachedVersion;
        static IList<string> cachedPlugins;
        static IList<string> cachedAvailable;

        /// <summary>Version of the Core assembly.</summary>
        [KRPCProperty]
        public static string Version {
            get {
                return cachedVersion ??
                       (cachedVersion = typeof (CoreService).Assembly.GetName ().Version.ToString ());
            }
        }

        /// <summary>
        /// Physics ticks since the Core started. Increments while the game simulates and
        /// freezes when it does not, so a stalled counter distinguishes "the game is
        /// paused or loading" from "my script is slow".
        /// </summary>
        [KRPCProperty]
        public static long Ticks {
            get { return BridgeCore.Ticks; }
        }

        /// <summary>
        /// Actions waiting in the main-thread queue. Should sit at 0. A number that keeps
        /// rising means a plugin is posting faster than one tick can drain.
        /// </summary>
        [KRPCProperty]
        public static int PendingMainThreadWork {
            get { return MainThread.PendingCount; }
        }

        // ------------------------------------------------------------------
        // What is loaded
        // ------------------------------------------------------------------

        /// <summary>
        /// Every registered plugin, as "name\tavailable\tplugin_version\tmod_version\treport".
        /// The availability field is "1" or "0".
        ///
        /// Call this first in any script. It is the difference between a clear
        /// "Trajectories is not installed" and a NullReferenceException twenty minutes
        /// into a flight.
        /// </summary>
        [KRPCProperty]
        public static IList<string> Plugins {
            get {
                if (cachedPlugins != null)
                    return cachedPlugins;
                var all = ModRegistry.All ();
                var rows = new List<string> (all.Count);
                foreach (var status in all) {
                    var row = new StringBuilder ();
                    row.Append (status.Name).Append ('\t')
                       .Append (status.Available ? "1" : "0").Append ('\t')
                       .Append (status.Version).Append ('\t')
                       .Append (status.ModVersion).Append ('\t')
                       .Append (Flatten (status.Report));
                    rows.Add (row.ToString ());
                }
                // Only cache once resolution has actually happened. Caching an empty list
                // read before Core's Start would freeze "nothing is installed" for the
                // whole session.
                if (rows.Count > 0)
                    cachedPlugins = rows;
                return rows;
            }
        }

        /// <summary>Names of the plugins whose target mod resolved. The short form of <see cref="Plugins"/>.</summary>
        [KRPCProperty]
        public static IList<string> AvailablePlugins {
            get {
                if (cachedAvailable != null)
                    return cachedAvailable;
                var all = ModRegistry.All ();
                var found = new List<string> ();
                foreach (var status in all)
                    if (status.Available)
                        found.Add (status.Name);
                if (all.Count > 0)
                    cachedAvailable = found;
                return found;
            }
        }

        /// <summary>True when the named plugin is registered and its mod resolved.</summary>
        [KRPCProcedure]
        public static bool HasPlugin (string name)
        {
            foreach (var status in ModRegistry.All ())
                if (status.Name == name)
                    return status.Available;
            return false;
        }

        // ------------------------------------------------------------------
        // Event log
        // ------------------------------------------------------------------

        /// <summary>
        /// Game events recorded since <paramref name="sinceId"/>, oldest first, as
        /// "id\tkind\tut\trealtime\tvessel_id\tdetail".
        ///
        /// Pass 0 on the first call, then the id from the last row you received.
        ///
        /// This is how you observe an instant from Python. A kRPC stream samples a value
        /// once per update: it reports what something IS, never that something HAPPENED,
        /// so a part destroyed between two samples leaves no trace at all. The bus
        /// subscribes on the C# side and buffers, so nothing is lost between polls.
        ///
        /// Pair it with <see cref="OnEvent"/> rather than sleeping in a loop.
        /// </summary>
        /// <param name="sinceId">Highest id already seen. 0 on the first call.</param>
        [KRPCProcedure]
        public static IList<string> PollEvents (long sinceId)
        {
            var rows = new List<string> ();
            foreach (var record in EventBus.Poll (sinceId)) {
                var row = new StringBuilder ();
                row.Append (record.Id).Append ('\t')
                   .Append (record.Kind).Append ('\t')
                   .Append (record.UT.ToString ("F3")).Append ('\t')
                   .Append (record.Realtime.ToString ("F3")).Append ('\t')
                   .Append (record.VesselId).Append ('\t')
                   .Append (Flatten (record.Detail));
                rows.Add (row.ToString ());
            }
            return rows;
        }

        /// <summary>
        /// Total events recorded this session, including any already aged out of the ring
        /// buffer. Compare it against the last id you polled: if it has run ahead by more
        /// than the buffer holds, you polled too slowly and lost events.
        /// </summary>
        [KRPCProperty]
        public static long EventsRecorded {
            get { return EventBus.TotalRecorded; }
        }

        /// <summary>Publish a marker into the event log from Python. Useful to align a flight log against a script's own phases.</summary>
        /// <param name="kind">Event name. Prefix your own with something unrecognisable as stock, e.g. "script.".</param>
        /// <param name="detail">Free-form payload.</param>
        [KRPCProcedure]
        public static void Mark (string kind, string detail)
        {
            EventBus.Record (kind, string.Empty, detail);
        }

        /// <summary>
        /// An event that fires the moment a matching game event is recorded, so a script
        /// can block instead of poll.
        ///
        /// In Python: evt = conn.bridge.on_event("part.die,vessel.destroy"), then
        /// "with evt.condition: evt.wait()", then read poll_events for what happened.
        ///
        /// The event carries no payload - it is a wake-up. Read <see cref="PollEvents"/>
        /// afterwards for the records themselves, which is also what makes two events
        /// inside one tick safe: the signal may coalesce them, the log never does.
        /// </summary>
        /// <param name="kinds">
        /// Comma-separated substrings matched case-insensitively against the event kind.
        /// Empty or "*" fires on every event.
        /// </param>
        [KRPCProcedure]
        public static KRPC.Service.Messages.Event OnEvent (string kinds = "")
        {
            return EventBus.Signal (kinds).Message;
        }

        // ------------------------------------------------------------------
        // HUD
        // ------------------------------------------------------------------

        /// <summary>
        /// Hide the game's HUD now, as if F2 had been pressed. Returns false if it was
        /// already hidden.
        ///
        /// This fires GameEvents.onHideUI rather than blanking the stock canvases
        /// directly, which is the difference that makes MOD windows disappear too -
        /// FMRS's window, OCISLY's, MechJeb's, the toolbar. For a recorded flight that is
        /// the whole point.
        /// </summary>
        [KRPCProcedure (GameScene = GameScene.Flight)]
        public static bool HideUi ()
        {
            return Hud.Hide ();
        }

        /// <summary>Bring the HUD back now. Returns false if it was already visible.</summary>
        [KRPCProcedure (GameScene = GameScene.Flight)]
        public static bool ShowUi ()
        {
            return Hud.Show ();
        }

        /// <summary>Whether the HUD is currently showing.</summary>
        [KRPCProperty]
        public static bool UiVisible {
            get { return Hud.Visible; }
        }

        // ------------------------------------------------------------------
        // Reflection probe
        // ------------------------------------------------------------------

        /// <summary>
        /// Public and non-public members of a type in any loaded assembly, as
        /// "kind\tname\tsignature".
        ///
        /// The tool for writing a new plugin, and for diagnosing an old one after a mod
        /// update. Every reflective lookup in this repo was found by asking a live game
        /// what a type really has, rather than trusting a name from a changelog - and
        /// when a mod renames a member, this says so in ten seconds without a rebuild.
        ///
        /// For example: conn.bridge.describe_type("FMRSContinued", "FMRS.FMRS_Core").
        ///
        /// DO NOT PUT A STREAM ON THIS. It walks every loaded assembly and reflects over
        /// a whole type; a kRPC stream would re-run that every physics tick, forever.
        /// </summary>
        /// <param name="assemblyFragment">Substring of the assembly's simple name, case-insensitive.</param>
        /// <param name="typeName">Full or simple type name.</param>
        [KRPCProcedure]
        public static IList<string> DescribeType (string assemblyFragment, string typeName)
        {
            var rows = new List<string> ();
            var candidates = ModRegistry.FindAssembliesContaining (assemblyFragment ?? string.Empty);
            if (candidates.Count == 0) {
                rows.Add ("error\tno loaded assembly matches\t" + assemblyFragment);
                return rows;
            }

            System.Type found = null;
            foreach (var candidate in candidates) {
                found = ModRegistry.FindType (candidate, typeName);
                if (found != null) {
                    rows.Add ("assembly\t" + candidate.GetName ().Name + "\t"
                              + candidate.GetName ().Version);
                    break;
                }
            }
            if (found == null) {
                rows.Add ("error\ttype not found in\t" + ModRegistry.NamesOf (candidates));
                return rows;
            }

            const System.Reflection.BindingFlags all =
                System.Reflection.BindingFlags.Public | System.Reflection.BindingFlags.NonPublic |
                System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.Static |
                System.Reflection.BindingFlags.FlattenHierarchy;

            foreach (var field in found.GetFields (all))
                rows.Add ("field\t" + field.Name + "\t"
                          + (field.IsStatic ? "static " : "") + (field.IsPublic ? "public " : "private ")
                          + field.FieldType.Name);

            foreach (var property in found.GetProperties (all))
                rows.Add ("property\t" + property.Name + "\t" + property.PropertyType.Name
                          + (property.CanRead ? " get" : "") + (property.CanWrite ? " set" : ""));

            foreach (var method in found.GetMethods (all)) {
                if (method.IsSpecialName)
                    continue;   // property accessors, already listed above
                var args = new StringBuilder ();
                foreach (var parameter in method.GetParameters ()) {
                    if (args.Length > 0)
                        args.Append (", ");
                    args.Append (parameter.ParameterType.Name).Append (' ').Append (parameter.Name);
                }
                rows.Add ("method\t" + method.Name + "\t"
                          + (method.IsStatic ? "static " : "") + (method.IsPublic ? "public " : "private ")
                          + method.ReturnType.Name + " (" + args + ")");
            }

            rows.Sort ();
            return rows;
        }

        // ------------------------------------------------------------------
        // Background jobs
        // ------------------------------------------------------------------

        /// <summary>
        /// State of a background job started by a plugin: "pending", "running", "done",
        /// "failed" or "cancelled".
        /// </summary>
        [KRPCProcedure]
        public static string GetJobState (long id)
        {
            return Jobs.State (id).ToString ().ToLowerInvariant ();
        }

        /// <summary>Progress of a background job, 0 to 1, as last reported by the worker.</summary>
        [KRPCProcedure]
        public static double GetJobProgress (long id)
        {
            return Jobs.Progress (id);
        }

        /// <summary>
        /// Result of a finished job, as a flat array. Throws while it is still running,
        /// so a caller that forgot to poll gets a clear error instead of an empty list.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GetJobResult (long id)
        {
            return Jobs.Result (id);
        }

        /// <summary>Error message of a failed job, or the empty string.</summary>
        [KRPCProcedure]
        public static string GetJobError (long id)
        {
            return Jobs.Error (id);
        }

        /// <summary>
        /// Block until the job finishes, then return its result.
        ///
        /// Uses a kRPC continuation, so the game keeps running at full framerate while
        /// the call is outstanding. This is what "blocking" should mean in a service
        /// method: never a loop, never a sleep.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AwaitJob (long id, float timeoutSeconds = 60f)
        {
            Wait.Until (() => {
                var state = Jobs.State (id);
                return state != Core.JobState.Running && state != Core.JobState.Pending;
            }, timeoutSeconds, "job " + id.ToString ());
            return Jobs.Result (id);
        }

        /// <summary>Ask a job to stop. Cooperative: the worker decides when to notice.</summary>
        [KRPCProcedure]
        public static void CancelJob (long id)
        {
            Jobs.Cancel (id);
        }

        /// <summary>Tabs and newlines would corrupt a packed row, so they are neutralised.</summary>
        static string Flatten (string text)
        {
            if (string.IsNullOrEmpty (text))
                return string.Empty;
            return text.Replace ('\t', ' ').Replace ('\n', ' ').Replace ('\r', ' ');
        }
    }
}
