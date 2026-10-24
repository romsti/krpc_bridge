using System;
using System.Collections.Generic;
using KRPC.Service.Attributes;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    public static partial class ControlService
    {
        /// <summary>
        /// Version of the atomic snapshot/frame protocol. Version 2 uses flat arrays described by
        /// their strides in the header.
        /// </summary>
        [KRPCProperty]
        public static int ProtocolVersion {
            get { return Actuation.ProtocolVersion; }
        }

        /// <summary>
        /// A coherent snapshot of the engines, gimbals and thrust transforms of the active vessel.
        /// The first 14 doubles describe the schema, the tick and the sections.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> ControlSnapshotV2 ()
        {
            return Actuation.ReadControlSnapshot ();
        }

        /// <summary>
        /// Acquires the exclusive v2 authority over the engines and gimbals of the active vessel.
        /// Returns an opaque token to present with every following command.
        /// </summary>
        [KRPCProcedure]
        public static string AcquireControl (string owner, float leaseSeconds = 1f)
        {
            return Actuation.AcquireControl (owner, leaseSeconds);
        }

        /// <summary>
        /// AcquireControl for the vessel the client names (refactor R3, state per vessel): refused
        /// with [NOT_LOADED] unless persistentId (decimal string) is the loaded active vessel, the
        /// only one the v2 protocol drives. Returns the token.
        /// </summary>
        [KRPCProcedure]
        public static string AcquireControlForV1 (string owner, float leaseSeconds, string persistentId)
        {
            return Actuation.AcquireControlFor (owner, leaseSeconds, persistentId);
        }

        /// <summary>Renews the controller's exclusive lease, between 0.1 and 5 s.</summary>
        [KRPCProcedure]
        public static bool RenewControl (string token, float leaseSeconds = 1f)
        {
            return Actuation.RenewControl (token, leaseSeconds);
        }

        /// <summary>
        /// Releases the authority and restores every captured actuator field. Returns the count
        /// of engine/gimbal leases restored.
        /// </summary>
        [KRPCProcedure]
        public static int ReleaseControl (string token)
        {
            return Actuation.ReleaseControl (token);
        }

        /// <summary>
        /// Schedules an engine/gimbal frame applied as a whole in one FixedUpdate. engineCommands
        /// holds rows flight_id, ordinal, throttle[0..100]; gimbalCommands rows flight_id, ordinal,
        /// x_deg, y_deg. applyTick=0 aims at the next tick; validUntilTick=0 uses the same tick.
        /// </summary>
        [KRPCProcedure]
        public static int ApplyControlFrame (
            string token, int sequence, int applyTick, int validUntilTick,
            float leaseSeconds, IList<double> engineCommands,
            IList<double> gimbalCommands)
        {
            var engines = ParseEngineCommands (engineCommands);
            var gimbals = ParseGimbalCommands (gimbalCommands);
            return Actuation.QueueControlFrame (
                token, sequence, applyTick, validUntilTick, leaseSeconds,
                engines, gimbals);
        }

        /// <summary>
        /// Compact state: protocol, tick, generation, owner active, TTL, pending sequence/tick,
        /// last accepted/applied sequence, result, applied tick and (schema 2, refactor R3) the
        /// persistentId of the vessel the authority drives, NaN without one.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> ControlStatusV2 ()
        {
            return Actuation.ControlStatus ();
        }

        static List<Actuation.EngineFrameCommand> ParseEngineCommands (
            IList<double> values)
        {
            if (values == null)
                throw new ArgumentNullException (nameof (values));
            if (values.Count % 3 != 0)
                throw new ArgumentException (
                    "engineCommands doit contenir des lignes de 3 doubles",
                    nameof (values));
            var output = new List<Actuation.EngineFrameCommand> (values.Count / 3);
            var seen = new HashSet<string> (StringComparer.Ordinal);
            for (int i = 0; i < values.Count; i += 3) {
                string flightId = ExactUIntString (values [i], "engine flight_id");
                int ordinal = ExactNonNegativeInt (values [i + 1], "engine ordinal");
                string key = flightId + ":" + ordinal;
                if (!seen.Add (key))
                    throw new ArgumentException (
                        "moteur duplique dans la trame: " + key, nameof (values));
                var engine = VesselLookup.FindEngine (flightId, ordinal);
                Actuation.ValidateEngineFrameCommand (engine, values [i + 2]);
                output.Add (new Actuation.EngineFrameCommand {
                    Engine = engine,
                    Percentage = (float)values [i + 2]
                });
            }
            return output;
        }

        static List<Actuation.GimbalFrameCommand> ParseGimbalCommands (
            IList<double> values)
        {
            if (values == null)
                throw new ArgumentNullException (nameof (values));
            if (values.Count % 4 != 0)
                throw new ArgumentException (
                    "gimbalCommands doit contenir des lignes de 4 doubles",
                    nameof (values));
            var output = new List<Actuation.GimbalFrameCommand> (values.Count / 4);
            var seen = new HashSet<string> (StringComparer.Ordinal);
            for (int i = 0; i < values.Count; i += 4) {
                string flightId = ExactUIntString (values [i], "gimbal flight_id");
                int ordinal = ExactNonNegativeInt (values [i + 1], "gimbal ordinal");
                string key = flightId + ":" + ordinal;
                if (!seen.Add (key))
                    throw new ArgumentException (
                        "gimbal duplique dans la trame: " + key, nameof (values));
                var gimbal = VesselLookup.FindGimbal (flightId, ordinal);
                Actuation.ValidateGimbalFrameCommand (
                    gimbal, values [i + 2], values [i + 3]);
                output.Add (new Actuation.GimbalFrameCommand {
                    Gimbal = gimbal,
                    XDegrees = (float)values [i + 2],
                    YDegrees = (float)values [i + 3]
                });
            }
            return output;
        }

        static string ExactUIntString (double value, string label)
        {
            if (double.IsNaN (value) || double.IsInfinity (value) ||
                    value < 0.0 || value > UInt32.MaxValue || value != Math.Floor (value))
                throw new ArgumentOutOfRangeException (label, "uint exact attendu");
            return ((uint)value).ToString (
                System.Globalization.CultureInfo.InvariantCulture);
        }

        static int ExactNonNegativeInt (double value, string label)
        {
            if (double.IsNaN (value) || double.IsInfinity (value) ||
                    value < 0.0 || value > Int32.MaxValue || value != Math.Floor (value))
                throw new ArgumentOutOfRangeException (
                    label, "entier positif exact attendu");
            return (int)value;
        }
    }
}
