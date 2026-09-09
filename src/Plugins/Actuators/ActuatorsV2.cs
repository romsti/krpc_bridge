using System;
using System.Collections.Generic;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Actuators
{
    public static partial class ActuatorsService
    {
        /// <summary>
        /// Version du protocole snapshot/trame atomique. La version 2 utilise des
        /// tableaux plats documentes par leurs strides dans l'en-tete.
        /// </summary>
        [KRPCProperty]
        public static int ProtocolVersion {
            get { return ActuatorsAddon.ProtocolVersion; }
        }

        /// <summary>
        /// Dernier snapshot coherent des moteurs, gimbals et transforms de poussee.
        /// Les 14 premiers doubles decrivent le schema, le tick et les sections.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> ControlSnapshotV2 ()
        {
            return ActuatorsAddon.ReadControlSnapshot ();
        }

        /// <summary>
        /// Acquiert l'autorite exclusive v2 sur les moteurs et gimbals du vaisseau
        /// actif. Renvoie un token opaque a presenter pour toute commande suivante.
        /// </summary>
        [KRPCProcedure]
        public static string AcquireControl (string owner, float leaseSeconds = 1f)
        {
            return ActuatorsAddon.AcquireControl (owner, leaseSeconds);
        }

        /// <summary>Renouvelle le bail exclusif du controleur, entre 0.1 et 5 s.</summary>
        [KRPCProcedure]
        public static bool RenewControl (string token, float leaseSeconds = 1f)
        {
            return ActuatorsAddon.RenewControl (token, leaseSeconds);
        }

        /// <summary>
        /// Libere l'autorite et restaure tous les champs actionneurs captures. Renvoie
        /// le nombre de baux moteur/gimbal restaures.
        /// </summary>
        [KRPCProcedure]
        public static int ReleaseControl (string token)
        {
            return ActuatorsAddon.ReleaseControl (token);
        }

        /// <summary>
        /// Programme une trame moteur/gimbal appliquee ensemble dans FixedUpdate.
        /// engineCommands contient des lignes flight_id, ordinal, throttle[0..100].
        /// gimbalCommands contient flight_id, ordinal, x_deg, y_deg. applyTick=0 vise
        /// le prochain tick ; validUntilTick=0 utilise le meme tick.
        /// </summary>
        [KRPCProcedure]
        public static int ApplyControlFrame (
            string token, int sequence, int applyTick, int validUntilTick,
            float leaseSeconds, IList<double> engineCommands,
            IList<double> gimbalCommands)
        {
            var engines = ParseEngineCommands (engineCommands);
            var gimbals = ParseGimbalCommands (gimbalCommands);
            return ActuatorsAddon.QueueControlFrame (
                token, sequence, applyTick, validUntilTick, leaseSeconds,
                engines, gimbals);
        }

        /// <summary>
        /// Etat compact : protocole, tick, generation, owner actif, TTL, sequence/tick
        /// pending, derniere sequence acceptee/appliquee, resultat et tick applique.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> ControlStatusV2 ()
        {
            return ActuatorsAddon.ControlStatus ();
        }

        static List<ActuatorsAddon.EngineFrameCommand> ParseEngineCommands (
            IList<double> values)
        {
            if (values == null)
                throw new ArgumentNullException (nameof (values));
            if (values.Count % 3 != 0)
                throw new ArgumentException (
                    "engineCommands doit contenir des lignes de 3 doubles",
                    nameof (values));
            var output = new List<ActuatorsAddon.EngineFrameCommand> (values.Count / 3);
            var seen = new HashSet<string> (StringComparer.Ordinal);
            for (int i = 0; i < values.Count; i += 3) {
                string flightId = ExactUIntString (values [i], "engine flight_id");
                int ordinal = ExactNonNegativeInt (values [i + 1], "engine ordinal");
                string key = flightId + ":" + ordinal;
                if (!seen.Add (key))
                    throw new ArgumentException (
                        "moteur duplique dans la trame: " + key, nameof (values));
                var engine = FindEngine (flightId, ordinal);
                ActuatorsAddon.ValidateEngineFrameCommand (engine, values [i + 2]);
                output.Add (new ActuatorsAddon.EngineFrameCommand {
                    Engine = engine,
                    Percentage = (float)values [i + 2]
                });
            }
            return output;
        }

        static List<ActuatorsAddon.GimbalFrameCommand> ParseGimbalCommands (
            IList<double> values)
        {
            if (values == null)
                throw new ArgumentNullException (nameof (values));
            if (values.Count % 4 != 0)
                throw new ArgumentException (
                    "gimbalCommands doit contenir des lignes de 4 doubles",
                    nameof (values));
            var output = new List<ActuatorsAddon.GimbalFrameCommand> (values.Count / 4);
            var seen = new HashSet<string> (StringComparer.Ordinal);
            for (int i = 0; i < values.Count; i += 4) {
                string flightId = ExactUIntString (values [i], "gimbal flight_id");
                int ordinal = ExactNonNegativeInt (values [i + 1], "gimbal ordinal");
                string key = flightId + ":" + ordinal;
                if (!seen.Add (key))
                    throw new ArgumentException (
                        "gimbal duplique dans la trame: " + key, nameof (values));
                var gimbal = FindGimbal (flightId, ordinal);
                ActuatorsAddon.ValidateGimbalFrameCommand (
                    gimbal, values [i + 2], values [i + 3]);
                output.Add (new ActuatorsAddon.GimbalFrameCommand {
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
