using System;
using System.Collections.Generic;
using System.Globalization;
using KRPC.Service;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// conn.actuators - autorite par moteur, sans boucle ni attente dans un RPC.
    ///
    /// EngineSample renvoie des lignes de neuf doubles : flight_id, ordinal du module,
    /// allume, manette realisee, poussee realisee, poussee maximale, limite de poussee,
    /// mode independant, pourcentage independant.
    ///
    /// GimbalSample renvoie des lignes de huit doubles : flight_id, ordinal du module,
    /// verrou, limiteur, plage, puis actuation locale X/Y/Z en degres. Cette actuation
    /// est l'etat calcule par ModuleGimbal au tick physique ; elle est mesuree, jamais
    /// reecrite ici.
    ///
    /// La limite de poussee stock kRPC est un plafond. LeaseIndependentThrottle expose
    /// le second canal stock de ModuleEngines : une consigne absolue par moteur qui ne
    /// suit plus la manette principale. Le bail est borne a une seconde et restaure les
    /// valeurs anterieures si le client ne le renouvelle pas.
    /// </summary>
    [KRPCService (Name = "Actuators", GameScene = GameScene.Flight)]
    public static class ActuatorsService
    {
        [KRPCProperty]
        public static bool Available { get { return true; } }

        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        [KRPCProcedure]
        public static IList<double> EngineSample ()
        {
            var output = new List<double> ();
            var vessel = RequireActiveVessel ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine == null)
                        continue;
                    output.Add (part.flightID);
                    output.Add (ordinal++);
                    output.Add (engine.EngineIgnited ? 1.0 : 0.0);
                    output.Add (engine.currentThrottle);
                    output.Add (engine.finalThrust);
                    output.Add (engine.maxThrust);
                    output.Add (engine.thrustPercentage);
                    output.Add (engine.independentThrottle ? 1.0 : 0.0);
                    output.Add (engine.independentThrottlePercentage);
                }
            }
            return output;
        }

        [KRPCProcedure]
        public static IList<double> GimbalSample ()
        {
            var output = new List<double> ();
            var vessel = RequireActiveVessel ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var gimbal = part.Modules [m] as ModuleGimbal;
                    if (gimbal == null)
                        continue;
                    output.Add (part.flightID);
                    output.Add (ordinal++);
                    output.Add (gimbal.gimbalLock ? 1.0 : 0.0);
                    output.Add (gimbal.gimbalLimiter);
                    output.Add (gimbal.gimbalRange);
                    output.Add (gimbal.actuationLocal.x);
                    output.Add (gimbal.actuationLocal.y);
                    output.Add (gimbal.actuationLocal.z);
                }
            }
            return output;
        }

        [KRPCProcedure]
        public static bool LeaseIndependentThrottle (
            string flightId, int engineOrdinal, float percentage, float leaseSeconds = 0.25f)
        {
            var engine = FindEngine (flightId, engineOrdinal);
            ActuatorsAddon.Command (engine, percentage, leaseSeconds);
            return true;
        }

        [KRPCProcedure]
        public static bool ReleaseIndependentThrottle (string flightId, int engineOrdinal)
        {
            return ActuatorsAddon.Release (FindEngine (flightId, engineOrdinal));
        }

        [KRPCProcedure]
        public static int ReleaseAll ()
        {
            return ActuatorsAddon.RestoreAll ();
        }

        static Vessel RequireActiveVessel ()
        {
            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == null || !vessel.loaded)
                throw new InvalidOperationException ("aucun vaisseau actif et charge");
            return vessel;
        }

        static ModuleEngines FindEngine (string flightId, int wantedOrdinal)
        {
            uint id;
            if (!uint.TryParse (flightId, NumberStyles.None, CultureInfo.InvariantCulture, out id))
                throw new ArgumentException ("flightId doit etre un uint decimal", nameof (flightId));
            if (wantedOrdinal < 0)
                throw new ArgumentOutOfRangeException (nameof (wantedOrdinal));

            var vessel = RequireActiveVessel ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                if (part.flightID != id)
                    continue;
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine == null)
                        continue;
                    if (ordinal++ == wantedOrdinal)
                        return engine;
                }
                break;
            }
            throw new InvalidOperationException (
                "moteur absent du vaisseau actif : part " + flightId +
                ", ordinal " + wantedOrdinal.ToString (CultureInfo.InvariantCulture));
        }
    }
}
