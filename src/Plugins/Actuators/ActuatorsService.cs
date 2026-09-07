using System;
using System.Collections.Generic;
using System.Globalization;
using KRPC.Service;
using KRPC.Service.Attributes;
using UnityEngine;

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
    /// verrou, limiteur, plage, puis actuation locale X/Y/Z en degres. Hors bail, cette
    /// actuation est l'etat calcule par ModuleGimbal. Pendant un bail, elle est la
    /// consigne appliquee directement aux transforms de tuyere.
    ///
    /// La limite de poussee stock kRPC est un plafond. LeaseIndependentThrottle expose
    /// le second canal stock de ModuleEngines : une consigne absolue par moteur qui ne
    /// suit plus la manette principale. Le bail est borne a une seconde et restaure les
    /// valeurs anterieures si le client ne le renouvelle pas.
    ///
    /// LeaseGimbal desactive temporairement le calcul stock de ModuleGimbal et applique
    /// une deflexion absolue, bornee par les plages et le limiteur de ce module. La
    /// vitesse de reponse configuree reste appliquee. Le bail restaure toujours l'etat,
    /// l'actuation et les rotations d'origine.
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

        /// <summary>
        /// Direction de poussée de chaque ModuleEngines dans le repère du véhicule.
        ///
        /// Lignes de cinq doubles : flight_id, ordinal moteur, puis direction X/Y/Z.
        /// Lit les thrustTransforms du ModuleEngines courant au lieu de conserver les
        /// wrappers Thruster de kRPC, qui peuvent devenir périmés après un revert.
        /// Les axes sont droite, avant et bas, comme Vessel.reference_frame de kRPC.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> ThrustDirectionSample ()
        {
            var output = new List<double> ();
            var vessel = RequireActiveVessel ();
            var reference = vessel.ReferenceTransform;
            if (reference == null)
                throw new InvalidOperationException ("repère du vaisseau absent");
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var engine = part.Modules [m] as ModuleEngines;
                    if (engine == null)
                        continue;
                    Vector3 world = Vector3.zero;
                    float poids = 0f;
                    if (engine.thrustTransforms == null ||
                            engine.thrustTransforms.Count == 0)
                        throw new InvalidOperationException (
                            "thrustTransforms absents : part " + part.flightID);
                    for (int i = 0; i < engine.thrustTransforms.Count; i++) {
                        var transform = engine.thrustTransforms [i];
                        if (transform == null)
                            throw new InvalidOperationException (
                                "thrustTransform nul : part " + part.flightID);
                        float multiplicateur = 1f;
                        if (engine.thrustTransformMultipliers != null &&
                                i < engine.thrustTransformMultipliers.Count)
                            multiplicateur = engine.thrustTransformMultipliers [i];
                        world += multiplicateur * transform.forward;
                        poids += Math.Abs (multiplicateur);
                    }
                    if (poids <= 0f || world.sqrMagnitude <= 1e-12f)
                        throw new InvalidOperationException (
                            "direction de poussée nulle : part " + part.flightID);
                    Vector3 locale = reference.InverseTransformDirection (
                        world.normalized);
                    output.Add (part.flightID);
                    output.Add (ordinal++);
                    output.Add (locale.x);
                    output.Add (locale.y);
                    output.Add (locale.z);
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
            return ActuatorsAddon.ReleaseThrottle (FindEngine (flightId, engineOrdinal));
        }

        /// <summary>
        /// Loue la deflexion absolue d'un gimbal, en degres locaux X/Y.
        /// La consigne est bornee par les plages directionnelles et le limiteur stock ;
        /// la vitesse de reponse configuree par la piece reste appliquee.
        /// </summary>
        [KRPCProcedure]
        public static bool LeaseGimbal (
            string flightId, int gimbalOrdinal, float xDegrees, float yDegrees,
            float leaseSeconds = 0.25f)
        {
            ActuatorsAddon.CommandGimbal (
                FindGimbal (flightId, gimbalOrdinal), xDegrees, yDegrees, leaseSeconds);
            return true;
        }

        /// <summary>Libere un bail de gimbal et restaure le calcul stock.</summary>
        [KRPCProcedure]
        public static bool ReleaseGimbal (string flightId, int gimbalOrdinal)
        {
            return ActuatorsAddon.ReleaseGimbal (FindGimbal (flightId, gimbalOrdinal));
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

        static ModuleGimbal FindGimbal (string flightId, int wantedOrdinal)
        {
            uint id = ParseFlightId (flightId);
            if (wantedOrdinal < 0)
                throw new ArgumentOutOfRangeException (nameof (wantedOrdinal));

            var vessel = RequireActiveVessel ();
            for (int p = 0; p < vessel.parts.Count; p++) {
                var part = vessel.parts [p];
                if (part.flightID != id)
                    continue;
                int ordinal = 0;
                for (int m = 0; m < part.Modules.Count; m++) {
                    var gimbal = part.Modules [m] as ModuleGimbal;
                    if (gimbal == null)
                        continue;
                    if (ordinal++ == wantedOrdinal)
                        return gimbal;
                }
                break;
            }
            throw new InvalidOperationException (
                "gimbal absent du vaisseau actif : part " + flightId +
                ", ordinal " + wantedOrdinal.ToString (CultureInfo.InvariantCulture));
        }

        static uint ParseFlightId (string flightId)
        {
            uint id;
            if (!uint.TryParse (flightId, NumberStyles.None, CultureInfo.InvariantCulture, out id))
                throw new ArgumentException ("flightId doit etre un uint decimal", nameof (flightId));
            return id;
        }
    }
}
