using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Service;
using KRPC.Service.Attributes;
using UnityEngine;

namespace KRPC.Bridge.Trajectories
{
    /// <summary>
    /// conn.trajectories - the Trajectories mod's atmospheric impact prediction, its
    /// landing target and its descent profile, over kRPC.
    ///
    /// WHY. Stock kRPC has no impact prediction of any kind, and Trajectories' is the
    /// good one: it integrates the descent through the stock drag cubes, at the attitude
    /// the vessel actually has - or at the descent profile you set - which is exactly
    /// what a ballistic predictor under-models for a booster trimmed at high angle of
    /// attack. It is a witness, not an autopilot: the prediction IGNORES THRUST, so it
    /// answers "where do I land if I cut the engines now", not "where will I land".
    ///
    /// THE FIRST FIVE MEMBERS ARE FROZEN. Available(), HasImpact(), GetImpactGeo(),
    /// GetImpactPosition() and GetTimeTillImpact() carry the names, shapes and failure
    /// behaviour of the previous single-DLL bridge (KRPCTrajectories.cs), because a
    /// Python client - aero/trajectories_oracle.py and its test - pins them byte for
    /// byte. Available() in particular is a PROCEDURE, not a property like every other
    /// plugin's: Python calls available() with parentheses there, and a property would
    /// make that call fail as "bool is not callable". Do not tidy it.
    ///
    /// UNITS AND FRAMES, once:
    ///   - angles in and out of this service are DEGREES. Trajectories.API itself speaks
    ///     radians (ResetDescentProfile, DescentProfileAngles); the conversion is here.
    ///   - GetImpactPosition(), GetImpactVelocity(), GetPlannedDirection() and
    ///     GetCorrectedDirection() return the mod's raw Vector3 as [x, y, z]. For the
    ///     impact position that is the mod's "ImpactPosition": body-relative, world
    ///     axes, already rotated to where that surface point is NOW - so
    ///     body.position + it is a world position at this instant, and that is how
    ///     GetImpactGeo() turns it into latitude and longitude. Velocity and the two
    ///     navball directions are world-axis vectors.
    ///   - GetImpactGeo() and GetTarget() are [latitude deg, longitude deg, altitude m]
    ///     on the active vessel's main body; the impact altitude is the terrain height
    ///     at that point, the target altitude is what was set.
    ///
    /// "NOTHING TO REPORT" has two conventions, and the split is deliberate: the two
    /// frozen vector members THROW (as the old bridge did, so a caller guards with
    /// has_impact()), while every NEW vector member returns an EMPTY LIST and the time
    /// members return -1, so a control loop can poll them without exceptions.
    ///
    /// Signature notes, because a malformed one stops the whole kRPC server: Vector3 and
    /// Vector3d are not legal kRPC types, so every vector is a flat IList of double, and
    /// every list is built as a List - never an array, which the scanner accepts and the
    /// wire cannot encode (measured on Ident, 17/08).
    ///
    /// Threading: kRPC runs these bodies on Unity's main thread inside FixedUpdate, which
    /// is where Trajectories expects to be called, so there is no dispatch here - the same
    /// reasoning as every other plugin in this repo (see Core/MainThread.cs).
    /// </summary>
    [KRPCService (Name = "Trajectories", GameScene = GameScene.Flight)]
    public static class TrajectoriesService
    {
        const double Deg2Rad = Math.PI / 180.0;
        const double Rad2Deg = 180.0 / Math.PI;

        // ------------------------------------------------------------------
        // Frozen API (names and shapes of the previous bridge)
        // ------------------------------------------------------------------

        /// <summary>
        /// True if the Trajectories mod is installed and its API resolved. A procedure,
        /// not a property, on purpose: existing clients call it with parentheses.
        /// </summary>
        [KRPCProcedure]
        public static bool Available ()
        {
            return TrajectoriesApi.Resolved;
        }

        /// <summary>
        /// True if Trajectories currently has an atmospheric impact prediction for the
        /// active vessel. False when the mod is absent, the vessel is not going to hit
        /// the ground, or the mod has not computed yet. Never throws.
        /// </summary>
        [KRPCProcedure]
        public static bool HasImpact ()
        {
            if (!TrajectoriesApi.Resolved)
                return false;
            try {
                Vector3d unused;
                return TrajectoriesApi.TryVector (TrajectoriesApi.GetImpactPosition, "GetImpactPosition", out unused);
            } catch (Exception) {
                return false;
            }
        }

        /// <summary>
        /// Predicted impact as [latitude deg, longitude deg, terrain altitude m] on the
        /// active vessel's main body. Throws if there is no impact: guard with
        /// <see cref="HasImpact"/>.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GetImpactGeo ()
        {
            var impact = RequireImpact ();
            var body = MainBody ();
            Vector3d world = impact + body.position;        // body-relative -> world
            double lat = body.GetLatitude (world);
            double lon = body.GetLongitude (world);
            double alt = body.TerrainAltitude (lat, lon);   // surface (PQS) height
            return new List<double> (3) { lat, lon, alt };
        }

        /// <summary>
        /// Predicted impact as the mod's raw vector [x, y, z] in metres: relative to the
        /// main body's centre, world axes, rotated to where that surface point is now.
        /// Add the body's position for a world position. Throws if there is no impact:
        /// guard with <see cref="HasImpact"/>.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GetImpactPosition ()
        {
            return TrajectoriesApi.Flat (RequireImpact ());
        }

        /// <summary>Seconds until the predicted impact, or -1 if there is none. Never throws.</summary>
        [KRPCProcedure]
        public static double GetTimeTillImpact ()
        {
            return SecondsOrMinusOne (TrajectoriesApi.GetTimeTillImpact, "GetTimeTillImpact");
        }

        // ------------------------------------------------------------------
        // Health
        // ------------------------------------------------------------------

        /// <summary>Health check.</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        /// <summary>What resolution found, member by member. Quote this line in a bug report.</summary>
        [KRPCProperty]
        public static string Diagnostics {
            get { return TrajectoriesApi.Report; }
        }

        /// <summary>Version of the Trajectories assembly, e.g. "2.4.5.4", or empty when absent.</summary>
        [KRPCProperty]
        public static string ModVersion {
            get { return TrajectoriesApi.ModVersion; }
        }

        /// <summary>The version the mod's API reports about itself, e.g. "2.4.5". Raises if the mod is absent.</summary>
        [KRPCProperty]
        public static string ApiVersion {
            get { return TrajectoriesApi.Get (TrajectoriesApi.GetVersionProp, "GetVersion") as string ?? string.Empty; }
        }

        // ------------------------------------------------------------------
        // Prediction, extended
        // ------------------------------------------------------------------

        /// <summary>
        /// Universal time of the predicted impact, or -1 if there is none. The same
        /// instant as <see cref="GetTimeTillImpact"/> counts down to, as an absolute.
        /// </summary>
        [KRPCProcedure]
        public static double GetEndTime ()
        {
            return SecondsOrMinusOne (TrajectoriesApi.GetEndTime, "GetEndTime");
        }

        /// <summary>
        /// Predicted velocity at impact, [x, y, z] in m/s, world axes. Empty list if there
        /// is no impact.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GetImpactVelocity ()
        {
            return VectorOrEmpty (TrajectoriesApi.GetImpactVelocity, "GetImpactVelocity");
        }

        /// <summary>
        /// Ask the mod to recompute now rather than at its own cadence. The result still
        /// lands on a later tick; this does not block. Cheap to call, not free to stream.
        /// </summary>
        [KRPCProcedure]
        public static void UpdateTrajectory ()
        {
            TrajectoriesApi.Call (TrajectoriesApi.UpdateTrajectory, "UpdateTrajectory");
        }

        /// <summary>
        /// Whether Trajectories integrates even with its window closed and the map view
        /// off. The bridge switches this on at every flight-scene start so a headless
        /// script gets a live prediction; set it false here to let the mod idle.
        /// </summary>
        [KRPCProperty]
        public static bool AlwaysUpdate {
            get {
                var raw = TrajectoriesApi.Get (TrajectoriesApi.AlwaysUpdateProp, "AlwaysUpdate");
                return raw is bool && (bool) raw;
            }
            set { TrajectoriesApi.Set (TrajectoriesApi.AlwaysUpdateProp, "AlwaysUpdate", value); }
        }

        // ------------------------------------------------------------------
        // Target
        // ------------------------------------------------------------------

        /// <summary>True if a landing target is set on the active vessel. False when none, or no vessel.</summary>
        [KRPCProcedure]
        public static bool HasTarget ()
        {
            var raw = TrajectoriesApi.Call (TrajectoriesApi.HasTarget, "HasTarget");
            return raw is bool && (bool) raw;
        }

        /// <summary>
        /// Set the landing target at a latitude and longitude (degrees) on the active
        /// vessel's main body. Pass NaN as the altitude to place it on the ground there,
        /// which is what the mod's own GUI does; any other value is metres above sea
        /// level. Enables <see cref="GetPlannedDirection"/> and
        /// <see cref="GetCorrectedDirection"/>.
        /// </summary>
        [KRPCProcedure]
        public static void SetTarget (double latitude, double longitude, double altitude)
        {
            if (double.IsNaN (latitude) || double.IsNaN (longitude)
                || double.IsInfinity (latitude) || double.IsInfinity (longitude))
                throw new ArgumentException ("latitude and longitude must be finite degrees");
            object alt = double.IsNaN (altitude) ? null : (object) altitude;
            TrajectoriesApi.Call (TrajectoriesApi.SetTarget, "SetTarget", latitude, longitude, alt);
        }

        /// <summary>Remove the landing target.</summary>
        [KRPCProcedure]
        public static void ClearTarget ()
        {
            TrajectoriesApi.Call (TrajectoriesApi.ClearTarget, "ClearTarget");
        }

        /// <summary>
        /// The landing target as [latitude deg, longitude deg, altitude m], or an empty
        /// list when none is set.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GetTarget ()
        {
            return VectorOrEmpty (TrajectoriesApi.GetTarget, "GetTarget");
        }

        /// <summary>
        /// The direction the mod would fly to reach the target from here, [x, y, z]
        /// world axes - the navball marker it draws. Empty list when no target is set.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GetPlannedDirection ()
        {
            return VectorOrEmpty (TrajectoriesApi.PlannedDirection, "PlannedDirection");
        }

        /// <summary>
        /// The correction the mod suggests to bring the predicted impact onto the target,
        /// [x, y, z] world axes - its second navball marker. Empty list when no target is
        /// set.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GetCorrectedDirection ()
        {
            return VectorOrEmpty (TrajectoriesApi.CorrectedDirection, "CorrectedDirection");
        }

        // ------------------------------------------------------------------
        // Descent profile
        //
        // Four nodes, always in this order: 0 = atmospheric entry, 1 = high altitude,
        // 2 = low altitude, 3 = final approach. The prediction integrates the attitude
        // these describe instead of the vessel's current one, which is how a script
        // tells the mod "I will be flying retrograde on the way down" while the booster
        // is still pointed the other way.
        // ------------------------------------------------------------------

        /// <summary>
        /// Angle of each of the four descent-profile nodes, in degrees (entry, high, low,
        /// final). An angle beyond 90 degrees in magnitude is a retrograde node.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GetDescentProfileAngles ()
        {
            var raw = TrajectoriesApi.Get (TrajectoriesApi.AnglesProp, "DescentProfileAngles") as IList<double>;
            var output = new List<double> (4);
            if (raw != null)
                foreach (var radians in raw)
                    output.Add (radians * Rad2Deg);
            return output;
        }

        /// <summary>
        /// Mode of each of the four nodes: true = the angle is an angle of attack relative
        /// to the velocity, false = it is measured from the horizon.
        /// </summary>
        [KRPCProcedure]
        public static IList<bool> GetDescentProfileModes ()
        {
            return Bools (TrajectoriesApi.Get (TrajectoriesApi.ModesProp, "DescentProfileModes"));
        }

        /// <summary>Grade of each of the four nodes: true = retrograde, false = prograde.</summary>
        [KRPCProcedure]
        public static IList<bool> GetDescentProfileGrades ()
        {
            return Bools (TrajectoriesApi.Get (TrajectoriesApi.GradesProp, "DescentProfileGrades"));
        }

        /// <summary>
        /// Write the whole descent profile in one call: four angles in degrees, four modes
        /// (true = angle of attack, false = from horizon) and four grades (true =
        /// retrograde). Each list must have exactly four entries, in the order entry,
        /// high, low, final. The mod saves the profile with the vessel.
        /// </summary>
        [KRPCProcedure]
        public static void SetDescentProfile (IList<double> anglesDegrees, IList<bool> modes, IList<bool> grades)
        {
            if (anglesDegrees == null || anglesDegrees.Count != 4)
                throw new ArgumentException ("anglesDegrees must hold exactly 4 values (entry, high, low, final)");
            if (modes == null || modes.Count != 4)
                throw new ArgumentException ("modes must hold exactly 4 values (entry, high, low, final)");
            if (grades == null || grades.Count != 4)
                throw new ArgumentException ("grades must hold exactly 4 values (entry, high, low, final)");

            var radians = new List<double> (4);
            foreach (var degrees in anglesDegrees) {
                if (double.IsNaN (degrees) || double.IsInfinity (degrees))
                    throw new ArgumentException ("descent profile angles must be finite degrees");
                radians.Add (degrees * Deg2Rad);
            }

            // The mod wants its own List types back, not whatever kRPC decoded into.
            TrajectoriesApi.Set (TrajectoriesApi.AnglesProp, "DescentProfileAngles", radians);
            TrajectoriesApi.Set (TrajectoriesApi.ModesProp, "DescentProfileModes", new List<bool> (modes));
            TrajectoriesApi.Set (TrajectoriesApi.GradesProp, "DescentProfileGrades", new List<bool> (grades));
        }

        /// <summary>
        /// Reset all four nodes to one angle of attack in degrees. 0 is prograde at zero
        /// AoA; 180 is retrograde, tail first; anything beyond 90 in magnitude is taken as
        /// retrograde by the mod. This is the one-liner for "predict my descent as if I
        /// were flying backwards".
        /// </summary>
        [KRPCProcedure]
        public static void ResetDescentProfile (double aoaDegrees)
        {
            if (double.IsNaN (aoaDegrees) || double.IsInfinity (aoaDegrees))
                throw new ArgumentException ("aoaDegrees must be finite");
            TrajectoriesApi.Call (TrajectoriesApi.ResetDescentProfile, "ResetDescentProfile", aoaDegrees * Deg2Rad);
        }

        /// <summary>
        /// True when all four nodes are retrograde. Setting it true resets the whole
        /// profile to retrograde at zero AoA (tail first), false to prograde at zero AoA.
        /// Reads false with no active vessel.
        /// </summary>
        [KRPCProperty]
        public static bool RetrogradeEntry {
            get {
                var raw = TrajectoriesApi.Get (TrajectoriesApi.RetrogradeEntryProp, "RetrogradeEntry");
                return raw is bool && (bool) raw;
            }
            set { TrajectoriesApi.Set (TrajectoriesApi.RetrogradeEntryProp, "RetrogradeEntry", value); }
        }

        /// <summary>
        /// True when all four nodes are prograde. Setting it true resets the whole
        /// profile to prograde at zero AoA, false to retrograde at zero AoA. Reads false
        /// with no active vessel.
        /// </summary>
        [KRPCProperty]
        public static bool ProgradeEntry {
            get {
                var raw = TrajectoriesApi.Get (TrajectoriesApi.ProgradeEntryProp, "ProgradeEntry");
                return raw is bool && (bool) raw;
            }
            set { TrajectoriesApi.Set (TrajectoriesApi.ProgradeEntryProp, "ProgradeEntry", value); }
        }

        // ------------------------------------------------------------------
        // Helpers
        // ------------------------------------------------------------------

        static Vector3d RequireImpact ()
        {
            TrajectoriesApi.Require ();
            Vector3d impact;
            if (!TrajectoriesApi.TryVector (TrajectoriesApi.GetImpactPosition, "GetImpactPosition", out impact))
                throw new InvalidOperationException ("Trajectories: no impact prediction");
            return impact;
        }

        static CelestialBody MainBody ()
        {
            var vessel = FlightGlobals.ActiveVessel;
            var body = vessel != null ? vessel.mainBody : FlightGlobals.currentMainBody;
            if (body == null)
                throw new InvalidOperationException ("Trajectories: no main body");
            return body;
        }

        static double SecondsOrMinusOne (System.Reflection.MethodInfo method, string name)
        {
            if (!TrajectoriesApi.Resolved || method == null)
                return -1.0;
            try {
                var raw = TrajectoriesApi.Call (method, name);
                if (raw is double)
                    return (double) raw;
                if (raw is float)
                    return (float) raw;
                return -1.0;
            } catch (Exception) {
                return -1.0;
            }
        }

        static IList<double> VectorOrEmpty (System.Reflection.MethodInfo method, string name)
        {
            Vector3d vector;
            if (!TrajectoriesApi.TryVector (method, name, out vector))
                return new List<double> ();
            return TrajectoriesApi.Flat (vector);
        }

        static IList<bool> Bools (object raw)
        {
            var output = new List<bool> (4);
            var list = raw as IList<bool>;
            if (list != null)
                output.AddRange (list);
            return output;
        }
    }
}
