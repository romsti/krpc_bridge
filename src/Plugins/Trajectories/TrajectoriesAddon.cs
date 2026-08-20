using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Trajectories
{
    /// <summary>
    /// Registration. Hands Core the resolver and gets out of the way.
    ///
    /// Startup.Instantly, once=true. Core's own addon is also Instantly, and KSP has
    /// already ordered the two assemblies by KSPAssemblyDependency, so this has
    /// registered before Core resolves.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class TrajectoriesAddon : MonoBehaviour
    {
        void Awake ()
        {
            ModRegistry.Register ("Trajectories", TrajectoriesApi.Resolve);
        }
    }

    /// <summary>
    /// Makes Trajectories compute even with its window closed.
    ///
    /// Trajectories only integrates the trajectory while something is looking at it: its
    /// own GUI, the map view, or the "always update" setting. A script reading the impact
    /// point over kRPC with the window closed would otherwise get a prediction that is
    /// several seconds stale, or none at all. The previous single-DLL bridge turned the
    /// setting on at its first call; this does it at every flight-scene start instead,
    /// because Trajectories reloads its settings from disk in its own Awake, which runs
    /// before this Start and would undo a write made any earlier.
    ///
    /// A script that wants the mod idle can set conn.trajectories.always_update = False
    /// afterwards; nothing here re-forces it mid-scene.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public sealed class TrajectoriesHeadless : MonoBehaviour
    {
        static bool complained;

        void Start ()
        {
            if (!TrajectoriesApi.Resolved || TrajectoriesApi.AlwaysUpdateProp == null)
                return;
            try {
                TrajectoriesApi.AlwaysUpdateProp.SetValue (null, true, null);
            } catch (Exception e) {
                if (!complained) {
                    complained = true;
                    BridgeLog.Warn ("Trajectories", "could not switch AlwaysUpdate on; the "
                                    + "prediction may be stale while the mod's window is closed. "
                                    + e.Message);
                }
            }
        }
    }

    /// <summary>
    /// Every reflective handle into Trajectories, resolved once per KSP session.
    ///
    /// WHY REFLECTION. Trajectories has a real public API - the static class
    /// Trajectories.API - but kRPC cannot call another mod's static methods, and a hard
    /// reference would make this assembly fail to load when the mod is absent (and the mod
    /// is GPL-3.0, so nothing of it may be linked or copied). So: find the assembly by
    /// name, find the type, resolve each member ONCE here, and cache the MethodInfo /
    /// PropertyInfo. Resolution depends only on which assemblies are loaded, never on the
    /// scene, so once is enough. Doing it per call is the most common performance mistake
    /// in a bridge plugin; the old single-DLL bridge did it lazily but still once.
    ///
    /// WHAT THE API PROMISES, from its own doc comments, which matter for every caller:
    ///   - it "only returns correct values for the active vessel";
    ///   - every getter returns null when there is no active vessel or no computed
    ///     trajectory - a boxed Nullable arrives here as a plain null, which is what the
    ///     service turns into "no impact" / -1 / an empty list;
    ///   - the prediction ignores thrust: it integrates the current attitude, or the
    ///     descent profile if one is set, through the stock aerodynamics;
    ///   - it recomputes at its own cadence, not on each read.
    ///
    /// Verified against Trajectories 2.4.5.4 (assembly "Trajectories", KSPAssembly
    /// 2.4.5, GPL-3.0, github.com/neuoy/KSPTrajectories). The installed API was read with
    /// System.Reflection from the DLL itself; no member was trusted from a changelog.
    /// Members that exist there and are deliberately NOT exposed: GetSpaceOrbit (returns
    /// a stock Orbit, which stock kRPC already gives you), GetVersionMajor/Minor/Patch
    /// (folded into ApiVersion).
    /// </summary>
    internal static class TrajectoriesApi
    {
        // -- type ----------------------------------------------------------------
        internal static Type ApiType;                   // Trajectories.API (static class)

        // -- prediction ----------------------------------------------------------
        internal static MethodInfo GetImpactPosition;   // Vector3? GetImpactPosition()
        internal static MethodInfo GetImpactVelocity;   // Vector3? GetImpactVelocity()
        internal static MethodInfo GetTimeTillImpact;   // double?  GetTimeTillImpact()
        internal static MethodInfo GetEndTime;          // double?  GetEndTime()
        internal static MethodInfo UpdateTrajectory;    // void     UpdateTrajectory()

        // -- target --------------------------------------------------------------
        internal static MethodInfo HasTarget;           // bool     HasTarget()
        internal static MethodInfo SetTarget;           // void     SetTarget(double lat, double lon, double? alt)
        internal static MethodInfo GetTarget;           // Vector3d? GetTarget()   (lat, lon, alt)
        internal static MethodInfo ClearTarget;         // void     ClearTarget()
        internal static MethodInfo PlannedDirection;    // Vector3? PlannedDirection()
        internal static MethodInfo CorrectedDirection;  // Vector3? CorrectedDirection()

        // -- descent profile -----------------------------------------------------
        internal static MethodInfo ResetDescentProfile; // void ResetDescentProfile(double AoA)  RADIANS
        internal static PropertyInfo ProgradeEntryProp;    // bool? { get; set; }
        internal static PropertyInfo RetrogradeEntryProp;  // bool? { get; set; }
        internal static PropertyInfo AnglesProp;           // List<double> { get; set; }  RADIANS, 4 nodes
        internal static PropertyInfo ModesProp;            // List<bool>   { get; set; }  true = AoA, false = horizon
        internal static PropertyInfo GradesProp;           // List<bool>   { get; set; }  true = retrograde

        // -- settings ------------------------------------------------------------
        internal static PropertyInfo AlwaysUpdateProp;     // bool { get; set; }
        internal static PropertyInfo GetVersionProp;       // string { get; }  "2.4.5"

        /// <summary>True when the type and the two members the oracle cannot do without resolved.</summary>
        internal static bool Resolved { get; private set; }

        /// <summary>Member-by-member account of what resolution found.</summary>
        internal static string Report { get; private set; } = "not resolved yet";

        /// <summary>Version of the Trajectories assembly, or empty.</summary>
        internal static string ModVersion { get; private set; } = string.Empty;

        /// <summary>
        /// Resolve everything. Called once by Core via <see cref="TrajectoriesAddon"/>.
        ///
        /// Only GetImpactPosition and GetTimeTillImpact feed <see cref="Resolved"/>: they
        /// are what the existing Python oracle reads, and a Trajectories release that
        /// renames the descent-profile API must not take the impact point down with it.
        /// Everything else is checked at its own point of use and names itself when
        /// missing.
        /// </summary>
        internal static PluginStatus Resolve ()
        {
            Resolved = false;

            var assembly = ModRegistry.FindAssembly ("Trajectories");
            if (assembly == null) {
                Report = "no assembly named Trajectories is loaded - mod not installed?";
                return new PluginStatus { Available = false, Report = Report };
            }
            ModVersion = ModRegistry.VersionOf (assembly);

            ApiType = ModRegistry.FindType (assembly, "Trajectories.API");
            if (ApiType == null) {
                Report = "assembly found (" + ModVersion + ") but type Trajectories.API did not resolve";
                return new PluginStatus { Available = false, ModVersion = ModVersion, Report = Report };
            }

            var missing = new List<string> ();

            GetImpactPosition = Method ("GetImpactPosition", missing);
            GetImpactVelocity = Method ("GetImpactVelocity", missing);
            GetTimeTillImpact = Method ("GetTimeTillImpact", missing);
            GetEndTime = Method ("GetEndTime", missing);
            UpdateTrajectory = Method ("UpdateTrajectory", missing);

            HasTarget = Method ("HasTarget", missing);
            SetTarget = Method ("SetTarget", missing);
            GetTarget = Method ("GetTarget", missing);
            ClearTarget = Method ("ClearTarget", missing);
            PlannedDirection = Method ("PlannedDirection", missing);
            CorrectedDirection = Method ("CorrectedDirection", missing);

            ResetDescentProfile = Method ("ResetDescentProfile", missing);
            ProgradeEntryProp = Property ("ProgradeEntry", missing);
            RetrogradeEntryProp = Property ("RetrogradeEntry", missing);
            AnglesProp = Property ("DescentProfileAngles", missing);
            ModesProp = Property ("DescentProfileModes", missing);
            GradesProp = Property ("DescentProfileGrades", missing);

            AlwaysUpdateProp = Property ("AlwaysUpdate", missing);
            GetVersionProp = Property ("GetVersion", missing);

            Resolved = GetImpactPosition != null && GetTimeTillImpact != null;

            Report = Resolved
                ? (missing.Count == 0
                    ? "resolved, every member present"
                    : "resolved; missing and unavailable: " + string.Join (", ", missing.ToArray ()))
                : "Trajectories.API found but GetImpactPosition/GetTimeTillImpact missing - version drift? "
                  + "(absent: " + string.Join (", ", missing.ToArray ()) + ")";

            return new PluginStatus { Available = Resolved, ModVersion = ModVersion, Report = Report };
        }

        static MethodInfo Method (string name, List<string> missing)
        {
            MethodInfo found = null;
            try {
                found = ApiType.GetMethod (name, ModRegistry.PubStatic);
            } catch (AmbiguousMatchException) {
                // An overload set. Take the first public static of that name; the API has
                // none today, so this is future-proofing rather than a live case.
                foreach (var candidate in ApiType.GetMethods (ModRegistry.PubStatic))
                    if (candidate.Name == name) { found = candidate; break; }
            }
            if (found == null)
                missing.Add (name + "()");
            return found;
        }

        static PropertyInfo Property (string name, List<string> missing)
        {
            var found = ApiType.GetProperty (name, ModRegistry.PubStatic);
            if (found == null)
                missing.Add (name);
            return found;
        }

        // ------------------------------------------------------------------------
        // Guarded invocation. Every service member goes through one of these, so the
        // three failure modes have one message each and none is a NullReferenceException:
        //   - mod absent                -> "the Trajectories mod is not installed"
        //   - this member absent        -> "Trajectories.API.X absent (v2.4.5.4)"
        //   - the mod threw             -> "Trajectories.API.X threw: <its message>"
        // ------------------------------------------------------------------------

        internal static void Require ()
        {
            if (!Resolved)
                throw new InvalidOperationException (
                    "the Trajectories mod is not installed or its API did not resolve. "
                    + "Check conn.trajectories.available() and conn.bridge.plugins. " + Report);
        }

        static string Absent (string name)
        {
            return "Trajectories.API." + name + " absent (v" + ModVersion + ")";
        }

        /// <summary>Invoke a resolved static method, unwrapping the reflection exception.</summary>
        internal static object Call (MethodInfo method, string name, params object[] args)
        {
            Require ();
            if (method == null)
                throw new InvalidOperationException (Absent (name));
            try {
                return method.Invoke (null, args);
            } catch (TargetInvocationException e) {
                var inner = e.InnerException ?? e;
                throw new InvalidOperationException ("Trajectories.API." + name + " threw: " + inner.Message, inner);
            }
        }

        /// <summary>Read a resolved static property, unwrapping the reflection exception.</summary>
        internal static object Get (PropertyInfo property, string name)
        {
            Require ();
            if (property == null || !property.CanRead)
                throw new InvalidOperationException (Absent (name));
            try {
                return property.GetValue (null, null);
            } catch (TargetInvocationException e) {
                var inner = e.InnerException ?? e;
                throw new InvalidOperationException ("Trajectories.API." + name + " threw: " + inner.Message, inner);
            }
        }

        /// <summary>Write a resolved static property, unwrapping the reflection exception.</summary>
        internal static void Set (PropertyInfo property, string name, object value)
        {
            Require ();
            if (property == null || !property.CanWrite)
                throw new InvalidOperationException (Absent (name));
            try {
                property.SetValue (null, value, null);
            } catch (TargetInvocationException e) {
                var inner = e.InnerException ?? e;
                throw new InvalidOperationException ("Trajectories.API." + name + " threw: " + inner.Message, inner);
            }
        }

        /// <summary>
        /// A boxed Vector3 from the API, or null. Boxing a Nullable with no value yields
        /// null, so the null check alone covers "no prediction".
        /// </summary>
        internal static bool TryVector (MethodInfo method, string name, out Vector3d vector)
        {
            vector = Vector3d.zero;
            var raw = Call (method, name);
            if (raw == null)
                return false;
            if (raw is Vector3) {
                var v = (Vector3) raw;
                vector = new Vector3d (v.x, v.y, v.z);
                return true;
            }
            if (raw is Vector3d) {
                vector = (Vector3d) raw;
                return true;
            }
            throw new InvalidOperationException (
                "Trajectories.API." + name + " returned a " + raw.GetType ().Name + ", not a Vector3");
        }

        /// <summary>Three doubles as the flat list every vector member returns.</summary>
        internal static IList<double> Flat (Vector3d v)
        {
            return new List<double> (3) { v.x, v.y, v.z };
        }
    }
}
