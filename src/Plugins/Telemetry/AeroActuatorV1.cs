using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;
using KRPC.Service.Attributes;
using KRPC.Bridge.Control;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// Same-FixedUpdate aerodynamic-actuator observability.
    /// Kept outside DynamicsSnapshotV3 so the established schema remains unchanged.
    /// </summary>
    public static partial class TelemetryService
    {
        /// <summary>Latest aero-actuator frame captured beside DynamicsSnapshotV3.</summary>
        [KRPCProcedure]
        public static IList<double> AeroActuatorSnapshotV1 ()
        {
            return AeroActuatorV1.ReadLatest ();
        }

        /// <summary>History of aero-actuator frames newer than sinceTick.</summary>
        [KRPCProcedure]
        public static IList<double> AeroActuatorFramesV1 (int sinceTick = 0, int maxFrames = 64)
        {
            return AeroActuatorV1.ReadHistory (sinceTick, maxFrames);
        }

        /// <summary>Protocol, schema, latest tick, history count and capacity.</summary>
        [KRPCProcedure]
        public static IList<double> AeroActuatorStatusV1 ()
        {
            return AeroActuatorV1.ReadStatus ();
        }
    }

    internal static class AeroActuatorV1
    {
        internal const int ProtocolVersion = 1;
        // Schema 2 (refactor R3): the frame ends with the clock section (ClockStride values).
        internal const int SchemaVersion = 2;
        // aero_actuator_status_v1 did not change: schema 1.
        internal const int StatusSchemaVersion = 1;
        internal const int HeaderStride = 6;
        // The trailing clock section: the GameClock master tick of the capture's step, and the
        // persistentId of the vessel the rows describe (the active vessel: the header never
        // named it).
        internal const int ClockStride = 2;
        internal const int RowStride = 9;
        internal const int HistoryCapacity = 1000;

        // module_kind:
        //   1 ModuleAeroSurface      (stock-style airbrake/aero surface)
        //   2 ModuleControlSurface   (control surface / many mod grid fins)
        //   3 ModuleAnimateGeneric   (deployment animation on the same part)
        //   4 ModuleDeployableAero   (mod/stock deployable aero helper)
        // Row fields:
        // part_flight_id, module_kind, module_ordinal, deploy_bool,
        // deploy_fraction, authority_limiter, deploy_angle_deg,
        // animation_open_hint, enabled_hint.
        // Core v2: the preallocated frame ring (its newest frame is the latest snapshot).
        static readonly Recorder history = new Recorder (HistoryCapacity);
        static long latestTick;
        // Scratch of one capture (Unity main thread only): no row list per physics step.
        static readonly List<double> rowsScratch = new List<double> ();

        internal static void Reset ()
        {
            history.Clear ();
            latestTick = 0;
        }

        internal static IList<double> ReadLatest ()
        {
            return history.CopyNewest ();
        }

        internal static IList<double> ReadStatus ()
        {
            return new List<double> {
                ProtocolVersion, StatusSchemaVersion, latestTick,
                history.Count, HistoryCapacity
            };
        }

        internal static IList<double> ReadHistory (int sinceTick, int maxFrames)
        {
            // The Core v2 reader: this transport's own copy of the ring reader gave the same
            // answer (build/tests/core compares both legacy readers with it).
            return history.ReadSince (ProtocolVersion, SchemaVersion, sinceTick, maxFrames, 64);
        }

        internal static void Capture (Vessel vessel, long physicsTick)
        {
            try {
                if (vessel == null || !vessel.loaded) {
                    Reset ();
                    return;
                }

                var rows = rowsScratch;
                rows.Clear ();
                int rowCount = BuildRows (vessel, rows);

                // Core v2: straight into the ring's next buffer, the values of the List +
                // ToArray it replaces.
                int length = HeaderStride + rows.Count + ClockStride;
                var frame = history.Reserve (length);
                frame [0] = ProtocolVersion;
                frame [1] = SchemaVersion;
                frame [2] = physicsTick;
                frame [3] = Planetarium.GetUniversalTime ();
                frame [4] = rowCount;
                frame [5] = RowStride;
                rows.CopyTo (frame, HeaderStride);
                // Schema 2 (refactor R3): the clock section, after every row.
                frame [HeaderStride + rows.Count] = GameClock.StepTick;
                frame [HeaderStride + rows.Count + 1] = vessel.persistentId;
                latestTick = physicsTick;
                history.Commit (length);
            } catch {
                // Pure observability: never perturb FixedUpdate or control.
            }
        }

        /// <summary>
        /// The rows of one capture (the body of Capture until QW6, unchanged), appended to
        /// rows; returns the row count. Also called twice by dynamics_reflection_check_v1.
        /// </summary>
        internal static int BuildRows (Vessel vessel, List<double> rows)
        {
                int rowCount = 0;
                for (int p = 0; p < vessel.parts.Count; p++) {
                    var part = vessel.parts [p];
                    if (part == null)
                        continue;
                    int aeroOrdinal = 0;
                    int controlOrdinal = 0;
                    int animateOrdinal = 0;
                    int deployableOrdinal = 0;
                    for (int m = 0; m < part.Modules.Count; m++) {
                        var module = part.Modules [m];
                        if (module == null)
                            continue;
                        string type = module.GetType ().Name;
                        int kind;
                        int ordinal;
                        if (String.Equals (type, "ModuleAeroSurface", StringComparison.Ordinal)) {
                            kind = 1; ordinal = aeroOrdinal++;
                        } else if (String.Equals (type, "ModuleControlSurface", StringComparison.Ordinal)) {
                            kind = 2; ordinal = controlOrdinal++;
                        } else if (String.Equals (type, "ModuleAnimateGeneric", StringComparison.Ordinal)) {
                            kind = 3; ordinal = animateOrdinal++;
                        } else if (String.Equals (type, "ModuleDeployableAero", StringComparison.Ordinal)) {
                            kind = 4; ordinal = deployableOrdinal++;
                        } else {
                            continue;
                        }

                        double deployBool = ReadBoolOrNaN (module, Names.Deploy);
                        double deployFraction = ReadDoubleOrNaN (module, Names.DeployFraction);
                        if (IsFinite (deployFraction) && deployFraction > 1.0 && deployFraction <= 100.0)
                            deployFraction /= 100.0;
                        if (IsFinite (deployFraction))
                            deployFraction = Math.Max (0.0, Math.Min (1.0, deployFraction));

                        double authority = ReadDoubleOrNaN (module, Names.Authority);
                        double deployAngle = ReadDoubleOrNaN (module, Names.DeployAngle);
                        double openHint = kind == 3 ? AnimationOpenHint (module) : double.NaN;
                        double enabledHint = ReadBoolOrNaN (module, Names.Enabled);

                        rows.Add (part.flightID);
                        rows.Add (kind);
                        rows.Add (ordinal);
                        rows.Add (deployBool);
                        rows.Add (deployFraction);
                        rows.Add (authority);
                        rows.Add (deployAngle);
                        rows.Add (openHint);
                        rows.Add (enabledHint);
                        rowCount++;
                    }
                }
                return rowCount;
        }

        static double AnimationOpenHint (object module)
        {
            double direct = ReadBoolOrNaN (module, Names.Deployed);
            if (IsFinite (direct))
                return direct;

            // Match the proven Python diagnostic: an active "Retract" action means
            // the animation is currently open. Conversely, an active Deploy/Extend
            // action means it is currently closed. This is only a hint, never control.
            object events = ReadMember (module, Names.Events);
            var enumerable = events as IEnumerable;
            if (enumerable == null)
                return double.NaN;
            bool sawOpenAction = false;
            bool sawCloseAction = false;
            foreach (object evt in enumerable) {
                if (evt == null)
                    continue;
                object activeObj = ReadMember (evt, Names.Active);
                if (activeObj is bool && !(bool)activeObj)
                    continue;
                object labelObj = ReadMember (evt, Names.Label);
                string label = labelObj == null ? "" : labelObj.ToString ().ToLowerInvariant ();
                if (label.Contains ("retract") || label.Contains ("close"))
                    sawOpenAction = true;
                if (label.Contains ("deploy") || label.Contains ("extend") || label.Contains ("open"))
                    sawCloseAction = true;
            }
            if (sawOpenAction && !sawCloseAction)
                return 1.0;
            if (sawCloseAction && !sawOpenAction)
                return 0.0;
            return double.NaN;
        }

        static bool IsFinite (double x)
        {
            return !double.IsNaN (x) && !double.IsInfinity (x);
        }

        /// <summary>The member names read by reflection, allocated once. Same names, same order.</summary>
        static class Names
        {
            internal static readonly string[] Deploy = { "deploy", "deployed", "isDeployed", "extended" };
            internal static readonly string[] DeployFraction = { "deployFraction", "deploymentFraction",
                "deployPercent", "deployLevel", "animTime", "normalizedTime", "progress" };
            internal static readonly string[] Authority = { "authorityLimiter", "authority" };
            internal static readonly string[] DeployAngle = { "deployAngle", "deploymentAngle" };
            internal static readonly string[] Enabled = { "enabled", "isEnabled", "moduleIsEnabled" };
            internal static readonly string[] Deployed = { "deployed", "isDeployed", "extended", "isExtended" };
            internal static readonly string[] Events = { "Events", "events" };
            internal static readonly string[] Active = { "active", "Active" };
            internal static readonly string[] Label = { "guiName", "name", "Name" };
        }

        static double ReadBoolOrNaN (object target, string[] names)
        {
            object value = ReadMember (target, names);
            if (value == null)
                return double.NaN;
            try {
                if (value is bool)
                    return (bool)value ? 1.0 : 0.0;
                return Convert.ToBoolean (value) ? 1.0 : 0.0;
            } catch {
                return double.NaN;
            }
        }

        static double ReadDoubleOrNaN (object target, string[] names)
        {
            object value = ReadMember (target, names);
            if (value == null)
                return double.NaN;
            try {
                double x = Convert.ToDouble (
                    value, System.Globalization.CultureInfo.InvariantCulture);
                return IsFinite (x) ? x : double.NaN;
            } catch {
                return double.NaN;
            }
        }

        /// <summary>
        /// QW6: cached lookups (ReflectionCache.ReadFieldThenProperty), same members and
        /// values as the legacy reader below, which stays for dynamics_reflection_check_v1.
        /// </summary>
        static object ReadMember (object target, string[] names)
        {
            if (ReflectionCache.UseLegacy)
                return ReadMemberLegacy (target, names);
            return ReflectionCache.ReadFieldThenProperty (target, names);
        }

        /// <summary>The reader flown until QW6, verbatim (GetField/GetProperty at every read).</summary>
        static object ReadMemberLegacy (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public |
                BindingFlags.NonPublic;
            for (int i = 0; i < names.Length; i++) {
                try {
                    var field = type.GetField (names [i], flags);
                    if (field != null)
                        return field.GetValue (target);
                    var property = type.GetProperty (names [i], flags);
                    if (property != null && property.GetIndexParameters ().Length == 0)
                        return property.GetValue (target, null);
                } catch { }
            }
            return null;
        }
    }
}
