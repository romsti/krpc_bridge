using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// Same-FixedUpdate aerodynamic-actuator observability.
    /// Kept outside DynamicsSnapshotV3 so PDG2's established schema remains unchanged.
    /// </summary>
    public static partial class ActuatorsService
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
        internal const int SchemaVersion = 1;
        internal const int HeaderStride = 6;
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
        static readonly LinkedList<double[]> history = new LinkedList<double[]> ();
        static double[] latest = new double[0];
        static long latestTick;

        internal static void Reset ()
        {
            history.Clear ();
            latest = new double[0];
            latestTick = 0;
        }

        internal static IList<double> ReadLatest ()
        {
            return new List<double> (latest);
        }

        internal static IList<double> ReadStatus ()
        {
            return new List<double> {
                ProtocolVersion, SchemaVersion, latestTick,
                history.Count, HistoryCapacity
            };
        }

        internal static IList<double> ReadHistory (int sinceTick, int maxFrames)
        {
            int limit = Math.Min (128, Math.Max (1, maxFrames));
            long oldestTick = history.First == null ? 0 : FrameTick (history.First.Value);
            long newestTick = history.Last == null ? 0 : FrameTick (history.Last.Value);
            bool droppedBefore = sinceTick > 0 && oldestTick > 0 && sinceTick < oldestTick - 1;

            var selected = new List<double[]> (limit);
            for (var node = history.First; node != null && selected.Count < limit;
                    node = node.Next) {
                if (FrameTick (node.Value) > sinceTick)
                    selected.Add (node.Value);
            }

            var output = new List<double> (6 + selected.Count * 64) {
                ProtocolVersion, SchemaVersion, selected.Count,
                oldestTick, newestTick, droppedBefore ? 1.0 : 0.0
            };
            for (int i = 0; i < selected.Count; i++) {
                output.Add (selected [i].Length);
                output.AddRange (selected [i]);
            }
            return output;
        }

        static long FrameTick (double[] frame)
        {
            return frame != null && frame.Length >= 3 ? (long)frame [2] : 0;
        }

        internal static void Capture (Vessel vessel, long physicsTick)
        {
            try {
                if (vessel == null || !vessel.loaded) {
                    Reset ();
                    return;
                }

                var rows = new List<double> ();
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

                        double deployBool = ReadBoolOrNaN (module,
                            new[] { "deploy", "deployed", "isDeployed", "extended" });
                        double deployFraction = ReadDoubleOrNaN (module,
                            new[] { "deployFraction", "deploymentFraction", "deployPercent",
                                    "deployLevel", "animTime", "normalizedTime", "progress" });
                        if (IsFinite (deployFraction) && deployFraction > 1.0 && deployFraction <= 100.0)
                            deployFraction /= 100.0;
                        if (IsFinite (deployFraction))
                            deployFraction = Math.Max (0.0, Math.Min (1.0, deployFraction));

                        double authority = ReadDoubleOrNaN (module,
                            new[] { "authorityLimiter", "authority" });
                        double deployAngle = ReadDoubleOrNaN (module,
                            new[] { "deployAngle", "deploymentAngle" });
                        double openHint = kind == 3 ? AnimationOpenHint (module) : double.NaN;
                        double enabledHint = ReadBoolOrNaN (module,
                            new[] { "enabled", "isEnabled", "moduleIsEnabled" });

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

                var frame = new List<double> (HeaderStride + rows.Count) {
                    ProtocolVersion,
                    SchemaVersion,
                    physicsTick,
                    Planetarium.GetUniversalTime (),
                    rowCount,
                    RowStride
                };
                frame.AddRange (rows);
                latest = frame.ToArray ();
                latestTick = physicsTick;
                history.AddLast (latest);
                while (history.Count > HistoryCapacity)
                    history.RemoveFirst ();
            } catch {
                // Pure observability: never perturb FixedUpdate or control.
            }
        }

        static double AnimationOpenHint (object module)
        {
            double direct = ReadBoolOrNaN (module,
                new[] { "deployed", "isDeployed", "extended", "isExtended" });
            if (IsFinite (direct))
                return direct;

            // Match the proven Python diagnostic: an active "Retract" action means
            // the animation is currently open. Conversely, an active Deploy/Extend
            // action means it is currently closed. This is only a hint, never control.
            object events = ReadMember (module, new[] { "Events", "events" });
            var enumerable = events as IEnumerable;
            if (enumerable == null)
                return double.NaN;
            bool sawOpenAction = false;
            bool sawCloseAction = false;
            foreach (object evt in enumerable) {
                if (evt == null)
                    continue;
                object activeObj = ReadMember (evt, new[] { "active", "Active" });
                if (activeObj is bool && !(bool)activeObj)
                    continue;
                object labelObj = ReadMember (evt, new[] { "guiName", "name", "Name" });
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

        static object ReadMember (object target, string[] names)
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
