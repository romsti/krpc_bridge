using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using ModuleWheels;
using UnityEngine;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// conn.telemetry.legs_geometry_v1, read-only: where the feet of a vessel's legs are, and
    /// what they take. One row per wheel module (ModuleWheelBase: legs and wheels) in the order of
    /// contact_v1 (part order, then module order), so a client joins the two by row or by (part
    /// persistentId, module ordinal).
    ///
    /// WHY. the client's datum (the height of the centre of mass above the feet, what D's last metres
    /// and the contact cut read) comes from a model of the craft: wrong by 0.45-0.88 m on the super hope
    /// (a study of 14/10, B4/E5), hence a 3-5 s zone C. The legs' colliders give it exactly.
    ///
    /// WHAT KSP DOES (Assembly-CSharp 1.12.5, through assembly inspection: the member, the name returned checked
    /// against the name asked, and decompiled where the geometry matters):
    ///   * ModuleWheelBase: public Transform wheelColliderHost, KSPFields float radius, Vector3 center;
    ///     Wheel (KSPWheelController, null before its setup);
    ///   * KSPWheelController: public VPWheelCollider wheelCollider; public properties WheelCenter
    ///     (cachedTransform.TransformPoint(center + up suspensionDistance (1 - suspensionAnchor)) + WheelDown
    ///     contactDepth), WheelDown (-cachedTransform.up), WheelRadius (wheelCollider.radius);
    ///   * VehiclePhysics.VPWheelCollider: public fields center, radius, suspensionDistance,
    ///     suspensionAnchor, springRate, damperRate; property cachedTransform. GetContactDepth(contactPoint,
    ///     travel) = travel + radius - (depth of the point below the transform's origin + suspensionDistance
    ///     (1 - suspensionAnchor)), the travel being suspensionDistance (FixedUpdateVehicle): the depth is 0,
    ///     the leg just touching at full extension, for a point radius + suspensionDistance suspensionAnchor
    ///     below the origin along -up;
    ///   * ModuleWheelSuspension (a ModuleWheelSubmodule: public int baseModuleIndex, the index of its
    ///     ModuleWheelBase among the part's modules): KSPFields suspensionDistance, springRatio, damperRatio,
    ///     maximumLoad, targetPosition; OnWheelSetup copies suspensionDistance x rescaleFactor and
    ///     targetPosition into the collider's suspensionDistance and suspensionAnchor;
    ///   * ModuleWheelDamage (a ModuleWheelSubmodule): KSPFields impactTolerance, stressTolerance,
    ///     isDamaged; Part: public float crashTolerance.
    /// Two feet are published, both in the body frame about the current centre of mass, metres: the foot
    /// AT FULL EXTENSION by the contact-depth rule above (origin - up (radius + suspensionDistance
    /// suspensionAnchor)), and the lowest point of the wheel NOW by KSP's own WheelCenter + WheelDown
    /// WheelRadius. Which one a touchdown datum should read is for the ground check (autopilot
    /// the client): on the ground the live contact point must sit on the second, and the first must sit
    /// contact_depth below it if contactDepth is the compression. The live contact point (WheelState.hit.point)
    /// and contactDepth are published beside them.
    /// </summary>
    internal static class LegsGeometry
    {
        internal const int Protocol = 1;
        internal const int Schema = 1;
        internal const int HeaderStride = 14;
        internal const int WheelStride = 42;

        /// <summary>A leg is deployed when its position is within this of deployedPosition (KSP's own test).</summary>
        const float DeployedTolerance = 0.01f;

        /// <summary>The answer (docs/API.md, "Legs geometry"). "" = the active vessel; not loaded: found 0, no row.</summary>
        internal static IList<double> Read (string persistentId)
        {
            Vessel v;
            uint pid;
            if (string.IsNullOrEmpty (persistentId)) {
                v = FlightGlobals.ActiveVessel;
                pid = v != null ? v.persistentId : 0u;
                if (v != null && !v.loaded)
                    v = null;
            } else {
                pid = VesselRegistry.ParsePersistentId (persistentId);
                v = VesselRegistry.FindLoaded (pid);
            }
            var reference = v != null ? v.ReferenceTransform : null;
            var rows = new List<double> ();
            int wheels = 0, legs = 0, deployed = 0, grounded = 0;
            double footBelow = double.NaN, lowestBelow = double.NaN, hitSum = 0.0;
            if (v != null && reference != null) {
                Vector3 com = v.CurrentCoM;
                var parts = v.parts;
                for (int p = 0; p < parts.Count; p++) {
                    var part = parts [p];
                    if ((object)part == null || part.Modules == null)
                        continue;
                    int ordinal = 0;
                    for (int m = 0; m < part.Modules.Count; m++) {
                        var wheel = part.Modules [m] as ModuleWheelBase;
                        if (wheel == null)
                            continue;
                        int start = rows.Count;
                        Row (rows, part, wheel, m, ordinal++, com, reference);
                        wheels++;
                        bool isLeg = wheel.wheelType == WheelType.LEG;
                        bool isOut = rows [start + 4] == 1.0;
                        if (isLeg)
                            legs++;
                        if (isLeg && isOut) {
                            deployed++;
                            double foot = -rows [start + 22], lowest = -rows [start + 25];
                            if (!double.IsNaN (foot) && !(foot <= footBelow))
                                footBelow = foot;
                            if (!double.IsNaN (lowest) && !(lowest <= lowestBelow))
                                lowestBelow = lowest;
                        }
                        if (rows [start + 27] == 1.0 && !double.IsNaN (rows [start + 29])) {
                            grounded++;
                            hitSum += -rows [start + 29];
                        }
                    }
                }
            }
            var o = new List<double> (HeaderStride + rows.Count);
            o.Add (Protocol);
            o.Add (Schema);
            o.Add (pid);
            o.Add (v != null && reference != null ? 1.0 : 0.0);
            o.Add (Ut ());
            o.Add (wheels);
            o.Add (WheelStride);
            o.Add (legs);
            o.Add (deployed);
            o.Add (grounded);
            o.Add (footBelow);
            o.Add (lowestBelow);
            o.Add (grounded > 0 ? hitSum / grounded : double.NaN);
            o.Add (v != null ? v.heightFromTerrain : double.NaN);
            o.AddRange (rows);
            return o;
        }

        /// <summary>One wheel module's row (WheelStride values; see the layout).</summary>
        static void Row (List<double> o, Part part, ModuleWheelBase wheel, int moduleIndex, int ordinal, Vector3 com,
            Transform reference)
        {
            const double nan = double.NaN;
            ModuleWheelDeployment deployment = null;
            ModuleWheelSuspension suspension = null;
            ModuleWheelDamage damage = null;
            int bases = 0;
            for (int m = 0; m < part.Modules.Count; m++)
                if (part.Modules [m] is ModuleWheelBase)
                    bases++;
            for (int m = 0; m < part.Modules.Count; m++) {
                var sub = part.Modules [m] as ModuleWheelSubmodule;
                if (sub == null || (bases > 1 && sub.baseModuleIndex != moduleIndex))
                    continue;
                if (deployment == null)
                    deployment = sub as ModuleWheelDeployment;
                if (suspension == null)
                    suspension = sub as ModuleWheelSuspension;
                if (damage == null)
                    damage = sub as ModuleWheelDamage;
            }
            bool isDeployed = deployment == null ||
                Math.Abs (deployment.position - deployment.deployedPosition) <= DeployedTolerance;
            var controller = wheel.Wheel;
            var collider = controller != null ? controller.wheelCollider : null;
            var host = collider != null ? collider.cachedTransform : null;
            var state = controller != null ? controller.currentState : null;
            o.Add (part.persistentId);
            o.Add (part.flightID);
            o.Add (ordinal);
            o.Add ((int)wheel.wheelType);
            o.Add (isDeployed ? 1.0 : 0.0);
            o.Add (deployment != null ? deployment.position : nan);
            o.Add (host != null ? 1.0 : 0.0);
            if (host != null) {
                Vector3 down = V3.Scale (host.up, -1f);
                AddBody (o, reference, V3.Sub (host.position, com));
                AddBody (o, reference, down);
                o.Add (collider.center.x);
                o.Add (collider.center.y);
                o.Add (collider.center.z);
                o.Add (collider.radius);
                o.Add (collider.suspensionDistance);
                o.Add (collider.suspensionAnchor);
                o.Add (collider.springRate);
                o.Add (collider.damperRate);
                // At full extension, by the contact-depth rule: radius + suspensionDistance suspensionAnchor below the origin.
                float reach = collider.radius + collider.suspensionDistance * collider.suspensionAnchor;
                AddBody (o, reference, V3.Sub (V3.Add (host.position, V3.Scale (down, reach)), com));
                // Now, by KSP's own wheel centre.
                AddBody (o, reference, V3.Sub (V3.Add (controller.WheelCenter,
                    V3.Scale (controller.WheelDown, controller.WheelRadius)), com));
            } else {
                for (int k = 0; k < 20; k++)
                    o.Add (nan);
            }
            bool on = state != null && state.grounded;
            o.Add (state != null ? (on ? 1.0 : 0.0) : nan);
            if (on)
                AddBody (o, reference, V3.Sub (state.hit.point, com));
            else {
                o.Add (nan);
                o.Add (nan);
                o.Add (nan);
            }
            o.Add (state != null ? state.contactDepth : nan);
            o.Add (suspension != null ? suspension.suspensionDistance : nan);
            o.Add (suspension != null ? suspension.springRatio : nan);
            o.Add (suspension != null ? suspension.damperRatio : nan);
            o.Add (suspension != null ? suspension.maximumLoad : nan);
            o.Add (suspension != null ? suspension.targetPosition : nan);
            o.Add (damage != null ? damage.impactTolerance : nan);
            o.Add (damage != null ? damage.stressTolerance : nan);
            o.Add (damage != null ? (damage.isDamaged ? 1.0 : 0.0) : nan);
            o.Add (part.crashTolerance);
            o.Add (wheel.radius);
        }

        static void AddBody (List<double> o, Transform reference, Vector3 world)
        {
            Vector3 b = reference.InverseTransformDirection (world);
            o.Add (b.x);
            o.Add (b.y);
            o.Add (b.z);
        }

        static double Ut ()
        {
            try {
                return Planetarium.fetch != null ? Planetarium.GetUniversalTime () : double.NaN;
            } catch (Exception) {
                return double.NaN;
            }
        }

        /// <summary>Float vector arithmetic component by component (Unity's operators, without depending on them).</summary>
        static class V3
        {
            internal static Vector3 Add (Vector3 a, Vector3 b)
            {
                return new Vector3 (a.x + b.x, a.y + b.y, a.z + b.z);
            }

            internal static Vector3 Sub (Vector3 a, Vector3 b)
            {
                return new Vector3 (a.x - b.x, a.y - b.y, a.z - b.z);
            }

            internal static Vector3 Scale (Vector3 a, float s)
            {
                return new Vector3 (a.x * s, a.y * s, a.z * s);
            }
        }
    }
}
