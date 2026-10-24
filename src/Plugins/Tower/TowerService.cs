using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Service;
using KRPC.Service.Attributes;
using UnityEngine;

using SCVessel = KRPC.SpaceCenter.Services.Vessel;
using SCPart = KRPC.SpaceCenter.Services.Parts.Part;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// conn.tower - ground-test tools for the booster catch tower.
    ///
    /// WHY THIS EXISTS. Before any flight, the catch has to be tried statically: the booster,
    /// pins out and aligned, put down on the tower rails from a few tens of centimetres, to see
    /// whether the pins, rails, hinges and clamps hold its weight, how far the arms sag and
    /// whether it hangs steady. kRPC can read a vessel's pose but cannot set it, and the stock
    /// cheat screen (Set Position) takes latitude, longitude and altitude of a point the user
    /// cannot choose. PlaceV1 sets the pose of the ROOT part exactly, in any kRPC reference
    /// frame, and holds the vessel still until ReleaseV1: long enough for a nearby vessel (the
    /// tower) to unpack before the drop.
    ///
    /// TEST TOOL, not a flight feature: it teleports a vessel.
    /// </summary>
    [KRPCService (Name = "Tower", GameScene = GameScene.Flight)]
    public static partial class TowerService
    {
        /// <summary>Health check.</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        /// <summary>
        /// Teleport a loaded, unpacked vessel so that its ROOT part has the given pose, and hold
        /// it there, velocity and spin zero, for at most <paramref name="holdSeconds"/> seconds
        /// or until ReleaseV1. Every rigid body of the vessel (robotic parts' moving bodies
        /// included) is moved by the same rigid transform, so the vessel's shape is kept.
        /// </summary>
        /// <param name="vessel">The vessel to place (loaded and unpacked).</param>
        /// <param name="position">Root part position, [x, y, z] in <paramref name="referenceFrame"/>.</param>
        /// <param name="rotation">Root part rotation, quaternion [x, y, z, w] in <paramref name="referenceFrame"/>.</param>
        /// <param name="referenceFrame">Frame of position and rotation (use a body frame: the pose is kept relative to the body).</param>
        /// <param name="holdSeconds">Safety limit of the hold, in seconds of game time (0.1 to 120).</param>
        /// <returns>True when the vessel was placed and is being held.</returns>
        [KRPCProcedure]
        public static bool PlaceV1 (SCVessel vessel, IList<double> position, IList<double> rotation,
                                    SCReferenceFrame referenceFrame, double holdSeconds)
        {
            if (vessel == null || referenceFrame == null)
                throw new ArgumentNullException ("vessel / referenceFrame");
            if (position == null || position.Count != 3)
                throw new ArgumentException ("position must be [x, y, z]");
            if (rotation == null || rotation.Count != 4)
                throw new ArgumentException ("rotation must be [x, y, z, w]");
            var v = vessel.InternalVessel;
            if (!v.loaded || v.packed)
                throw new InvalidOperationException ("vessel must be loaded and unpacked");
            var worldPos = referenceFrame.PositionToWorldSpace (new Vector3d (position [0], position [1], position [2]));
            var worldRot = referenceFrame.RotationToWorldSpace (new QuaternionD (rotation [0], rotation [1], rotation [2], rotation [3]));
            return TowerAddon.Hold (v, worldPos, worldRot, Math.Max (0.1, Math.Min (120.0, holdSeconds)));
        }

        /// <summary>Stop holding the vessel placed by PlaceV1: physics takes over.</summary>
        [KRPCProcedure]
        public static void ReleaseV1 ()
        {
            TowerAddon.Release ("released by client");
        }

        /// <summary>Whether a vessel is being held by PlaceV1.</summary>
        [KRPCProperty]
        public static bool Holding {
            get { return TowerAddon.Holding; }
        }

        /// <summary>
        /// Whether this vessel (a catch tower, root part TG.TowerBase) is anchored to the ground
        /// by TowerAnchor right now. False while it is packed, not landed, or in its first
        /// physics ticks after unpacking.
        /// </summary>
        [KRPCProcedure]
        public static bool IsAnchored (SCVessel vessel)
        {
            if (vessel == null)
                throw new ArgumentNullException ("vessel");
            return TowerAnchor.IsAnchored (vessel.InternalVessel);
        }

        /// <summary>
        /// What the joint of a robotic hinge really has, read in the game. Catch tower diagnostics
        /// (30/09/2026): in flight an arm now and then never stops and swings freely, while the pilot
        /// always sends the same settings; kRPC only sees the module's fields, not the joint they
        /// are meant to drive (ModuleRoboticServoHinge.OnModifyServo: spring = motorOutput x
        /// driveSpringMutliplier, damper = driveDampingMutliplier x maxMotorOutput x size x damping).
        /// READ ONLY. NaN where a value cannot be read.
        /// [0] positionSpring, [1] positionDamper, [2] maximumForce of the joint's angular X drive;
        /// [3] motorOutput, [4] servoMotorLimit (%), [5] servoMotorSize (%), [6] hingeDamping (%),
        /// [7] driveTargetAngle, [8] targetAngle, [9] traverseVelocity (deg/s);
        /// [10] low and [11] high angular X limit of the joint (deg);
        /// [12] has enough resources, [13] motor engaged, [14] locked (1 or 0);
        /// [15] currentAngle (deg).
        /// </summary>
        /// <param name="part">A part carrying a ModuleRoboticServoHinge (TG.TowerHinge).</param>
        [KRPCProcedure]
        public static IList<double> HingeDriveV1 (SCPart part)
        {
            if (part == null)
                throw new ArgumentNullException ("part");
            var m = part.InternalPart.FindModuleImplementing<Expansions.Serenity.ModuleRoboticServoHinge> ();
            if (m == null)
                throw new InvalidOperationException ("no ModuleRoboticServoHinge on this part");
            var joint = Get (m, "servoJoint") as ConfigurableJoint;
            var r = new List<double> ();
            if (joint != null) {
                var d = joint.angularXDrive;
                r.Add (d.positionSpring);
                r.Add (d.positionDamper);
                r.Add (d.maximumForce);
            } else {
                r.Add (double.NaN);
                r.Add (double.NaN);
                r.Add (double.NaN);
            }
            foreach (var n in new [] { "motorOutput", "servoMotorLimit", "servoMotorSize", "hingeDamping",
                                       "driveTargetAngle", "targetAngle", "traverseVelocity" })
                r.Add (Num (Get (m, n)));
            if (joint != null) {
                r.Add (joint.lowAngularXLimit.limit);
                r.Add (joint.highAngularXLimit.limit);
            } else {
                r.Add (double.NaN);
                r.Add (double.NaN);
            }
            r.Add (m.HasEnoughResources ? 1.0 : 0.0);
            r.Add (Num (Get (m, "servoMotorIsEngaged")));
            r.Add (Num (Get (m, "servoIsLocked")));
            r.Add (Num (Get (m, "currentAngle")));
            return r;
        }

        /// <summary>
        /// Turn the exhaust damage of every engine of a vessel on or off, and say how many engines
        /// changed. Catch tower (02/10/2026): ModuleEngines.EngineExhaustDamage raycasts along each
        /// nozzle up to exhaustDamageMaxRange (10 m by default) and, on any part it hits, adds the
        /// WHOLE thrust of that nozzle as a force at the hit point (besides the heat): a booster that
        /// hovers over the tower arms pushes them with ~110 kN per engine at idle, when a hinge motor
        /// holds 12 kN.m. The flight code turns it off on the booster while a catch is armed.
        /// Not persistent: the next load of the vessel has the engines' own setting back.
        /// </summary>
        /// <param name="vessel">The vessel whose engines are set (loaded).</param>
        /// <param name="enabled">The exhaust damage wanted (false: no push, no heat on other parts).</param>
        /// <returns>The number of engine modules that changed.</returns>
        [KRPCProcedure]
        public static int ExhaustDamageV1 (SCVessel vessel, bool enabled)
        {
            if (vessel == null)
                throw new ArgumentNullException ("vessel");
            var v = vessel.InternalVessel;
            var n = 0;
            foreach (var p in v.parts) {
                if (p == null)
                    continue;
                foreach (var m in p.FindModulesImplementing<ModuleEngines> ()) {
                    if (m.exhaustDamage != enabled) {
                        m.exhaustDamage = enabled;
                        n++;
                    }
                }
            }
            return n;
        }

        /// <summary>
        /// Every physics joint that touches a part, read in the game: those on its own game object and
        /// those of other parts whose connected body is this part. Catch tower diagnostics (02/10/2026):
        /// once loaded as a non-active vessel, one arm of the tower moves as if held by an extra spring
        /// (no overshoot, stops early, ends past its target), while the drive and its target are those
        /// of the other arm; Kerbal Joint Reinforcement (multiPartAttachNodeReinforcement) adds joints
        /// between parts that are not neighbours. READ ONLY. One string per joint:
        /// "owner|connected|type|role|xMotion,yMotion,zMotion|angX,angY,angZ|breakForce|breakTorque",
        /// role: "attach" (the part's own attach joint), "servo" (a robotic part's servo joint) or "other".
        /// </summary>
        /// <param name="part">The part (loaded and unpacked).</param>
        [KRPCProcedure]
        public static IList<string> JointsV1 (SCPart part)
        {
            if (part == null)
                throw new ArgumentNullException ("part");
            var p = part.InternalPart;
            var rb = p.Rigidbody;
            var attach = new HashSet<Joint> ();
            if (p.attachJoint != null && p.attachJoint.Joint != null)
                attach.Add (p.attachJoint.Joint);
            var servo = new HashSet<Joint> ();
            foreach (var m in p.Modules) {
                var sj = Get (m, "servoJoint") as Joint;
                if (sj != null)
                    servo.Add (sj);
            }
            var r = new List<string> ();
            foreach (var j in UnityEngine.Object.FindObjectsOfType<Joint> ()) {
                if (j == null)
                    continue;
                var own = j.GetComponent<Rigidbody> ();
                if (own != rb && (rb == null || j.connectedBody != rb))
                    continue;
                var ownerPart = j.gameObject.GetComponentInParent<Part> ();
                var conPart = j.connectedBody != null ? j.connectedBody.gameObject.GetComponentInParent<Part> () : null;
                var role = attach.Contains (j) ? "attach" : (servo.Contains (j) ? "servo" : "other");
                var cj = j as ConfigurableJoint;
                var lin = cj != null ? cj.xMotion + "," + cj.yMotion + "," + cj.zMotion : "-";
                var ang = cj != null ? cj.angularXMotion + "," + cj.angularYMotion + "," + cj.angularZMotion : "-";
                r.Add (Name (ownerPart, j.gameObject) + "|" + Name (conPart, j.connectedBody != null ? j.connectedBody.gameObject : null)
                       + "|" + j.GetType ().Name + "|" + role + "|" + lin + "|" + ang + "|" + j.breakForce + "|" + j.breakTorque);
            }
            return r;
        }

        /// <summary>
        /// Lock a robotic hinge AT REST, at the first physics step whose INSTANTANEOUS rate is at
        /// or under <paramref name="maxRateDegS"/>, within <paramref name="timeoutS"/> seconds of
        /// game time. Catch tower (06/10/2026): KSP refuses to lock a hinge that moves
        /// (BaseServo.EngageServoLock: nothing when IsMoving), and IsMoving compares to 0.049 deg/s
        /// a MOVING AVERAGE of the absolute rate over 20 physics steps: an arm that vibrates on the
        /// spot keeps it near 0.3 deg/s, even at its turning points, and is never locked. A watcher
        /// reads the hinge's angle every physics step (modelInitialAngle + currentTransformAngle,
        /// as kRPC's current_angle) and, at the first step slow enough (a turning point of the
        /// vibration), runs the two instructions of EngageServoLock that follow its test:
        /// Fields["servoIsLocked"].SetValue(true) and ModifyLocked(null). A hinge locked already:
        /// nothing is touched (state 3). Arming the same part again replaces its watcher. The
        /// watcher ends Lost when its vessel unloads, packs or dies, and is forgotten at the end of
        /// the flight scene. A hinge at rest when armed locks on the second physics step: arm only
        /// when the lock is wanted now. Returns the status (layout hinge_lock_at_rest_v1, as
        /// hinge_lock_status_v1). Raises on a part without a hinge module ([NOT_FOUND]), a vessel
        /// that is not loaded or is packed ([NOT_LOADED]), a threshold or a time limit that is not
        /// finite and strictly positive.
        /// </summary>
        /// <param name="part">A part carrying a ModuleRoboticServoHinge (TG.TowerHinge), loaded and unpacked.</param>
        /// <param name="maxRateDegS">Instantaneous rate at or under which the hinge is locked (deg/s, &gt; 0).</param>
        /// <param name="timeoutS">Time limit of the watcher, in seconds of game time (&gt; 0).</param>
        [KRPCProcedure]
        public static IList<double> HingeLockAtRestV1 (SCPart part, double maxRateDegS, double timeoutS)
        {
            if (part == null)
                throw new ArgumentNullException ("part");
            return HingeRestLocks.Arm (part.InternalPart, maxRateDegS, timeoutS);
        }

        /// <summary>
        /// The status of the hinge lock at rest of this part (layout hinge_lock_at_rest_v1, protocol
        /// and schema in conn.bridge.layout_v1 when the Core is installed):
        /// [0] protocol (1); [1] state: 0 none (never armed in this scene), 1 armed, 2 locked by the
        /// watcher, 3 already locked (when armed, or locked by someone else while armed: nothing
        /// done), 4 expired, 5 cancelled (by the client or a new arming), 6 lost (vessel unloaded,
        /// packed or destroyed, part or module gone), 7 failed (the lock did not take);
        /// [2] elapsed_s since arming (game time, frozen at the end); [3] lock_angle_deg and
        /// [4] lock_rate_deg_s (instantaneous) of the step it locked at, NaN otherwise;
        /// [5] max_rate_deg_s and [6] timeout_s as armed; [7] physics steps watched;
        /// [8] min_rate_deg_s, the slowest instantaneous rate seen; [9] ksp_average_rate_deg_s, the
        /// average IsMoving compares to 0.049 (BaseServo.transformRateOfMotion), read now;
        /// [10] locked (1 or 0) and [11] angle_deg of the hinge now (NaN when its vessel is not
        /// loaded); [12] turning_points, the reversals of direction seen. Read only.
        /// </summary>
        /// <param name="part">A part carrying a ModuleRoboticServoHinge.</param>
        [KRPCProcedure]
        public static IList<double> HingeLockStatusV1 (SCPart part)
        {
            if (part == null)
                throw new ArgumentNullException ("part");
            return HingeRestLocks.Read (part.InternalPart);
        }

        /// <summary>
        /// Cancel the hinge lock at rest of this part if it is armed (state 5), and return its status
        /// (as hinge_lock_status_v1). Nothing else is touched: a hinge it already locked stays locked.
        /// </summary>
        /// <param name="part">A part carrying a ModuleRoboticServoHinge.</param>
        [KRPCProcedure]
        public static IList<double> HingeLockCancelV1 (SCPart part)
        {
            if (part == null)
                throw new ArgumentNullException ("part");
            return HingeRestLocks.Cancel (part.InternalPart);
        }

        static string Name (Part p, GameObject go)
        {
            if (p != null)
                return p.partInfo != null ? p.partInfo.name + "#" + p.flightID : p.name;
            return go != null ? "go:" + go.name : "world";
        }

        // A field or property of the module or one of its base classes, public or not.
        static object Get (object o, string name)
        {
            const BindingFlags f = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic
                                   | BindingFlags.DeclaredOnly;
            for (var t = o.GetType (); t != null; t = t.BaseType) {
                var fi = t.GetField (name, f);
                if (fi != null)
                    return fi.GetValue (o);
                var pi = t.GetProperty (name, f);
                if (pi != null && pi.GetIndexParameters ().Length == 0)
                    return pi.GetValue (o, null);
            }
            return null;
        }

        static double Num (object v)
        {
            if (v == null)
                return double.NaN;
            if (v is bool)
                return (bool) v ? 1.0 : 0.0;
            try {
                return Convert.ToDouble (v);
            } catch (Exception) {
                return double.NaN;
            }
        }
    }
}
