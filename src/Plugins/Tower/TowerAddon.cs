using System.Collections.Generic;
using UnityEngine;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// Holds the vessel placed by TowerService.PlaceV1: re-applies its pose and zeroes its
    /// velocities every FixedUpdate until released or until the hold time runs out.
    ///
    /// The pose is kept RELATIVE TO THE BODY (position and rotation in the body's frame, in
    /// double precision) and turned back into world space every tick: the world frame moves
    /// under a vessel (floating origin, a rotating frame near the surface), the body frame does
    /// not.
    ///
    /// MOVING A VESSEL. In flight every physical part is its own scene object joined to the
    /// others by joints, physicsless parts are children of their parent, and a robotic part
    /// (Breaking Ground) has a second rigid body for its moving side (Part.servoRb). Every rigid
    /// body is moved by the same rigid transform, except those whose transform is a child of
    /// another moved one (they follow their parent). The joints see no stress and a piston keeps
    /// its extension.
    ///
    /// It also steps the hinge locks at rest of hinge_lock_at_rest_v1 (HingeRestLocks), the
    /// tower bubbles of tower_bubble_v1 (TowerBubbles) and the ground joints of tower_ground_v1
    /// (TowerGround) every FixedUpdate, hands every floating origin shift to the ground joints
    /// (onFloatingOriginShift: they are destroyed and made again at once), and forgets all of them at
    /// the end of the scene. Each does nothing while no client has posed one.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public class TowerAddon : MonoBehaviour
    {
        static Vessel held;
        static Vector3d relPos;        // root part position in the body frame
        static QuaternionD relRot;     // root part rotation in the body frame
        static double untilUT;

        internal static bool Holding {
            get { return held != null; }
        }

        internal static bool Hold (Vessel v, Vector3d worldPos, QuaternionD worldRot, double seconds)
        {
            var body = v.mainBody;
            var inv = QuaternionD.Inverse (body.rotation);
            relPos = inv * (worldPos - body.position);
            relRot = inv * worldRot;
            untilUT = Planetarium.GetUniversalTime () + seconds;
            held = v;
            Apply ();
            Debug.Log ("[KRPC.Bridge.Tower] holding " + v.vesselName + " for at most " + seconds + " s");
            return true;
        }

        internal static void Release (string why)
        {
            if (held != null)
                Debug.Log ("[KRPC.Bridge.Tower] release " + held.vesselName + ": " + why);
            held = null;
        }

        void Start ()
        {
            // the ground joints are made again in the shift event itself (TowerGround); nothing when none is posed.
            GameEvents.onFloatingOriginShift.Add (TowerGround.OnOriginShift);
        }

        void FixedUpdate ()
        {
            // the hinge locks at rest (hinge_lock_at_rest_v1); nothing to do when none is armed.
            HingeRestLocks.Step ();
            // the tower bubbles (tower_bubble_v1); nothing to do when none is held.
            TowerBubbles.Step ();
            // the ground joints (tower_ground_v1); nothing to do when none is posed.
            TowerGround.Step ();
            if (held == null)
                return;
            if (held.state == Vessel.State.DEAD || !held.loaded) {
                Release ("vessel gone");
                return;
            }
            if (Planetarium.GetUniversalTime () > untilUT) {
                Release ("hold time elapsed");
                return;
            }
            if (!held.packed)
                Apply ();
        }

        void OnDestroy ()
        {
            held = null;
            GameEvents.onFloatingOriginShift.Remove (TowerGround.OnOriginShift);
            HingeRestLocks.Clear ("scene end");
            TowerBubbles.Clear ("scene end");
            TowerGround.Clear ("scene end");
        }

        static void Apply ()
        {
            var v = held;
            var body = v.mainBody;
            Vector3d targetPos = body.position + body.rotation * relPos;
            QuaternionD q = body.rotation * relRot;
            var targetRot = new Quaternion ((float) q.x, (float) q.y, (float) q.z, (float) q.w);
            var root = v.rootPart.transform;
            Quaternion dq = targetRot * Quaternion.Inverse (root.rotation);
            Vector3d pivot = root.position;

            var bodies = new List<Rigidbody> ();
            foreach (var p in v.parts) {
                if (p.rb != null)
                    bodies.Add (p.rb);
                if (p.servoRb != null)
                    bodies.Add (p.servoRb);
            }
            var moved = new HashSet<Transform> ();
            foreach (var rb in bodies)
                moved.Add (rb.transform);

            foreach (var rb in bodies) {
                var t = rb.transform;
                if (!HasMovedAncestor (t, moved)) {
                    Vector3 rel = (Vector3) ((Vector3d) t.position - pivot);
                    Vector3d np = targetPos + (Vector3d) (dq * rel);
                    t.position = (Vector3) np;
                    t.rotation = dq * t.rotation;
                    rb.position = t.position;
                    rb.rotation = t.rotation;
                }
                rb.velocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }
            v.IgnoreGForces (10);
        }

        static bool HasMovedAncestor (Transform t, HashSet<Transform> moved)
        {
            for (var a = t.parent; a != null; a = a.parent)
                if (moved.Contains (a))
                    return true;
            return false;
        }
    }
}
