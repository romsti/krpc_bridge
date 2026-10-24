using System;
using System.Collections.Generic;
using UnityEngine;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// Anchors the catch tower to the ground, whatever the distance to the floating origin.
    ///
    /// WHY. Stock launch clamps hold a vessel with soft joints: a tall vessel rocks on them.
    /// Kerbal Joint Reinforcement fixes that with world "ground joints", but only when the
    /// vessel unpacks within a fixed distance of the floating origin (KJRGroundJointModule,
    /// MaxDistanceFromOrigin). The origin follows the active vessel in jumps of kilometres, so
    /// a tower that unpacks as a booster comes near it can be far from the origin: KJR logs
    /// "Vessel Tour Catch is 600m from origin, skipping ground joint creation" and the tower,
    /// its centre of mass 1.6 m from its axis on a 5 m base, rocks by 1 to 4.5 degrees (ground
    /// test of 29/09/2026: the booster slid off the moving rails).
    ///
    /// WHAT. Every loaded, unpacked, landed vessel whose ROOT part is TG.TowerBase gets one
    /// FixedJoint from its root rigid body to the world, unbreakable, on its FIRST unpacked
    /// physics tick, and destroyed when it packs. A world joint keeps its world anchor, so a
    /// floating-origin shift would yank the tower back to where it was: every anchor is
    /// destroyed in the shift event itself and recreated on the next tick.
    ///
    /// NO FREE WINDOW (02/10/2026). The tower used to be anchored "at its pose of the moment"
    /// after 20 free ticks: a ratchet. The free tower tips as one block on its foot toward its
    /// heavy side (the arms, and the booster once caught) and sinks, the anchor freezes it,
    /// packing saves that pose and the next unpack starts from it (north tower 0.50 to 0.54 deg
    /// in one load, 1.29 deg after three vessel switches with a booster on its arms, whose pins
    /// ended 0.16 m low under the 3-4 cm rail panels; south tower rails 0.33 m under their
    /// survey after four loads). The jolt of each free window also loaded the arm hinges (the
    /// v2 tower's arms, once upright, are symmetric to the hundredth: the old +/- asymmetry was
    /// the tilt). Anchored on its first tick, the tower keeps its packed pose, i.e. the pose it
    /// was anchored in: a tower posed upright stays upright (ground test 02/10 18h47-18h54:
    /// 0.0000 deg at every anchoring, arm hinges unchanged to the thousandth over four loads).
    ///
    /// NOT STRAIGHTENED. A first version also rotated a tilted tower upright before anchoring
    /// it (rigid rotation of every part, launch clamp world joints destroyed and remade): both
    /// v2 towers it touched, even by 0.0000 deg, ended with an arm running from stop to stop a
    /// few loads later. A tower saved tilted (before this fix) is re-posed instead
    /// (tools/poser_tour_catch.py); its tilt is logged at every anchoring.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Flight, false)]
    public class TowerAnchor : MonoBehaviour
    {
        /// <summary>The root part of every catch tower (conn.tower_site uses the same rule).</summary>
        internal const string RootPartName = "TG.TowerBase";

        static readonly Dictionary<Guid, FixedJoint> anchors = new Dictionary<Guid, FixedJoint> ();

        internal static bool IsAnchored (Vessel v)
        {
            FixedJoint j;
            return v != null && anchors.TryGetValue (v.id, out j) && j != null;
        }

        void Start ()
        {
            GameEvents.onFloatingOriginShift.Add (OnOriginShift);
        }

        void OnDestroy ()
        {
            GameEvents.onFloatingOriginShift.Remove (OnOriginShift);
            DestroyAll ("scene end");
        }

        void OnOriginShift (Vector3d offset, Vector3d nonFrame)
        {
            // re-anchored on the next tick (FixedUpdate)
            DestroyAll ("floating origin shift");
        }

        static void DestroyAll (string why)
        {
            foreach (var j in anchors.Values)
                if (j != null)
                    Destroy (j);
            if (anchors.Count > 0)
                Debug.Log ("[KRPC.Bridge.Tower] anchors released: " + why);
            anchors.Clear ();
        }

        static bool IsTower (Vessel v)
        {
            var root = v.rootPart;
            return root != null && root.partInfo != null && root.partInfo.name == RootPartName;
        }

        /// <summary>The tower's root axis against the local vertical (degrees).</summary>
        internal static double Tilt (Vessel v)
        {
            var root = v.rootPart.transform;
            Vector3 up = ((Vector3d)root.position - v.mainBody.position).normalized;
            return Vector3.Angle (root.up, up);
        }

        // Scratch of FixedUpdate (Unity main thread only): no set and no list allocated per
        // physics step (refactor R1). Same membership, same order of release.
        static readonly HashSet<Guid> seenScratch = new HashSet<Guid> ();
        static readonly List<Guid> goneScratch = new List<Guid> ();

        void FixedUpdate ()
        {
            var seen = seenScratch;
            seen.Clear ();
            foreach (var v in FlightGlobals.VesselsLoaded) {
                if (v == null || !IsTower (v))
                    continue;
                seen.Add (v.id);
                bool grounded = v.situation == Vessel.Situations.PRELAUNCH || v.situation == Vessel.Situations.LANDED;
                if (v.packed || !grounded || v.rootPart.rb == null) {
                    Release (v.id, v.packed ? "packed" : "not landed");
                    continue;
                }
                if (IsAnchored (v))
                    continue;
                var j = v.rootPart.gameObject.AddComponent<FixedJoint> ();
                j.connectedBody = null;
                j.breakForce = float.PositiveInfinity;
                j.breakTorque = float.PositiveInfinity;
                anchors [v.id] = j;
                Debug.Log ("[KRPC.Bridge.Tower] anchored " + v.vesselName + " to the ground, tilt " + Tilt (v).ToString ("F4") + " deg");
            }
            var gone = goneScratch;
            gone.Clear ();
            foreach (var id in anchors.Keys)
                if (!seen.Contains (id))
                    gone.Add (id);
            foreach (var id in gone)
                Release (id, "unloaded");
        }

        static void Release (Guid id, string why)
        {
            FixedJoint j;
            if (!anchors.TryGetValue (id, out j))
                return;
            if (j != null)
                Destroy (j);
            anchors.Remove (id);
            Debug.Log ("[KRPC.Bridge.Tower] anchor released: " + why);
        }
    }
}
