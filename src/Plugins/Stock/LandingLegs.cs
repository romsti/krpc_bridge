using System;
using System.Collections.Generic;
using ModuleWheels;

using SCVessel = KRPC.SpaceCenter.Services.Vessel;

namespace KRPC.Bridge.Stock
{
    /// <summary>
    /// The deployment of the vessel's landing legs, as KSP's ModuleWheelDeployment runs it.
    ///
    /// WHY. Terminal Guidance asks for its legs at the start of the powered descent, at 45 to 160 kPa of
    /// dynamic pressure, so the whole high-q descent flies legs out (02-03/10/2026: legs asked 0.0-1.1 s
    /// after ignition on every booster). It would rather ask for them as late as their deployment allows,
    /// and that needs the deployment's duration before asking. kRPC exposes Leg.State and Leg.Deployed, not
    /// how long the deployment lasts.
    ///
    /// WHAT KSP DOES (Assembly-CSharp 1.12, decompiled). ModuleWheelDeployment.OnStart reads animLength from
    /// its animation clip (AnimationState.length, the clip at speed 1) and sets animatePhysics, so the
    /// animation runs on physics time. Entering "Deploying..." sets the animation speed to
    /// extendDurationFactor (its opposite when deployedPosition is 0); the leg is "Deployed" when the
    /// normalized time is within 0.01 of deployedPosition. A deployment from `position` therefore lasts
    /// animLength * |deployedPosition - position| / |extendDurationFactor| seconds of game time.
    ///
    /// Read-only: nothing here touches the legs.
    /// </summary>
    public static class LandingLegs
    {
        /// <summary>The state codes of LegsDeploymentV1.</summary>
        public const double Retracted = 0.0, Deploying = 1.0, Deployed = 2.0, Retracting = 3.0, Other = 4.0;

        /// <summary>
        /// Six numbers per landing leg (kRPC's definition: a part whose ModuleWheelBase is of type LEG),
        /// in part order: [0] animLength (s), [1] extendDurationFactor, [2] retractDurationFactor,
        /// [3] position (0..1, now), [4] deployedPosition, [5] state (0 retracted, 1 deploying, 2 deployed,
        /// 3 retracting, 4 other: inoperable or unknown). A fixed leg (no ModuleWheelDeployment) gives
        /// [0, 0, 0, 1, 1, 2]. Empty list: the vessel has no landing leg.
        /// </summary>
        public static IList<double> Deployment (SCVessel vessel)
        {
            if (vessel == null)
                throw new ArgumentNullException ("vessel");
            var r = new List<double> ();
            var parts = vessel.InternalVessel.Parts;
            for (int i = 0; i < parts.Count; i++) {
                Part p = parts [i];
                var wheel = p.FindModuleImplementing<ModuleWheelBase> ();
                if (wheel == null || wheel.wheelType != WheelType.LEG)
                    continue;
                var d = p.FindModuleImplementing<ModuleWheelDeployment> ();
                if (d == null) {
                    r.AddRange (new [] { 0.0, 0.0, 0.0, 1.0, 1.0, Deployed });
                    continue;
                }
                r.Add (d.animLength);
                r.Add (d.extendDurationFactor);
                r.Add (d.retractDurationFactor);
                r.Add (d.position);
                r.Add (d.deployedPosition);
                r.Add (State (d));
            }
            return r;
        }

        static double State (ModuleWheelDeployment d)
        {
            var fsm = d.fsm;
            if (fsm == null)
                return Other;
            var e = fsm.CurrentState;
            if (e == null)
                return Other;
            if (e == d.st_retracted)
                return Retracted;
            if (e == d.st_deploying)
                return Deploying;
            if (e == d.st_deployed)
                return Deployed;
            if (e == d.st_retracting)
                return Retracting;
            return Other;
        }

        /// <summary>
        /// The game-time duration (s) of a deployment from now, given the six numbers of one leg: 0 when
        /// already deployed; NaN when the numbers do not describe a deployment (speed or clip length not
        /// positive). Pure: shared with the tests.
        /// </summary>
        public static double DeploymentDuration (double animLength, double extendDurationFactor, double position,
                                          double deployedPosition, double state)
        {
            if (state == Deployed)
                return 0.0;
            double speed = Math.Abs (extendDurationFactor);
            if (!(animLength > 0.0) || !(speed > 0.0))
                return double.NaN;
            return animLength * Math.Abs (deployedPosition - position) / speed;
        }
    }
}
