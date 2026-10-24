using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // Lot 3, capacity 3.1: SurfaceModelV1,
    // the EXACT replica of a stock ModuleControlSurface, PURE (System only, compiled into
    // build/tests like AttitudeMath.cs). The game side (AttitudeV1Surfaces.cs) reads the module
    // into the plain inputs below once per physics step and calls these functions.
    //
    // Source of every formula: KSP 1.12.5 Assembly-CSharp (sha256 d9e42483...), decompiled with
    // ilspycmd through an assembly inspector (the member, and the whole type where the
    // server returned a neighbour): ModuleControlSurface.FixedUpdate, CtrlSurfaceUpdate,
    // FixedCtrlSurfaceUpdate, ModuleLiftingSurface.SetupCoefficients, GetLiftVector,
    // GetDragVector (4 arguments), ApplyLiftForceAdjustments. What the game does each physics
    // step, in that order (Normal stage, the part's own FixedUpdate):
    //
    //   1. flow:   nVel = unit point velocity; Qlift, Qdrag (Pa) from the part's dynamic pressure;
    //              mach = part.machNumber (written by FlightIntegrator, which runs AFTER the parts).
    //   2. fixed part:  lv0 = baseTransform axis (Z forward, Y up, X right) * transformSign;
    //              liftDot = nVel . lv0; absDot = |liftDot| (omnidirectional) or clamp01(liftDot);
    //              lift = GetLiftVector(lv0, liftDot, absDot) * (1 - ctrlSurfaceArea);
    //              drag = GetDragVector(nVel, absDot) * (1 - ctrlSurfaceArea)   (useInternalDragModel).
    //   3. command -> target deflection (FixedCtrlSurfaceUpdate, surfaces that are not rotor blades):
    //              a = (R_ref (pitch, 0, yaw)) . hinge;  a = -a when the CoM is below the surface
    //              (baseTransform.InverseTransformPoint(CoM).y < 0);
    //              roll = rhs.x (1 - (|rhs^ . R_base^-1 up_ref| / 2 + 1/2)) sign(base.up . up_ref)
    //                     ctrlState.roll sign(range),  rhs = (local CoM x, 0, local CoM z);
    //              target = dir * clamp(dir * clamp(a - roll, -1, 1) * range * authority / 100
    //                       + deployAngle term, -1.5 range, 1.5 range)
    //              (dir = the protected deflectionDirection, +-1 from OnStart; 0 while the lot-1
    //              neutralizer holds it).
    //   4. actuator: deflection = MoveTowards(deflection, target, actuatorSpeed dt) (degrees), or
    //              Lerp(..., actuatorSpeed dt) with useExponentialSpeed; ctrlSurface.localRotation =
    //              AngleAxis(deflection, right) * neutral.
    //   5. moving part: unless alwaysRecomputeLift is false and localRotation == neutral (Unity's
    //              quaternion equality: Quaternion.Dot, rounded to float at its return, > 0.999999f;
    //              |deflection| below ~0.16 deg; the game side calls Unity's operator itself, 
    //              within RotationBandLoDeg-HiDeg, ~0.140-0.179 deg, it depends on the stored
    //              quaternion's last bits and cannot be predicted from the deflection), lv = lv0
    //              rotated by `deflection` degrees about the hinge (baseTransform.rotation * right),
    //              f = nVel . lv, absDot = |f| (whatever omnidirectional);
    //              lift += GetLiftVector(lv, f, |f|) * area, then the mission adjusters
    //              (ApplyLiftForceAdjustments: none in flight);
    //              drag += GetDragVector(nVel, |f|) * area;
    //              lift applied at partTransform.TransformPoint(CoLOffset), drag at ...(CoPOffset).
    //   GetLiftVector(v, dot, absDot) = -v * Q LiftMultiplier * sign(dot) liftCurve(absDot)
    //              liftMachCurve(mach) deflectionLiftCoeff (projected on the plane normal to nVel
    //              with perpendicularOnly; 0 when nodeEnabled and the node holds a part);
    //   GetDragVector(nVel, absDot) = -nVel * Q LiftDragMultiplier * dragCurve(absDot)
    //              dragMachCurve(mach) deflectionLiftCoeff.
    //
    // KSP computes in float, the replica in double: the identity is checked to 1e-3 relative.
    // Units: what KSP stores (degrees, Pa, N) in the inputs; the published layout converts to SI.
    // -----------------------------------------------------------------------------------

    /// <summary>The four FloatCurves of one lifting surface (any function in the tests).</summary>
    internal interface ISurfaceCurves
    {
        double Lift (double absDot);
        double LiftMach (double mach);
        double Drag (double absDot);
        double DragMach (double mach);
    }

    /// <summary>
    /// What the force replica reads, world frame (KSP's), plain doubles: the module's constants
    /// of the step, the flow the module computed, the geometry. Preallocated, refilled per step.
    /// </summary>
    internal sealed class SurfaceFlow
    {
        internal double Area;               // ctrlSurfaceArea (moving fraction)
        internal double LiftCoeff;          // deflectionLiftCoeff
        internal double LiftMultiplier;     // PhysicsGlobals.LiftMultiplier
        internal double DragMultiplier;     // PhysicsGlobals.LiftDragMultiplier
        internal bool PerpendicularOnly;
        internal bool Omnidirectional;
        internal bool UseDrag;              // useInternalDragModel
        internal bool NodeBlocked;          // nodeEnabled and its node holds a part: no lift, no drag
        internal readonly double[] NVel = new double[3];          // protected nVel (unit)
        internal double QLift, QDrag;       // protected Qlift, Qdrag (Pa)
        internal double Mach;
        internal readonly double[] LiftVector0 = new double[3];   // base axis * transformSign
        internal readonly double[] Hinge = new double[3];         // baseTransform.rotation * right
        internal readonly double[] LeverLift = new double[3];     // CoL point - CoM
        internal readonly double[] LeverDrag = new double[3];     // CoP point - CoM
    }

    /// <summary>
    /// The command law of one surface (FixedCtrlSurfaceUpdate) as coefficients of ctrlState, from
    /// the geometry of the step; degrees and percent as KSP stores them.
    /// </summary>
    internal sealed class SurfaceCommand
    {
        internal double CPitch;     // (R_ref x) . hinge; 0 with ignorePitch
        internal double CYaw;       // (R_ref z) . hinge; 0 with ignoreYaw
        internal bool Flip;         // local CoM y < 0: the pitch/yaw action changes sign
        internal double KRoll;      // the roll term per unit of ctrlState.roll; 0 with ignoreRoll
        internal double Range;      // ctrlSurfaceRange (deg)
        internal double Authority;  // authorityLimiter (%)
        internal double Direction;  // deflectionDirection as the step sees it (0 when held neutral)
        internal double DeployTerm; // currentDeployAngle when deploy, else 0 (deg)
        internal double Speed;      // actuatorSpeed (deg/s)
        internal bool Exponential;  // useExponentialSpeed

        internal void CopyFrom (SurfaceCommand other)
        {
            CPitch = other.CPitch;
            CYaw = other.CYaw;
            Flip = other.Flip;
            KRoll = other.KRoll;
            Range = other.Range;
            Authority = other.Authority;
            Direction = other.Direction;
            DeployTerm = other.DeployTerm;
            Speed = other.Speed;
            Exponential = other.Exponential;
        }
    }

    internal static class SurfaceModel
    {
        // ------------------------------------------------------------- wire and modes
        internal const int Protocol = 1;
        internal const int Schema = 1;
        internal const int HeaderStride = 50;
        internal const int RowStride = 48;
        internal const int ClockStride = 1;
        internal const int ConfigReplyLength = 6;

        // attitude_surfaces_v1 modes (born closed: 0 at every arming).
        internal const int ModeOff = 0;
        internal const int ModeShadow = 1;      // computed and published, nothing written
        internal const int ModeArmed = 2;       // the surfaces become an actuator of the law
        internal const int ModeMask = 3;
        // (born closed), ORed into mode 1 or 2: the replica's torques enter the law -- and the
        // publication -- in N.m (forces in N), as every other actuator of the law. Without it they
        // stay in KSP's kN.m, as built up to ad35d85 (see LawTorqueScale).
        internal const int ModeFlagNewtonMetres = 4;
        internal const double NewtonPerKilonewton = 1000.0;
        // (born closed), ORed into mode 1 or 2 (with or without 4): the physical validation of
        // the replica (SurfaceKappa.cs): the per-axis slope kappa of the measured angular acceleration
        // on the torque the law is given for the surfaces' deflections, published by
        // surface_kappa_v1; armed, a surface is engaged only on axes whose kappa is proven within
        // [ScaleMin, ScaleMax], and never on an axis found out of tolerance in the arming.
        internal const int ModeFlagKappa = 8;
        // (born closed), ORed into a mode that carries ModeFlagKappa (8): the LEAK GATE of the
        // estimator's counted steps (SurfaceKappa.cs): a step counts on a body axis only while the
        // surfaces carry at least 2/3 of the excited torque increment there (the filtered increment of the
        // engines', RCS' and wheels' realized torque at most LeakMax = 1/2 of the surfaces' deflection-driven
        // one, an earlier change's rule computed on the realized increments). Changing it starts a fresh estimator.
        internal const int ModeFlagKappaLeak = 16;
        // (born closed), ORed into a mode that carries ModeFlagKappa (8): the estimator's measured side
        // multiplies by the PHYSICAL inertia, Vessel.MOI plus the parallel-axis terms it leaves out
        // (SurfaceKappa.AddInertiaOffset): the stock sum puts each rigidbody's mass at rb.position, the part's
        // origin, while the physics puts it at rb.worldCenterOfMass, the origin plus the part's CoMOffset
        // (FlightIntegrator sets rb.centerOfMass = CoMOffset at every step). Changing it starts a fresh estimator.
        internal const int ModeFlagKappaInertia = 32;
        internal const int ModeFlags = ModeFlagNewtonMetres | ModeFlagKappa | ModeFlagKappaLeak | ModeFlagKappaInertia;

        // Surface status (row offset 2): the effector registry's codes.
        internal const int StatusExact = EffectorRegistry.StatusExact;
        internal const int StatusUnmodeled = EffectorRegistry.StatusUnmodeled;

        // Why a surface is not modeled, or sanctioned (row offset 3, bits).
        internal const int ReasonSubclass = 1 << 0;        // not exactly ModuleControlSurface
        internal const int ReasonRotor = 1 << 1;           // displaceVelocity (rotor blade path)
        internal const int ReasonAdjusters = 1 << 2;       // mission adjusters on lift or speed
        internal const int ReasonReflection = 1 << 3;      // a protected member was not found
        internal const int ReasonNoTransform = 1 << 4;     // baseTransform or ctrlSurface missing
        internal const int ReasonNodeLift = 1 << 5;        // nodeEnabled
        internal const int ReasonShielded = 1 << 6;        // shielded or no rigidbody: no force this step
        internal const int ReasonIdentity = 1 << 7;        // replica off the realized force (sanction)
        internal const int ReasonNotCaptured = 1 << 8;     // no capture yet in this arming

        // Family-level sanctions (header offset 19, bits).
        internal const int SanctionScaleX = 1 << 0;        // scale guard of body x out of [0.5, 2]
        internal const int SanctionScaleY = 1 << 1;
        internal const int SanctionScaleZ = 1 << 2;
        internal const int SanctionLag = 1 << 3;           // response in the same step not established
        internal const int SanctionIdentity = 1 << 4;      // a surface's replica is off
        internal const int SanctionNotArmed = 1 << 5;      // mode is shadow (informational)
        // armed only: the scale of a body axis is not proven yet (fewer than
        // ScaleEvidenceSteps counted steps). Not a family block: a surface whose column contributes
        // to such an axis (ContributedAxes) stays out, the others may be engaged.
        internal const int SanctionUnprovenX = 1 << 6;
        internal const int SanctionUnprovenY = 1 << 7;
        internal const int SanctionUnprovenZ = 1 << 8;
        internal const int SanctionUnprovenShift = 6;
        internal const int SanctionUnproven = SanctionUnprovenX | SanctionUnprovenY | SanctionUnprovenZ;
        // armed with ModeFlagKappa only: the physical slope kappa of a body axis is not proven
        // (bits 9-11), or out of tolerance for the arming (bits 12-14). Not family blocks: per
        // surface, like the bits (a surface acting on such an axis stays out).
        internal const int SanctionKappaUnprovenShift = 9;
        internal const int SanctionKappaOutShift = 12;
        internal const int SanctionKappaUnproven = 7 << SanctionKappaUnprovenShift;
        internal const int SanctionKappaOut = 7 << SanctionKappaOutShift;
        // armed only: the ctrlState axis of a body axis (pitch x, roll y, yaw z) is not OWNED by
        // the loop for this step's engagement (AttitudeWrite.LoopOwnedAxes: not written by its own
        // fly-by-wire link at the previous physics step, or no longer permitted). Not a family block:
        // a surface whose column contributes to such an axis, or whose command answers it
        // (EngagementAxes), stays out -- another regulator (the kRPC AutoPilot) drives it there.
        internal const int SanctionNotOwnedShift = 15;
        internal const int SanctionNotOwned = 7 << SanctionNotOwnedShift;
        internal const int SanctionPerSurface = SanctionIdentity | SanctionUnproven | SanctionKappaUnproven |
            SanctionKappaOut | SanctionNotOwned;

        // ------------------------------------------------------------- thresholds
        /// <summary>Identity: |F_model - F_real| / |F_real| below this (plan 3.1, step 1).</summary>
        internal const double IdentityTolerance = 1e-3;
        /// <summary>Force below which the identity is not judged (N): q ~ 0.</summary>
        internal const double IdentityFloorN = 1.0;
        /// <summary>Consecutive judged steps above the tolerance that sanction a surface.</summary>
        internal const int IdentityStrikes = 5;
        /// <summary>Central difference of the column (deg), audit C 1.2.</summary>
        internal const double ColumnStepDeg = 0.1;
        /// <summary>Command law: a target matches the module's own within this (deg).</summary>
        internal const double TargetMatchDeg = 0.01;
        /// <summary>A step tells the response lag apart when the two targets differ by this (deg).</summary>
        internal const double LagDistinctDeg = 0.05;
        /// <summary>Same-step matches needed before the armed mode engages the surfaces.</summary>
        internal const int LagMatchesRequired = 25;
        /// <summary>... and at least this many times more than every other outcome.</summary>
        internal const int LagMatchesRatio = 20;
        /// <summary>Scale guard: the band (plan 3.1), the forgetting per step, the evidence.</summary>
        internal const double ScaleMin = 0.5;
        internal const double ScaleMax = 2.0;
        internal const double ScaleForgetting = 0.995;
        internal const int ScaleEvidenceSteps = 25;
        // a step counts on an axis when its uncertainty (everything of the realized-minus-
        // predicted increment that is not the surfaces' response, SurfaceScaleGuard) is below
        // IdentityTolerance of its predicted increment -- every counted step predicted to the
        // replica's own precision. No excitation share, no floor: both are computed per step.

        // ------------------------------------------------------------- Unity's "turned" test
        /// <summary>Unity's quaternion equality as the IL has it: Dot(a, b) &gt; 0.999999f (1 - kEpsilon).</summary>
        internal const float UnityEqualDot = 0.999999f;
        /// <summary>Spacing of the floats just below 1 (2^-24): one rounding of a unit quaternion's components moves a dot with a unit quaternion by at most this.</summary>
        internal const double FloatSpacingBelowOne = 1.0 / 16777216.0;
        /// <summary>
        /// Float roundings of unit quaternions between the exact rotation and the dot the module
        /// tests: the neutral as stored, Quaternion.AngleAxis, the product, the transform's own
        /// storage of localRotation. Each moves the dot by at most FloatSpacingBelowOne (the
        /// rounding error vector of a unit quaternion is at most 2^-25 per component, norm 2^-24,
        /// dotted with a unit quaternion); the return rounding of Dot is the threshold itself.
        /// </summary>
        internal const int RotationRoundings = 4;
        /// <summary>|deflection| (deg) where the exact sum crosses Unity's threshold: the float dot rounds to 0.999999f or below (turned) when the sum is under 0.999999f + 2^-25.</summary>
        internal static readonly double RotationThresholdDeg = RotationAngleDeg (0.0);
        /// <summary>
        /// The band of |deflection| (deg) where the module's turned state is not a function of the
        /// deflection alone: the threshold +- RotationRoundings float spacings of the dot,
        /// about 0.140-0.179 deg. Inside it, which side of Unity's test a rotation falls on depends on
        /// the last bits of the quaternion the module stored, which the bridge cannot reproduce: the
        /// a recorded flight saw the guard's prediction of the turned state disagree
        /// with the module only for fins inside it (every move of k above 0.003).
        /// </summary>
        internal static readonly double RotationBandLoDeg = RotationAngleDeg (RotationRoundings);
        internal static readonly double RotationBandHiDeg = RotationAngleDeg (-RotationRoundings);

        internal static bool ValidMode (int mode)
        {
            int baseMode = mode & ModeMask;
            int flags = mode & ~ModeMask;
            if (!(baseMode == ModeOff || baseMode == ModeShadow || baseMode == ModeArmed))
                return false;
            // the flags (unit 4, kappa 8) mean something only with the model on;
            // the leak gate (16) only with kappa (8); the physical inertia (32) likewise.
            if ((flags & (ModeFlagKappaLeak | ModeFlagKappaInertia)) != 0 && (flags & ModeFlagKappa) == 0)
                return false;
            return flags == 0 || ((flags & ~ModeFlags) == 0 && baseMode != ModeOff);
        }

        /// <summary>The mode without its flags (0 off, 1 shadow, 2 armed).</summary>
        internal static int BaseMode (int mode)
        {
            return mode & ModeMask;
        }

        /// <summary>
        /// what the law (and the publication) multiplies the replica's forces and torques by.
        /// The replica works in KSP's own force unit, the kilonewton: Qlift is in Pa, but the force
        /// goes to the part's rigidbody, whose mass is in tonnes (the bridge converts engine
        /// thrust, RCS and wheel torques the same way, x 1000). Without the flag (the build of
        /// ad35d85 and before) the law took kN.m for N.m: every fin torque it saw was 1000 times too
        /// small (a recorded flight).
        /// </summary>
        internal static double LawTorqueScale (bool newtonMetres)
        {
            return newtonMetres ? NewtonPerKilonewton : 1.0;
        }

        /// <summary>
        /// The deflection (deg) whose exact dot cos(delta/2) sits `spacings` float spacings above the
        /// point where Unity's test flips (0.999999f + 2^-25).
        /// </summary>
        static double RotationAngleDeg (double spacings)
        {
            double dot = (double)UnityEqualDot + 0.5 * FloatSpacingBelowOne + spacings * FloatSpacingBelowOne;
            return 2.0 * Math.Acos (dot) * 180.0 / Math.PI;
        }

        /// <summary>|deflection| (deg) inside the rotation band.</summary>
        internal static bool InRotationBand (double deflectionDeg)
        {
            double a = Math.Abs (deflectionDeg);
            return a >= RotationBandLoDeg && a <= RotationBandHiDeg;
        }

        /// <summary>
        /// the body axes (bit 0 x, 1 y, 2 z) a surface contributes to, from its column: those
        /// whose component is above IdentityTolerance of the column's norm. Whatever the command,
        /// a surface's torque increment lies along its column; a component below that share is
        /// within the replica's own precision (the identity tolerance, the same share under which
        /// the scale guard counts no step), so no scale proof of that axis can bear on it. A
        /// column without a finite positive norm tells nothing: every axis.
        /// </summary>
        internal static int ContributedAxes (double[] column)
        {
            double n = Norm (column [0], column [1], column [2]);
            if (!(n > 0.0) || double.IsInfinity (n))
                return 7;
            double floor = IdentityTolerance * n;
            int axes = 0;
            for (int a = 0; a < 3; a++)
                if (Math.Abs (column [a]) > floor)
                    axes |= 1 << a;
            return axes;
        }

        /// <summary>
        /// the ctrlState axes (bit 0 pitch, 1 roll, 2 yaw) a surface's command answers, from
        /// the linear coefficients of its command law (pitch CPitch, roll KRoll, yaw CYaw: the
        /// gradient of Target before any clamp, TargetGradient's), with ContributedAxes' floor: a
        /// coefficient below IdentityTolerance of the coefficients' norm moves the surface within
        /// the replica's own precision. A command with no coefficient answers nothing (0). Unlike
        /// the ignore flags, the geometry decides: a fin on the vessel's axis has no roll term.
        /// </summary>
        internal static int CommandedAxes (SurfaceCommand c)
        {
            double p = c.CPitch, r = c.KRoll, y = c.CYaw;
            double n = Norm (p, r, y);
            if (!(n > 0.0) || double.IsInfinity (n))
                return double.IsNaN (n) || double.IsInfinity (n) ? 7 : 0;
            double floor = IdentityTolerance * n;
            return (Math.Abs (p) > floor ? 1 : 0) | (Math.Abs (r) > floor ? 2 : 0) | (Math.Abs (y) > floor ? 4 : 0);
        }

        /// <summary>
        /// the axes a surface must have OWNED by the loop to be engaged: every body axis its
        /// column contributes to (its torque, whoever commands it, lands there) and every ctrlState
        /// axis its command answers (whoever writes that axis moves it). Body axis a and ctrlState
        /// axis a are the same bit (0 pitch x, 1 roll y, 2 yaw z), as AttitudeWrite's masks.
        /// </summary>
        internal static int EngagementAxes (double[] column, SurfaceCommand c)
        {
            return (ContributedAxes (column) | CommandedAxes (c)) & 7;
        }

        /// <summary>the loop owns every axis the surface needs (EngagementAxes) this step.</summary>
        internal static bool Owned (int engagementAxes, int ownedAxes)
        {
            return (engagementAxes & ~ownedAxes & 7) == 0;
        }

        // ------------------------------------------------------------- Unity's scalar rules

        /// <summary>Mathf.Sign: 1 for 0 and above.</summary>
        internal static double Sign (double v)
        {
            return v >= 0.0 ? 1.0 : -1.0;
        }

        /// <summary>Mathf.Clamp(v, min, max), in its order (min first), even when min &gt; max.</summary>
        internal static double Clamp (double v, double min, double max)
        {
            if (v < min)
                return min;
            if (v > max)
                return max;
            return v;
        }

        internal static double Clamp01 (double v)
        {
            return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
        }

        /// <summary>Mathf.MoveTowards.</summary>
        internal static double MoveTowards (double current, double target, double maxDelta)
        {
            if (Math.Abs (target - current) <= maxDelta)
                return target;
            return current + Sign (target - current) * maxDelta;
        }

        /// <summary>Mathf.Lerp (t clamped to [0, 1]).</summary>
        internal static double Lerp (double a, double b, double t)
        {
            return a + (b - a) * Clamp01 (t);
        }

        // ------------------------------------------------------------- command law

        /// <summary>
        /// Target deflection (deg) of FixedCtrlSurfaceUpdate for ctrlState (pitch, roll, yaw), the
        /// `action` field it leaves. Steps 3 above, in the game's order.
        /// </summary>
        internal static double Target (SurfaceCommand c, double pitch, double roll, double yaw)
        {
            double a = c.CPitch * pitch + c.CYaw * yaw;
            if (c.Flip)
                a = -a;
            double x = Clamp (a - c.KRoll * roll, -1.0, 1.0);
            double v = c.Direction * x;
            v = c.Range * v * c.Authority * 0.01;
            v += c.DeployTerm;
            v = Clamp (v, -1.5 * c.Range, 1.5 * c.Range);
            return v * c.Direction;
        }

        /// <summary>
        /// d target / d ctrlState (deg per unit) at (pitch, roll, yaw): g[0] pitch, g[1] roll, g[2]
        /// yaw. Zero on an axis where a clamp is reached (the target no longer moves with it).
        /// </summary>
        internal static void TargetGradient (SurfaceCommand c, double pitch, double roll, double yaw,
            double[] g)
        {
            g [0] = 0.0;
            g [1] = 0.0;
            g [2] = 0.0;
            double a = c.CPitch * pitch + c.CYaw * yaw;
            double s = c.Flip ? -1.0 : 1.0;
            double x = s * a - c.KRoll * roll;
            if (!(x > -1.0 && x < 1.0))
                return;
            double v = c.Direction * c.Range * x * c.Authority * 0.01 + c.DeployTerm;
            double lo = -1.5 * c.Range, hi = 1.5 * c.Range;
            if (!(lo < hi) || !(v > lo && v < hi))
                return;
            double k = c.Direction * c.Direction * c.Range * c.Authority * 0.01;
            g [0] = k * s * c.CPitch;
            g [1] = -k * c.KRoll;
            g [2] = k * s * c.CYaw;
        }

        /// <summary>One physics step of the actuator from `deflection` toward `target` (deg).</summary>
        internal static double Step (SurfaceCommand c, double deflection, double target, double dt)
        {
            if (c.Exponential)
                return Lerp (deflection, target, c.Speed * dt);
            return MoveTowards (deflection, target, c.Speed * dt);
        }

        // ------------------------------------------------------------- forces

        static double Dot (double[] a, double[] b)
        {
            return a [0] * b [0] + a [1] * b [1] + a [2] * b [2];
        }

        /// <summary>v rotated by `degrees` about the unit-normalized `axis` (Quaternion.AngleAxis * v).</summary>
        internal static void Rotate (double[] v, double[] axis, double degrees, double[] result)
        {
            double n = Math.Sqrt (Dot (axis, axis));
            if (!(n > 1e-12)) {
                result [0] = v [0];
                result [1] = v [1];
                result [2] = v [2];
                return;
            }
            double kx = axis [0] / n, ky = axis [1] / n, kz = axis [2] / n;
            double th = degrees * Math.PI / 180.0;
            double cs = Math.Cos (th), sn = Math.Sin (th);
            double kv = kx * v [0] + ky * v [1] + kz * v [2];
            double cx = ky * v [2] - kz * v [1];
            double cy = kz * v [0] - kx * v [2];
            double cz = kx * v [1] - ky * v [0];
            result [0] = v [0] * cs + cx * sn + kx * kv * (1.0 - cs);
            result [1] = v [1] * cs + cy * sn + ky * kv * (1.0 - cs);
            result [2] = v [2] * cs + cz * sn + kz * kv * (1.0 - cs);
        }

        /// <summary>ModuleLiftingSurface.GetLiftVector, added times `scale` into `into`.</summary>
        static void AddLift (SurfaceFlow f, ISurfaceCurves curves, double[] lv, double dot, double absDot,
            double scale, double[] into)
        {
            if (f.NodeBlocked)
                return;
            double s = Sign (dot) * curves.Lift (absDot) * curves.LiftMach (f.Mach);
            s *= f.LiftCoeff;
            if (s == 0.0 || double.IsNaN (s))
                return;
            s = f.QLift * (f.LiftMultiplier * s);
            double vx = -lv [0] * s, vy = -lv [1] * s, vz = -lv [2] * s;
            if (f.PerpendicularOnly) {
                // Vector3.ProjectOnPlane(v, -nVel).
                double nn = Dot (f.NVel, f.NVel);
                if (nn > 0.0) {
                    double d = (vx * f.NVel [0] + vy * f.NVel [1] + vz * f.NVel [2]) / nn;
                    vx -= f.NVel [0] * d;
                    vy -= f.NVel [1] * d;
                    vz -= f.NVel [2] * d;
                }
            }
            into [0] += vx * scale;
            into [1] += vy * scale;
            into [2] += vz * scale;
        }

        /// <summary>ModuleLiftingSurface.GetDragVector (4 arguments), added times `scale`.</summary>
        static void AddDrag (SurfaceFlow f, ISurfaceCurves curves, double absDot, double scale, double[] into)
        {
            if (f.NodeBlocked)
                return;
            double s = curves.Drag (absDot) * curves.DragMach (f.Mach);
            s *= f.LiftCoeff;
            if (s == 0.0 || double.IsNaN (s))
                return;
            s = f.QDrag * (s * f.DragMultiplier);
            into [0] += -f.NVel [0] * s * scale;
            into [1] += -f.NVel [1] * s * scale;
            into [2] += -f.NVel [2] * s * scale;
        }

        // Scratch (Unity main thread only, like every caller; the tests are single-threaded).
        static readonly double[] rotated = new double[3];
        static readonly double[] scratchLift = new double[3], scratchDrag = new double[3];
        static readonly double[] scratchTorque = new double[3];

        /// <summary>
        /// The lift and drag forces (N, world) the module computes at `deflection` (deg): steps 2
        /// and 5 above. rotate = false reproduces the step where the moving part is not turned
        /// (localRotation == neutral and not alwaysRecomputeLift).
        /// </summary>
        internal static void Forces (SurfaceFlow f, ISurfaceCurves curves, double deflection, bool rotate,
            double[] lift, double[] drag)
        {
            lift [0] = lift [1] = lift [2] = 0.0;
            drag [0] = drag [1] = drag [2] = 0.0;
            double liftDot = Dot (f.NVel, f.LiftVector0);
            double absDot = f.Omnidirectional ? Math.Abs (liftDot) : Clamp01 (liftDot);
            AddLift (f, curves, f.LiftVector0, liftDot, absDot, 1.0 - f.Area, lift);
            if (f.UseDrag)
                AddDrag (f, curves, absDot, 1.0 - f.Area, drag);
            double dot = liftDot, abs = absDot;
            double[] lv = f.LiftVector0;
            if (rotate) {
                Rotate (f.LiftVector0, f.Hinge, deflection, rotated);
                lv = rotated;
                dot = Dot (f.NVel, lv);
                abs = Math.Abs (dot);
            }
            AddLift (f, curves, lv, dot, abs, f.Area, lift);
            if (f.UseDrag)
                AddDrag (f, curves, abs, f.Area, drag);
        }

        /// <summary>Torque (N.m, world) about the CoM of a lift and a drag at their points.</summary>
        internal static void Torque (SurfaceFlow f, double[] lift, double[] drag, double[] torque)
        {
            double[] l = f.LeverLift, d = f.LeverDrag;
            torque [0] = l [1] * lift [2] - l [2] * lift [1] + d [1] * drag [2] - d [2] * drag [1];
            torque [1] = l [2] * lift [0] - l [0] * lift [2] + d [2] * drag [0] - d [0] * drag [2];
            torque [2] = l [0] * lift [1] - l [1] * lift [0] + d [0] * drag [1] - d [1] * drag [0];
        }

        /// <summary>The model torque (N.m, world) at `deflection` (deg), the moving part turned.</summary>
        internal static void TorqueAt (SurfaceFlow f, ISurfaceCurves curves, double deflection, double[] torque)
        {
            TorqueAt (f, curves, deflection, true, torque);
        }

        /// <summary>
        /// The model torque (N.m, world) at `deflection` (deg), the moving part turned or not as the
        /// module has it at that deflection (rotate = false below Unity's threshold).
        /// </summary>
        internal static void TorqueAt (SurfaceFlow f, ISurfaceCurves curves, double deflection, bool rotate,
            double[] torque)
        {
            Forces (f, curves, deflection, rotate, scratchLift, scratchDrag);
            Torque (f, scratchLift, scratchDrag, torque);
        }

        /// <summary>
        /// The torque step (N.m, world) between the moving part turned and not turned at
        /// `deflection` (deg): what a wrong guess of the module's turned state costs a prediction
        ///. Not merely column x deflection: a one-sided surface's unturned part keeps
        /// clamp01(liftDot) where the turned one uses |f|.
        /// </summary>
        internal static void TurnJump (SurfaceFlow f, ISurfaceCurves curves, double deflection, double[] jump)
        {
            TorqueAt (f, curves, deflection, true, jump);
            double x = jump [0], y = jump [1], z = jump [2];
            TorqueAt (f, curves, deflection, false, jump);
            jump [0] = x - jump [0];
            jump [1] = y - jump [1];
            jump [2] = z - jump [2];
        }

        /// <summary>
        /// Bound (deg) of what separates the replica's predicted deflection from the module's for a
        /// reason other than the response: the module's float command against the replica's
        /// double target, as measured in the step (|action - target|; a larger gap is a lag or a
        /// command-law defect, the response-lag rule's to judge), and one float spacing of the
        /// deflections (the module's MoveTowards in float). +Infinity when a term is unknown.
        /// </summary>
        internal static double DeflectionResolution (double action, double target, double previous,
            double predicted, double deflection)
        {
            double command = Math.Abs (action - target);
            double m = Math.Max (Math.Abs (previous), Math.Max (Math.Abs (predicted), Math.Abs (deflection)));
            double r = command + 2.0 * m * FloatSpacingBelowOne;
            return double.IsNaN (r) ? double.PositiveInfinity : r;
        }

        /// <summary>
        /// The exact column dM/d(deflection) (N.m per DEGREE, world) at `deflection`, by central
        /// differences of the same replica (+-ColumnStepDeg), the moving part turned.
        /// </summary>
        internal static void Column (SurfaceFlow f, ISurfaceCurves curves, double deflection, double[] column)
        {
            double h = ColumnStepDeg;
            TorqueAt (f, curves, deflection + h, scratchTorque);
            double px = scratchTorque [0], py = scratchTorque [1], pz = scratchTorque [2];
            TorqueAt (f, curves, deflection - h, scratchTorque);
            column [0] = (px - scratchTorque [0]) / (2.0 * h);
            column [1] = (py - scratchTorque [1]) / (2.0 * h);
            column [2] = (pz - scratchTorque [2]) / (2.0 * h);
        }

        /// <summary>
        /// Identity of the replica (plan 3.1, step 1): (|L_m - L_r| + |D_m - D_r|) / (|L_r| + |D_r|),
        /// NaN when the realized force is below floorN (nothing to judge).
        /// </summary>
        internal static double Identity (double[] liftModel, double[] dragModel, double[] liftReal,
            double[] dragReal, double floorN)
        {
            double dl = Norm (liftModel [0] - liftReal [0], liftModel [1] - liftReal [1], liftModel [2] - liftReal [2]);
            double dd = Norm (dragModel [0] - dragReal [0], dragModel [1] - dragReal [1], dragModel [2] - dragReal [2]);
            double r = Norm (liftReal [0], liftReal [1], liftReal [2]) + Norm (dragReal [0], dragReal [1], dragReal [2]);
            if (!(r >= floorN))
                return double.NaN;
            return (dl + dd) / r;
        }

        internal static double Norm (double x, double y, double z)
        {
            return Math.Sqrt (x * x + y * y + z * z);
        }

        // ------------------------------------------------------------- response lag

        /// <summary>
        /// Which ctrlState the module answered in this step (open point C 6.2: the parts'
        /// FixedUpdate against the fly-by-wire): 0 = this step's (same-step response), 1 = the
        /// previous step's, 2 = neither, -1 = the two targets do not tell them apart.
        /// </summary>
        internal static int LagOutcome (double action, double targetSame, double targetPrevious)
        {
            if (double.IsNaN (action) || double.IsNaN (targetSame) || double.IsNaN (targetPrevious))
                return -1;
            if (!(Math.Abs (targetSame - targetPrevious) > LagDistinctDeg))
                return -1;
            if (Math.Abs (action - targetSame) <= TargetMatchDeg)
                return 0;
            if (Math.Abs (action - targetPrevious) <= TargetMatchDeg)
                return 1;
            return 2;
        }

        /// <summary>The same-step response is established (the armed mode may engage the surfaces).</summary>
        internal static bool LagEstablished (long same, long previous, long neither)
        {
            return same >= LagMatchesRequired && same >= LagMatchesRatio * (previous + neither);
        }
    }

    /// <summary>
    /// The mono-flight scale guard of the surface family (plan 3.1, audit C 5.2 M2): per body axis,
    /// k = sum(dM_real dM_pred) / sum(dM_pred^2), exponential forgetting, updated only on steps
    /// predicted to the replica's precision. dM_pred is the model's increment of the surfaces'
    /// torque over the step at the step's flow (deflection predicted from the command law, minus
    /// the model at the previous deflection), dM_real the realized torque minus the same model at
    /// the previous deflection. A step counts on an axis only when its uncertainty U (the
    /// replica's measured mismatch at the realized state, the float resolution of the deflections,
    /// and the turned-state jump of every surface whose step moves it inside the rotation band) is
    /// below IdentityTolerance x |dM_pred|: a weakly excited axis, or a step
    /// whose increment is a jump at Unity's threshold, adds no evidence. A GUARD, never an
    /// adaptation: B is not scaled by k. Reset to k = 1 with no evidence at every arming
    /// (INTER-FLIGHT IDENTIFICATION FORBIDDEN). K = 1 without evidence is not a proof; the
    /// armed mode engages a surface only once every axis its column contributes to is Proven
    /// (ScaleEvidenceSteps counted steps, not tripped).
    /// </summary>
    internal sealed class SurfaceScaleGuard
    {
        internal readonly double[] Num = new double[3];
        internal readonly double[] Den = new double[3];
        internal readonly int[] Steps = new int[3];
        internal int Tripped;   // bits SurfaceModel.SanctionScaleX/Y/Z, sticky for the arming

        internal void Reset ()
        {
            for (int a = 0; a < 3; a++) {
                Num [a] = 0.0;
                Den [a] = 0.0;
                Steps [a] = 0;
            }
            Tripped = 0;
        }

        /// <summary>k of axis a (1 without evidence).</summary>
        internal double K (int a)
        {
            return Den [a] > 0.0 ? Num [a] / Den [a] : 1.0;
        }

        /// <summary>
        /// the axes (bits 0-2) with the counted steps the guard needs to judge them
        /// (ScaleEvidenceSteps, the same count from which it may trip), tripped or not.
        /// </summary>
        internal int Judged ()
        {
            int axes = 0;
            for (int a = 0; a < 3; a++)
                if (Steps [a] >= SurfaceModel.ScaleEvidenceSteps)
                    axes |= 1 << a;
            return axes;
        }

        /// <summary>the axes whose scale is proven in this arming: judged and not tripped.</summary>
        internal int Proven ()
        {
            return Judged () & ~Tripped & 7;
        }

        /// <summary>
        /// One step: predicted and realized increments (N.m, body), and the uncertainty of each
        /// axis (N.m): the axis counts the step when its uncertainty is below IdentityTolerance x
        /// |predicted|. Returns the tripped mask after the step.
        /// </summary>
        internal int Update (double[] predicted, double[] realized, double[] uncertainty)
        {
            for (int a = 0; a < 3; a++) {
                double p = predicted [a], r = realized [a];
                if (double.IsNaN (p) || double.IsNaN (r) || double.IsInfinity (p) || double.IsInfinity (r))
                    continue;
                if (!(uncertainty [a] < SurfaceModel.IdentityTolerance * Math.Abs (p)))
                    continue;
                Num [a] = SurfaceModel.ScaleForgetting * Num [a] + r * p;
                Den [a] = SurfaceModel.ScaleForgetting * Den [a] + p * p;
                Steps [a]++;
                if (Steps [a] >= SurfaceModel.ScaleEvidenceSteps) {
                    double k = K (a);
                    if (!(k >= SurfaceModel.ScaleMin && k <= SurfaceModel.ScaleMax))
                        Tripped |= 1 << a;
                }
            }
            return Tripped;
        }
    }
}
