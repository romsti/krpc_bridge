using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using System.Reflection.Emit;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;
using UnityEngine;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, lot 3 capacity 3.1: SurfaceModelV1, the control surfaces as a MODELED actuator.
    /// The replica is pure (Model/SurfaceModel.cs); this file reads the game into it and wires it
    /// into the loop.
    ///
    /// BORN CLOSED: AttitudeSurfacesV1 (mode 1 shadow, 2 armed) sets it for one arming; every
    /// arming resets it to 0 (and the scale guard to k = 1 with no evidence: mono-flight only),
    /// and with 0 nothing below runs: every frame, return and write is the build before's.
    ///
    /// When it runs (KSP 1.12.5 order, decompiled; assembly inspection):
    ///   * Earlyish (the attitude tick, before the parts): part.machNumber of each modeled surface
    ///     is kept -- the module reads it in its FixedUpdate, and FlightIntegrator rewrites it after
    ///     the parts; in the armed mode, the last capture is turned into the law's inputs.
    ///   * BetterLateThanNever (after every part's FixedUpdate, before the physics integration; the
    ///     transforms have not moved since the module ran): the CAPTURE. For each surface, the
    ///     module's own fields of this step (deflection, action, nVel, Qlift, Qdrag, liftForce,
    ///     dragForce) against the replica fed with the same inputs:
    ///       - identity of the force: |F_model - F_real| / |F_real| (plan 3.1 step 1, below 1e-3);
    ///       - the command law: the module's target (action) against the replica's target from this
    ///         step's ctrlState AND from the previous step's (which one the surface answered is the
    ///         open point C 6.2; counted, the armed mode needs the same-step answer established). A
    ///         surface held neutral answers nothing (its target is 0), so with the loop armed the
    ///         fly-by-wire callback also notes each module's liftForce: equal to this step's capture,
    ///         the module ran BEFORE the callback (it read the previous step's ctrlState); equal to
    ///         the previous capture while the force moved, AFTER it (same step);
    ///       - the realized torque about the CoM (lift at CoLOffset, drag at CoPOffset), the exact
    ///         column dM/d(deflection) (central differences of the same replica), the command gain
    ///         d(target)/d(ctrlState), body frame;
    ///       - the mono-flight scale guard per body axis: the realized torque increment of the step
    ///         against the model's predicted one, both at this step's flow, counted only when the
    ///         step is predicted to the replica's precision (its uncertainty, the turned-state
    ///         jump of a surface in Unity's rotation band included, below IdentityTolerance of it).
    ///     Captured BEFORE the lot-1 restore of deflectionDirection (same callback), so a surface
    ///     held neutral in this step is known as such.
    ///
    /// Shadow (1): computed and published (SurfaceModelV1), nothing in the law, nothing written.
    /// Armed (2): implies the family mechanics of option 1. While the family is engageable (the
    /// same-step response established, no scale axis out of [0.5, 2], the surface's replica within
    /// tolerance, captured in the previous step, not shielded), each such surface whose column
    /// contributes only to body axes whose scale is PROVEN in this arming (the guard's
    /// ScaleEvidenceSteps counted steps, not tripped; SurfaceModel.ContributedAxes) is ENGAGED: it
    /// leaves the neutralization and the roll gate of option 1, its exact sub-columns (column x
    /// gain, per ctrlState axis) are added to the ctrlState columns of the WLS, its realized torque
    /// to the law's M, and the torque it will add anyway while slewing toward the target of the
    /// served command (M_free) is taken off the allocation target (AttitudeMath.UseTorqueFree).
    /// Every other surface stays neutralized exactly as with option 1. The guard counts its steps
    /// whether a surface is engaged or not (shadow, neutralized slews, steps the AutoPilot flies
    /// them): the proof is gathered before the engagement. The surfaces share the
    /// ctrlState variable with the RCS and the wheels (FixedCtrlSurfaceUpdate reads it): their
    /// columns are separate SUB-columns of that variable, as the effector registry's RCS and wheel
    /// ones, with their own status, guard and publication.
    ///
    /// (born closed, mode flag 4, SurfaceModel.ModeFlagNewtonMetres): the replica works in
    /// KSP's own force unit, the kilonewton (the module's liftForce/dragForce: Qlift is in Pa, the
    /// force goes to the part's rigidbody, whose mass is in tonnes). Every other torque of the law
    /// is converted to N.m (engines, RCS, wheels: x 1000); without the flag the surfaces' realized
    /// torque, sub-columns, free torque, authority and residual input entered the law in kN.m,
    /// 1000 times too small (a recorded flight, the measured angular
    /// acceleration is RCS + wheel + 870-1030 x the replica's fin torque; the loop's roll, written
    /// in the coast once the fins were engaged, went into a saturated 3 Hz cycle). With the flag
    /// they enter the law -- and the publication: torques, columns and authority in N.m, forces in
    /// N -- multiplied by SurfaceModel.LawTorqueScale. The replica, its identity and its scale guard
    /// are unit-free and stay in KSP's unit. Without the flag nothing changes, to the bit.
    ///
    /// (born closed, mode flag 8, SurfaceModel.ModeFlagKappa): the PHYSICAL validation that
    /// would have caught an earlier change in the first shadow flight (Model/SurfaceKappa.cs). The capture adds,
    /// per step, the deflection-driven torque increment of the modeled surfaces (the realized
    /// torque minus the replica at the previous deflection and this step's flow, body, double) and
    /// the whole vessel's aerodynamic torque as the game applied it (every part's cube drag at its
    /// CoP, body lift, every lifting surface's forces, the rigidbody angular drag: kRPC's
    /// Flight.AerodynamicTorque, read the same way); the attitude tick regresses the measured
    /// I x d(omega_dot) minus the engines', RCS' and wheels' realized increments on them, per body
    /// axis, through the law's own H(z). kappa is published (surface_kappa_v1); armed, a surface is
    /// engaged only when, besides an earlier change, every axis its column contributes to has kappa PROVEN
    /// within the proof band [0.707, 1.414] (sanction bits 9-11 while not proven), and an
    /// axis found out of tolerance keeps every surface acting on it out for the arming (bits
    /// 12-14). The measured side includes the gyroscopic torque omega x I omega (the body
    /// rate is passed with the measurement), and out of tolerance is judged on the jackknife's
    /// own degrees of freedom. Without the flag nothing changes, to the bit.
    ///
    /// (born closed, mode flag 16 with 8, SurfaceModel.ModeFlagKappaLeak): the estimator counts
    /// a step on an axis only while the surfaces carry at least 2/3 of the excited torque increment there
    /// (Model/SurfaceKappa.cs, the leak gate; the realized increments of the engines', RCS' and wheels'
    /// torque against the surfaces' deflection-driven one, through the same H(z)). Switching it starts a
    /// fresh estimator. The roll excitation of an earlier change counts as counted only the roll the gate let through.
    /// The leak slope of the counted steps (leak_beta) and the refused steps are published whatever the
    /// flag (surface_kappa_v1 schema 3). Without the flag nothing else changes, to the bit.
    ///
    /// (born closed, mode flag 32 with 8, SurfaceModel.ModeFlagKappaInertia): the estimator's measured
    /// side multiplies by the PHYSICAL inertia, Vessel.MOI plus the parallel-axis terms its stock sum leaves
    /// out: it takes each rigidbody's mass at rb.position, the physics at rb.worldCenterOfMass (the part's
    /// CoMOffset; Model/SurfaceKappa.cs, AddInertiaOffset). The offset is computed at every attitude tick of
    /// kappa from the vessel's parts and published per axis whatever the flag (surface_kappa_v1 schema 4);
    /// switching the flag starts a fresh estimator. The law's own inertia is untouched. Without the flag
    /// nothing else changes, to the bit.
    ///
    /// (armed only): a surface is engaged only when the loop OWNS every axis it needs -- each
    /// body axis its column contributes to and each ctrlState axis its command answers
    /// (SurfaceModel.EngagementAxes; bit 0 pitch x, 1 roll y, 2 yaw z). Owned (AttitudeWrite.
    /// LoopOwnedAxes): written by the loop's own fly-by-wire link at the previous physics step and
    /// still in its arm and write masks now. The decision cannot wait for this step's own write
    /// decision, which comes after the law the engagement feeds (the order of TickEntry: engagement,
    /// columns and M, the law, then the writes); a surface is so never engaged before the loop has
    /// written its axes, and a write the loop refuses at the same step costs that one physics step
    /// at most (the surface moves at most actuatorSpeed x dt toward the other regulator's command),
    /// disengaged from the next. a recorded flight: engaged through the coast while
    /// the kRPC AutoPilot wrote pitch/yaw and the loop the roll only -- the roll gate of option 1
    /// lifted by the engagement, the law counting as its own the fins' torque the AutoPilot
    /// commanded. Out, a surface is stock again (the roll gate and the neutralization of option 1
    /// apply), and the coast is the shadow's. Sanction bits 15-17 say which axes are not owned. At
    /// an engagement or a disengagement the law's M changes definition: its torque filter is moved
    /// by the change (AttitudeLawState.ShiftTorque), so the allocation target takes no step.
    ///
    /// No allocation per physics step: the records, their arrays and the curve adapters are built
    /// when the vessel's layout changes; the protected fields are read by compiled getters
    /// (DynamicMethod), and fall back to FieldInfo (boxing) only if the runtime refuses them.
    /// </summary>
    internal static partial class AttitudeV1
    {
        // Capability bit of the frame header (offset 23) and informational motifs (offset 12),
        // set only while SurfaceModelV1 is on.
        internal const long CapSurfaceModel = 1L << 15;
        internal const long MotifSurfacesEngaged = 1L << 37;
        internal const long MotifSurfaceSanction = 1L << 38;

        // ---------------------------------------------------------------- state

        /// <summary>The curves of one module, read live (a mod may swap them).</summary>
        internal sealed class SurfaceCurves : ISurfaceCurves
        {
            internal ModuleControlSurface Module;

            public double Lift (double absDot)
            {
                var c = Module.liftCurve;
                return c == null ? 0.0 : c.Evaluate ((float)absDot);
            }

            public double LiftMach (double mach)
            {
                var c = Module.liftMachCurve;
                return c == null ? 0.0 : c.Evaluate ((float)mach);
            }

            public double Drag (double absDot)
            {
                var c = Module.dragCurve;
                return c == null ? 0.0 : c.Evaluate ((float)absDot);
            }

            public double DragMach (double mach)
            {
                var c = Module.dragMachCurve;
                return c == null ? 0.0 : c.Evaluate ((float)mach);
            }
        }

        /// <summary>One surface of an armed vessel: its constant classification and its last capture.</summary>
        internal sealed class SurfaceRecord
        {
            internal ModuleControlSurface Module;
            internal Part Part;
            internal uint PartPersistentId;
            internal int ModuleIndex;
            internal int StaticReasons;          // classification that does not move in a layout
            internal int Reasons;                // this capture's (static + dynamic + sanctions)
            internal int Sanctions;              // sticky for the arming (identity)
            internal int Axes;
            internal bool Engaged;               // this step (decided at Earlyish)
            // what the law's M carried for this surface at the last tick (engaged then), law unit.
            internal bool InLaw;
            internal readonly double[] LawTorque = new double[3];
            internal int EngagementAxes;         // the axes the loop must own (last capture)
            internal bool Neutralized;           // held neutral in the captured step
            internal bool Shielded;
            internal readonly SurfaceCurves Curves = new SurfaceCurves ();
            internal readonly SurfaceFlow Flow = new SurfaceFlow ();
            internal readonly SurfaceCommand Command = new SurfaceCommand ();       // as the step saw it
            internal readonly SurfaceCommand Nominal = new SurfaceCommand ();       // deflectionDirection restored
            internal double MachEarlyish = double.NaN;
            // The fly-by-wire probe: liftForce when our callback ran, and the previous capture's.
            internal readonly double[] FbwLift = new double[3];
            internal long FbwTick = -1;
            internal readonly double[] LiftPrevious = new double[3];
            internal bool HaveLiftPrevious;
            internal bool MachFromEarlyish;
            internal bool MachUsedFromEarlyish;
            // Last capture.
            internal long CaptureTick = -1;
            internal bool HavePrevious;
            internal double Deflection = double.NaN, DeflectionPrevious = double.NaN;   // deg
            internal double Action = double.NaN;
            internal double TargetSame = double.NaN, TargetPrevious = double.NaN, TargetNominal = double.NaN;
            internal double PredictedSame = double.NaN, PredictedPrevious = double.NaN;
            internal int LagOutcome = -1;
            internal bool Rotated;
            internal double Identity = double.NaN;
            internal int IdentityStrikes;
            internal readonly double[] Lift = new double[3], Drag = new double[3];             // realized, world
            internal readonly double[] LiftModel = new double[3], DragModel = new double[3];   // replica, world
            internal readonly double[] ForceBody = new double[3], ForceModelBody = new double[3];
            internal readonly double[] TorqueBody = new double[3];        // realized, N.m
            internal readonly double[] ColumnBody = new double[3];        // N.m per degree
            internal readonly double[] Gain = new double[3];              // deg per unit (pitch, roll, yaw)
            internal double Exact { get { return Reasons == 0 ? 1.0 : 0.0; } }
        }

        /// <summary>SurfaceModelV1 of one armed vessel. Reset at every arming (born closed, mono-flight).</summary>
        internal sealed class SurfaceState
        {
            internal int Mode;                   // 0 off, 1 shadow, 2 armed (without the flags)
            internal bool NewtonMetres;          // flag: the law and the publication in N.m
            internal double LawScale = 1.0;      // SurfaceModel.LawTorqueScale (NewtonMetres)
            // flag (SurfaceModel.ModeFlagKappa): the physical slope kappa, published and, armed,
            // a condition of every engagement. Its estimator and the capture's increments.
            internal bool Kappa;
            // flag (SurfaceModel.ModeFlagKappaLeak, with the kappa flag only): the leak gate of
            // the estimator's counted steps.
            internal bool KappaLeak;
            // flag (SurfaceModel.ModeFlagKappaInertia, with the kappa flag only): the measured side on
            // the physical inertia; the offset (kg.m^2, body) Vessel.MOI leaves out, the tick's, and the
            // inertia the estimator was given.
            internal bool KappaInertia;
            internal readonly double[] KappaInertiaOffset = { double.NaN, double.NaN, double.NaN };
            internal readonly double[] KappaInertiaUsed = new double[3];
            internal readonly SurfaceKappaEstimator KappaEstimator = new SurfaceKappaEstimator ();
            internal readonly double[] KappaDeflection = new double[3];   // last capture, kN.m, body
            internal readonly double[] KappaAero = new double[3];         // last capture, kN.m, body
            internal readonly double[] KappaNuisance = new double[3];     // last capture, N.m, body
            internal readonly double[] KappaDeflectionLaw = new double[3]; // the tick's, law unit
            internal long KappaAeroTick = -1;          // capture tick of KappaAero (valid)
            internal long KappaIncrementTick = -1;     // capture tick whose increments are valid
            internal int KappaAxes;                     // the tick's axes the surfaces can act on
            internal readonly double[] KappaNonFin = new double[3];        // the tick's, N.m, body
            internal bool KappaEngine;
            internal bool KappaCounted;                 // the estimator counted this tick's step
            // born closed: the roll excitation (window, plan, command of the step), and what
            // the surfaces answer a unit of ctrlState.roll with at this tick (SurfacesColumns).
            internal readonly SurfaceKappaRollState Roll = new SurfaceKappaRollState ();
            internal double RollPerCommand;             // the law's unit per unit of roll, body y
            internal double RollGainMax;                // the steepest |d(target)/d(roll)|, deg per unit
            internal double RollSpeedMin = double.NaN;  // the slowest actuator among them, deg/s
            internal bool RollExponential;              // one of them has an exponential actuator
            // The lifting modules of the vessel (layout cache).
            internal readonly List<ModuleLiftingSurface> KappaLifting = new List<ModuleLiftingSurface> ();
            internal long KappaGeneration = -1;
            internal int KappaParts;
            internal double KappaCostLast = double.NaN, KappaCostMax = double.NaN;
            internal long Generation = -1;
            internal readonly List<SurfaceRecord> Records = new List<SurfaceRecord> ();
            internal long CaptureTick = -1;
            internal double CaptureUt = double.NaN;
            internal double CaptureDt = double.NaN;
            internal long CaptureMasterTick = -1;
            internal readonly double[] Ctrl = new double[3];           // ctrlState of the captured step
            internal readonly double[] CtrlPrevious = new double[3];
            internal bool HaveCtrlPrevious;
            internal long LagSame, LagPrevious, LagNeither;
            internal long ProbeSame, ProbePrevious;     // the fly-by-wire probe's answers
            internal readonly SurfaceScaleGuard Guard = new SurfaceScaleGuard ();
            internal double IdentityMax = double.NaN;
            internal int IdentityJudged;
            internal int Modeled, Engaged, NeutralizedCount;
            internal int Sanctions;
            internal long EarlyishTick = -1;
            // the axes the loop owned for this tick's engagement (AttitudeWrite.LoopOwnedAxes),
            // and the change of definition of the law's M this tick (surfaces entering minus leaving,
            // law unit), which AttitudeLawState.ShiftTorque absorbs; its opposite for the published
            // residual's own filter (the surfaces outside M).
            internal int OwnedAxes;
            internal readonly double[] LawShift = new double[3];
            internal readonly double[] PublishedShift = new double[3];
            internal bool HaveShift;
            internal long EngagementChanges;      // engagements and disengagements in the arming
            internal readonly double[] TorqueBody = new double[3];      // modeled surfaces, last capture
            internal readonly double[] TorqueFree = new double[3];      // last Earlyish
            internal readonly double[] Authority = new double[3];
            internal readonly double[] ResidualLaw = new double[3];
            internal readonly double[] ResidualSurfaces = new double[3];
            internal readonly double[] ResidualSurfacesF = new double[3];
            internal bool HaveResidualF;
            internal readonly Biquad3 Filter = new Biquad3 ();
            internal readonly double[] FilterIn = new double[3], FilterOut = new double[3];
            internal readonly double[] FilterPrevious = new double[3];
            internal bool HaveFilterPrevious;
            // Scale guard scratch.
            internal readonly double[] Predicted = new double[3], Realized = new double[3];
            internal readonly double[] Uncertainty = new double[3];
            // diagnostic (KSP.log only): captures of a surface inside the rotation band, and
            // how many of them the bare rotation AngleAxis(deflection) * neutral tests otherwise
            // than the rotation the module stored.
            internal long BandCaptures, BareDisagreements;

            /// <summary>The mode as the wire says it: the base mode and its flags.</summary>
            internal int WireMode {
                get {
                    return Mode | (NewtonMetres ? SurfaceModel.ModeFlagNewtonMetres : 0) |
                        (Kappa ? SurfaceModel.ModeFlagKappa : 0) | (Kappa && KappaLeak ? SurfaceModel.ModeFlagKappaLeak : 0) |
                        (Kappa && KappaInertia ? SurfaceModel.ModeFlagKappaInertia : 0);
                }
            }

            /// <summary>a fresh estimator (every arming, model on, unit or flag change).</summary>
            internal void ResetKappa ()
            {
                KappaEstimator.Reset ();
                KappaAeroTick = -1;
                KappaIncrementTick = -1;
                KappaEngine = false;
                KappaCounted = false;
                KappaAxes = 0;
                KappaGeneration = -1;
                // a fresh estimator, a closed window (the client reopens it).
                Roll.Reset ();
                RollPerCommand = 0.0;
                RollGainMax = 0.0;
                RollSpeedMin = double.NaN;
                RollExponential = false;
                KappaLifting.Clear ();
                KappaParts = 0;
                KappaCostLast = KappaCostMax = double.NaN;
                for (int a = 0; a < 3; a++) {
                    KappaDeflection [a] = KappaAero [a] = KappaNuisance [a] = 0.0;
                    KappaDeflectionLaw [a] = KappaNonFin [a] = 0.0;
                    KappaInertiaOffset [a] = double.NaN;    // computed again at the next tick
                    KappaInertiaUsed [a] = 0.0;
                }
            }

            internal void Reset ()
            {
                Mode = SurfaceModel.ModeOff;
                NewtonMetres = false;
                LawScale = 1.0;
                Kappa = false;
                KappaLeak = false;
                KappaInertia = false;
                ResetKappa ();
                Generation = -1;
                Records.Clear ();
                CaptureTick = -1;
                CaptureUt = double.NaN;
                CaptureDt = double.NaN;
                CaptureMasterTick = -1;
                HaveCtrlPrevious = false;
                LagSame = LagPrevious = LagNeither = 0;
                ProbeSame = ProbePrevious = 0;
                Guard.Reset ();
                BandCaptures = BareDisagreements = 0;
                IdentityMax = double.NaN;
                IdentityJudged = 0;
                Modeled = Engaged = NeutralizedCount = 0;
                Sanctions = 0;
                EarlyishTick = -1;
                OwnedAxes = 0;
                HaveShift = false;
                EngagementChanges = 0;
                for (int a = 0; a < 3; a++) {
                    LawShift [a] = 0.0;
                    PublishedShift [a] = 0.0;
                    TorqueBody [a] = 0.0;
                    TorqueFree [a] = 0.0;
                    Authority [a] = 0.0;
                    ResidualLaw [a] = double.NaN;
                    ResidualSurfaces [a] = double.NaN;
                    ResidualSurfacesF [a] = double.NaN;
                }
                HaveResidualF = false;
                Filter.Reset ();
                HaveFilterPrevious = false;
            }
        }

        // ---------------------------------------------------------------- reflection

        // KSP 1.12.5 members (decompiled Assembly-CSharp sha256 d9e42483..., assembly inspection
        // get_member / ilspycmd): ModuleControlSurface: protected float deflection, action,
        // deflectionDirection; protected Transform ctrlSurface; protected Quaternion neutral;
        // protected List<AdjusterControlSurfaceBase> adjusterCache. ModuleLiftingSurface: protected
        // Vector3 nVel; protected double Qlift, Qdrag; protected Transform baseTransform; private
        // List<AdjusterLiftingSurfaceBase> adjusterCache.
        static bool surfaceReflectionResolved;
        static Func<ModuleControlSurface, float> getDeflection, getAction, getDirection;
        static Func<ModuleControlSurface, Vector3> getNVel;
        static Func<ModuleControlSurface, double> getQLift, getQDrag;
        static Func<ModuleControlSurface, Transform> getBaseTransform, getCtrlSurface;
        static Func<ModuleControlSurface, Quaternion> getNeutral;
        static FieldInfo controlAdjusters, liftAdjusters;
        static int slowGetters;

        static void ResolveSurfaceReflection ()
        {
            if (surfaceReflectionResolved)
                return;
            surfaceReflectionResolved = true;
            var mcs = typeof (ModuleControlSurface);
            var mls = typeof (ModuleLiftingSurface);
            getDeflection = Getter<float> (mcs, "deflection");
            getAction = Getter<float> (mcs, "action");
            getDirection = Getter<float> (mcs, "deflectionDirection");
            getCtrlSurface = Getter<Transform> (mcs, "ctrlSurface");
            getNeutral = Getter<Quaternion> (mcs, "neutral");
            getNVel = Getter<Vector3> (mls, "nVel");
            getQLift = Getter<double> (mls, "Qlift");
            getQDrag = Getter<double> (mls, "Qdrag");
            getBaseTransform = Getter<Transform> (mls, "baseTransform");
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic |
                BindingFlags.DeclaredOnly;
            try {
                controlAdjusters = mcs.GetField ("adjusterCache", flags);
                liftAdjusters = mls.GetField ("adjusterCache", flags);
            } catch {
                controlAdjusters = null;
                liftAdjusters = null;
            }
        }

        static bool SurfaceReflectionComplete ()
        {
            ResolveSurfaceReflection ();
            return getDeflection != null && getAction != null && getDirection != null &&
                getCtrlSurface != null && getNeutral != null && getNVel != null && getQLift != null &&
                getQDrag != null && getBaseTransform != null;
        }

        /// <summary>
        /// A typed reader of an instance field declared on `declaring` (public or not), compiled
        /// once (DynamicMethod: no boxing per read); a FieldInfo reader if the runtime refuses it;
        /// null if the field is missing or of another type.
        /// </summary>
        static Func<ModuleControlSurface, T> Getter<T> (Type declaring, string name)
        {
            FieldInfo field;
            try {
                field = declaring.GetField (name, BindingFlags.Instance | BindingFlags.Public |
                    BindingFlags.NonPublic | BindingFlags.DeclaredOnly);
            } catch {
                return null;
            }
            if (field == null || field.FieldType != typeof (T))
                return null;
            try {
                var method = new DynamicMethod ("bridge_get_" + name, typeof (T),
                    new[] { typeof (ModuleControlSurface) }, typeof (ModuleControlSurface), true);
                var il = method.GetILGenerator ();
                il.Emit (OpCodes.Ldarg_0);
                il.Emit (OpCodes.Ldfld, field);
                il.Emit (OpCodes.Ret);
                return (Func<ModuleControlSurface, T>)method.CreateDelegate (typeof (Func<ModuleControlSurface, T>));
            } catch (Exception exc) {
                slowGetters++;
                BridgeLog.Info ("SurfaceModelV1: compiled reader of " + name + " refused (" + exc.Message +
                    "), FieldInfo used");
                return m => (T)field.GetValue (m);
            }
        }

        static int AdjusterCount (FieldInfo field, ModuleControlSurface module)
        {
            if (field == null)
                return 0;
            var list = field.GetValue (module) as ICollection;
            return list == null ? 0 : list.Count;
        }

        // ---------------------------------------------------------------- RPC bodies

        /// <summary>
        /// Sets SurfaceModelV1 of an armed vessel: 0 off, 1 shadow, 2 armed; 1 or 2 ORed with 4
        /// (SurfaceModel.ModeFlagNewtonMetres), the surfaces' torques in N.m in the law and the
        /// publication; ORed with 8 (SurfaceModel.ModeFlagKappa), the physical slope kappa
        /// estimated and published, and, armed, required on every axis an engaged surface acts on;
        /// with 8, ORed with 16 (SurfaceModel.ModeFlagKappaLeak), the leak gate of its counted steps;
        /// with 8, ORed with 32 (SurfaceModel.ModeFlagKappaInertia), its measured side on the physical inertia.
        /// Returns [accepted (1), protocol, mode now (with its flags), surfaces answering
        /// a ctrlState axis, of which modeled (exact class, members found), attitude tick]; the counts
        /// are NaN while the vessel is not loaded.
        /// </summary>
        internal static IList<double> SurfacesConfig (string token, int mode)
        {
            if (!SurfaceModel.ValidMode (mode))
                throw new ArgumentOutOfRangeException (nameof (mode), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "0 off, 1 shadow, 2 armed (ORed with 4 N.m, 8 kappa, 16 leak gate with 8, " +
                    "32 physical inertia with 8)"));
            var entry = RequireToken (token);
            var sm = entry.Sm;
            int baseMode = SurfaceModel.BaseMode (mode);
            if (baseMode == SurfaceModel.ModeOff) {
                // Option 1 (if any) keeps its own neutralization; the surfaces engaged by the armed
                // mode were never held, so nothing to put back. They leave the law's M.
                SurfacesLeaveLaw (entry);
                sm.Reset ();
            } else {
                if (sm.Mode == SurfaceModel.ModeOff)
                    sm.Reset ();
                bool newtonMetres = (mode & SurfaceModel.ModeFlagNewtonMetres) != 0;
                bool kappa = (mode & SurfaceModel.ModeFlagKappa) != 0;
                bool leak = kappa && (mode & SurfaceModel.ModeFlagKappaLeak) != 0;
                bool inertia = kappa && (mode & SurfaceModel.ModeFlagKappaInertia) != 0;
                if (newtonMetres != sm.NewtonMetres) {
                    // the law's M changes unit for the engaged surfaces: they leave it in the
                    // old one, and enter again in the new one at the next tick.
                    SurfacesLeaveLaw (entry);
                    // the residual filter holds torques of the other unit.
                    sm.Filter.Reset ();
                    sm.HaveFilterPrevious = false;
                    sm.HaveResidualF = false;
                }
                // kappa is the slope on the law's unit: a fresh estimator when it is switched
                // on or the unit changes (a mode change within the same flags keeps its evidence).
                // and when the leak gate changes (the evidence it counts changes). And when
                // the inertia of the measured side changes.
                if (kappa && (!sm.Kappa || newtonMetres != sm.NewtonMetres || leak != sm.KappaLeak ||
                        inertia != sm.KappaInertia))
                    sm.ResetKappa ();
                sm.Mode = baseMode;
                sm.NewtonMetres = newtonMetres;
                sm.Kappa = kappa;
                sm.KappaLeak = leak;
                sm.KappaInertia = inertia;
                sm.LawScale = SurfaceModel.LawTorqueScale (newtonMetres);
                RegisterLate ();
            }
            double answering = double.NaN, modeled = double.NaN;
            var vessel = Resolve (entry);
            if (vessel != null) {
                answering = modeled = 0.0;
                bool complete = SurfaceReflectionComplete ();
                for (int i = 0; i < entry.Surfaces.Count; i++) {
                    var s = entry.Surfaces [i];
                    if (s == null || AttitudeGeometry.SurfaceAnsweredAxes (s) == 0)
                        continue;
                    answering++;
                    if (complete && StaticReasons (s) == 0)
                        modeled++;
                }
            }
            return new List<double> (SurfaceModel.ConfigReplyLength) {
                1.0, SurfaceModel.Protocol, sm.WireMode, answering, modeled, ownTick
            };
        }

        static readonly SurfaceState emptySurfaceState = new SurfaceState ();

        /// <summary>The SurfaceModelV1 table of the last captured step (same layout when off or absent).</summary>
        internal static IList<double> SurfacesRead (string persistentId)
        {
            uint pid = VesselLookup.ParsePersistentId (persistentId);
            Entry entry;
            if (!entries.TryGetValue (pid, out entry))
                return SurfaceReply (null, emptySurfaceState);
            return SurfaceReply (entry, entry.Sm);
        }

        static List<double> SurfaceReply (Entry e, SurfaceState sm)
        {
            bool on = e != null && sm.Mode != SurfaceModel.ModeOff && sm.CaptureTick >= 0;
            int rows = on ? sm.Records.Count : 0;
            var output = new List<double> (SurfaceModel.HeaderStride + rows * SurfaceModel.RowStride +
                SurfaceModel.ClockStride);
            output.Add (SurfaceModel.Protocol);
            output.Add (SurfaceModel.Schema);
            output.Add (on ? 1.0 : 0.0);
            output.Add (e == null ? double.NaN : sm.WireMode);
            output.Add (on ? sm.CaptureTick : double.NaN);
            output.Add (on ? sm.CaptureUt : double.NaN);
            output.Add (on ? sm.CaptureDt : double.NaN);
            output.Add (e == null ? double.NaN : e.PersistentId);
            output.Add (rows);
            output.Add (SurfaceModel.RowStride);
            output.Add (on ? sm.Modeled : double.NaN);
            output.Add (on ? sm.Engaged : double.NaN);
            output.Add (on ? sm.NeutralizedCount : double.NaN);
            output.Add (on ? sm.LagSame : double.NaN);
            output.Add (on ? sm.LagPrevious : double.NaN);
            output.Add (on ? sm.LagNeither : double.NaN);
            output.Add (on ? (LagEstablished (sm) ? 1.0 : 0.0) : double.NaN);
            output.Add (on ? sm.IdentityMax : double.NaN);
            output.Add (on ? sm.IdentityJudged : double.NaN);
            output.Add (on ? sm.Sanctions : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.Guard.K (a) : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.Guard.Steps [a] : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.TorqueBody [a] * sm.LawScale : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.TorqueFree [a] : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.ResidualLaw [a] : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.ResidualSurfaces [a] : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.ResidualSurfacesF [a] : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.Authority [a] : double.NaN);
            for (int a = 0; a < 3; a++)
                output.Add (on ? sm.Ctrl [a] : double.NaN);
            output.Add (on ? sm.EarlyishTick : double.NaN);
            output.Add (on ? sm.ProbeSame : double.NaN);
            output.Add (on ? sm.ProbePrevious : double.NaN);
            for (int i = 0; i < rows; i++)
                SurfaceRow (sm.Records [i], output, sm.LawScale);
            output.Add (on ? sm.CaptureMasterTick : double.NaN);
            return output;
        }

        /// <summary>One surface's row; forces, torques and the column times `scale` (1000, in N and N.m).</summary>
        static void SurfaceRow (SurfaceRecord r, List<double> output, double scale)
        {
            const double rad = Math.PI / 180.0;
            double degPerRad = 180.0 / Math.PI;
            output.Add (r.PartPersistentId);
            output.Add (r.ModuleIndex);
            output.Add (r.Reasons == 0 ? SurfaceModel.StatusExact : SurfaceModel.StatusUnmodeled);
            output.Add (r.Reasons);
            output.Add (r.Axes);
            output.Add (r.Engaged ? 1.0 : 0.0);
            output.Add (r.Neutralized ? 1.0 : 0.0);
            output.Add (r.Shielded ? 1.0 : 0.0);
            output.Add (r.Deflection * rad);
            output.Add (r.DeflectionPrevious * rad);
            output.Add (r.Action * rad);
            output.Add (r.TargetSame * rad);
            output.Add (r.TargetPrevious * rad);
            output.Add (r.TargetNominal * rad);
            output.Add (r.PredictedSame * rad);
            output.Add (r.PredictedPrevious * rad);
            output.Add (r.LagOutcome);
            output.Add (r.Command.Speed * rad);
            output.Add (r.Command.Range * rad);
            output.Add (r.Command.Authority);
            output.Add (r.Command.Direction);
            output.Add (r.Flow.QLift);
            output.Add (r.Flow.QDrag);
            output.Add (r.Flow.Mach);
            output.Add (r.Rotated ? 1.0 : 0.0);
            for (int a = 0; a < 3; a++)
                output.Add (r.ForceBody [a] * scale);
            for (int a = 0; a < 3; a++)
                output.Add (r.ForceModelBody [a] * scale);
            output.Add (r.Identity);
            for (int a = 0; a < 3; a++)
                output.Add (r.TorqueBody [a] * scale);
            for (int a = 0; a < 3; a++)
                output.Add (r.ColumnBody [a] * degPerRad * scale);
            for (int a = 0; a < 3; a++)
                output.Add (r.Gain [a] * rad);
            output.Add (r.IdentityStrikes);
            output.Add (r.MachUsedFromEarlyish ? 1.0 : 2.0);
            output.Add (r.Flow.LiftCoeff);
            output.Add (r.Flow.Area);
            output.Add (r.Flow.UseDrag ? 1.0 : 0.0);
            output.Add (r.Command.Exponential ? 1.0 : 0.0);
            output.Add (r.Command.DeployTerm * rad);
        }

        // ---------------------------------------------------------------- classification

        /// <summary>What keeps a surface out of the replica for the whole layout (0: modeled).</summary>
        static int StaticReasons (ModuleControlSurface s)
        {
            int reasons = 0;
            if (s.GetType () != typeof (ModuleControlSurface))
                reasons |= SurfaceModel.ReasonSubclass;
            if (s.displaceVelocity)
                reasons |= SurfaceModel.ReasonRotor;
            if (s.nodeEnabled)
                reasons |= SurfaceModel.ReasonNodeLift;
            if (!SurfaceReflectionComplete ())
                reasons |= SurfaceModel.ReasonReflection;
            else if (getBaseTransform (s) == null || getCtrlSurface (s) == null)
                reasons |= SurfaceModel.ReasonNoTransform;
            return reasons;
        }

        /// <summary>The records of the vessel's current layout (built once per layout, aligned with e.Surfaces).</summary>
        static void EnsureSurfaceRecords (Entry e)
        {
            var sm = e.Sm;
            if (sm.Generation == e.TopologyGeneration && sm.Records.Count == e.Surfaces.Count)
                return;
            sm.Records.Clear ();
            sm.Generation = e.TopologyGeneration;
            sm.HaveCtrlPrevious = false;
            sm.HaveFilterPrevious = false;
            sm.Filter.Reset ();
            for (int i = 0; i < e.Surfaces.Count; i++) {
                var s = e.Surfaces [i];
                var r = new SurfaceRecord { Module = s };
                r.Curves.Module = s;
                if (s != null) {
                    r.Part = s.part;
                    r.PartPersistentId = s.part != null ? s.part.persistentId : 0u;
                    r.ModuleIndex = s.part != null ? s.part.Modules.IndexOf (s) : -1;
                    try {
                        r.StaticReasons = StaticReasons (s);
                    } catch {
                        r.StaticReasons = SurfaceModel.ReasonReflection;
                    }
                } else {
                    r.StaticReasons = SurfaceModel.ReasonNoTransform;
                }
                r.Reasons = r.StaticReasons | SurfaceModel.ReasonNotCaptured;
                sm.Records.Add (r);
            }
        }

        /// <summary>The record of e.Surfaces[i] is engaged in this step (false whenever the model is off).</summary>
        static bool SurfaceEngaged (Entry e, int i)
        {
            var sm = e.Sm;
            return sm.Mode == SurfaceModel.ModeArmed && i < sm.Records.Count &&
                sm.Records.Count == e.Surfaces.Count && sm.Records [i].Engaged;
        }

        // ---------------------------------------------------------------- the tick (Earlyish)

        /// <summary>
        /// Start of the attitude tick, model on: records of this layout, part.machNumber kept for the
        /// capture, and the engagement of this step decided from the last capture. `owned`, the
        /// axes the loop owns for this engagement (AttitudeWrite.LoopOwnedAxes); and the change of
        /// definition of the law's M this tick brings (sm.LawShift, for AttitudeLawState.ShiftTorque).
        /// </summary>
        static void SurfacesBegin (Entry e, int stage, int owned)
        {
            var sm = e.Sm;
            EnsureSurfaceRecords (e);
            sm.EarlyishTick = ownTick;
            sm.OwnedAxes = owned & 7;
            bool lag = LagEstablished (sm);
            int family = 0;
            if (sm.Mode != SurfaceModel.ModeArmed) {
                family |= SurfaceModel.SanctionNotArmed;
            } else {
                // an axis without the counted steps the guard needs to judge it is not
                // proven (published; shadow unchanged: nothing is engaged there anyway).
                family |= (7 & ~sm.Guard.Judged ()) << SurfaceModel.SanctionUnprovenShift;
                // an axis the loop does not own for this step (published; shadow unchanged).
                family |= (7 & ~sm.OwnedAxes) << SurfaceModel.SanctionNotOwnedShift;
            }
            if (!lag)
                family |= SurfaceModel.SanctionLag;
            family |= sm.Guard.Tripped & 7;
            int proven = sm.Guard.Proven ();
            // the axes whose physical slope kappa is proven now (7: no condition without
            // the flag), and, armed, why the others are not (per surface, as an earlier change).
            int kappaProven = 7;
            if (sm.Kappa) {
                kappaProven = sm.KappaEstimator.ProvenAxes ();
                int kappaOut = sm.KappaEstimator.TrippedAxes ();
                if (sm.Mode == SurfaceModel.ModeArmed)
                    family |= ((7 & ~kappaProven & ~kappaOut) << SurfaceModel.SanctionKappaUnprovenShift) |
                        (kappaOut << SurfaceModel.SanctionKappaOutShift);
            }
            bool fresh = sm.CaptureTick >= 0 && sm.CaptureTick == ownTick - 1;
            double scale = sm.LawScale;
            int engaged = 0;
            sm.HaveShift = false;
            for (int a = 0; a < 3; a++) {
                sm.LawShift [a] = 0.0;
                sm.PublishedShift [a] = 0.0;
            }
            for (int i = 0; i < sm.Records.Count; i++) {
                var r = sm.Records [i];
                if (r.Sanctions != 0)
                    family |= SurfaceModel.SanctionIdentity;
                if (r.Module != null && r.Part != null && stage == StageEarlyish) {
                    r.MachEarlyish = r.Part.machNumber;
                    r.MachFromEarlyish = true;
                } else {
                    r.MachFromEarlyish = false;
                }
                // engaged only when the scale of every body axis its column (last capture)
                // contributes to is proven in this arming. A surface can only be engaged as a whole
                // (deflectionDirection is per module and it answers every ctrlState axis at once),
                // so an axis without proof keeps out each surface that would act on it: held
                // neutral exactly as before, while the others are engaged.
                // and, with the kappa flag, only when every one of those axes has its physical
                // slope proven (an axis out of tolerance never is again in the arming).
                // and only when the loop OWNS every axis the surface needs -- each body axis
                // its column contributes to and each ctrlState axis its command answers
                // (SurfaceModel.EngagementAxes): otherwise another regulator (the kRPC AutoPilot,
                // writing the axes the loop does not) moves it there while the law counts it as its
                // own actuator (a recorded flight: engaged in the coast, the loop
                // writing the roll only). Out, it is stock again: it follows the served ctrlState,
                // and the roll gate and the neutralization of option 1 apply to it.
                int contributed = SurfaceModel.ContributedAxes (r.ColumnBody);
                r.EngagementAxes = SurfaceModel.EngagementAxes (r.ColumnBody, r.Nominal);
                r.Engaged = stage == StageEarlyish &&
                    (family & ~SurfaceModel.SanctionPerSurface) == 0 && fresh &&
                    r.Module != null && r.Reasons == 0 && r.CaptureTick == sm.CaptureTick && !r.Shielded &&
                    r.Axes != 0 && r.IdentityStrikes == 0 && (contributed & ~proven) == 0 &&
                    (contributed & ~kappaProven) == 0 && SurfaceModel.Owned (r.EngagementAxes, sm.OwnedAxes);
                // a surface entering the law's M brings its realized torque of the last
                // capture (what SurfacesTorque adds now); one leaving takes away what M carried
                // for it at the last tick. The law's torque filter absorbs the sum (ShiftTorque);
                // the published residual's filter (the surfaces outside M) the opposite change.
                if (r.Engaged != r.InLaw) {
                    sm.HaveShift = true;
                    sm.EngagementChanges++;
                    bool inPublished = fresh && r.CaptureTick == sm.CaptureTick;
                    for (int a = 0; a < 3; a++) {
                        double now = r.TorqueBody [a] * scale;
                        if (r.Engaged) {
                            sm.LawShift [a] += now;
                            sm.PublishedShift [a] -= now;
                        } else {
                            sm.LawShift [a] -= r.LawTorque [a];
                            if (inPublished)
                                sm.PublishedShift [a] += now;
                        }
                    }
                }
                r.InLaw = r.Engaged;
                for (int a = 0; a < 3; a++)
                    r.LawTorque [a] = r.Engaged ? r.TorqueBody [a] * scale : 0.0;
                if (r.Engaged)
                    engaged++;
            }
            sm.Sanctions = family;
            sm.Engaged = engaged;
        }

        /// <summary>
        /// the change of definition of the law's M this tick (surfaces entering or leaving it),
        /// absorbed by the law's torque filter and, opposite, by the published residual's: neither the
        /// allocation target nor the residuals take a step at an engagement or a disengagement.
        /// </summary>
        static void SurfacesShift (Entry e)
        {
            var sm = e.Sm;
            if (!sm.HaveShift)
                return;
            e.Law.ShiftTorque (sm.LawShift);
            sm.Filter.Shift (sm.PublishedShift);
            if (sm.HaveFilterPrevious)
                for (int a = 0; a < 3; a++)
                    sm.FilterPrevious [a] += sm.PublishedShift [a];
        }

        /// <summary>
        /// SurfaceModelV1 turned off (or reset) while surfaces were in the law's M: they leave
        /// it with the records, the law's torque filter moved by what M carried for them.
        /// </summary>
        static void SurfacesLeaveLaw (Entry e)
        {
            var sm = e.Sm;
            bool any = false;
            for (int a = 0; a < 3; a++)
                sm.LawShift [a] = 0.0;
            for (int i = 0; i < sm.Records.Count; i++) {
                var r = sm.Records [i];
                if (!r.InLaw)
                    continue;
                any = true;
                for (int a = 0; a < 3; a++)
                    sm.LawShift [a] -= r.LawTorque [a];
                r.InLaw = false;
            }
            if (any)
                e.Law.ShiftTorque (sm.LawShift);
        }

        /// <summary>Armed and engaged: the engaged surfaces' realized torque (last capture) joins the law's M.</summary>
        static void SurfacesTorque (Entry e, AttitudeInput inp)
        {
            var sm = e.Sm;
            if (sm.Mode != SurfaceModel.ModeArmed || sm.Engaged == 0)
                return;
            for (int i = 0; i < sm.Records.Count; i++) {
                var r = sm.Records [i];
                if (!r.Engaged)
                    continue;
                // exactly what SurfacesBegin recorded as the law's M for this surface.
                inp.Torque [0] += r.LawTorque [0];
                inp.Torque [1] += r.LawTorque [1];
                inp.Torque [2] += r.LawTorque [2];
            }
        }

        static readonly double[] surfaceGainScratch = new double[3];

        /// <summary>
        /// After the ctrlState columns: the free torque of every modeled surface toward the target of
        /// the served command (published), its static authority; armed and engaged, the engaged
        /// surfaces' sub-columns added to the ctrlState columns and M_free taken off the target.
        /// </summary>
        static void SurfacesColumns (Entry e, AttitudeInput inp, int n, int nGimbalColumns, double[] served,
            double dt)
        {
            var sm = e.Sm;
            bool armed = sm.Mode == SurfaceModel.ModeArmed && sm.Engaged > 0;
            for (int a = 0; a < 3; a++) {
                sm.TorqueFree [a] = 0.0;
                sm.Authority [a] = 0.0;
                inp.TorqueFree [a] = 0.0;
            }
            sm.RollPerCommand = 0.0;
            sm.RollGainMax = 0.0;
            sm.RollSpeedMin = double.NaN;
            sm.RollExponential = false;
            bool fresh = sm.CaptureTick >= 0 && sm.CaptureTick == ownTick - 1;
            double scale = sm.LawScale;
            for (int i = 0; i < sm.Records.Count; i++) {
                var r = sm.Records [i];
                if (!fresh || r.Reasons != 0 || r.CaptureTick != sm.CaptureTick || r.Shielded)
                    continue;
                double target = SurfaceModel.Target (r.Nominal, served [0], served [1], served [2]);
                SurfaceModel.TargetGradient (r.Nominal, served [0], served [1], served [2], surfaceGainScratch);
                double free = SurfaceModel.Step (r.Nominal, r.Deflection, target, dt) - r.Deflection;
                bool engaged = armed && r.Engaged;
                // what the surfaces that answer now (a held one does not) give a unit of
                // ctrlState.roll on body y, the steepest roll gain and the slowest actuator among them.
                if (!r.Neutralized || engaged) {
                    double rollGain = Math.Abs (surfaceGainScratch [1]);
                    if (rollGain > 0.0) {
                        sm.RollPerCommand += r.ColumnBody [1] * surfaceGainScratch [1] * scale;
                        if (rollGain > sm.RollGainMax)
                            sm.RollGainMax = rollGain;
                        if (double.IsNaN (sm.RollSpeedMin) || r.Nominal.Speed < sm.RollSpeedMin)
                            sm.RollSpeedMin = r.Nominal.Speed;
                        if (r.Nominal.Exponential)
                            sm.RollExponential = true;
                    }
                }
                for (int k = 0; k < 3; k++) {
                    // Published: what every modeled surface would add on its own if it followed the
                    // served command (a neutralized one does not: its target is 0).
                    if (!r.Neutralized || engaged)
                        sm.TorqueFree [k] += r.ColumnBody [k] * free * scale;
                    for (int a = 0; a < 3; a++)
                        sm.Authority [k] += Math.Abs (r.ColumnBody [k] * surfaceGainScratch [a]) * scale;
                }
                if (!engaged)
                    continue;
                for (int k = 0; k < 3; k++)
                    inp.TorqueFree [k] += r.ColumnBody [k] * free * scale;
                for (int a = 0; a < 3; a++) {
                    int j = nGimbalColumns + a;
                    for (int k = 0; k < 3; k++)
                        inp.B [k * n + j] += r.ColumnBody [k] * surfaceGainScratch [a] * scale;
                }
            }
            inp.UseTorqueFree = armed;
        }

        /// <summary>
        /// After the law: the INDI residual with the realized torque of the surfaces that are NOT in
        /// the law's M (all of them in shadow), on the law's own filter (published).
        /// </summary>
        static void SurfacesResidual (Entry e, AttitudeInput inp, AttitudeOutput o)
        {
            var sm = e.Sm;
            for (int a = 0; a < 3; a++) {
                sm.ResidualLaw [a] = o.Filtered ? o.Residual [a] : double.NaN;
                sm.FilterIn [a] = 0.0;
            }
            bool fresh = sm.CaptureTick >= 0 && sm.CaptureTick == ownTick - 1;
            double scale = sm.LawScale;
            for (int i = 0; i < sm.Records.Count; i++) {
                var r = sm.Records [i];
                if (r.Engaged || r.CaptureTick != sm.CaptureTick || !fresh)
                    continue;
                for (int a = 0; a < 3; a++)
                    sm.FilterIn [a] += r.TorqueBody [a] * scale;
            }
            if (!o.Filtered || !inp.OmegaDotValid) {
                sm.Filter.Reset ();
                sm.HaveFilterPrevious = false;
                sm.HaveResidualF = false;
                for (int a = 0; a < 3; a++) {
                    sm.ResidualSurfaces [a] = double.NaN;
                    sm.ResidualSurfacesF [a] = double.NaN;
                }
                return;
            }
            var g = e.Gains;
            sm.Filter.Configure (inp.Dt, g.FilterOmegaC, g.FilterZeta);
            sm.Filter.Step (sm.FilterIn, sm.FilterOut);
            double beta = inp.Dt / (g.TauResidual + inp.Dt);
            for (int a = 0; a < 3; a++) {
                double d = sm.HaveFilterPrevious ? (sm.FilterOut [a] - sm.FilterPrevious [a]) / inp.Inertia [a] : 0.0;
                sm.ResidualSurfaces [a] = o.Residual [a] - d;
                sm.ResidualSurfacesF [a] = sm.HaveResidualF
                    ? sm.ResidualSurfacesF [a] + beta * (sm.ResidualSurfaces [a] - sm.ResidualSurfacesF [a])
                    : sm.ResidualSurfaces [a];
                sm.FilterPrevious [a] = sm.FilterOut [a];
            }
            sm.HaveFilterPrevious = true;
            sm.HaveResidualF = true;
        }

        /// <summary>
        /// Informational motifs and the capability of this step (0 when the model is off). Motif 38
        /// says a family condition holds surfaces back: a trip, the lag, an identity sanction, and
        /// (armed) an axis not proven yet, an axis the loop does not own.
        /// </summary>
        static long SurfaceMotifs (Entry e)
        {
            var sm = e.Sm;
            if (sm.Mode == SurfaceModel.ModeOff)
                return 0;
            long m = 0;
            if (sm.Mode == SurfaceModel.ModeArmed && sm.Engaged > 0)
                m |= MotifSurfacesEngaged;
            if ((sm.Sanctions & ~SurfaceModel.SanctionNotArmed) != 0)
                m |= MotifSurfaceSanction;
            return m;
        }

        /// <summary>The same-step response, from the targets and the fly-by-wire probe together.</summary>
        static bool LagEstablished (SurfaceState sm)
        {
            return SurfaceModel.LagEstablished (sm.LagSame + sm.ProbeSame, sm.LagPrevious + sm.ProbePrevious,
                sm.LagNeither);
        }

        /// <summary>Our fly-by-wire callback (armed loop only): each module's liftForce at this instant.</summary>
        static void SurfacesFlyByWire (Entry e)
        {
            var sm = e.Sm;
            for (int i = 0; i < sm.Records.Count; i++) {
                var r = sm.Records [i];
                var s = r.Module;
                if (s == null)
                    continue;
                Vector3 lift = s.liftForce;
                r.FbwLift [0] = lift.x;
                r.FbwLift [1] = lift.y;
                r.FbwLift [2] = lift.z;
                r.FbwTick = ownTick;
            }
        }

        // ---------------------------------------------------------------- the capture (BetterLateThanNever)

        static void CaptureAllSurfaces ()
        {
            foreach (var pair in entries) {
                var e = pair.Value;
                if (e.Sm.Mode == SurfaceModel.ModeOff || e.Dropped)
                    continue;
                try {
                    CaptureSurfaces (e);
                } catch (Exception exc) {
                    e.Sm.CaptureTick = -1;
                    if (++e.Exceptions <= 5)
                        BridgeLog.Error ("SurfaceModelV1 capture: " + exc);
                }
            }
        }

        static readonly double[] capTorque = new double[3], capPrevTorque = new double[3];
        static readonly double[] capPredTorque = new double[3], capColumn = new double[3];
        static readonly double[] capMismatch = new double[3], capJump = new double[3];
        // The reference's rotation of this capture, inverted, in double (the guard's body frame).
        static double capQx, capQy, capQz, capQw;

        /// <summary>The capture of one armed vessel at the end of the physics step (see the class summary).</summary>
        static void CaptureSurfaces (Entry e)
        {
            var sm = e.Sm;
            var vessel = Resolve (e);
            if (vessel == null || vessel.ReferenceTransform == null) {
                sm.CaptureTick = -1;
                return;
            }
            EnsureSurfaceRecords (e);
            var reference = vessel.ReferenceTransform;
            Vector3 com = vessel.CurrentCoM;
            var ctrl = vessel.ctrlState;
            double dt = TimeWarp.fixedDeltaTime;
            sm.CaptureTick = ownTick;
            sm.CaptureUt = Planetarium.GetUniversalTime ();
            sm.CaptureDt = dt;
            sm.CaptureMasterTick = GameClock.StepTick;
            sm.Ctrl [0] = ctrl == null ? 0.0 : ctrl.pitch;
            sm.Ctrl [1] = ctrl == null ? 0.0 : ctrl.roll;
            sm.Ctrl [2] = ctrl == null ? 0.0 : ctrl.yaw;
            sm.IdentityMax = double.NaN;
            sm.IdentityJudged = 0;
            sm.Modeled = 0;
            sm.NeutralizedCount = 0;
            double liftMultiplier = PhysicsGlobals.LiftMultiplier;
            double dragMultiplier = PhysicsGlobals.LiftDragMultiplier;
            bool guardStep = false;
            SetBodyInverse (reference.rotation);
            for (int a = 0; a < 3; a++) {
                sm.TorqueBody [a] = 0.0;
                sm.Predicted [a] = 0.0;
                sm.Realized [a] = 0.0;
                sm.Uncertainty [a] = 0.0;
                sm.KappaDeflection [a] = 0.0;
            }
            for (int i = 0; i < sm.Records.Count; i++) {
                var r = sm.Records [i];
                var s = r.Module;
                if (s == null || r.Part == null) {
                    r.Reasons = r.StaticReasons | SurfaceModel.ReasonNoTransform;
                    r.CaptureTick = -1;
                    r.HavePrevious = false;
                    continue;
                }
                r.Axes = AttitudeGeometry.SurfaceAnsweredAxes (s);
                r.Neutralized = e.Neutralized.Contains (s);
                if (r.Neutralized)
                    sm.NeutralizedCount++;
                int reasons = r.StaticReasons | r.Sanctions;
                if (AdjusterCount (controlAdjusters, s) + AdjusterCount (liftAdjusters, s) > 0)
                    reasons |= SurfaceModel.ReasonAdjusters;
                if ((r.StaticReasons & (SurfaceModel.ReasonReflection | SurfaceModel.ReasonNoTransform)) != 0) {
                    r.Reasons = reasons;
                    r.CaptureTick = -1;
                    r.HavePrevious = false;
                    continue;
                }
                var baseT = getBaseTransform (s);
                var surfaceT = getCtrlSurface (s);
                if (baseT == null || surfaceT == null) {
                    r.Reasons = reasons | SurfaceModel.ReasonNoTransform;
                    r.CaptureTick = -1;
                    r.HavePrevious = false;
                    continue;
                }
                // ---- command law of this step (FixedCtrlSurfaceUpdate), geometry as the module saw it.
                double direction = getDirection (s);
                FillCommand (r.Command, s, baseT, reference, com, r.Neutralized ? 0.0 : direction);
                r.Nominal.CopyFrom (r.Command);
                r.Nominal.Direction = r.Neutralized ? NeutralizedOriginal (e, s, direction) : direction;
                double deflection = getDeflection (s);
                r.Action = getAction (s);
                r.TargetSame = SurfaceModel.Target (r.Command, sm.Ctrl [0], sm.Ctrl [1], sm.Ctrl [2]);
                r.TargetPrevious = sm.HaveCtrlPrevious
                    ? SurfaceModel.Target (r.Command, sm.CtrlPrevious [0], sm.CtrlPrevious [1], sm.CtrlPrevious [2])
                    : double.NaN;
                r.TargetNominal = SurfaceModel.Target (r.Nominal, sm.Ctrl [0], sm.Ctrl [1], sm.Ctrl [2]);
                SurfaceModel.TargetGradient (r.Nominal, sm.Ctrl [0], sm.Ctrl [1], sm.Ctrl [2], r.Gain);
                bool havePrevious = r.HavePrevious && r.CaptureTick == sm.CaptureTick - 1;
                r.DeflectionPrevious = havePrevious ? r.Deflection : double.NaN;
                r.PredictedSame = havePrevious ? SurfaceModel.Step (r.Command, r.DeflectionPrevious, r.TargetSame, dt)
                    : double.NaN;
                r.PredictedPrevious = havePrevious && sm.HaveCtrlPrevious
                    ? SurfaceModel.Step (r.Command, r.DeflectionPrevious, r.TargetPrevious, dt) : double.NaN;
                r.Deflection = deflection;
                r.LagOutcome = SurfaceModel.LagOutcome (r.Action, r.TargetSame, r.TargetPrevious);
                if ((reasons & (SurfaceModel.ReasonSubclass | SurfaceModel.ReasonRotor)) == 0) {
                    if (r.LagOutcome == 0)
                        sm.LagSame++;
                    else if (r.LagOutcome == 1)
                        sm.LagPrevious++;
                    else if (r.LagOutcome == 2)
                        sm.LagNeither++;
                }
                // ---- flow and forces of this step.
                var part = r.Part;
                r.Shielded = part.ShieldedFromAirstream || part.Rigidbody == null;
                FillFlow (r, s, baseT, part, com, liftMultiplier, dragMultiplier);
                // The module's own test, through Unity's own operator: see RotationTest. The
                // previous capture's state is the module's at the previous deflection.
                bool rotatedPrevious = r.Rotated;
                Quaternion neutral = getNeutral (s);
                r.Rotated = s.alwaysRecomputeLift || RotationTest (surfaceT.localRotation, neutral);
                if (!s.alwaysRecomputeLift && SurfaceModel.InRotationBand (deflection))
                    BandDiagnostic (sm, r, (float)deflection, neutral);
                if (r.Shielded) {
                    reasons |= SurfaceModel.ReasonShielded;
                    r.HaveLiftPrevious = false;
                    for (int k = 0; k < 3; k++) {
                        r.Lift [k] = r.Drag [k] = r.LiftModel [k] = r.DragModel [k] = 0.0;
                        r.ForceBody [k] = r.ForceModelBody [k] = r.TorqueBody [k] = r.ColumnBody [k] = 0.0;
                    }
                    r.Identity = double.NaN;
                } else {
                    Vector3 lift = s.liftForce, drag = s.dragForce;
                    for (int k = 0; k < 3; k++)
                        r.LiftPrevious [k] = r.Lift [k];
                    r.Lift [0] = lift.x;
                    r.Lift [1] = lift.y;
                    r.Lift [2] = lift.z;
                    // The fly-by-wire probe (exact float copies of the same field).
                    if (r.FbwTick == sm.CaptureTick && r.HaveLiftPrevious &&
                            (reasons & (SurfaceModel.ReasonSubclass | SurfaceModel.ReasonRotor)) == 0) {
                        bool moved = r.Lift [0] != r.LiftPrevious [0] || r.Lift [1] != r.LiftPrevious [1] ||
                            r.Lift [2] != r.LiftPrevious [2];
                        bool atNow = r.FbwLift [0] == r.Lift [0] && r.FbwLift [1] == r.Lift [1] &&
                            r.FbwLift [2] == r.Lift [2];
                        bool atPrevious = r.FbwLift [0] == r.LiftPrevious [0] &&
                            r.FbwLift [1] == r.LiftPrevious [1] && r.FbwLift [2] == r.LiftPrevious [2];
                        if (moved && atNow)
                            sm.ProbePrevious++;
                        else if (moved && atPrevious)
                            sm.ProbeSame++;
                    }
                    r.HaveLiftPrevious = true;
                    if (r.Flow.UseDrag) {
                        r.Drag [0] = drag.x;
                        r.Drag [1] = drag.y;
                        r.Drag [2] = drag.z;
                    } else {
                        // Not applied (and not recomputed) without the internal drag model.
                        r.Drag [0] = r.Drag [1] = r.Drag [2] = 0.0;
                    }
                    SurfaceModel.Forces (r.Flow, r.Curves, deflection, r.Rotated, r.LiftModel, r.DragModel);
                    r.Identity = SurfaceModel.Identity (r.LiftModel, r.DragModel, r.Lift, r.Drag,
                        SurfaceModel.IdentityFloorN);
                    if (!double.IsNaN (r.Identity) && (reasons & SurfaceModel.ReasonSubclass) == 0) {
                        sm.IdentityJudged++;
                        if (double.IsNaN (sm.IdentityMax) || r.Identity > sm.IdentityMax)
                            sm.IdentityMax = r.Identity;
                        if (r.Identity > SurfaceModel.IdentityTolerance) {
                            if (++r.IdentityStrikes >= SurfaceModel.IdentityStrikes)
                                r.Sanctions |= SurfaceModel.ReasonIdentity;
                        } else {
                            r.IdentityStrikes = 0;
                        }
                    }
                    Body (reference, r.Lift [0] + r.Drag [0], r.Lift [1] + r.Drag [1], r.Lift [2] + r.Drag [2],
                        r.ForceBody);
                    Body (reference, r.LiftModel [0] + r.DragModel [0], r.LiftModel [1] + r.DragModel [1],
                        r.LiftModel [2] + r.DragModel [2], r.ForceModelBody);
                    SurfaceModel.Torque (r.Flow, r.Lift, r.Drag, capTorque);
                    Body (reference, capTorque [0], capTorque [1], capTorque [2], r.TorqueBody);
                    SurfaceModel.Column (r.Flow, r.Curves, deflection, capColumn);
                    Body (reference, capColumn [0], capColumn [1], capColumn [2], r.ColumnBody);
                    // ---- scale guard: the step's realized increment against the predicted one, both
                    // from the model at the previous deflection and THIS step's flow.
                    // A surface whose replica is off in this step (before its sanction) stays out.
                    // each model torque in the state the module has at that deflection -- the
                    // previous capture's for the previous one, the module's own test on the rotation
                    // it would set (FixedCtrlSurfaceUpdate: AngleAxis(deflection, right) * neutral)
                    // for the predicted one. Always turned, a fin resting below the threshold
                    // (|deflection| < ~0.16 deg, moving part not turned) added -column x deflection to
                    // the realized increment of every judged step: up to 0.5 % of its authority,
                    // enough to drag k out of [0.5, 2] when the other fins' increments are small.
                    //
                    // (a recorded flight, k_y 1.000 -> 0.503 on the roll axis's
                    // first counted step, two fins at +-0.157 deg): the module's turned state at the
                    // predicted deflection is not reproducible inside Unity's rotation band -- the
                    // test of AngleAxis(predicted) * neutral need not answer as the rotation the
                    // module stores. Every move of k above 0.003 in that flight fell on steps with a
                    // fin inside the band. So each step brings its uncertainty: the replica's
                    // measured mismatch at the realized state (exactly what a surface the step leaves
                    // in place adds to realized - predicted, the float rounding of the game's forces
                    // included), the float resolution of the deflections, and, for a surface whose
                    // step ends inside the band, the whole turned-state jump. An axis counts the step
                    // only when that is below IdentityTolerance of the predicted increment
                    // (SurfaceScaleGuard.Update).
                    // A surface the step leaves where it was (the same float deflection, predicted
                    // and realized) keeps its turned state, certain: the module stores the same
                    // rotation again. Increments and uncertainty in double (BodyExact): a float
                    // rotation would leak 2^-24 of the strong axes into a weakly excited one.
                    bool identityOk = double.IsNaN (r.Identity) || r.Identity <= SurfaceModel.IdentityTolerance;
                    if (havePrevious && reasons == 0 && identityOk && r.IdentityStrikes == 0 &&
                            !double.IsNaN (r.PredictedSame)) {
                        bool resting = (float)r.PredictedSame == (float)r.DeflectionPrevious &&
                            deflection == r.DeflectionPrevious;
                        bool rotatedPredicted = resting ? rotatedPrevious : s.alwaysRecomputeLift || RotationTest (
                            Quaternion.AngleAxis ((float)r.PredictedSame, Vector3.right) * neutral, neutral);
                        SurfaceModel.TorqueAt (r.Flow, r.Curves, r.DeflectionPrevious, rotatedPrevious, capPrevTorque);
                        SurfaceModel.TorqueAt (r.Flow, r.Curves, r.PredictedSame, rotatedPredicted, capPredTorque);
                        SurfaceModel.Torque (r.Flow, r.LiftModel, r.DragModel, capMismatch);
                        for (int k = 0; k < 3; k++) {
                            capPredTorque [k] -= capPrevTorque [k];
                            capPrevTorque [k] = capTorque [k] - capPrevTorque [k];
                            capMismatch [k] = capTorque [k] - capMismatch [k];
                        }
                        BodyExact (capPredTorque);
                        BodyExact (capPrevTorque);
                        BodyExact (capMismatch);
                        bool jump = false;
                        if (!s.alwaysRecomputeLift && !resting) {
                            bool bandPredicted = SurfaceModel.InRotationBand (r.PredictedSame);
                            if (bandPredicted || SurfaceModel.InRotationBand (deflection)) {
                                SurfaceModel.TurnJump (r.Flow, r.Curves, bandPredicted ? r.PredictedSame : deflection,
                                    capJump);
                                BodyExact (capJump);
                                jump = true;
                            }
                        }
                        double deflections = SurfaceModel.DeflectionResolution (r.Action, r.TargetSame,
                            r.DeflectionPrevious, r.PredictedSame, deflection);
                        for (int k = 0; k < 3; k++) {
                            sm.Predicted [k] += capPredTorque [k];
                            sm.Realized [k] += capPrevTorque [k];
                            sm.Uncertainty [k] += Math.Abs (capMismatch [k]) + Math.Abs (r.ColumnBody [k]) * deflections +
                                (jump ? Math.Abs (capJump [k]) : 0.0);
                        }
                        guardStep = true;
                    }
                    if (reasons == 0)
                        for (int k = 0; k < 3; k++)
                            sm.TorqueBody [k] += r.TorqueBody [k];
                    // the deflection-driven torque increment of a modeled surface: its
                    // realized torque minus the replica at the previous deflection (in the state the
                    // module had there) and THIS step's flow -- what the law's columns predict
                    // for its move. The flow-driven rest of its increment (angle of attack, q) stays
                    // in the vessel's aerodynamic increment, with the hull's (CaptureKappaAero). A
                    // surface without the previous capture leaves its whole increment there.
                    if (sm.Kappa && reasons == 0 && havePrevious) {
                        SurfaceModel.TorqueAt (r.Flow, r.Curves, r.DeflectionPrevious, rotatedPrevious, capKappa);
                        for (int k = 0; k < 3; k++)
                            capKappa [k] = capTorque [k] - capKappa [k];
                        BodyExact (capKappa);
                        for (int k = 0; k < 3; k++)
                            sm.KappaDeflection [k] += capKappa [k];
                    }
                }
                r.Reasons = reasons;
                if (reasons == 0)
                    sm.Modeled++;
                r.MachUsedFromEarlyish = r.MachFromEarlyish;
                r.CaptureTick = sm.CaptureTick;
                r.HavePrevious = true;
            }
            if (guardStep)
                sm.Guard.Update (sm.Predicted, sm.Realized, sm.Uncertainty);
            if (sm.Kappa)
                CaptureKappaAero (e, vessel, com);
            for (int a = 0; a < 3; a++)
                sm.CtrlPrevious [a] = sm.Ctrl [a];
            sm.HaveCtrlPrevious = true;
        }

        // ---------------------------------------------------------------- kappa

        static readonly double[] capKappa = new double[3], capKappaAero = new double[3];

        /// <summary>
        /// end of the capture: the whole vessel's aerodynamic torque of this step (body, kN.m)
        /// and, against the previous capture's, the nuisance increment: everything aerodynamic that is
        /// not the modeled surfaces' deflection response (N.m). Its cost is published.
        /// </summary>
        static void CaptureKappaAero (Entry e, Vessel vessel, Vector3 com)
        {
            var sm = e.Sm;
            long start = System.Diagnostics.Stopwatch.GetTimestamp ();
            if (KappaAeroTorque (e, vessel, com, capKappaAero)) {
                BodyExact (capKappaAero);
                bool increment = sm.KappaAeroTick >= 0 && sm.KappaAeroTick == sm.CaptureTick - 1;
                for (int k = 0; k < 3; k++) {
                    sm.KappaNuisance [k] = increment ? SurfaceModel.NewtonPerKilonewton *
                        (capKappaAero [k] - sm.KappaAero [k] - sm.KappaDeflection [k]) : 0.0;
                    sm.KappaAero [k] = capKappaAero [k];
                }
                sm.KappaAeroTick = sm.CaptureTick;
                sm.KappaIncrementTick = increment ? sm.CaptureTick : -1;
            } else {
                sm.KappaAeroTick = -1;
                sm.KappaIncrementTick = -1;
            }
            double us = (System.Diagnostics.Stopwatch.GetTimestamp () - start) * 1e6 /
                System.Diagnostics.Stopwatch.Frequency;
            sm.KappaCostLast = us;
            if (double.IsNaN (sm.KappaCostMax) || us > sm.KappaCostMax)
                sm.KappaCostMax = us;
        }

        /// <summary>
        /// The vessel's aerodynamic torque about the CoM (world, kN.m) as the game applied it this
        /// step, read as kRPC's Flight.AerodynamicTorque does (KSP 1.12.5, FlightIntegrator.
        /// ApplyAeroDrag / ApplyAeroLift and ModuleLiftingSurface, decompiled): for every part with a
        /// rigidbody, its cube drag -dragVectorDir x dragScalar at its CoPOffset, its body lift
        /// (bodyLiftLocalVector at bodyLiftLocalPosition, unless the part has a lift module or a
        /// lifting body-lift-only provider), minus its rigidbody's angular drag times the angular
        /// momentum it acts on; and every lifting surface's liftForce at its part's CoLOffset and
        /// dragForce at its CoPOffset (the drag only with the internal drag model, the only case it
        /// is applied). Double sums, no allocation (the lifting modules are listed once per layout).
        /// </summary>
        static bool KappaAeroTorque (Entry e, Vessel vessel, Vector3 com, double[] torque)
        {
            var sm = e.Sm;
            var parts = vessel.parts;
            if (parts == null)
                return false;
            if (sm.KappaGeneration < 0 || sm.KappaGeneration != e.TopologyGeneration) {
                sm.KappaLifting.Clear ();
                for (int i = 0; i < parts.Count; i++) {
                    var p = parts [i];
                    if (p == null || p.Modules == null)
                        continue;
                    for (int k = 0; k < p.Modules.Count; k++) {
                        var m = p.Modules [k] as ModuleLiftingSurface;
                        if (m != null)
                            sm.KappaLifting.Add (m);
                    }
                }
                sm.KappaGeneration = e.TopologyGeneration;
            }
            double tx = 0.0, ty = 0.0, tz = 0.0;
            int counted = 0;
            for (int i = 0; i < parts.Count; i++) {
                var p = parts [i];
                if (p == null || p.Rigidbody == null)
                    continue;
                var pt = p.partTransform;
                if (pt == null)
                    continue;
                counted++;
                float scalar = p.dragScalar;
                if (scalar != 0f) {
                    Vector3 dir = p.dragVectorDir;
                    AddMoment (pt.TransformPoint (p.CoPOffset), com, -(double)dir.x * scalar, -(double)dir.y * scalar,
                        -(double)dir.z * scalar, ref tx, ref ty, ref tz);
                }
                bool liftOnly = p.bodyLiftOnlyUnattachedLiftActual && p.bodyLiftOnlyProvider != null &&
                    p.bodyLiftOnlyProvider.IsLifting;
                if (!p.hasLiftModule && !liftOnly) {
                    Vector3 local = p.bodyLiftLocalVector;
                    if (local.x != 0f || local.y != 0f || local.z != 0f) {
                        Vector3 lift = pt.TransformDirection (local);
                        AddMoment (pt.TransformPoint (p.bodyLiftLocalPosition), com, lift.x, lift.y, lift.z,
                            ref tx, ref ty, ref tz);
                    }
                }
                var rb = p.rb;
                if (rb != null) {
                    float drag = rb.angularDrag;
                    if (drag > 0f) {
                        // kRPC's StockAerodynamics.PartAngularMomentum: the rigidbody's own inertia
                        // tensor (t.m^2) about its principal axes, at its angular velocity.
                        Quaternion q = rb.rotation * rb.inertiaTensorRotation;
                        Vector3 w = Quaternion.Inverse (q) * rb.angularVelocity;
                        Vector3 inertia = rb.inertiaTensor;
                        Vector3 l = q * new Vector3 (inertia.x * w.x, inertia.y * w.y, inertia.z * w.z);
                        tx -= drag * (double)l.x;
                        ty -= drag * (double)l.y;
                        tz -= drag * (double)l.z;
                    }
                }
            }
            for (int j = 0; j < sm.KappaLifting.Count; j++) {
                var m = sm.KappaLifting [j];
                var p = m == null ? null : m.part;
                if (p == null || p.Rigidbody == null || p.partTransform == null)
                    continue;
                Vector3 lift = m.liftForce;
                if (lift.x != 0f || lift.y != 0f || lift.z != 0f)
                    AddMoment (p.partTransform.TransformPoint (p.CoLOffset), com, lift.x, lift.y, lift.z,
                        ref tx, ref ty, ref tz);
                if (!m.useInternalDragModel)
                    continue;
                Vector3 drag = m.dragForce;
                if (drag.x != 0f || drag.y != 0f || drag.z != 0f)
                    AddMoment (p.partTransform.TransformPoint (p.CoPOffset), com, drag.x, drag.y, drag.z,
                        ref tx, ref ty, ref tz);
            }
            torque [0] = tx;
            torque [1] = ty;
            torque [2] = tz;
            sm.KappaParts = counted;
            double sum = tx + ty + tz;
            return !double.IsNaN (sum) && !double.IsInfinity (sum);
        }

        /// <summary>(point - com) x force, added in double.</summary>
        static void AddMoment (Vector3 point, Vector3 com, double fx, double fy, double fz, ref double tx,
            ref double ty, ref double tz)
        {
            double lx = (double)point.x - com.x, ly = (double)point.y - com.y, lz = (double)point.z - com.z;
            tx += ly * fz - lz * fy;
            ty += lz * fx - lx * fz;
            tz += lx * fy - ly * fx;
        }

        /// <summary>the attitude tick before the engaged surfaces join M: the law's engine + RCS + wheel torque.</summary>
        static void SurfacesKappaNonFin (Entry e, double[] torque, Vector3 engineBody)
        {
            var sm = e.Sm;
            if (!sm.Kappa)
                return;
            sm.KappaNonFin [0] = torque [0];
            sm.KappaNonFin [1] = torque [1];
            sm.KappaNonFin [2] = torque [2];
            sm.KappaEngine = engineBody.x != 0f || engineBody.y != 0f || engineBody.z != 0f;
        }

        /// <summary>
        /// the attitude tick (kappa on): the inertia Vessel.MOI leaves out, per body axis (kg.m^2). For
        /// every part with a rigidbody, as VesselPrecalculate.CalculatePhysicsStats sums it (rb.mass, the
        /// reference transform's axes, the vessel's CoM), the parallel-axis term at rb.worldCenterOfMass --
        /// where the physics has the mass, FlightIntegrator setting rb.centerOfMass = CoMOffset -- minus the
        /// one at rb.position, which the stock sum takes (SurfaceKappa.AddInertiaOffset). NaN when a value is
        /// not finite. Published whatever the flag; used by the estimator only with it. No allocation.
        /// </summary>
        static void SurfacesKappaInertia (Entry e, Vessel vessel, Transform reference, Vector3 com)
        {
            var sm = e.Sm;
            if (!sm.Kappa)
                return;
            var offset = sm.KappaInertiaOffset;
            offset [0] = offset [1] = offset [2] = 0.0;
            var parts = vessel.parts;
            bool ok = parts != null && reference != null;
            if (ok) {
                Quaternion inverse = Quaternion.Inverse (reference.rotation);
                for (int i = 0; i < parts.Count; i++) {
                    var p = parts [i];
                    var rb = p == null ? null : p.rb;
                    if (rb == null)
                        continue;
                    Vector3 c = inverse * (rb.worldCenterOfMass - com);
                    Vector3 o = inverse * (rb.position - com);
                    inertiaCentre [0] = c.x;
                    inertiaCentre [1] = c.y;
                    inertiaCentre [2] = c.z;
                    inertiaOrigin [0] = o.x;
                    inertiaOrigin [1] = o.y;
                    inertiaOrigin [2] = o.z;
                    if (!SurfaceKappa.AddInertiaOffset (rb.mass * Frames.TonneToKilogram, inertiaCentre, inertiaOrigin,
                            offset)) {
                        ok = false;
                        break;
                    }
                }
            }
            if (!ok)
                offset [0] = offset [1] = offset [2] = double.NaN;
        }

        static readonly double[] inertiaCentre = new double[3], inertiaOrigin = new double[3];

        /// <summary>
        /// the attitude tick after the law: one step of the estimator, with the law's measured
        /// omega_dot and inertia, the non-surface torques, and the increments of the capture of the
        /// previous physics step (the one this tick's omega_dot measures), the deflection's in the
        /// law's unit (x LawScale: 1000 with an earlier change's flag, 1 as up to ad35d85 -- where kappa reads 1000).
        /// </summary>
        static void SurfacesKappaStep (Entry e, AttitudeInput inp, AttitudeOutput o)
        {
            var sm = e.Sm;
            if (!sm.Kappa)
                return;
            bool increment = sm.CaptureTick >= 0 && sm.CaptureTick == ownTick - 1 &&
                sm.KappaIncrementTick == sm.CaptureTick;
            // The axes the surfaces can act on: their static authority (physical N.m, whatever the
            // law's unit) at least the replica's precision of the loop's whole authority there.
            int axes = 0;
            double physical = sm.LawScale > 0.0 ? SurfaceModel.NewtonPerKilonewton / sm.LawScale : 0.0;
            for (int a = 0; a < 3; a++) {
                sm.KappaDeflectionLaw [a] = sm.KappaDeflection [a] * sm.LawScale;
                double loop = o.Filtered ? o.Alpha [a] * inp.Inertia [a] : double.NaN;
                if (sm.Authority [a] * physical >= SurfaceModel.IdentityTolerance * loop)
                    axes |= 1 << a;
            }
            sm.KappaAxes = axes;
            // with the flag, the measured side on the physical inertia (a NaN offset: the step is not usable).
            double[] inertia = inp.Inertia;
            if (sm.KappaInertia) {
                for (int a = 0; a < 3; a++)
                    sm.KappaInertiaUsed [a] = inp.Inertia [a] + sm.KappaInertiaOffset [a];
                inertia = sm.KappaInertiaUsed;
            }
            // with the body rate, for the gyroscopic torque omega x I omega of the step.
            sm.KappaCounted = sm.KappaEstimator.Step (ownTick, Planetarium.GetUniversalTime (), inp.OmegaDotValid, inp.Dt,
                e.Gains.FilterOmegaC, e.Gains.FilterZeta, inp.Omega, inp.OmegaDotMeasured, inertia, sm.KappaNonFin,
                sm.KappaEngine, increment, sm.KappaDeflectionLaw, sm.KappaNuisance, axes, sm.KappaLeak, physical);
        }

        /// <summary>the kappa table of an armed vessel (same layout when off or absent).</summary>
        internal static IList<double> SurfacesKappaRead (string persistentId)
        {
            uint pid = VesselLookup.ParsePersistentId (persistentId);
            Entry entry;
            if (!entries.TryGetValue (pid, out entry))
                return KappaReply (null, emptySurfaceState);
            return KappaReply (entry, entry.Sm);
        }

        static List<double> KappaReply (Entry e, SurfaceState sm)
        {
            bool on = e != null && sm.Mode != SurfaceModel.ModeOff && sm.Kappa;
            var k = sm.KappaEstimator;
            var output = new List<double> (SurfaceKappa.HeaderStride + SurfaceKappa.AxisCount * SurfaceKappa.AxisStride +
                SurfaceKappa.ClockStride);
            output.Add (SurfaceKappa.Protocol);
            output.Add (SurfaceKappa.Schema);
            output.Add (on ? 1.0 : 0.0);
            output.Add (e == null ? double.NaN : sm.WireMode);
            output.Add (on ? k.LastTick : double.NaN);
            output.Add (on ? k.LastUt : double.NaN);
            output.Add (e == null ? double.NaN : e.PersistentId);
            output.Add (on ? k.BatchSteps : double.NaN);
            output.Add (on ? k.BaseBatchSteps : double.NaN);
            output.Add (SurfaceKappa.BatchCapacity);
            output.Add (SurfaceKappa.FalseAlarm);
            // the band kappa is judged against (an earlier change's proof band; schema 1 published [0.5, 2]).
            output.Add (SurfaceKappa.ProofMin);
            output.Add (SurfaceKappa.ProofMax);
            output.Add (on ? k.StepsSeen : double.NaN);
            output.Add (on ? k.StepsCounted : double.NaN);
            output.Add (on ? k.Restarts : double.NaN);
            output.Add (on ? k.EngineQuietSteps : double.NaN);
            output.Add (on ? sm.KappaParts : double.NaN);
            output.Add (on ? sm.KappaLifting.Count : double.NaN);
            output.Add (on ? sm.KappaCostLast : double.NaN);
            output.Add (on ? sm.KappaCostMax : double.NaN);
            output.Add (on ? sm.KappaAxes : double.NaN);
            output.Add (on ? k.ProvenAxes () : double.NaN);
            output.Add (on ? k.TrippedAxes () : double.NaN);
            output.Add (SurfaceKappa.AxisCount);
            output.Add (SurfaceKappa.AxisStride);
            for (int a = 0; a < SurfaceKappa.AxisCount; a++) {
                var x = k.Axes [a];
                double tol = x.T * x.Se;
                output.Add (a);
                output.Add (on ? x.State : double.NaN);
                output.Add (on ? (x.Proven ? 1.0 : 0.0) : double.NaN);
                output.Add (on ? (x.Tripped ? 1.0 : 0.0) : double.NaN);
                output.Add (on ? (x.Consistent ? 1.0 : 0.0) : double.NaN);
                output.Add (on ? x.Kappa : double.NaN);
                output.Add (on ? x.Se : double.NaN);
                output.Add (on ? x.T : double.NaN);
                output.Add (on ? tol : double.NaN);
                output.Add (on ? x.Kappa - tol : double.NaN);
                output.Add (on ? x.Kappa + tol : double.NaN);
                output.Add (on ? (x.Kappa - 1.0) / x.Se : double.NaN);
                output.Add (on ? x.Nuisance : double.NaN);
                output.Add (on ? x.NuisanceSe : double.NaN);
                output.Add (on ? x.RegressorsUsed : double.NaN);
                output.Add (on ? x.Steps : double.NaN);
                output.Add (on ? x.Count : double.NaN);
                output.Add (on ? x.EffectiveBatches : double.NaN);
                output.Add (on ? x.RmsY : double.NaN);
                output.Add (on ? x.RmsFins : double.NaN);
                output.Add (on ? x.RmsNuisance : double.NaN);
                output.Add (on ? x.RmsResidual : double.NaN);
                // (schema 2): the out-of-tolerance test and the gyroscopic increment.
                output.Add (on ? x.OutDof : double.NaN);
                output.Add (on ? x.OutT : double.NaN);
                output.Add (on ? x.OutLow : double.NaN);
                output.Add (on ? x.OutHigh : double.NaN);
                output.Add (on ? x.RmsGyro : double.NaN);
                // (schema 3): the leak gate, the steps it refused, the leak slope of the counted steps.
                output.Add (on ? (sm.KappaLeak ? 1.0 : 0.0) : double.NaN);
                output.Add (on ? x.LeakRefused : double.NaN);
                output.Add (on ? x.LeakBeta : double.NaN);
                // (schema 4): the measured side on the physical inertia, and the offset Vessel.MOI leaves out.
                output.Add (on ? (sm.KappaInertia ? 1.0 : 0.0) : double.NaN);
                output.Add (on ? sm.KappaInertiaOffset [a] : double.NaN);
            }
            output.Add (on ? sm.CaptureMasterTick : double.NaN);
            return output;
        }

        // ---------------------------------------------------------------- roll excitation

        /// <summary>
        /// the attitude tick after the loop's write decisions: the roll excitation of this
        /// step (Model/SurfaceKappaRoll.cs), added to the AutoPilot's ctrlState.roll by our
        /// fly-by-wire link. Born closed: without a window (attitude_surfaces_roll_v1) nothing is
        /// computed. B_y is the law's ctrlState roll column on body y -- the wheels and RCS only under
        /// option 1, which the excitation requires.
        /// </summary>
        static void SurfacesRollExcitation (Entry e, double ut, double dt, bool families, bool lawRoll)
        {
            var sm = e.Sm;
            var x = sm.Roll;
            if (double.IsNaN (x.Until)) {
                x.Value = 0.0;
                x.ValueTick = -1;
                return;
            }
            bool shadowKappa = sm.Mode == SurfaceModel.ModeShadow && sm.Kappa;
            bool loop = e.Mode == ModeArmed && !e.Dropped && e.FlyByWire != null;
            double physical = sm.LawScale > 0.0 ? SurfaceModel.NewtonPerKilonewton / sm.LawScale : 0.0;
            double nonSurface = 0.5 * (e.BPos [3 * SurfaceKappaRoll.RollAxis + SurfaceKappaRoll.RollAxis] +
                e.BNeg [3 * SurfaceKappaRoll.RollAxis + SurfaceKappaRoll.RollAxis]);
            // the roll the estimator counted at this tick, after the leak gate (without it: the
            // tick was counted and roll was among its axes, as before).
            bool counted = sm.Kappa && sm.KappaCounted &&
                (sm.KappaEstimator.LastCountedAxes & (1 << SurfaceKappaRoll.RollAxis)) != 0;
            x.Step (ownTick, ut, shadowKappa, loop, families, lawRoll, sm.RollExponential, sm.KappaEstimator, counted,
                sm.RollPerCommand, physical, nonSurface, sm.RollGainMax, sm.RollSpeedMin, dt, e.Gains.FilterOmegaC,
                e.Gains.FilterZeta, e.FbwApRoll);
        }

        /// <summary>
        /// our fly-by-wire link, after the loop's own roll write: the excitation decided at
        /// this step's Earlyish added to the AutoPilot's roll -- never when the loop wrote the roll
        /// itself this step, never when the loop does not own its link.
        /// </summary>
        static void SurfacesRollFlyByWire (Entry e, FlightCtrlState state)
        {
            var x = e.Sm.Roll;
            if (x.ValueTick != ownTick || x.Value == 0.0 || e.Mode != ModeArmed || e.Dropped ||
                    !double.IsNaN (e.FbwWrittenRoll))
                return;
            double ap = state.roll;
            double value = ap + x.Value;
            value = value > 1.0 ? 1.0 : (value < -1.0 ? -1.0 : value);
            state.roll = (float)value;
            x.LastWritten = state.roll - ap;
            x.Writes++;
        }

        /// <summary>
        /// the client's window -- the excitation is allowed until the game time `utUntil`
        /// (a past time closes it), then the state. Returns [accepted (1), protocol, schema, state,
        /// reasons, until, ut and attitude tick of the last decision, the command of the last step,
        /// its amplitude (command and torque, the law's unit), the plan's torque, horizon batches,
        /// information of one unit batch, F_y (law unit per unit), B_y (N.m per unit), leak ratio,
        /// leak max, steps, episodes, plans, writes, last written, window calls, the estimator's batch and a
        /// plan of N_h batches in game seconds (the client's margin before the ignition)].
        /// </summary>
        internal static IList<double> SurfacesRollWindow (string token, double utUntil)
        {
            if (double.IsNaN (utUntil) || double.IsInfinity (utUntil))
                throw new ArgumentOutOfRangeException (nameof (utUntil), BridgeErrors.Tag (
                    BridgeErrors.UnsupportedMode, "UT de fin de fenetre fini attendu"));
            var entry = RequireToken (token);
            var x = entry.Sm.Roll;
            x.Until = utUntil;
            x.WindowCalls++;
            return RollReply (x);
        }

        static List<double> RollReply (SurfaceKappaRollState x)
        {
            return new List<double> (SurfaceKappaRoll.ReplyLength) {
                1.0, SurfaceKappaRoll.Protocol, SurfaceKappaRoll.Schema, x.State, x.Reasons, x.Until, x.LastUt,
                x.LastTick, x.Value, x.Amplitude, x.AmplitudeTorque, x.PlanTorque, x.Horizon,
                x.BatchInformationValue, x.SurfacePerUnit, x.NonSurfacePerUnit, x.LeakRatio, SurfaceKappaRoll.LeakMax,
                x.Steps, x.Episodes, x.Plans, x.Writes, x.LastWritten, x.WindowCalls, x.BatchSeconds, x.PlanSeconds
            };
        }

        /// <summary>deflectionDirection as it was before the lot-1 hold of this step.</summary>
        static double NeutralizedOriginal (Entry e, ModuleControlSurface s, double fallback)
        {
            int k = e.Neutralized.IndexOf (s);
            if (k < 0 || k >= e.NeutralizedOriginal.Count)
                return fallback;
            object o = e.NeutralizedOriginal [k];
            return o is float ? (float)o : fallback;
        }

        /// <summary>
        /// The command coefficients of FixedCtrlSurfaceUpdate for this step's geometry
        /// (direction: deflectionDirection as the step used it).
        /// </summary>
        static void FillCommand (SurfaceCommand c, ModuleControlSurface s, Transform baseT, Transform reference,
            Vector3 com, double direction)
        {
            Quaternion refRotation = reference.rotation;
            Quaternion baseRotation = baseT.rotation;
            Vector3 hinge = baseRotation * Vector3.right;
            Vector3 local = baseT.InverseTransformPoint (com);
            Vector3 px = refRotation * Vector3.right;
            Vector3 pz = refRotation * Vector3.forward;
            c.CPitch = s.ignorePitch ? 0.0 : Dot (px, hinge);
            c.CYaw = s.ignoreYaw ? 0.0 : Dot (pz, hinge);
            c.Flip = local.y < 0f;
            if (s.ignoreRoll) {
                c.KRoll = 0.0;
            } else {
                // rhs = (local.x, 0, local.z); Vector3.normalized is zero below 1e-5.
                double rx = local.x, rz = local.z;
                double norm = Math.Sqrt (rx * rx + rz * rz);
                Vector3 upLocal = Quaternion.Inverse (baseRotation) * reference.up;
                double cosine = norm > 1e-5 ? (rx * upLocal.x + rz * upLocal.z) / norm : 0.0;
                double sideUp = Dot (baseT.up, reference.up);
                c.KRoll = rx * (1.0 - (Math.Abs (cosine) * 0.5 + 0.5)) * SurfaceModel.Sign (sideUp) *
                    SurfaceModel.Sign (s.ctrlSurfaceRange);
            }
            c.Range = s.ctrlSurfaceRange;
            c.Authority = s.authorityLimiter;
            c.Direction = direction;
            c.Speed = s.actuatorSpeed;
            c.Exponential = s.useExponentialSpeed;
            c.DeployTerm = 0.0;
            if (s.deploy) {
                double num;
                if (!s.usesMirrorDeploy) {
                    Vector3 offset = Quaternion.Inverse (refRotation) * (baseT.position - com);
                    num = (s.deployInvert ? -1.0 : 1.0) * SurfaceModel.Sign (offset.x);
                } else {
                    num = (s.deployInvert ? -1.0 : 1.0) * (s.partDeployInvert ? -1.0 : 1.0) *
                        (s.mirrorDeploy ? -1.0 : 1.0);
                }
                c.DeployTerm = -num * s.deployAngle;
            }
        }

        /// <summary>The flow and geometry of this step (the module's own fields), world frame.</summary>
        static void FillFlow (SurfaceRecord r, ModuleControlSurface s, Transform baseT, Part part, Vector3 com,
            double liftMultiplier, double dragMultiplier)
        {
            var f = r.Flow;
            f.Area = s.ctrlSurfaceArea;
            f.LiftCoeff = s.deflectionLiftCoeff;
            f.LiftMultiplier = liftMultiplier;
            f.DragMultiplier = dragMultiplier;
            f.PerpendicularOnly = s.perpendicularOnly;
            f.Omnidirectional = s.omnidirectional;
            f.UseDrag = s.useInternalDragModel;
            f.NodeBlocked = false;      // nodeEnabled surfaces are not modeled (StaticReasons)
            Vector3 n = getNVel (s);
            f.NVel [0] = n.x;
            f.NVel [1] = n.y;
            f.NVel [2] = n.z;
            f.QLift = getQLift (s);
            f.QDrag = getQDrag (s);
            f.Mach = r.MachFromEarlyish ? r.MachEarlyish : part.machNumber;
            Vector3 axis;
            if (s.transformDir == ModuleLiftingSurface.TransformDir.Z)
                axis = baseT.forward;
            else if (s.transformDir == ModuleLiftingSurface.TransformDir.Y)
                axis = baseT.up;
            else
                axis = baseT.right;
            double sign = s.transformSign;
            f.LiftVector0 [0] = axis.x * sign;
            f.LiftVector0 [1] = axis.y * sign;
            f.LiftVector0 [2] = axis.z * sign;
            Vector3 hinge = baseT.rotation * Vector3.right;
            f.Hinge [0] = hinge.x;
            f.Hinge [1] = hinge.y;
            f.Hinge [2] = hinge.z;
            var partT = part.partTransform;
            Vector3 col = partT != null ? partT.TransformPoint (part.CoLOffset) : baseT.position;
            Vector3 cop = partT != null ? partT.TransformPoint (part.CoPOffset) : baseT.position;
            f.LeverLift [0] = col.x - com.x;
            f.LeverLift [1] = col.y - com.y;
            f.LeverLift [2] = col.z - com.z;
            f.LeverDrag [0] = cop.x - com.x;
            f.LeverDrag [1] = cop.y - com.y;
            f.LeverDrag [2] = cop.z - com.z;
        }

        static double Dot (Vector3 a, Vector3 b)
        {
            return (double)a.x * b.x + (double)a.y * b.y + (double)a.z * b.z;
        }

        /// <summary>
        /// The moving part is turned in this step: `ctrlSurface.localRotation != neutral`, the very
        /// expression of ModuleControlSurface.FixedUpdate, evaluated by Unity's own operator.
        ///
        /// (a recorded flight, 3 captures of 461 with an identity of 0.19-0.37,
        /// each on one fin at |deflection| 0.1639-0.1653 deg): Unity's operator is Dot, a method
        /// that returns float32 (its sum is rounded to float there), then `dot &gt; 0.999999f`. A copy
        /// written inline (the sum compared straight off the evaluation stack, no store, no return)
        /// is not rounded when the runtime keeps float arithmetic in double, as KSP's Mono does: in
        /// the half-ulp band just above the threshold (the double sum within 3e-8 above 0.999999f:
        /// about 0.0024 deg of deflection, between 0.160 and 0.167 deg depending on how the fin's
        /// neutral rounds) the copy said "not turned" while the module had turned the moving part,
        /// and the replica was judged against the wrong force for that step. The direction of the
        /// three errors (copy "equal", module "different", never the converse in 121 judged turned
        /// steps) is the signature of that rounding. Calling the same operator rounds where the
        /// module rounds, whatever the JIT does.
        /// </summary>
        static bool RotationTest (Quaternion local, Quaternion neutral)
        {
            return local != neutral;
        }

        static void Body (Transform reference, double x, double y, double z, double[] into)
        {
            Vector3 v = reference.InverseTransformDirection (new Vector3 ((float)x, (float)y, (float)z));
            into [0] = v.x;
            into [1] = v.y;
            into [2] = v.z;
        }

        /// <summary>The capture's body frame for BodyExact: the reference's rotation, normalized and inverted in double.</summary>
        static void SetBodyInverse (Quaternion q)
        {
            double n = Math.Sqrt ((double)q.x * q.x + (double)q.y * q.y + (double)q.z * q.z + (double)q.w * q.w);
            if (!(n > 0.0)) {
                capQx = capQy = capQz = 0.0;
                capQw = 1.0;
                return;
            }
            capQx = -q.x / n;
            capQy = -q.y / n;
            capQz = -q.z / n;
            capQw = q.w / n;
        }

        /// <summary>
        /// World to body in double, in place (InverseTransformDirection without its float
        /// rounding): v' = v + 2w (u x v) + 2 u x (u x v), (u, w) the inverse rotation.
        /// </summary>
        static void BodyExact (double[] v)
        {
            double tx = 2.0 * (capQy * v [2] - capQz * v [1]);
            double ty = 2.0 * (capQz * v [0] - capQx * v [2]);
            double tz = 2.0 * (capQx * v [1] - capQy * v [0]);
            double x = v [0] + capQw * tx + (capQy * tz - capQz * ty);
            double y = v [1] + capQw * ty + (capQz * tx - capQx * tz);
            double z = v [2] + capQw * tz + (capQx * ty - capQy * tx);
            v [0] = x;
            v [1] = y;
            v [2] = z;
        }

        /// <summary>
        /// diagnostic, a surface inside the rotation band: does Unity's test of the bare
        /// rotation the module computed, AngleAxis(deflection, right) * neutral, answer as the
        /// rotation the transform stored? (The guard predicts from the bare one.) The first three
        /// disagreements of an arming go to KSP.log; nothing else is published.
        /// </summary>
        static void BandDiagnostic (SurfaceState sm, SurfaceRecord r, float deflection, Quaternion neutral)
        {
            sm.BandCaptures++;
            bool bare = RotationTest (Quaternion.AngleAxis (deflection, Vector3.right) * neutral, neutral);
            if (bare == r.Rotated)
                return;
            if (++sm.BareDisagreements <= 3)
                BridgeLog.Info ("SurfaceModelV1 (D159): part " + r.PartPersistentId + " at " +
                    deflection.ToString ("R", System.Globalization.CultureInfo.InvariantCulture) +
                    " deg: the stored rotation tests " + (r.Rotated ? "turned" : "not turned") +
                    ", the bare AngleAxis(deflection) * neutral the opposite (" + sm.BareDisagreements + " of " +
                    sm.BandCaptures + " captures in the rotation band)");
        }
    }
}
