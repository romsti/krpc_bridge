using System.Collections.Generic;
using KRPC.Service.Attributes;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;
using KRPC.Bridge.Model;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1 RPCs: the physics-rate attitude loop, in shadow (M2) or armed on the roll
    /// only (M3). See AttitudeV1.cs, AttitudeWrite.cs and docs/API.md.
    /// </summary>
    public static partial class ControlService
    {
        /// <summary>Protocol version of the AttitudeV1 frames and status.</summary>
        [KRPCProperty]
        public static int AttitudeProtocolVersion {
            get { return AttitudeV1.ProtocolVersion; }
        }

        /// <summary>
        /// Arms the AttitudeV1 loop for the loaded vessel persistentId (decimal string),
        /// active or not, for leaseSeconds of real time (0.2-10 s). mode 0 = shadow: the
        /// loop computes and publishes, it writes nothing. mode 1 = armed on the channels of
        /// axesMask (bit 1 roll, bit 2 pitch/yaw): the roll is written into ctrlState.roll
        /// after the kRPC AutoPilot in the fly-by-wire chain; pitch/yaw leases the gimbals
        /// under the loop's own actuator authority (refused here if a v2 owner or a legacy
        /// lease is present) and writes ctrlState.pitch/yaw after the AutoPilot. Each channel
        /// writes only when a fresh V2 reference allows it and no fallback motif is set.
        /// At most two vessels. Re-arming a vessel returns a new token and invalidates the
        /// old one.
        /// </summary>
        [KRPCProcedure]
        public static string AttitudeArmV1 (string persistentId, string owner,
            float leaseSeconds = 1f, int mode = 0, int axesMask = 3)
        {
            return AttitudeV1.Arm (persistentId, owner, leaseSeconds, mode, axesMask);
        }

        /// <summary>
        /// (born closed): AttitudeArmV1 plus inactiveOptions, for the NON-ACTIVE booster of
        /// a double flight. 0 = exactly AttitudeArmV1. Bit 1 (needs mode 1 with axes bit 2): the
        /// pitch/yaw channel of this vessel, while it is loaded and NOT the active vessel, leases
        /// its gimbals under its own per-vessel authority instead of the active vessel's (which it
        /// never gets: every step fell back with motif 28); the active vessel keeps the unchanged
        /// path, and becoming active hands the per-vessel leases back first. Bit 2 (with bit 1,
        /// options = 3): the lateral force (AttitudeTranslationV1) may run there too. With bit 1,
        /// a vessel loaded and not active at arming skips the check "another owner of the active
        /// vessel's actuators is present". Other values are refused.
        /// </summary>
        [KRPCProcedure]
        public static string AttitudeArmV2 (string persistentId, string owner,
            float leaseSeconds = 1f, int mode = 0, int axesMask = 3, int inactiveOptions = 0)
        {
            return AttitudeV1.Arm (persistentId, owner, leaseSeconds, mode, axesMask,
                inactiveOptions);
        }

        /// <summary>
        /// read-only: the per-vessel authorities AttitudeV1 holds on non-active vessels.
        /// [version (1), count, stride (6)], then per authority: vessel persistentId, gimbals
        /// leased, throttles leased, real-time lease left (s), the vessel is the active one now
        /// (handed back at the next step), held.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeInactiveStatusV1 ()
        {
            return Actuation.InactiveStatus ();
        }

        /// <summary>
        /// (born closed, reset to 0 at every arming): how the gimbal modules with SEVERAL
        /// pivots (gimbalTransforms) are driven by an armed vessel. mode 0 = V1: one (x, y) on
        /// every pivot, in the pivot's own frame -- on a cluster whose pivots are turned about
        /// the thrust axis (Near Future's AR-1 Cluster) the nozzles tilt apart, a swirl: roll
        /// torque and no pitch/yaw. mode 1 = such a module is mapped per pivot as stock does
        /// (the module's (x, y) is a rotation in pivot 0's frame, each pivot turns by its
        /// projection), in the columns, the lease and the lateral-force geometry; frames then
        /// carry capability bit 9. mode 2 (born closed) = mode 1, and the law's authority of
        /// a mapped module is its exact largest realized torque over its box instead of its free
        /// columns times the span; frames then also carry capability bit 10. Modules with one
        /// pivot, or with parallel pivots, are the same in every mode. Returns [accepted (1),
        /// version (1), mode, gimbal modules, modules with two pivots or more, modules mapped in
        /// mode 1 or 2, most pivots in one module, attitude tick] (counts NaN while the vessel is
        /// not loaded).
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeGimbalPivotsV1 (string token, int mode)
        {
            return AttitudeV1.GimbalPivotsConfig (token, mode);
        }

        /// <summary>
        /// One reference per GNC tick, the one given to the kRPC AutoPilot: frame and
        /// direction as AutoPilot.ReferenceFrame / TargetDirection, rollMode 0 = roll-rate
        /// damping (target_roll NaN), omegaFf = rate of the wanted direction (rad/s, same
        /// frame), utStamp = UT of the state the guidance used, valid until utValidUntil,
        /// omegaMax = AutoPilot max angular velocity (rad/s, 0 = default). An accepted
        /// reference renews the lease. Returns 1 accepted, 0 ignored (sequence not newer).
        /// </summary>
        [KRPCProcedure]
        public static int AttitudeReferenceV1 (string token, int sequence, SCReferenceFrame frame,
            double dirX, double dirY, double dirZ, int rollMode, double omegaFfX,
            double omegaFfY, double omegaFfZ, double utStamp, double utValidUntil,
            double omegaMax)
        {
            return AttitudeV1.Reference (token, sequence, frame, dirX, dirY, dirZ, rollMode,
                omegaFfX, omegaFfY, omegaFfZ, utStamp, utValidUntil, omegaMax);
        }

        /// <summary>
        /// The V1 reference plus the AutoPilot's max angular velocity per body axis
        /// (omegaMaxX pitch, omegaMaxY roll, omegaMaxZ yaw, rad/s, 0 = its default), for the
        /// per-axis omega guard, and writeMask, the write permission of this reference: bit 1
        /// lets an armed (mode 1) loop write the roll while the reference is valid, bit 2
        /// pitch/yaw (while it is younger than 0.3 s), 0 keeps it in shadow. The law's rate
        /// limit stays max(omegaMaxX, omegaMaxZ), as in V1. Returns 0 ignored (sequence not
        /// newer), else 1, plus 2 if the loop wrote pitch/yaw at the last physics step, plus 4
        /// if it wrote the roll.
        /// </summary>
        [KRPCProcedure]
        public static int AttitudeReferenceV2 (string token, int sequence, SCReferenceFrame frame,
            double dirX, double dirY, double dirZ, int rollMode, double omegaFfX,
            double omegaFfY, double omegaFfZ, double utStamp, double utValidUntil,
            double omegaMaxX, double omegaMaxY, double omegaMaxZ, int writeMask)
        {
            return AttitudeV1.ReferenceV2 (token, sequence, frame, dirX, dirY, dirZ, rollMode,
                omegaFfX, omegaFfY, omegaFfZ, utStamp, utValidUntil, omegaMaxX, omegaMaxY,
                omegaMaxZ, writeMask);
        }

        /// <summary>
        /// born closed: AttitudeReferenceV2 plus a roll mode and an up vector (upX, upY, upZ, in frame).
        /// rollMode 0 is exactly V2 (the up vector ignored: the roll is rate damping). rollMode 1, refused without
        /// option 1024 of attitude_options_v1, points the roll: the loop's roll reference brings the vessel's roof (-z)
        /// onto the up vector's part across the nose, with the square-root profile of the other axes on the roll's own
        /// authority and AutoPilot limit (omegaMaxY); the roll is written by the roll channel (writeMask bit 1). rollMode
        /// 2 (option 1024 too) is the SHADOW of mode 1: the roll error published in the frame's error y, the law exactly
        /// mode 0's. The up vector need not be normalized nor across the nose. Returns as AttitudeReferenceV2.
        /// </summary>
        [KRPCProcedure]
        public static int AttitudeReferenceV3 (string token, int sequence, SCReferenceFrame frame,
            double dirX, double dirY, double dirZ, int rollMode, double upX, double upY, double upZ,
            double omegaFfX, double omegaFfY, double omegaFfZ, double utStamp, double utValidUntil,
            double omegaMaxX, double omegaMaxY, double omegaMaxZ, int writeMask)
        {
            return AttitudeV1.ReferenceV3 (token, sequence, frame, dirX, dirY, dirZ, rollMode, upX, upY, upZ,
                omegaFfX, omegaFfY, omegaFfZ, utStamp, utValidUntil, omegaMaxX, omegaMaxY, omegaMaxZ, writeMask);
        }

        /// <summary>
        /// BORN CLOSED (refused without option 2048 of attitude_options_v1): the REFERENCE RING, 2 to 64 dated
        /// samples [ut, dir_x, dir_y, dir_z, a_x, a_y, a_z, u] in frame (flat, ut strictly increasing). Three channels,
        /// each finite in every sample or NaN in every one: the wanted nose (consecutive samples under 90 deg apart),
        /// the lateral acceleration of the translation request (m/s^2) and the main throttle (0-1). At every physics
        /// step whose UT lies in the ring's span the loop serves, by interpolation (nlerp for the nose, linear for the
        /// rest), the ring's direction instead of the standing reference's (with no omega_ff), its lateral
        /// acceleration instead of the live request's, and its throttle to a live ramp order of attitude_throttle_v1
        /// (motif 18); outside the span, nothing (the loop as without a ring). The ring renews neither the lease nor
        /// the standing reference's validity or age. A sequence not newer reads; an empty list clears the ring.
        /// Returns layout attitude_reference_ring_v1 (see docs/API.md).
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeReferenceRingV1 (string token, int sequence, SCReferenceFrame frame,
            IList<double> samples)
        {
            return AttitudeV1.ReferenceRingV1 (token, sequence, frame, samples);
        }

        /// <summary>
        /// Lateral force (chantier 19 suite), born closed. (ax, ay, az) = the TOTAL horizontal
        /// acceleration wanted from the engines (nose x thrust + translation), m/s^2, in frame
        /// (the frame of the V2 reference); valid until utValidUntil (game time). mode 0 stop,
        /// 1 differential thrust as two motif columns of the attitude allocation, 3 differential
        /// + translation by a common gimbal deflection whose torque the differential cancels in
        /// advance (2 alone is refused). Acts only on a vessel armed (mode 1) on pitch/yaw while
        /// its pitch/yaw channel writes; a stale request or mode 0 winds the offsets down and
        /// gives the throttles back. Returns [accepted 0/1, lateral authority now (m/s^2, 0 if
        /// unavailable), differential torque authority (N.m), realized common gimbal fraction,
        /// layout version (1), state, reason bits, wanted lateral accel, commanded lateral
        /// accel, realized lateral accel (m/s^2), full common-deflection torque, cancelled torque
        /// (N.m), motif engines, main throttle, attitude tick, UT, throttle writes]: the values of
        /// the last physics step. Bit 4 "computed levers" (born closed): mode 5 = 1 and mode 7 =
        /// 3 with a computed motif-lever criterion, the RCS translation (ctrlState.X/Y) as a
        /// lever, a joint allocation (tilt + RCS + differential, zero torque) and the largest
        /// torque-free lateral acceleration as the authority; the return is then layout 2, eight
        /// values appended (authority of the differential + tilt, authority with the RCS, RCS X
        /// and Y written, RCS translation force capacity (N), motif lever (m), residual pitch/yaw
        /// torque after the feed-forward (N.m), frac_rcs). Bits 16 and 32 "honest translation"
        /// (born closed, only on top of 5 or 7: modes 21, 23, 37, 39, 53, 55): bit 16 shares the
        /// RCS with the rotation (translation columns of the nozzles the served rotation leaves
        /// linear, the law's RCS pitch/yaw left out while ModuleRCS's combined rotation makes them
        /// nonlinear, no roll on the gimbals while the law does not write the roll), bit 32
        /// publishes the authority times the measured share of the common deflection the gimbals
        /// hold; the return is then layout 3, seven values appended (the model's authority, the
        /// measured share (NaN without evidence), its evidence, the RCS capacity without the
        /// headroom (N), the weaker-sign share kept, the largest held rotation fraction, the
        /// bit-16 flags). See docs/API.md.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeTranslationV1 (string token, int sequence,
            SCReferenceFrame frame, double ax, double ay, double az, double utStamp,
            double utValidUntil, int mode)
        {
            return AttitudeV1.Translation (token, sequence, frame, ax, ay, az, utStamp,
                utValidUntil, mode);
        }

        /// <summary>
        /// Refactor R1: AttitudeTranslationV1 with a return in ONE layout whatever the mode and
        /// the options (attitude_translation_v2, protocol 1): the 32 fields of layout 3 (a field
        /// the stored mode does not fill is NaN; offset 4 still says which part it fills: 1, 2 or
        /// 3), then the evidence block [protocol (1), schema (1), n (11), n values] (NaN values
        /// without option 2). Same arguments, same effect. Field names:
        /// conn.bridge.layout_v1("attitude_translation_v2").
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeTranslationV2 (string token, int sequence,
            SCReferenceFrame frame, double ax, double ay, double az, double utStamp,
            double utValidUntil, int mode)
        {
            return AttitudeV1.TranslationV2 (token, sequence, frame, ax, ay, az, utStamp,
                utValidUntil, mode);
        }

        /// <summary>
        /// Tunables of the lateral-force layer of one armed vessel (reset at every arming):
        /// fracGimbal (0-0.95, share of each gimbal's range the common deflection may use,
        /// default 0.8), fracDiff (0-0.95, share of the throttle margin the feed-forward may
        /// use, 0.8), diffPriority (0.1-10, WLS priority of the motif columns; gimbals 1,
        /// ctrlState 3; default 0.5). Returns the effective values then the fixed ones.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeTranslationConfigV1 (string token, double fracGimbal,
            double fracDiff, double diffPriority)
        {
            return AttitudeV1.TranslationConfig (token, fracGimbal, fracDiff, diffPriority);
        }

        /// <summary>
        /// Lateral-force frames of one armed vessel newer than sinceTick (attitude tick), in
        /// the six-value history header + length-prefixed layout of DynamicsFramesV3 (own
        /// protocol 1, schema 1; see docs/API.md). Empty when never requested.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeTranslationFramesV1 (string persistentId,
            int sinceTick = 0, int maxFrames = 64)
        {
            return AttitudeV1.ReadTranslationFrames (persistentId, sinceTick, maxFrames);
        }

        /// <summary>
        /// BORN CLOSED (option 4096 of attitude_options_v1): the engine-by-engine allocation of the last
        /// physics step, IN SHADOW (nothing is written): [block_protocol (1), block_schema (1), block_n, block_n
        /// values], field names conn.bridge.layout_v1("attitude_allocation"). One problem for every actuator (the
        /// throttle of each engine, the gimbals, the RCS rotation and translation), rows the force and the torque,
        /// strict priorities (the axial force of the served main throttle, then pitch and yaw, then the lateral
        /// force of the live translation request, then roll), the realized change at a common horizon (each
        /// actuator's Lerp); the commands, the residuals, and the lateral authority with the attitude kept (free)
        /// and from the trim at main throttle levels (model). State 0 and NaN values without the option.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeAllocationV1 (string token)
        {
            return AttitudeV1.AllocationV1 (token);
        }

        /// <summary>
        /// the shadow allocation's frames of one armed vessel newer than sinceTick (attitude tick), in the
        /// six-value history header + length-prefixed layout of DynamicsFramesV3 (own protocol 1, schema 1, layout
        /// attitude_allocation_frames_v1). Empty without option 4096.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeAllocationFramesV1 (string persistentId, int sinceTick = 0,
            int maxFrames = 64)
        {
            return AttitudeV1.ReadAllocationFrames (persistentId, sinceTick, maxFrames);
        }

        /// <summary>
        /// Refactor lot 1, BORN CLOSED (every arming resets the options to 0, which is exactly the
        /// behaviour without this call). Options of an armed vessel, bits: 1 = per-family blocking
        /// (the model motifs 13, 14 and 20 become informational; 13 and 20 freeze the
        /// ctrlState columns and the leased gimbals are allocated alone; the control surfaces
        /// leave the law's columns and, in each step where the loop writes ctrlState pitch/yaw,
        /// are held neutral for that physics step only; frame motif bits 32-36, capability bit
        /// 11); 2 = evidence aging (the measured share of the common deflection forgets in
        /// game time without samples, the published factor relaxes toward the model, and the
        /// AttitudeTranslationV1 return carries the named evidence block; capability bit 13);
        /// 4 = the non-converged WLS iterate is written (motif 15 informational; capability bit
        /// 12). Returns [accepted (1), version (1), options now, supported options, surfaces
        /// answering a ctrlState axis, of which neutralizable, axes they answer (bit 0 pitch, 1
        /// roll, 2 yaw), attitude tick]; the surface counts are NaN while the vessel is not
        /// loaded. Other bits are refused.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeOptionsV1 (string token, int options)
        {
            return AttitudeV1.SetOptions (token, options);
        }

        /// <summary>
        /// Refactor lot 1, read-only, no token: [version (1), options AttitudeOptionsV1 accepts,
        /// evidence block protocol (1), its schema, its field count, surface neutralizer available
        /// (1, 0 if ModuleControlSurface.deflectionDirection is missing)]. A DLL without this
        /// procedure has none of the options.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeCapabilitiesV1 ()
        {
            return AttitudeV1.Capabilities ();
        }

        /// <summary>
        /// The field names of a named layout, in order (fields are only ever appended; the
        /// block header says the schema and how many). "translation_evidence" (the block
        /// AttitudeTranslationV1 appends with option 2) as before; since refactor R1 every layout
        /// of conn.bridge.layouts_v1 (frames, returns, status replies, blocks), sections
        /// concatenated; conn.bridge.layout_v1 gives the sections, strides and units. Unknown
        /// names are refused ([NOT_FOUND]).
        /// </summary>
        [KRPCProcedure]
        public static IList<string> AttitudeLayoutV1 (string name)
        {
            return AttitudeV1.Layout (name);
        }

        /// <summary>
        /// Refactor R1, BORN CLOSED (every arming resets it to 0): the effector registry of an
        /// armed vessel. mode 1 = every column of the allocation gets a model status (exact,
        /// measured, approximate, unmodeled) and a family is sanctioned only on its own columns:
        /// a ctrlState axis is frozen only for what is unmodeled on that axis (an RCS module with
        /// fullThrust that answers it, a control surface that answers it and cannot be
        /// neutralized; all three while the fly-by-wire order is not verified); RCS in precision
        /// mode without useLever keeps its columns scaled by precisionFactor (exact); control
        /// surfaces leave the columns and are held neutral while the loop writes an axis they
        /// answer (the mechanics of option 1, which the registry implies; frame capability bits
        /// 11 and 14). The authority of each family is computed every step and published by
        /// AttitudeEffectorsReadV1. mode 0 = off. Returns [accepted (1), table protocol (1),
        /// mode now, attitude tick]. Other modes are refused.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeEffectorsV1 (string token, int mode)
        {
            return AttitudeV1.EffectorsConfig (token, mode);
        }

        /// <summary>
        /// Read-only: the effector registry of the last physics step of an armed vessel.
        /// Header [protocol (1), schema (1), enabled, attitude tick, families (5), family
        /// stride (12), columns, column stride (10), frozen ctrlState axes (bit 0 pitch, 1 roll,
        /// 2 yaw), surfaces held neutral]; one row per family (0 gimbal, 1 rcs, 2 wheel, 3
        /// surface, 4 diff_motif): [family, status (-1 absent, 0 exact, 1 measured, 2
        /// approximate, 3 unmodeled), modules, scale, authority x, y, z (N.m), alpha x, y, z
        /// (rad/s2), columns, frozen columns]; one row per allocation column: [column, kind (1
        /// gimbal, 3 ctrlState, 4 differential motif), family, axis, status, action (0
        /// allocated, 1 frozen, 2 bounded), B x, y, z, half span]. The layout never depends on
        /// the mode: NaN and no column rows while the registry is off or the vessel not armed.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeEffectorsReadV1 (string persistentId)
        {
            return AttitudeV1.EffectorsRead (persistentId);
        }

        /// <summary>
        /// Lot 3 (3.1), BORN CLOSED (every arming resets it to 0, and the scale guard to k = 1):
        /// SurfaceModelV1, the exact replica of every stock control surface of an armed vessel.
        /// mode 1 = shadow: at the end of every physics step each surface's realized force is
        /// compared with the replica fed the same inputs (identity), its target with the command
        /// law's (from this step's and the previous step's ctrlState), and its exact column, its
        /// command gain, its realized torque and the mono-flight scale guard are computed and
        /// published by SurfaceModelV1; nothing is written. mode 2 = armed: implies the family
        /// mechanics of AttitudeOptionsV1 bit 1; while the same-step response is established, no
        /// axis of the scale guard is out of [0.5, 2] and a surface's replica holds, that surface
        /// is an actuator of the law (its sub-columns in the ctrlState columns, its realized torque
        /// in M, its free torque off the allocation target) instead of being held neutral; every
        /// other surface is held neutral as with bit 1. mode 0 = off. (born closed): mode 1 or
        /// 2 ORed with 4, the surfaces' torques enter the law and the publication in N.m (KSP's
        /// forces are in kN; without the flag they stay in kN.m, as up to ad35d85). (born
        /// closed): ORed with 8, the physical slope kappa of every body axis is estimated
        /// (SurfaceKappaV1) and, armed, a surface is engaged only on axes whose kappa is proven
        /// within [0.5, 2]; an axis out of tolerance keeps every surface acting on it out for the
        /// arming. (born closed): with 8, ORed with 16, the leak gate of kappa's counted
        /// steps (a step counts on an axis only while the surfaces carry at least 2/3 of the excited
        /// torque increment); 16 without 8 is refused. (born closed): with 8, ORed with 32,
        /// kappa's measured side on the physical inertia (Vessel.MOI plus the parallel-axis terms its
        /// stock sum leaves out at the parts' CoMOffset); 32 without 8 is refused. Returns
        /// [accepted (1), protocol (1), mode now (with its flags), surfaces answering a ctrlState
        /// axis, of which modeled, attitude tick] (NaN counts while the vessel is not loaded).
        /// Other modes are refused.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeSurfacesV1 (string token, int mode)
        {
            return AttitudeV1.SurfacesConfig (token, mode);
        }

        /// <summary>
        /// Read-only: SurfaceModelV1 of the last physics step of an armed vessel (layout
        /// surface_model_v1, conn.bridge.layout_v1). Header [protocol (1), schema (1), enabled,
        /// mode, attitude tick, ut, physics dt, vessel persistentId, surface count, row stride (48),
        /// modeled, engaged, held neutral, same-step answers, previous-step answers, neither,
        /// same-step established, identity max, surfaces judged, sanctions, scale k x y z, scale
        /// steps x y z, realized torque x y z (N.m, body), free torque x y z, residual of the law x
        /// y z (rad/s2), residual with the surfaces x y z, filtered x y z, static authority x y z
        /// (N.m), ctrlState pitch roll yaw, Earlyish tick]; one row per control surface (part
        /// persistentId, module index, status, reasons, axes, engaged, neutralized, shielded,
        /// deflections, targets and predictions in rad, rate, range, authority limiter (%),
        /// deflectionDirection, Qlift and Qdrag (Pa), Mach, rotated, realized and model force x y z
        /// (N, body), identity, realized torque x y z (N.m), column x y z (N.m/rad), gain pitch roll
        /// yaw (rad per unit), identity strikes, Mach source, lift coefficient, moving area, drag
        /// model, exponential speed, deploy term (rad)); then the master tick. The layout never
        /// depends on the mode: NaN and no rows while the model is off or the vessel not armed.
        /// Units: forces in N and torques, columns and authority in N.m only with the mode
        /// flag 4; without it they are KSP's own kN and kN.m (1000 times smaller).
        /// </summary>
        [KRPCProcedure]
        public static IList<double> SurfaceModelV1 (string persistentId)
        {
            return AttitudeV1.SurfacesRead (persistentId);
        }

        /// <summary>
        /// Read-only, (attitude_surfaces_v1 mode ORed with 8, born closed): the PHYSICAL slope
        /// kappa of SurfaceModelV1 per body axis -- what the vessel's measured angular acceleration
        /// attributes to the surfaces' deflections over the torque the law is given for them (1 when
        /// the law's unit, levers and frames are right, 1000 for an earlier change). Layout surface_kappa_v1
        /// (conn.bridge.layout_v1): header [protocol (1), schema (4), enabled, mode, attitude tick of
        /// the last step, ut, vessel persistentId, batch steps, base batch steps, batch capacity,
        /// false alarm, band min, band max (the proof band), steps seen, steps counted, filter restarts, engine-quiet
        /// steps, parts read, lifting modules, aerodynamic readout cost last / max (us), axes the
        /// surfaces can act on this step, proven axes, out-of-tolerance axes, axis count (3), axis
        /// stride]; one row per body axis [axis, state
        /// (0 not proven, 1 proven, 2 out of tolerance), proven, out, consistent with 1, kappa, its
        /// standard error (jackknife), t, tolerance (t x SE), kappa low, kappa high, (kappa - 1) / SE,
        /// nuisance slope and its SE, regressors, steps, batches, effective batches, rms of the
        /// left-hand side, of the deflection regressor, of the nuisance, of the residual; the
        /// out-of-tolerance test's degrees of freedom (the jackknife's own), quantile, low and high,
        /// and the rms of the filtered gyroscopic increment the left-hand side includes; an earlier change
        /// (schema 3): the leak gate (1 with the mode flag 16), the steps it refused, and the leak
        /// slope beta of the counted steps (sum d m / sum d^2); (schema 4): the physical inertia (1
        /// with the mode flag 32) and the inertia Vessel.MOI leaves out on the axis (kg.m^2, whatever the
        /// flag)]; then the master tick. The layout never depends on the mode: NaN while the flag is off.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> SurfaceKappaV1 (string persistentId)
        {
            return AttitudeV1.SurfacesKappaRead (persistentId);
        }

        /// <summary>
        /// BORN CLOSED (no window before this call, every arming closes it): the client's
        /// window for the ROLL excitation of SurfaceModelV1's physical slope kappa
        /// (Model/SurfaceKappaRoll.cs). Until the game time utUntil (a past time closes it), while the
        /// model is in shadow with the kappa flag (attitude_surfaces_v1 mode 13), the loop owns its
        /// fly-by-wire link (roll armed) with the per-family rules, does not write the roll itself,
        /// roll is neither proven nor any axis out of tolerance, the estimator counts roll and the
        /// surfaces carry at least two thirds of the roll torque a roll command excites, the loop
        /// adds to the AutoPilot's ctrlState.roll a square wave switching twice per kappa batch:
        /// amplitude from the information still missing for the roll interval to fit in the proof
        /// band, spread over the batches whose Student quantile is within the gain margin of the full
        /// store's, bounded by the surfaces' one-step travel and the command headroom; it stops at
        /// once when a condition fails. utUntil must be finite. Returns (layout
        /// attitude_surfaces_roll_v1) [accepted (1), protocol (1), schema (1), state (0 off, 1
        /// waiting, 2 exciting, 3 roll proven, 4 out of tolerance), reasons (bits), until, ut and
        /// attitude tick of the last decision, the command of the last step, its amplitude (command
        /// and torque in the law's unit), the plan's torque, horizon batches, information of a unit
        /// batch, roll torque per unit of the surfaces (law unit) and of the wheels and RCS (N.m),
        /// leak ratio and its maximum, steps, episodes, plans, writes, last written, window calls, the
        /// estimator's batch and a plan of N_h batches in game seconds].
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeSurfacesRollV1 (string token, double utUntil)
        {
            return AttitudeV1.SurfacesRollWindow (token, utUntil);
        }

        /// <summary>
        /// BORN CLOSED (refused without option 512 of attitude_options_v1): an order of the MAIN THROTTLE,
        /// decided by the loop at every physics step (Earlyish, after the law) and written into ctrlState.mainThrottle
        /// by its fly-by-wire link, last in the chain; the client's own throttle flies whenever the loop does not
        /// write. kind 0 stop; 1 dated ramp [u0, slope (1/s)] from utStamp; 2 pulse [u, steps]; 3 cut when s(t) = 0
        /// [u_before, u_after, a_x, a_y, a_z, c, velocity (0 surface, 1 orbital)], s = a . V + c, a in frame; 4
        /// ignition when h &lt;= h_ign(v) [u_before, u_after, h_ref, c0, c1, c2], h = altitude - h_ref, v the vertical
        /// speed, h_ign = c0 + c1 v + c2 v^2; 5 the flip's calage [boostback direction x, y, z, burn axis x, y, z (in
        /// frame), an earlier change's slope (m/s^2, NaN: no push), RCS + wheels authority x, z (N.m, NaN: the loop's ctrlState
        /// columns), an earlier change's verdict (-1, 0, 1)]. Every kind but 0 ends with an earlier change's guard [threshold, chain steps]. The cut
        /// fires at the first step s &lt;= 0 (the client signs s positive before the event), the ignition at the first
        /// step h &lt;= h_ign(v); both stay fired for the order.
        /// Valid until utValidUntil (game time); a renewal of the same kind keeps its runtime (episode, guard, fired
        /// trigger). Returns layout attitude_throttle_v1 (accepted, then the last step's decision; see docs/API.md).
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeThrottleV1 (string token, int sequence, int kind, SCReferenceFrame frame,
            IList<double> parameters, double utStamp, double utValidUntil)
        {
            return AttitudeV1.Throttle (token, sequence, kind, frame, parameters, utStamp, utValidUntil);
        }

        /// <summary>Renews the real-time lease of an armed vessel (0.2-10 s).</summary>
        [KRPCProcedure]
        public static bool AttitudeRenewV1 (string token, float leaseSeconds = 1f)
        {
            return AttitudeV1.Renew (token, leaseSeconds);
        }

        /// <summary>Disarms a vessel. Its ring is dropped.</summary>
        [KRPCProcedure]
        public static bool AttitudeReleaseV1 (string token)
        {
            return AttitudeV1.Release (token);
        }

        /// <summary>
        /// Header: protocol, schema, row count, row stride (22), Earlyish hook registered,
        /// Earlyish calls, pump-fallback ticks, attitude tick, UT. Then one row per armed
        /// vessel, see docs/API.md. Streamable.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeStatusV1 ()
        {
            return AttitudeV1.ReadStatus ();
        }

        /// <summary>
        /// AttitudeV1 frames of one armed vessel newer than sinceTick (attitude tick), in the
        /// six-value history header + length-prefixed layout of DynamicsFramesV3. Read by the
        /// observer process, not by the flight loop. Empty when the vessel is not armed.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeFramesV1 (string persistentId, int sinceTick = 0,
            int maxFrames = 64)
        {
            return AttitudeV1.ReadFrames (persistentId, sinceTick, maxFrames);
        }

        /// <summary>
        /// T-sol-0 order probe, read-only: stamps every TimingManager stage, the Actuators
        /// watcher and the active vessel's fly-by-wire chain for the next frames (1-300)
        /// physics steps. Read the result with attitude_order_probe_read_v1.
        /// </summary>
        [KRPCProcedure]
        public static bool AttitudeOrderProbeV1 (int frames = 50)
        {
            return AttitudeProbe.Start (frames);
        }

        /// <summary>
        /// Order-probe rows: header version (1), still armed, row count, stride (13), then
        /// frame index, call index in the frame, stage, fixedTime, UT, pump physics tick,
        /// render frame, served ctrlState pitch, first gimbal x, first engine thrust, omega x,
        /// first RCS thrust sum, real time.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AttitudeOrderProbeReadV1 ()
        {
            return AttitudeProbe.Read ();
        }

        /// <summary>
        /// read-only: the authority of the gimbals per kN of thrust, without thrust, pivots
        /// included. Every gimbal module of the loaded vessel persistentId (decimal string; empty = the
        /// active vessel), its columns at the REST pose of its pivots for one kN of its part's engines (split
        /// by their maxThrust), body frame, about the current centre of mass, N.m per degree per kN; its box,
        /// its authority per kN (the law's alpha rule) and its exact authority per kN (an earlier change's largest realized
        /// torque for a module with non-parallel pivots, the alpha rule otherwise); for such a module its
        /// per-pivot rows and columns. Header: protocol (1), schema (1), persistentId, found, UT, module count,
        /// module stride (31), pivot row count, pivot stride (12), summed maxThrust of every engine (kN),
        /// of the gimballed parts (kN), the vessel's authority per kN of that sum (x, y, z), its exact
        /// authority per kN (x, y, z), the centre of mass from the reference transform (body, m), every nozzle
        /// carried to rest (1/0). Layout gimbal_authority_v1; see docs/API.md. A vessel not loaded: found 0,
        /// no row.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> GimbalAuthorityV1 (string persistentId)
        {
            return GimbalAuthorityRead.Read (persistentId);
        }
    }
}
