using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // option 16384 of AttitudeOptionsV1, born closed. The RCS of the ctrlState channel as ModuleRCS FIRES it.
    // PURE: no Unity, no KSP type; the game side is Control/AttitudeV1RcsCombined.cs.
    //
    // How stock KSP fires an RCS module (KSP 1.12.5, ModuleRCS decompiled from the installed Assembly-CSharp.dll;
    // ModuleRCSFX inherits both methods):
    //   * Update: inputRot = ReferenceTransform.rotation x (pitch, roll, yaw), each component zero unless enabled
    //     and zero below EPSILON (its square below EPSILON^2);
    //   * FixedUpdate: EVERY nozzle of the module on that ONE vector:
    //       rot = inputRot x unit(ProjectOnPlane(r_i, inputRot)),  f_i = min(1, max(v_i . rot, 0) + max(v_i . lin, 0)),
    //     fullThrust: 1 from fullThrustMin up; CalculateThrust gives nothing at f_i <= EPSILON, else f_i x the
    //     nozzle's full force (and f_i x its full mass flow).
    //   rot is |inputRot| times the unit tangent of the rotation about the axis inputRot THROUGH THE COM: every nozzle
    //   fires at |inputRot| times its alignment with that tangent, whatever the share of each axis in the command.
    //   The torque is therefore NOT the sum of the per-axis columns (AttitudeGeometry.AddRcsColumns: the law's B,
    //   its alpha = sum |B| span / I, and the in-flight torque of an earlier change all assume that sum).
    //
    // The heavy booster: its RCS is a ring far above the CoM, ~h/R = 23 in the units of its columns
    // (pitch 2.78 MN.m, roll 0.243 MN.m per unit). A roll command only rolls while the commanded axis passes INSIDE
    // the ring, |(pitch, yaw)| below ~R/h |roll|; with the law's pitch/yaw above the dead zone the ring fires as
    // pitch/yaw at |u| and the roll vanishes. Measured on the coast of five flights (133906, 120710, 123627, 181655,
    // 175041: the RCS torque realized at step k+1 against the ctrlState served at step k): a pure roll gives its
    // column (alpha 0.2486 rad/s^2 measured for 0.2486 predicted, the inertia exact: omega_dot = tau / I to 0.1 %),
    // pure pitch/yaw 0.97-0.99 of their columns; a MIXED command gives 0.07 of the roll column (alpha 0.015 for 0.25)
    // and 1.5-1.7 times the pitch/yaw columns; the combined rule below (geometry rebuilt on the columns) gives the
    // pitch/yaw within 0.87-1.01 (R2 0.81-0.94 against 0.67-0.73) and the roll within a factor 1.6. The RCS
    // propellant of those coasts follows the combined rule's thrust with one coefficient for the five flights (R2
    // 0.998 over 54 slices of 10 s). The law, believing the sum, saw its pitch/yaw answered x 1.5-4.8 whenever the roll
    // commanded (period-2 chatter at +-1 MN.m on 133906) and its roll answered x 1/15 (a 4.9 deg overshoot on 120710).
    //
    // With the option (Control/AttitudeV1RcsCombined.cs):
    //   1. the IN-FLIGHT torque of (option 32) is the combined rule's, module by module: Torque(next) -
    //      Torque(fired), the commands ModuleRCS.Update makes / made (ModuleTorque, exact);
    //   2. the ctrlState command the step writes is one the RCS ANSWERS (Decide): of the law's command, the same with
    //      its roll held and the same with its pitch/yaw held, the one whose torque by the combined rule (plus the
    //      linear rest of the ctrlState columns: wheels) is nearest, in the law's own metric (the WLS rows: sum of
    //      (Wv_a / I_a x error_a)^2), to the torque the law's INDI expects (the served command's, by the combined rule,
    //      plus the increment through its columns); the cost of holding a part again counted for every step it has
    //      waited (no axis starves); the law's command on a tie, then pitch/yaw held. A held part is written just under
    //      the RCS dead zone (HeldValue: the RCS ignores it, the wheels and the other ctrlState users still get it; on
    //      the heavy booster, which has no wheel, as 0). The law, its B, its alpha and its allocation are untouched (a
    //      served roll answers its column: the measured 0.2486 rad/s^2), and the next step's INDI starts from what was
    //      served. On the heavy booster the RCS then never fires the combinations the flights wasted it on; the roll and
    //      the pitch/yaw take turns as their weighted demands require -- a roll that must brake gets the RCS, pitch/yaw
    //      (60 times the roll's inertia) lose a step now and then.
    // Without the option nothing here runs: the law, its inputs and its outputs are those of before, bit for bit.
    // -----------------------------------------------------------------------------------

    /// <summary>
    /// The nozzles of the RCS modules of a vessel as ModuleRCS fires them, body frame about the CoM: per nozzle its
    /// lever (nozzle - CoM, m), its direction v (thruster up, or forward with useZaxis; the force is -v) and its full
    /// force (N); per module its first nozzle, its count, its enable flags, its EPSILON, fullThrust and fullThrustMin.
    /// Arrays grow with the layout (no allocation per step once grown). Prepare fills each module's per-axis columns.
    /// </summary>
    internal sealed class RcsNozzleSet
    {
        internal int Modules;
        internal int Nozzles;
        internal int[] Start = new int[0];
        internal int[] Count = new int[0];
        internal bool[] EnablePitch = new bool[0];
        internal bool[] EnableRoll = new bool[0];
        internal bool[] EnableYaw = new bool[0];
        internal double[] Epsilon = new double[0];
        internal bool[] FullThrust = new bool[0];
        internal double[] FullThrustMin = new double[0];
        internal double[] Lever = new double[0];
        internal double[] Dir = new double[0];
        internal double[] Power = new double[0];
        // Prepare: each module's per-axis columns, 9 per module (axis a at [9m + 3a .. 9m + 3a + 2]): the torque of
        // the command +1 (ColPos) and per unit of a negative command (ColNeg), the rule of AttitudeGeometry.AddRcsColumns.
        internal double[] ColPos = new double[0];
        internal double[] ColNeg = new double[0];
        // Prepare: the RCS's ctrlState columns, the mean of the halves summed over the modules (the law's B for the RCS),
        // column a at [3a .. 3a + 2].
        internal readonly double[] ColMean = new double[9];
        // Prepare: per axis (0 pitch, 1 roll, 2 yaw), the largest |command| every module that enables it ignores (just
        // under the smallest EPSILON among them; +infinity when none enables it): what a HELD axis is written at, so
        // that the RCS ignores it while the wheels and the other ctrlState users still get it.
        internal readonly double[] HeldLimit = new double[3];

        internal void Clear ()
        {
            Modules = 0;
            Nozzles = 0;
        }

        /// <summary>
        /// Appends a module of `count` nozzles and returns the index of its first nozzle: Lever and Dir (3 per nozzle)
        /// and Power are to be filled from there.
        /// </summary>
        internal int Add (int count, bool enablePitch, bool enableRoll, bool enableYaw, double epsilon,
            bool fullThrust, double fullThrustMin)
        {
            if (count < 0)
                count = 0;
            int m = Modules;
            if (Start.Length <= m) {
                int cap = Math.Max (4, 2 * Start.Length);
                Array.Resize (ref Start, cap);
                Array.Resize (ref Count, cap);
                Array.Resize (ref EnablePitch, cap);
                Array.Resize (ref EnableRoll, cap);
                Array.Resize (ref EnableYaw, cap);
                Array.Resize (ref Epsilon, cap);
                Array.Resize (ref FullThrust, cap);
                Array.Resize (ref FullThrustMin, cap);
                Array.Resize (ref ColPos, 9 * cap);
                Array.Resize (ref ColNeg, 9 * cap);
            }
            int first = Nozzles;
            int need = first + count;
            if (Power.Length < need) {
                int cap = Math.Max (need, 2 * Power.Length);
                Array.Resize (ref Lever, 3 * cap);
                Array.Resize (ref Dir, 3 * cap);
                Array.Resize (ref Power, cap);
            }
            Start [m] = first;
            Count [m] = count;
            EnablePitch [m] = enablePitch;
            EnableRoll [m] = enableRoll;
            EnableYaw [m] = enableYaw;
            Epsilon [m] = epsilon;
            FullThrust [m] = fullThrust;
            FullThrustMin [m] = fullThrustMin;
            Modules = m + 1;
            Nozzles = need;
            return first;
        }
    }

    internal static class RcsCombined
    {
        static bool Finite (double v)
        {
            return !double.IsNaN (v) && !double.IsInfinity (v);
        }

        /// <summary>
        /// One module's torque for the body command (cx, cy, cz) -- what ModuleRCS.Update made of the ctrlState: its
        /// enable flags and dead zone already applied -- ADDED into torque[0..2] (N.m, body, about the CoM); returns
        /// the module's thrust (N, the sum of its nozzles' forces). exact: ModuleRCS.FixedUpdate and CalculateThrust
        /// (fullThrust, nothing at f &lt;= EPSILON); false: the per-axis column rule of AttitudeGeometry.AddRcsColumns
        /// (neither). A lever along the command gives 0, as Unity's normalized of a vector under 1e-5.
        /// </summary>
        internal static double ModuleTorque (RcsNozzleSet s, int m, double cx, double cy, double cz, bool exact,
            double[] torque)
        {
            double n2 = cx * cx + cy * cy + cz * cz;
            if (!(n2 > 0.0) || !Finite (n2))
                return 0.0;
            int start = s.Start [m], count = s.Count [m];
            double eps = s.Epsilon [m];
            bool full = exact && s.FullThrust [m];
            double fullMin = s.FullThrustMin [m];
            double thrust = 0.0;
            for (int k = 0; k < count; k++) {
                int i = start + k;
                double p = s.Power [i];
                if (!(p > 0.0) || !Finite (p))
                    continue;
                double rx = s.Lever [3 * i], ry = s.Lever [3 * i + 1], rz = s.Lever [3 * i + 2];
                double along = (rx * cx + ry * cy + rz * cz) / n2;
                double qx = rx - along * cx, qy = ry - along * cy, qz = rz - along * cz;
                double qn = Math.Sqrt (qx * qx + qy * qy + qz * qz);
                if (!(qn > 1e-5) || !Finite (qn))
                    continue;
                qx /= qn;
                qy /= qn;
                qz /= qn;
                // rot = inputRot x q
                double ox = cy * qz - cz * qy, oy = cz * qx - cx * qz, oz = cx * qy - cy * qx;
                double vx = s.Dir [3 * i], vy = s.Dir [3 * i + 1], vz = s.Dir [3 * i + 2];
                double f = vx * ox + vy * oy + vz * oz;
                if (!(f > 0.0))
                    continue;
                if (f > 1.0)
                    f = 1.0;
                if (full && f >= fullMin)
                    f = 1.0;
                if (exact && !(f > eps))
                    continue;
                double force = p * f;
                thrust += force;
                double fx = -force * vx, fy = -force * vy, fz = -force * vz;
                torque [0] += ry * fz - rz * fy;
                torque [1] += rz * fx - rx * fz;
                torque [2] += rx * fy - ry * fx;
            }
            return thrust;
        }

        /// <summary>What ModuleRCS.Update makes of the ctrlState for module m (enable flags, then its dead zone).</summary>
        internal static void ModuleCommand (RcsNozzleSet s, int m, double pitch, double roll, double yaw, double[] c)
        {
            RcsInFlight.UpdateCommand (pitch, roll, yaw, s.EnablePitch [m], s.EnableRoll [m], s.EnableYaw [m],
                s.Epsilon [m], c);
        }

        /// <summary>
        /// Every module's per-axis columns (ColPos / ColNeg, the AddRcsColumns rule: each module on the unit command
        /// of one axis, 0 for an axis it does not enable). scratch: 3.
        /// </summary>
        internal static void Prepare (RcsNozzleSet s, double[] scratch)
        {
            for (int a = 0; a < 3; a++)
                s.HeldLimit [a] = double.PositiveInfinity;
            for (int k = 0; k < 9; k++)
                s.ColMean [k] = 0.0;
            for (int m = 0; m < s.Modules; m++) {
                for (int a = 0; a < 3; a++) {
                    bool on = a == 0 ? s.EnablePitch [m] : (a == 1 ? s.EnableRoll [m] : s.EnableYaw [m]);
                    for (int sg = 0; sg < 2; sg++) {
                        double sign = sg == 0 ? 1.0 : -1.0;
                        scratch [0] = scratch [1] = scratch [2] = 0.0;
                        if (on)
                            ModuleTorque (s, m, a == 0 ? sign : 0.0, a == 1 ? sign : 0.0, a == 2 ? sign : 0.0, false,
                                scratch);
                        double[] col = sg == 0 ? s.ColPos : s.ColNeg;
                        // ColNeg is per unit of a negative command: the command -1 gave scratch.
                        for (int r = 0; r < 3; r++)
                            col [9 * m + 3 * a + r] = sign * scratch [r];
                    }
                    // Update zeroes a component whose square (in float) is below EPSILON^2: a margin of 1e-5 keeps the
                    // float of the written command strictly under it.
                    for (int r = 0; r < 3; r++)
                        s.ColMean [3 * a + r] += 0.5 * (s.ColPos [9 * m + 3 * a + r] + s.ColNeg [9 * m + 3 * a + r]);
                    double eps = s.Epsilon [m];
                    double under = Finite (eps) && eps > 0.0 ? eps * (1.0 - 1e-5) : 0.0;
                    if (on && under < s.HeldLimit [a])
                        s.HeldLimit [a] = under;
                }
            }
        }

        /// <summary>
        /// The command of axis a (0 pitch, 1 roll, 2 yaw) when it is held: u clipped to the axis' HeldLimit (the RCS
        /// ignores it, the wheels and the other ctrlState users still get it); u itself when no module enables the axis.
        /// </summary>
        internal static double HeldValue (RcsNozzleSet s, int a, double u)
        {
            double limit = s.HeldLimit [a];
            if (!(Math.Abs (u) > limit))
                return u;
            return u > 0.0 ? limit : -limit;
        }

        /// <summary>
        /// The torque (body, N.m, into torque[0..2]) of every module for the ctrlState (pitch, roll, yaw): exact, the
        /// combined rule (ModuleTorque exact); otherwise the per-axis columns of Prepare at what each module's Update
        /// makes of the command (the law's model, an earlier change's linear in-flight torque). Returns the thrust (N; 0 for the
        /// columns). c: scratch of 3.
        /// </summary>
        internal static double Torque (RcsNozzleSet s, double pitch, double roll, double yaw, bool exact, double[] torque,
            double[] c)
        {
            torque [0] = torque [1] = torque [2] = 0.0;
            double thrust = 0.0;
            for (int m = 0; m < s.Modules; m++) {
                ModuleCommand (s, m, pitch, roll, yaw, c);
                if (exact) {
                    thrust += ModuleTorque (s, m, c [0], c [1], c [2], true, torque);
                    continue;
                }
                for (int a = 0; a < 3; a++) {
                    double u = c [a];
                    if (u == 0.0)
                        continue;
                    double[] col = u > 0.0 ? s.ColPos : s.ColNeg;
                    torque [0] += u * col [9 * m + 3 * a];
                    torque [1] += u * col [9 * m + 3 * a + 1];
                    torque [2] += u * col [9 * m + 3 * a + 2];
                }
            }
            return thrust;
        }

        /// <summary>The command kept as the law wrote it (Decide).</summary>
        internal const int KeepCommand = 0;
        /// <summary>The roll left out: pitch/yaw alone (Decide).</summary>
        internal const int RollHeld = 1;
        /// <summary>The pitch/yaw left out: the roll alone (Decide).</summary>
        internal const int PitchYawHeld = 2;

        /// <summary>
        /// The ctrlState command of this step the RCS answers: KeepCommand, RollHeld (the roll at HeldValue: the RCS sees
        /// pitch/yaw alone) or PitchYawHeld (pitch and yaw at HeldValue: the RCS sees the roll alone). served: the
        /// ctrlState the law's increment started from (its u0: pitch, roll, yaw). (pitch, roll, yaw): the command the law
        /// would leave in the ctrlState. rest: the torque per unit of the ctrlState columns' part that is not RCS (wheels,
        /// linear), 9 values, column a (0 pitch, 1 roll, 2 yaw) at [3a..3a+2], body. weight: the law's row weights (Wv_a /
        /// I_a). canHoldPitchYaw: the step writes pitch/yaw too (else the roll alone is not a candidate).
        /// rollHeldBefore, pitchYawHeldBefore: the steps in a row, up to the step before, that the roll / the pitch-yaw was
        /// left out (0 when it was served).
        ///
        /// The law's INDI expects the RCS to give, at its command, the torque of the served command (the combined rule:
        /// option 32's in-flight torque tells it) plus its increment through its own columns (the mean of the halves, its
        /// B), plus the linear rest at its command. Each candidate gives the combined rule's torque plus the rest at its
        /// own command; its cost is sum_a (weight_a x error_a)^2, the cost of leaving a part out once more counted for
        /// every step it has already waited (times 1 + its steps held in a row: the unserved demand accumulates, so no
        /// axis starves -- the ctrlState roll also drives the wheels, which a hold reduces with it); the cheapest wins,
        /// the law's command on a tie, then pitch/yaw alone. A combination the RCS answers smoothly (its increments
        /// close to the columns') is kept; one that jumps (the heavy booster's: a pitch above the dead zone takes the roll
        /// away and multiplies itself) is not. KeepCommand when there is nothing to choose (no roll, or no pitch/yaw) or a
        /// value is not finite.
        /// </summary>
        internal static int Decide (RcsNozzleSet s, double[] served, double pitch, double roll, double yaw, double[] rest,
            double[] weight, bool canHoldPitchYaw, int rollHeldBefore, int pitchYawHeldBefore, RcsCombinedWork w)
        {
            if (!Finite (pitch) || !Finite (roll) || !Finite (yaw) || roll == 0.0 || (pitch == 0.0 && yaw == 0.0))
                return KeepCommand;
            if (!Finite (served [0]) || !Finite (served [1]) || !Finite (served [2]))
                return KeepCommand;
            Torque (s, served [0], served [1], served [2], true, w.Expected, w.Command);
            double dp = pitch - served [0], dr = roll - served [1], dy = yaw - served [2];
            for (int r = 0; r < 3; r++)
                w.Expected [r] += dp * s.ColMean [r] + dr * s.ColMean [3 + r] + dy * s.ColMean [6 + r];
            AddRest (rest, pitch, roll, yaw, w.Expected);
            Torque (s, pitch, roll, yaw, true, w.Combined, w.Command);
            AddRest (rest, pitch, roll, yaw, w.Combined);
            double keep = Error (w.Combined, w.Expected, weight);
            double rh = HeldValue (s, 1, roll);
            Torque (s, pitch, rh, yaw, true, w.Alone, w.Command);
            AddRest (rest, pitch, rh, yaw, w.Alone);
            double noRoll = rh == roll ? double.PositiveInfinity
                : Error (w.Alone, w.Expected, weight) * (1.0 + (rollHeldBefore > 0 ? rollHeldBefore : 0));
            double rollAlone = double.PositiveInfinity;
            double ph = HeldValue (s, 0, pitch), yh = HeldValue (s, 2, yaw);
            if (canHoldPitchYaw && (ph != pitch || yh != yaw)) {
                Torque (s, ph, roll, yh, true, w.Alone, w.Command);
                AddRest (rest, ph, roll, yh, w.Alone);
                rollAlone = Error (w.Alone, w.Expected, weight) * (1.0 + (pitchYawHeldBefore > 0 ? pitchYawHeldBefore : 0));
            }
            if (!Finite (keep) || double.IsNaN (noRoll) || double.IsNaN (rollAlone))
                return KeepCommand;
            if (keep <= noRoll && keep <= rollAlone)
                return KeepCommand;
            return noRoll <= rollAlone ? RollHeld : PitchYawHeld;
        }

        static void AddRest (double[] rest, double pitch, double roll, double yaw, double[] torque)
        {
            for (int r = 0; r < 3; r++)
                torque [r] += pitch * rest [r] + roll * rest [3 + r] + yaw * rest [6 + r];
        }

        static double Error (double[] torque, double[] expected, double[] weight)
        {
            return Sq (weight [0] * (torque [0] - expected [0])) + Sq (weight [1] * (torque [1] - expected [1])) +
                Sq (weight [2] * (torque [2] - expected [2]));
        }

        static double Sq (double v)
        {
            return v * v;
        }
    }

    /// <summary>Scratch of RcsCombined (one per armed vessel; no allocation per step).</summary>
    internal sealed class RcsCombinedWork
    {
        internal readonly double[] Command = new double[3];
        internal readonly double[] Expected = new double[3];
        internal readonly double[] Combined = new double[3];
        internal readonly double[] Alone = new double[3];
        internal readonly double[] Scratch = new double[3];
        internal readonly double[] Fired = new double[3];
    }
}
