using System;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // the RCS torque ALREADY COMMANDED but not yet realized, taken off the allocation
    // target (option 32 of AttitudeOptionsV1, born closed). PURE: no Unity, no KSP type.
    //
    // How stock KSP fires an RCS module (KSP 1.12.5, ModuleRCS, decompiled from the installed
    // Assembly-CSharp.dll; ModuleRCSFX inherits both methods and overrides only the effects):
    //   * Update() -- the RENDER frame, not the physics step -- reads vessel.ctrlState.pitch,
    //     roll and yaw (each zero unless enablePitch / enableRoll / enableYaw), zeroes every
    //     component whose square is below EPSILON^2 (the KSPField EPSILON, 0.05 by default),
    //     rotates the vector into the world (ReferenceTransform.rotation) and stores it in the
    //     private field inputRot (then ApplyInputRotationAdjustments);
    //   * FixedUpdate() fires every nozzle from that stored inputRot:
    //     max(dot(thrust axis, inputRot x unit(ProjectOnPlane(r, inputRot))), 0), clamped to 1,
    //     and CalculateThrust gives nothing below EPSILON again (per nozzle), the force
    //     fraction x maxFuelFlow x thrustPercentage x Isp x g above: PROPORTIONAL above the dead
    //     zone (no pulse, no minimum impulse; fullThrust and precision mode aside).
    // So the ctrlState the loop writes at the fly-by-wire of physics step k is read by the next
    // render frame's Update, and fired at physics step k + 1 at the earliest: ONE STEP LATER than
    // the wheels (ModuleReactionWheel reads ctrlState in its own FixedUpdate, after the
    // fly-by-wire of the same step, and lerps at torqueResponseSpeed x dt) and than the leased
    // gimbals (actuationLocal set at Earlyish). When two physics steps share one render frame,
    // later still. Measured on the super hope (08/10, three flights): the RCS torque
    // realized at step t + 1 follows the ctrlState SERVED at step t (correlation 0.98-0.99, 0.49-0.92
    // at the neighbouring lags), exactly for 92-96 % of the steps and one step older for 4-7 %
    // (two physics steps in a frame); zero below |u| = 0.05 (98-100 % of the steps), slope 0.93-1.09
    // above; the wheel follows the same step's command with Lerp 0.600.
    //
    // Why it matters (a recorded flight, the loop flying the coast on RCS and
    // wheels): the INDI allocation target is M_d - M, an increment from the ctrlState SERVED, u0,
    // that assumes the measured torque M is that of u0. For the RCS it is that of the command
    // BEFORE u0. With a dominant column b: u_k = u_{k-1} + (M_d - b u_{k-2}) / b, poles
    // z^2 - z + 1 = 0, |z| = 1 at +-60 deg: a period of SIX steps (8.3 Hz), marginal, held at the
    // +-1 bounds by the saturation -- the flown -1, -1, 0, +1, +1, 0 at +-697 kN.m for a demand of
    // 40-60 kN.m, served at +-1 on 65 % of the steps, 1.56 t of propellant in the coast.
    //
    // The fix is the synchronization of the increment: the torque the RCS will realize anyway,
    // whatever the new command -- the served command's own torque minus the torque of the command
    // M was realized from -- is taken off the target (AttitudeInput.TorqueFree, the mechanism lot
    // 3 added for the slewing surfaces; AttitudeMath is unchanged):
    //     M_free,rcs = sum over modules of  C_m(DZ_m(served)) - C_m(realized_m)
    // C_m the module's linear columns (AttitudeGeometry.AddRcsColumns, bPos for a positive
    // component, bNeg for a negative one), DZ_m what its Update makes of the served ctrlState
    // (enable flags, EPSILON), realized_m the body command its FixedUpdate fired at the step M was
    // measured from (its inputRot read at that step's Earlyish: exact whatever the frame rate).
    // Then u_k = u0 + du gives b DZ(u_k) = M_d - (M - M_rcs) + (realized torque - modeled): the
    // RCS lands on the torque the law wants when it fires, the model's error stays in the
    // increment (the INDI's integral action is kept), and the recurrence's poles go to 0 (exact
    // model) or to sqrt(|1 - eta|) for an efficiency eta. Nothing changes where served and realized
    // agree (both in the dead zone: the descent, where the gimbals hold everything).
    // -----------------------------------------------------------------------------------
    internal static class RcsInFlight
    {
        /// <summary>ModuleRCS.Update's dead zone: a component whose square is below epsilon^2 is zero.</summary>
        internal static double DeadZone (double u, double epsilon)
        {
            if (double.IsNaN (u) || double.IsInfinity (u))
                return 0.0;
            return u * u < epsilon * epsilon ? 0.0 : u;
        }

        /// <summary>
        /// What ModuleRCS.Update makes of ctrlState (pitch, roll, yaw) in the body frame: each axis zero
        /// unless enabled, then the dead zone. Into command[0..2] (x pitch, y roll, z yaw).
        /// </summary>
        internal static void UpdateCommand (double pitch, double roll, double yaw, bool enablePitch,
            bool enableRoll, bool enableYaw, double epsilon, double[] command)
        {
            command [0] = enablePitch ? DeadZone (pitch, epsilon) : 0.0;
            command [1] = enableRoll ? DeadZone (roll, epsilon) : 0.0;
            command [2] = enableYaw ? DeadZone (yaw, epsilon) : 0.0;
        }

        /// <summary>
        /// Adds into torque (body, N.m) what a module's linear columns give the body command c: per axis a,
        /// c_a x bPos[3a..3a+2] for c_a &gt; 0, c_a x bNeg[3a..3a+2] for c_a &lt; 0 (bNeg is per unit of a
        /// negative command, AttitudeGeometry.AddRcsColumns).
        /// </summary>
        internal static void AddColumnsTorque (double[] bPos, double[] bNeg, double[] c, double[] torque)
        {
            for (int a = 0; a < 3; a++) {
                double u = c [a];
                if (u == 0.0 || double.IsNaN (u) || double.IsInfinity (u))
                    continue;
                double[] col = u > 0.0 ? bPos : bNeg;
                torque [0] += u * col [3 * a];
                torque [1] += u * col [3 * a + 1];
                torque [2] += u * col [3 * a + 2];
            }
        }

        /// <summary>
        /// One module's torque in flight, ADDED into torque (body, N.m): its columns' torque at the command
        /// its Update will make of the served ctrlState (next) minus their torque at the command it fired
        /// when the measured torque was realized (realized). False (nothing added) when a value is not finite.
        /// </summary>
        internal static bool AddInFlight (double[] bPos, double[] bNeg, double[] next, double[] realized,
            double[] scratch, double[] torque)
        {
            for (int k = 0; k < 9; k++)
                if (double.IsNaN (bPos [k]) || double.IsInfinity (bPos [k]) || double.IsNaN (bNeg [k]) ||
                        double.IsInfinity (bNeg [k]))
                    return false;
            for (int a = 0; a < 3; a++)
                if (double.IsNaN (realized [a]) || double.IsInfinity (realized [a]) || double.IsNaN (next [a]) ||
                        double.IsInfinity (next [a]))
                    return false;
            // scratch = C(realized) - C(next), taken off torque: the module's whole difference or nothing.
            scratch [0] = 0.0;
            scratch [1] = 0.0;
            scratch [2] = 0.0;
            AddColumnsTorque (bPos, bNeg, next, scratch);
            for (int a = 0; a < 3; a++)
                scratch [a] = -scratch [a];
            AddColumnsTorque (bPos, bNeg, realized, scratch);
            torque [0] -= scratch [0];
            torque [1] -= scratch [1];
            torque [2] -= scratch [2];
            return true;
        }
    }
}
