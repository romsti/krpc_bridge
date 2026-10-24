using System;
using KRPC.Bridge.Model;
using UnityEngine;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, (option 1024 of AttitudeOptionsV1, born closed): the roll pointed. The law's part is
    /// AttitudeMath (RollError, AttitudeInput.RollTargetValid); this file touches the game.
    ///
    /// A reference of roll mode 1 (attitude_reference_v3) keeps an up vector in its frame. At every step with a valid
    /// reference, the up vector is carried to the world (the frame's DirectionToWorldSpace, as the direction) and to the
    /// body (the reference transform), its roll error goes into the law's Error[1] and RollTargetValid is set: the roll
    /// then follows the square-root reference of the other axes on its own authority, served by the same allocation
    /// (gimbal roll columns, RCS and wheels through ctrlState) and written by the roll channel of M3. A frame that no
    /// longer converts, or an up vector along the nose, leaves the roll rate damped for the step (as without the
    /// option). Capability bit 21 on the steps whose law had the roll target. Roll mode 2 (the shadow) computes the same
    /// error into Error[1] for the frame and leaves RollTargetValid false: the law is mode 0's, bit for bit; capability
    /// bit 21 then says the frame's error y is the shadow's roll error.
    /// </summary>
    internal static partial class AttitudeV1
    {
        // Capability bit of the frame header (offset 23), option 1024 only.
        internal const long CapRollTarget = 1L << 21;

        /// <summary>
        /// Option 1024: the roll error of the reference's up vector into the law (Error[1], RollTargetValid). True when it
        /// was set this step. Never throws.
        /// </summary>
        static bool RollTargetStep (Entry e, Transform reference, AttitudeInput inp)
        {
            try {
                Vector3d w = e.Frame.DirectionToWorldSpace (new Vector3d (e.UpX, e.UpY, e.UpZ));
                Vector3 b = reference.InverseTransformDirection (new Vector3 ((float)w.x, (float)w.y, (float)w.z));
                double err = AttitudeMath.RollError (b.x, b.y, b.z);
                if (double.IsNaN (err) || double.IsInfinity (err)) {
                    inp.Error [1] = 0.0;
                    return false;
                }
                inp.Error [1] = err;
                inp.RollTargetValid = true;
                return true;
            } catch {
                inp.Error [1] = 0.0;
                inp.RollTargetValid = false;
                return false;
            }
        }
    }
}
