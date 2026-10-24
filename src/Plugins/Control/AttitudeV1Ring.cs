using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;
using KRPC.Bridge.Model;
using SCReferenceFrame = KRPC.SpaceCenter.Services.ReferenceFrame;

namespace KRPC.Bridge.Control
{
    /// <summary>
    /// AttitudeV1, (option 2048 of AttitudeOptionsV1, born closed): the reference ring. The interpolation
    /// is ReferenceRing.cs (Model, pure); this file stores the client's ring and serves it.
    ///
    /// attitude_reference_ring_v1 stores a ring of dated samples (a new sequence; an empty list clears it). At every
    /// Earlyish, before the lateral force and the reference (RingStep), the ring's value at the step's UT is taken
    /// once; then the step's consumers read it instead of their own value, each only when its channel is served:
    /// the reference section takes the ring's DIRECTION (in the ring's frame) instead of the standing reference's,
    /// and serves no omega_ff; the lateral force takes the ring's LATERAL acceleration instead of the
    /// request's (TranslationGap); the main throttle's live ramp order (K5, kind 1) takes the ring's THROTTLE
    /// instead of u0 + slope (t - t0) (motif 18), under every guard of K5. Each consumer acts only where it acts
    /// without the ring: the standing reference still valid, the request live, the order live. Outside the ring's
    /// span, nothing is served (the fallback is the loop without a ring). Capability bit 22 on the steps that
    /// served a channel.
    /// </summary>
    internal static partial class AttitudeV1
    {
        // Capability bit of the frame header (offset 23), option 2048 only.
        internal const long CapReferenceRing = 1L << 22;
        internal const int RingReplyLength = 20;

        internal sealed class RingState
        {
            internal readonly RingSamples Samples = new RingSamples ();
            internal readonly RingSample Now = new RingSample ();
            internal SCReferenceFrame Frame;
            internal int Sequence;
            internal double ServedUt = double.NaN;
            internal long ServedSteps, OutsideSteps;

            internal void Reset ()
            {
                Samples.Clear ();
                Now.Clear ();
                Frame = null;
                Sequence = 0;
                ServedUt = double.NaN;
                ServedSteps = OutsideSteps = 0;
            }
        }

        // ---------------------------------------------------------------- RPC body

        /// <summary>
        /// Stores a ring (ReferenceRing's samples, in frame). Refused without option 2048. A sequence not newer reads
        /// (nothing changes); an empty list clears the ring. Returns the attitude_reference_ring_v1 reply.
        /// </summary>
        internal static IList<double> ReferenceRingV1 (string token, int sequence, SCReferenceFrame frame,
            IList<double> samples)
        {
            var entry = RequireToken (token);
            if ((entry.Options & AttitudeWrite.OptionReferenceRing) == 0)
                throw new InvalidOperationException (BridgeErrors.Tag (BridgeErrors.UnsupportedMode,
                    "anneau de references : option 2048 d'attitude_options_v1 requise (D252 K7)"));
            var r = entry.Ring;
            if (sequence <= r.Sequence)
                return RingReply (entry, 0.0);
            if (samples == null || samples.Count == 0) {
                r.Sequence = sequence;
                r.Samples.Clear ();
                r.Frame = null;
                return RingReply (entry, 1.0);
            }
            if (frame == null)
                throw new ArgumentNullException (nameof (frame));
            int count, channels;
            int code = ReferenceRing.Validate (samples, out count, out channels);
            if (code != ReferenceRing.Valid)
                throw new ArgumentOutOfRangeException (nameof (samples), BridgeErrors.Tag (BridgeErrors.UnsupportedMode,
                    ReferenceRing.Refusal (code)));
            r.Sequence = sequence;
            ReferenceRing.Store (samples, count, channels, r.Samples);
            r.Frame = frame;
            return RingReply (entry, 1.0);
        }

        static IList<double> RingReply (Entry e, double accepted)
        {
            var r = e.Ring;
            var s = r.Samples;
            var now = r.Now;
            bool any = s.Count >= 2;
            return new List<double> (RingReplyLength) {
                accepted, ReferenceRing.Protocol, ReferenceRing.Schema, r.Sequence, s.Count, s.Channels,
                any ? s.Ut [0] : double.NaN, any ? s.Ut [s.Count - 1] : double.NaN,
                now.Served, r.ServedUt, now.Segment, now.Fraction,
                now.Dir [0], now.Dir [1], now.Dir [2], now.A [0], now.A [1], now.A [2], now.U, ownTick
            };
        }

        // ---------------------------------------------------------------- the step

        /// <summary>
        /// Option 2048: the ring's value at this step's UT (Earlyish, before the lateral force and the reference).
        /// Returns CapReferenceRing when a channel is served. Nothing without the option.
        /// </summary>
        static long RingStep (Entry e)
        {
            var r = e.Ring;
            if ((e.Options & AttitudeWrite.OptionReferenceRing) == 0 || r.Samples.Count < 2 || r.Frame == null) {
                r.Now.Clear ();
                return 0L;
            }
            double ut = Planetarium.GetUniversalTime ();
            ReferenceRing.Sample (r.Samples, ut, r.Now);
            r.ServedUt = ut;
            if (r.Now.Served == 0) {
                r.OutsideSteps++;
                return 0L;
            }
            r.ServedSteps++;
            return CapReferenceRing;
        }

        /// <summary>The ring's direction of this step in place of the standing reference's (false: unchanged).</summary>
        static bool RingDirection (Entry e, ref SCReferenceFrame frame, ref double x, ref double y, ref double z)
        {
            var now = e.Ring.Now;
            if ((now.Served & ReferenceRing.ChannelDirection) == 0)
                return false;
            frame = e.Ring.Frame;
            x = now.Dir [0];
            y = now.Dir [1];
            z = now.Dir [2];
            return true;
        }

        /// <summary>The ring's lateral acceleration of this step in place of the request's (false: unchanged).</summary>
        static bool RingLateral (Entry e, ref SCReferenceFrame frame, ref double ax, ref double ay, ref double az)
        {
            var now = e.Ring.Now;
            if ((now.Served & ReferenceRing.ChannelLateral) == 0)
                return false;
            frame = e.Ring.Frame;
            ax = now.A [0];
            ay = now.A [1];
            az = now.A [2];
            return true;
        }

        /// <summary>The ring's throttle of this step (NaN: none).</summary>
        static double RingThrottle (Entry e)
        {
            var now = e.Ring.Now;
            return (now.Served & ReferenceRing.ChannelThrottle) != 0 ? now.U : double.NaN;
        }
    }
}
