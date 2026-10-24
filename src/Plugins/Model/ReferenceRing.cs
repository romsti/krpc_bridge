using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Model
{
    // -----------------------------------------------------------------------------------
    // K7 (option 2048 of AttitudeOptionsV1, born closed): the REFERENCE RING, a buffered
    // reference trajectory. Pure maths (System only): compiled as is by build/tests
    // (ReferenceRingTests.cs), mirrored operation by operation by the autopilot
    // (the client) and checked against golden vectors
    // (build/tests/golden/attitude_v1_golden_ring.json).
    //
    // WHY. The client's reference reaches the loop 60 ms late at the median, 80 ms at p95, up to
    // 0.2 s (the transport, study of 14/10), and the loop holds it until the next one: the law sees a
    // staircase that is always late. A guidance that knows where its reference goes (D's plan) can
    // send it ahead, dated; the loop then serves, at every physics step, the reference of THAT step.
    //
    // WHAT. A ring is 2 to 64 dated samples [ut, dir x, y, z, a x, y, z, u] in one frame, the ut
    // strictly increasing. Three channels, each present in every sample (finite) or in none (NaN):
    //   1 DIRECTION  the wanted nose (any length but not null; normalized when stored); two
    //                consecutive samples less than 90 deg apart; served by normalized linear
    //                interpolation (nlerp: on such an arc the sum is never shorter than cos 45);
    //   2 LATERAL    the lateral acceleration of the translation request (m/s^2), linear;
    //   4 THROTTLE   the main throttle in [0, 1], linear.
    // At a step's UT inside [ut_first, ut_last] the segment k with ut_k <= ut <= ut_k+1 gives the
    // fraction f = (ut - ut_k) / (ut_k+1 - ut_k) and every present channel; outside, nothing (the
    // fallback: the standing reference, the request, the order, exactly as without a ring).
    //
    // NEVER A FEED-FORWARD: the ring is a position of the reference, never its rate; the
    // loop serves no omega_ff while the ring serves the direction. The ring renews nothing either:
    // the standing reference keeps its validity, its age (the pitch/yaw write rule) and the lease.
    // -----------------------------------------------------------------------------------

    /// <summary>The stored ring (frame components, directions normalized).</summary>
    internal sealed class RingSamples
    {
        internal int Count;
        internal int Channels;
        internal readonly double[] Ut = new double[ReferenceRing.Capacity];
        internal readonly double[] Dir = new double[3 * ReferenceRing.Capacity];
        internal readonly double[] A = new double[3 * ReferenceRing.Capacity];
        internal readonly double[] U = new double[ReferenceRing.Capacity];

        internal void Clear ()
        {
            Count = 0;
            Channels = 0;
        }
    }

    /// <summary>The ring's value at one UT.</summary>
    internal sealed class RingSample
    {
        internal int Served;                 // channels served (0: outside the ring)
        internal int Segment = -1;
        internal double Fraction = double.NaN;
        internal readonly double[] Dir = new double[3];
        internal readonly double[] A = new double[3];
        internal double U = double.NaN;

        internal void Clear ()
        {
            Served = 0;
            Segment = -1;
            Fraction = double.NaN;
            for (int k = 0; k < 3; k++) {
                Dir [k] = double.NaN;
                A [k] = double.NaN;
            }
            U = double.NaN;
        }
    }

    internal static class ReferenceRing
    {
        internal const int Protocol = 1;
        internal const int Schema = 1;
        internal const int Stride = 8;
        internal const int Capacity = 64;

        internal const int ChannelDirection = 1;
        internal const int ChannelLateral = 2;
        internal const int ChannelThrottle = 4;

        // Validate's codes (0 valid).
        internal const int Valid = 0;
        internal const int BadLength = 1;        // not a multiple of the stride
        internal const int BadCount = 2;         // fewer than 2 or more than Capacity samples
        internal const int BadUt = 3;            // an ut not finite
        internal const int NotIncreasing = 4;    // ut not strictly increasing
        internal const int MixedChannel = 5;     // a channel partly finite, or present in some samples only
        internal const int NoChannel = 6;        // no channel at all
        internal const int NullDirection = 7;    // a direction null
        internal const int DirectionTurn = 8;    // two consecutive directions 90 deg or more apart
        internal const int BadThrottle = 9;      // a throttle outside [0, 1]
        internal const int CodeCount = 10;

        static bool Finite (double v)
        {
            return !double.IsNaN (v) && !double.IsInfinity (v);
        }

        /// <summary>0 when all three are finite, 1 when all three are NaN, -1 otherwise.</summary>
        static int Triple (double x, double y, double z)
        {
            bool fx = Finite (x), fy = Finite (y), fz = Finite (z);
            if (fx && fy && fz)
                return 0;
            if (double.IsNaN (x) && double.IsNaN (y) && double.IsNaN (z))
                return 1;
            return -1;
        }

        /// <summary>
        /// The validation of a flat list of samples (Stride values each): one of the codes above; on Valid, the count
        /// and the channels present.
        /// </summary>
        internal static int Validate (IList<double> s, out int count, out int channels)
        {
            count = 0;
            channels = 0;
            if (s == null || s.Count % Stride != 0)
                return BadLength;
            int n = s.Count / Stride;
            if (n < 2 || n > Capacity)
                return BadCount;
            int dir0 = -1, a0 = -1, u0 = -1;
            for (int k = 0; k < n; k++) {
                int b = k * Stride;
                if (!Finite (s [b]))
                    return BadUt;
                if (k > 0 && !(s [b] > s [b - Stride]))
                    return NotIncreasing;
                int d = Triple (s [b + 1], s [b + 2], s [b + 3]);
                int a = Triple (s [b + 4], s [b + 5], s [b + 6]);
                double uv = s [b + 7];
                int u = Finite (uv) ? 0 : (double.IsNaN (uv) ? 1 : -1);
                if (d < 0 || a < 0 || u < 0)
                    return MixedChannel;
                if (k == 0) {
                    dir0 = d;
                    a0 = a;
                    u0 = u;
                } else if (d != dir0 || a != a0 || u != u0) {
                    return MixedChannel;
                }
            }
            if (dir0 == 1 && a0 == 1 && u0 == 1)
                return NoChannel;
            for (int k = 0; k < n; k++) {
                int b = k * Stride;
                if (dir0 == 0) {
                    double x = s [b + 1], y = s [b + 2], z = s [b + 3];
                    double nn = x * x + y * y + z * z;
                    if (!(nn >= 1e-18) || double.IsInfinity (nn))
                        return NullDirection;
                    if (k > 0) {
                        double px = s [b - Stride + 1], py = s [b - Stride + 2], pz = s [b - Stride + 3];
                        double dot = px * x + py * y + pz * z;
                        if (!(dot > 0.0))
                            return DirectionTurn;
                    }
                }
                if (u0 == 0 && !(s [b + 7] >= 0.0 && s [b + 7] <= 1.0))
                    return BadThrottle;
            }
            count = n;
            channels = (dir0 == 0 ? ChannelDirection : 0) | (a0 == 0 ? ChannelLateral : 0) |
                (u0 == 0 ? ChannelThrottle : 0);
            return Valid;
        }

        /// <summary>Stores a VALIDATED list (Validate's count and channels); the directions normalized.</summary>
        internal static void Store (IList<double> s, int count, int channels, RingSamples d)
        {
            d.Count = count;
            d.Channels = channels;
            for (int k = 0; k < count; k++) {
                int b = k * Stride;
                d.Ut [k] = s [b];
                if ((channels & ChannelDirection) != 0) {
                    double x = s [b + 1], y = s [b + 2], z = s [b + 3];
                    double n = Math.Sqrt (x * x + y * y + z * z);
                    d.Dir [3 * k] = x / n;
                    d.Dir [3 * k + 1] = y / n;
                    d.Dir [3 * k + 2] = z / n;
                } else {
                    d.Dir [3 * k] = d.Dir [3 * k + 1] = d.Dir [3 * k + 2] = double.NaN;
                }
                d.A [3 * k] = s [b + 4];
                d.A [3 * k + 1] = s [b + 5];
                d.A [3 * k + 2] = s [b + 6];
                d.U [k] = s [b + 7];
            }
        }

        /// <summary>
        /// The ring's value at ut: the segment k (ut_k &lt;= ut &lt;= ut_k+1, the first such), its fraction, and every
        /// channel the ring has; nothing served outside [ut_first, ut_last] or before two samples.
        /// </summary>
        internal static void Sample (RingSamples d, double ut, RingSample o)
        {
            o.Clear ();
            if (d.Count < 2 || !Finite (ut) || ut < d.Ut [0] || ut > d.Ut [d.Count - 1])
                return;
            int k = 0;
            while (k < d.Count - 2 && ut > d.Ut [k + 1])
                k++;
            double f = (ut - d.Ut [k]) / (d.Ut [k + 1] - d.Ut [k]);
            double g = 1.0 - f;
            o.Segment = k;
            o.Fraction = f;
            int served = 0;
            if ((d.Channels & ChannelDirection) != 0) {
                double x = g * d.Dir [3 * k] + f * d.Dir [3 * k + 3];
                double y = g * d.Dir [3 * k + 1] + f * d.Dir [3 * k + 4];
                double z = g * d.Dir [3 * k + 2] + f * d.Dir [3 * k + 5];
                double n = Math.Sqrt (x * x + y * y + z * z);
                if (n > 1e-9) {
                    o.Dir [0] = x / n;
                    o.Dir [1] = y / n;
                    o.Dir [2] = z / n;
                    served |= ChannelDirection;
                }
            }
            if ((d.Channels & ChannelLateral) != 0) {
                o.A [0] = g * d.A [3 * k] + f * d.A [3 * k + 3];
                o.A [1] = g * d.A [3 * k + 1] + f * d.A [3 * k + 4];
                o.A [2] = g * d.A [3 * k + 2] + f * d.A [3 * k + 5];
                served |= ChannelLateral;
            }
            if ((d.Channels & ChannelThrottle) != 0) {
                o.U = g * d.U [k] + f * d.U [k + 1];
                served |= ChannelThrottle;
            }
            o.Served = served;
        }

        /// <summary>The refusal message of a Validate code (French, the client's language).</summary>
        internal static string Refusal (int code)
        {
            switch (code) {
            case BadLength:
                return "anneau : " + Stride + " valeurs par echantillon (ut, dir x y z, a x y z, u)";
            case BadCount:
                return "anneau : de 2 a " + Capacity + " echantillons";
            case BadUt:
                return "anneau : ut non fini";
            case NotIncreasing:
                return "anneau : ut strictement croissants";
            case MixedChannel:
                return "anneau : chaque canal (direction, acceleration laterale, manette) fini dans tous les " +
                    "echantillons ou NaN dans tous";
            case NoChannel:
                return "anneau : aucun canal";
            case NullDirection:
                return "anneau : direction nulle";
            case DirectionTurn:
                return "anneau : deux directions consecutives a 90 deg ou plus";
            case BadThrottle:
                return "anneau : manette hors de [0, 1]";
            default:
                return "anneau invalide";
            }
        }
    }
}
