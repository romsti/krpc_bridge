// -----------------------------------------------------------------------------------
// the pure part of conn.control.gimbal_authority_v1 (GimbalAuthority.cs). What these tests pin
// down:
//   * the quaternion arithmetic, Unity's convention (x, y, z, w; q * v rotates v): known rotations, the
//     product as composition, the conjugate as the inverse, on random draws;
//   * a nozzle carried back to its pivot's rest pose: a nozzle deflected by any pivot rotation is put back
//     exactly where it is at rest (direction and position, the lever's offset from the pivot kept);
//   * the authority per kN: the alpha rule (|column| x span) is linear in the thrust, so the module's
//     authority at T kN is T times its authority per kN; on the heavy booster's AR-1 Cluster (PivotTests'
//     flight geometry), the per-kN free columns times 4372.473 kN are the free columns at that thrust,
//     and the exact authority per kN times the thrust is the exact authority at that thrust.
// Writes nothing into the golden vectors.
// -----------------------------------------------------------------------------------

using System;

using KRPC.Bridge.Model;

namespace KRPC.Bridge.Model.Tests
{
    public static class GimbalAuthorityTests
    {
        static int checks;
        static int failures;

        public static int Run (out int checkCount)
        {
            checks = 0;
            failures = 0;
            TestQuaternions ();
            TestRestNozzle ();
            TestPerKilonewton ();
            checkCount = checks;
            Console.WriteLine ("gimbal authority checks: " + checks + ", failures: " + failures);
            return failures;
        }

        static void Check (bool condition, string what)
        {
            checks++;
            if (!condition) {
                failures++;
                Console.Error.WriteLine ("FAIL: " + what);
            }
        }

        static double[] AngleAxis (double degrees, double x, double y, double z)
        {
            double n = Math.Sqrt (x * x + y * y + z * z);
            double h = degrees * Math.PI / 360.0;
            double s = Math.Sin (h) / n;
            return new[] { x * s, y * s, z * s, Math.Cos (h) };
        }

        static double[] RandomRotation (Random rng)
        {
            return AngleAxis (rng.NextDouble () * 360.0 - 180.0, rng.NextDouble () - 0.5, rng.NextDouble () - 0.5,
                rng.NextDouble () - 0.5);
        }

        static bool Near3 (double[] a, double[] b, double tol)
        {
            for (int r = 0; r < 3; r++)
                if (Math.Abs (a [r] - b [r]) > tol)
                    return false;
            return true;
        }

        static void TestQuaternions ()
        {
            var v = new double[3];
            GimbalAuthority.Rotate (AngleAxis (90.0, 0, 0, 1), new[] { 1.0, 0.0, 0.0 }, v);
            Check (Near3 (v, new[] { 0.0, 1.0, 0.0 }, 1e-15), "90 deg about z takes x to y (Unity: left-handed frame, same formula)");
            GimbalAuthority.Rotate (AngleAxis (90.0, 1, 0, 0), new[] { 0.0, 1.0, 0.0 }, v);
            Check (Near3 (v, new[] { 0.0, 0.0, 1.0 }, 1e-15), "90 deg about x takes y to z");
            var rng = new Random (2374);
            var ab = new double[4];
            var inv = new double[4];
            var w1 = new double[3];
            var w2 = new double[3];
            var w3 = new double[3];
            for (int k = 0; k < 2000; k++) {
                var a = RandomRotation (rng);
                var b = RandomRotation (rng);
                var x = new[] { rng.NextDouble () - 0.5, rng.NextDouble () - 0.5, rng.NextDouble () - 0.5 };
                GimbalAuthority.Multiply (a, b, ab);
                GimbalAuthority.Rotate (ab, x, w1);
                GimbalAuthority.Rotate (b, x, w2);
                GimbalAuthority.Rotate (a, w2, w3);
                Check (Near3 (w1, w3, 1e-14), "(a b) x = a (b x)");
                GimbalAuthority.Conjugate (a, inv);
                GimbalAuthority.Rotate (a, x, w2);
                GimbalAuthority.Rotate (inv, w2, w3);
                Check (Near3 (w3, x, 1e-14), "the conjugate undoes the rotation");
                double nx = Math.Sqrt (x [0] * x [0] + x [1] * x [1] + x [2] * x [2]);
                double nw = Math.Sqrt (w1 [0] * w1 [0] + w1 [1] * w1 [1] + w1 [2] * w1 [2]);
                Check (Math.Abs (nx - nw) < 1e-14, "a rotation keeps the length");
            }
        }

        static void TestRestNozzle ()
        {
            var rng = new Random (2375);
            var nowRot = new double[4];
            var nozzleNow = new double[3];
            var dirNow = new double[3];
            var restPos = new double[3];
            var restDir = new double[3];
            var carried = new double[3];
            for (int k = 0; k < 2000; k++) {
                // The pivot at rest (world rotation Q), its nozzle there; then the pivot turned by a local
                // deflection D (its world rotation now R = Q D, as parent x initRot x AngleAxis).
                var q = RandomRotation (rng);
                var d = AngleAxis ((rng.NextDouble () - 0.5) * 21.0, rng.NextDouble () - 0.5, 0.0, rng.NextDouble () - 0.5);
                GimbalAuthority.Multiply (q, d, nowRot);
                var pivot = new[] { rng.NextDouble () * 10 - 5, -9.3 + rng.NextDouble (), rng.NextDouble () * 10 - 5 };
                var offsetRest = new[] { rng.NextDouble () - 0.5, -1.5 * rng.NextDouble (), rng.NextDouble () - 0.5 };
                var dirRest = new[] { 0.1 * (rng.NextDouble () - 0.5), -1.0, 0.1 * (rng.NextDouble () - 0.5) };
                double n = Math.Sqrt (dirRest [0] * dirRest [0] + dirRest [1] * dirRest [1] + dirRest [2] * dirRest [2]);
                for (int r = 0; r < 3; r++)
                    dirRest [r] /= n;
                // Carried rigidly by the pivot: R Q^-1 applied to the rest offset and direction.
                var invQ = new double[4];
                var delta = new double[4];
                GimbalAuthority.Conjugate (q, invQ);
                GimbalAuthority.Multiply (nowRot, invQ, delta);
                GimbalAuthority.Rotate (delta, offsetRest, carried);
                for (int r = 0; r < 3; r++)
                    nozzleNow [r] = pivot [r] + carried [r];
                GimbalAuthority.Rotate (delta, dirRest, dirNow);
                GimbalAuthority.RestNozzle (pivot, nowRot, q, nozzleNow, dirNow, restPos, restDir);
                var expected = new double[3];
                for (int r = 0; r < 3; r++)
                    expected [r] = pivot [r] + offsetRest [r];
                Check (Near3 (restPos, expected, 1e-12), "the nozzle's rest position (draw " + k + ")");
                Check (Near3 (restDir, dirRest, 1e-12), "the nozzle's rest direction (draw " + k + ")");
            }
            // At rest already: nothing moves.
            var q0 = AngleAxis (30.0, 0, 1, 0);
            GimbalAuthority.RestNozzle (new[] { 1.0, 2.0, 3.0 }, q0, q0, new[] { 1.5, 0.5, 3.0 }, new[] { 0.0, -1.0, 0.0 },
                restPos, restDir);
            Check (Near3 (restPos, new[] { 1.5, 0.5, 3.0 }, 1e-15) && Near3 (restDir, new[] { 0.0, -1.0, 0.0 }, 1e-15),
                "a pivot at rest leaves its nozzle where it is");
        }

        static void TestPerKilonewton ()
        {
            // PivotTests' heavy booster flight geometry, one AR-1 Cluster: six nozzles, pivots turned to their
            // nozzle's azimuth (pivot at the nozzle: the lever term only), finalThrust 4372.473 kN.
            const int n = 6;
            const double thrustKn = 4372.473;
            var levers = PivotTests.FlightLevers ();
            var directions = PivotTests.FlightDirections ();
            double cx = 0.0, cz = 0.0;
            for (int k = 0; k < n; k++) {
                cx += levers [k, 0] / n;
                cz += levers [k, 2] / n;
            }
            var axes = new double[GimbalPivots.AxesStride * n];
            for (int k = 0; k < n; k++) {
                double rx = levers [k, 0] - cx, rz = levers [k, 2] - cz;
                double r = Math.Sqrt (rx * rx + rz * rz);
                int a = GimbalPivots.AxesStride * k;
                axes [a] = rz / r;
                axes [a + 2] = -rx / r;
                axes [a + 3] = rx / r;
                axes [a + 5] = rz / r;
            }
            var map = new double[GimbalPivots.MapStride * n];
            Check (GimbalPivots.Build (axes, n, map), "the AR-1 Cluster is mapped");
            var perKnX = new double[3];
            var perKnY = new double[3];
            var fullX = new double[3];
            var fullY = new double[3];
            var colsPerKn = new double[GimbalPivots.ColumnStride * n];
            var colsFull = new double[GimbalPivots.ColumnStride * n];
            var offset = new double[3];
            for (int k = 0; k < n; k++) {
                var ax = new[] { axes [6 * k], axes [6 * k + 1], axes [6 * k + 2] };
                var ay = new[] { axes [6 * k + 3], axes [6 * k + 4], axes [6 * k + 5] };
                var d = new[] { directions [k, 0], directions [k, 1], directions [k, 2] };
                var lever = new[] { levers [k, 0], levers [k, 1], levers [k, 2] };
                foreach (bool full in new[] { false, true }) {
                    var cX = new double[3];
                    var cY = new double[3];
                    double thrustN = (full ? thrustKn : 1.0) * 1000.0 / n;
                    GimbalPivots.AddNozzleDerivative (ax, d, lever, offset, thrustN, cX, 0, null);
                    GimbalPivots.AddNozzleDerivative (ay, d, lever, offset, thrustN, cY, 0, null);
                    GimbalPivots.AddPivotColumns (map, k, true, true, cX, cY, full ? fullX : perKnX, full ? fullY : perKnY);
                    var cols = full ? colsFull : colsPerKn;
                    for (int r = 0; r < 3; r++) {
                        cols [6 * k + r] = cX [r] * Math.PI / 180.0;
                        cols [6 * k + 3 + r] = cY [r] * Math.PI / 180.0;
                    }
                }
            }
            GimbalPivots.PerDegree (perKnX);
            GimbalPivots.PerDegree (perKnY);
            GimbalPivots.PerDegree (fullX);
            GimbalPivots.PerDegree (fullY);
            var aPerKn = new double[3];
            var aFull = new double[3];
            GimbalAuthority.ModuleAuthority (perKnX, perKnY, 2.0, 2.0, aPerKn);
            GimbalAuthority.ModuleAuthority (fullX, fullY, 2.0, 2.0, aFull);
            for (int r = 0; r < 3; r++)
                Check (Math.Abs (aPerKn [r] * thrustKn - aFull [r]) <= 1e-9 * Math.Max (1.0, aFull [r]),
                    "the alpha rule per kN times the thrust is the alpha rule at the thrust (axis " + r + ")");
            var exactPerKn = new double[3];
            var exactFull = new double[3];
            var work = new double[3];
            GimbalPivots.ExactAuthority (map, n, colsPerKn, -2, 2, -2, 2, -2, 2, -2, 2, exactPerKn, work);
            GimbalPivots.ExactAuthority (map, n, colsFull, -2, 2, -2, 2, -2, 2, -2, 2, exactFull, work);
            for (int r = 0; r < 3; r++)
                Check (Math.Abs (exactPerKn [r] * thrustKn - exactFull [r]) <= 1e-9 * Math.Max (1.0, exactFull [r]),
                    "the exact authority per kN times the thrust is the exact authority at the thrust (axis " + r + ")");
            // on this cluster the free columns times the span overstate the exact authority (1.159 in flight).
            double ratio = Math.Max (aFull [0], aFull [2]) / Math.Max (exactFull [0], exactFull [2]);
            Check (ratio > 1.0 && ratio < 1.25, "alpha rule over exact on the AR-1 Cluster: " + ratio);
            Check (GimbalAuthority.Span (-2.0, 1.5) == 2.0 && GimbalAuthority.Span (-0.5, 3.0) == 3.0, "the span: the larger bound");
        }
    }
}
