using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The pure geometry of conn.tower_site (TowerSpawnV1, TowerStatusV1): where a tower stands and
    /// how it is turned, from latitude, longitude and heading, with no world position anywhere.
    ///
    /// THE BODY FRAME. Every vector here is in a body's LOCAL frame, the one of
    /// CelestialBody.GetRelSurfaceNVector (KSP 1.12.5, decompiled): the unit vector of (lat, lon) is
    /// Planetarium.SphericalVector(lat, lon).xzy = (cos lat cos lon, sin lat, cos lat sin lon). The
    /// world direction of a body-local vector is bodyTransform.rotation times it
    /// (GetSurfaceNVector = BodyFrame.LocalToWorld(r).xzy, bodyTransform.rotation =
    /// BodyFrame.Rotation.swizzle), and a saved vessel's `rot` is its root part's rotation in that
    /// same frame (Vessel: srfRelRotation = Inverse(bodyTransform.rotation) * vesselTransform.rotation;
    /// ProtoVessel.Load: transform.rotation = bodyTransform.rotation * rot). So `rot` follows from
    /// lat, lon and heading alone: no floating origin, no rotating frame, no time.
    ///
    /// THE CRAFT FRAME. A craft file places its parts in the editor's frame, +Y up. KSP launches it
    /// by translating that frame onto a spawn transform and turning it by the transform's rotation
    /// (ShipConstruction.PutShipToGround). The two towers Kerbal Konstructs launched from a spawn
    /// point of yaw 0 (tools/site_tour_catch.py kk, lacet 0) are saved with the craft's +X pointing
    /// east (azimuth 90.0000 deg), +Y up (0.0013 deg off) and +Z north: their arms, built along the
    /// craft's +X by the craft generator, point east, cap_bras_deg = 90 in the client's site file.
    /// The heading of this class is therefore the AZIMUTH OF THE CRAFT'S +X AXIS (0 north, 90 east,
    /// clockwise seen from above): for those towers, the direction mast -> booster, the --cap of
    /// site_tour_catch.py. Heading 90 reproduces the Kerbal Konstructs orientation; any other heading
    /// is that orientation turned about the local vertical by (heading - 90) clockwise.
    ///
    /// Quaternions are [x, y, z, w], Hamilton products, rotating a vector as Unity's
    /// Quaternion * Vector3 does (the same matrix). PURE: System only (build/tests/tower).
    /// </summary>
    public static class TowerSiteGeometry
    {
        const double Deg = Math.PI / 180.0;

        /// <summary>The local vertical of (lat, lon): GetRelSurfaceNVector.</summary>
        public static double[] Up (double latDeg, double lonDeg)
        {
            double p = latDeg * Deg, l = lonDeg * Deg;
            return new[] { Math.Cos (p) * Math.Cos (l), Math.Sin (p), Math.Cos (p) * Math.Sin (l) };
        }

        /// <summary>Local north of (lat, lon): the derivative of Up along latitude.</summary>
        public static double[] North (double latDeg, double lonDeg)
        {
            double p = latDeg * Deg, l = lonDeg * Deg;
            return new[] { -Math.Sin (p) * Math.Cos (l), Math.Cos (p), -Math.Sin (p) * Math.Sin (l) };
        }

        /// <summary>Local east of (lat, lon): the direction of increasing longitude.</summary>
        public static double[] East (double latDeg, double lonDeg)
        {
            double l = lonDeg * Deg;
            return new[] { -Math.Sin (l), 0.0, Math.Cos (l) };
        }

        /// <summary>The horizontal unit vector of azimuth <paramref name="azimuthDeg"/> (0 north, 90 east).</summary>
        public static double[] Horizontal (double latDeg, double lonDeg, double azimuthDeg)
        {
            var e = East (latDeg, lonDeg);
            var n = North (latDeg, lonDeg);
            double s = Math.Sin (azimuthDeg * Deg), c = Math.Cos (azimuthDeg * Deg);
            return new[] { s * e [0] + c * n [0], s * e [1] + c * n [1], s * e [2] + c * n [2] };
        }

        /// <summary>The azimuth of vector <paramref name="v"/> at (lat, lon), [0, 360), from its horizontal part.</summary>
        public static double AzimuthDeg (double latDeg, double lonDeg, double[] v)
        {
            double a = Math.Atan2 (Dot (v, East (latDeg, lonDeg)), Dot (v, North (latDeg, lonDeg))) / Deg;
            a %= 360.0;
            return a < 0.0 ? a + 360.0 : a;
        }

        /// <summary>
        /// The rotation that takes the unit X, Y, Z axes to <paramref name="x"/>, <paramref name="y"/>,
        /// <paramref name="z"/> (orthonormal, right-handed as columns), as a quaternion [x, y, z, w].
        /// </summary>
        public static double[] FromAxes (double[] x, double[] y, double[] z)
        {
            // m[i][j] = column j, component i.
            double m00 = x [0], m01 = y [0], m02 = z [0];
            double m10 = x [1], m11 = y [1], m12 = z [1];
            double m20 = x [2], m21 = y [2], m22 = z [2];
            double tr = m00 + m11 + m22;
            double qx, qy, qz, qw, s;
            if (tr > 0.0) {
                s = Math.Sqrt (tr + 1.0) * 2.0;
                qw = 0.25 * s;
                qx = (m21 - m12) / s;
                qy = (m02 - m20) / s;
                qz = (m10 - m01) / s;
            } else if (m00 > m11 && m00 > m22) {
                s = Math.Sqrt (1.0 + m00 - m11 - m22) * 2.0;
                qw = (m21 - m12) / s;
                qx = 0.25 * s;
                qy = (m01 + m10) / s;
                qz = (m02 + m20) / s;
            } else if (m11 > m22) {
                s = Math.Sqrt (1.0 + m11 - m00 - m22) * 2.0;
                qw = (m02 - m20) / s;
                qx = (m01 + m10) / s;
                qy = 0.25 * s;
                qz = (m12 + m21) / s;
            } else {
                s = Math.Sqrt (1.0 + m22 - m00 - m11) * 2.0;
                qw = (m10 - m01) / s;
                qx = (m02 + m20) / s;
                qy = (m12 + m21) / s;
                qz = 0.25 * s;
            }
            return Normalize (new[] { qx, qy, qz, qw });
        }

        /// <summary>The Hamilton product a b: b applied first, then a (Unity's a * b).</summary>
        public static double[] Multiply (double[] a, double[] b)
        {
            return new[] {
                a [3] * b [0] + a [0] * b [3] + a [1] * b [2] - a [2] * b [1],
                a [3] * b [1] - a [0] * b [2] + a [1] * b [3] + a [2] * b [0],
                a [3] * b [2] + a [0] * b [1] - a [1] * b [0] + a [2] * b [3],
                a [3] * b [3] - a [0] * b [0] - a [1] * b [1] - a [2] * b [2]
            };
        }

        /// <summary>The inverse of a unit quaternion (its conjugate).</summary>
        public static double[] Inverse (double[] q)
        {
            return new[] { -q [0], -q [1], -q [2], q [3] };
        }

        /// <summary>The vector <paramref name="v"/> turned by <paramref name="q"/> (Unity's q * v).</summary>
        public static double[] Rotate (double[] q, double[] v)
        {
            double x = q [0], y = q [1], z = q [2], w = q [3];
            double tx = 2.0 * (y * v [2] - z * v [1]);
            double ty = 2.0 * (z * v [0] - x * v [2]);
            double tz = 2.0 * (x * v [1] - y * v [0]);
            return new[] {
                v [0] + w * tx + (y * tz - z * ty),
                v [1] + w * ty + (z * tx - x * tz),
                v [2] + w * tz + (x * ty - y * tx)
            };
        }

        /// <summary>The angle between two vectors, degrees (atan2, exact near 0 and 180).</summary>
        public static double AngleDeg (double[] a, double[] b)
        {
            var c = Cross (a, b);
            return Math.Atan2 (Norm (c), Dot (a, b)) / Deg;
        }

        /// <summary>
        /// The angle of the rotation that takes <paramref name="a"/> to <paramref name="b"/>, degrees
        /// (q and -q equal): 2 atan2(|v|, |w|) of b a^-1, exact near 0 where an acos of the dot is not.
        /// </summary>
        public static double QuaternionAngleDeg (double[] a, double[] b)
        {
            var r = Multiply (Normalize (b), Inverse (Normalize (a)));
            double v = Math.Sqrt (r [0] * r [0] + r [1] * r [1] + r [2] * r [2]);
            return 2.0 * Math.Atan2 (v, Math.Abs (r [3])) / Deg;
        }

        /// <summary>
        /// The tilt of a root part saved with rotation <paramref name="rot"/> (body frame) at (lat,
        /// lon): its local +Y against the local vertical, degrees. The tilt TowerAnchor logs and
        /// TowerStatusV1 publishes for a tower that is not loaded.
        /// </summary>
        public static double TiltDeg (double[] rot, double latDeg, double lonDeg)
        {
            return AngleDeg (Rotate (Normalize (rot), new[] { 0.0, 1.0, 0.0 }), Up (latDeg, lonDeg));
        }

        /// <summary>
        /// A site: the body-frame images of the craft's axes at (lat, lon) for a heading. +X is the
        /// horizontal of azimuth heading, +Y the local vertical, +Z the horizontal of azimuth heading - 90
        /// (so that X, Y, Z keep the craft's handedness).
        /// </summary>
        public sealed class Site
        {
            public readonly double Latitude, Longitude, Heading;
            public readonly double[] X, Y, Z;

            public Site (double latDeg, double lonDeg, double headingDeg)
            {
                Latitude = latDeg;
                Longitude = lonDeg;
                Heading = headingDeg;
                X = Horizontal (latDeg, lonDeg, headingDeg);
                Y = Up (latDeg, lonDeg);
                Z = Horizontal (latDeg, lonDeg, headingDeg - 90.0);
            }

            /// <summary>The rotation of the craft frame in the body frame.</summary>
            public double[] CraftRotation ()
            {
                return FromAxes (X, Y, Z);
            }

            /// <summary>
            /// The body-frame rotation (`rot`) of a part whose rotation IN THE CRAFT FRAME is
            /// <paramref name="partInCraft"/> (for the root, the `rot` of its PART in the craft file).
            /// </summary>
            public double[] PartRotation (double[] partInCraft)
            {
                return Normalize (Multiply (CraftRotation (), Normalize (partInCraft)));
            }

            /// <summary>
            /// The unit direction from the body's centre to the point at craft offset (x, 0, z) from
            /// the root, the root at <paramref name="radial"/> metres from the centre.
            /// </summary>
            public double[] Direction (double x, double z, double radial)
            {
                return Normalize (new[] {
                    radial * Y [0] + x * X [0] + z * Z [0],
                    radial * Y [1] + x * X [1] + z * Z [1],
                    radial * Y [2] + x * X [2] + z * Z [2]
                });
            }
        }

        /// <summary>
        /// The ground footprint of one collider of the craft: its horizontal extent along the craft's
        /// X and Z axes and its lowest point, all relative to the ROOT part's origin, in metres.
        /// </summary>
        public sealed class Footprint
        {
            public double X0, X1, Z0, Z1, Bottom;
            public string Name = "";
        }

        /// <summary>What Place found.</summary>
        public sealed class Placement
        {
            /// <summary>The root part's altitude (m above the body's radius).</summary>
            public double RootAltitude = double.NaN;
            /// <summary>The terrain under the root (m).</summary>
            public double TerrainAtRoot = double.NaN;
            public double TerrainMin = double.NaN, TerrainMax = double.NaN;
            /// <summary>Terrain samples taken.</summary>
            public int Samples;
            /// <summary>The footprint that set the altitude, and the point where it touches.</summary>
            public int Governing = -1;
            public double GoverningX = double.NaN, GoverningZ = double.NaN, GoverningTerrain = double.NaN;
            /// <summary>The effective bottom of the governing footprint (m, root-relative).</summary>
            public double GoverningBottom = double.NaN;
            /// <summary>The spacing of the sampling grid along X and Z (m).</summary>
            public double StepX, StepZ;
            /// <summary>
            /// How far the terrain between two samples can rise above the samples, from the largest
            /// second difference of the grid (kappa h^2 / 8 per axis, m). Logged, never added.
            /// </summary>
            public double ErrorBound = double.NaN;
            /// <summary>The farthest footprint corner from the root, horizontally (m).</summary>
            public double Extent;
        }

        /// <summary>Why these footprints cannot be placed, or null.</summary>
        public static string InvalidFootprints (IList<Footprint> prints)
        {
            if (prints == null || prints.Count == 0)
                return "no collider footprint";
            for (int i = 0; i < prints.Count; i++) {
                var f = prints [i];
                if (f == null)
                    return "footprint " + i + " is null";
                if (!Finite (f.X0) || !Finite (f.X1) || !Finite (f.Z0) || !Finite (f.Z1) || !Finite (f.Bottom))
                    return "footprint " + i + " (" + f.Name + ") is not finite";
                if (f.X1 < f.X0 || f.Z1 < f.Z0)
                    return "footprint " + i + " (" + f.Name + ") has a negative extent";
            }
            return null;
        }

        /// <summary>The farthest footprint corner from the root, horizontally (m).</summary>
        public static double HorizontalExtent (IList<Footprint> prints)
        {
            double r = 0.0;
            foreach (var f in prints) {
                double x = Math.Max (Math.Abs (f.X0), Math.Abs (f.X1));
                double z = Math.Max (Math.Abs (f.Z0), Math.Abs (f.Z1));
                r = Math.Max (r, Math.Sqrt (x * x + z * z));
            }
            return r;
        }

        /// <summary>
        /// The root altitude at which the craft stands on the terrain: every collider at or above the
        /// HIGHEST terrain under its own footprint, the one that touches exactly on it, no margin.
        ///
        /// <paramref name="terrain"/>(x, z) is the terrain altitude (m) under the craft offset (x, z)
        /// from the root. It is sampled on one grid over the union of the footprints, of spacing at
        /// most <paramref name="step"/> (at least three points along every axis that has an extent),
        /// plus the four corners of every footprint, exactly, plus the root's own point. A footprint's
        /// candidate is max over its samples of terrain - bottom - rho^2 / (2 (R + terrain)): the
        /// collider's bottom is that high above the root's level, and a point rho metres from the root
        /// axis is rho^2 / 2R higher above the sphere than the root's tangent plane (0.3 mm at 20 m on
        /// Kerbin). The bottom of a footprint is max(its own, <paramref name="floorBottom"/>): KSP's
        /// launch lets parts raise the craft's lowest point (LaunchClamp.OnPutToGround sets it to the
        /// clamp's own level, ShipConstruction.PutShipToGround then puts THAT on the ground); NaN: none.
        /// The root altitude is the largest candidate.
        /// </summary>
        public static Placement Place (IList<Footprint> prints, double floorBottom, double step,
                                       Func<double, double, double> terrain, double radius)
        {
            var bad = InvalidFootprints (prints);
            if (bad != null)
                throw new ArgumentException (bad, "prints");
            if (!Finite (step) || step <= 0.0)
                throw new ArgumentException ("step must be finite and > 0", "step");
            if (!Finite (radius) || radius <= 0.0)
                throw new ArgumentException ("radius must be finite and > 0", "radius");
            if (terrain == null)
                throw new ArgumentNullException ("terrain");

            double gx0 = double.MaxValue, gx1 = double.MinValue, gz0 = double.MaxValue, gz1 = double.MinValue;
            foreach (var f in prints) {
                gx0 = Math.Min (gx0, f.X0);
                gx1 = Math.Max (gx1, f.X1);
                gz0 = Math.Min (gz0, f.Z0);
                gz1 = Math.Max (gz1, f.Z1);
            }
            int nx = Intervals (gx1 - gx0, step), nz = Intervals (gz1 - gz0, step);
            double dx = nx == 0 ? 0.0 : (gx1 - gx0) / nx, dz = nz == 0 ? 0.0 : (gz1 - gz0) / nz;
            var p = new Placement { StepX = dx, StepZ = dz, Extent = HorizontalExtent (prints) };

            var grid = new double [nx + 1, nz + 1];
            for (int i = 0; i <= nx; i++)
                for (int j = 0; j <= nz; j++)
                    grid [i, j] = Sample (p, terrain, gx0 + i * dx, gz0 + j * dz);
            p.TerrainAtRoot = Sample (p, terrain, 0.0, 0.0);

            double best = double.NegativeInfinity;
            for (int k = 0; k < prints.Count; k++) {
                var f = prints [k];
                double bottom = double.IsNaN (floorBottom) ? f.Bottom : Math.Max (f.Bottom, floorBottom);
                Consider (p, k, bottom, f.X0, f.Z0, Sample (p, terrain, f.X0, f.Z0), radius, ref best);
                Consider (p, k, bottom, f.X1, f.Z0, Sample (p, terrain, f.X1, f.Z0), radius, ref best);
                Consider (p, k, bottom, f.X0, f.Z1, Sample (p, terrain, f.X0, f.Z1), radius, ref best);
                Consider (p, k, bottom, f.X1, f.Z1, Sample (p, terrain, f.X1, f.Z1), radius, ref best);
                int i0 = First (f.X0, gx0, dx, nx), i1 = Last (f.X1, gx0, dx, nx);
                int j0 = First (f.Z0, gz0, dz, nz), j1 = Last (f.Z1, gz0, dz, nz);
                for (int i = i0; i <= i1; i++)
                    for (int j = j0; j <= j1; j++)
                        Consider (p, k, bottom, gx0 + i * dx, gz0 + j * dz, grid [i, j], radius, ref best);
            }
            p.RootAltitude = best;

            // Second differences of the grid: the terrain's curvature between samples.
            double kx = 0.0, kz = 0.0;
            for (int i = 1; i < nx; i++)
                for (int j = 0; j <= nz; j++)
                    kx = Math.Max (kx, Math.Abs (grid [i + 1, j] - 2.0 * grid [i, j] + grid [i - 1, j]));
            for (int i = 0; i <= nx; i++)
                for (int j = 1; j < nz; j++)
                    kz = Math.Max (kz, Math.Abs (grid [i, j + 1] - 2.0 * grid [i, j] + grid [i, j - 1]));
            // kappa = second difference / h^2, bound kappa h^2 / 8: the h^2 cancel.
            p.ErrorBound = (kx + kz) / 8.0;
            return p;
        }

        static int Intervals (double span, double step)
        {
            if (span <= 0.0)
                return 0;
            return Math.Max (2, (int) Math.Ceiling (span / step));
        }

        // The first and last grid index inside [lo, hi] (a tolerance of a micrometre for the edges).
        static int First (double lo, double origin, double d, int n)
        {
            if (n == 0 || d <= 0.0)
                return 0;
            return Math.Max (0, (int) Math.Ceiling ((lo - origin) / d - 1e-6));
        }

        static int Last (double hi, double origin, double d, int n)
        {
            if (n == 0 || d <= 0.0)
                return 0;
            return Math.Min (n, (int) Math.Floor ((hi - origin) / d + 1e-6));
        }

        static double Sample (Placement p, Func<double, double, double> terrain, double x, double z)
        {
            double t = terrain (x, z);
            if (!Finite (t))
                throw new ArgumentException ("terrain not finite at (" + x + ", " + z + ")", "terrain");
            p.Samples++;
            if (double.IsNaN (p.TerrainMin) || t < p.TerrainMin)
                p.TerrainMin = t;
            if (double.IsNaN (p.TerrainMax) || t > p.TerrainMax)
                p.TerrainMax = t;
            return t;
        }

        static void Consider (Placement p, int k, double bottom, double x, double z, double t, double radius,
                              ref double best)
        {
            double rho2 = x * x + z * z;
            double candidate = t - bottom - rho2 / (2.0 * (radius + t));
            if (candidate > best) {
                best = candidate;
                p.Governing = k;
                p.GoverningX = x;
                p.GoverningZ = z;
                p.GoverningTerrain = t;
                p.GoverningBottom = bottom;
            }
        }

        /// <summary>
        /// The horizontal extent of a saved vessel: the farthest of its parts' origins from its root,
        /// across the local vertical (m). <paramref name="rot"/> is its saved rotation (body frame),
        /// <paramref name="positions"/> its parts' saved positions (root frame). Part origins only: a
        /// lower bound of its colliders' extent.
        /// </summary>
        public static double SavedExtent (double[] rot, double latDeg, double lonDeg, IList<double[]> positions)
        {
            var up = Up (latDeg, lonDeg);
            var q = Normalize (rot);
            double r = 0.0;
            foreach (var pos in positions) {
                var v = Rotate (q, pos);
                double h = Dot (v, up);
                var hv = new[] { v [0] - h * up [0], v [1] - h * up [1], v [2] - h * up [2] };
                r = Math.Max (r, Norm (hv));
            }
            return r;
        }

        /// <summary>The distance along the surface between two (lat, lon) at radius <paramref name="radius"/> (m).</summary>
        public static double SurfaceDistance (double lat1, double lon1, double lat2, double lon2, double radius)
        {
            return AngleDeg (Up (lat1, lon1), Up (lat2, lon2)) * Deg * radius;
        }

        /// <summary>Why this site is refused, or null: latitude in [-90, 90], longitude in [-360, 360], heading finite.</summary>
        public static string InvalidSite (double latDeg, double lonDeg, double headingDeg)
        {
            if (!Finite (latDeg) || latDeg < -90.0 || latDeg > 90.0)
                return "latitude must be finite, in [-90, 90] deg";
            if (!Finite (lonDeg) || lonDeg < -360.0 || lonDeg > 360.0)
                return "longitude must be finite, in [-360, 360] deg";
            if (!Finite (headingDeg))
                return "heading must be finite";
            return null;
        }

        /// <summary>A longitude in (-180, 180], as KSP's GetLatLonAlt gives it.</summary>
        public static double NormalizeLongitude (double lonDeg)
        {
            double l = lonDeg % 360.0;
            if (l > 180.0)
                l -= 360.0;
            if (l <= -180.0)
                l += 360.0;
            return l;
        }

        /// <summary>A heading in [0, 360).</summary>
        public static double NormalizeHeading (double headingDeg)
        {
            double h = headingDeg % 360.0;
            return h < 0.0 ? h + 360.0 : h;
        }

        public static double Dot (double[] a, double[] b)
        {
            return a [0] * b [0] + a [1] * b [1] + a [2] * b [2];
        }

        public static double[] Cross (double[] a, double[] b)
        {
            return new[] { a [1] * b [2] - a [2] * b [1], a [2] * b [0] - a [0] * b [2], a [0] * b [1] - a [1] * b [0] };
        }

        public static double Norm (double[] a)
        {
            return Math.Sqrt (Dot (a, a));
        }

        /// <summary>A vector or quaternion scaled to unit length (returned as is when zero).</summary>
        public static double[] Normalize (double[] v)
        {
            double n = 0.0;
            foreach (var c in v)
                n += c * c;
            n = Math.Sqrt (n);
            if (n == 0.0)
                return (double[]) v.Clone ();
            var r = new double [v.Length];
            for (int i = 0; i < v.Length; i++)
                r [i] = v [i] / n;
            return r;
        }

        static bool Finite (double x)
        {
            return !double.IsNaN (x) && !double.IsInfinity (x);
        }
    }
}
