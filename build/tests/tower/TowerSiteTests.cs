// -----------------------------------------------------------------------------------
// conn.tower_site: pure tests of TowerSiteGeometry (the rotation from latitude, longitude and
// heading; the altitude from the footprint), TowerSiteFormat (towers_v1, tower_status_v1) and their
// layout declaration. See TowerTests.csproj; run by Program.Main.
//
// The reference: the two towers Kerbal Konstructs launched (spawn yaw 0, arms east), as KSP saved
// them in a save's persistent.sfs (12/10/2026): lat, lon and rot of their VESSEL
// nodes, and their root part's rotation in their craft (Tour Catch Sud.craft / Nord: rot = 0,1,0,0).
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.Tower
{
    public static partial class Program
    {
        // persistent.sfs, VESSEL "Tour Catch Sud" and "Tour Catch Nord".
        static readonly double SudLat = -0.15449600451330517, SudLon = -74.558940266150842;
        static readonly double[] SudRot = { -0.0951294526, -0.699750721, 0.701639891, -0.094873026 };
        static readonly double NordLat = -0.039903999326541872, NordLon = -74.558935706722437;
        static readonly double[] NordRot = { -0.0950342417, -0.700452209, 0.700939715, -0.0949681252 };
        // The root part TG.TowerBase in both craft files: rot = 0,1,0,0 (180 deg about +Y).
        static readonly double[] RootInCraft = { 0.0, 1.0, 0.0, 0.0 };
        const double KerbinRadius = 600000.0;

        static void TowerSiteTests ()
        {
            TestSavedTowersAtHeading90 ();
            TestHeadingConvention ();
            TestFromAxesRoundTrip ();
            TestPlaceFlat ();
            TestPlaceSlopeAndFloor ();
            TestPlaceArmsOverAHill ();
            TestPlaceCurvatureAndErrorBound ();
            TestPlaceRefusals ();
            TestExtentsAndDistances ();
            TestSiteAndNameRules ();
            TestTowerSiteFormat ();
            TestTowerSiteLayouts ();
            TestZeroQuaternion ();
        }

        // The values a tower spawned at the Space Center saved (ground test of 13/10/2026) and the ones the
        // Kerbal Konstructs towers hold (persistent.sfs of a test install).
        static void TestZeroQuaternion ()
        {
            Check (TowerSiteFormat.IsZeroQuaternion ("(0, 0, 0, 0)"), "the spawned hinge's servoTransformRotation is the zero quaternion");
            Check (TowerSiteFormat.IsZeroQuaternion ("0,0,0,0"), "the zero quaternion, KSPUtil format");
            Check (TowerSiteFormat.IsZeroQuaternion (" ( -0 , 0.0 , 0e0 , 0 ) "), "signed zeros, spaces, exponents");
            Check (!TowerSiteFormat.IsZeroQuaternion ("0,0,9.50748508E-05,-1"), "the Kerbal Konstructs tower's servoTransformRotation is a rotation");
            Check (!TowerSiteFormat.IsZeroQuaternion ("0,0,0,1"), "identity");
            Check (!TowerSiteFormat.IsZeroQuaternion ("(0, 0, 0)"), "a zero vector is a position, kept");
            Check (!TowerSiteFormat.IsZeroQuaternion ("(86.5940018, 90.9069977)"), "soft limits kept");
            Check (!TowerSiteFormat.IsZeroQuaternion ("0"), "a number kept");
            Check (!TowerSiteFormat.IsZeroQuaternion ("True"), "a flag kept");
            Check (!TowerSiteFormat.IsZeroQuaternion (""), "empty kept");
            Check (!TowerSiteFormat.IsZeroQuaternion ("a,b,c,d"), "words kept");
        }

        static void NearVec (double[] expected, double[] actual, double tol, string what)
        {
            for (int i = 0; i < expected.Length; i++)
                Near (expected [i], actual [i], tol, what + " [" + i + "]");
        }

        static void TestSavedTowersAtHeading90 ()
        {
            foreach (var t in new[] {
                new { Name = "Sud", Lat = SudLat, Lon = SudLon, Rot = SudRot },
                new { Name = "Nord", Lat = NordLat, Lon = NordLon, Rot = NordRot } }) {
                var site = new TowerSiteGeometry.Site (t.Lat, t.Lon, 90.0);
                var rot = site.PartRotation (RootInCraft);
                // The saved tower stands 0.0013 deg off the vertical: the computed upright one is that far.
                double tilt = TowerSiteGeometry.TiltDeg (t.Rot, t.Lat, t.Lon);
                Check (tilt > 0.0010 && tilt < 0.0016, t.Name + ": saved tilt 0.0013 deg (got " + tilt + ")");
                double angle = TowerSiteGeometry.QuaternionAngleDeg (rot, t.Rot);
                Check (angle < 0.002, t.Name + ": heading 90 reproduces the Kerbal Konstructs rot within 0.002 deg (got " + angle + ")");
                Near (0.0, TowerSiteGeometry.TiltDeg (rot, t.Lat, t.Lon), 1e-9, t.Name + ": computed tower upright");
                // The arms: craft +X = root -X (the root is turned 180 deg about +Y in the craft).
                var arms = TowerSiteGeometry.Rotate (t.Rot, new[] { -1.0, 0.0, 0.0 });
                Near (90.0, TowerSiteGeometry.AzimuthDeg (t.Lat, t.Lon, arms), 0.001, t.Name + ": saved arms at azimuth 90");
                var mine = TowerSiteGeometry.Rotate (rot, new[] { -1.0, 0.0, 0.0 });
                Near (90.0, TowerSiteGeometry.AzimuthDeg (t.Lat, t.Lon, mine), 1e-9, t.Name + ": computed arms at azimuth 90");
                var south = TowerSiteGeometry.Rotate (t.Rot, new[] { 0.0, 0.0, 1.0 });
                Near (180.0, TowerSiteGeometry.AzimuthDeg (t.Lat, t.Lon, south), 0.001, t.Name + ": saved root +Z south (craft +Z north)");
            }
        }

        static void TestHeadingConvention ()
        {
            double lat = 12.5, lon = -74.5;
            var up = TowerSiteGeometry.Up (lat, lon);
            foreach (var h in new[] { 0.0, 37.0, 90.0, 180.0, 270.0, 359.0 }) {
                var site = new TowerSiteGeometry.Site (lat, lon, h);
                var q = site.CraftRotation ();
                NearVec (site.X, TowerSiteGeometry.Rotate (q, new[] { 1.0, 0.0, 0.0 }), 1e-12, "h " + h + ": craft +X");
                NearVec (site.Y, TowerSiteGeometry.Rotate (q, new[] { 0.0, 1.0, 0.0 }), 1e-12, "h " + h + ": craft +Y");
                NearVec (site.Z, TowerSiteGeometry.Rotate (q, new[] { 0.0, 0.0, 1.0 }), 1e-12, "h " + h + ": craft +Z");
                NearVec (up, site.Y, 1e-15, "h " + h + ": +Y vertical");
                Near (h, TowerSiteGeometry.AzimuthDeg (lat, lon, site.X), 1e-9, "h " + h + ": +X at the heading");
                Near ((h + 270.0) % 360.0, TowerSiteGeometry.AzimuthDeg (lat, lon, site.Z), 1e-9, "h " + h + ": +Z at heading - 90");
                // A proper rotation (the craft's handedness kept): X x Y . Z the same at every heading.
                Near (TowerSiteGeometry.Dot (TowerSiteGeometry.Cross (site.X, site.Y), site.Z),
                      TowerSiteGeometry.Dot (TowerSiteGeometry.Cross (new[] { 1.0, 0.0, 0.0 }, new[] { 0.0, 1.0, 0.0 }), new[] { 0.0, 0.0, 1.0 }),
                      1e-12, "h " + h + ": handedness");
            }
            // Another heading = the heading-90 orientation turned about the vertical by the difference.
            var r90 = new TowerSiteGeometry.Site (lat, lon, 90.0).PartRotation (RootInCraft);
            var r120 = new TowerSiteGeometry.Site (lat, lon, 120.0).PartRotation (RootInCraft);
            Near (30.0, TowerSiteGeometry.QuaternionAngleDeg (r90, r120), 1e-9, "30 deg more heading: a 30 deg turn");
            var rel = TowerSiteGeometry.Multiply (r120, TowerSiteGeometry.Inverse (r90));
            var axis = TowerSiteGeometry.Normalize (new[] { rel [0], rel [1], rel [2] });
            Near (1.0, Math.Abs (TowerSiteGeometry.Dot (axis, up)), 1e-12, "the turn is about the local vertical");
        }

        static void TestFromAxesRoundTrip ()
        {
            var rng = new Random (20261012);
            for (int k = 0; k < 2000; k++) {
                var q = TowerSiteGeometry.Normalize (new[] {
                    rng.NextDouble () * 2 - 1, rng.NextDouble () * 2 - 1, rng.NextDouble () * 2 - 1, rng.NextDouble () * 2 - 1 });
                var x = TowerSiteGeometry.Rotate (q, new[] { 1.0, 0.0, 0.0 });
                var y = TowerSiteGeometry.Rotate (q, new[] { 0.0, 1.0, 0.0 });
                var z = TowerSiteGeometry.Rotate (q, new[] { 0.0, 0.0, 1.0 });
                var back = TowerSiteGeometry.FromAxes (x, y, z);
                if (TowerSiteGeometry.QuaternionAngleDeg (q, back) > 1e-6) {
                    Check (false, "FromAxes round trip " + k);
                    return;
                }
            }
            Check (true, "FromAxes round trip, 2000 rotations");
        }

        // The tower's foot as the footprints of its colliders: a 5 m base whose bottom is 0.015 m under
        // the root, four clamps beside it, the arms 37.5 m up and 20 m out.
        static List<TowerSiteGeometry.Footprint> Tower ()
        {
            return new List<TowerSiteGeometry.Footprint> {
                new TowerSiteGeometry.Footprint { X0 = -2.5, X1 = 2.5, Z0 = -2.5, Z1 = 2.5, Bottom = -0.015, Name = "base" },
                new TowerSiteGeometry.Footprint { X0 = -3.6, X1 = -3.0, Z0 = 0.9, Z1 = 1.5, Bottom = 0.2, Name = "clamp" },
                new TowerSiteGeometry.Footprint { X0 = 5.7, X1 = 20.2, Z0 = -5.7, Z1 = 5.7, Bottom = 37.5, Name = "arms" }
            };
        }

        static void TestPlaceFlat ()
        {
            var prints = Tower ();
            var p = TowerSiteGeometry.Place (prints, double.NaN, 0.5, (x, z) => 64.743, KerbinRadius);
            // Flat: the base on the ground, its root 0.015 m up (less rho^2 / 2R at the sample nearest
            // the root axis, under 1e-7 m).
            Near (64.758, p.RootAltitude, 1e-6, "flat: root 0.015 m above the terrain");
            Equal (0, p.Governing, "flat: the base governs");
            Near (64.743, p.TerrainAtRoot, 1e-12, "terrain at the root");
            Near (64.743, p.TerrainMin, 1e-12, "terrain min");
            Near (64.743, p.TerrainMax, 1e-12, "terrain max");
            Near (0.0, p.ErrorBound, 1e-12, "flat: no error between samples");
            Check (p.StepX <= 0.5 && p.StepZ <= 0.5, "steps at most 0.5 m");
            int nx = (int) Math.Round ((20.2 + 3.6) / p.StepX), nz = (int) Math.Round (11.4 / p.StepZ);
            Equal ((nx + 1) * (nz + 1) + 1 + 4 * prints.Count, p.Samples, "grid + root + four corners per footprint");
            Near (Math.Sqrt (20.2 * 20.2 + 5.7 * 5.7), p.Extent, 1e-12, "extent: the farthest corner");
        }

        static void TestPlaceSlopeAndFloor ()
        {
            var prints = Tower ();
            // Rising 2 % toward +X: the base's +X edge is the highest ground under it.
            Func<double, double, double> slope = (x, z) => 64.0 + 0.02 * x;
            var p = TowerSiteGeometry.Place (prints, double.NaN, 0.5, slope, KerbinRadius);
            double rho2 = 2.5 * 2.5 + 2.5 * 2.5;
            Near (64.0 + 0.05 + 0.015 - rho2 / (2.0 * (KerbinRadius + 64.05)), p.RootAltitude, 1e-9, "slope: base corner on the ground");
            Equal (0, p.Governing, "slope: the base governs");
            Near (2.5, p.GoverningX, 1e-12, "slope: at its +X edge");
            // A floor (OnPutToGround raised the lowest point to the clamps' level): the base counts from it.
            var f = TowerSiteGeometry.Place (prints, 0.0, 0.5, (x, z) => 64.0, KerbinRadius);
            Near (64.0, f.RootAltitude, 1e-6, "floor 0: the root on the ground");
            Near (0.0, f.GoverningBottom, 1e-12, "floor 0: the effective bottom");
            // A floor under every collider changes nothing.
            var g = TowerSiteGeometry.Place (prints, -1.0, 0.5, (x, z) => 64.0, KerbinRadius);
            Near (64.015, g.RootAltitude, 1e-6, "floor under the colliders: no effect");
        }

        static void TestPlaceArmsOverAHill ()
        {
            var prints = Tower ();
            // A 10 m mound under the arms: still 27 m under them, the base governs.
            Func<double, double, double> mound = (x, z) => 64.0 + (x > 10.0 && x < 15.0 ? 10.0 : 0.0);
            var p = TowerSiteGeometry.Place (prints, double.NaN, 0.5, mound, KerbinRadius);
            Equal (0, p.Governing, "mound: the base governs");
            Near (64.015, p.RootAltitude, 1e-5, "mound: root unchanged");
            Near (74.0, p.TerrainMax, 1e-12, "mound: seen");
            // A 40 m cliff there: the arms touch it, the tower goes up.
            Func<double, double, double> cliff = (x, z) => 64.0 + (x > 10.0 && x < 15.0 ? 40.0 : 0.0);
            var c = TowerSiteGeometry.Place (prints, double.NaN, 0.5, cliff, KerbinRadius);
            Equal (2, c.Governing, "cliff: the arms govern");
            Check (Math.Abs (c.RootAltitude - (104.0 - 37.5)) < 1e-3, "cliff: arms' bottom on the cliff (got " + c.RootAltitude + ")");
        }

        static void TestPlaceCurvatureAndErrorBound ()
        {
            // A toy body of 100 m: a footprint 10-12 m from the root axis sits rho^2 / 2(R + T) lower.
            var prints = new List<TowerSiteGeometry.Footprint> {
                new TowerSiteGeometry.Footprint { X0 = 10.0, X1 = 12.0, Z0 = 0.0, Z1 = 0.0, Bottom = 0.0, Name = "far" }
            };
            var p = TowerSiteGeometry.Place (prints, double.NaN, 0.5, (x, z) => 5.0, 100.0);
            Near (5.0 - 100.0 / (2.0 * 105.0), p.RootAltitude, 1e-12, "curvature: the nearest point governs");
            Near (10.0, p.GoverningX, 1e-12, "curvature: at x = 10");
            // A quadratic terrain: the bound is k (dx^2 + dz^2) / 4 (second difference 2 k h^2, over 8).
            double k = 0.003;
            var sq = new List<TowerSiteGeometry.Footprint> {
                new TowerSiteGeometry.Footprint { X0 = -4.0, X1 = 4.0, Z0 = -3.0, Z1 = 3.0, Bottom = 0.0, Name = "square" }
            };
            var q = TowerSiteGeometry.Place (sq, double.NaN, 0.5, (x, z) => k * (x * x + z * z), KerbinRadius);
            Near (k * (q.StepX * q.StepX + q.StepZ * q.StepZ) / 4.0, q.ErrorBound, 1e-12, "error bound of a paraboloid");
            Near (k * (16.0 + 9.0), q.TerrainMax, 1e-12, "the corners are sampled exactly");
        }

        static void TestPlaceRefusals ()
        {
            Func<double, double, double> flat = (x, z) => 0.0;
            Check (Throws (() => TowerSiteGeometry.Place (new List<TowerSiteGeometry.Footprint> (), double.NaN, 0.5, flat, 1.0)), "no footprint");
            Check (Throws (() => TowerSiteGeometry.Place (Tower (), double.NaN, 0.0, flat, 1.0)), "step 0");
            Check (Throws (() => TowerSiteGeometry.Place (Tower (), double.NaN, double.NaN, flat, 1.0)), "step NaN");
            Check (Throws (() => TowerSiteGeometry.Place (Tower (), double.NaN, 0.5, flat, 0.0)), "radius 0");
            var bad = Tower ();
            bad [1].X1 = bad [1].X0 - 1.0;
            Check (Throws (() => TowerSiteGeometry.Place (bad, double.NaN, 0.5, flat, 1.0)), "negative extent");
            var nan = Tower ();
            nan [0].Bottom = double.NaN;
            Check (Throws (() => TowerSiteGeometry.Place (nan, double.NaN, 0.5, flat, 1.0)), "NaN bottom");
            Check (Throws (() => TowerSiteGeometry.Place (Tower (), double.NaN, 0.5, (x, z) => double.NaN, 1000.0)), "terrain NaN");
            Check (TowerSiteGeometry.InvalidFootprints (Tower ()) == null, "a valid footprint list");
        }

        static void TestExtentsAndDistances ()
        {
            // At lat 0, lon 0 the vertical is +x of the body frame.
            var parts = new List<double[]> { new[] { 0.0, 10.0, 0.0 }, new[] { 5.0, 0.0, 0.0 }, new[] { 0.0, 0.0, -3.0 } };
            Near (10.0, TowerSiteGeometry.SavedExtent (new[] { 0.0, 0.0, 0.0, 1.0 }, 0.0, 0.0, parts), 1e-12, "extent: the vertical part does not count");
            // The saved Sud tower: its top (51.3 m up the mast) is not an extent, its arm tip (x = -20 in the root) is.
            var sud = new List<double[]> { new[] { 0.0, 51.3, 0.0 }, new[] { -20.239, 37.55, 5.65 } };
            Near (Math.Sqrt (20.239 * 20.239 + 5.65 * 5.65), TowerSiteGeometry.SavedExtent (SudRot, SudLat, SudLon, sud), 2e-3,
                  "Sud: the arm tip's horizontal distance");
            Near (KerbinRadius * Math.PI / 180.0, TowerSiteGeometry.SurfaceDistance (0.0, 10.0, 1.0, 10.0, KerbinRadius), 1e-6, "1 deg of latitude");
            double d = TowerSiteGeometry.SurfaceDistance (SudLat, SudLon, NordLat, NordLon, KerbinRadius + 64.0);
            Check (d > 1200.0 && d < 1205.0, "Sud to Nord: about 1.2 km (got " + d + ")");
        }

        static void TestSiteAndNameRules ()
        {
            Check (TowerSiteGeometry.InvalidSite (-0.15, -74.56, 90.0) == null, "a valid site");
            Check (TowerSiteGeometry.InvalidSite (90.0, 360.0, -720.0) == null, "the bounds are valid");
            Check (TowerSiteGeometry.InvalidSite (90.01, 0.0, 0.0) != null, "latitude > 90");
            Check (TowerSiteGeometry.InvalidSite (double.NaN, 0.0, 0.0) != null, "latitude NaN");
            Check (TowerSiteGeometry.InvalidSite (0.0, 360.5, 0.0) != null, "longitude > 360");
            Check (TowerSiteGeometry.InvalidSite (0.0, double.PositiveInfinity, 0.0) != null, "longitude infinite");
            Check (TowerSiteGeometry.InvalidSite (0.0, 0.0, double.NaN) != null, "heading NaN");
            Near (-74.5, TowerSiteGeometry.NormalizeLongitude (285.5), 1e-12, "longitude 285.5 -> -74.5");
            Near (180.0, TowerSiteGeometry.NormalizeLongitude (-180.0), 1e-12, "longitude -180 -> 180");
            Near (270.0, TowerSiteGeometry.NormalizeHeading (-90.0), 1e-12, "heading -90 -> 270");
            Near (0.0, TowerSiteGeometry.NormalizeHeading (720.0), 1e-12, "heading 720 -> 0");
            Check (TowerSiteFormat.InvalidCraftName ("Tour Catch Sud") == null, "a craft name");
            foreach (var bad in new[] { "", "  ", ".", "..", "../Tour", "VAB/Tour", "SPH\\Tour", "C:Tour", "a\0b" })
                Check (TowerSiteFormat.InvalidCraftName (bad) != null, "craft name refused: '" + bad.Replace ("\0", "\\0") + "'");
            uint id;
            Check (TowerSiteFormat.ParseId ("1057973474", out id) == null && id == 1057973474u, "a persistentId");
            Check (TowerSiteFormat.ParseId ("-1", out id) != null, "a negative persistentId");
            Check (TowerSiteFormat.ParseId ("4294967296", out id) != null, "a persistentId over uint");
            Check (TowerSiteFormat.ParseId (" 12", out id) != null, "a persistentId with a space");
            Check (TowerSiteFormat.ParseId (null, out id) != null, "no persistentId");
        }

        static Dictionary<string, string> Header (string row)
        {
            var head = new Dictionary<string, string> ();
            foreach (var kv in row.Split ('\t')) {
                int eq = kv.IndexOf ('=');
                head [kv.Substring (0, eq)] = kv.Substring (eq + 1);
            }
            return head;
        }

        static void TestTowerSiteFormat ()
        {
            var rows = new List<TowerSiteFormat.TowerRow> {
                new TowerSiteFormat.TowerRow { PersistentId = 1057973474, Name = "Tour Catch Sud", Body = "Kerbin",
                    Latitude = SudLat, Longitude = SudLon, Altitude = 64.891518746269867, Situation = "LANDED" },
                new TowerSiteFormat.TowerRow { PersistentId = 7, Name = "a\tb\nc", Body = "Kerbin", Latitude = double.NaN,
                    Situation = "LANDED", Loaded = true, Packed = false }
            };
            var lines = TowerSiteFormat.Towers ("SPACECENTER", rows);
            Equal (4, lines.Count, "towers: header, columns, two rows");
            var head = Header (lines [0]);
            Equal ("1", head ["protocol"], "towers: protocol");
            Equal ("1", head ["schema"], "towers: schema");
            Equal ("2", head ["n"], "towers: n");
            Equal ("SPACECENTER", head ["scene"], "towers: scene");
            Equal (TowerSiteFormat.TowersHeader.Length, lines [0].Split ('\t').Length, "towers: every header key");
            Equal (string.Join ("\t", TowerSiteFormat.TowersColumns), lines [1], "towers: the columns");
            var a = lines [2].Split ('\t');
            Equal (TowerSiteFormat.TowersColumns.Length, a.Length, "towers: a full row");
            Equal ("1057973474", a [0], "towers: persistent_id");
            Equal (SudLat, double.Parse (a [3], System.Globalization.CultureInfo.InvariantCulture), "towers: lat round trip");
            Equal ("0", a [7], "towers: not loaded");
            Equal ("", a [8], "towers: packed empty when not loaded");
            var b = lines [3].Split ('\t');
            Equal (TowerSiteFormat.TowersColumns.Length, b.Length, "towers: a tab in a name never splits it");
            Equal ("a b c", b [1], "towers: the name cleaned");
            Equal ("", b [3], "towers: NaN is empty");
            Equal ("1", b [7], "towers: loaded");
            Equal ("0", b [8], "towers: unpacked");

            var status = TowerSiteFormat.Status (
                new TowerSiteFormat.StatusHead { PersistentId = 1057973474, Name = "Tour Catch Sud", Source = TowerSiteFormat.SourceSaved,
                    Situation = "LANDED", Tilt = 0.0013, ChargeAmount = 4000.0, ChargeMax = 4000.0 },
                new List<TowerSiteFormat.HingeRow> {
                    new TowerSiteFormat.HingeRow { PartPersistentId = 1, PartFlightId = 2, Angle = 89.9891, Target = 90.0, Locked = true, MotorEngaged = true },
                    new TowerSiteFormat.HingeRow { PartPersistentId = 3, PartFlightId = 4 }
                });
            Equal (4, status.Count, "status: header, columns, two hinges");
            var sh = Header (status [0]);
            Equal (TowerSiteFormat.StatusHeader.Length, status [0].Split ('\t').Length, "status: every header key");
            foreach (var key in TowerSiteFormat.StatusHeader)
                Check (sh.ContainsKey (key), "status header has " + key);
            Equal ("saved", sh ["source"], "status: source");
            Equal ("", sh ["anchored"], "status: anchored empty when not loaded");
            Equal ("", sh ["packed"], "status: packed empty when not loaded");
            Equal ("0", sh ["loaded"], "status: not loaded");
            Equal ("2", sh ["n"], "status: n");
            Equal (string.Join ("\t", TowerSiteFormat.StatusColumns), status [1], "status: the columns");
            Equal ("1\t2\t89.9891\t90\t1\t1", status [2], "status: a hinge");
            Equal ("3\t4\t\t\t\t", status [3], "status: a hinge read nothing: all empty");
        }

        static void TestTowerSiteLayouts ()
        {
            string said = TowerLayoutDeclaration.DeclareTextInto (typeof (LayoutRegistry));
            Check (said.Contains ("layout towers_v1 declared") && said.Contains ("layout tower_status_v1 declared"), "text layouts declared: " + said);
            Equal (0, LayoutRegistry.Problems ().Count, "the descriptors fit the columns");
            foreach (var t in new[] {
                new { Name = TowerSiteFormat.TowersLayout, Header = TowerSiteFormat.TowersHeader, Columns = TowerSiteFormat.TowersColumns },
                new { Name = TowerSiteFormat.StatusLayout, Header = TowerSiteFormat.StatusHeader, Columns = TowerSiteFormat.StatusColumns } }) {
                var lines = LayoutRegistry.Lines (t.Name);
                var head = Header (lines [0]);
                Equal ("TowerSite", head ["service"], t.Name + ": service");
                Equal (LayoutRegistry.KindText, head ["kind"], t.Name + ": kind");
                Equal ("2", head ["sections"], t.Name + ": header and columns");
                Equal ((t.Header.Length + t.Columns.Length).ToString (), head ["fields"], t.Name + ": every field");
                Equal ("#section\theader\t1\t" + t.Header.Length + "\t0\t", lines [1], t.Name + ": header section");
                Equal ("#section\tcolumns\t*\t" + t.Columns.Length + "\t0\t", lines [2 + t.Header.Length], t.Name + ": columns section");
            }
            var units = new Dictionary<string, string> ();
            foreach (var line in LayoutRegistry.Lines (TowerSiteFormat.TowersLayout)) {
                var cells = line.Split ('\t');
                if (cells.Length == 4 && cells [0] == "columns")
                    units [cells [2]] = cells [3];
            }
            Equal ("deg", units ["lat_deg"], "unit of lat_deg");
            Equal ("m", units ["alt_m"], "unit of alt_m");
            Equal ("", units ["persistent_id"], "persistent_id: no unit");
        }
    }
}
