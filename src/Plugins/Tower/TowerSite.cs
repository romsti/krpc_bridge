using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using Expansions.Serenity;
using UnityEngine;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// The game side of conn.tower_site (TowerSiteService): the towers of a game, their state, and
    /// a tower put on the ground or taken away from the Space Center, with no Kerbal Konstructs and
    /// no restart. A tower is a vessel whose ROOT part is TG.TowerBase (TowerAnchor's rule).
    ///
    /// THE LIST OF VESSELS. FlightGlobals.Vessels in every scene: Game.Start loads every saved
    /// vessel as a Vessel object in a scene with a planetarium (flightState.Load() when
    /// HighLogic.LoadedSceneHasPlanetarium: Flight, Space Center, Tracking Station), and
    /// GamePersistence.SaveGame rebuilds the game's flightState from FlightGlobals.Vessels there
    /// (Game.Updated: flightState = new FlightState(), one BackupVessel per vessel; KSP 1.12.5,
    /// decompiled). So FlightGlobals.Vessels is what the next save writes; flightState.protoVessels
    /// is only what the last one read. A vessel's root part is rootPart.partInfo.name when its parts
    /// exist, its ProtoVessel's protoPartSnapshots[rootIndex].partName otherwise.
    ///
    /// No Core reference (see the csproj), no kRPC.SpaceCenter type: persistentIds are decimal
    /// strings. Main thread only (kRPC runs every RPC in FixedUpdate).
    /// </summary>
    internal static class TowerSite
    {
        const string Log = "[KRPC.Bridge.Tower] ";

        /// <summary>
        /// The spacing of the terrain samples under the craft's footprint (m): a resolution, never
        /// added to the altitude. The error it leaves between samples is computed from the samples
        /// themselves and logged (TowerSiteGeometry.Placement.ErrorBound).
        /// </summary>
        internal const double SampleStep = 0.5;

        // The layer ShipConstruction.PutShipToGround skips when it looks for a craft's lowest point
        // ("Part Triggers", KSP 1.12.5 decompiled: collider.gameObject.layer == 21).
        const int PartTriggersLayer = 21;

        const string ElectricCharge = "ElectricCharge";
        const string HingeModule = "ModuleRoboticServoHinge";

        // ------------------------------------------------------------------ helpers

        internal static void RequireScene (string procedure, params GameScenes[] allowed)
        {
            var now = HighLogic.LoadedScene;
            foreach (var s in allowed)
                if (now == s)
                    return;
            var names = new string [allowed.Length];
            for (int i = 0; i < allowed.Length; i++)
                names [i] = allowed [i].ToString ();
            throw new InvalidOperationException ("[WRONG_SCENE] " + procedure + " runs in " + string.Join (", ", names)
                                                 + " only; the game is in " + now);
        }

        static Game RequireGame ()
        {
            var game = HighLogic.CurrentGame;
            if (game == null || game.flightState == null)
                throw new InvalidOperationException ("[TRANSIENT] no game loaded");
            return game;
        }

        // GamePersistence.SaveGame writes nothing to "persistent" when the game's FLIGHT parameter
        // CanAutoSave is off (decompiled): refuse before touching anything.
        static void RequireAutoSave (Game game, string procedure)
        {
            if (game.Parameters == null || game.Parameters.Flight == null || !game.Parameters.Flight.CanAutoSave)
                throw new InvalidOperationException ("[SAVE_DISABLED] " + procedure + " saves the game, and this game's "
                                                     + "FLIGHT parameter CanAutoSave is off (persistent.sfs would not be written)");
        }

        static List<Vessel> Vessels ()
        {
            if (FlightGlobals.fetch == null)
                throw new InvalidOperationException ("[TRANSIENT] no FlightGlobals in this scene yet");
            return FlightGlobals.Vessels;
        }

        /// <summary>The name of a vessel's root part, loaded or not; null when it cannot be read.</summary>
        internal static string RootName (Vessel v)
        {
            if (v == null)
                return null;
            if (v.rootPart != null && v.rootPart.partInfo != null)
                return v.rootPart.partInfo.name;
            var pv = v.protoVessel;
            if (pv != null && pv.protoPartSnapshots != null && pv.rootIndex >= 0 && pv.rootIndex < pv.protoPartSnapshots.Count) {
                var root = pv.protoPartSnapshots [pv.rootIndex];
                return root == null ? null : root.partName;
            }
            return null;
        }

        static Vessel Find (string persistentId)
        {
            uint id;
            var bad = TowerSiteFormat.ParseId (persistentId, out id);
            if (bad != null)
                throw new ArgumentException (bad, "persistentId");
            var vessels = Vessels ();
            for (int i = 0; i < vessels.Count; i++) {
                var v = vessels [i];
                if (v != null && v.persistentId == id)
                    return v;
            }
            throw new InvalidOperationException ("[NOT_FOUND] no vessel with persistentId " + persistentId + " in this game");
        }

        internal static Vessel FindTower (string persistentId)
        {
            var v = Find (persistentId);
            var root = RootName (v);
            if (root != TowerAnchor.RootPartName)
                throw new InvalidOperationException ("[NOT_A_TOWER] vessel " + v.persistentId + " (" + v.vesselName
                                                     + ") has the root part " + (root ?? "(unreadable)") + ", not "
                                                     + TowerAnchor.RootPartName);
            return v;
        }

        static CelestialBody Body (Vessel v)
        {
            if (v.orbitDriver != null && v.orbitDriver.orbit != null && v.orbitDriver.orbit.referenceBody != null)
                return v.orbitDriver.orbit.referenceBody;
            var pv = v.protoVessel;
            if (pv != null && pv.orbitSnapShot != null && FlightGlobals.Bodies != null) {
                int i = pv.orbitSnapShot.ReferenceBodyIndex;
                if (i >= 0 && i < FlightGlobals.Bodies.Count)
                    return FlightGlobals.Bodies [i];
            }
            return null;
        }

        static double[] D (Quaternion q)
        {
            return new double[] { q.x, q.y, q.z, q.w };
        }

        static Vector3d V (double[] v)
        {
            return new Vector3d (v [0], v [1], v [2]);
        }

        static string F (double x)
        {
            return x.ToString ("R", CultureInfo.InvariantCulture);
        }

        // ------------------------------------------------------------------ SaveFolderV1, TowersV1

        internal static string SaveFolder ()
        {
            return HighLogic.SaveFolder ?? string.Empty;
        }

        internal static List<string> Towers ()
        {
            var rows = new List<TowerSiteFormat.TowerRow> ();
            foreach (var v in Vessels ()) {
                if (v == null || RootName (v) != TowerAnchor.RootPartName)
                    continue;
                var body = Body (v);
                var pv = v.protoVessel;
                bool live = v.loaded || pv == null;
                rows.Add (new TowerSiteFormat.TowerRow {
                    PersistentId = v.persistentId,
                    Name = v.vesselName,
                    Body = body == null ? string.Empty : body.bodyName,
                    Latitude = live ? v.latitude : pv.latitude,
                    Longitude = live ? v.longitude : pv.longitude,
                    Altitude = live ? v.altitude : pv.altitude,
                    Situation = v.situation.ToString (),
                    Loaded = v.loaded,
                    Packed = v.loaded ? (bool?) v.packed : null
                });
            }
            return TowerSiteFormat.Towers (HighLogic.LoadedScene.ToString (), rows);
        }

        // ------------------------------------------------------------------ TowerStatusV1

        internal static List<string> Status (string persistentId)
        {
            var v = FindTower (persistentId);
            var head = new TowerSiteFormat.StatusHead {
                PersistentId = v.persistentId,
                Name = v.vesselName,
                Situation = v.situation.ToString (),
                Loaded = v.loaded
            };
            var hinges = new List<TowerSiteFormat.HingeRow> ();
            if (v.loaded && v.rootPart != null && v.parts != null) {
                head.Source = TowerSiteFormat.SourceLoaded;
                head.Packed = v.packed;
                head.Anchored = HighLogic.LoadedSceneIsFlight ? (bool?) TowerAnchor.IsAnchored (v) : null;
                head.Tilt = TowerAnchor.Tilt (v);
                double amount = 0.0, max = 0.0;
                bool any = false;
                foreach (var p in v.parts) {
                    if (p == null)
                        continue;
                    var m = p.FindModuleImplementing<ModuleRoboticServoHinge> ();
                    if (m != null)
                        hinges.Add (new TowerSiteFormat.HingeRow {
                            PartPersistentId = p.persistentId,
                            PartFlightId = p.flightID,
                            Angle = HingeRestLocks.CurrentAngle (m),
                            Target = m.targetAngle,
                            Locked = m.servoIsLocked,
                            MotorEngaged = m.servoMotorIsEngaged
                        });
                    if (p.Resources == null)
                        continue;
                    foreach (var r in p.Resources) {
                        if (r == null || r.resourceName != ElectricCharge)
                            continue;
                        amount += r.amount;
                        max += r.maxAmount;
                        any = true;
                    }
                }
                head.ChargeAmount = any ? amount : double.NaN;
                head.ChargeMax = any ? max : double.NaN;
            } else {
                var pv = v.protoVessel;
                if (pv == null || pv.protoPartSnapshots == null)
                    throw new InvalidOperationException ("[NOT_FOUND] vessel " + v.persistentId + " is not loaded and has no saved state");
                head.Source = TowerSiteFormat.SourceSaved;
                head.Tilt = TowerSiteGeometry.TiltDeg (D (pv.rotation), pv.latitude, pv.longitude);
                double amount = 0.0, max = 0.0;
                bool any = false;
                foreach (var pps in pv.protoPartSnapshots) {
                    if (pps == null)
                        continue;
                    if (pps.modules != null)
                        foreach (var mod in pps.modules)
                            if (mod != null && mod.moduleName == HingeModule)
                                hinges.Add (SavedHinge (pps, mod));
                    if (pps.resources == null)
                        continue;
                    foreach (var r in pps.resources) {
                        if (r == null || r.resourceName != ElectricCharge)
                            continue;
                        amount += r.amount;
                        max += r.maxAmount;
                        any = true;
                    }
                }
                head.ChargeAmount = any ? amount : double.NaN;
                head.ChargeMax = any ? max : double.NaN;
            }
            return TowerSiteFormat.Status (head, hinges);
        }

        /// <summary>
        /// A hinge as saved: targetAngle, servoIsLocked, servoMotorIsEngaged as written; the angle
        /// the way kRPC's RoboticHinge.CurrentAngle reads a loaded one, modelInitialAngle +
        /// currentTransformAngle(), from what BaseServo.OnSave wrote of its moving transform
        /// (servoTransformRotation = movingPartObject.transform.localRotation): the Euler angle about
        /// the module's mainAxis, wrapped to (-180, 180] as currentTransformAngle does; the
        /// modelInitialAngle and mainAxis of the part's prefab module.
        /// </summary>
        static TowerSiteFormat.HingeRow SavedHinge (ProtoPartSnapshot pps, ProtoPartModuleSnapshot mod)
        {
            var row = new TowerSiteFormat.HingeRow { PartPersistentId = pps.persistentId, PartFlightId = pps.flightID };
            var values = mod.moduleValues;
            if (values == null)
                return row;
            float target = 0f;
            if (values.TryGetValue ("targetAngle", ref target))
                row.Target = target;
            bool flag = false;
            if (values.TryGetValue ("servoIsLocked", ref flag))
                row.Locked = flag;
            flag = false;
            if (values.TryGetValue ("servoMotorIsEngaged", ref flag))
                row.MotorEngaged = flag;
            var prefab = pps.partInfo != null && pps.partInfo.partPrefab != null
                ? pps.partInfo.partPrefab.FindModuleImplementing<ModuleRoboticServoHinge> () : null;
            var q = Quaternion.identity;
            if (prefab != null && values.TryGetValue ("servoTransformRotation", ref q))
                row.Angle = prefab.modelInitialAngle + EulerAbout (q, prefab.mainAxis);
            return row;
        }

        static double EulerAbout (Quaternion q, string axis)
        {
            var e = q.eulerAngles;
            double a;
            if (axis == "X")
                a = e.x;
            else if (axis == "Y")
                a = e.y;
            else if (axis == "Z")
                a = e.z;
            else
                return double.NaN;
            if (a > 180.0)
                a -= 360.0;
            return a;
        }

        // ------------------------------------------------------------------ TowerSpawnV1

        /// <summary>What a spawn computed while the craft was loaded, for the log and the node.</summary>
        sealed class Plan
        {
            internal ConfigNode Vessel;
            internal string CraftPath = "", Root = "", Type = "";
            internal int Parts, Colliders, RootIndex, DroppedRotations;
            internal double Latitude, Longitude, Heading, Altitude, Hgt, Mass;
            internal double LowestBeforeClamps, FloorBottom;
            internal double[] Rot;
            internal Vector3 CoM, Nrm;
            internal TowerSiteGeometry.Placement Place;
            internal string Governing = "";
            internal uint MissionId, LaunchId, RefId;
            internal int MinLevel, MaxLevel;
            internal string Nearest = "none";
        }

        internal static string Spawn (string craftName, double latitude, double longitude, double headingDeg, string vesselName)
        {
            RequireScene ("TowerSpawnV1", GameScenes.SPACECENTER);
            var game = RequireGame ();
            var bad = TowerSiteGeometry.InvalidSite (latitude, longitude, headingDeg) ?? TowerSiteFormat.InvalidCraftName (craftName);
            if (bad != null)
                throw new ArgumentException (bad);
            RequireAutoSave (game, "TowerSpawnV1");
            double lon = TowerSiteGeometry.NormalizeLongitude (longitude);
            double heading = TowerSiteGeometry.NormalizeHeading (headingDeg);

            string path = CraftPath (craftName);
            var craft = ConfigNode.Load (path);
            if (craft == null)
                throw new InvalidOperationException ("[NOT_FOUND] craft file unreadable: " + path);
            var first = craft.GetNode ("PART");
            string rootName = first == null ? null : PartName (first.GetValue ("part"));
            if (rootName != TowerAnchor.RootPartName)
                throw new InvalidOperationException ("[NOT_A_TOWER] the root part of " + path + " is " + (rootName ?? "(none)")
                                                     + ", not " + TowerAnchor.RootPartName);
            var body = FlightGlobals.GetHomeBody ();
            if (body == null || body.pqsController == null)
                throw new InvalidOperationException ("[TRANSIENT] the home body or its terrain is not ready");
            if (!body.rotates)
                throw new InvalidOperationException ("[UNSUPPORTED] " + body.bodyName + " does not rotate: no surface orbit to save");

            // The vessel type the craft declares (ShipConstruct keeps it internal); the existing towers'
            // (Base) when the craft has none KSP can parse.
            VesselType vesselType;
            var declared = craft.GetValue ("vesselType");
            if (string.IsNullOrEmpty (declared) || !Enum.TryParse (declared, out vesselType)
                || !Enum.IsDefined (typeof (VesselType), vesselType))
                vesselType = VesselType.Base;

            var ship = new ShipConstruct ();
            Plan plan;
            try {
                if (!ship.LoadShip (craft))
                    throw new InvalidOperationException ("[NOT_FOUND] KSP could not load the craft " + path + " (see KSP.log)");
                plan = Prepare (game, ship, body, path, latitude, lon, heading, vesselName, vesselType);
            } finally {
                Discard (ship);
            }

            // The construct is gone (its part ids released): KSP's own vessel creation, the one
            // Game.Start uses for every saved vessel of the scene, then KSP's own save.
            var pv = game.AddVessel (plan.Vessel);
            uint pid = pv.vesselRef != null ? pv.vesselRef.persistentId : pv.persistentId;
            string saved = GamePersistence.SaveGame ("persistent", HighLogic.SaveFolder, SaveMode.OVERWRITE);

            var p = plan.Place;
            Debug.Log (Log + "spawned " + pv.vesselName + " persistentId " + pid + " from " + plan.CraftPath
                       + ": lat " + F (plan.Latitude) + " lon " + F (plan.Longitude) + " heading " + F (plan.Heading)
                       + " alt " + F (plan.Altitude) + " m (terrain at root " + F (p.TerrainAtRoot) + ", hgt " + F (plan.Hgt)
                       + "; terrain under the footprint " + F (p.TerrainMin) + " to " + F (p.TerrainMax) + " m, "
                       + p.Samples + " samples, step " + F (p.StepX) + " x " + F (p.StepZ) + " m, sampling error bound "
                       + F (p.ErrorBound) + " m; governed by " + plan.Governing + " at craft (" + F (p.GoverningX) + ", "
                       + F (p.GoverningZ) + ") on terrain " + F (p.GoverningTerrain) + " m, bottom " + F (p.GoverningBottom)
                       + " m from the root; lowest point " + F (plan.LowestBeforeClamps) + " m, after OnPutToGround "
                       + F (plan.FloorBottom) + " m; extent " + F (p.Extent) + " m, nearest landed vessel " + plan.Nearest
                       + "); rot " + KSPUtil.WriteQuaternion (new Quaternion ((float) plan.Rot [0], (float) plan.Rot [1],
                           (float) plan.Rot [2], (float) plan.Rot [3]))
                       + " (tilt " + F (TowerSiteGeometry.TiltDeg (plan.Rot, plan.Latitude, plan.Longitude)) + " deg), nrm "
                       + KSPUtil.WriteVector (plan.Nrm) + ", CoM " + KSPUtil.WriteVector (plan.CoM) + ", mass "
                       + F (plan.Mass) + " t, " + plan.Parts + " parts, " + plan.Colliders + " colliders, root "
                       + plan.Root + " #" + plan.RootIndex + ", type " + plan.Type + ", ref " + plan.RefId + ", mission " + plan.MissionId
                       + ", launch " + plan.LaunchId + ", PQS " + plan.MinLevel + "-" + plan.MaxLevel + ", sit LANDED, "
                       + plan.DroppedRotations + " never-captured module rotation(s) left to the model; "
                       + (string.IsNullOrEmpty (saved) ? "SAVE FAILED" : "saved to " + HighLogic.SaveFolder + "/" + saved));
            if (string.IsNullOrEmpty (saved))
                throw new InvalidOperationException ("[SAVE_FAILED] the tower " + pid + " is in the game but persistent.sfs "
                                                     + "was not written (see KSP.log)");
            return pid.ToString (CultureInfo.InvariantCulture);
        }

        /// <summary>
        /// saves/&lt;game&gt;/Ships/VAB/&lt;name&gt;.craft, else SPH, else the crafts the tower package ships
        /// (GameData/TowerCatch/Crafts, next to the patch that defines their parts), else the ones the bridge
        /// shipped before the package existed (GameData/KRPC.Bridge/Crafts, kept as a fallback for the
        /// transition): the game's own copy wins, so a player's edited tower is the one spawned.
        /// [NOT_FOUND] naming every place looked at.
        /// </summary>
        static string CraftPath (string craftName)
        {
            var folder = HighLogic.SaveFolder;
            if (string.IsNullOrEmpty (folder))
                throw new InvalidOperationException ("[TRANSIENT] no save folder");
            var ships = Path.GetFullPath (Path.Combine (Path.Combine (Path.Combine (KSPUtil.ApplicationRootPath, "saves"), folder), "Ships"));
            var dirs = new List<string> { Path.Combine (ships, "VAB"), Path.Combine (ships, "SPH") };
            dirs.AddRange (TowerPackage.CraftDirs (GameDataDir ()));
            var tried = new List<string> ();
            var path = TowerPackage.FindCraft (dirs, craftName, File.Exists, tried);
            if (path != null)
                return path;
            throw new InvalidOperationException ("[NOT_FOUND] no craft '" + craftName + "' in this game: " + string.Join (", ", tried.ToArray ()));
        }

        /// <summary>The game's GameData folder.</summary>
        static string GameDataDir ()
        {
            return Path.GetFullPath (Path.Combine (KSPUtil.ApplicationRootPath, "GameData"));
        }

        /// <summary>
        /// Where the tower's files are in GameData (TowerPackage.Rows): the patch and the craft, in the package's
        /// folder and in the bridge's. Read only, any scene.
        /// </summary>
        internal static List<string> PackageFiles (string craftName)
        {
            var bad = TowerSiteFormat.InvalidCraftName (craftName);
            if (bad != null)
                throw new ArgumentException (bad);
            return TowerPackage.Rows (GameDataDir (), craftName, File.Exists);
        }

        /// <summary>
        /// The distinct part names of a craft (CraftPath: the game's, else the package's, else the bridge's), in
        /// the order of its PART nodes: what an installation must have loaded to spawn it. Read only, any scene.
        /// </summary>
        internal static List<string> CraftPartNames (string craftName)
        {
            var bad = TowerSiteFormat.InvalidCraftName (craftName);
            if (bad != null)
                throw new ArgumentException (bad);
            var path = CraftPath (craftName);
            var craft = ConfigNode.Load (path);
            if (craft == null)
                throw new InvalidOperationException ("[NOT_FOUND] craft file unreadable: " + path);
            var names = new List<string> ();
            foreach (var part in craft.GetNodes ("PART")) {
                var name = PartName (part.GetValue ("part"));
                if (!string.IsNullOrEmpty (name) && !names.Contains (name))
                    names.Add (name);
            }
            return names;
        }

        /// <summary>For each part name, "name", a tab and 1 when KSP has loaded that part (PartLoader), else 0.</summary>
        internal static List<string> PartsLoaded (IList<string> partNames)
        {
            var rows = new List<string> ();
            if (partNames == null)
                return rows;
            foreach (var name in partNames) {
                bool loaded = !string.IsNullOrEmpty (name) && PartLoader.getPartInfoByName (name) != null;
                rows.Add (TowerSiteFormat.Clean (name) + "\t" + TowerSiteFormat.Flag (loaded));
            }
            return rows;
        }

        /// <summary>For each expansion ("MakingHistory", "Serenity"), "name", a tab and 1 when installed, else 0.</summary>
        internal static List<string> Expansions (IList<string> names)
        {
            var rows = new List<string> ();
            if (names == null)
                return rows;
            foreach (var name in names) {
                bool installed = !string.IsNullOrEmpty (name) && global::Expansions.ExpansionsLoader.IsExpansionInstalled (name);
                rows.Add (TowerSiteFormat.Clean (name) + "\t" + TowerSiteFormat.Flag (installed));
            }
            return rows;
        }

        // KSPUtil.GetPartName without its exception: the part name is what precedes the first '_'.
        static string PartName (string partField)
        {
            if (string.IsNullOrEmpty (partField))
                return null;
            int i = partField.IndexOf ('_');
            return i < 0 ? partField : partField.Substring (0, i);
        }

        /// <summary>
        /// Everything computed from the loaded construct: the footprints, the placement, the ids, the
        /// part nodes and the vessel node. The construct is still alive here.
        /// </summary>
        static Plan Prepare (Game game, ShipConstruct ship, CelestialBody body, string path, double lat, double lon,
                             double heading, string vesselName, VesselType vesselType)
        {
            if (ship.parts == null || ship.parts.Count == 0)
                throw new InvalidOperationException ("[NOT_FOUND] the craft " + path + " has no part");
            var root = ship.parts [0].localRoot;
            if (root == null || root.partInfo == null || root.partInfo.name != TowerAnchor.RootPartName)
                throw new InvalidOperationException ("[NOT_A_TOWER] the loaded root part of " + path + " is not " + TowerAnchor.RootPartName);
            var plan = new Plan {
                CraftPath = path, Root = root.partInfo.name, Parts = ship.parts.Count, RootIndex = ship.parts.IndexOf (root),
                Latitude = lat, Longitude = lon, Heading = heading, Type = vesselType.ToString ()
            };

            // 1. The footprints, by KSP's own rule for a launch (ShipConstruction.PutShipToGround): the
            // world bounds of every enabled collider not on layer 21, in the craft's own frame (the
            // construct is loaded at the craft file's positions, +Y up), relative to the root's origin.
            // Then the same OnPutToGround message KSP sends: launch clamps size their tower to the
            // craft's lowest point (their `height`, saved) and may raise that lowest point.
            // A part's transform holds its children's (LoadShip parents them), so KSP's per-part
            // GetComponentsInChildren meets a collider once per ancestor: the query keeps KSP's
            // semantics, the footprints take each collider once, named after the part that owns it.
            Physics.SyncTransforms ();
            var r0 = root.transform.position;
            var prints = new List<TowerSiteGeometry.Footprint> ();
            var seen = new HashSet<Collider> ();
            var query = new PartHeightQuery (float.MaxValue);
            foreach (var part in ship.parts) {
                query.lowestOnParts [part] = float.MaxValue;
                foreach (var c in part.GetComponentsInChildren<Collider> ()) {
                    if (c == null || c.gameObject.layer == PartTriggersLayer || !c.enabled)
                        continue;
                    var b = c.bounds;
                    query.lowestPoint = Mathf.Min (query.lowestPoint, b.min.y);
                    query.lowestOnParts [part] = Mathf.Min (query.lowestOnParts [part], b.min.y);
                    if (!seen.Add (c))
                        continue;
                    var owner = c.GetComponentInParent<Part> ();
                    prints.Add (new TowerSiteGeometry.Footprint {
                        X0 = b.min.x - r0.x, X1 = b.max.x - r0.x, Z0 = b.min.z - r0.z, Z1 = b.max.z - r0.z,
                        Bottom = b.min.y - r0.y,
                        Name = (owner != null && owner.partInfo != null ? owner.partInfo.name : part.partInfo.name) + "/" + c.name
                    });
                }
            }
            if (prints.Count == 0)
                throw new InvalidOperationException ("[NOT_FOUND] the craft " + path + " has no collider to stand on");
            plan.Colliders = prints.Count;
            plan.LowestBeforeClamps = query.lowestPoint - r0.y;
            foreach (var part in ship.parts)
                part.SendMessage ("OnPutToGround", query, SendMessageOptions.DontRequireReceiver);
            plan.FloorBottom = query.lowestPoint - r0.y;

            // 2. The altitude: every collider at or above the highest terrain under its footprint
            // (the PQS's analytic height, the one Vessel.getCorrectedLandedAltitude checks a landed
            // vessel against), the craft turned to its heading about the root.
            var site = new TowerSiteGeometry.Site (lat, lon, heading);
            var pqs = body.pqsController;
            double radius = body.Radius;
            double t0 = pqs.GetSurfaceHeight (V (site.Y)) - radius;
            Func<double, double, double> terrain = (x, z) => pqs.GetSurfaceHeight (V (site.Direction (x, z, radius + t0))) - radius;
            var place = TowerSiteGeometry.Place (prints, plan.FloorBottom, SampleStep, terrain, radius);
            plan.Place = place;
            if (place.Governing >= 0)
                plan.Governing = prints [place.Governing].Name;
            if (body.ocean && place.TerrainMin < 0.0)
                throw new InvalidOperationException ("[REFUSED] the terrain under the footprint goes down to " + F (place.TerrainMin)
                                                     + " m, under the sea: a tower stands on dry land");
            if (place.RootAltitude < place.TerrainAtRoot)
                throw new InvalidOperationException ("[REFUSED] the root part's origin would be " + F (place.TerrainAtRoot - place.RootAltitude)
                                                     + " m under the terrain: KSP lifts such a landed vessel when it loads it "
                                                     + "(Vessel.getCorrectedLandedAltitude)");
            plan.Altitude = place.RootAltitude;
            plan.Hgt = place.RootAltitude - place.TerrainAtRoot;

            // 3. Nobody there: no landed vessel of this body whose extent (its parts' origins about
            // its root) meets the craft's own (the farthest footprint corner from the root).
            Clear (body, lat, lon, radius + plan.Altitude, place.Extent, plan);

            // 4. The rotation of the root in the body frame, and what the vessel node says of it.
            var rootInCraft = root.transform.rotation;
            plan.Rot = site.PartRotation (D (rootInCraft));
            var rotQ = new Quaternion ((float) plan.Rot [0], (float) plan.Rot [1], (float) plan.Rot [2], (float) plan.Rot [3]);
            var nrm = TowerSiteGeometry.Rotate (TowerSiteGeometry.Inverse (plan.Rot), site.Y);
            plan.Nrm = new Vector3 ((float) nrm [0], (float) nrm [1], (float) nrm [2]);

            // 5. The parts as AssembleForLaunch sets them: one flightID each, one mission and one
            // launch for all, the craft's flag; their mass brought up to date (prefab + modules) and
            // their temperatures to the air's, the way Part does for a part saved without one.
            double ut = Planetarium.GetUniversalTime ();
            double ambient = FlightGlobals.getExternalTemperature (plan.Altitude, body);
            plan.MissionId = (uint) Guid.NewGuid ().GetHashCode ();
            plan.LaunchId = game.launchID++;
            string flag = string.IsNullOrEmpty (ship.missionFlag) ? game.flagURL : ship.missionFlag;
            var flightIds = new HashSet<uint> ();
            double mass = 0.0;
            var moment = Vector3d.zero;
            int stage = -1;
            foreach (var part in ship.parts) {
                uint fid;
                do {
                    fid = ShipConstruction.GetUniqueFlightID (game.flightState);
                } while (!flightIds.Add (fid));
                part.flightID = fid;
                part.missionID = plan.MissionId;
                part.launchID = plan.LaunchId;
                part.flagURL = flag;
                part.UpdateMass ();
                part.temperature = ambient;
                part.skinTemperature = ambient;
                part.skinUnexposedTemperature = ambient;
                double m = part.mass + part.GetResourceMass ();
                var local = root.transform.InverseTransformPoint (part.transform.TransformPoint (part.CoMOffset));
                moment += (Vector3d) local * m;
                mass += m;
                stage = Math.Max (stage, part.inverseStage);
            }
            plan.Mass = mass;
            plan.CoM = mass > 0.0 ? (Vector3) (moment / mass) : Vector3.zero;
            plan.RefId = ReferencePart (ship, root).flightID;
            bool controllable = false;
            foreach (var part in ship.parts)
                if (part.FindModuleImplementing<ModuleCommand> () != null)
                    controllable = true;

            // 6. The part nodes, written by ProtoPartSnapshot like any saved part (positions and
            // rotations about the root: orgPos and orgRot, which LoadShip set). A ProtoVessel only
            // holds them while storePartRefs indexes parents, symmetry and attach nodes; its
            // persistentId, registered by its constructor, is released at once.
            var partNodes = PartNodes (ship, out plan.DroppedRotations);

            // 7. The vessel node, as KSP saves a landed tower (the two towers of persistent.sfs),
            // with the ORBIT of a point fixed on the rotating surface at this instant (KSP's own
            // rotating-frame velocity, OrbitDriver.TrackRigidbody / Orbit.GetRotFrameVelAtPos).
            plan.MinLevel = body.pqsController.minLevel;
            plan.MaxLevel = body.pqsController.maxLevel;
            var node = new ConfigNode ("VESSEL");
            node.AddValue ("pid", Guid.NewGuid ().ToString ("N"));
            node.AddValue ("persistentId", FlightGlobals.GetUniquepersistentId ());
            node.AddValue ("name", string.IsNullOrEmpty (vesselName) ? ship.shipName : vesselName);
            node.AddValue ("type", plan.Type);
            node.AddValue ("sit", Vessel.Situations.LANDED.ToString ());
            node.AddValue ("landed", true);
            node.AddValue ("skipGroundPositioning", false);
            node.AddValue ("skipGroundPositioningForDroppedPart", false);
            node.AddValue ("vesselSpawning", false);
            node.AddValue ("launchedFrom", string.Empty);
            node.AddValue ("landedAt", string.Empty);
            node.AddValue ("displaylandedAt", string.Empty);
            node.AddValue ("splashed", false);
            node.AddValue ("met", 0.0);
            node.AddValue ("lct", ut);
            node.AddValue ("lastUT", ut);
            node.AddValue ("distanceTraveled", 0.0);
            node.AddValue ("root", plan.RootIndex);
            node.AddValue ("lat", lat);
            node.AddValue ("lon", lon);
            node.AddValue ("alt", plan.Altitude);
            node.AddValue ("hgt", (float) plan.Hgt);
            node.AddValue ("nrm", KSPUtil.WriteVector (plan.Nrm));
            node.AddValue ("rot", KSPUtil.WriteQuaternion (rotQ));
            node.AddValue ("CoM", KSPUtil.WriteVector (plan.CoM));
            node.AddValue ("stg", Math.Max (0, stage + 1));
            node.AddValue ("prst", false);
            node.AddValue ("ref", plan.RefId);
            node.AddValue ("ctrl", controllable);
            node.AddValue ("PQSMin", plan.MinLevel);
            node.AddValue ("PQSMax", plan.MaxLevel);
            node.AddValue ("GroupOverride", 0);
            node.AddValue ("OverrideDefault", ConfigNode.WriteBoolArray (Groups (ship.OverrideDefault)));
            node.AddValue ("OverrideActionControl", ConfigNode.WriteEnumIntArray (Groups (ship.OverrideActionControl)));
            node.AddValue ("OverrideAxisControl", ConfigNode.WriteEnumIntArray (Groups (ship.OverrideAxisControl)));
            node.AddValue ("OverrideGroupNames", ConfigNode.WriteStringArray (Names (ship.OverrideGroupNames)));
            node.AddNode (ProtoVessel.CreateOrbitNode (SurfaceOrbit (body, lat, lon, plan.Altitude, plan.Rot, plan.CoM, ut)));
            foreach (var pn in partNodes)
                node.AddNode (pn);
            node.AddNode ("ACTIONGROUPS");
            node.AddNode (ProtoVessel.CreateDiscoveryNode (DiscoveryLevels.Owned, UntrackedObjectClass.C,
                                                           double.PositiveInfinity, double.PositiveInfinity));
            plan.Vessel = node;
            return plan;
        }

        static T[] Groups<T> (T[] a)
        {
            var r = new T [Vessel.NumOverrideGroups];
            if (a != null)
                Array.Copy (a, r, Math.Min (a.Length, r.Length));
            return r;
        }

        static string[] Names (string[] a)
        {
            var r = Groups (a);
            for (int i = 0; i < r.Length; i++)
                if (r [i] == null)
                    r [i] = string.Empty;
            return r;
        }

        /// <summary>
        /// The part the vessel is controlled from, as AssembleForLaunch picks it: the root when it
        /// commands, else the first crewable part (ShipConstruction.findFirstCrewablePart), else the
        /// first part with a command module (findFirstControlSource needs a Vessel), else the root.
        /// </summary>
        static Part ReferencePart (ShipConstruct ship, Part root)
        {
            if (root.FindModuleImplementing<ModuleCommand> () != null)
                return root;
            var crewable = ShipConstruction.findFirstCrewablePart (root);
            if (crewable != null)
                return crewable;
            foreach (var part in ship.parts)
                if (part.FindModuleImplementing<ModuleCommand> () != null)
                    return part;
            return root;
        }

        /// <summary>
        /// Takes out of every MODULE of a saved part the values that are the zero quaternion
        /// (TowerSiteFormat.IsZeroQuaternion): rotations its module captures at OnStart, which a part built at the
        /// Space Center never ran (ground test of 13/10/2026: both hinges' servoTransformRotation and
        /// jointParentRotation saved as (0, 0, 0, 0), where the Kerbal Konstructs towers hold their model's
        /// rotation). Absent, BaseServo.OnStart takes the model's own, KSP's path for a part never saved;
        /// any other module keeps the field's default, which is that same zero. Returns how many were taken.
        /// </summary>
        static int DropZeroQuaternions (ConfigNode partNode)
        {
            int dropped = 0;
            foreach (var module in partNode.GetNodes ("MODULE")) {
                var names = new List<string> ();
                foreach (ConfigNode.Value v in module.values)
                    if (TowerSiteFormat.IsZeroQuaternion (v.value))
                        names.Add (v.name);
                foreach (var name in names)
                    module.RemoveValue (name);
                dropped += names.Count;
            }
            return dropped;
        }

        static List<ConfigNode> PartNodes (ShipConstruct ship, out int droppedRotations)
        {
            droppedRotations = 0;
            var holder = new ProtoVessel (new ConfigNode (), null);
            // storePartRefs builds each attach node's AttachNodeSnapshot, which looks the attached part
            // up in holder.vesselRef.parts (KSP 1.12.5, decompiled): without a vesselRef it throws a
            // NullReferenceException (ground test of 13/10/2026, a test install). The vessel here is only that
            // list, in the construct's order, the order of the snapshots, so partIdx is the snapshot
            // index. Constructed, not added as a component: no Awake, no scene object, no FlightGlobals
            // entry (Vessel's constructor only initialises fields); Unity notes in the log that a
            // MonoBehaviour was created with new.
            holder.vesselRef = new Vessel { parts = ship.parts };
            try {
                foreach (var part in ship.parts)
                    holder.protoPartSnapshots.Add (new ProtoPartSnapshot (part, holder));
                foreach (var s in holder.protoPartSnapshots)
                    s.storePartRefs ();
                var nodes = new List<ConfigNode> (holder.protoPartSnapshots.Count);
                foreach (var s in holder.protoPartSnapshots) {
                    var n = new ConfigNode ("PART");
                    s.Save (n);
                    droppedRotations += DropZeroQuaternions (n);
                    nodes.Add (n);
                }
                return nodes;
            } finally {
                Vessel registered;
                if (FlightGlobals.fetch != null && FlightGlobals.FindVessel (holder.persistentId, out registered) && registered == null)
                    FlightGlobals.PersistentVesselIds.Remove (holder.persistentId);
            }
        }

        /// <summary>
        /// The orbit of the vessel's centre of mass held on the rotating surface at <paramref name="ut"/>:
        /// position relative to the body (Unity axes swizzled, as OrbitDriver.TrackRigidbody passes it),
        /// velocity Cross(zUpAngularVelocity, -pos), the rotating-frame velocity KSP adds for a landed
        /// vessel (Orbit.GetRotFrameVelAtPos). KSP recomputes it from the position when the vessel's
        /// OrbitDriver starts (landed: UpdateMode.IDLE, TrackRigidbody).
        /// </summary>
        static Orbit SurfaceOrbit (CelestialBody body, double lat, double lon, double alt, double[] rot, Vector3 com, double ut)
        {
            var offset = TowerSiteGeometry.Rotate (rot, new double[] { com.x, com.y, com.z });
            Vector3d rel = body.GetWorldSurfacePosition (lat, lon, alt) - body.position
                           + (Vector3d) (body.bodyTransform.rotation * new Vector3 ((float) offset [0], (float) offset [1], (float) offset [2]));
            var pos = rel.xzy;
            var vel = Vector3d.Cross (body.zUpAngularVelocity, -pos);
            var orbit = new Orbit (0.0, 0.0, body.Radius + alt, 0.0, 0.0, 0.0, ut, body);
            orbit.UpdateFromStateVectors (pos, vel, body, ut);
            return orbit;
        }

        /// <summary>
        /// [OCCUPIED] when a landed, splashed or pre-launch vessel of this body stands closer to the
        /// point than the two extents: the craft's (its farthest collider corner from the root,
        /// horizontally) and the other's (its farthest part origin from its root, horizontally).
        /// </summary>
        static void Clear (CelestialBody body, double lat, double lon, double radial, double extent, Plan plan)
        {
            double nearest = double.PositiveInfinity;
            foreach (var v in Vessels ()) {
                if (v == null || Body (v) != body)
                    continue;
                if (v.situation != Vessel.Situations.LANDED && v.situation != Vessel.Situations.SPLASHED
                    && v.situation != Vessel.Situations.PRELAUNCH)
                    continue;
                var pv = v.protoVessel;
                double vlat = pv != null && !v.loaded ? pv.latitude : v.latitude;
                double vlon = pv != null && !v.loaded ? pv.longitude : v.longitude;
                double other = 0.0;
                if (pv != null && pv.protoPartSnapshots != null) {
                    var positions = new List<double[]> ();
                    foreach (var pps in pv.protoPartSnapshots)
                        if (pps != null)
                            positions.Add (new[] { pps.position.x, pps.position.y, pps.position.z });
                    other = TowerSiteGeometry.SavedExtent (D (pv.rotation), pv.latitude, pv.longitude, positions);
                }
                double d = TowerSiteGeometry.SurfaceDistance (lat, lon, vlat, vlon, radial);
                if (d < nearest) {
                    nearest = d;
                    plan.Nearest = v.vesselName + " (" + v.persistentId + ") " + F (d) + " m, extent " + F (other) + " m";
                }
                if (d < extent + other)
                    throw new InvalidOperationException ("[OCCUPIED] " + v.vesselName + " (persistentId " + v.persistentId
                                                         + ", " + v.situation + ") stands " + F (d) + " m from the point, "
                                                         + "closer than the craft's extent " + F (extent) + " m plus its own "
                                                         + F (other) + " m");
            }
        }

        /// <summary>
        /// Destroys the construct's parts the way KSP disposes of a loaded ShipConstruct (DestroyImmediate,
        /// ShipConstruction.CreateSubassemblyIcon), after releasing the part persistentIds its load
        /// registered (ShipConstruct.LoadShip: CheckPartpersistentId(..., addNewId: true); Part.OnDestroy
        /// does not remove them, only Part.OnDelete does).
        /// </summary>
        static void Discard (ShipConstruct ship)
        {
            if (ship == null || ship.parts == null)
                return;
            var parts = new List<Part> (ship.parts);
            if (FlightGlobals.fetch != null)
                foreach (var part in parts) {
                    if (part == null)
                        continue;
                    Part registered;
                    if (FlightGlobals.FindLoadedPart (part.persistentId, out registered) && registered == part)
                        FlightGlobals.PersistentLoadedPartIds.Remove (part.persistentId);
                }
            foreach (var part in parts)
                if (part != null)
                    UnityEngine.Object.DestroyImmediate (part.transform.root.gameObject);
            ship.Clear ();
        }

        // ------------------------------------------------------------------ TowerRemoveV1, TowerFocusV1

        /// <summary>
        /// Removes a tower the way the Tracking Station deletes a vessel (SpaceTracking.OnVesselDeleteConfirm,
        /// decompiled): onVesselTerminated, the crew back to the roster, DestroyImmediate of the vessel's
        /// object (Vessel.OnDestroy takes it out of FlightGlobals), then the save.
        /// </summary>
        internal static bool Remove (string persistentId)
        {
            RequireScene ("TowerRemoveV1", GameScenes.SPACECENTER);
            var game = RequireGame ();
            var v = FindTower (persistentId);
            RequireAutoSave (game, "TowerRemoveV1");
            string name = v.vesselName;
            uint pid = v.persistentId;
            double lat = v.protoVessel != null ? v.protoVessel.latitude : v.latitude;
            double lon = v.protoVessel != null ? v.protoVessel.longitude : v.longitude;
            if (v.protoVessel != null)
                GameEvents.onVesselTerminated.Fire (v.protoVessel);
            var crew = v.GetVesselCrew ();
            if (crew != null)
                foreach (var member in crew)
                    if (member != null)
                        member.StartRespawnPeriod ();
            UnityEngine.Object.DestroyImmediate (v.gameObject);
            string saved = GamePersistence.SaveGame ("persistent", HighLogic.SaveFolder, SaveMode.OVERWRITE);
            Debug.Log (Log + "removed " + name + " persistentId " + pid + " (lat " + F (lat) + " lon " + F (lon) + "); "
                       + (string.IsNullOrEmpty (saved) ? "SAVE FAILED" : "saved to " + HighLogic.SaveFolder + "/" + saved));
            if (string.IsNullOrEmpty (saved))
                throw new InvalidOperationException ("[SAVE_FAILED] the tower " + pid + " is gone from the game but "
                                                     + "persistent.sfs was not written (see KSP.log)");
            return true;
        }

        /// <summary>
        /// Flies a tower the way the Tracking Station's Fly button does (SpaceTracking.FlyVessel,
        /// decompiled): save "persistent", then FlightDriver.StartAndFocusVessel("persistent", index).
        /// The index is the tower's in the flightState that save just wrote (Game.Updated rebuilds it
        /// from FlightGlobals.Vessels, in their order; the Fly button passes
        /// FlightGlobals.Vessels.IndexOf), so the file the flight scene loads has the tower there.
        /// </summary>
        internal static bool Focus (string persistentId)
        {
            RequireScene ("TowerFocusV1", GameScenes.SPACECENTER, GameScenes.TRACKSTATION);
            var game = RequireGame ();
            var v = FindTower (persistentId);
            RequireAutoSave (game, "TowerFocusV1");
            uint pid = v.persistentId;
            int listed = FlightGlobals.Vessels.IndexOf (v);
            string saved = GamePersistence.SaveGame ("persistent", HighLogic.SaveFolder, SaveMode.OVERWRITE);
            if (string.IsNullOrEmpty (saved))
                throw new InvalidOperationException ("[SAVE_FAILED] persistent.sfs was not written (see KSP.log): no flight started");
            var state = HighLogic.CurrentGame.flightState;
            int index = -1;
            for (int i = 0; state != null && state.protoVessels != null && i < state.protoVessels.Count; i++)
                if (state.protoVessels [i] != null && state.protoVessels [i].persistentId == pid) {
                    index = i;
                    break;
                }
            if (index < 0)
                throw new InvalidOperationException ("[NOT_FOUND] the tower " + pid + " is not in the flight state just saved");
            Debug.Log (Log + "focus " + v.vesselName + " persistentId " + pid + ": saved to " + HighLogic.SaveFolder + "/" + saved
                       + ", flight scene requested on vessel index " + index + " (FlightGlobals.Vessels index " + listed + ")");
            FlightDriver.StartAndFocusVessel ("persistent", index);
            return true;
        }
    }
}
