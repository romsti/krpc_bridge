using System.Collections.Generic;
using KRPC.Service;
using KRPC.Service.Attributes;

namespace KRPC.Bridge.Tower
{
    /// <summary>
    /// conn.tower_site - the catch towers of a game, from the Space Center: list them, read their
    /// state (loaded or saved), put one on the ground from a craft of the game, take one away, fly to
    /// one. No Kerbal Konstructs, no restart. A tower is a vessel whose root part is TG.TowerBase.
    /// Vessels are named by their persistentId as a decimal string (the first field of
    /// conn.ident.vessel_ids); no kRPC.SpaceCenter type appears here.
    /// </summary>
    [KRPCService (Name = "TowerSite", GameScene = GameScene.SpaceCenter | GameScene.Flight | GameScene.TrackingStation)]
    public static class TowerSiteService
    {
        /// <summary>The folder of the game being played under saves/ (HighLogic.SaveFolder): its name.</summary>
        [KRPCProcedure]
        public static string SaveFolderV1 ()
        {
            return TowerSite.SaveFolder ();
        }

        /// <summary>
        /// Every catch tower of the game (root part TG.TowerBase), loaded or not, in the order of the
        /// game's vessels (FlightGlobals.Vessels, what the next save writes). Layout towers_v1,
        /// tab-separated text: row 0 the header protocol (1), schema (1), n (towers), scene; row 1 the
        /// columns persistent_id, name, body, lat_deg, lon_deg, alt_m (the root part's position:
        /// the live one when loaded, the saved one otherwise), situation (Vessel.Situations), loaded
        /// (1 or 0), packed (1 or 0, empty when not loaded); then one row per tower. Unknown values
        /// are empty. Read only.
        /// </summary>
        [KRPCProcedure]
        public static IList<string> TowersV1 ()
        {
            return TowerSite.Towers ();
        }

        /// <summary>
        /// The state of one tower, read in the game when it is loaded (source "loaded"), in its saved
        /// state otherwise (source "saved": ProtoVessel, ProtoPartSnapshot, ProtoPartModuleSnapshot).
        /// Layout tower_status_v1: row 0 the header protocol (1), schema (1), persistent_id, name,
        /// source, situation, loaded, packed and anchored (1 or 0; empty when not loaded, anchored
        /// only in flight: TowerAnchor's ground joint), tilt_deg (the root part's +Y against the
        /// local vertical: the live root when loaded, the saved rotation and the vertical of the
        /// saved latitude and longitude otherwise), ec_amount and ec_max (ElectricCharge of every
        /// part), n (hinges); row 1 the columns part_persistent_id, part_flight_id, angle_deg (as
        /// kRPC's RoboticHinge.current_angle: modelInitialAngle + the servo transform's angle about
        /// the module's main axis, saved: from servoTransformRotation), target_angle_deg, locked,
        /// motor_engaged; then one row per ModuleRoboticServoHinge, in part order. Read only.
        /// Refused: [NOT_FOUND] no such vessel, [NOT_A_TOWER] its root is not TG.TowerBase;
        /// ValueError on a persistentId that is not a decimal uint.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static IList<string> TowerStatusV1 (string persistentId)
        {
            return TowerSite.Status (persistentId);
        }

        /// <summary>
        /// Puts a tower on the ground from a craft of this game, at the Space Center only, and saves
        /// the game (persistent.sfs). Reads saves/&lt;game&gt;/Ships/VAB/&lt;craftName&gt;.craft, else
        /// Ships/SPH, else the tower package's GameData/TowerCatch/Crafts, else the bridge's
        /// GameData/KRPC.Bridge/Crafts (where the crafts were before the package: a fallback), nothing
        /// else; its root part must be TG.TowerBase. The vessel is built LANDED on
        /// the home body and added to the game the way KSP adds a saved vessel; it is loaded at the
        /// next flight near it, like any landed vessel, and anchored by TowerAnchor on its first
        /// unpacked tick. Conventions: latitude and longitude are those of the ROOT part (the mast's
        /// foot, where Kerbal Konstructs puts the spawn point; a client computes it from the capture
        /// point); headingDeg is the azimuth of the craft's +X axis (0 north, 90 east, clockwise seen
        /// from above), i.e. for the towers of the craft generator the direction of the arms,
        /// mast to booster, the --cap of tools/site_tour_catch.py: at 90 the tower has exactly the
        /// orientation of the towers Kerbal Konstructs launched with a spawn yaw of 0, any other
        /// heading turns it about the local vertical by (headingDeg - 90) clockwise. The altitude is
        /// computed: every collider at or above the highest terrain under its own footprint, the
        /// lowest one on it, no margin (see docs/API.md). Returns the new persistentId (decimal).
        /// Refused: [WRONG_SCENE] outside the Space Center; [NOT_FOUND] no such craft;
        /// [NOT_A_TOWER] its root is not TG.TowerBase; [OCCUPIED] a landed vessel within the two
        /// extents; [SAVE_DISABLED] the game cannot save; [REFUSED] terrain under the sea, or a root
        /// KSP would lift; ValueError on a latitude outside [-90, 90], a longitude outside
        /// [-360, 360], a heading that is not finite, a craft name that is not a bare file name.
        /// [SAVE_FAILED]: the tower is in the game, persistent.sfs was not written.
        /// </summary>
        /// <param name="craftName">The craft's file name without .craft (e.g. "Tour Catch Sud").</param>
        /// <param name="latitude">Latitude of the root part (deg).</param>
        /// <param name="longitude">Longitude of the root part (deg).</param>
        /// <param name="headingDeg">Azimuth of the craft's +X axis (deg, 0 north, 90 east).</param>
        /// <param name="vesselName">The vessel's name (empty: the craft's ship name).</param>
        /// <returns>The persistentId of the new tower, decimal.</returns>
        [KRPCProcedure]
        public static string TowerSpawnV1 (string craftName, double latitude, double longitude, double headingDeg,
                                           string vesselName)
        {
            return TowerSite.Spawn (craftName, latitude, longitude, headingDeg, vesselName);
        }

        /// <summary>
        /// Takes a tower out of the game, at the Space Center only, as the Tracking Station deletes a
        /// vessel (onVesselTerminated, then the vessel destroyed), and saves the game. Only a tower
        /// (root TG.TowerBase). True when done. Refused: [WRONG_SCENE], [NOT_FOUND], [NOT_A_TOWER],
        /// [SAVE_DISABLED]; [SAVE_FAILED]: the tower is gone, persistent.sfs was not written.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static bool TowerRemoveV1 (string persistentId)
        {
            return TowerSite.Remove (persistentId);
        }

        /// <summary>
        /// Flies to a tower from the Space Center or the Tracking Station, as the Tracking Station's
        /// Fly button does: saves the game (persistent.sfs), then asks for the flight scene on that
        /// vessel (FlightDriver.StartAndFocusVessel). True when the scene change is requested: the
        /// scene loads afterwards, and this connection's next calls answer in flight. Only a tower.
        /// Refused: [WRONG_SCENE] in flight or elsewhere, [NOT_FOUND], [NOT_A_TOWER],
        /// [SAVE_DISABLED], [SAVE_FAILED] (nothing started).
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static bool TowerFocusV1 (string persistentId)
        {
            return TowerSite.Focus (persistentId);
        }

        /// <summary>
        /// The distinct part names of a craft, in the order of its PART nodes: Ships/VAB, then Ships/SPH of the
        /// game, then the crafts of the tower package (GameData/TowerCatch/Crafts), then the ones the bridge
        /// shipped before the package existed (GameData/KRPC.Bridge/Crafts). What an installation must have
        /// loaded to spawn it (see PartsLoadedV1). Read only. [NOT_FOUND] no such craft.
        /// </summary>
        /// <param name="craftName">A craft's file name without .craft.</param>
        [KRPCProcedure]
        public static IList<string> CraftPartNamesV1 (string craftName)
        {
            return TowerSite.CraftPartNames (craftName);
        }

        /// <summary>
        /// Where the tower's patch and craft are in GameData: one row per place, "path" relative to GameData, a
        /// tab, then 1 when the file is there, else 0. In this order: TowerCatch/Patches/TowerCatch.cfg and
        /// KRPC.Bridge/Patches/TowerCatch.cfg (the patch of the tower package, then the one the bridge carried
        /// before the package existed), TowerCatch/Crafts/&lt;craftName&gt;.craft and
        /// KRPC.Bridge/Crafts/&lt;craftName&gt;.craft. Exactly one patch is right: none and the parts do not
        /// exist (KSP removes the towers of a save at load), both and every part is made twice. The game's
        /// Ships folders are not looked at. Read only, any scene. ValueError on a craft name that is not a bare
        /// file name.
        /// </summary>
        /// <param name="craftName">A craft's file name without .craft.</param>
        [KRPCProcedure]
        public static IList<string> TowerPackageV1 (string craftName)
        {
            return TowerSite.PackageFiles (craftName);
        }

        /// <summary>
        /// Which of these parts KSP has loaded (PartLoader.getPartInfoByName): one row per name, in order,
        /// "name", a tab, then 1 (loaded) or 0. A part that a mod or a ModuleManager patch defines is loaded
        /// only when that mod and the patch are installed. Read only.
        /// </summary>
        /// <param name="partNames">Part names as KSP knows them (underscores read as dots).</param>
        [KRPCProcedure]
        public static IList<string> PartsLoadedV1 (IList<string> partNames)
        {
            return TowerSite.PartsLoaded (partNames);
        }

        /// <summary>
        /// Which of these expansions (DLC) are installed (ExpansionsLoader.IsExpansionInstalled): one row per
        /// name, in order, "name", a tab, then 1 or 0. "MakingHistory", "Serenity" (Breaking Ground). Read only.
        /// </summary>
        /// <param name="names">Expansion names.</param>
        [KRPCProcedure]
        public static IList<string> ExpansionsV1 (IList<string> names)
        {
            return TowerSite.Expansions (names);
        }

        /// <summary>
        /// Flight only. Poses the BUBBLE of a tower: the tower is LOADED out to
        /// <paramref name="rangeM"/> while it stays packed, and UNPACKED only once its load has finished.
        /// Loading: load = 0.9 R and unload = R in every situation, pack and unpack the vessel's own. Ready
        /// when the tower is loaded and packed, every part has started (Part.started: rigid body and attach
        /// joint made, robotic servo joints connected), every BaseServo has ServoInitComplete, and the pose
        /// of every rigid body against the root has stayed still, within the float32 resolution of its
        /// world position, over 3 consecutive physics steps (the longest joint work KSP defers once a part
        /// has started, Part.SetJoints). Then pack = R and unpack = 0.9 R as well (kRPC's physics_range
        /// values) and KSP unpacks it on its own path. No range is ever made shorter than the vessel's own;
        /// a bubble already held keeps the larger range. The game does both steps itself (one call); the
        /// status follows (TowerBubbleStatusV1). Not owned by the client: it lasts until
        /// TowerBubbleReleaseV1 (any client) or the end of the flight scene, which writes the original
        /// ranges back. Do not combine with kRPC's physics_range on the same tower (flag overwritten).
        /// Returns the status, layout tower_bubble_v1: [0] protocol (1); [1] state: 0 none, 1 loading,
        /// 2 settling (loaded, packed, load watched), 3 unpacking (ready, unpack ranges written),
        /// 4 unpacked, 5 released, 6 lost; [2] range_m; [3] elapsed_s since the pose (game time, frozen at
        /// the end); [4] loaded and [5] packed now (1 or 0, NaN when unknown); [6] parts, [7] parts_started,
        /// [8] servos, [9] servos_ready (last step); [10] still_ticks, [11] settle_ticks (3); [12] shift_m,
        /// the largest move of a rigid body against the root since the step before, [13] tolerance_m,
        /// [14] settle_shift_max_m (the largest with every part started); [15] loaded_after_s,
        /// [16] ready_after_s, [17] unpacked_after_s (since the pose, NaN until then); [18] early_unpack
        /// (1: the tower got unpacked before its load was finished); [19] overwritten (1: another writer
        /// changed its ranges); [20] loads, [21] unpacks; [22] distance_m to the active vessel.
        /// Refused: [WRONG_SCENE] outside flight, [NOT_FOUND], [NOT_A_TOWER]; ValueError on a range that is
        /// not finite and strictly positive or a persistentId that is not a decimal uint.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        /// <param name="rangeM">The range R (m).</param>
        [KRPCProcedure]
        public static IList<double> TowerBubbleV1 (string persistentId, double rangeM)
        {
            TowerSite.RequireScene ("TowerBubbleV1", GameScenes.FLIGHT);
            return TowerBubbles.Pose (TowerSite.FindTower (persistentId), rangeM);
        }

        /// <summary>
        /// Flight only. The status of the bubble of a tower (layout tower_bubble_v1, as TowerBubbleV1);
        /// state 0 and NaN counts when none was posed in this flight scene. Read only. Same refusals.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static IList<double> TowerBubbleStatusV1 (string persistentId)
        {
            TowerSite.RequireScene ("TowerBubbleStatusV1", GameScenes.FLIGHT);
            return TowerBubbles.Read (TowerSite.FindTower (persistentId));
        }

        /// <summary>
        /// Flight only. Releases the bubble of a tower, whoever posed it: the ranges it had before the pose
        /// written back (state 5), as physics_range = 0 does for kRPC's. Nothing held: nothing done. Returns
        /// the status (layout tower_bubble_v1). Same refusals.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static IList<double> TowerBubbleReleaseV1 (string persistentId)
        {
            TowerSite.RequireScene ("TowerBubbleReleaseV1", GameScenes.FLIGHT);
            return TowerBubbles.Release (TowerSite.FindTower (persistentId));
        }

        /// <summary>
        /// Flight only. Poses the GROUND JOINTS of a tower: what Kerbal Joint Reinforcement fixes to the world
        /// only within 500 m of the floating origin, and only until the first origin shift, fixed here at any distance
        /// and made again at every shift - the parts KJR's rule picks (its 3 heaviest parts, not a clamp, symmetry
        /// included) and the fixed half of every hinge whose parent holds it by a fixed node; never a part on the
        /// moving side of a robotic servo (the arms), never a servo's moving half. One FixedJoint per part, to the
        /// world, unbreakable, on the part's own rigid body (KJR's joint), made on the first physics step where the
        /// tower is loaded, unpacked and landed (now if it is), made again at once in every floating origin shift
        /// (destroyed immediately, then made from the moved bodies: no step without it, none with a stale anchor),
        /// and whenever one is found destroyed by someone else; destroyed when the tower packs, unloads or leaves the
        /// ground (made again at its next unpack), at TowerGroundReleaseV1 (any client) and at the end of the flight
        /// scene. Nothing is fixed while no client poses it. Already posed: nothing changes. Returns the status, layout
        /// tower_ground_v1: [0] protocol (1); [1] state: 0 none, 1 waiting (not loaded, packed, not landed: nothing
        /// fixed now), 2 held, 3 released, 4 lost; [2] joints alive now; [3] parts_fixed, [4] kjr_parts (KJR's rule),
        /// [5] kjr_excluded (mobile or without a body), [6] hinges, [7] hinges_fixed, [8] hinges_skipped,
        /// [9] mass_fixed_tonnes (at the last make, NaN before); [10] makes, [11] remade_after_shift, [12] shifts (seen while
        /// posed), [13] broken (joints found destroyed by someone else), [14] releases (pack, unload, off the ground);
        /// [15] elapsed_s since the pose (game time, frozen at the end), [16] held_after_s (NaN until held); [17] loaded
        /// and [18] packed now (1 or 0, NaN when unknown); [19] origin_distance_m (|the vessel's position|, KJR's
        /// measure, NaN unloaded); [20] frame_rotating (1 or 0) and [21] frame_speed_ms (Krakensbane, m/s) now;
        /// [22] last_shift_m (the move of the last shift seen). Refused: [WRONG_SCENE] outside flight, [NOT_FOUND],
        /// [NOT_A_TOWER]; ValueError on a persistentId that is not a decimal uint.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static IList<double> TowerGroundV1 (string persistentId)
        {
            TowerSite.RequireScene ("TowerGroundV1", GameScenes.FLIGHT);
            return TowerGround.Pose (TowerSite.FindTower (persistentId));
        }

        /// <summary>
        /// Flight only. The status of the ground joints of a tower (layout tower_ground_v1, as TowerGroundV1); state 0
        /// and zero counts when none were posed in this flight scene. Read only. Same refusals.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static IList<double> TowerGroundStatusV1 (string persistentId)
        {
            TowerSite.RequireScene ("TowerGroundStatusV1", GameScenes.FLIGHT);
            return TowerGround.Read (TowerSite.FindTower (persistentId));
        }

        /// <summary>
        /// Flight only. Releases the ground joints of a tower, whoever posed them: every joint destroyed (state 3).
        /// Nothing posed: nothing done. Returns the status (layout tower_ground_v1). Same refusals.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static IList<double> TowerGroundReleaseV1 (string persistentId)
        {
            TowerSite.RequireScene ("TowerGroundReleaseV1", GameScenes.FLIGHT);
            return TowerGround.Release (TowerSite.FindTower (persistentId));
        }

        /// <summary>
        /// Flight only. The parts of the last selection of the ground joints of a tower, layout tower_ground_parts_v1,
        /// tab-separated text: row 0 the header protocol (1), schema (1), persistent_id (the tower), name, state (as the
        /// status), n; row 1 the columns part_persistent_id, part_flight_id, part_name, physics_mass_tonnes (Part.physicsMass,
        /// what KJR sorts by), kjr (1: picked by KJR's rule), hinge (1: a hinge), mobile (1: on a moving side, never
        /// fixed), held (1: a joint holds it now); then one row per part KJR's rule picks and per hinge, the fixed ones
        /// first. No row before the first make. Read only. Same refusals.
        /// </summary>
        /// <param name="persistentId">The tower's persistentId, decimal.</param>
        [KRPCProcedure]
        public static IList<string> TowerGroundPartsV1 (string persistentId)
        {
            TowerSite.RequireScene ("TowerGroundPartsV1", GameScenes.FLIGHT);
            return TowerGround.PartsTable (TowerSite.FindTower (persistentId));
        }

        /// <summary>Health check.</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }
    }
}
