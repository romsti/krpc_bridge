using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Service;
using KRPC.Service.Attributes;

using SCVessel = KRPC.SpaceCenter.Services.Vessel;
using SCModule = KRPC.SpaceCenter.Services.Parts.Module;

namespace KRPC.Bridge.Stock
{
    /// <summary>
    /// conn.stock (conn.stock_delta_v before refactor R3) - KSP's stock utilities, first of all its
    /// delta-v simulation (VesselDeltaV) for a vessel.
    ///
    /// WHY THIS EXISTS. kRPC's Vessel.Stages reads every number (start and end mass, vacuum
    /// thrust and Isp) from VesselDeltaV, and refuses with "Delta-v has not been calculated for
    /// this vessel yet" until VesselDeltaV.IsReady. Terminal Guidance's ascent shadow (UPFG, mode
    /// ASCENSION) needs that table. On 01/10/2026, two ascents out of three on the same craft
    /// never got it: refused from the pad to orbit, 184 reads, while the first one of the day had
    /// it. Nothing in the logs said why. StateV1 says what the simulation is doing; RecalculateV1
    /// asks it to run again; UnstickV1 clears a simulation left marked as running.
    ///
    /// Nothing here touches the vessel or its flight: only KSP's own delta-v bookkeeping.
    /// </summary>
    [KRPCService (Name = "Stock", GameScene = GameScene.Flight)]
    public static class StockService
    {
        const BindingFlags PrivateInstance = BindingFlags.Instance | BindingFlags.NonPublic;

        /// <summary>Health check.</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        static object Field (object o, string name)
        {
            var f = o.GetType ().GetField (name, PrivateInstance);
            return f == null ? null : f.GetValue (o);
        }

        static double Flag (bool b)
        {
            return b ? 1.0 : 0.0;
        }

        /// <summary>
        /// The stock delta-v simulation of a vessel, as numbers (1 = true, 0 = false, NaN = not
        /// readable): [0] the vessel has a VesselDeltaV, [1] IsReady, [2] DoStockSimulation,
        /// [3] SimulationRunning, [4] UpdateFlightScene, [5] DeltaVGlobals.DoStockSimulations,
        /// [6] GameSettings.DELTAV_CALCULATIONS_ENABLED, [7] operating stage count,
        /// [8] the vessel is the active vessel, [9] loaded, [10] packed, [11] calcsDirty
        /// (private), [12] a simulation coroutine is held (private), [13] FlightGlobals.ready.
        /// </summary>
        /// <param name="vessel">The vessel.</param>
        [KRPCProcedure]
        public static IList<double> StateV1 (SCVessel vessel)
        {
            if (vessel == null)
                throw new ArgumentNullException ("vessel");
            var v = vessel.InternalVessel;
            var d = v.VesselDeltaV;
            var r = new List<double> ();
            r.Add (Flag (d != null));
            if (d != null) {
                r.Add (Flag (d.IsReady));
                r.Add (Flag (d.DoStockSimulation));
                r.Add (Flag (d.SimulationRunning));
                r.Add (Flag (d.UpdateFlightScene));
            } else {
                r.Add (double.NaN);
                r.Add (double.NaN);
                r.Add (double.NaN);
                r.Add (double.NaN);
            }
            r.Add (Flag (DeltaVGlobals.DoStockSimulations));
            r.Add (Flag (GameSettings.DELTAV_CALCULATIONS_ENABLED));
            double n = double.NaN;
            if (d != null) {
                try {
                    var info = d.OperatingStageInfo;
                    n = info == null ? 0.0 : info.Count;
                } catch (Exception) {
                    n = double.NaN;
                }
            }
            r.Add (n);
            r.Add (Flag (FlightGlobals.ActiveVessel == v));
            r.Add (Flag (v.loaded));
            r.Add (Flag (v.packed));
            if (d != null) {
                var dirty = Field (d, "calcsDirty");
                r.Add (dirty is bool ? Flag ((bool)dirty) : double.NaN);
                r.Add (Flag (Field (d, "simulation") != null));
            } else {
                r.Add (double.NaN);
                r.Add (double.NaN);
            }
            r.Add (Flag (FlightGlobals.ready));
            return r;
        }

        /// <summary>
        /// Ask the stock delta-v simulation of a vessel to run again: enable it if KSP's
        /// settings allow and it is off, then mark it dirty (part caches reset). Returns false
        /// when the vessel has no VesselDeltaV.
        /// </summary>
        /// <param name="vessel">The vessel.</param>
        [KRPCProcedure]
        public static bool RecalculateV1 (SCVessel vessel)
        {
            if (vessel == null)
                throw new ArgumentNullException ("vessel");
            var d = vessel.InternalVessel.VesselDeltaV;
            if (d == null)
                return false;
            if (!d.DoStockSimulation && GameSettings.DELTAV_CALCULATIONS_ENABLED)
                d.EnableStockSimluation ();
            d.SetCalcsDirty (true, true);
            return true;
        }

        /// <summary>
        /// Clear a stock delta-v simulation left marked as running (a coroutine that died
        /// without resetting its handle never lets another one start), then mark it dirty.
        /// Only for a simulation that stays "running" and never becomes ready. Returns false
        /// when the vessel has no VesselDeltaV or a private field is missing.
        /// </summary>
        /// <param name="vessel">The vessel.</param>
        [KRPCProcedure]
        public static bool UnstickV1 (SCVessel vessel)
        {
            if (vessel == null)
                throw new ArgumentNullException ("vessel");
            var d = vessel.InternalVessel.VesselDeltaV;
            if (d == null)
                return false;
            var simulation = d.GetType ().GetField ("simulation", PrivateInstance);
            var flightScene = d.GetType ().GetField ("updateFlightScene", PrivateInstance);
            if (simulation == null || flightScene == null)
                return false;
            simulation.SetValue (d, null);
            flightScene.SetValue (d, false);
            d.SetCalcsDirty (true, true);
            return true;
        }

        /// <summary>
        /// Activates the next stage of the active vessel at the next frame, from the frame loop
        /// like the space bar (see FrameStaging), not from the physics loop where kRPC runs this
        /// call. Returns the count of such stagings done before this request: wait for
        /// StagesActivatedInFrameV1 to exceed it. Throws when staging is locked.
        /// </summary>
        [KRPCProcedure]
        public static int StageInFrameV1 ()
        {
            if (FlightGlobals.ActiveVessel == null)
                throw new InvalidOperationException ("no active vessel");
            if (InputLockManager.IsLocked (ControlTypes.STAGING))
                throw new InvalidOperationException ("Staging is locked by: " + StagingLockers ());
            return FrameStaging.Request ();
        }

        /// <summary>
        /// StageInFrameV1 that does not stop at the lock KSP sets while its window does not have
        /// the focus ("flightDriver_ApplicationFocus", every control type: FlightDriver.OnApplicationFocus,
        /// KSP 1.12.5 decompiled). Any other lock holding the staging still refuses, with the same
        /// "Staging is locked by: id=0xMASK, ..." message. Same return value, same frame-loop
        /// staging (FrameStaging). Why: StageManager.ActivateNextStage checks no lock and kRPC's
        /// Control.ActivateNextStage only the Alt+L stage lock, but V1 refused on the focus lock, so
        /// an ascent launched from a window beside KSP never left the pad (20/10/2026 16h01 and
        /// 16h02, 16/10 10h49) or kept its spent side boosters (20/10 16h05); see StagingLocks.
        /// </summary>
        [KRPCProcedure]
        public static int StageInFrameV2 ()
        {
            if (FlightGlobals.ActiveVessel == null)
                throw new InvalidOperationException ("no active vessel");
            List<string> holders;
            try {
                holders = StagingLocks.ProgramHolders (InputLockManager.lockStack,
                                                       Convert.ToUInt64 (ControlTypes.STAGING));
            } catch (Exception) {
                // An unreadable lock stack: V1's rule, which never stages through a lock.
                if (InputLockManager.IsLocked (ControlTypes.STAGING))
                    throw new InvalidOperationException ("Staging is locked by: " + StagingLockers ());
                holders = new List<string> ();
            }
            if (holders.Count > 0)
                throw new InvalidOperationException ("Staging is locked by: " + string.Join (", ", holders.ToArray ()));
            return FrameStaging.Request ();
        }

        /// <summary>
        /// The input locks holding the staging now, each as "id=0xMASK" (every id of
        /// InputLockManager.lockStack whose mask includes ControlTypes.STAGING), the focus lock
        /// included: empty when nothing holds it. Lets a client say before a launch that the game
        /// window has lost the focus ("flightDriver_ApplicationFocus"), which StageInFrameV1
        /// refuses and StageInFrameV2 does not. Read-only, never throws ("(ExceptionType)" when
        /// the stack cannot be read).
        /// </summary>
        [KRPCProcedure]
        public static IList<string> StagingLocksV1 ()
        {
            try {
                return StagingLocks.Holders (InputLockManager.lockStack,
                                             Convert.ToUInt64 (ControlTypes.STAGING), null);
            } catch (Exception e) {
                return new List<string> { "(" + e.GetType ().Name + ")" };
            }
        }

        /// <summary>
        /// Who holds the staging lock: every id of InputLockManager.lockStack whose mask includes
        /// ControlTypes.STAGING, each as id=0xMASK, so a refusal says what locked staging (a window
        /// of a mod under the mouse, a focused text field, the stage-lock toggle...). 12/10/2026:
        /// a refusal at 77 s of an ascent left the spent booster attached; the log said only
        /// "Staging is locked". Read-only, never throws.
        /// </summary>
        static string StagingLockers ()
        {
            try {
                var ids = StagingLocks.Holders (InputLockManager.lockStack,
                                                Convert.ToUInt64 (ControlTypes.STAGING), null);
                return ids.Count == 0 ? "(no lock id found)" : string.Join (", ", ids.ToArray ());
            } catch (Exception e) {
                return "(" + e.GetType ().Name + ")";
            }
        }

        /// <summary>The count of stagings done by StageInFrameV1 since the flight scene started.</summary>
        [KRPCProcedure]
        public static int StagesActivatedInFrameV1 ()
        {
            return FrameStaging.Done;
        }

        /// <summary>
        /// The aerodynamic force kRPC's Flight.SimulateAerodynamicForceAt computes, the same
        /// arguments in the same reference frame, without the parts carrying a LaunchClamp module
        /// when withoutClamps is true (see ClampFreeAero). Returns [Fx, Fy, Fz] in newtons, the
        /// count of clamp parts left out and the count of parts summed. With withoutClamps false
        /// it must equal SimulateAerodynamicForceAt.
        /// </summary>
        [KRPCProcedure]
        public static IList<double> AeroForceV1 (SCVessel vessel, KRPC.SpaceCenter.Services.CelestialBody body,
                                                 Tuple<double, double, double> position,
                                                 Tuple<double, double, double> velocity,
                                                 Tuple<double, double, double, double> rotation,
                                                 KRPC.SpaceCenter.Services.ReferenceFrame referenceFrame,
                                                 bool withoutClamps)
        {
            return ClampFreeAero.Force (vessel, body, position, velocity, rotation,
                                         referenceFrame, withoutClamps);
        }

        /// <summary>
        /// The deployment of each landing leg (see LandingLegs): six numbers per leg, in part order --
        /// [0] animLength (s, the animation clip at speed 1), [1] extendDurationFactor (the animation
        /// speed while deploying), [2] retractDurationFactor, [3] position (0..1, now),
        /// [4] deployedPosition, [5] state (0 retracted, 1 deploying, 2 deployed, 3 retracting,
        /// 4 other). A deployment from position lasts animLength * |deployedPosition - position| /
        /// |extendDurationFactor| seconds of game time (the animation runs on physics time). Empty
        /// list: no landing leg. Read-only.
        /// </summary>
        /// <param name="vessel">The vessel.</param>
        [KRPCProcedure]
        public static IList<double> LegsDeploymentV1 (SCVessel vessel)
        {
            return LandingLegs.Deployment (vessel);
        }

        /// <summary>
        /// The stage table kRPC's Vessel.Stages would give, read from the vessel at each call:
        /// seven numbers per activation stage, stages in increasing order, as
        /// [number, start mass (kg), end mass (kg), vacuum thrust (N), vacuum Isp (s),
        /// sea-level thrust (N), sea-level Isp (s)]. Empty while the simulation is not ready.
        ///
        /// WHY. kRPC's object store hands back an existing object when a new one is equal to it,
        /// and a kRPC Stage is equal to another one with the same vessel id and number while it
        /// keeps the Unity vessel it was built with. A revert to launch that reloads the vessel
        /// with the same id (FMRS) therefore gets the Stage objects of the previous flight, whose
        /// vessel is destroyed: "Delta-v has not been calculated for this vessel yet" for as long
        /// as the kRPC server runs (02/10/2026: 60 s of refusals while KSP said ready, 8 stages).
        /// Same stage numbers and same fields as kRPC (ActivationStageNumbers, GetStage).
        /// </summary>
        /// <param name="vessel">The vessel.</param>
        [KRPCProcedure]
        public static IList<double> StagesV1 (SCVessel vessel)
        {
            if (vessel == null)
                throw new ArgumentNullException ("vessel");
            var r = new List<double> ();
            var d = vessel.InternalVessel.VesselDeltaV;
            if (d == null || !d.IsReady)
                return r;
            var info = d.OperatingStageInfo;
            if (info == null)
                return r;
            var numbers = new List<int> ();
            for (int i = 0; i < info.Count; i++)
                if (info [i] != null && !numbers.Contains (info [i].stage))
                    numbers.Add (info [i].stage);
            numbers.Sort ();
            for (int k = 0; k < numbers.Count; k++) {
                var s = d.GetStage (numbers [k]);
                if (s == null)
                    continue;
                r.Add (s.stage);
                r.Add (s.startMass * 1000.0);
                r.Add (s.endMass * 1000.0);
                r.Add (s.thrustVac * 1000.0);
                r.Add (s.ispVac);
                r.Add (s.thrustASL * 1000.0);
                r.Add (s.ispASL);
            }
            return r;
        }

        // ------------------------------------------------------------------ part modules
        //
        // The text members of kRPC's SpaceCenter.Module (fields, events, has_field, get_field,
        // get_field_by_id, trigger_event) that kRPC 0.7.0 removed, in ONE call, the same on 0.6 and
        // 0.7 (see PartModules and ModuleStates). They take the kRPC Module objects the client holds.

        /// <summary>
        /// The events and the visible fields of any number of part modules, in ONE call: what kRPC
        /// 0.6.0's <c>Module.events</c> and <c>Module.fields</c> gave, and what 0.7.0 gives only through
        /// <c>event_list</c> and <c>field_list</c> at one to three RPCs per member. A flat list of strings,
        /// one record per module of <paramref name="modules"/>, in order: <c>moduleName, nEvents, then the
        /// guiName of each event that is visible AND active, nFields, then (guiName, value) for each field
        /// that is visible</c>, fields in the module's order, a repeated guiName kept (the 0.6 dictionary
        /// raised on it; the client does). A count of "-1" stands for a section the game could not read
        /// and is followed by ONE string, the message; a module whose object is gone gives an empty name and
        /// both sections "-1". Without <paramref name="withValues"/> no value is read and each is the empty
        /// string (the question "has this module a field of that name" needs none). Refused with
        /// "[MODULE_ACCESS]" when this kRPC's Module cannot be opened. Read only.
        /// </summary>
        /// <param name="modules">The kRPC modules.</param>
        /// <param name="withValues">Read the value of each visible field.</param>
        [KRPCProcedure]
        public static IList<string> ModuleStatesV1 (IList<SCModule> modules, bool withValues)
        {
            return PartModules.States (modules, withValues);
        }

        /// <summary>
        /// Clicks the first event of the module that is visible AND active and has that guiName:
        /// kRPC 0.6.0's <c>Module.trigger_event</c>. Raises <c>RuntimeError</c> ("Sequence contains no
        /// matching element") when there is none.
        /// </summary>
        /// <param name="module">The kRPC module.</param>
        /// <param name="guiName">The text of the button.</param>
        [KRPCProcedure]
        public static void ModuleTriggerEventV1 (SCModule module, string guiName)
        {
            PartModules.TriggerEvent (module, guiName);
        }

        /// <summary>
        /// The value of the first visible field of the module that has that guiName:
        /// kRPC 0.6.0's <c>Module.get_field</c>. Raises <c>RuntimeError</c> when there is none.
        /// Read only.
        /// </summary>
        /// <param name="module">The kRPC module.</param>
        /// <param name="guiName">The label of the field.</param>
        [KRPCProcedure]
        public static string ModuleFieldV1 (SCModule module, string guiName)
        {
            return PartModules.Field (module, guiName);
        }

        /// <summary>
        /// The value of the first visible field of the module whose identifier is <paramref name="id"/>,
        /// else of the first one of that identifier even if it is not shown: kRPC 0.6.0's
        /// <c>Module.get_field_by_id</c>. Raises <c>RuntimeError</c> when there is none. Read only.
        /// </summary>
        /// <param name="module">The kRPC module.</param>
        /// <param name="id">The identifier of the field.</param>
        [KRPCProcedure]
        public static string ModuleFieldByIdV1 (SCModule module, string id)
        {
            return PartModules.FieldById (module, id);
        }

        // ------------------------------------------------------------------ by persistentId
        //
        // Refactor R3 (state per vessel): the same procedures, the vessel named by its
        // persistentId (decimal string) instead of a kRPC Vessel object. kRPC's object store can
        // hand back a stale Vessel after a revert or an FMRS jump; a persistentId is
        // looked up in FlightGlobals.Vessels at each call. Each one is its V1 twin on a kRPC
        // Vessel built for the vessel found: the same answer, the same refusals.

        /// <summary>
        /// The vessel of that persistentId (decimal string), from FlightGlobals.Vessels, loaded or
        /// not; refused with [NOT_FOUND] when the game has none (no Core here: the code is the
        /// convention of the Core's typed errors, written out).
        /// </summary>
        static SCVessel ByPersistentId (string persistentId)
        {
            uint id;
            if (!uint.TryParse (persistentId, System.Globalization.NumberStyles.None,
                    System.Globalization.CultureInfo.InvariantCulture, out id))
                throw new ArgumentException ("persistentId must be a decimal uint", "persistentId");
            var vessels = FlightGlobals.Vessels;
            for (int i = 0; vessels != null && i < vessels.Count; i++) {
                var v = vessels [i];
                if (v != null && v.persistentId == id)
                    return new SCVessel (v);
            }
            throw new InvalidOperationException ("[NOT_FOUND] no vessel with persistentId " + persistentId);
        }

        /// <summary>StateV1 of the vessel of that persistentId (decimal string).</summary>
        [KRPCProcedure]
        public static IList<double> StateV2 (string persistentId)
        {
            return StateV1 (ByPersistentId (persistentId));
        }

        /// <summary>RecalculateV1 of the vessel of that persistentId (decimal string).</summary>
        [KRPCProcedure]
        public static bool RecalculateV2 (string persistentId)
        {
            return RecalculateV1 (ByPersistentId (persistentId));
        }

        /// <summary>UnstickV1 of the vessel of that persistentId (decimal string).</summary>
        [KRPCProcedure]
        public static bool UnstickV2 (string persistentId)
        {
            return UnstickV1 (ByPersistentId (persistentId));
        }

        /// <summary>StagesV1 of the vessel of that persistentId (decimal string).</summary>
        [KRPCProcedure]
        public static IList<double> StagesV2 (string persistentId)
        {
            return StagesV1 (ByPersistentId (persistentId));
        }

        /// <summary>LegsDeploymentV1 of the vessel of that persistentId (decimal string).</summary>
        [KRPCProcedure]
        public static IList<double> LegsDeploymentV2 (string persistentId)
        {
            return LegsDeploymentV1 (ByPersistentId (persistentId));
        }

        /// <summary>AeroForceV1 of the vessel of that persistentId (decimal string), the other arguments as AeroForceV1.</summary>
        [KRPCProcedure]
        public static IList<double> AeroForceV2 (string persistentId, KRPC.SpaceCenter.Services.CelestialBody body,
                                                 Tuple<double, double, double> position,
                                                 Tuple<double, double, double> velocity,
                                                 Tuple<double, double, double, double> rotation,
                                                 KRPC.SpaceCenter.Services.ReferenceFrame referenceFrame,
                                                 bool withoutClamps)
        {
            return AeroForceV1 (ByPersistentId (persistentId), body, position, velocity, rotation,
                referenceFrame, withoutClamps);
        }
    }
}
