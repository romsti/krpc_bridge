using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.Ocisly
{
    /// <summary>
    /// Every reflective handle into OCISLY (OfCourseIStillLoveYou) and Hullcam VDS,
    /// resolved once per KSP session.
    ///
    /// OCISLY streams Hullcam cameras out of the game over gRPC. It subscribes to no
    /// scene GameEvent and uses no DontDestroyOnLoad, so a flight scene reload - an FMRS
    /// jump, a quickload, a revert - silently drops every camera it was tracking and
    /// nothing brings them back. That is what this plugin exists to fix.
    ///
    /// TWO ASSEMBLY TRAPS, both of which cost a debugging session to find the first time.
    /// OCISLY ships more than one assembly whose name contains "OfCourseIStillLoveYou" -
    /// the KSP plugin and its gRPC client library - and only one holds the types we want,
    /// so the search takes the first candidate that actually has all three rather than
    /// the first whose name matches. Hullcam is the same story under "HullcamVDS".
    /// </summary>
    internal static class OcislyApi
    {
        internal static Type CoreType;            // Core           (static TrackedCameras, GetAllTrackingCameras)
        internal static Type GuiType;             // Gui            (static Fetch, GuiEnabled; OpenCameraInstance)
        internal static Type TrackingCameraType;  // TrackingCamera (Name, StreamingEnabled)
        internal static Type HullcamType;         // MuMechModuleHullCamera, from HullcamVDSContinued

        internal static FieldInfo TrackedCamerasField;     // static Dictionary<int, TrackingCamera>
        internal static MethodInfo GetAllTrackingCameras;  // static List<MuMechModuleHullCamera>
        internal static FieldInfo GuiFetchField;           // static Gui Fetch
        internal static FieldInfo GuiEnabledField;         // static bool GuiEnabled
        internal static MethodInfo OpenCameraInstance;     // instance, takes a MuMechModuleHullCamera
        internal static PropertyInfo StreamingEnabledProp;
        internal static FieldInfo StreamingEnabledField;   // fallback if it is a field, not a property
        internal static PropertyInfo CameraNameProp;       // TrackingCamera.Name
        internal static FieldInfo CameraNameField;
        internal static FieldInfo HullcamNameField;        // MuMechModuleHullCamera.cameraName, a [KSPField]

        internal static bool Resolved { get; private set; }

        /// <summary>Member-by-member account of what resolution found.</summary>
        internal static string Report { get; private set; } = "not resolved yet";

        /// <summary>Resolve the OCISLY and Hullcam types. Called once by <see cref="OcislyAddon"/>.</summary>
        internal static PluginStatus Resolve ()
        {
            Resolved = false;

            var candidates = ModRegistry.FindAssembliesContaining ("OfCourseIStillLoveYou");
            if (candidates.Count == 0) {
                Report = "no loaded assembly matches 'OfCourseIStillLoveYou' - mod not installed?";
                return new PluginStatus { Available = false, Report = Report };
            }

            Assembly assembly = null;
            var tried = string.Empty;
            foreach (var candidate in candidates) {
                var core = ModRegistry.FindType (candidate, "Core");
                var gui = ModRegistry.FindType (candidate, "Gui");
                var tracking = ModRegistry.FindType (candidate, "TrackingCamera");
                tried += string.Format ("{0}[Core={1} Gui={2} TrackingCamera={3}] ",
                                        candidate.GetName ().Name,
                                        core != null, gui != null, tracking != null);
                if (core != null && gui != null && tracking != null) {
                    assembly = candidate;
                    CoreType = core;
                    GuiType = gui;
                    TrackingCameraType = tracking;
                    break;
                }
            }
            if (assembly == null) {
                Report = "assemblies found but none holds Core+Gui+TrackingCamera: " + tried
                         + "-- the mod's class names may have changed";
                return new PluginStatus { Available = false, Report = Report };
            }

            ResolveHullcam ();

            TrackedCamerasField = CoreType.GetField ("TrackedCameras", ModRegistry.PubStatic);
            GetAllTrackingCameras = CoreType.GetMethod ("GetAllTrackingCameras", ModRegistry.PubStatic);
            GuiFetchField = GuiType.GetField ("Fetch", ModRegistry.PubStatic);
            GuiEnabledField = GuiType.GetField ("GuiEnabled", ModRegistry.PubStatic);
            OpenCameraInstance = GuiType.GetMethod ("OpenCameraInstance", ModRegistry.PubInst);

            // StreamingEnabled is a property with a private setter in current OCISLY.
            // AnyInst rather than PubInst is deliberate and is the only non-public
            // binding in this repo: the getter is public but the setter is not, and
            // GetProperty with public-only flags would still find it while SetValue then
            // threw. Tolerate a plain field too, in case a future build changes it.
            StreamingEnabledProp = TrackingCameraType.GetProperty ("StreamingEnabled", ModRegistry.AnyInst);
            StreamingEnabledField = TrackingCameraType.GetField ("StreamingEnabled", ModRegistry.AnyInst);
            CameraNameProp = TrackingCameraType.GetProperty ("Name", ModRegistry.AnyInst);
            CameraNameField = TrackingCameraType.GetField ("Name", ModRegistry.AnyInst);

            Resolved = TrackedCamerasField != null && GetAllTrackingCameras != null
                && GuiFetchField != null && GuiEnabledField != null && OpenCameraInstance != null
                && (StreamingEnabledProp != null || StreamingEnabledField != null);

            Report = string.Format (
                "assembly={0} ns={1} | TrackedCameras={2} GetAllTrackingCameras={3} Gui.Fetch={4} "
                + "Gui.GuiEnabled={5} OpenCameraInstance={6} StreamingEnabled={7} TrackingCamera.Name={8} "
                + "|| MuMechModuleHullCamera={9} cameraName={10}",
                assembly.GetName ().Name, CoreType.Namespace ?? "<global>",
                TrackedCamerasField != null, GetAllTrackingCameras != null,
                GuiFetchField != null, GuiEnabledField != null, OpenCameraInstance != null,
                StreamingEnabledProp != null || StreamingEnabledField != null,
                CameraNameProp != null || CameraNameField != null,
                HullcamType != null, HullcamNameField != null);

            return new PluginStatus {
                Available = Resolved,
                ModVersion = ModRegistry.VersionOf (assembly),
                Report = Report
            };
        }

        static void ResolveHullcam ()
        {
            foreach (var candidate in ModRegistry.FindAssembliesContaining ("HullcamVDS")) {
                HullcamType = ModRegistry.FindType (candidate, "MuMechModuleHullCamera");
                if (HullcamType != null)
                    break;
            }
            if (HullcamType != null)
                HullcamNameField = HullcamType.GetField ("cameraName", ModRegistry.PubInst);
            else
                BridgeLog.Warn ("OCISLY", "MuMechModuleHullCamera not found - camera names "
                                + "will fall back to '<vessel>.Hull' and cannot be disambiguated");
        }

        internal static void Require ()
        {
            if (!Resolved)
                throw new InvalidOperationException ("OCISLY is not usable: " + Report);
        }

        /// <summary>OCISLY's table of cameras it is tracking, keyed by Unity instance id.</summary>
        internal static IDictionary TrackedCameras {
            get {
                Require ();
                var table = TrackedCamerasField.GetValue (null) as IDictionary;
                if (table == null)
                    throw new InvalidOperationException ("OCISLY has not initialised its camera table");
                return table;
            }
        }

        /// <summary>Every Hullcam part module on every loaded vessel, as OCISLY sees them.</summary>
        internal static IEnumerable AllHullcams ()
        {
            Require ();
            return GetAllTrackingCameras.Invoke (null, null) as IEnumerable ?? new object[0];
        }

        /// <summary>The display name OCISLY gave a tracked camera, or null before its first repaint.</summary>
        internal static string NameOf (object trackingCamera)
        {
            if (trackingCamera == null)
                return null;
            object raw = null;
            if (CameraNameProp != null)
                raw = CameraNameProp.GetValue (trackingCamera, null);
            else if (CameraNameField != null)
                raw = CameraNameField.GetValue (trackingCamera);
            return raw as string;
        }

        internal static bool IsStreaming (object trackingCamera)
        {
            if (trackingCamera == null)
                return false;
            try {
                if (StreamingEnabledProp != null)
                    return (bool) StreamingEnabledProp.GetValue (trackingCamera, null);
                return (bool) StreamingEnabledField.GetValue (trackingCamera);
            } catch (Exception) {
                return false;
            }
        }

        /// <summary>
        /// Turn a tracked camera's stream on or off.
        ///
        /// Throws rather than returning quietly when neither a usable setter nor a field
        /// is reachable. Resolution only requires one of the two to EXIST, and a
        /// get-only property with no backing field would otherwise make this a silent
        /// no-op while the caller counted it as a camera successfully armed.
        /// </summary>
        internal static void SetStreaming (object trackingCamera, bool value)
        {
            if (StreamingEnabledProp != null) {
                // Private setter: SetValue would throw, so go through the setter method
                // with nonPublic:true.
                var setter = StreamingEnabledProp.GetSetMethod (true);
                if (setter != null) {
                    setter.Invoke (trackingCamera, new object[] { value });
                    return;
                }
            }
            if (StreamingEnabledField != null) {
                StreamingEnabledField.SetValue (trackingCamera, value);
                return;
            }
            throw new InvalidOperationException (
                "OCISLY's TrackingCamera.StreamingEnabled is readable but not writable in "
                + "this build - the bridge cannot arm a camera. " + Report);
        }

        /// <summary>
        /// "&lt;vessel&gt;.&lt;cameraName&gt;" for a Hullcam part - the same string OCISLY
        /// sends on the wire, so a filter written against it matches what a downstream
        /// client sees.
        /// </summary>
        internal static string KeyOf (object hullcam)
        {
            var module = hullcam as PartModule;
            if (module == null)
                return null;
            var camName = HullcamNameField != null
                ? HullcamNameField.GetValue (hullcam) as string : null;
            string vesselName;
            try {
                vesselName = module.vessel != null ? module.vessel.GetDisplayName () : "?";
            } catch (Exception) {
                vesselName = "?";
            }
            return vesselName + "." + (camName ?? "Hull");
        }
    }
}
