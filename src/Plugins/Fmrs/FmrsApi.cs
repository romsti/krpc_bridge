using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.Fmrs
{
    /// <summary>
    /// Every reflective handle into FMRS, resolved once per KSP session.
    ///
    /// WHY REFLECTION AT ALL. FMRS exposes no KSPEvent, no action group and no key
    /// binding, so there is nothing for stock kRPC to reach. It is MIT-licensed and
    /// everything behind its GUI buttons is public, so no NonPublic binding is needed
    /// anywhere in this file - a property worth preserving if you extend it.
    ///
    /// WHERE THE STATE ACTUALLY LIVES, which is the thing that is easy to get wrong.
    /// FMRS is a partial class hierarchy: FMRS_Util holds the persisted state,
    /// FMRS_Core adds the flight logic, and FOUR KSPAddon subclasses derive from
    /// FMRS_Core - FMRS (flight), FMRS_Space_Center, FMRS_TrackingStation and
    /// FMRS_Main_Menu. The state is on the base, so whichever subclass this scene
    /// happens to have is a valid handle to it. Binding only to the flight subclass
    /// makes the whole service dead in the space centre for no reason, which is exactly
    /// where you want to read a recovery report.
    ///
    /// Hence <see cref="Live"/> searches for anything assignable to FMRS_Core.
    ///
    /// Verified against FMRS Continued 1.2.9.6 (assembly FMRSContinued, MIT,
    /// github.com/linuxgurugamer/FMRS).
    /// </summary>
    internal static class FmrsApi
    {
        // -- types ---------------------------------------------------------------
        internal static Type CoreType;        // FMRS.FMRS_Core   (state + flight logic)
        internal static Type SaveUtilType;    // FMRS.FMRS_SAVE_Util (DontDestroyOnLoad)
        internal static Type SaveCatType;     // FMRS.FMRS_Util+save_cat   (nested enum)
        internal static Type VesselStateType; // FMRS.FMRS_Util+vesselstate (nested enum)

        // -- the jump API --------------------------------------------------------
        // Three overloads, and binding by name alone is ambiguous between two of them.
        // Always bind by exact parameter types.
        internal static MethodInfo JumpToVesselGuid;    // jump_to_vessel(Guid, bool)   - jump to a dropped stage
        internal static MethodInfo JumpToVesselMain;    // jump_to_vessel(string)       - back to the main mission
        internal static MethodInfo JumpToVesselSave;    // jump_to_vessel(Guid, string) - load a named FMRS save

        // -- tracked stages ------------------------------------------------------
        internal static FieldInfo DroppedField;         // Dictionary<Guid,string>  Vessels_dropped        guid -> save file
        internal static FieldInfo DroppedNamesField;    // Dictionary<Guid,string>  Vessels_dropped_names  guid -> vessel name
        internal static FieldInfo SubSaveField;         // Dictionary<Guid,string>  Vessel_sub_save
        internal static FieldInfo KerbalDroppedField;   // Dictionary<string,Guid>  Kerbal_dropped
        internal static FieldInfo MainVesselField;      // Guid _SAVE_Main_Vessel

        // -- flight state --------------------------------------------------------
        internal static FieldInfo ArmedField;           // bool _SETTING_Armed            (instance)
        internal static FieldInfo EnabledField;         // bool _SETTING_Enabled          (instance)
        internal static FieldInfo SwitchedField;        // bool _SAVE_Switched_To_Dropped (instance)
        internal static FieldInfo HasLaunchedField;     // bool _SAVE_Has_Launched        (instance)
        internal static FieldInfo KickToMainField;      // bool _SAVE_Kick_To_Main        (instance)
        internal static FieldInfo LaunchedAtField;      // double _SAVE_Launched_At       (instance)

        // -- session settings ----------------------------------------------------
        // These are STATIC on FMRS_Util. FMRS copies them out of its GameParameters node
        // once, from the space-centre addon, so writing the difficulty setting mid-flight
        // does nothing while writing the static takes effect immediately.
        internal static FieldInfo AutoRecoverField;     // static bool _SETTING_Auto_Recover
        internal static FieldInfo ParachutesField;      // static bool _SETTING_Parachutes
        internal static FieldInfo AutoCutOffField;      // static bool _SETTING_Auto_Cut_Off
        internal static FieldInfo MessagesField;        // static bool _SETTING_Messages
        internal static FieldInfo UncontrollableField;  // static bool _SETTING_Control_Uncontrollable
        internal static FieldInfo HideUiField;          // static bool HideFMRSUI

        // -- methods -------------------------------------------------------------
        internal static MethodInfo WriteSaveValues;     // write_save_values_to_file()
        internal static MethodInfo GetVesselState;      // vesselstate get_vessel_state(Guid)
        internal static MethodInfo SetVesselState;      // bool set_vessel_state(Guid, vesselstate)
        internal static MethodInfo CloseFmrs;           // close_FMRS()
        internal static MethodInfo DeleteDropped;       // delete_dropped_vessel(Guid)
        internal static MethodInfo DeleteAllDropped;    // delete_dropped_vessels()
        internal static MethodInfo SaveLandedVessel;    // save_landed_vessel(bool autoRecoverAllowed, bool forceRecover)
        internal static MethodInfo GetSaveValue;        // string get_save_value(save_cat, string)
        internal static MethodInfo ReadRecoverFile;     // read_recover_file()

        // -- recovery ledger -----------------------------------------------------
        internal static FieldInfo RecoverValuesField;   // List<recover_value> recover_values
        internal static FieldInfo RecoverCatField;      // recover_value.cat
        internal static FieldInfo RecoverKeyField;      // recover_value.key
        internal static FieldInfo RecoverValueField;    // recover_value.value

        // -- FMRS_SAVE_Util ------------------------------------------------------
        internal static FieldInfo SaveUtilInstance;     // static FMRS_SAVE_Util Instance   (a FIELD, not a property)
        internal static PropertyInfo JumpInProgressProp;// bool jumpInProgress { get; private set; }

        /// <summary>True when the jump API resolved. Everything else degrades member by member.</summary>
        internal static bool Resolved { get; private set; }

        /// <summary>Member-by-member account of what resolution found.</summary>
        internal static string Report { get; private set; } = "not resolved yet";

        /// <summary>Version of the FMRS assembly, or empty.</summary>
        internal static string ModVersion { get; private set; } = string.Empty;

        /// <summary>
        /// Resolve everything. Called once by <see cref="FmrsAddon"/>.
        ///
        /// The jump API and the extras are resolved separately and reported separately,
        /// on purpose: an FMRS release that renames one recovery field must not take the
        /// ability to fly a booster down with it. Only the jump members feed
        /// <see cref="Resolved"/>; everything else is checked at its own point of use.
        /// </summary>
        internal static PluginStatus Resolve ()
        {
            Resolved = false;

            // The GameData folder is called FMRS but the assembly was renamed to
            // FMRSContinued in 1.2.9.2 for CKAN. Looking for "FMRS" alone returns null on
            // every current install, so both names are always tried.
            var assembly = ModRegistry.FindAssembly ("FMRSContinued", "FMRS");
            if (assembly == null) {
                Report = "no assembly named FMRSContinued or FMRS is loaded - mod not installed?";
                return new PluginStatus { Available = false, Report = Report };
            }
            ModVersion = ModRegistry.VersionOf (assembly);

            CoreType = ModRegistry.FindType (assembly, "FMRS.FMRS_Core");
            SaveUtilType = ModRegistry.FindType (assembly, "FMRS.FMRS_SAVE_Util");
            if (CoreType == null || SaveUtilType == null) {
                Report = "assembly found but its types did not resolve (FMRS_Core="
                         + (CoreType != null) + " FMRS_SAVE_Util=" + (SaveUtilType != null) + ")";
                return new PluginStatus { Available = false, ModVersion = ModVersion, Report = Report };
            }

            ResolveJumpApi ();
            ResolveState ();
            ResolveSettings ();
            ResolveMethods ();
            ResolveRecoveryLedger ();

            Resolved = JumpToVesselGuid != null && JumpToVesselMain != null
                && DroppedField != null && DroppedNamesField != null
                && MainVesselField != null && SaveUtilInstance != null
                && JumpInProgressProp != null;

            Report = string.Format (
                "jump[guid,bool]={0} jump[string]={1} jump[guid,string]={2} Vessels_dropped={3} "
                + "names={4} sub_save={5} kerbals={6} main_vessel={7} SAVE_Util.Instance={8} "
                + "jumpInProgress={9} || state: armed={10} enabled={11} switched={12} launched={13} "
                + "kick_to_main={14} launched_at={15} || settings: auto_recover={16} parachutes={17} "
                + "auto_cutoff={18} messages={19} uncontrollable={20} hide_ui={21} || methods: "
                + "write_save={22} get_state={23} set_state={24} close={25} delete_one={26} "
                + "delete_all={27} save_landed={28} get_save_value={29} read_recover={30} || "
                + "recovery ledger={31}",
                JumpToVesselGuid != null, JumpToVesselMain != null, JumpToVesselSave != null,
                DroppedField != null, DroppedNamesField != null, SubSaveField != null,
                KerbalDroppedField != null, MainVesselField != null, SaveUtilInstance != null,
                JumpInProgressProp != null,
                ArmedField != null, EnabledField != null, SwitchedField != null,
                HasLaunchedField != null, KickToMainField != null, LaunchedAtField != null,
                AutoRecoverField != null, ParachutesField != null, AutoCutOffField != null,
                MessagesField != null, UncontrollableField != null, HideUiField != null,
                WriteSaveValues != null, GetVesselState != null, SetVesselState != null,
                CloseFmrs != null, DeleteDropped != null, DeleteAllDropped != null,
                SaveLandedVessel != null, GetSaveValue != null, ReadRecoverFile != null,
                RecoverValuesField != null && RecoverCatField != null);

            return new PluginStatus { Available = Resolved, ModVersion = ModVersion, Report = Report };
        }

        static void ResolveJumpApi ()
        {
            // FMRS 1.2.9.6 declares THREE overloads of jump_to_vessel:
            //   (Guid, bool)   fly a dropped stage; the bool saves the current state first
            //   (string)       back to the main mission; the argument is ignored entirely
            //   (Guid, string) load a named FMRS save - "before_launch" is revert-to-launch
            // GetMethod(name) alone is ambiguous, and (Guid,bool) versus (Guid,string) are
            // ambiguous even on arity. Bind by exact parameter types or get the wrong one.
            JumpToVesselGuid = CoreType.GetMethod ("jump_to_vessel", new[] { typeof (Guid), typeof (bool) });
            JumpToVesselMain = CoreType.GetMethod ("jump_to_vessel", new[] { typeof (string) });
            JumpToVesselSave = CoreType.GetMethod ("jump_to_vessel", new[] { typeof (Guid), typeof (string) });

            SaveUtilInstance = SaveUtilType.GetField ("Instance", ModRegistry.PubStatic);
            JumpInProgressProp = SaveUtilType.GetProperty ("jumpInProgress", ModRegistry.PubInst);
        }

        static void ResolveState ()
        {
            DroppedField = CoreType.GetField ("Vessels_dropped", ModRegistry.PubInst);
            DroppedNamesField = CoreType.GetField ("Vessels_dropped_names", ModRegistry.PubInst);
            SubSaveField = CoreType.GetField ("Vessel_sub_save", ModRegistry.PubInst);
            KerbalDroppedField = CoreType.GetField ("Kerbal_dropped", ModRegistry.PubInst);
            MainVesselField = CoreType.GetField ("_SAVE_Main_Vessel", ModRegistry.PubInst);

            ArmedField = CoreType.GetField ("_SETTING_Armed", ModRegistry.PubInst);
            EnabledField = CoreType.GetField ("_SETTING_Enabled", ModRegistry.PubInst);
            SwitchedField = CoreType.GetField ("_SAVE_Switched_To_Dropped", ModRegistry.PubInst);
            HasLaunchedField = CoreType.GetField ("_SAVE_Has_Launched", ModRegistry.PubInst);
            KickToMainField = CoreType.GetField ("_SAVE_Kick_To_Main", ModRegistry.PubInst);
            LaunchedAtField = CoreType.GetField ("_SAVE_Launched_At", ModRegistry.PubInst);
        }

        static void ResolveSettings ()
        {
            // FlattenHierarchy is what makes a static declared on the BASE class visible
            // through the derived type. Without it these all come back null even though
            // they are plainly public.
            const BindingFlags staticFlat = BindingFlags.Public | BindingFlags.Static
                                            | BindingFlags.FlattenHierarchy;
            AutoRecoverField = CoreType.GetField ("_SETTING_Auto_Recover", staticFlat);
            ParachutesField = CoreType.GetField ("_SETTING_Parachutes", staticFlat);
            AutoCutOffField = CoreType.GetField ("_SETTING_Auto_Cut_Off", staticFlat);
            MessagesField = CoreType.GetField ("_SETTING_Messages", staticFlat);
            UncontrollableField = CoreType.GetField ("_SETTING_Control_Uncontrollable", staticFlat);
            HideUiField = CoreType.GetField ("HideFMRSUI", staticFlat);
        }

        static void ResolveMethods ()
        {
            WriteSaveValues = CoreType.GetMethod ("write_save_values_to_file", Type.EmptyTypes);
            GetVesselState = CoreType.GetMethod ("get_vessel_state", new[] { typeof (Guid) });
            CloseFmrs = CoreType.GetMethod ("close_FMRS", Type.EmptyTypes);
            DeleteDropped = CoreType.GetMethod ("delete_dropped_vessel", new[] { typeof (Guid) });
            DeleteAllDropped = CoreType.GetMethod ("delete_dropped_vessels", Type.EmptyTypes);
            SaveLandedVessel = CoreType.GetMethod ("save_landed_vessel",
                                                   new[] { typeof (bool), typeof (bool) });
            ReadRecoverFile = CoreType.GetMethod ("read_recover_file", Type.EmptyTypes);

            // vesselstate and save_cat are nested enums on FMRS_Util, so their parameter
            // types have to come from the resolved method rather than being named.
            VesselStateType = GetVesselState != null ? GetVesselState.ReturnType : null;
            if (VesselStateType != null)
                SetVesselState = CoreType.GetMethod ("set_vessel_state",
                                                     new[] { typeof (Guid), VesselStateType });

            foreach (var candidate in CoreType.GetMethods (ModRegistry.PubInst)) {
                if (candidate.Name != "get_save_value")
                    continue;
                var parameters = candidate.GetParameters ();
                if (parameters.Length != 2 || parameters [1].ParameterType != typeof (string))
                    continue;
                GetSaveValue = candidate;
                SaveCatType = parameters [0].ParameterType;
                break;
            }
        }

        static void ResolveRecoveryLedger ()
        {
            RecoverValuesField = CoreType.GetField ("recover_values", ModRegistry.PubInst);
            if (RecoverValuesField == null)
                return;

            // recover_value is a public struct with three public string fields. Its
            // element type comes off the list rather than being named, so a namespace
            // move costs nothing.
            var listType = RecoverValuesField.FieldType;
            if (!listType.IsGenericType)
                return;
            var element = listType.GetGenericArguments () [0];
            RecoverCatField = element.GetField ("cat", ModRegistry.PubInst);
            RecoverKeyField = element.GetField ("key", ModRegistry.PubInst);
            RecoverValueField = element.GetField ("value", ModRegistry.PubInst);
        }

        // --------------------------------------------------------------------------
        // Live handles
        // --------------------------------------------------------------------------

        /// <summary>
        /// Whichever FMRS_Core subclass this scene has.
        ///
        /// Never cache the result. Every one of them is a [KSPAddon] MonoBehaviour and is
        /// destroyed on scene change, so a held reference is a dead Unity object after a
        /// jump - and note that FMRS's own FMRS_Core.Instance is unusable for the same
        /// reason with a twist: FMRS assigns it in its flight Update and never clears it
        /// in OnDestroy, so after leaving flight it is a destroyed MonoBehaviour that only
        /// Unity's fake-null would catch.
        /// </summary>
        internal static object Live {
            get {
                Require ();
                var live = ModRegistry.FindLiveAssignable (CoreType);
                if (live == null)
                    throw new InvalidOperationException (
                        "FMRS is not active in this scene - it runs in flight, the space "
                        + "centre, the tracking station and the main menu, but nowhere else");
                return live;
            }
        }

        /// <summary>The DontDestroyOnLoad save helper, which owns the jump interlock.</summary>
        internal static object SaveUtil {
            get {
                Require ();
                var instance = SaveUtilInstance.GetValue (null);
                if (instance == null)
                    throw new InvalidOperationException (
                        "FMRS_SAVE_Util is not initialised yet - it is created by the first flight scene");
                return instance;
            }
        }

        internal static void Require ()
        {
            if (!Resolved)
                throw new InvalidOperationException ("FMRS is not usable: " + Report);
        }

        internal static void RequireMember (object handle, string name)
        {
            if (handle == null)
                throw new InvalidOperationException (
                    "FMRS." + name + " did not resolve in this FMRS build - " + Report);
        }

        /// <summary>Read a public instance bool off the live FMRS object.</summary>
        internal static bool ReadFlag (FieldInfo field, string name)
        {
            RequireMember (field, name);
            var raw = ModRegistry.Field (field, Live);
            return raw is bool && (bool) raw;
        }

        /// <summary>Read a public static bool.</summary>
        internal static bool ReadStaticFlag (FieldInfo field, string name)
        {
            RequireMember (field, name);
            var raw = ModRegistry.Field (field, null);
            return raw is bool && (bool) raw;
        }

        /// <summary>Parse a vessel id, with a message that says what was wrong.</summary>
        internal static Guid ParseId (string vesselId)
        {
            if (string.IsNullOrEmpty (vesselId))
                throw new ArgumentException ("vessel id is empty");
            try {
                return new Guid (vesselId);
            } catch (FormatException) {
                throw new ArgumentException ("malformed vessel id: " + vesselId);
            } catch (OverflowException) {
                throw new ArgumentException ("malformed vessel id: " + vesselId);
            }
        }

        /// <summary>The Vessels_dropped dictionary of the live FMRS object, never null.</summary>
        internal static IDictionary DroppedTable {
            get {
                var table = ModRegistry.Field (DroppedField, Live) as IDictionary;
                return table ?? new Dictionary<Guid, string> ();
            }
        }

        /// <summary>Ids currently tracked, as strings. Used for validation before a jump.</summary>
        internal static HashSet<string> TrackedIds ()
        {
            var found = new HashSet<string> ();
            foreach (DictionaryEntry entry in DroppedTable)
                if (entry.Key is Guid)
                    found.Add (((Guid) entry.Key).ToString ());
            return found;
        }
    }
}
