using System;
using System.Reflection;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.MechJeb
{
    /// <summary>
    /// Every reflective handle into MechJeb 2, resolved once per KSP session.
    ///
    /// WHY THIS EXISTS RATHER THAN kRPC.MechJeb. The established bridge
    /// (Genhis/KRPC.MechJeb) targets MechJeb 2.14.3 and stopped loading against 2.15.x.
    /// It broke for two reasons, and both are avoidable:
    ///
    ///   1. It looks members up in camelCase. MechJeb 2.15 renamed ComputerModule's
    ///      enabled/users to Enabled/Users. Since ComputerModule is the base of every
    ///      wrapper, that one rename took down every module at once.
    ///   2. It looks TYPES up by name. MechJebModuleAscentAutopilot and
    ///      MechJebModuleAscentGuidance were deleted in 2.15, replaced by an abstract
    ///      base plus per-path subclasses, with the settings moved out into
    ///      MechJebModuleAscentSettings. The second path has since been renamed again -
    ///      it is MechJebModuleAscentPSGAutopilot now, and the enum reads CLASSIC, PSG -
    ///      which is the point: naming any of them here would have rotted twice by now.
    ///
    /// So this plugin never names a MechJebModule* type and never assumes a casing. It
    /// walks the SHORT public members of MechJebCore instead:
    ///
    ///     vessel.GetMasterMechJeb()        (static extension on MuMech.VesselExtensions)
    ///        -&gt; MechJebCore.AscentSettings (field)
    ///        -&gt; MechJebCore.Staging        (field)
    ///        -&gt; MechJebCore.Ascent         (property; resolves which path itself)
    ///
    /// Those names have survived every refactor so far, where the class names have
    /// changed twice. Every lookup below is also tried in both casings, so a future flip
    /// back would not take the service down either.
    ///
    /// Note the assembly hunt: MechJeb builds as MechJeb2.dll but its AssemblyTitle and
    /// AssemblyProduct both say "MuMechLib". Matching on the title finds nothing, and
    /// matching a name fragment can find the wrong DLL - so every candidate is tried and
    /// the one that actually holds MechJebCore wins. Same lesson as the OCISLY
    /// multi-assembly case.
    /// </summary>
    internal static class MechJebApi
    {
        internal static Type CoreType;               // MuMech.MechJebCore (a PartModule)
        internal static Type SettingsType;           // MuMech.MechJebModuleAscentSettings
        internal static MethodInfo GetMasterMechJeb; // static MechJebCore GetMasterMechJeb(Vessel)

        /// <summary>
        /// GetMasterMechJeb as a delegate. EVERY member of this service reaches the core
        /// through it, and a kRPC client can put a stream on any of them - so it can run
        /// sixty times a second, and MethodInfo.Invoke would allocate an object[] each
        /// time. Null when the binding could not be made; Core falls back to Invoke.
        /// </summary>
        static Func<Vessel, object> getMasterFast;

        internal static FieldInfo AscentSettingsField;  // MechJebCore.AscentSettings
        internal static FieldInfo StagingField;         // MechJebCore.Staging
        internal static PropertyInfo AscentProp;        // MechJebCore.Ascent -> Classic or PVG

        internal static PropertyInfo AutostageProp;     // AscentSettings.Autostage
        internal static PropertyInfo AscentTypeProp;    // AscentSettings.AscentType
        internal static PropertyInfo EnabledProp;       // ComputerModule.Enabled, on the ascent module
        internal static PropertyInfo UsersProp;         // ComputerModule.Users (a UserPool)
        internal static FieldInfo UsersField;           // ... or a field, depending on the build

        internal static MethodInfo UserPoolAdd;         // UserPool.Add(object)
        internal static MethodInfo UserPoolRemove;      // UserPool.Remove(object)
        internal static MethodInfo UserPoolClear;       // UserPool.Clear()

        static string poolReport = "unresolved";

        /// <summary>
        /// Our handle in MechJeb's user pools.
        ///
        /// UserPool derives from List&lt;object&gt; and keys on object identity, so the same
        /// instance has to come back out that went in. One per session is right: the pool
        /// is rebuilt with the module on every scene load, so a stale sentinel is simply
        /// absent, never wrong.
        /// </summary>
        internal static readonly object AscentUser = new object ();

        internal static bool Resolved { get; private set; }

        /// <summary>Member-by-member account of what resolution found.</summary>
        internal static string Report { get; private set; } = "not resolved yet";

        internal static PluginStatus Resolve ()
        {
            Resolved = false;

            var candidates = ModRegistry.FindAssembliesContaining ("MechJeb");
            if (candidates.Count == 0) {
                Report = "no loaded assembly matches 'MechJeb' - mod not installed?";
                return new PluginStatus { Available = false, Report = Report };
            }

            Assembly assembly = null;
            Type extensionsType = null;
            var tried = string.Empty;
            foreach (var candidate in candidates) {
                var core = ModRegistry.FindType (candidate, "MuMech.MechJebCore")
                           ?? ModRegistry.FindType (candidate, "MechJebCore");
                var ext = ModRegistry.FindType (candidate, "MuMech.VesselExtensions")
                          ?? ModRegistry.FindType (candidate, "VesselExtensions");
                tried += string.Format ("{0}[MechJebCore={1} VesselExtensions={2}] ",
                                        candidate.GetName ().Name, core != null, ext != null);
                if (core == null)
                    continue;
                assembly = candidate;
                CoreType = core;
                extensionsType = ext;
                break;
            }
            if (assembly == null) {
                Report = "assemblies found but none holds MechJebCore: " + tried;
                return new PluginStatus { Available = false, Report = Report };
            }

            GetMasterMechJeb = FindMasterLookup (extensionsType, assembly);
            getMasterFast = ModRegistry.StaticCall<Vessel> (GetMasterMechJeb);

            AscentSettingsField = FindField (CoreType, "AscentSettings", "ascentSettings");
            StagingField = FindField (CoreType, "Staging", "staging");
            AscentProp = FindProperty (CoreType, "Ascent", "ascent");

            SettingsType = AscentSettingsField != null ? AscentSettingsField.FieldType : null;
            if (SettingsType != null) {
                AutostageProp = FindProperty (SettingsType, "Autostage", "autostage");
                AscentTypeProp = FindProperty (SettingsType, "AscentType", "ascentType");
            }

            // Enabled and Users are read off the DECLARED return type of MechJebCore.Ascent,
            // which is the abstract base - exactly where ComputerModule puts them.
            var ascentType = AscentProp != null ? AscentProp.PropertyType : null;
            if (ascentType != null) {
                EnabledProp = FindProperty (ascentType, "Enabled", "enabled");
                UsersProp = FindProperty (ascentType, "Users", "users");
                UsersField = FindField (ascentType, "Users", "users");
                var poolType = UsersProp != null ? UsersProp.PropertyType
                             : (UsersField != null ? UsersField.FieldType : null);
                ResolveUserPool (poolType);
            }

            // Usable as soon as we can reach a core and its ascent module. Everything
            // else is reported member by member, so a partial resolve still says
            // something useful instead of dying whole.
            Resolved = GetMasterMechJeb != null && AscentProp != null;

            Report = string.Format (
                "assembly={0} | MechJebCore={1} GetMasterMechJeb={2} AscentSettings={3} Staging={4} "
                + "Ascent={5} || Autostage={6} AscentType={7} Enabled={8} Users={9} || pool: {10}",
                assembly.GetName ().Name,
                CoreType != null, GetMasterMechJeb != null, AscentSettingsField != null,
                StagingField != null, AscentProp != null,
                AutostageProp != null, AscentTypeProp != null,
                EnabledProp != null, UsersProp != null || UsersField != null, poolReport);

            return new PluginStatus {
                Available = Resolved,
                ModVersion = ModRegistry.VersionOf (assembly),
                Report = Report
            };
        }

        /// <summary>
        /// Resolve UserPool's Add/Remove/Clear, and insist on UserPool's OWN copies.
        ///
        /// This matters more than it looks. UserPool derives from List&lt;object&gt; and HIDES
        /// Add/Remove/Clear with <c>new</c> - List's versions are not virtual, so they
        /// cannot be overridden. Only UserPool's versions touch the controlled module:
        /// Add sets Enabled true, Remove sets it false when the count reaches zero, Clear
        /// sets it false outright. Binding to List's copies instead would compile, run,
        /// add the object to a list, and engage nothing at all - a silent no-op, which is
        /// the worst failure this plugin could have.
        ///
        /// Hence DeclaredOnly first. The fallback is recorded in the diagnostics rather
        /// than hidden, so "engage does nothing" stays diagnosable from the outside.
        /// </summary>
        static void ResolveUserPool (Type poolType)
        {
            UserPoolAdd = UserPoolRemove = UserPoolClear = null;
            if (poolType == null) {
                poolReport = "no user pool type";
                return;
            }

            const BindingFlags own = BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly;
            UserPoolAdd = poolType.GetMethod ("Add", own, null, new[] { typeof (object) }, null);
            UserPoolRemove = poolType.GetMethod ("Remove", own, null, new[] { typeof (object) }, null);
            UserPoolClear = poolType.GetMethod ("Clear", own, null, Type.EmptyTypes, null);

            var declared = UserPoolAdd != null && UserPoolRemove != null && UserPoolClear != null;

            // Only fall back to inherited members if the pool declares none of its own -
            // that would mean UserPool is not the shape we think it is, and a plain list
            // is at least better than nothing.
            if (UserPoolAdd == null)
                UserPoolAdd = poolType.GetMethod ("Add", ModRegistry.PubInst, null, new[] { typeof (object) }, null);
            if (UserPoolRemove == null)
                UserPoolRemove = poolType.GetMethod ("Remove", ModRegistry.PubInst, null, new[] { typeof (object) }, null);
            if (UserPoolClear == null)
                UserPoolClear = poolType.GetMethod ("Clear", ModRegistry.PubInst, null, Type.EmptyTypes, null);

            poolReport = string.Format ("{0} Add={1} Remove={2} Clear={3}{4}",
                                        poolType.Name, UserPoolAdd != null, UserPoolRemove != null,
                                        UserPoolClear != null,
                                        declared ? " (own)" : " (WARNING: inherited - may not enable the module)");
        }

        /// <summary>
        /// The static that maps a Vessel to its master MechJebCore.
        ///
        /// In current MechJeb it is <c>public static MechJebCore GetMasterMechJeb(this
        /// Vessel)</c> on MuMech.VesselExtensions. If the name ever moves, fall back to
        /// scanning for any public static that takes one Vessel and returns a
        /// MechJebCore: the SHAPE is far more stable than the home.
        /// </summary>
        static MethodInfo FindMasterLookup (Type extensionsType, Assembly assembly)
        {
            var byName = extensionsType == null ? null
                : extensionsType.GetMethod ("GetMasterMechJeb", ModRegistry.PubStatic,
                                            null, new[] { typeof (Vessel) }, null);
            if (byName != null)
                return byName;

            foreach (var type in ModRegistry.SafeTypes (assembly)) {
                MethodInfo[] methods;
                try {
                    methods = type.GetMethods (ModRegistry.PubStatic);
                } catch (Exception) {
                    continue;
                }
                foreach (var method in methods) {
                    if (method.ReturnType != CoreType)
                        continue;
                    var parameters = method.GetParameters ();
                    if (parameters.Length == 1 && parameters [0].ParameterType == typeof (Vessel))
                        return method;
                }
            }
            return null;
        }

        internal static FieldInfo FindField (Type type, params string[] names)
        {
            if (type == null)
                return null;
            foreach (var name in names) {
                var found = type.GetField (name, ModRegistry.PubInst);
                if (found != null)
                    return found;
            }
            return null;
        }

        internal static PropertyInfo FindProperty (Type type, params string[] names)
        {
            if (type == null)
                return null;
            foreach (var name in names) {
                var found = type.GetProperty (name, ModRegistry.PubInst);
                if (found != null)
                    return found;
            }
            return null;
        }

        internal static void Require ()
        {
            if (!Resolved)
                throw new InvalidOperationException ("MechJeb is not usable: " + Report);
        }

        internal static void RequireMember (object handle, string name)
        {
            if (handle == null)
                throw new InvalidOperationException (
                    "MechJeb." + name + " did not resolve in this MechJeb build - " + Report);
        }

        /// <summary>
        /// The master MechJebCore of the active vessel.
        ///
        /// NEVER cache this. MechJeb's own lookup clears its cache every FixedUpdate, it
        /// filters on the part's running flag - which the player can toggle - and the
        /// PartModule is destroyed on every scene change, so a held reference is a dead
        /// Unity object after a jump or a quickload. Fetch it per call.
        /// </summary>
        internal static object Core {
            get {
                Require ();
                var vessel = FlightGlobals.ActiveVessel;
                if (vessel == null)
                    throw new InvalidOperationException ("no active vessel (not in flight?)");
                var core = getMasterFast != null
                    ? getMasterFast (vessel)
                    : GetMasterMechJeb.Invoke (null, new object[] { vessel });
                if (core == null)
                    throw new InvalidOperationException (
                        "this vessel carries no running MechJeb part - add an AR202 case "
                        + "(or a command pod with MechJeb embedded), and check it is enabled");
                return core;
            }
        }

        /// <summary>The master MechJebCore of a vessel, or null. Uses the fast path when bound.</summary>
        internal static object MasterCoreOf (Vessel vessel)
        {
            if (vessel == null || GetMasterMechJeb == null)
                return null;
            return getMasterFast != null
                ? getMasterFast (vessel)
                : GetMasterMechJeb.Invoke (null, new object[] { vessel });
        }

        internal static object AscentModule {
            get {
                RequireMember (AscentProp, "Ascent");
                var module = AscentProp.GetValue (Core, null);
                if (module == null)
                    throw new InvalidOperationException (
                        "MechJeb has no ascent autopilot instance for this vessel");
                return module;
            }
        }

        internal static object Settings {
            get {
                RequireMember (AscentSettingsField, "AscentSettings");
                var settings = ModRegistry.Field (AscentSettingsField, Core);
                if (settings == null)
                    throw new InvalidOperationException ("MechJebCore.AscentSettings is null");
                return settings;
            }
        }

        internal static object PoolOf (object module)
        {
            if (module == null)
                return null;
            return UsersProp != null ? UsersProp.GetValue (module, null)
                                     : ModRegistry.Field (UsersField, module);
        }
    }
}
