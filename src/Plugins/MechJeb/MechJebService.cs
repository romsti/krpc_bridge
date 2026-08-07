using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;
using KRPC.Service;
using KRPC.Service.Attributes;
using UnityEngine;

namespace KRPC.Bridge.MechJeb
{
    /// <summary>Registration. Hands Core the resolver and gets out of the way.</summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class MechJebAddon : MonoBehaviour
    {
        void Awake ()
        {
            ModRegistry.Register ("MechJeb", MechJebApi.Resolve);
        }
    }

    /// <summary>
    /// Remote control of MechJeb 2's ascent autopilot and staging controller.
    ///
    /// TWO THINGS IT IS FOR.
    ///
    /// One: fly the ascent under script control. Set the target orbit and the turn, then
    /// engage with <see cref="AscentEnabled"/>.
    ///
    /// Two, and this is the one that is hard to get any other way: fly the ascent with
    /// MechJeb while deciding the STAGING yourself. MechJeb stages when the current stage
    /// has no active engines left, which on a launcher whose side boosters and core sit
    /// in the same stage stays false for as long as the core burns - so the side boosters
    /// are never dropped. No setting fixes that; the criterion itself is the wrong
    /// question for that vehicle shape. Setting <see cref="Autostage"/> to false removes
    /// the ascent autopilot from the staging controller's user pool while leaving it in
    /// the attitude and thrust pools: MechJeb keeps flying, and the staging decision comes
    /// back to you.
    ///
    /// This service is deliberately name-driven rather than one hard-coded property per
    /// knob. <see cref="AscentSettingNames"/> lists what the INSTALLED MechJeb actually
    /// has, and <see cref="SetAscentSetting"/> writes by name - so a MechJeb that renames
    /// a field costs a string change in a script, not a rebuilt DLL. That is the failure
    /// mode that killed the previous kRPC-to-MechJeb bridge, and it is worth designing
    /// around.
    /// </summary>
    [KRPCService (Name = "MechJeb", GameScene = GameScene.All)]
    public static class MechJebService
    {
        // Reflection walks over the settings type, computed once. These are properties,
        // and a kRPC client can put a stream on any of them - which would re-run the walk
        // every physics tick, allocating a fresh list each time. The answer cannot change
        // during a session, since it depends only on which assembly is loaded, so caching
        // makes streaming them harmless instead of a GC problem.
        static IList<string> cachedSettingNames;
        static IList<string> cachedFlagNames;
        static IList<string> cachedCoreMembers;

        /// <summary>
        /// Whether MechJeb is installed and its API resolved. Check this before anything
        /// else; every other member throws when it is <c>false</c>.
        /// </summary>
        [KRPCProperty]
        public static bool Available {
            get { return MechJebApi.Resolved; }
        }

        /// <summary>
        /// What type resolution found, member by member. Read this when
        /// <see cref="Available"/> is false, or when one member throws: it names the exact
        /// lookup that came back empty, which is the same information a MechJeb rename
        /// would change.
        /// </summary>
        [KRPCProperty]
        public static string Diagnostics {
            get { return MechJebApi.Report; }
        }

        /// <summary>
        /// Whether the active vessel actually carries a running MechJeb part.
        ///
        /// Distinct from <see cref="Available"/>: the mod can be installed and resolved
        /// while this particular craft has no MechJeb on it, in which case every command
        /// below throws. False outside flight.
        /// </summary>
        [KRPCProperty]
        public static bool OnVessel {
            get {
                if (!MechJebApi.Resolved)
                    return false;
                try {
                    var vessel = FlightGlobals.ActiveVessel;
                    if (vessel == null)
                        return false;
                    return MechJebApi.MasterCoreOf (vessel) != null;
                } catch (Exception) {
                    return false;
                }
            }
        }

        /// <summary>Round-trip check that the service is loaded and callable. Returns "pong".</summary>
        [KRPCProcedure]
        public static string Ping ()
        {
            return "pong";
        }

        // ==================================================================
        // Ascent
        // ==================================================================

        /// <summary>
        /// Whether MechJeb's ascent autopilot is flying. Settable: this is how a script
        /// launches.
        ///
        /// Writing <c>true</c> adds this bridge to the autopilot's user pool, which is the
        /// path MechJeb's own window uses. The module is enabled while it has at least one
        /// user, so engaging this way COMPOSES with the GUI instead of fighting it: if you
        /// engaged from a script and the player also engaged from the window, setting this
        /// back to <c>false</c> withdraws only your request and leaves theirs standing.
        /// Use <see cref="DisengageAscent"/> for "stop, whoever asked".
        ///
        /// This is deliberately not a raw write to the module's Enabled property.
        /// MechJeb's own source says a module should be driven either entirely through
        /// Enabled or entirely through Users and that the two should not be mixed - and
        /// the pool sets Enabled itself, so nothing is lost by going through it.
        ///
        /// ORDER MATTERS. Set <see cref="Autostage"/> and the ascent settings BEFORE
        /// engaging: the autopilot reads Autostage as it is enabled, to decide whether to
        /// register with the staging controller at all. Flipping it afterwards leaves a
        /// window in which MechJeb may stage on its own.
        /// </summary>
        [KRPCProperty]
        public static bool AscentEnabled {
            get {
                MechJebApi.RequireMember (MechJebApi.EnabledProp, "AscentEnabled");
                var raw = MechJebApi.EnabledProp.GetValue (MechJebApi.AscentModule, null);
                return raw is bool && (bool) raw;
            }
            set {
                var module = MechJebApi.AscentModule;
                var pool = MechJebApi.PoolOf (module);
                if (pool != null && MechJebApi.UserPoolAdd != null && MechJebApi.UserPoolRemove != null) {
                    if (value) {
                        // UserPool derives from List<object> and permits duplicates, so
                        // without this check two writes of true would need two writes of
                        // false to undo - a property setter that is not idempotent.
                        var members = pool as ICollection;
                        var already = false;
                        if (members != null) {
                            foreach (var member in members)
                                if (ReferenceEquals (member, MechJebApi.AscentUser)) {
                                    already = true;
                                    break;
                                }
                        }
                        if (!already)
                            MechJebApi.UserPoolAdd.Invoke (pool, new[] { MechJebApi.AscentUser });
                    } else {
                        MechJebApi.UserPoolRemove.Invoke (pool, new[] { MechJebApi.AscentUser });
                    }
                    return;
                }
                // No reachable pool: fall back to the property. Same effect - its setter
                // is what fires the module's enable and disable hooks - just without the
                // shared ownership the pool gives.
                MechJebApi.RequireMember (MechJebApi.EnabledProp, "AscentEnabled");
                MechJebApi.EnabledProp.SetValue (module, value, null);
            }
        }

        /// <summary>
        /// Stop the ascent autopilot outright, whoever asked for it.
        ///
        /// Clears the user pool, which MechJeb defines as "disable the module". Use this
        /// on a campaign's cleanup path, where leaving an autopilot engaged because the
        /// player had also clicked Engage would carry into the next flight.
        /// <see cref="AscentEnabled"/> = false is the polite version: it withdraws only
        /// this bridge's request.
        /// </summary>
        [KRPCProcedure]
        public static void DisengageAscent ()
        {
            var module = MechJebApi.AscentModule;
            var pool = MechJebApi.PoolOf (module);
            if (pool != null && MechJebApi.UserPoolClear != null) {
                MechJebApi.UserPoolClear.Invoke (pool, null);
                return;
            }
            MechJebApi.RequireMember (MechJebApi.EnabledProp, "AscentEnabled");
            MechJebApi.EnabledProp.SetValue (module, false, null);
        }

        /// <summary>
        /// Which ascent path MechJeb uses. Settable, case-insensitive.
        ///
        /// Do not hard-code the value list. Current MechJeb declares "CLASSIC" and "PSG";
        /// older builds named the second one "PVG". Read this property, or write a wrong
        /// value on purpose - the exception carries the list the INSTALLED MechJeb offers,
        /// which is the only list that is ever right.
        ///
        /// A string rather than an enum on purpose. MechJeb's AscentType is a nested enum
        /// whose integer values changed when the gravity-turn path was removed in 2.15,
        /// and pinning them here would rot exactly the way the previous bridge did.
        /// Matching on the NAME is what survives. An unknown value throws with the list
        /// the installed MechJeb actually offers.
        /// </summary>
        [KRPCProperty]
        public static string AscentPath {
            get {
                MechJebApi.RequireMember (MechJebApi.AscentTypeProp, "AscentPath");
                var raw = MechJebApi.AscentTypeProp.GetValue (MechJebApi.Settings, null);
                return raw == null ? "UNKNOWN" : raw.ToString ().ToUpperInvariant ();
            }
            set {
                MechJebApi.RequireMember (MechJebApi.AscentTypeProp, "AscentPath");
                var enumType = MechJebApi.AscentTypeProp.PropertyType;
                object parsed = null;
                foreach (var name in Enum.GetNames (enumType)) {
                    if (string.Equals (name, value, StringComparison.OrdinalIgnoreCase)) {
                        parsed = Enum.Parse (enumType, name);
                        break;
                    }
                }
                if (parsed == null)
                    throw new ArgumentException (
                        "unknown ascent path '" + value + "' - this MechJeb offers: "
                        + string.Join (", ", Enum.GetNames (enumType)));
                MechJebApi.AscentTypeProp.SetValue (MechJebApi.Settings, parsed, null);
            }
        }

        /// <summary>
        /// Whether MechJeb stages the rocket by itself during the ascent.
        ///
        /// THE MEMBER THIS SERVICE EXISTS FOR. MechJeb decides to stage when the current
        /// stage has no active engines left, an engine counting as active while it is
        /// neither flamed out nor shut down. On a launcher where the side boosters and the
        /// core sit in the same stage, that stays false for as long as the core burns - so
        /// the side boosters are never dropped.
        ///
        /// Setting this to false removes the ascent autopilot from the staging
        /// controller's user pool, and ONLY that one: it stays registered with attitude
        /// and thrust. MechJeb keeps flying the rocket, and the staging decision comes back
        /// to the caller, who can use a criterion that works - a relative drop in total
        /// thrust, say - and stage through stock kRPC's
        /// <c>vessel.control.activate_next_stage()</c>.
        ///
        /// Set it BEFORE engaging the ascent. Check it took with
        /// <see cref="StagingUsers"/>.
        ///
        /// IT DOES NOT GO BACK. Setting it false works; setting it true again afterwards
        /// changes the flag and re-registers nothing, so MechJeb still will not stage.
        /// That is MechJeb's own defect rather than ours - its setter only re-registers
        /// `if (_autostage &amp;&amp; Enabled)`, where Enabled belongs to the settings module, and
        /// the only assignment to that field anywhere in MechJeb sets it FALSE. The Add
        /// branch is unreachable. To genuinely re-enable staging, cycle the ascent
        /// autopilot off and on, which runs the registration MechJeb does reach.
        ///
        /// SOLID BOOSTERS ARE A SEPARATE CASE. Everything above is about liquid side
        /// boosters. MechJeb's staging controller has a DropSolids mode whose guard skips
        /// the active-engine test for a throttle-locked engine, so with solids it will drop
        /// them while the core burns and taking the decision over is unnecessary. That
        /// setting is not reachable from this service today.
        /// </summary>
        [KRPCProperty]
        public static bool Autostage {
            get {
                MechJebApi.RequireMember (MechJebApi.AutostageProp, "Autostage");
                var raw = MechJebApi.AutostageProp.GetValue (MechJebApi.Settings, null);
                return raw is bool && (bool) raw;
            }
            set {
                MechJebApi.RequireMember (MechJebApi.AutostageProp, "Autostage");
                MechJebApi.AutostageProp.SetValue (MechJebApi.Settings, value, null);
            }
        }

        // ==================================================================
        // Settings, by name
        // ==================================================================

        /// <summary>
        /// Every numeric ascent setting the INSTALLED MechJeb exposes, as "Name : Type" -
        /// the argument list for <see cref="SetAscentSetting"/>.
        ///
        /// Read this rather than trusting a name from documentation. It is generated from
        /// the live assembly, so it is right for the build actually in GameData, and it
        /// tells you a knob has been renamed without waiting for a failure.
        ///
        /// Computed once per session and cached, so putting a stream on it is harmless.
        /// </summary>
        [KRPCProperty]
        public static IList<string> AscentSettingNames {
            get {
                if (cachedSettingNames != null)
                    return cachedSettingNames;
                MechJebApi.Require ();
                MechJebApi.RequireMember (MechJebApi.SettingsType, "AscentSettings");
                var names = new List<string> ();
                foreach (var field in MechJebApi.SettingsType.GetFields (ModRegistry.PubInst))
                    if (IsNumericLike (field.FieldType) && IsPublicApiName (field.Name))
                        names.Add (field.Name + " : " + field.FieldType.Name);
                foreach (var property in MechJebApi.SettingsType.GetProperties (ModRegistry.PubInst))
                    if (IsNumericLike (property.PropertyType) && property.CanWrite
                        && IsPublicApiName (property.Name))
                        names.Add (property.Name + " : " + property.PropertyType.Name);
                names.Sort ();
                cachedSettingNames = names;
                return names;
            }
        }

        /// <summary>
        /// Every boolean ascent setting the installed MechJeb exposes - the argument list
        /// for <see cref="SetAscentFlag"/>. Cached like <see cref="AscentSettingNames"/>.
        /// </summary>
        [KRPCProperty]
        public static IList<string> AscentFlagNames {
            get {
                if (cachedFlagNames != null)
                    return cachedFlagNames;
                MechJebApi.Require ();
                MechJebApi.RequireMember (MechJebApi.SettingsType, "AscentSettings");
                var names = new List<string> ();
                foreach (var field in MechJebApi.SettingsType.GetFields (ModRegistry.PubInst))
                    if (field.FieldType == typeof (bool) && IsPublicApiName (field.Name))
                        names.Add (field.Name);
                foreach (var property in MechJebApi.SettingsType.GetProperties (ModRegistry.PubInst))
                    if (property.PropertyType == typeof (bool) && property.CanWrite
                        && IsPublicApiName (property.Name))
                        names.Add (property.Name);
                names.Sort ();
                cachedFlagNames = names;
                return names;
            }
        }

        /// <summary>
        /// Read a numeric ascent setting by name, for example "DesiredOrbitAltitude".
        ///
        /// Case-insensitive. Handles MechJeb's Editable wrappers transparently: those are
        /// readonly fields holding an object with a Val property, which is why you cannot
        /// simply assign them and why the previous bridge had to wrap each one by hand.
        /// </summary>
        /// <param name="name">Setting name, from <see cref="AscentSettingNames"/>.</param>
        [KRPCProcedure]
        public static double AscentSetting (string name)
        {
            var holder = FindSettingMember (name, true);
            var raw = ReadMember (holder, MechJebApi.Settings);
            var editable = ValProperty (raw);
            if (editable != null)
                raw = editable.GetValue (raw, null);
            if (raw == null)
                throw new InvalidOperationException ("ascent setting '" + name + "' read as null");
            return Convert.ToDouble (raw);
        }

        /// <summary>
        /// Write a numeric ascent setting by name.
        ///
        /// This is the escape hatch that keeps the service alive across MechJeb releases:
        /// a renamed knob costs a string in your script, not a rebuilt DLL.
        ///
        /// Units are MechJeb's own - metres, m/s, degrees - and note that its
        /// EditableDoubleMult knobs, altitudes mostly, store the SI value: an altitude of
        /// 100 km is 100000, not 100.
        /// </summary>
        /// <param name="name">Setting name, from <see cref="AscentSettingNames"/>.</param>
        /// <param name="value">New value, in MechJeb's units.</param>
        [KRPCProcedure]
        public static void SetAscentSetting (string name, double value)
        {
            var holder = FindSettingMember (name, true);
            var settings = MechJebApi.Settings;
            var current = ReadMember (holder, settings);

            var editable = ValProperty (current);
            if (editable != null) {
                // Editable wrappers are readonly fields: the object stays, its Val
                // changes. This is exactly what MechJeb does internally.
                editable.SetValue (current, Convert.ChangeType (value, editable.PropertyType), null);
                return;
            }

            var field = holder as FieldInfo;
            if (field != null) {
                if (field.IsInitOnly)
                    throw new InvalidOperationException (
                        "'" + name + "' is a readonly field with no Val property - it cannot be set");
                field.SetValue (settings, Convert.ChangeType (value, field.FieldType));
                return;
            }
            var property = (PropertyInfo) holder;
            if (!property.CanWrite)
                throw new InvalidOperationException ("'" + name + "' is read-only in this MechJeb");
            property.SetValue (settings, Convert.ChangeType (value, property.PropertyType), null);
        }

        /// <summary>Read a boolean ascent setting by name, for example "LimitAoA".</summary>
        /// <param name="name">Flag name, from <see cref="AscentFlagNames"/>.</param>
        [KRPCProcedure]
        public static bool AscentFlag (string name)
        {
            var holder = FindSettingMember (name, false);
            var raw = ReadMember (holder, MechJebApi.Settings);
            return raw is bool && (bool) raw;
        }

        /// <summary>Write a boolean ascent setting by name, for example "SkipCircularization".</summary>
        /// <param name="name">Flag name, from <see cref="AscentFlagNames"/>.</param>
        /// <param name="value">New value.</param>
        [KRPCProcedure]
        public static void SetAscentFlag (string name, bool value)
        {
            var holder = FindSettingMember (name, false);
            var settings = MechJebApi.Settings;
            var field = holder as FieldInfo;
            if (field != null) {
                field.SetValue (settings, value);
                return;
            }
            var property = (PropertyInfo) holder;
            if (!property.CanWrite)
                throw new InvalidOperationException ("'" + name + "' is read-only in this MechJeb");
            property.SetValue (settings, value, null);
        }

        // ==================================================================
        // Staging
        // ==================================================================

        /// <summary>
        /// How many users the staging controller currently has.
        ///
        /// Zero means nothing is asking MechJeb to stage, which is the state
        /// <see cref="Autostage"/> = false is meant to produce - so this is how you CHECK
        /// that it took, rather than assuming. Returns -1 if the pool could not be read at
        /// all, which is a different failure from "the pool is empty".
        /// </summary>
        [KRPCProperty]
        public static int StagingUsers {
            get {
                MechJebApi.Require ();
                MechJebApi.RequireMember (MechJebApi.StagingField, "Staging");
                var staging = ModRegistry.Field (MechJebApi.StagingField, MechJebApi.Core);
                var pool = MechJebApi.PoolOf (staging) as ICollection;
                return pool == null ? -1 : pool.Count;
            }
        }

        /// <summary>
        /// Take the ascent autopilot out of the staging controller's user pool by hand,
        /// and report what happened.
        ///
        /// A fallback for a MechJeb build where the Autostage property has moved or been
        /// renamed: it does what that property's setter does, but reaches the pool
        /// directly. Prefer <see cref="Autostage"/>. This exists so that a rename cannot
        /// leave you with no way to stop MechJeb from staging.
        /// </summary>
        [KRPCProcedure]
        public static string ReleaseStaging ()
        {
            MechJebApi.Require ();
            MechJebApi.RequireMember (MechJebApi.StagingField, "Staging");
            MechJebApi.RequireMember (MechJebApi.UserPoolRemove, "UserPool.Remove");

            var core = MechJebApi.Core;
            var staging = ModRegistry.Field (MechJebApi.StagingField, core);
            if (staging == null)
                return "MechJebCore.Staging is null - nothing to release";

            var module = MechJebApi.AscentProp.GetValue (core, null);
            if (module == null)
                return "no ascent autopilot instance - nothing to release";

            var pool = MechJebApi.PoolOf (staging);
            if (pool == null)
                return "the staging controller exposes no user pool - nothing to release";

            MechJebApi.UserPoolRemove.Invoke (pool, new[] { module });
            return "ascent autopilot removed from the staging controller's user pool";
        }

        /// <summary>
        /// Names of the public members the bridge found on MechJebCore, for when a future
        /// MechJeb renames something and <see cref="Diagnostics"/> says a lookup failed.
        /// Computed once and cached.
        /// </summary>
        [KRPCProperty]
        public static IList<string> CoreMembers {
            get {
                if (cachedCoreMembers != null)
                    return cachedCoreMembers;
                MechJebApi.Require ();
                var names = new List<string> ();
                foreach (var field in MechJebApi.CoreType.GetFields (ModRegistry.PubInst))
                    names.Add ("field " + field.Name + " : " + field.FieldType.Name);
                foreach (var property in MechJebApi.CoreType.GetProperties (ModRegistry.PubInst))
                    names.Add ("prop  " + property.Name + " : " + property.PropertyType.Name);
                names.Sort ();
                cachedCoreMembers = names;
                return names;
            }
        }

        // ==================================================================
        // Plumbing
        // ==================================================================

        /// <summary>
        /// Find a settings member by name, and check it is the KIND the caller asked for.
        ///
        /// The type check is not pedantry. Without it, ascent_flag("DesiredOrbitAltitude")
        /// reads a double, fails the <c>raw is bool</c> test, and returns false - which is
        /// indistinguishable from a flag that is legitimately off. A wrong answer that
        /// looks like a right one is the worst result this service can produce.
        /// </summary>
        static MemberInfo FindSettingMember (string name, bool numeric)
        {
            MechJebApi.Require ();
            MechJebApi.RequireMember (MechJebApi.SettingsType, "AscentSettings");
            if (string.IsNullOrEmpty (name))
                throw new ArgumentException ("setting name is empty");

            MemberInfo found = null;
            Type type = null;

            foreach (var field in MechJebApi.SettingsType.GetFields (ModRegistry.PubInst)) {
                if (!string.Equals (field.Name, name, StringComparison.OrdinalIgnoreCase))
                    continue;
                found = field;
                type = field.FieldType;
                break;
            }
            if (found == null) {
                foreach (var property in MechJebApi.SettingsType.GetProperties (ModRegistry.PubInst)) {
                    if (!string.Equals (property.Name, name, StringComparison.OrdinalIgnoreCase))
                        continue;
                    found = property;
                    type = property.PropertyType;
                    break;
                }
            }

            if (found == null)
                throw new ArgumentException (
                    "no ascent setting called '" + name + "' in this MechJeb - read "
                    + (numeric ? "ascent_setting_names" : "ascent_flag_names") + " for what it has");

            if (numeric && !IsNumericLike (type))
                throw new ArgumentException (
                    "'" + found.Name + "' is a " + type.Name + ", not a number - use "
                    + (type == typeof (bool) ? "ascent_flag" : "a different accessor"));
            if (!numeric && type != typeof (bool))
                throw new ArgumentException (
                    "'" + found.Name + "' is a " + type.Name + ", not a flag - use "
                    + (IsNumericLike (type) ? "ascent_setting" : "a different accessor"));

            return found;
        }

        static object ReadMember (MemberInfo member, object target)
        {
            var field = member as FieldInfo;
            return field != null ? field.GetValue (target)
                                 : ((PropertyInfo) member).GetValue (target, null);
        }

        /// <summary>
        /// The Val property of a MechJeb Editable wrapper, or null if this is a plain
        /// value. Detected by SHAPE, not by type name: EditableDouble,
        /// EditableDoubleMult, EditableInt and friends all share it, and a future
        /// EditableWhatever would too.
        /// </summary>
        static PropertyInfo ValProperty (object candidate)
        {
            if (candidate == null || candidate is double || candidate is float
                || candidate is int || candidate is bool || candidate is string)
                return null;
            var type = candidate.GetType ();
            var val = type.GetProperty ("Val", ModRegistry.PubInst)
                      ?? type.GetProperty ("val", ModRegistry.PubInst);
            return val != null && val.CanWrite && IsNumericLike (val.PropertyType) ? val : null;
        }

        /// <summary>
        /// Whether a reflected member name should be offered to scripts at all.
        ///
        /// MechJeb declares the backing fields of several settings as PUBLIC, right next to
        /// the property that wraps them - `public bool _autostage` sits beside `Autostage`.
        /// Both turn up in a reflection walk and only one of them works. Writing the field
        /// sets the value and skips the property's side effect, which for Autostage is the
        /// staging-pool registration - so `set_ascent_flag("_autostage", False)` would
        /// change the flag and leave MechJeb still staging, the exact silent failure this
        /// service exists to prevent. Two names, one real, one that looks identical.
        ///
        /// The convention is reliable enough to filter on: a leading underscore in this
        /// codebase means "internal, use the property".
        /// </summary>
        static bool IsPublicApiName (string name)
        {
            return !string.IsNullOrEmpty (name) && name [0] != '_';
        }

        static bool IsNumericLike (Type type)
        {
            if (type == typeof (double) || type == typeof (float)
                || type == typeof (int) || type == typeof (long))
                return true;
            // An Editable wrapper: not numeric itself, but it holds a numeric Val.
            var val = type.GetProperty ("Val", ModRegistry.PubInst)
                      ?? type.GetProperty ("val", ModRegistry.PubInst);
            return val != null && (val.PropertyType == typeof (double) || val.PropertyType == typeof (float)
                                   || val.PropertyType == typeof (int) || val.PropertyType == typeof (long));
        }
    }
}
