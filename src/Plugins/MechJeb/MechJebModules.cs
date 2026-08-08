using System;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.MechJeb
{
    /// <summary>
    /// Finding a MechJeb module by name, and turning it on and off correctly.
    ///
    /// MechJebCore publishes twenty-one modules as short public members - Staging, Thrust,
    /// Target, Hoverslam and the rest - and carries a lookup, GetComputerModule(string),
    /// that resolves any of the other forty by type name. Between them that is the whole
    /// mod, reachable without this file ever naming a MechJebModule type: the twenty-one
    /// are found by walking MechJebCore for members whose type descends from
    /// ComputerModule, and ComputerModule itself is identified as whatever type declares
    /// the Enabled property we already resolved.
    ///
    /// ENGAGING IS NOT UNIFORM, and that is the awkward part. For most modules the lever
    /// is the user pool: add a token, the module enables; remove it, and it disables once
    /// the last user leaves. But some modules pin themselves and must never be released,
    /// some are settings bags that are never enabled at all, and three do nothing useful
    /// when merely enabled because their real entry point is a method. None of that is
    /// visible in the type - it is behaviour - so it lives in the table below. That table
    /// is the only place in this plugin where MechJeb member names are hard-coded, and it
    /// is short on purpose: getting it wrong means a script believes it engaged something
    /// and nothing happened, which is precisely the failure this service exists to prevent.
    /// </summary>
    internal static class MechJebModules
    {
        /// <summary>How a module is turned on.</summary>
        internal enum Engagement
        {
            /// <summary>The user pool is the lever. The common case.</summary>
            Pool,
            /// <summary>Runs itself and must keep running. Engage is a no-op, disengage is refused.</summary>
            AlwaysOn,
            /// <summary>A settings bag with no autopilot behind it. Engaging is meaningless.</summary>
            NotEngageable,
            /// <summary>Enabling alone does nothing; there is a method that must be called.</summary>
            NeedsMethod
        }

        /// <summary>
        /// The behaviour that cannot be read off a type. Keyed on the MechJebCore member
        /// name, which has been stable across every refactor so far - unlike the class
        /// names, which have changed twice.
        /// </summary>
        static readonly Dictionary<string, Engagement> Behaviour =
            new Dictionary<string, Engagement> (StringComparer.OrdinalIgnoreCase) {
                // Settings bags. Nothing in MechJeb ever sets AscentSettings.Enabled true;
                // the ascent autopilot behind it is reached as "Ascent".
                { "AscentSettings", Engagement.NotEngageable },
                { "Settings",       Engagement.NotEngageable },

                // Self-pinned. Releasing these does real damage: Thrust holds the throttle
                // limiters, Target is what every other module steers at, StageStats is the
                // delta-v simulation the rest of MechJeb reads, and Hoverslam is the
                // landing predictor. Disengaging them silences things nobody asked to lose.
                { "Thrust",         Engagement.AlwaysOn },
                { "Target",         Engagement.AlwaysOn },
                { "Warp",           Engagement.AlwaysOn },
                { "StageStats",     Engagement.AlwaysOn },
                { "Solarpanel",     Engagement.AlwaysOn },
                { "AntennaControl", Engagement.AlwaysOn },
                { "Hoverslam",      Engagement.AlwaysOn },

                // Enabling sets a flag and leaves the module with no work to do. Landing
                // needs a step set, Node needs a node chosen, SmartASS needs its target
                // pushed to the attitude controller.
                { "Landing",        Engagement.NeedsMethod },
                { "Node",           Engagement.NeedsMethod },
                { "SmartASS",       Engagement.NeedsMethod }
            };

        static Type computerModuleType;
        static MethodInfo getComputerModule;      // MechJebCore.GetComputerModule(string)
        static List<string> cachedNames;

        internal static string Report { get; private set; } = "not resolved";

        /// <summary>
        /// Work out what ComputerModule is, structurally, and bind the by-type-name lookup.
        /// Called from MechJebApi.Resolve once the ascent module's type is known.
        /// </summary>
        internal static void Resolve (Type coreType, PropertyInfo enabledProp)
        {
            computerModuleType = enabledProp != null ? enabledProp.DeclaringType : null;
            getComputerModule = null;
            cachedNames = null;

            if (coreType != null) {
                try {
                    getComputerModule = coreType.GetMethod ("GetComputerModule", ModRegistry.PubInst,
                                                            null, new[] { typeof (string) }, null);
                } catch (Exception) {
                    getComputerModule = null;
                }
            }

            Report = string.Format ("ComputerModule={0} GetComputerModule={1}",
                                    computerModuleType != null ? computerModuleType.Name : "?",
                                    getComputerModule != null);
        }

        /// <summary>
        /// Every module MechJebCore publishes under a short name, in alphabetical order.
        ///
        /// Not a hard-coded list: whatever the live core has. A MechJeb that adds a module
        /// gains it here with no rebuild, and one that removes a module loses it cleanly
        /// rather than throwing on a name that no longer resolves.
        /// </summary>
        internal static IList<string> Names (object core)
        {
            if (cachedNames != null)
                return cachedNames;
            var names = new List<string> ();
            if (core == null || computerModuleType == null)
                return names;

            var type = core.GetType ();
            foreach (var field in type.GetFields (ModRegistry.PubInst))
                if (computerModuleType.IsAssignableFrom (field.FieldType))
                    names.Add (field.Name);
            foreach (var property in type.GetProperties (ModRegistry.PubInst))
                if (property.CanRead && property.GetIndexParameters ().Length == 0
                    && computerModuleType.IsAssignableFrom (property.PropertyType))
                    names.Add (property.Name);

            names.Sort (StringComparer.Ordinal);
            cachedNames = names;
            return names;
        }

        /// <summary>
        /// Resolve a module: first the short names MechJebCore publishes, then MechJeb's
        /// own by-type-name lookup.
        ///
        /// The fallback is worth more than the twenty-one put together, because it is the
        /// only way to the maneuver planner, the rendezvous autopilot and the docking
        /// autopilot - none of which MechJebCore holds a field for.
        /// </summary>
        internal static object Get (string name)
        {
            if (string.IsNullOrEmpty (name))
                throw new ArgumentException ("module name is empty");
            var core = MechJebApi.Core;
            var type = core.GetType ();

            var field = MechJebApi.FindField (type, name);
            if (field != null && IsModule (field.FieldType))
                return Instance (field.GetValue (core), name);

            var property = MechJebApi.FindProperty (type, name);
            if (property != null && property.CanRead && IsModule (property.PropertyType))
                return Instance (property.GetValue (core, null), name);

            // Case-insensitive over the published names, before falling through.
            foreach (var published in Names (core)) {
                if (!string.Equals (published, name, StringComparison.OrdinalIgnoreCase))
                    continue;
                var byField = MechJebApi.FindField (type, published);
                if (byField != null)
                    return Instance (byField.GetValue (core), published);
                var byProperty = MechJebApi.FindProperty (type, published);
                if (byProperty != null)
                    return Instance (byProperty.GetValue (core, null), published);
            }

            if (getComputerModule != null) {
                object found = null;
                try {
                    found = getComputerModule.Invoke (core, new object[] { name });
                } catch (Exception e) {
                    throw new InvalidOperationException (
                        "MechJeb's own module lookup threw for '" + name + "': " + Inner (e).Message);
                }
                if (found != null)
                    return found;
            }

            throw new ArgumentException (
                "MechJeb has no module called '" + name + "' - modules() lists the ones it "
                + "publishes by short name, and any other module can be asked for by its "
                + "class name, for example 'MechJebModuleManeuverPlanner'");
        }

        static bool IsModule (Type type)
        {
            return computerModuleType != null && computerModuleType.IsAssignableFrom (type);
        }

        static object Instance (object module, string name)
        {
            if (module == null)
                throw new InvalidOperationException (
                    "MechJeb." + name + " is null on this vessel - the module exists but has "
                    + "no instance, which usually means the core has not finished starting");
            return module;
        }

        internal static Engagement BehaviourOf (string name)
        {
            Engagement how;
            return Behaviour.TryGetValue (name, out how) ? how : Engagement.Pool;
        }

        // ==================================================================
        // Turning modules on and off
        // ==================================================================

        /// <summary>
        /// Add our token to a module's user pool, which is what enables it.
        ///
        /// Idempotent by MechJeb's own guard, and deliberately reported rather than
        /// asserted: in a career save an engage can be silently reverted a frame later by
        /// MechJeb's part-and-tech unlock check, so the caller is told the resulting user
        /// count and can look at it.
        /// </summary>
        internal static int Engage (string name, object module)
        {
            var how = BehaviourOf (name);
            if (how == Engagement.NotEngageable)
                throw new InvalidOperationException (
                    name + " is a settings module, not an autopilot - there is nothing to engage. "
                    + (string.Equals (name, "AscentSettings", StringComparison.OrdinalIgnoreCase)
                       ? "The autopilot behind it is 'Ascent'." : string.Empty));

            if (how == Engagement.AlwaysOn)
                return UserCount (module);   // already running; adding a token would be noise

            MechJebApi.RequireMember (MechJebApi.UserPoolAdd, "UserPool.Add");
            var pool = MechJebApi.PoolOf (module);
            if (pool == null)
                throw new InvalidOperationException (name + " has no user pool");
            MechJebApi.UserPoolAdd.Invoke (pool, new object[] { MechJebApi.AscentUser });
            return UserCount (module);
        }

        /// <summary>
        /// Withdraw our token. The module keeps running if anything else still wants it -
        /// MechJeb's own window, or another autopilot - which is correct and is why the
        /// remaining user count comes back rather than a bare success.
        /// </summary>
        internal static int Disengage (string name, object module)
        {
            var how = BehaviourOf (name);
            if (how == Engagement.AlwaysOn)
                throw new InvalidOperationException (
                    name + " runs itself and other parts of MechJeb depend on it - disengaging "
                    + "it would silence them. Refused deliberately.");
            if (how == Engagement.NotEngageable)
                throw new InvalidOperationException (name + " is a settings module; it was never engaged");

            MechJebApi.RequireMember (MechJebApi.UserPoolRemove, "UserPool.Remove");
            var pool = MechJebApi.PoolOf (module);
            if (pool == null)
                throw new InvalidOperationException (name + " has no user pool");
            MechJebApi.UserPoolRemove.Invoke (pool, new object[] { MechJebApi.AscentUser });
            return UserCount (module);
        }

        internal static int UserCount (object module)
        {
            var pool = MechJebApi.PoolOf (module);
            if (pool == null)
                return -1;
            var count = MechJebApi.FindProperty (pool.GetType (), "Count");
            if (count == null)
                return -1;
            try {
                return Convert.ToInt32 (count.GetValue (pool, null));
            } catch (Exception) {
                return -1;
            }
        }

        internal static bool EnabledOf (object module)
        {
            if (MechJebApi.EnabledProp == null || module == null)
                return false;
            try {
                return Convert.ToBoolean (MechJebApi.EnabledProp.GetValue (module, null));
            } catch (Exception) {
                return false;
            }
        }

        /// <summary>
        /// Call a plain method on a module by name, for the three whose real entry point is
        /// a method rather than the pool. Arguments are matched by count only, because the
        /// ones we need take either nothing or a single user token.
        /// </summary>
        internal static void Call (object module, string method, bool passUserToken)
        {
            var found = passUserToken
                ? module.GetType ().GetMethod (method, ModRegistry.PubInst, null, new[] { typeof (object) }, null)
                : module.GetType ().GetMethod (method, ModRegistry.PubInst, null, Type.EmptyTypes, null);
            if (found == null)
                throw new InvalidOperationException (
                    "this MechJeb has no " + method + "(" + (passUserToken ? "user" : string.Empty)
                    + ") on " + module.GetType ().Name);
            try {
                found.Invoke (module, passUserToken ? new[] { MechJebApi.AscentUser } : null);
            } catch (Exception e) {
                throw new InvalidOperationException (method + " threw: " + Inner (e).Message);
            }
        }

        internal static Exception Inner (Exception e)
        {
            var reflection = e as TargetInvocationException;
            return reflection != null && reflection.InnerException != null ? reflection.InnerException : e;
        }
    }
}
