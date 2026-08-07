using System;
using System.Collections.Generic;
using System.Reflection;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// What one plugin reports about itself and the mod it wraps.
    ///
    /// THE SETTERS ARE PUBLIC, AND MUST STAY THAT WAY. This type crosses an assembly
    /// boundary by design: every plugin builds one of these in its own DLL and hands it
    /// back to the Core. <c>internal set</c> looks tidier and compiles fine inside Core,
    /// but no plugin can then fill one in - CS0200, "cannot assign, it is read only".
    ///
    /// Name and Version are filled in by <see cref="ModRegistry.ResolveAll"/> after the
    /// resolver returns, so a plugin can leave them alone.
    /// </summary>
    public sealed class PluginStatus
    {
        /// <summary>Plugin name, as registered. Set by the registry, not by the plugin.</summary>
        public string Name { get; set; }

        /// <summary>Version of the plugin assembly. Set by the registry, not by the plugin.</summary>
        public string Version { get; set; }

        /// <summary>True when the target mod was found and every member resolved.</summary>
        public bool Available { get; set; }

        /// <summary>Version string of the wrapped mod, or empty.</summary>
        public string ModVersion { get; set; }

        /// <summary>Why it is unavailable, or what resolved. Written for a human reading KSP.log.</summary>
        public string Report { get; set; }
    }

    /// <summary>
    /// The single place a plugin announces itself, and the single place Python asks
    /// "what is actually loaded".
    ///
    /// Core deliberately knows nothing about any specific mod. A plugin calls
    /// <see cref="Register"/> from its own KSPAddon, Core calls the resolver once at the
    /// first scene load and caches the result. Reflection results only depend on which
    /// assemblies are in the AppDomain, never on the current scene, so once is enough.
    /// </summary>
    public static class ModRegistry
    {
        sealed class Registration
        {
            internal string Name;
            internal string Version;
            internal Func<PluginStatus> Resolve;
            internal PluginStatus Cached;
        }

        static readonly List<Registration> registrations = new List<Registration> ();
        static readonly object gate = new object ();
        static bool resolved;

        /// <summary>
        /// Announce a plugin. Call this from the plugin's own
        /// [KSPAddon(Startup.Instantly)] so it lands before Core resolves.
        /// </summary>
        /// <param name="name">Short name, matching the kRPC service name.</param>
        /// <param name="resolver">
        /// Does the reflection and returns the outcome. Must not throw: catch inside and
        /// report Available=false with a Report explaining why.
        /// </param>
        public static void Register (string name, Func<PluginStatus> resolver)
        {
            if (string.IsNullOrEmpty (name) || resolver == null)
                return;
            lock (gate) {
                registrations.Add (new Registration {
                    Name = name,
                    Version = Assembly.GetCallingAssembly ().GetName ().Version.ToString (),
                    Resolve = resolver
                });
            }
        }

        /// <summary>
        /// Run every registered resolver once.
        ///
        /// Called by <see cref="BridgeCore"/> from Start - not Awake, because Core's
        /// Awake runs BEFORE any plugin's, KSP having loaded Core first to satisfy the
        /// dependency every plugin declares on it. <see cref="All"/> also calls this, so
        /// a read that somehow arrives first still gets resolved data rather than an
        /// empty list.
        /// </summary>
        public static void ResolveAll ()
        {
            List<Registration> pending;
            lock (gate) {
                if (resolved)
                    return;
                resolved = true;
                pending = new List<Registration> (registrations);
            }

            // Resolvers run OUTSIDE the lock. They do arbitrary third-party reflection -
            // Assembly.GetTypes, FindObjectOfType, whatever a future plugin needs - and
            // holding our own lock across a call into another subsystem is how lock
            // cycles get built.
            foreach (var registration in pending) {
                PluginStatus status;
                try {
                    status = registration.Resolve () ?? new PluginStatus {
                        Available = false, Report = "resolver returned null"
                    };
                } catch (Exception e) {
                    // One plugin's bad reflection must never stop the others.
                    status = new PluginStatus { Available = false, Report = "resolver threw: " + e.Message };
                }
                status.Name = registration.Name;
                status.Version = registration.Version;
                registration.Cached = status;
                BridgeLog.Info (registration.Name,
                                (status.Available ? "available" : "unavailable") +
                                (string.IsNullOrEmpty (status.ModVersion) ? "" : " (" + status.ModVersion + ")") +
                                " - " + status.Report);
            }
        }

        /// <summary>Status of every registered plugin. This is what conn.bridge.plugins returns.</summary>
        public static IList<PluginStatus> All ()
        {
            // Resolve on demand as well as from BridgeCore.Start, so the answer can never
            // be "nothing is registered" merely because a client asked early.
            ResolveAll ();
            lock (gate) {
                var found = new List<PluginStatus> ();
                foreach (var registration in registrations)
                    if (registration.Cached != null)
                        found.Add (registration.Cached);
                return found;
            }
        }

        // ------------------------------------------------------------------------
        // Reflection helpers. Shared so no plugin reimplements them, and so the fast
        // paths below exist in one place.
        // ------------------------------------------------------------------------

        /// <summary>Binding flags for public instance members.</summary>
        public const BindingFlags PubInst = BindingFlags.Public | BindingFlags.Instance;
        /// <summary>Binding flags for public static members.</summary>
        public const BindingFlags PubStatic = BindingFlags.Public | BindingFlags.Static;
        /// <summary>Binding flags for any instance member, public or not.</summary>
        public const BindingFlags AnyInst = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
        /// <summary>Binding flags for any static member, public or not.</summary>
        public const BindingFlags AnyStatic = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Static;

        /// <summary>Find a loaded assembly by simple name, trying each candidate in order.</summary>
        public static Assembly FindAssembly (params string[] candidates)
        {
            foreach (var wanted in candidates) {
                foreach (var loaded in AssemblyLoader.loadedAssemblies) {
                    if (loaded == null || loaded.assembly == null)
                        continue;
                    if (string.Equals (loaded.assembly.GetName ().Name, wanted,
                                       StringComparison.OrdinalIgnoreCase))
                        return loaded.assembly;
                }
            }
            return null;
        }

        /// <summary>Every loaded assembly whose simple name contains the fragment, shortest name first.</summary>
        public static List<Assembly> FindAssembliesContaining (string fragment)
        {
            var found = new List<Assembly> ();
            foreach (var loaded in AssemblyLoader.loadedAssemblies) {
                if (loaded == null || loaded.assembly == null)
                    continue;
                var name = loaded.assembly.GetName ().Name;
                if (name != null && name.IndexOf (fragment, StringComparison.OrdinalIgnoreCase) >= 0)
                    found.Add (loaded.assembly);
            }
            found.Sort ((a, b) => a.GetName ().Name.Length.CompareTo (b.GetName ().Name.Length));
            return found;
        }

        /// <summary>All types of an assembly, tolerating a partially broken one.</summary>
        public static IEnumerable<Type> SafeTypes (Assembly assembly)
        {
            if (assembly == null)
                return new Type[0];
            try {
                return assembly.GetTypes ();
            } catch (ReflectionTypeLoadException e) {
                var found = new List<Type> ();
                foreach (var t in e.Types)
                    if (t != null)
                        found.Add (t);
                return found;
            } catch (Exception) {
                return new Type[0];
            }
        }

        /// <summary>Find a type by full name, then by simple name, so the namespace need not be pinned.</summary>
        public static Type FindType (Assembly assembly, string fullOrSimpleName)
        {
            if (assembly == null || string.IsNullOrEmpty (fullOrSimpleName))
                return null;
            var direct = assembly.GetType (fullOrSimpleName, false);
            if (direct != null)
                return direct;
            foreach (var t in SafeTypes (assembly))
                if (t.FullName == fullOrSimpleName || t.Name == fullOrSimpleName)
                    return t;
            return null;
        }

        /// <summary>Informational version of an assembly, for the status report.</summary>
        public static string VersionOf (Assembly assembly)
        {
            return assembly == null ? string.Empty : assembly.GetName ().Version.ToString ();
        }

        /// <summary>Simple names of a set of assemblies, comma separated. For log lines and diagnostics.</summary>
        public static string NamesOf (IEnumerable<Assembly> assemblies)
        {
            var names = new List<string> ();
            if (assemblies != null)
                foreach (var assembly in assemblies)
                    if (assembly != null)
                        names.Add (assembly.GetName ().Name);
            return names.Count == 0 ? "(none)" : string.Join (", ", names.ToArray ());
        }

        /// <summary>
        /// The live MonoBehaviour of the given type in the current scene, or null.
        ///
        /// Always call this per use rather than caching. A KSPAddon MonoBehaviour is
        /// destroyed on every scene change, and a cached reference to one is a destroyed
        /// Unity object that only Unity's fake-null would catch.
        /// </summary>
        public static UnityEngine.Object FindLive (Type type)
        {
            return type == null ? null : UnityEngine.Object.FindObjectOfType (type);
        }

        // Concrete subclasses of a base type, worked out once. Keyed by the base.
        static readonly Dictionary<Type, Type[]> derivedTypes = new Dictionary<Type, Type[]> ();

        /// <summary>
        /// The live MonoBehaviour of <paramref name="type"/> or of any type derived from
        /// it, whichever this scene has.
        ///
        /// The difference matters when a mod splits one logical singleton across several
        /// per-scene KSPAddon subclasses of a common base - FMRS does exactly this, with
        /// FMRS, FMRS_Space_Center, FMRS_TrackingStation and FMRS_Main_Menu all deriving
        /// from FMRS_Core, which is where the state actually lives. Looking only for the
        /// flight subclass makes the whole service dead outside flight for no reason.
        ///
        /// THE SUBCLASS LIST IS CACHED, and that is not a micro-optimisation. Every FMRS
        /// property getter reaches the live object through here, a kRPC client can put a
        /// stream on any of them, and a streamed getter runs every physics tick forever.
        /// Walking Assembly.GetTypes() on each of those calls - which allocates the whole
        /// type array - would turn one stream into steady GC pressure, and GC pressure in
        /// FixedUpdate is a stutter. The set of types in a loaded assembly cannot change
        /// during a session, so computing it once is not merely faster, it is exact.
        /// </summary>
        public static UnityEngine.Object FindLiveAssignable (Type type)
        {
            if (type == null)
                return null;

            // FindObjectOfType already matches derived components, so this alone usually
            // answers. The list below is the fallback for a Unity build that does not.
            var exact = UnityEngine.Object.FindObjectOfType (type);
            if (exact != null)
                return exact;

            foreach (var candidate in DerivedFrom (type)) {
                var live = UnityEngine.Object.FindObjectOfType (candidate);
                if (live != null)
                    return live;
            }
            return null;
        }

        /// <summary>Concrete types in the same assembly that derive from this one. Computed once.</summary>
        public static Type[] DerivedFrom (Type type)
        {
            if (type == null)
                return new Type[0];
            lock (gate) {
                Type[] cached;
                if (derivedTypes.TryGetValue (type, out cached))
                    return cached;

                var found = new List<Type> ();
                foreach (var candidate in SafeTypes (type.Assembly)) {
                    if (candidate == type || candidate.IsAbstract || !type.IsAssignableFrom (candidate))
                        continue;
                    found.Add (candidate);
                }
                cached = found.ToArray ();
                derivedTypes [type] = cached;
                return cached;
            }
        }

        /// <summary>Read a field, returning null on any failure rather than throwing.</summary>
        public static object Field (FieldInfo field, object target)
        {
            if (field == null)
                return null;
            if (target == null && !field.IsStatic)
                return null;
            try {
                return field.GetValue (target);
            } catch (Exception) {
                return null;
            }
        }

        /// <summary>
        /// Comma-separated substring filter, case-insensitive, whitespace trimmed.
        /// An empty filter or "*" matches everything.
        ///
        /// One filter syntax for the whole mod: camera names, event kinds, plugin names.
        /// </summary>
        public static bool MatchesFilter (string candidate, string filter)
        {
            if (string.IsNullOrEmpty (filter) || filter.Trim () == "*")
                return true;
            if (candidate == null)
                return false;
            foreach (var raw in filter.Split (',')) {
                var term = raw.Trim ();
                if (term.Length == 0)
                    continue;
                if (candidate.IndexOf (term, StringComparison.OrdinalIgnoreCase) >= 0)
                    return true;
            }
            return false;
        }

        /// <summary>
        /// Compile a static getter into a delegate.
        ///
        /// Use this for anything a kRPC stream will poll. A stream re-reads its value
        /// every FixedUpdate: at 60 Hz with a dozen streams, MethodInfo.Invoke's boxing
        /// and argument-array allocation show up as GC stutter, while a compiled delegate
        /// costs roughly a virtual call. Resolve once at load, keep the delegate.
        /// </summary>
        public static Func<T> StaticGetter<T> (Type type, string propertyName)
        {
            if (type == null)
                return null;
            var property = type.GetProperty (propertyName, AnyStatic);
            if (property != null && property.GetGetMethod (true) != null)
                return (Func<T>)Delegate.CreateDelegate (typeof (Func<T>), property.GetGetMethod (true), false);
            return null;
        }

        /// <summary>
        /// Bind a one-argument static method to a delegate, so calling it costs a virtual
        /// call rather than a <c>MethodInfo.Invoke</c> plus an <c>object[]</c> allocation.
        ///
        /// The return type is <c>object</c> deliberately: the method's real return type
        /// belongs to a mod assembly and cannot be named at compile time here. Delegate
        /// creation allows reference-type covariance on the return, so this binds
        /// correctly to any method returning a class - and returns null rather than
        /// throwing if it cannot, so the caller can keep a reflection fallback.
        ///
        /// Worth doing for anything on the path of a streamed property. MechJeb's
        /// GetMasterMechJeb is the case in point: every single MechJeb getter goes through
        /// it, and a stream would call it sixty times a second.
        /// </summary>
        public static Func<TArg, object> StaticCall<TArg> (MethodInfo method)
        {
            if (method == null || !method.IsStatic || method.ReturnType.IsValueType)
                return null;
            var parameters = method.GetParameters ();
            if (parameters.Length != 1 || parameters [0].ParameterType != typeof (TArg))
                return null;
            return (Func<TArg, object>) Delegate.CreateDelegate (
                typeof (Func<TArg, object>), method, false);
        }

        /// <summary>Compile an instance getter into an open delegate: getter(instance).</summary>
        public static Func<object, T> InstanceGetter<T> (Type type, string propertyName)
        {
            if (type == null)
                return null;
            var property = type.GetProperty (propertyName, AnyInst);
            var getter = property != null ? property.GetGetMethod (true) : null;
            if (getter == null)
                return null;
            // Late-bound but resolved once; still an order of magnitude cheaper than
            // MethodInfo.Invoke on every call.
            return instance => (T)getter.Invoke (instance, null);
        }
    }
}
