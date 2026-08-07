using System;
using System.Reflection;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Template
{
    /// <summary>
    /// Registration. Every plugin has exactly one of these and it does exactly one thing:
    /// hand Core a resolver and get out of the way.
    ///
    /// Startup.Instantly, once=true. Core's own addon is also Instantly, and KSP has
    /// already ordered the two assemblies by KSPAssemblyDependency, so Core's Awake runs
    /// after this one has registered.
    ///
    /// The resolver must not throw. Core catches anyway, but a plugin that reports its
    /// own failure produces a line in KSP.log that says which member was missing, and
    /// that is the difference between a five-minute fix and an evening.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class TemplateAddon : MonoBehaviour
    {
        void Awake ()
        {
            ModRegistry.Register ("Template", Resolve);
        }

        static PluginStatus Resolve ()
        {
            // ---------------------------------------------------------------
            // Pattern for wrapping a third-party mod.
            //
            // Never add a hard <Reference> to the mod's DLL. Resolve it by name out of
            // AssemblyLoader.loadedAssemblies instead. A hard reference means the plugin
            // fails to load at all when the mod is absent, and takes its kRPC service
            // with it. Reflection means it loads, reports available=false, and the rest
            // of the bridge is unaffected.
            //
            // Assembly names drift. FMRS ships as "FMRSContinued" but its folder is
            // "FMRS". Always pass every historical name as a candidate.
            // ---------------------------------------------------------------
            var assembly = ModRegistry.FindAssembly ("SomeMod", "SomeModContinued");
            if (assembly == null) {
                return new PluginStatus {
                    Available = false,
                    Report = "no assembly named SomeMod or SomeModContinued is loaded"
                };
            }

            var apiType = ModRegistry.FindType (assembly, "SomeMod.API");
            if (apiType == null) {
                return new PluginStatus {
                    Available = false,
                    ModVersion = ModRegistry.VersionOf (assembly),
                    Report = "assembly found but type SomeMod.API did not resolve"
                };
            }

            // Resolve every member ONCE, here, and cache the MethodInfo or a compiled
            // delegate. Resolution depends only on which assemblies are loaded, never on
            // the scene, so it never has to be redone. Doing it per call is the single
            // most common performance mistake in a bridge plugin.
            method = apiType.GetMethod ("DoSomething", ModRegistry.PubStatic);
            if (method == null) {
                return new PluginStatus {
                    Available = false,
                    ModVersion = ModRegistry.VersionOf (assembly),
                    Report = "SomeMod.API found but DoSomething() is missing - version drift?"
                };
            }

            return new PluginStatus {
                Available = true,
                ModVersion = ModRegistry.VersionOf (assembly),
                Report = "resolved"
            };
        }

        internal static MethodInfo method;

        /// <summary>Whether the target mod resolved. Read by the service.</summary>
        internal static bool Available { get { return method != null; } }
    }
}
