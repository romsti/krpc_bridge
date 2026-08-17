using System.Reflection;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.Ident
{
    /// <summary>
    /// Registration, so conn.bridge.plugins reports this plugin like any other.
    ///
    /// There is no third-party mod to resolve here, so unlike every other plugin in this
    /// repo the resolver has nothing to look up by reflection: the types it needs are a
    /// compile-time reference. What it CAN do is report the kRPC SpaceCenter version, so a
    /// human reading conn.bridge.plugins sees which build the identifiers came from.
    ///
    /// One honest limitation, worth writing down rather than discovering: if
    /// KRPC.SpaceCenter.dll is missing, this resolver never runs, because the assembly
    /// fails to load before it. The report below cannot describe that case - KSP.log can,
    /// and the csproj says where to look.
    /// </summary>
    [KSPAddon (KSPAddon.Startup.Instantly, true)]
    public sealed class IdentAddon : MonoBehaviour
    {
        void Awake ()
        {
            ModRegistry.Register ("Ident", Resolve);
        }

        static PluginStatus Resolve ()
        {
            var assembly = ModRegistry.FindAssembly ("KRPC.SpaceCenter");
            if (assembly == null) {
                // Reaching here means the assembly loaded (so the reference resolved at
                // JIT time) but is not in AssemblyLoader's list under that name. Odd
                // enough to report rather than assume.
                return new PluginStatus {
                    Available = false,
                    Report = "KRPC.SpaceCenter is referenced but not listed in AssemblyLoader - partial kRPC install?"
                };
            }

            resolved = true;
            return new PluginStatus {
                Available = true,
                ModVersion = ModRegistry.VersionOf (assembly),
                Report = "resolved"
            };
        }

        static bool resolved;

        /// <summary>
        /// Whether SpaceCenter resolved. Read by the service.
        ///
        /// ModRegistry resolves once per session and caches, so this is set before any RPC
        /// can arrive: kRPC's server starts after KSP has loaded every assembly and run
        /// every Instantly addon.
        /// </summary>
        internal static bool Available { get { return resolved; } }
    }
}
