using UnityEngine;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// One prefix for the whole mod, so a single findstr pulls every line the bridge
    /// wrote out of a 200 MB KSP.log:
    ///
    ///     findstr /C:"[KRPC.Bridge]" "D:\Games\KSP_1.12.5\KSP.log"
    ///
    /// Plugins log through here rather than Debug.Log directly, and pass their own name,
    /// so the source is always visible in the line itself.
    /// </summary>
    public static class BridgeLog
    {
        const string Prefix = "[KRPC.Bridge] ";

        /// <summary>Normal progress. Keep these rare and meaningful.</summary>
        public static void Info (string message)
        {
            Debug.Log (Prefix + message);
        }

        /// <summary>Something degraded but the mod still works. A missing third-party mod is this, not Error.</summary>
        public static void Warn (string message)
        {
            Debug.LogWarning (Prefix + message);
        }

        /// <summary>Something is broken.</summary>
        public static void Error (string message)
        {
            Debug.LogError (Prefix + message);
        }

        /// <summary>Scoped variant: BridgeLog.Info("Fmrs", "resolved").</summary>
        public static void Info (string source, string message)
        {
            Debug.Log (Prefix + source + ": " + message);
        }

        /// <summary>Scoped variant of <see cref="Warn(string)"/>.</summary>
        public static void Warn (string source, string message)
        {
            Debug.LogWarning (Prefix + source + ": " + message);
        }

        /// <summary>Scoped variant of <see cref="Error(string)"/>.</summary>
        public static void Error (string source, string message)
        {
            Debug.LogError (Prefix + source + ": " + message);
        }
    }
}
