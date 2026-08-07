// -----------------------------------------------------------------------------------
// Offline kRPC service-signature validation.
//
// WHY THIS MATTERS MORE THAN ANY OTHER CHECK IN THIS REPO.
//
// kRPC scans every loaded assembly when the server starts. ONE malformed service
// signature - anywhere, in any DLL - sets ServicesChecker.OK to false, which gates
// kRPC's FixedUpdate. The result is that the whole kRPC server goes dead, not just the
// offending service, and the only symptom is a popup and a line in KSP.log. From Python
// it looks identical to "the server is not running".
//
// With one DLL that was a nuisance. With a Core and three plugins it is four times the
// surface, and it is the number-one risk of the whole architecture. There is no runtime
// fix available: .NET does not unload assemblies, so nothing can be quarantined once KSP
// has started. The only place to catch it is before the DLL reaches GameData.
//
// This tool runs kRPC's OWN scanner - the same KRPC.Service.Scanner.Scanner.GetServices
// the server calls - against the built DLLs, in about a second, with no KSP launch.
//
// Two deliberate choices:
//
//   * It passes an error LIST rather than letting the scanner throw. The scanner collects
//     one error per bad procedure and property instead of stopping at the first, so a
//     build that broke five signatures reports five, not one at a time over five
//     rebuilds. (kRPC's own krpc-servicedefs tool passes null and therefore stops at the
//     first error. That is the one thing this tool does better.)
//
//   * It reaches the scanner by reflection, so this project needs no compile-time
//     reference to KRPC.Core and can stay a plain net8.0 console app while the assemblies
//     it inspects are net472.
//
// Usage:
//     dotnet run --project build/scan -- <KSP root> <dll> [<dll> ...]
//
// Exit code 0 means every signature is one kRPC will accept.
// -----------------------------------------------------------------------------------

using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Loader;

namespace KRPC.Bridge.Scan
{
    public static class Program
    {
        static readonly List<string> probeDirectories = new List<string> ();

        public static int Main (string[] args)
        {
            if (args.Length < 2) {
                Console.Error.WriteLine (
                    "usage: dotnet run --project build/scan -- <KSP root> <dll> [<dll> ...]\n" +
                    "\n" +
                    "  <KSP root>  the KSP install, used to find KSP_x64_Data/Managed and\n" +
                    "              GameData/kRPC. Nothing is written to it.\n" +
                    "  <dll>       one or more assemblies to validate.");
                return 2;
            }

            var kspRoot = args [0];
            var targets = args.Skip (1).ToArray ();

            var managed = FindManaged (kspRoot);
            if (managed == null) {
                Console.Error.WriteLine (
                    "KSP managed assemblies not found under '" + kspRoot + "'.\n" +
                    "Looked for KSP_x64_Data/Managed and KSP_Data/Managed.");
                return 2;
            }
            var krpcDirectory = Path.Combine (kspRoot, "GameData", "kRPC");
            var krpcCore = Path.Combine (krpcDirectory, "KRPC.Core.dll");
            if (!File.Exists (krpcCore)) {
                Console.Error.WriteLine ("KRPC.Core.dll not found at '" + krpcCore + "'. Is kRPC installed?");
                return 2;
            }

            probeDirectories.Add (managed);
            probeDirectories.Add (krpcDirectory);
            foreach (var target in targets) {
                var directory = Path.GetDirectoryName (Path.GetFullPath (target));
                if (directory != null && !probeDirectories.Contains (directory))
                    probeDirectories.Add (directory);
            }

            // Resolve KSP, Unity and inter-plugin references out of the probe directories.
            // Without this, loading a plugin fails on its first reference to
            // Assembly-CSharp and the scan never happens.
            AssemblyLoadContext.Default.Resolving += (context, name) => {
                foreach (var directory in probeDirectories) {
                    var candidate = Path.Combine (directory, name.Name + ".dll");
                    if (File.Exists (candidate))
                        return context.LoadFromAssemblyPath (candidate);
                }
                return null;
            };

            Assembly krpcCoreAssembly;
            try {
                krpcCoreAssembly = AssemblyLoadContext.Default.LoadFromAssemblyPath (krpcCore);
            } catch (Exception e) {
                Console.Error.WriteLine ("could not load KRPC.Core.dll: " + e.Message);
                return 2;
            }

            foreach (var target in targets) {
                var full = Path.GetFullPath (target);
                if (!File.Exists (full)) {
                    Console.Error.WriteLine ("not found: " + full);
                    return 2;
                }
                try {
                    var loaded = AssemblyLoadContext.Default.LoadFromAssemblyPath (full);
                    Console.WriteLine ("loaded  " + loaded.GetName ().Name + " " + loaded.GetName ().Version);
                    // Force type resolution now, so a missing dependency is reported here
                    // with a readable message instead of surfacing inside the scanner.
                    TouchTypes (loaded);
                } catch (Exception e) {
                    Console.Error.WriteLine ("could not load " + Path.GetFileName (full) + ": " + e.Message);
                    return 2;
                }
            }

            return RunScanner (krpcCoreAssembly);
        }

        /// <summary>
        /// Call KRPC.Service.Scanner.Scanner.GetServices(IList&lt;string&gt; errors) by
        /// reflection and report what it found.
        /// </summary>
        static int RunScanner (Assembly krpcCore)
        {
            var scannerType = krpcCore.GetType ("KRPC.Service.Scanner.Scanner");
            if (scannerType == null) {
                Console.Error.WriteLine (
                    "KRPC.Service.Scanner.Scanner not found in KRPC.Core.dll.\n" +
                    "The class moved from server/ to core/ in kRPC 0.5.4; on an older kRPC,\n" +
                    "point this at KRPC.dll instead.");
                return 2;
            }

            var getServices = scannerType.GetMethod (
                "GetServices", BindingFlags.Public | BindingFlags.Static);
            if (getServices == null) {
                Console.Error.WriteLine ("Scanner.GetServices not found - kRPC's internals have changed.");
                return 2;
            }

            var errors = new List<string> ();
            object services;
            try {
                // The optional IList<string> parameter is the whole point: with it the
                // scanner COLLECTS errors instead of throwing on the first one.
                services = getServices.GetParameters ().Length == 1
                    ? getServices.Invoke (null, new object[] { errors })
                    : getServices.Invoke (null, null);
            } catch (TargetInvocationException e) {
                Console.Error.WriteLine ("scanner threw: " + (e.InnerException ?? e).Message);
                return 1;
            }

            var table = services as IDictionary;
            if (table != null) {
                Console.WriteLine ();
                Console.WriteLine ("services found: " + table.Count);
                var names = new List<string> ();
                foreach (var key in table.Keys)
                    names.Add (Convert.ToString (key));
                names.Sort ();
                foreach (var name in names)
                    Console.WriteLine ("  " + name);
            }

            if (errors.Count == 0) {
                Console.WriteLine ();
                Console.WriteLine ("OK - every signature is one kRPC will accept.");
                return 0;
            }

            Console.Error.WriteLine ();
            Console.Error.WriteLine (errors.Count + " service error(s). In game these would");
            Console.Error.WriteLine ("disable the ENTIRE kRPC server, not just this plugin:");
            foreach (var error in errors)
                Console.Error.WriteLine ("  " + error);
            return 1;
        }

        /// <summary>
        /// Resolve every type in an assembly, tolerating a partial failure so the message
        /// names what is actually missing.
        /// </summary>
        static void TouchTypes (Assembly assembly)
        {
            try {
                assembly.GetTypes ();
            } catch (ReflectionTypeLoadException e) {
                var reasons = e.LoaderExceptions
                    .Where (x => x != null)
                    .Select (x => x.Message)
                    .Distinct ()
                    .Take (5);
                throw new InvalidOperationException (
                    "some types could not be loaded: " + string.Join ("; ", reasons));
            }
        }

        static string FindManaged (string kspRoot)
        {
            foreach (var data in new[] { "KSP_x64_Data", "KSP_Data" }) {
                var candidate = Path.Combine (kspRoot, data, "Managed");
                if (Directory.Exists (candidate) &&
                    File.Exists (Path.Combine (candidate, "Assembly-CSharp.dll")))
                    return candidate;
            }
            return null;
        }
    }
}
