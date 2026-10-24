// -----------------------------------------------------------------------------------
// The manifest (conn.bridge.manifest_v1): pure tests. See ManifestTests.csproj.
//
// Usage:
//     dotnet run --project build/tests/manifest/ManifestTests.csproj -c Release
//
// This assembly is named KRPC.Bridge.ManifestTests, so the manifest treats it as a bridge
// assembly: it carries a stamp (below), two stand-in kRPC services and constants of its own.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Reflection;

[assembly: AssemblyVersion ("9.8.7.6")]
[assembly: AssemblyInformationalVersion ("1.1.0+0123456789abcdef0123456789abcdef01234567")]
[assembly: AssemblyMetadata ("KRPC.Bridge.GitSha", "0123456789abcdef0123456789abcdef01234567")]
[assembly: AssemblyMetadata ("KRPC.Bridge.GitDirty", "0")]
[assembly: AssemblyMetadata ("KRPC.Bridge.SourceDate", "2026-10-03T17:56:02Z")]
[assembly: AssemblyMetadata ("Unrelated.Key", "ignored")]

namespace KRPC.Service.Attributes
{
    // Stand-in for kRPC's attribute: the manifest matches it by full name, as attribute data.
    [AttributeUsage (AttributeTargets.Class)]
    public sealed class KRPCServiceAttribute : Attribute
    {
        public string Name { get; set; }
    }
}

namespace KRPC.Bridge.ManifestTests
{
    [KRPC.Service.Attributes.KRPCService (Name = "Alpha")]
    public static class AlphaService { }

    // No Name: kRPC names the service after the class, and so must the manifest.
    [KRPC.Service.Attributes.KRPCService]
    public static class BetaService { }

    internal static class Versions
    {
        internal const int ProtocolVersion = 4;
        internal const int SchemaVersion = 7;
        internal const long Wide = 1L << 40;

        // Not a constant: reading it would run the type initializer, which must not happen.
        // The flag lives in another class, so asking for it does not run this initializer.
        internal static readonly int NotConst = Touch ();

        static int Touch ()
        {
            InitProbe.Touched = true;
            return 3;
        }
    }

    internal static class InitProbe
    {
        internal static bool Touched;
    }
}

namespace KRPC.Bridge.Core
{
    public static class Program
    {
        static int failures;
        static int checks;

        const string Sha = "0123456789abcdef0123456789abcdef01234567";
        const string Me = "KRPC.Bridge.ManifestTests";

        public static int Main (string[] args)
        {
            TestIsBridgeAssembly ();
            TestReadStamp ();
            TestServicesOf ();
            TestReadConst ();
            TestCollect ();
            TestCollectSkipsOtherAssemblies ();
            TestSerializeConsistent ();
            TestSerializeMixedDirtyUnstamped ();
            TestSerializeCleansFields ();
            TestKnownTable ();

            Console.WriteLine ();
            Console.WriteLine ("checks: " + checks + ", failures: " + failures);
            return failures == 0 ? 0 : 1;
        }

        // ------------------------------------------------------------------ helpers

        static void Check (bool ok, string what)
        {
            checks++;
            if (ok)
                return;
            failures++;
            Console.WriteLine ("FAIL: " + what);
        }

        static void Equal<T> (T expected, T actual, string what)
        {
            Check (EqualityComparer<T>.Default.Equals (expected, actual),
                   what + " (expected " + expected + ", got " + actual + ")");
        }

        static Assembly Self {
            get { return typeof (Program).Assembly; }
        }

        static Dictionary<string, string> Header (string line)
        {
            var map = new Dictionary<string, string> ();
            foreach (var pair in line.Split ('\t')) {
                var at = pair.IndexOf ('=');
                Check (at > 0, "header field '" + pair + "' is key=value");
                if (at > 0)
                    map [pair.Substring (0, at)] = pair.Substring (at + 1);
            }
            return map;
        }

        static BridgeManifest.Row Row (string assembly, string sha, string dirty)
        {
            return new BridgeManifest.Row {
                Assembly = assembly,
                Stamp = new BridgeManifest.Stamp { GitSha = sha, GitDirty = dirty }
            };
        }

        // ------------------------------------------------------------------ tests

        static void TestIsBridgeAssembly ()
        {
            Check (BridgeManifest.IsBridgeAssembly ("KRPC.Bridge"), "the old single DLL counts");
            Check (BridgeManifest.IsBridgeAssembly ("KRPC.Bridge.Core"), "Core counts");
            Check (BridgeManifest.IsBridgeAssembly ("KRPC.Bridge.Actuators"), "a plugin counts");
            Check (!BridgeManifest.IsBridgeAssembly ("KRPC.BridgeFoo"), "a name that only starts alike does not");
            Check (!BridgeManifest.IsBridgeAssembly ("KRPC.Core"), "kRPC itself does not");
            Check (!BridgeManifest.IsBridgeAssembly (""), "empty does not");
            Check (!BridgeManifest.IsBridgeAssembly (null), "null does not");
        }

        static void TestReadStamp ()
        {
            var s = BridgeManifest.ReadStamp (Self);
            Equal (Sha, s.GitSha, "git sha read back");
            Equal ("0", s.GitDirty, "dirty flag read back");
            Equal ("2026-10-03T17:56:02Z", s.SourceDate, "source date read back");
            Equal ("1.1.0+" + Sha, s.InformationalVersion, "informational version read back");
            Equal ("9.8.7.6", s.AssemblyVersion, "assembly version read back");

            var none = BridgeManifest.ReadStamp (typeof (object).Assembly);
            Equal ("", none.GitSha, "an assembly without a stamp has an empty sha");
            Equal ("", none.GitDirty, "and an empty dirty flag");

            var nil = BridgeManifest.ReadStamp (null);
            Equal ("", nil.GitSha, "null assembly: empty stamp, no exception");

            Equal ("commit " + Sha + ", 2026-10-03T17:56:02Z", BridgeManifest.StampLine (Self), "log line");
            Equal ("no build stamp", BridgeManifest.StampLine (typeof (object).Assembly), "log line without stamp");
        }

        static void TestServicesOf ()
        {
            var names = BridgeManifest.ServicesOf (Self);
            Equal (2, names.Count, "two services declared");
            if (names.Count == 2) {
                Equal ("Alpha", names [0], "named service, sorted first");
                Equal ("BetaService", names [1], "unnamed service named after its class");
            }
            Equal (0, BridgeManifest.ServicesOf (typeof (object).Assembly).Count, "corelib declares no kRPC service");
            Equal (0, BridgeManifest.ServicesOf (null).Count, "null assembly: no service, no exception");
        }

        static void TestReadConst ()
        {
            const string T = "KRPC.Bridge.ManifestTests.Versions";
            Equal ("4", BridgeManifest.ReadConst (Self, T, "ProtocolVersion"), "int constant");
            Equal ("7", BridgeManifest.ReadConst (Self, T, "SchemaVersion"), "second constant");
            Equal ("1099511627776", BridgeManifest.ReadConst (Self, T, "Wide"), "long constant, invariant text");
            Equal ("", BridgeManifest.ReadConst (Self, T, ""), "no field named: empty");
            Equal (BridgeManifest.Missing, BridgeManifest.ReadConst (Self, T, "Nope"), "missing field");
            Equal (BridgeManifest.Missing, BridgeManifest.ReadConst (Self, "KRPC.Bridge.Nope", "ProtocolVersion"), "missing type");
            Equal (BridgeManifest.Missing, BridgeManifest.ReadConst (Self, T, "NotConst"), "static readonly is not read");
            Check (!KRPC.Bridge.ManifestTests.InitProbe.Touched, "no type initializer ran");
        }

        static void TestCollect ()
        {
            var known = new[] {
                new BridgeManifest.Layout (Me, "Alpha", "zeta_v1", "KRPC.Bridge.ManifestTests.Versions", "ProtocolVersion", "SchemaVersion"),
                new BridgeManifest.Layout (Me, "Alpha", "alpha_v1", "KRPC.Bridge.ManifestTests.Versions", "ProtocolVersion", ""),
                new BridgeManifest.Layout (Me, "Alpha", "gone_v1", "KRPC.Bridge.ManifestTests.Versions", "Gone", ""),
                new BridgeManifest.Layout ("KRPC.Bridge.Other", "Other", "other_v1", "X", "Y", "Z"),
            };
            var rows = BridgeManifest.Collect (new[] { Self, Self.GetType ().Assembly }, known);
            // Self appears once in the input that matters; corelib is dropped.
            Equal (5, rows.Count, "two service rows and three layout rows");
            if (rows.Count != 5)
                return;
            Equal ("Alpha", rows [0].Service, "service rows first");
            Equal ("", rows [0].Layout, "a service row has no layout");
            Equal ("BetaService", rows [1].Service, "second service");
            Equal ("alpha_v1", rows [2].Layout, "layouts sorted by name");
            Equal ("4", rows [2].Protocol, "protocol read");
            Equal ("", rows [2].Schema, "no schema number");
            Equal ("gone_v1", rows [3].Layout, "a missing constant still gets its row");
            Equal (BridgeManifest.Missing, rows [3].Protocol, "and says so");
            Equal ("zeta_v1", rows [4].Layout, "last layout");
            Equal ("7", rows [4].Schema, "schema read");
            foreach (var row in rows) {
                Equal (Me, row.Assembly, "assembly name on every row");
                Equal (Sha, row.Stamp.GitSha, "stamp on every row");
            }
        }

        static void TestCollectSkipsOtherAssemblies ()
        {
            var rows = BridgeManifest.Collect (new[] { typeof (object).Assembly, typeof (Console).Assembly, null });
            Equal (0, rows.Count, "no bridge assembly, no row");
            Equal (0, BridgeManifest.Collect (null).Count, "null input, no row");
        }

        static void TestSerializeConsistent ()
        {
            var rows = new List<BridgeManifest.Row> {
                Row ("KRPC.Bridge.Core", Sha, "0"),
                Row ("KRPC.Bridge.Core", Sha, "0"),
                Row ("KRPC.Bridge.Fmrs", Sha, "0"),
            };
            rows [1].Layout = "manifest_v1";
            rows [1].Protocol = "1";
            rows [1].Schema = "1";
            var lines = BridgeManifest.Serialize (rows);
            Equal (5, lines.Count, "header, columns, three rows");
            var h = Header (lines [0]);
            Equal ("1", h ["protocol"], "protocol");
            Equal ("1", h ["schema"], "schema");
            Equal ("3", h ["n"], "n counts rows");
            Equal ("2", h ["assemblies"], "assemblies counted once each");
            Equal (Sha, h ["git_sha"], "one commit");
            Equal ("0", h ["git_dirty"], "clean");
            Equal ("1", h ["consistent"], "one clean build");
            Equal (string.Join ("\t", BridgeManifest.Columns), lines [1], "column row");
            Equal (BridgeManifest.Columns.Length, lines [3].Split ('\t').Length, "every row has every column");
            Equal ("KRPC.Bridge.Core\t\tmanifest_v1\t1\t1\t" + Sha + "\t0\t\t\t", lines [3], "row text");
        }

        static void TestSerializeMixedDirtyUnstamped ()
        {
            var mixed = Header (BridgeManifest.Serialize (new List<BridgeManifest.Row> {
                Row ("KRPC.Bridge.Core", Sha, "0"),
                Row ("KRPC.Bridge.Actuators", "ffff", "0"),
            }) [0]);
            Equal ("mixed", mixed ["git_sha"], "two commits: mixed");
            Equal ("0", mixed ["consistent"], "two commits: not consistent");

            var dirty = Header (BridgeManifest.Serialize (new List<BridgeManifest.Row> {
                Row ("KRPC.Bridge.Core", Sha, "0"),
                Row ("KRPC.Bridge.Actuators", Sha, "1"),
            }) [0]);
            Equal (Sha, dirty ["git_sha"], "same commit");
            Equal ("1", dirty ["git_dirty"], "one dirty assembly makes the set dirty");
            Equal ("0", dirty ["consistent"], "dirty: not consistent");

            var unstamped = Header (BridgeManifest.Serialize (new List<BridgeManifest.Row> {
                Row ("KRPC.Bridge.Core", Sha, "0"),
                Row ("KRPC.Bridge.Tower", "", ""),
            }) [0]);
            Equal (Sha, unstamped ["git_sha"], "the stamped commit");
            Equal ("1", unstamped ["git_dirty"], "an unstamped assembly cannot be proven clean");
            Equal ("0", unstamped ["consistent"], "an old DLL among new ones: not consistent");

            var empty = BridgeManifest.Serialize (new List<BridgeManifest.Row> ());
            var e = Header (empty [0]);
            Equal ("0", e ["n"], "empty: n = 0");
            Equal ("", e ["git_sha"], "empty: no commit");
            Equal ("0", e ["consistent"], "empty: not consistent");
            Equal (2, BridgeManifest.Serialize (null).Count, "null rows: header and columns");
        }

        static void TestSerializeCleansFields ()
        {
            var r = Row ("KRPC.Bridge.Core", Sha, "0");
            r.Service = "a\tb\nc\rd";
            var lines = BridgeManifest.Serialize (new List<BridgeManifest.Row> { r });
            Equal (BridgeManifest.Columns.Length, lines [2].Split ('\t').Length, "tabs inside a field never split it");
            Check (lines [2].IndexOf ('\n') < 0 && lines [2].IndexOf ('\r') < 0, "no line break inside a row");
        }

        static void TestKnownTable ()
        {
            var seen = new HashSet<string> ();
            foreach (var layout in BridgeManifest.Known) {
                var key = layout.Assembly + "/" + layout.Name;
                Check (seen.Add (key), "layout listed once: " + key);
                Check (BridgeManifest.IsBridgeAssembly (layout.Assembly), "bridge assembly: " + key);
                Check (!string.IsNullOrEmpty (layout.Service), "service named: " + key);
                Check (!string.IsNullOrEmpty (layout.Type), "type named: " + key);
                Check (!string.IsNullOrEmpty (layout.ProtocolField), "protocol constant named: " + key);
                Check (layout.SchemaField != null, "schema constant never null: " + key);
                Check (layout.Name == layout.Name.ToLowerInvariant () && layout.Name.IndexOf (' ') < 0,
                       "layout names are snake_case: " + key);
            }
            Check (seen.Count >= 16, "the table knows every versioned layout of this build (" + seen.Count + ")");
            // The Core's own layout is in the table and resolves in this very assembly's copy.
            Equal ("1", BridgeManifest.ReadConst (typeof (BridgeManifest).Assembly,
                                                   "KRPC.Bridge.Core.BridgeManifest", "Protocol"),
                   "manifest_v1 protocol constant resolves");
        }
    }
}
