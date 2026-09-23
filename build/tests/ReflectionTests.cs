// -----------------------------------------------------------------------------------
// QW6: the cached reflection readers return exactly what the legacy readers returned.
//
// The two legacy readers below are VERBATIM copies of DynamicsV3.ReadMemberLegacy /
// InvokeZeroArgLegacy and AeroActuatorV1.ReadMemberLegacy (those files need Unity and
// KSP, this harness does not). The in-game counterpart, on the real vessel, is the RPC
// dynamics_reflection_check_v1.
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;
using System.Reflection;

namespace KRPC.Bridge.Actuators
{
    public static class ReflectionTests
    {
        // ------------------------------------------------ legacy readers (verbatim)

        static readonly Dictionary<string, MemberInfo> memberCache =
            new Dictionary<string, MemberInfo> (StringComparer.Ordinal);
        static readonly Dictionary<string, MethodInfo> methodCache =
            new Dictionary<string, MethodInfo> (StringComparer.Ordinal);

        static object LegacyV3ReadMember (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            for (int i = 0; i < names.Length; i++) {
                string cacheKey = type.AssemblyQualifiedName + "|" + names [i];
                MemberInfo member;
                if (!memberCache.TryGetValue (cacheKey, out member)) {
                    member = (MemberInfo)type.GetProperty (
                        names [i], BindingFlags.Instance | BindingFlags.Public |
                        BindingFlags.NonPublic)
                        ?? type.GetField (
                            names [i], BindingFlags.Instance | BindingFlags.Public |
                            BindingFlags.NonPublic);
                    memberCache [cacheKey] = member;
                }
                if (member == null)
                    continue;
                try {
                    var property = member as PropertyInfo;
                    if (property != null)
                        return property.GetValue (target, null);
                    var field = member as FieldInfo;
                    if (field != null)
                        return field.GetValue (target);
                } catch {
                    // A single unavailable live member must not kill the physics loop.
                }
            }
            return null;
        }

        static object LegacyV3InvokeZeroArg (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            for (int i = 0; i < names.Length; i++) {
                string cacheKey = type.AssemblyQualifiedName + "|" + names [i] + "()";
                MethodInfo method;
                if (!methodCache.TryGetValue (cacheKey, out method)) {
                    method = type.GetMethod (
                        names [i], BindingFlags.Instance | BindingFlags.Public |
                        BindingFlags.NonPublic, null, Type.EmptyTypes, null);
                    methodCache [cacheKey] = method;
                }
                if (method == null)
                    continue;
                try {
                    return method.Invoke (target, null);
                } catch {
                    // Best-effort capability discovery only.
                }
            }
            return null;
        }

        static object LegacyAeroReadMember (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            const BindingFlags flags = BindingFlags.Instance | BindingFlags.Public |
                BindingFlags.NonPublic;
            for (int i = 0; i < names.Length; i++) {
                try {
                    var field = type.GetField (names [i], flags);
                    if (field != null)
                        return field.GetValue (target);
                    var property = type.GetProperty (names [i], flags);
                    if (property != null && property.GetIndexParameters ().Length == 0)
                        return property.GetValue (target, null);
                } catch { }
            }
            return null;
        }

        // ------------------------------------------------------------ synthetic types

        public struct Vec
        {
            public float x, y, z;
            public Vec (float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
            public float magnitude { get { return (float)Math.Sqrt (x * x + y * y + z * z); } }
        }

        public class Base
        {
            public int Pub = 1;
            private double priv = 2.5;
            protected string Prot { get { return "prot"; } }
            public virtual float VProp { get { return 3.5f; } }
            public Vec Field3 = new Vec (1f, 2f, 3f);
            public Vec Prop3 { get { return new Vec (4f, 5f, 6f); } }
            public double NaNValue { get { return double.NaN; } }
            public bool Flag { get { return true; } }
            public Vec GetWorldPos () { return new Vec (7f, 8f, 9f); }
            public double Throwing () { throw new InvalidOperationException ("m"); }
            public double Overloaded (int x) { return x; }
            public double Overloaded () { return 42.0; }
            public double Priv () { return priv; }
        }

        public class Derived : Base
        {
            public override float VProp { get { return 4.5f; } }
            public int Throws { get { throw new InvalidOperationException ("p"); } }
            public double this [int i] { get { return i; } }
            public string Item2 { get { return "item"; } }
            public object Boxed { get { return 12L; } }
            internal float deployFraction = 0.25f;
            public float deployAngle { get { return 80f; } }
            public double[] Array { get { return new[] { 1.0, 2.0 }; } }
        }

        public class Hider : Base
        {
            public string VPropHidden;
            // Two properties named Item (indexers with different arity): ambiguous.
            public int this [int i] { get { return i; } }
            public int this [int i, int j] { get { return i + j; } }
        }

        public class ShadowProperty : Base
        {
            public new double Flag { get { return 5.0; } }
        }

        // ------------------------------------------------------------------ the test

        static int checks, failures;

        static void Check (bool ok, string what)
        {
            checks++;
            if (!ok) {
                failures++;
                Console.Error.WriteLine ("FAIL: " + what);
            }
        }

        static bool Same (object a, object b)
        {
            if (a == null || b == null)
                return a == null && b == null;
            if (a.GetType () != b.GetType ())
                return false;
            if (a is double)
                return BitConverter.DoubleToInt64Bits ((double)a) == BitConverter.DoubleToInt64Bits ((double)b);
            if (a is float)
                return BitConverter.SingleToInt32Bits ((float)a) == BitConverter.SingleToInt32Bits ((float)b);
            if (a is double[]) {
                var x = (double[])a;
                var y = (double[])b;
                if (x.Length != y.Length)
                    return false;
                for (int i = 0; i < x.Length; i++)
                    if (BitConverter.DoubleToInt64Bits (x [i]) != BitConverter.DoubleToInt64Bits (y [i]))
                        return false;
                return true;
            }
            return a.Equals (b);
        }

        static void Compare (string reader, Func<object, string[], object> legacy,
            Func<object, string[], object> cached, object target, string[] names)
        {
            object l = null, c = null;
            Type le = null, ce = null;
            try { l = legacy (target, names); } catch (Exception e) { le = e.GetType (); }
            try { c = cached (target, names); } catch (Exception e) { ce = e.GetType (); }
            string what = reader + " " + (target == null ? "null" : target.GetType ().Name) +
                "." + string.Join ("|", names);
            Check (le == ce, what + ": same exception (" + le + " / " + ce + ")");
            Check (Same (l, c), what + ": same value (" + l + " / " + c + ")");
        }

        public static int Run (out int total)
        {
            ReflectionCache.Clear ();
            object[] targets = {
                new Base (), new Derived (), new Hider (), new ShadowProperty (),
                new Vec (1f, -2f, 3.25f), 1.5f, "text", null
            };
            string[][] nameSets = {
                new[] { "Pub" }, new[] { "priv" }, new[] { "Prot" }, new[] { "VProp" },
                new[] { "Field3" }, new[] { "Prop3" }, new[] { "NaNValue" }, new[] { "Flag" },
                new[] { "Throws" }, new[] { "Throws", "Pub" }, new[] { "missing" },
                new[] { "missing", "priv" }, new[] { "Item" }, new[] { "Item2" },
                new[] { "Boxed" }, new[] { "deployFraction", "deployAngle" },
                new[] { "deployAngle" }, new[] { "x" }, new[] { "y" }, new[] { "z" },
                new[] { "magnitude" }, new[] { "Array" }, new[] { "Length" },
                new[] { "VPropHidden", "VProp" }
            };
            string[][] methodSets = {
                new[] { "GetWorldPos" }, new[] { "Throwing" }, new[] { "Throwing", "Priv" },
                new[] { "Overloaded" }, new[] { "missing" }, new[] { "ToString" },
                new[] { "GetHashCode" }
            };
            // Twice: the second pass reads through a warm cache.
            for (int pass = 0; pass < 2; pass++) {
                foreach (var target in targets) {
                    foreach (var names in nameSets) {
                        Compare ("v3", LegacyV3ReadMember, ReflectionCache.ReadPropertyThenField,
                            target, names);
                        Compare ("aero", LegacyAeroReadMember, ReflectionCache.ReadFieldThenProperty,
                            target, names);
                    }
                    foreach (var names in methodSets)
                        Compare ("method", LegacyV3InvokeZeroArg, ReflectionCache.InvokeZeroArg,
                            target, names);
                }
            }
            total = checks;
            Console.WriteLine ("  reflection cache vs legacy: " + checks + " comparisons, " +
                ReflectionCache.CachedMembers + " members cached");
            return failures;
        }
    }
}
