using System;
using System.Collections.Generic;
using System.Reflection;

namespace KRPC.Bridge.Actuators
{
    /// <summary>
    /// QW6 (plan GNC v3): cached reflection for the per-tick captures (DynamicsSnapshotV3,
    /// AeroActuatorV1).
    ///
    /// The legacy readers built a key "AssemblyQualifiedName|name" on EVERY access (about
    /// thirty per tick in v3, a string built by the runtime plus a concatenation) and
    /// AeroActuatorV1 called GetField/GetProperty/GetIndexParameters for every module at
    /// every tick. Here a member is resolved once per (runtime type, name), with EXACTLY
    /// the legacy resolution order and binding flags, and property getters are bound once
    /// to a delegate (fallback: PropertyInfo.GetValue). The values read are the same
    /// objects, so the frames are the same bytes: build/tests compares both readers on
    /// synthetic types, and the in-game RPC dynamics_reflection_check_v1 captures the
    /// active vessel through both paths and compares the doubles bit for bit.
    ///
    /// Pure System.Reflection: no Unity type, so build/tests compiles it as is.
    /// </summary>
    internal static class ReflectionCache
    {
        /// <summary>True only inside dynamics_reflection_check_v1: route to the legacy readers.</summary>
        internal static bool UseLegacy;

        const BindingFlags Flags = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;

        sealed class Accessor
        {
            internal MemberInfo Member;          // null: absent
            internal Func<object, object> Getter; // compiled property getter, or null
        }

        static readonly Dictionary<Type, Dictionary<string, Accessor>> propertyThenField =
            new Dictionary<Type, Dictionary<string, Accessor>> ();
        static readonly Dictionary<Type, Dictionary<string, Accessor>> fieldThenProperty =
            new Dictionary<Type, Dictionary<string, Accessor>> ();
        static readonly Dictionary<Type, Dictionary<string, MethodInfo>> zeroArgMethods =
            new Dictionary<Type, Dictionary<string, MethodInfo>> ();

        static readonly MethodInfo makeGetter = typeof (ReflectionCache).GetMethod (
            "MakeGetter", BindingFlags.NonPublic | BindingFlags.Static);

        internal static int CachedMembers {
            get {
                int n = 0;
                foreach (var pair in propertyThenField)
                    n += pair.Value.Count;
                foreach (var pair in fieldThenProperty)
                    n += pair.Value.Count;
                return n;
            }
        }

        /// <summary>
        /// DynamicsV3 semantics: for each name in order, the instance property (public or
        /// not), else the field; the first member that exists and reads without throwing
        /// wins. Resolution exceptions propagate, as in the legacy reader.
        /// </summary>
        internal static object ReadPropertyThenField (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            Dictionary<string, Accessor> byName;
            if (!propertyThenField.TryGetValue (type, out byName)) {
                byName = new Dictionary<string, Accessor> (StringComparer.Ordinal);
                propertyThenField [type] = byName;
            }
            for (int i = 0; i < names.Length; i++) {
                Accessor accessor;
                if (!byName.TryGetValue (names [i], out accessor)) {
                    MemberInfo member = (MemberInfo)type.GetProperty (names [i], Flags)
                        ?? type.GetField (names [i], Flags);
                    accessor = new Accessor {
                        Member = member,
                        Getter = CompileGetter (member as PropertyInfo)
                    };
                    byName [names [i]] = accessor;
                }
                if (accessor.Member == null)
                    continue;
                try {
                    var property = accessor.Member as PropertyInfo;
                    if (property != null)
                        return accessor.Getter != null
                            ? accessor.Getter (target)
                            : property.GetValue (target, null);
                    var field = accessor.Member as FieldInfo;
                    if (field != null)
                        return field.GetValue (target);
                } catch {
                    // A single unavailable live member must not kill the physics loop.
                }
            }
            return null;
        }

        /// <summary>DynamicsV3 InvokeZeroArg semantics: first zero-argument instance method.</summary>
        internal static object InvokeZeroArg (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            Dictionary<string, MethodInfo> byName;
            if (!zeroArgMethods.TryGetValue (type, out byName)) {
                byName = new Dictionary<string, MethodInfo> (StringComparer.Ordinal);
                zeroArgMethods [type] = byName;
            }
            for (int i = 0; i < names.Length; i++) {
                MethodInfo method;
                if (!byName.TryGetValue (names [i], out method)) {
                    method = type.GetMethod (names [i], Flags, null, Type.EmptyTypes, null);
                    byName [names [i]] = method;
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

        /// <summary>
        /// AeroActuatorV1 semantics: for each name, the field, else a non-indexed property;
        /// a lookup or read that throws moves on to the next name (the legacy reader did
        /// both inside one try).
        /// </summary>
        internal static object ReadFieldThenProperty (object target, string[] names)
        {
            if (target == null)
                return null;
            Type type = target.GetType ();
            Dictionary<string, Accessor> byName;
            if (!fieldThenProperty.TryGetValue (type, out byName)) {
                byName = new Dictionary<string, Accessor> (StringComparer.Ordinal);
                fieldThenProperty [type] = byName;
            }
            for (int i = 0; i < names.Length; i++) {
                try {
                    Accessor accessor;
                    if (!byName.TryGetValue (names [i], out accessor)) {
                        MemberInfo member = type.GetField (names [i], Flags);
                        if (member == null) {
                            var property = type.GetProperty (names [i], Flags);
                            if (property != null && property.GetIndexParameters ().Length == 0)
                                member = property;
                        }
                        accessor = new Accessor {
                            Member = member,
                            Getter = CompileGetter (member as PropertyInfo)
                        };
                        byName [names [i]] = accessor;
                    }
                    var field = accessor.Member as FieldInfo;
                    if (field != null)
                        return field.GetValue (target);
                    var prop = accessor.Member as PropertyInfo;
                    if (prop != null)
                        return accessor.Getter != null
                            ? accessor.Getter (target)
                            : prop.GetValue (target, null);
                } catch { }
            }
            return null;
        }

        /// <summary>
        /// An open-instance delegate over the property getter, boxed like GetValue boxes.
        /// Reference types only (a struct getter needs a by-ref target); null when anything
        /// does not bind, and the caller then uses PropertyInfo.GetValue.
        /// </summary>
        static Func<object, object> CompileGetter (PropertyInfo property)
        {
            if (property == null || makeGetter == null)
                return null;
            try {
                var getter = property.GetGetMethod (true);
                if (getter == null || getter.IsStatic || getter.GetParameters ().Length != 0)
                    return null;
                var declaring = getter.DeclaringType;
                var result = getter.ReturnType;
                if (declaring == null || declaring.IsValueType || declaring.ContainsGenericParameters ||
                        result == typeof (void) || result.IsByRef || result.IsPointer)
                    return null;
                return (Func<object, object>)makeGetter.MakeGenericMethod (declaring, result)
                    .Invoke (null, new object[] { getter });
            } catch {
                return null;
            }
        }

        static Func<object, object> MakeGetter<TTarget, TValue> (MethodInfo getter)
        {
            var typed = (Func<TTarget, TValue>)Delegate.CreateDelegate (
                typeof (Func<TTarget, TValue>), getter);
            return target => typed ((TTarget)target);
        }

        internal static void Clear ()
        {
            propertyThenField.Clear ();
            fieldThenProperty.Clear ();
            zeroArgMethods.Clear ();
        }
    }
}
