using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using KRPC.Bridge.Core;

namespace KRPC.Bridge.MechJeb
{
    /// <summary>
    /// Reading and writing any MechJeb member, chosen BY SHAPE rather than by name.
    ///
    /// This file is the reason the service can reach all of MechJeb without naming any of
    /// it. Nowhere below is there a <c>typeof (EditableDouble)</c> or a string like
    /// "MechJebModuleStagingController". A member is classified by what it looks like -
    /// is it an enum, does it carry a writable numeric <c>Val</c>, does it decompose into
    /// degrees/minutes/seconds - and the classification decides which channel carries it.
    ///
    /// That is not stylistic. The bridge this one replaces died because it bound roughly
    /// forty-eight MuMech types by exact name and MechJeb renamed them; binding two hundred
    /// individual MEMBERS by name would be the same mistake at finer grain, and would rot
    /// more slowly and less visibly. Shape survives renames. When it does not, the member
    /// simply reports its type through <see cref="Describe"/> and refuses the write with a
    /// message naming what it found, instead of vanishing.
    ///
    /// FOUR CHANNELS, because kRPC needs one type per procedure and MechJeb's settings are
    /// not all doubles: numbers, flags, enums (carried as their string names, so a script
    /// says "KEEP_SURFACE" rather than 2) and integer lists (as MechJeb's own "1,2,3" or
    /// "1-3" text grammar). Anything else is readable as text where that means something,
    /// and refused otherwise.
    /// </summary>
    internal static class MechJebMembers
    {
        internal const string ChannelNumber = "number";
        internal const string ChannelFlag = "flag";
        internal const string ChannelEnum = "enum";
        internal const string ChannelList = "list";
        internal const string ChannelText = "text";
        internal const string ChannelUnsupported = "unsupported";

        /// <summary>
        /// Bookkeeping members that are public because C# gave the author no better option,
        /// not because anyone should set them. Filtering by name is a blunt instrument and
        /// this list is deliberately short - everything else is filtered structurally.
        /// </summary>
        static readonly HashSet<string> Blocked = new HashSet<string> (StringComparer.OrdinalIgnoreCase) {
            "Dirty", "UnlockChecked", "Priority", "ProfilerName", "Hidden",
            "WindowVector", "WindowVectorEditor", "PrevShouldDeploy", "PrevAutoDeploy",
            "unlockParts", "unlockTechs", "Core", "Vessel", "VesselState", "Orbit", "MainBody", "Part"
        };

        // ==================================================================
        // Enumerating a module's members
        // ==================================================================

        /// <summary>
        /// Every member of a module worth offering to a script, filtered and classified.
        /// Fields first, then properties, both public and instance.
        /// </summary>
        internal static List<MemberInfo> MembersOf (Type type)
        {
            var result = new List<MemberInfo> ();
            if (type == null)
                return result;

            PropertyInfo[] properties;
            FieldInfo[] fields;
            try {
                properties = type.GetProperties (ModRegistry.PubInst);
                fields = type.GetFields (ModRegistry.PubInst);
            } catch (Exception) {
                return result;
            }

            foreach (var field in fields)
                if (Offerable (field.Name) && !ShadowedByProperty (field, properties))
                    result.Add (field);

            foreach (var property in properties) {
                if (!Offerable (property.Name))
                    continue;
                if (property.GetIndexParameters ().Length > 0)   // an indexer has no name to address
                    continue;
                if (!property.CanRead)
                    continue;
                result.Add (property);
            }
            return result;
        }

        static bool Offerable (string name)
        {
            // A leading underscore is MechJeb's own marker for "internal, use the property".
            return !string.IsNullOrEmpty (name) && name [0] != '_' && !Blocked.Contains (name);
        }

        /// <summary>
        /// Whether a public field is really the backing store of a public property, in
        /// which case the property is the one to offer.
        ///
        /// MechJeb declares several backing fields public right beside their property:
        /// <c>_autostage</c> beside <c>Autostage</c>, <c>AscentTypeInteger</c> beside
        /// <c>AscentType</c>, <c>showInFlight</c> beside <c>ShowInFlight</c>. Writing the
        /// field sets the value and SKIPS the property's side effect - for Autostage that
        /// is the staging-pool registration, which is the exact silent failure this whole
        /// service exists to prevent. Two names, one of which quietly does nothing.
        ///
        /// The suffix rule is a heuristic on a convention, but it is the convention MechJeb
        /// follows, and it fails safe: at worst it hides a field whose property twin works.
        ///
        /// It deliberately does NOT hide a field whose property is READ-ONLY. AscentSettings
        /// exposes pairs like <c>CoastStageInternal</c> + <c>CoastStageFlag</c> under a
        /// get-only <c>CoastStage</c> - there the fields are the only write path, and hiding
        /// them would make the setting unreachable rather than safer.
        /// </summary>
        static bool ShadowedByProperty (FieldInfo field, PropertyInfo[] properties)
        {
            var name = field.Name;
            var stems = new List<string> { name };
            foreach (var suffix in new[] { "Config", "Internal", "Integer" })
                if (name.Length > suffix.Length && name.EndsWith (suffix, StringComparison.Ordinal))
                    stems.Add (name.Substring (0, name.Length - suffix.Length));

            foreach (var property in properties) {
                if (!property.CanWrite)
                    continue;
                foreach (var stem in stems)
                    if (string.Equals (property.Name, stem, StringComparison.OrdinalIgnoreCase))
                        return true;
            }
            return false;
        }

        internal static string NameOf (MemberInfo member)
        {
            return member.Name;
        }

        internal static Type TypeOf (MemberInfo member)
        {
            var field = member as FieldInfo;
            if (field != null)
                return field.FieldType;
            var property = member as PropertyInfo;
            return property != null ? property.PropertyType : null;
        }

        internal static object RawValue (MemberInfo member, object target)
        {
            var field = member as FieldInfo;
            if (field != null)
                return field.GetValue (target);
            var property = (PropertyInfo) member;
            return property.GetValue (target, null);
        }

        static void SetRaw (MemberInfo member, object target, object value)
        {
            var field = member as FieldInfo;
            if (field != null) {
                field.SetValue (target, value);
                return;
            }
            ((PropertyInfo) member).SetValue (target, value, null);
        }

        // ==================================================================
        // Classification, by shape
        // ==================================================================

        /// <summary>
        /// Which channel carries this member. Order matters: enum before numeric, because
        /// an enum IS an integral type and would otherwise be handed to a script as 2
        /// instead of "KEEP_SURFACE".
        /// </summary>
        internal static string ChannelOf (MemberInfo member, object target)
        {
            var declared = TypeOf (member);
            if (declared == null)
                return ChannelUnsupported;
            if (declared.IsEnum)
                return ChannelEnum;
            if (declared == typeof (bool))
                return ChannelFlag;
            if (IsPlainNumber (declared))
                return ChannelNumber;
            if (declared == typeof (string))
                return ChannelText;

            // Past this point the declared type may lie: OperationEccentricity declares
            // NewEcc as EditableDoubleMult and instantiates an EditableDouble. Behaviour is
            // the same, but classify on what is actually there when we can see it.
            Type actual = declared;
            if (target != null) {
                try {
                    var raw = RawValue (member, target);
                    if (raw != null)
                        actual = raw.GetType ();
                } catch (Exception) {
                    // Fall back to the declared type; a getter that throws is not a reason
                    // to hide the member from Describe.
                }
            }

            if (ValProperty (actual) != null)
                return ChannelNumber;               // EditableDouble / DoubleMult / Time / Int
            if (IsAngle (actual))
                return ChannelNumber;               // EditableAngle, via degrees/minutes/seconds
            if (IntListField (actual) != null)
                return ChannelList;                 // EditableIntList
            if (ReadOnlyValueProperty (actual) != null)
                return ChannelNumber;               // MovingAverage - readable, never writable
            return ChannelUnsupported;
        }

        static bool IsPlainNumber (Type type)
        {
            return type == typeof (double) || type == typeof (float)
                || type == typeof (int) || type == typeof (long)
                || type == typeof (uint) || type == typeof (short)
                || type == typeof (byte) || type == typeof (decimal);
        }

        /// <summary>The writable numeric <c>Val</c> of an Editable wrapper, or null.</summary>
        static PropertyInfo ValProperty (Type type)
        {
            if (type == null || type.IsPrimitive || type.IsEnum)
                return null;
            PropertyInfo val;
            try {
                val = type.GetProperty ("Val", ModRegistry.PubInst)
                      ?? type.GetProperty ("val", ModRegistry.PubInst);
            } catch (AmbiguousMatchException) {
                // A derived type new-hides Val. Take the most derived one.
                val = MostDerived (type, "Val");
            }
            if (val == null || !val.CanRead || !val.CanWrite)
                return null;
            return IsPlainNumber (val.PropertyType) ? val : null;
        }

        /// <summary>
        /// A read-only numeric <c>Value</c>, which is MovingAverage's shape.
        ///
        /// Deliberately never written. Its setter pushes one sample into a ring buffer, so
        /// a write would shift the average by a tenth rather than set it - the kind of
        /// member that appears to work and does something else.
        /// </summary>
        static PropertyInfo ReadOnlyValueProperty (Type type)
        {
            if (type == null || type.IsPrimitive || type.IsEnum)
                return null;
            PropertyInfo value;
            try {
                value = type.GetProperty ("Value", ModRegistry.PubInst);
            } catch (AmbiguousMatchException) {
                value = MostDerived (type, "Value");
            }
            if (value == null || !value.CanRead)
                return null;
            return IsPlainNumber (value.PropertyType) ? value : null;
        }

        static PropertyInfo MostDerived (Type type, string name)
        {
            const BindingFlags own = BindingFlags.Public | BindingFlags.Instance | BindingFlags.DeclaredOnly;
            for (var walk = type; walk != null; walk = walk.BaseType) {
                var found = walk.GetProperty (name, own);
                if (found != null)
                    return found;
            }
            return null;
        }

        /// <summary>
        /// EditableAngle's shape: three Editable components plus a sign, and no Val at all.
        /// Detected structurally so the type name never appears here.
        /// </summary>
        static bool IsAngle (Type type)
        {
            if (type == null || type.IsPrimitive || type.IsEnum)
                return false;
            return MechJebApi.FindField (type, "Degrees") != null
                && MechJebApi.FindField (type, "Minutes") != null
                && MechJebApi.FindField (type, "Seconds") != null
                && MechJebApi.FindField (type, "Negative") != null;
        }

        /// <summary>EditableIntList's shape: a Val FIELD holding a list, plus a Text property.</summary>
        static FieldInfo IntListField (Type type)
        {
            if (type == null || type.IsPrimitive || type.IsEnum)
                return null;
            var val = MechJebApi.FindField (type, "Val");
            if (val == null || !typeof (IEnumerable).IsAssignableFrom (val.FieldType)
                || val.FieldType == typeof (string))
                return null;
            return MechJebApi.FindProperty (type, "Text") != null ? val : null;
        }

        // ==================================================================
        // Writability
        // ==================================================================

        /// <summary>
        /// Whether a script can set this member.
        ///
        /// A readonly FIELD is not automatically a refusal. Most of MechJeb's readonly
        /// fields hold a mutable Editable wrapper - the reference is fixed, the value
        /// behind it is not, and mutating Val is both correct and what MechJeb's own GUI
        /// does. Only a readonly field holding a plain value is genuinely closed, and even
        /// there MechJeb writes some of them reflectively from its own toggle widgets.
        /// </summary>
        internal static bool Writable (MemberInfo member, object target)
        {
            var channel = ChannelOf (member, target);
            if (channel == ChannelUnsupported || channel == ChannelText)
                return false;

            var property = member as PropertyInfo;
            if (property != null)
                return property.CanWrite;

            var field = (FieldInfo) member;
            if (!field.IsInitOnly)
                return true;

            // readonly: writable only if the value behind it is a mutable wrapper.
            var actual = ActualType (member, target);
            return ValProperty (actual) != null || IsAngle (actual) || IntListField (actual) != null;
        }

        static Type ActualType (MemberInfo member, object target)
        {
            var declared = TypeOf (member);
            if (target == null)
                return declared;
            try {
                var raw = RawValue (member, target);
                return raw != null ? raw.GetType () : declared;
            } catch (Exception) {
                return declared;
            }
        }

        // ==================================================================
        // The number channel
        // ==================================================================

        internal static double ReadNumber (MemberInfo member, object target)
        {
            var raw = RawValue (member, target);
            if (raw == null)
                throw new InvalidOperationException (member.Name + " is null");
            if (IsPlainNumber (raw.GetType ()))
                return Convert.ToDouble (raw, CultureInfo.InvariantCulture);

            var val = ValProperty (raw.GetType ());
            if (val != null)
                return Convert.ToDouble (val.GetValue (raw, null), CultureInfo.InvariantCulture);

            if (IsAngle (raw.GetType ()))
                return ReadAngle (raw);

            var value = ReadOnlyValueProperty (raw.GetType ());
            if (value != null)
                return Convert.ToDouble (value.GetValue (raw, null), CultureInfo.InvariantCulture);

            throw new ArgumentException (
                member.Name + " is a " + raw.GetType ().Name + ", which is not a number");
        }

        internal static void WriteNumber (MemberInfo member, object target, double value)
        {
            var declared = TypeOf (member);

            if (IsPlainNumber (declared)) {
                // Convert.ChangeType to an integral type uses banker's rounding: 2.5 -> 2
                // but 3.5 -> 4. Round away from zero instead, which is what someone typing
                // a number into a settings box means.
                object converted = declared == typeof (double) || declared == typeof (float)
                                   || declared == typeof (decimal)
                    ? Convert.ChangeType (value, declared, CultureInfo.InvariantCulture)
                    : Convert.ChangeType (Math.Round (value, MidpointRounding.AwayFromZero),
                                          declared, CultureInfo.InvariantCulture);
                SetRaw (member, target, converted);
                return;
            }

            var raw = RawValue (member, target);
            if (raw == null)
                throw new InvalidOperationException (member.Name + " is null");

            var val = ValProperty (raw.GetType ());
            if (val != null) {
                object converted = val.PropertyType == typeof (double) || val.PropertyType == typeof (float)
                    ? Convert.ChangeType (value, val.PropertyType, CultureInfo.InvariantCulture)
                    : Convert.ChangeType (Math.Round (value, MidpointRounding.AwayFromZero),
                                          val.PropertyType, CultureInfo.InvariantCulture);
                val.SetValue (raw, converted, null);
                return;
            }

            if (IsAngle (raw.GetType ())) {
                WriteAngle (raw, value);
                return;
            }

            throw new ArgumentException (
                member.Name + " is a " + raw.GetType ().Name + ", which cannot take a number");
        }

        /// <summary>
        /// An EditableAngle carries degrees, minutes, seconds and a sign rather than one
        /// number, so it is composed and decomposed here rather than assigned.
        /// </summary>
        static double ReadAngle (object angle)
        {
            var type = angle.GetType ();
            var degrees = SubValue (angle, MechJebApi.FindField (type, "Degrees"));
            var minutes = SubValue (angle, MechJebApi.FindField (type, "Minutes"));
            var seconds = SubValue (angle, MechJebApi.FindField (type, "Seconds"));
            var negative = Convert.ToBoolean (MechJebApi.FindField (type, "Negative").GetValue (angle));
            var magnitude = degrees + minutes / 60.0 + seconds / 3600.0;
            return negative ? -magnitude : magnitude;
        }

        /// <summary>
        /// Written in place, component by component, reproducing MechJeb's own constructor
        /// including its clamp to +/-180. The components are readonly FIELDS holding
        /// mutable wrappers, so each one is set through its Val - the fields themselves are
        /// never reassigned.
        /// </summary>
        static void WriteAngle (object angle, double degreesValue)
        {
            var type = angle.GetType ();
            var clamped = degreesValue % 360.0;
            if (clamped > 180.0)
                clamped -= 360.0;
            if (clamped < -180.0)
                clamped += 360.0;

            var negative = clamped < 0;
            var magnitude = Math.Abs (clamped);

            var wholeDegrees = Math.Floor (magnitude);
            magnitude -= wholeDegrees;
            var wholeMinutes = Math.Floor (60.0 * magnitude);
            magnitude -= wholeMinutes / 60.0;
            var wholeSeconds = Math.Round (3600.0 * magnitude);

            SetSubValue (angle, MechJebApi.FindField (type, "Degrees"), wholeDegrees);
            SetSubValue (angle, MechJebApi.FindField (type, "Minutes"), wholeMinutes);
            SetSubValue (angle, MechJebApi.FindField (type, "Seconds"), wholeSeconds);
            MechJebApi.FindField (type, "Negative").SetValue (angle, negative);
        }

        static double SubValue (object owner, FieldInfo field)
        {
            if (field == null)
                return 0;
            var part = field.GetValue (owner);
            if (part == null)
                return 0;
            if (IsPlainNumber (part.GetType ()))
                return Convert.ToDouble (part, CultureInfo.InvariantCulture);
            var val = ValProperty (part.GetType ());
            return val == null ? 0 : Convert.ToDouble (val.GetValue (part, null), CultureInfo.InvariantCulture);
        }

        static void SetSubValue (object owner, FieldInfo field, double value)
        {
            if (field == null)
                return;
            var part = field.GetValue (owner);
            if (part == null)
                return;
            var val = ValProperty (part.GetType ());
            if (val != null)
                val.SetValue (part, Convert.ChangeType (value, val.PropertyType, CultureInfo.InvariantCulture), null);
        }

        // ==================================================================
        // The flag, enum, list and text channels
        // ==================================================================

        internal static bool ReadFlag (MemberInfo member, object target)
        {
            var raw = RawValue (member, target);
            if (!(raw is bool))
                throw new ArgumentException (member.Name + " is not a boolean");
            return (bool) raw;
        }

        internal static void WriteFlag (MemberInfo member, object target, bool value)
        {
            if (TypeOf (member) != typeof (bool))
                throw new ArgumentException (member.Name + " is not a boolean");
            SetRaw (member, target, value);
        }

        internal static string ReadEnum (MemberInfo member, object target)
        {
            var raw = RawValue (member, target);
            return raw == null ? string.Empty : raw.ToString ();
        }

        internal static void WriteEnum (MemberInfo member, object target, string name)
        {
            var type = TypeOf (member);
            if (type == null || !type.IsEnum)
                throw new ArgumentException (member.Name + " is not an enum");
            object parsed;
            try {
                parsed = Enum.Parse (type, name, true);
            } catch (Exception) {
                throw new ArgumentException (
                    "'" + name + "' is not a value of " + member.Name + " - this build offers "
                    + string.Join (", ", Enum.GetNames (type)));
            }
            SetRaw (member, target, parsed);
        }

        /// <summary>The legal values of an enum member, or an empty list if it is not one.</summary>
        internal static IList<string> EnumNames (MemberInfo member)
        {
            var type = TypeOf (member);
            if (type == null || !type.IsEnum)
                return new List<string> ();
            return new List<string> (Enum.GetNames (type));
        }

        internal static string ReadList (MemberInfo member, object target)
        {
            var raw = RawValue (member, target);
            if (raw == null)
                return string.Empty;
            var text = MechJebApi.FindProperty (raw.GetType (), "Text");
            if (text == null)
                throw new ArgumentException (member.Name + " is not a list");
            var value = text.GetValue (raw, null);
            return value == null ? string.Empty : value.ToString ();
        }

        /// <summary>
        /// Set an integer list from MechJeb's own text grammar - "1,2,3" or "1-3".
        ///
        /// Through the Text property, never by mutating the list behind it. BOTH the list
        /// and its text are persisted separately, so writing the list directly leaves the
        /// saved text stale and the next load quietly restores the old value.
        /// </summary>
        internal static void WriteList (MemberInfo member, object target, string value)
        {
            var raw = RawValue (member, target);
            if (raw == null)
                throw new InvalidOperationException (member.Name + " is null");
            var text = MechJebApi.FindProperty (raw.GetType (), "Text");
            if (text == null || !text.CanWrite)
                throw new ArgumentException (member.Name + " is not a writable list");
            text.SetValue (raw, value ?? string.Empty, null);
        }

        internal static string ReadText (MemberInfo member, object target)
        {
            var raw = RawValue (member, target);
            return raw == null ? string.Empty : raw.ToString ();
        }

        // ==================================================================
        // Description
        // ==================================================================

        /// <summary>
        /// One row per member: <c>name(TAB)channel(TAB)type(TAB)writable(TAB)persistence</c>.
        ///
        /// Generated from the live assembly, so it is right for the MechJeb actually in the
        /// player's GameData rather than the one this was written against. This is the
        /// member that turns a MechJeb rename into a one-string script edit instead of a
        /// bug report.
        /// </summary>
        internal static IList<string> Describe (object module)
        {
            var rows = new List<string> ();
            if (module == null)
                return rows;
            foreach (var member in MembersOf (module.GetType ())) {
                var channel = ChannelOf (member, module);
                var type = TypeOf (member);
                rows.Add (string.Join ("\t", new[] {
                    member.Name,
                    channel,
                    type == null ? "?" : type.Name,
                    Writable (member, module) ? "rw" : "r",
                    PersistenceOf (member)
                }));
            }
            rows.Sort (StringComparer.Ordinal);
            return rows;
        }

        /// <summary>
        /// Whether writing this member also writes the player's saved MechJeb config, and
        /// at what scope - the attribute is read by NAME so nothing here references MuMech.
        ///
        /// It matters because GLOBAL is not per-vessel or per-save: it is every vessel in
        /// every save on that install. A script that tunes an ascent setting for one
        /// launcher has quietly changed the default for all of them.
        /// </summary>
        internal static string PersistenceOf (MemberInfo member)
        {
            object[] attributes;
            try {
                attributes = member.GetCustomAttributes (false);
            } catch (Exception) {
                return "-";
            }
            foreach (var attribute in attributes) {
                if (attribute == null || attribute.GetType ().Name != "Persistent")
                    continue;
                var pass = MechJebApi.FindField (attribute.GetType (), "pass", "Pass");
                if (pass == null)
                    return "persistent";
                try {
                    var value = pass.GetValue (attribute);
                    return "persistent:" + (value == null ? "?" : value.ToString ());
                } catch (Exception) {
                    return "persistent";
                }
            }
            return "-";
        }

        // ==================================================================
        // Lookup
        // ==================================================================

        /// <summary>
        /// Find a member by name, case-insensitively, over the FILTERED list.
        ///
        /// Over the filtered list rather than through GetField/GetProperty with
        /// IgnoreCase, deliberately: a case-insensitive reflective lookup would happily
        /// return the backing field this file works to hide, and would trip over
        /// MechJeb's field-versus-nested-type collisions - SmartASS has a field `mode` and
        /// a nested type `Mode`.
        /// </summary>
        internal static MemberInfo Find (object module, string name, string what)
        {
            if (module == null)
                throw new InvalidOperationException ("module is null");
            if (string.IsNullOrEmpty (name))
                throw new ArgumentException ("member name is empty");

            var members = MembersOf (module.GetType ());
            foreach (var member in members)
                if (string.Equals (member.Name, name, StringComparison.Ordinal))
                    return member;
            foreach (var member in members)
                if (string.Equals (member.Name, name, StringComparison.OrdinalIgnoreCase))
                    return member;

            throw new ArgumentException (
                "no member called '" + name + "' on " + what + " in this MechJeb - "
                + "read describe_module(\"" + what + "\") for what it does have");
        }
    }
}
