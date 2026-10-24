using System;
using System.Collections.Generic;
using System.Globalization;

namespace KRPC.Bridge.Stock
{
    /// <summary>One clickable event of a part module (a button of the right-click menu), as the game holds it.</summary>
    public sealed class ModuleEventView
    {
        /// <summary>BaseEvent.name, the identifier.</summary>
        public string Name;
        /// <summary>BaseEvent.guiName, the text of the button.</summary>
        public string GuiName;
        /// <summary>guiActive in flight (guiActiveEditor in the editor).</summary>
        public bool Visible;
        /// <summary>BaseEvent.active.</summary>
        public bool Active;
        /// <summary>BaseEvent.Invoke.</summary>
        public Action Invoke;
    }

    /// <summary>One field of a part module, as the game holds it.</summary>
    public sealed class ModuleFieldView
    {
        /// <summary>BaseField.name, the identifier.</summary>
        public string Name;
        /// <summary>BaseField.guiName, the label.</summary>
        public string GuiName;
        /// <summary>guiActive in flight (guiActiveEditor in the editor).</summary>
        public bool Visible;
        /// <summary>The value as a string (invariant culture); read only when asked, and it may throw.</summary>
        public Func<string> Value;
    }

    /// <summary>
    /// The fields and events of part modules, filtered and encoded the way kRPC 0.6.0's
    /// <c>SpaceCenter.Module</c> gave them through its text members (<c>fields</c>, <c>events</c>,
    /// <c>has_field</c>, <c>get_field</c>, <c>get_field_by_id</c>, <c>trigger_event</c>), members that
    /// kRPC 0.7.0 removed. Pure: System only, so build/tests/stock tests it exactly as the plugin
    /// compiles it; PartModules hands it what the game holds.
    ///
    /// WHY. A client on kRPC 0.7 (or on 0.6 with the list path) reads a module through
    /// <c>event_list</c> and <c>field_list</c>: one RPC for the list, then one to three per member.
    /// the client's ship preparation (every module of every part, for the "Fuel Flow" field) went
    /// from under a second to 3 s on the heavy booster (a recorded flight), and the
    /// opening of the grid fins reads and triggers a module several times a second. Here one call
    /// answers for as many modules as the client asks for.
    ///
    /// THE RULES, those of Module.cs at kRPC v0.6.0 (decompiled; the same bodies in the 0.7.0
    /// <c>PartField</c> and <c>PartEvent</c> properties):
    ///   * a field is VISIBLE when <c>guiActive</c> (<c>guiActiveEditor</c> in the editor);
    ///   * an event is listed and triggered when it is visible AND <c>active</c>;
    ///   * fields = the visible ones as {guiName: value}, in the module's order; a repeated guiName made
    ///     the 0.6 <c>fields</c> raise <c>ArgumentException</c> (kept here: the pairs are all given,
    ///     the client raises);
    ///   * events = the guiName of the visible and active ones, in the module's order;
    ///   * trigger = the FIRST visible and active event of that guiName (<c>First</c>: no match raises
    ///     <c>InvalidOperationException</c>);
    ///   * get_field = the first VISIBLE field of that guiName; get_field_by_id = the first visible
    ///     field of that name (its identifier), else the first of all (a persistent field that is not
    ///     shown, TweakScale's currentScale, reads too).
    /// </summary>
    public static class ModuleStates
    {
        /// <summary>The count written for a section the game could not read; the next string is the message.</summary>
        public const string Unreadable = "-1";

        /// <summary>
        /// Appends the record of one module to <paramref name="output"/>:
        /// <c>moduleName, nEvents, event guiName x nEvents, nFields, (guiName, value) x nFields</c>.
        /// A count of "-1" stands for a section that could not be read and is followed by ONE string, the
        /// message (the other section of the module is still given): a field whose value throws spoils the
        /// fields of that module only, as it spoiled the 0.6 <c>fields</c> of that module only.
        /// Without <paramref name="withValues"/> each value is the empty string and none is read.
        /// </summary>
        public static void Append (IList<string> output, string moduleName,
                                   Func<IList<ModuleEventView>> readEvents,
                                   Func<IList<ModuleFieldView>> readFields, bool withValues)
        {
            if (output == null)
                throw new ArgumentNullException ("output");
            output.Add (moduleName ?? "");

            try {
                var events = readEvents ();
                var names = new List<string> ();
                for (int i = 0; events != null && i < events.Count; i++) {
                    var e = events [i];
                    if (e != null && e.Visible && e.Active)
                        names.Add (e.GuiName ?? "");
                }
                output.Add (Count (names.Count));
                for (int i = 0; i < names.Count; i++)
                    output.Add (names [i]);
            } catch (Exception ex) {
                output.Add (Unreadable);
                output.Add (Describe (ex));
            }

            try {
                var fields = readFields ();
                var pairs = new List<string> ();
                for (int i = 0; fields != null && i < fields.Count; i++) {
                    var f = fields [i];
                    if (f == null || !f.Visible)
                        continue;
                    pairs.Add (f.GuiName ?? "");
                    pairs.Add (withValues ? (f.Value () ?? "") : "");
                }
                output.Add (Count (pairs.Count / 2));
                for (int i = 0; i < pairs.Count; i++)
                    output.Add (pairs [i]);
            } catch (Exception ex) {
                output.Add (Unreadable);
                output.Add (Describe (ex));
            }
        }

        /// <summary>
        /// The record of a module the game could not give at all (its object is gone): the name unknown,
        /// both sections unreadable with the message.
        /// </summary>
        public static void AppendUnreadable (IList<string> output, Exception ex)
        {
            if (output == null)
                throw new ArgumentNullException ("output");
            output.Add ("");
            output.Add (Unreadable);
            output.Add (Describe (ex));
            output.Add (Unreadable);
            output.Add (Describe (ex));
        }

        /// <summary>The first event that is visible AND active with that guiName, or null.</summary>
        public static ModuleEventView FirstEvent (IList<ModuleEventView> events, string guiName)
        {
            if (guiName == null)
                throw new ArgumentNullException ("guiName");
            for (int i = 0; events != null && i < events.Count; i++) {
                var e = events [i];
                if (e != null && e.Visible && e.Active && string.Equals (e.GuiName, guiName, StringComparison.Ordinal))
                    return e;
            }
            return null;
        }

        /// <summary>The first VISIBLE field with that guiName, or null.</summary>
        public static ModuleFieldView FirstVisibleField (IList<ModuleFieldView> fields, string guiName)
        {
            if (guiName == null)
                throw new ArgumentNullException ("guiName");
            for (int i = 0; fields != null && i < fields.Count; i++) {
                var f = fields [i];
                if (f != null && f.Visible && string.Equals (f.GuiName, guiName, StringComparison.Ordinal))
                    return f;
            }
            return null;
        }

        /// <summary>The first visible field with that name (identifier), else the first of all, or null.</summary>
        public static ModuleFieldView FieldById (IList<ModuleFieldView> fields, string id)
        {
            if (id == null)
                throw new ArgumentNullException ("id");
            ModuleFieldView any = null;
            for (int i = 0; fields != null && i < fields.Count; i++) {
                var f = fields [i];
                if (f == null || !string.Equals (f.Name, id, StringComparison.Ordinal))
                    continue;
                if (f.Visible)
                    return f;
                if (any == null)
                    any = f;
            }
            return any;
        }

        /// <summary>The message the 0.6 first() raised: LINQ's, which the client sees as a RuntimeError.</summary>
        public const string NoMatch = "Sequence contains no matching element";

        static string Count (int n)
        {
            return n.ToString (CultureInfo.InvariantCulture);
        }

        static string Describe (Exception ex)
        {
            return ex.GetType ().Name + ": " + ex.Message;
        }
    }
}
