// -----------------------------------------------------------------------------------
// conn.stock.module_states_v1 / module_trigger_event_v1 / module_field_v1 / module_field_by_id_v1:
// pure tests of the filtering and the encoding of ModuleStates.cs. See StockTests.csproj.
//
// The module below is the one the client's own tests use on the Python side
// (the client, _module_fin and _module_moteur): a grid fin's ModuleAnimateGeneric
// with a hidden event, an inactive one and a repeated guiName, and an engine module with a hidden
// field of the same name, a repeated identifier and a field that is not a text. The expected values
// are what kRPC 0.6.0's Module.events, Module.fields, trigger_event, get_field and get_field_by_id
// returned on it (Module.cs v0.6.0, decompiled).
// -----------------------------------------------------------------------------------

using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Stock
{
    public static class ModuleStatesTests
    {
        static Action<bool, string> check;

        static ModuleEventView Ev (string name, string gui, bool visible = true, bool active = true)
        {
            return new ModuleEventView { Name = name, GuiName = gui, Visible = visible, Active = active };
        }

        static int reads;

        static ModuleFieldView Fd (string name, string gui, string value, bool visible = true)
        {
            return new ModuleFieldView { Name = name, GuiName = gui, Visible = visible,
                                         Value = () => { reads++; return value; } };
        }

        static IList<ModuleEventView> FinEvents (bool open)
        {
            return new List<ModuleEventView> {
                Ev ("ToggleHidden", "Retract Fins", visible: false),
                Ev ("Toggle", open ? "Retract Fins" : "Deploy Fins"),
                Ev ("Toggle2", "Retract Fins", active: false),
                Ev ("Toggle3", "Retract Fins"),
            };
        }

        static IList<ModuleEventView> EngineEvents ()
        {
            return new List<ModuleEventView> {
                Ev ("Activate", "Activate Engine", active: false),
                Ev ("Shutdown", "Shutdown Engine"),
            };
        }

        static IList<ModuleFieldView> EngineFields ()
        {
            return new List<ModuleFieldView> {
                Fd ("fuelFlowGui", "Fuel Flow", "0.0000"),
                Fd ("thrustPercentage", "Thrust Limiter", "100"),
                Fd ("minThrustHidden", "Min Thrust", "999 kN", visible: false),
                Fd ("minThrust", "Min Thrust", "1,250.5 kN"),
                Fd ("currentScale", "Scale", "1.5", visible: false),
                Fd ("currentScale", "Scale vue", "2.5", visible: false),
                Fd ("note", "Note", "texte"),
            };
        }

        static string J (IList<string> l)
        {
            return string.Join ("|", new List<string> (l).ToArray ());
        }

        static List<string> Record (string name, IList<ModuleEventView> events, IList<ModuleFieldView> fields,
                                    bool values)
        {
            var o = new List<string> ();
            ModuleStates.Append (o, name, () => events, () => fields, values);
            return o;
        }

        /// <summary>The reader the Python side mirrors: n records, each name, events, fields.</summary>
        sealed class Parsed
        {
            public string Name;
            public List<string> Events;
            public string EventsError;
            public List<KeyValuePair<string, string>> Fields;
            public string FieldsError;
        }

        static List<Parsed> Parse (IList<string> flat, int n)
        {
            var result = new List<Parsed> ();
            int i = 0;
            for (int m = 0; m < n; m++) {
                var p = new Parsed { Name = flat [i++] };
                int ne = int.Parse (flat [i++]);
                if (ne < 0) {
                    p.EventsError = flat [i++];
                } else {
                    p.Events = new List<string> ();
                    for (int k = 0; k < ne; k++)
                        p.Events.Add (flat [i++]);
                }
                int nf = int.Parse (flat [i++]);
                if (nf < 0) {
                    p.FieldsError = flat [i++];
                } else {
                    p.Fields = new List<KeyValuePair<string, string>> ();
                    for (int k = 0; k < nf; k++) {
                        p.Fields.Add (new KeyValuePair<string, string> (flat [i], flat [i + 1]));
                        i += 2;
                    }
                }
                result.Add (p);
            }
            check (i == flat.Count, "the records use every string of the list (" + i + " of " + flat.Count + ")");
            return result;
        }

        public static void Run (Action<bool, string> checker)
        {
            check = checker;
            TestEventsAreVisibleAndActive ();
            TestFieldsAreVisibleWithTheirValues ();
            TestValuesAreNotReadUnlessAsked ();
            TestARepeatedGuiNameIsKept ();
            TestAnUnreadableSectionIsAloneUnreadable ();
            TestAGoneModule ();
            TestSeveralModulesInOrder ();
            TestTheEmptyModule ();
            TestTriggerPicksTheFirstVisibleAndActive ();
            TestGetFieldIsTheFirstVisible ();
            TestGetFieldByIdIsVisibleFirstThenAny ();
            TestNullArguments ();
        }

        /// <summary>Module.events of 0.6: the guiName of the events visible AND active, in order.</summary>
        static void TestEventsAreVisibleAndActive ()
        {
            var r = Parse (Record ("ModuleAnimateGeneric", FinEvents (true), new List<ModuleFieldView> (), false), 1) [0];
            check (r.Name == "ModuleAnimateGeneric", "the module name leads the record");
            check (J (r.Events) == "Retract Fins|Retract Fins",
                   "the hidden and the inactive events are left out, the others kept in order, a repeat too: "
                   + J (r.Events));
            var closed = Parse (Record ("ModuleAnimateGeneric", FinEvents (false), new List<ModuleFieldView> (), false), 1) [0];
            check (J (closed.Events) == "Deploy Fins|Retract Fins", "a closed fin: " + J (closed.Events));
            var engine = Parse (Record ("ModuleEnginesFX", EngineEvents (), new List<ModuleFieldView> (), false), 1) [0];
            check (J (engine.Events) == "Shutdown Engine", "an inactive Activate is left out: " + J (engine.Events));
            var withNull = new List<ModuleEventView> { null, Ev ("a", "A") };
            check (J (Parse (Record ("M", withNull, new List<ModuleFieldView> (), false), 1) [0].Events) == "A",
                   "a null entry is skipped");
        }

        /// <summary>Module.fields of 0.6: {guiName: value} of the visible fields, in order.</summary>
        static void TestFieldsAreVisibleWithTheirValues ()
        {
            var r = Parse (Record ("ModuleEnginesFX", EngineEvents (), EngineFields (), true), 1) [0];
            var s = new List<string> ();
            foreach (var kv in r.Fields)
                s.Add (kv.Key + "=" + kv.Value);
            check (J (s) == "Fuel Flow=0.0000|Thrust Limiter=100|Min Thrust=1,250.5 kN|Note=texte",
                   "the visible fields with their values, the hidden ones left out: " + J (s));
        }

        /// <summary>Without values no value is read (the question "is there a field of that name").</summary>
        static void TestValuesAreNotReadUnlessAsked ()
        {
            reads = 0;
            var r = Parse (Record ("ModuleEnginesFX", EngineEvents (), EngineFields (), false), 1) [0];
            check (reads == 0, "no value read when none is asked: " + reads);
            check (r.Fields.Count == 4 && r.Fields [0].Key == "Fuel Flow" && r.Fields [0].Value == "",
                   "the names are there, the values empty");
            reads = 0;
            Record ("ModuleEnginesFX", EngineEvents (), EngineFields (), true);
            check (reads == 4, "one read per visible field when asked: " + reads);
            var nullValue = new List<ModuleFieldView> { new ModuleFieldView { Name = "n", GuiName = "N", Visible = true, Value = () => null } };
            check (Parse (Record ("M", EngineEvents (), nullValue, true), 1) [0].Fields [0].Value == "",
                   "a null value is the empty string");
        }

        /// <summary>The 0.6 dictionary raised on a repeated guiName; here the pairs are all given and the
        /// client raises: nothing is lost on the way.</summary>
        static void TestARepeatedGuiNameIsKept ()
        {
            var two = new List<ModuleFieldView> { Fd ("a", "X", "1"), Fd ("b", "X", "2") };
            var r = Parse (Record ("M", new List<ModuleEventView> (), two, true), 1) [0];
            check (r.Fields.Count == 2 && r.Fields [0].Value == "1" && r.Fields [1].Value == "2",
                   "both fields of the repeated guiName are given");
        }

        /// <summary>A section that throws is unreadable alone: the 0.6 fields of one module spoiled only that
        /// module, and its events were another call.</summary>
        static void TestAnUnreadableSectionIsAloneUnreadable ()
        {
            var bad = new List<ModuleFieldView> {
                Fd ("a", "A", "1"),
                new ModuleFieldView { Name = "b", GuiName = "B", Visible = true,
                                      Value = () => { throw new InvalidOperationException ("boom"); } },
            };
            var o = new List<string> ();
            ModuleStates.Append (o, "M", () => EngineEvents (), () => bad, true);
            var r = Parse (o, 1) [0];
            check (r.FieldsError == "InvalidOperationException: boom", "the fields' message: " + r.FieldsError);
            check (r.Fields == null && J (r.Events) == "Shutdown Engine", "its events are still given");
            var o2 = new List<string> ();
            ModuleStates.Append (o2, "M", () => EngineEvents (), () => bad, false);
            var r2 = Parse (o2, 1) [0];
            check (r2.FieldsError == null && r2.Fields.Count == 2, "without values the throwing value is never read");
            var o3 = new List<string> ();
            ModuleStates.Append (o3, "M", () => { throw new NullReferenceException ("gone"); }, () => EngineFields (), false);
            var r3 = Parse (o3, 1) [0];
            check (r3.EventsError == "NullReferenceException: gone" && r3.Events == null && r3.Fields.Count == 4,
                   "an events section that throws leaves the fields alone");
        }

        static void TestAGoneModule ()
        {
            var o = new List<string> ();
            ModuleStates.AppendUnreadable (o, new InvalidOperationException ("destroyed"));
            var r = Parse (o, 1) [0];
            check (r.Name == "" && r.EventsError == "InvalidOperationException: destroyed"
                   && r.FieldsError == "InvalidOperationException: destroyed",
                   "a gone module: no name, both sections unreadable with the message");
        }

        static void TestSeveralModulesInOrder ()
        {
            var o = new List<string> ();
            ModuleStates.Append (o, "ModuleControlSurface", () => new List<ModuleEventView> (), () => new List<ModuleFieldView> (), false);
            ModuleStates.Append (o, "ModuleAnimateGeneric", () => FinEvents (false), () => new List<ModuleFieldView> (), false);
            ModuleStates.Append (o, "ModuleEnginesFX", () => EngineEvents (), () => EngineFields (), true);
            var p = Parse (o, 3);
            check (p [0].Name == "ModuleControlSurface" && p [0].Events.Count == 0 && p [0].Fields.Count == 0,
                   "the first record is read from its own place");
            check (J (p [1].Events) == "Deploy Fins|Retract Fins", "the second");
            check (p [2].Name == "ModuleEnginesFX" && p [2].Fields.Count == 4, "the third");
        }

        static void TestTheEmptyModule ()
        {
            var o = Record ("M", new List<ModuleEventView> (), new List<ModuleFieldView> (), true);
            check (J (o) == "M|0|0", "a module with nothing: name, 0, 0 (" + J (o) + ")");
            var nothing = new List<string> ();
            ModuleStates.Append (nothing, null, () => null, () => null, false);
            check (J (nothing) == "|0|0", "null readers and a null name are empty, not a failure");
        }

        /// <summary>trigger_event of 0.6: First(visible and active and guiName ==).Invoke(): the hidden
        /// "Retract Fins" before it and the inactive one after it are not the one.</summary>
        static void TestTriggerPicksTheFirstVisibleAndActive ()
        {
            var events = FinEvents (true);
            var e = ModuleStates.FirstEvent (events, "Retract Fins");
            check (e != null && e.Name == "Toggle", "the first visible and active, not the hidden one before it: "
                   + (e == null ? "null" : e.Name));
            check (ModuleStates.FirstEvent (events, "Deploy Fins") == null, "no such event: null (the procedure raises)");
            check (ModuleStates.FirstEvent (EngineEvents (), "Activate Engine") == null, "an inactive event: none");
            check (ModuleStates.FirstEvent (events, "retract fins") == null, "the name is compared as it is, case included");
            check (ModuleStates.FirstEvent (null, "x") == null, "no list: none");
            var third = FinEvents (false);
            check (ModuleStates.FirstEvent (third, "Retract Fins").Name == "Toggle3",
                   "closed fin: the first visible and active Retract Fins is the fourth entry");
        }

        /// <summary>get_field of 0.6: the first VISIBLE field of that guiName (a hidden "Min Thrust" before it
        /// is not the one).</summary>
        static void TestGetFieldIsTheFirstVisible ()
        {
            var fields = EngineFields ();
            var f = ModuleStates.FirstVisibleField (fields, "Min Thrust");
            check (f != null && f.Name == "minThrust" && f.Value () == "1,250.5 kN", "visible one, not the hidden one");
            check (ModuleStates.FirstVisibleField (fields, "Scale") == null, "a hidden field is not found by guiName");
            check (ModuleStates.FirstVisibleField (fields, "Rien") == null, "no such field: null");
        }

        /// <summary>get_field_by_id of 0.6: visible first, then any (TweakScale's hidden currentScale).</summary>
        static void TestGetFieldByIdIsVisibleFirstThenAny ()
        {
            var fields = EngineFields ();
            check (ModuleStates.FieldById (fields, "minThrust").Value () == "1,250.5 kN", "a visible field by id");
            check (ModuleStates.FieldById (fields, "currentScale").Value () == "1.5",
                   "a hidden one is found, the first of that id");
            var late = new List<ModuleFieldView> { Fd ("v", "A", "hidden", visible: false), Fd ("v", "B", "shown") };
            check (ModuleStates.FieldById (late, "v").Value () == "shown", "a visible one after a hidden one wins");
            check (ModuleStates.FieldById (fields, "rien") == null, "no such id: null");
        }

        static void TestNullArguments ()
        {
            bool a = false, b = false, c = false;
            try { ModuleStates.FirstEvent (FinEvents (true), null); } catch (ArgumentNullException) { a = true; }
            try { ModuleStates.FirstVisibleField (EngineFields (), null); } catch (ArgumentNullException) { b = true; }
            try { ModuleStates.FieldById (EngineFields (), null); } catch (ArgumentNullException) { c = true; }
            check (a && b && c, "a null name is refused, not matched");
        }
    }
}
