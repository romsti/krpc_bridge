using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.Reflection;
using System.Runtime.ExceptionServices;

using SCModule = KRPC.SpaceCenter.Services.Parts.Module;

namespace KRPC.Bridge.Stock
{
    /// <summary>
    /// The text members of kRPC's <c>SpaceCenter.Module</c> (fields, events, has_field, get_field,
    /// get_field_by_id, trigger_event), which kRPC 0.7.0 removed, served from KSP's own PartModule:
    /// the same answers as 0.6.0 and the same code for both versions of kRPC. The rules, and their
    /// source, are in <see cref="ModuleStates"/> (pure, tested); this file only reads the game.
    ///
    /// THE PART MODULE. A kRPC <c>Module</c> keeps its PartModule private: <c>internal PartModule
    /// InternalModule</c> in 0.6.0 (over a private field <c>module</c>) and in 0.7.0 (resolved on the part
    /// at each read: by name, occurrence and PersistentId, <c>ModuleRef</c>). It is read by reflection, once
    /// resolved, so the procedures take the kRPC <c>Module</c> objects the client already holds instead of
    /// a part identifier and an index that the client would have to rediscover with two RPCs per module.
    /// If neither member exists (a kRPC that renamed it) every procedure here refuses with
    /// "[MODULE_ACCESS]" and the client goes back to the lists it used before: nothing else breaks.
    /// </summary>
    public static class PartModules
    {
        /// <summary>The prefix of the refusal when a kRPC <c>Module</c> cannot be opened.</summary>
        public const string AccessError = "[MODULE_ACCESS]";

        static bool resolved;
        static PropertyInfo internalModule;
        static FieldInfo moduleField;

        static PartModule Resolve (SCModule module)
        {
            if (module == null)
                throw new ArgumentNullException ("module");
            if (!resolved) {
                const BindingFlags any = BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic;
                var type = typeof(SCModule);
                internalModule = type.GetProperty ("InternalModule", any);
                if (internalModule == null)
                    moduleField = type.GetField ("module", any);
                resolved = true;
            }
            object found;
            if (internalModule != null) {
                try {
                    found = internalModule.GetValue (module, null);
                } catch (TargetInvocationException e) {
                    // The module is gone (0.7 raises ObjectDestroyedException): that is the module's own
                    // error, not an access problem.
                    if (e.InnerException == null)
                        throw;
                    ExceptionDispatchInfo.Capture (e.InnerException).Throw ();
                    throw;
                }
            } else if (moduleField != null) {
                found = moduleField.GetValue (module);
            } else {
                throw new InvalidOperationException (AccessError
                    + " kRPC's SpaceCenter Module has neither an InternalModule property nor a module field");
            }
            var pm = found as PartModule;
            if (pm == null)
                throw new InvalidOperationException (AccessError + " the part module of a kRPC Module is not readable");
            return pm;
        }

        static bool InEditor {
            get { return HighLogic.LoadedSceneIsEditor; }
        }

        /// <summary>The events of the module as the game holds them (nulls left out), in its order.</summary>
        static IList<ModuleEventView> EventsOf (PartModule pm)
        {
            var list = new List<ModuleEventView> ();
            foreach (BaseEvent e in (IEnumerable<BaseEvent>)pm.Events) {
                if (e == null)
                    continue;
                var ev = e;
                list.Add (new ModuleEventView {
                    Name = ev.name,
                    GuiName = ev.guiName,
                    Visible = InEditor ? ev.guiActiveEditor : ev.guiActive,
                    Active = ev.active,
                    Invoke = () => ev.Invoke ()
                });
            }
            return list;
        }

        /// <summary>The fields of the module as the game holds them (nulls left out), in its order.</summary>
        static IList<ModuleFieldView> FieldsOf (PartModule pm)
        {
            var list = new List<ModuleFieldView> ();
            foreach (BaseField f in (IEnumerable)pm.Fields) {
                if (f == null)
                    continue;
                var field = f;
                var typed = (BaseField<KSPField>)(object)field;
                list.Add (new ModuleFieldView {
                    Name = typed.name,
                    GuiName = typed.guiName,
                    Visible = InEditor ? field.guiActiveEditor : field.guiActive,
                    // kRPC's BaseField.GetValueString: Convert.ToString in the invariant culture
                    Value = () => Convert.ToString (typed.GetValue ((object)pm), CultureInfo.InvariantCulture)
                });
            }
            return list;
        }

        /// <summary>
        /// One record per module of <paramref name="modules"/>, in order (see ModuleStates.Append). A module
        /// whose object is gone gives an unreadable record, not a failure of the call; a kRPC that cannot be
        /// opened at all fails the call with the "[MODULE_ACCESS]" refusal.
        /// </summary>
        public static IList<string> States (IList<SCModule> modules, bool withValues)
        {
            if (modules == null)
                throw new ArgumentNullException ("modules");
            var output = new List<string> ();
            for (int i = 0; i < modules.Count; i++) {
                PartModule pm;
                try {
                    pm = Resolve (modules [i]);
                } catch (InvalidOperationException e) when (e.Message.StartsWith (AccessError, StringComparison.Ordinal)) {
                    throw;
                } catch (Exception e) {
                    ModuleStates.AppendUnreadable (output, e);
                    continue;
                }
                ModuleStates.Append (output, pm.moduleName, () => EventsOf (pm), () => FieldsOf (pm), withValues);
            }
            return output;
        }

        /// <summary>
        /// Clicks the first visible and active event of that guiName; raises InvalidOperationException
        /// ("Sequence contains no matching element") when there is none, as 0.6.0's TriggerEvent did.
        /// </summary>
        public static void TriggerEvent (SCModule module, string guiName)
        {
            var e = ModuleStates.FirstEvent (EventsOf (Resolve (module)), guiName);
            if (e == null)
                throw new InvalidOperationException (ModuleStates.NoMatch);
            e.Invoke ();
        }

        /// <summary>
        /// The value of the first visible field of that guiName (0.6.0's GetField); raises
        /// InvalidOperationException when there is none.
        /// </summary>
        public static string Field (SCModule module, string guiName)
        {
            var f = ModuleStates.FirstVisibleField (FieldsOf (Resolve (module)), guiName);
            if (f == null)
                throw new InvalidOperationException (ModuleStates.NoMatch);
            return f.Value ();
        }

        /// <summary>
        /// The value of the first visible field of that name, else of the first of all (0.6.0's
        /// GetFieldById); raises InvalidOperationException when there is none.
        /// </summary>
        public static string FieldById (SCModule module, string id)
        {
            var f = ModuleStates.FieldById (FieldsOf (Resolve (module)), id);
            if (f == null)
                throw new InvalidOperationException (ModuleStates.NoMatch);
            return f.Value ();
        }
    }
}
