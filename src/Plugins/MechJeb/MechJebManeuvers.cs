using System;
using System.Collections;
using System.Collections.Generic;
using System.Reflection;
using KRPC.Bridge.Core;
using UnityEngine;

namespace KRPC.Bridge.MechJeb
{
    /// <summary>
    /// MechJeb's maneuver planner, exposed without a single vector crossing the wire.
    ///
    /// THE PROBLEM. MechJeb computes transfers with OrbitalManeuverCalculator, whose
    /// statics return Vector3d and C# ValueTuple and use out parameters. None of those is
    /// a legal kRPC type - and a malformed signature does not fail politely, it disables
    /// the entire kRPC server for that install, every service, including stock SpaceCenter.
    /// So marshalling MechJeb's answers directly is not merely awkward, it is forbidden.
    ///
    /// THE WAY ROUND, which turns out to be better than a workaround. An Operation returns
    /// a plan; it does not place anything. MechJeb's own extension then turns that plan
    /// into a real ManeuverNode on the vessel's patched-conic solver - which is exactly the
    /// list stock kRPC exposes as vessel.control.nodes. So this file drives MechJeb to
    /// place the node and then gets out of the way: the script reads a genuine kRPC Node,
    /// with prograde, normal, radial and UT, and executes or deletes it with tooling that
    /// already exists and that nobody has to learn.
    ///
    /// The delta-v never becomes our problem. It comes out of a field as a BOXED Vector3d
    /// in an object, and goes straight back into a method call as an object. This file
    /// never names the type, never unpacks it, and never lets it near a signature.
    ///
    /// WHAT IS NOT HERE. The interplanetary porkchop transfer cannot be driven headlessly:
    /// its solver is created by the GUI repaint path, and calling it cold returns "Started
    /// computation" forever. It is excluded and says so, rather than hanging.
    /// </summary>
    internal static class MechJebManeuvers
    {
        static Type operationType;
        static MethodInfo getAvailableOperations;   // static Operation[] GetAvailableOperations()
        static MethodInfo makeNodes;                // Operation.MakeNodes(Orbit, double, TargetController)
        static MethodInfo getErrorMessage;          // Operation.GetErrorMessage()
        static MethodInfo getOperationName;         // Operation.GetName()
        static FieldInfo parametersDv;              // ManeuverParameters.dV   (Vector3d, kept boxed)
        static FieldInfo parametersUt;              // ManeuverParameters.UT
        static MethodInfo placeManeuverNode;        // VesselExtensions.PlaceManeuverNode(Vessel, Orbit, Vector3d, double)
        static MethodInfo patchedConicsUnlocked;    // VesselExtensions.patchedConicsUnlocked(Vessel)

        /// <summary>
        /// One instance per operation, kept for the session.
        ///
        /// Not an optimisation. GetAvailableOperations hands back FRESH instances on every
        /// call, and every parameter a script sets - the target apoapsis, the resonance
        /// ratio - lives as a field on the instance. Fetching them again between setting a
        /// parameter and planning the node would silently discard the parameter. MechJeb's
        /// own window keeps a single static array for exactly this reason, with the comment
        /// "Keep all Operation objects so parameters are saved".
        /// </summary>
        static Dictionary<string, object> operations;

        static string lastWarning = string.Empty;

        internal static string Report { get; private set; } = "not resolved";
        internal static bool Resolved { get; private set; }

        internal static void Resolve (Assembly assembly, Type extensionsType)
        {
            operationType = null;
            operations = null;
            Resolved = false;

            operationType = ModRegistry.FindType (assembly, "MuMech.Operation")
                            ?? ModRegistry.FindType (assembly, "Operation");
            var parametersType = ModRegistry.FindType (assembly, "MuMech.ManeuverParameters")
                                 ?? ModRegistry.FindType (assembly, "ManeuverParameters");

            if (operationType != null) {
                getAvailableOperations = operationType.GetMethod ("GetAvailableOperations",
                                                                  ModRegistry.PubStatic, null,
                                                                  Type.EmptyTypes, null);
                getErrorMessage = operationType.GetMethod ("GetErrorMessage", ModRegistry.PubInst,
                                                           null, Type.EmptyTypes, null);
                getOperationName = operationType.GetMethod ("GetName", ModRegistry.PubInst,
                                                            null, Type.EmptyTypes, null);
                foreach (var method in operationType.GetMethods (ModRegistry.PubInst)) {
                    if (method.Name != "MakeNodes")
                        continue;
                    if (method.GetParameters ().Length == 3) {
                        makeNodes = method;
                        break;
                    }
                }
            }

            if (parametersType != null) {
                parametersDv = MechJebApi.FindField (parametersType, "dV", "DV", "dv");
                parametersUt = MechJebApi.FindField (parametersType, "UT", "ut");
            }

            if (extensionsType != null) {
                foreach (var method in extensionsType.GetMethods (ModRegistry.PubStatic)) {
                    var parameters = method.GetParameters ();
                    if (method.Name == "PlaceManeuverNode" && parameters.Length == 4)
                        placeManeuverNode = method;
                    else if (string.Equals (method.Name, "patchedConicsUnlocked",
                                            StringComparison.OrdinalIgnoreCase)
                             && parameters.Length == 1)
                        patchedConicsUnlocked = method;
                }
            }

            Resolved = operationType != null && makeNodes != null && placeManeuverNode != null
                       && parametersDv != null && parametersUt != null;

            Report = string.Format (
                "Operation={0} GetAvailableOperations={1} MakeNodes={2} ManeuverParameters[dV={3} UT={4}] "
                + "PlaceManeuverNode={5} patchedConicsUnlocked={6}",
                operationType != null, getAvailableOperations != null, makeNodes != null,
                parametersDv != null, parametersUt != null,
                placeManeuverNode != null, patchedConicsUnlocked != null);
        }

        internal static void Require ()
        {
            if (!Resolved)
                throw new InvalidOperationException (
                    "MechJeb's maneuver planner did not resolve in this build - " + Report);
        }

        // ==================================================================
        // The operation catalogue
        // ==================================================================

        static Dictionary<string, object> All ()
        {
            if (operations != null)
                return operations;
            Require ();
            MechJebApi.RequireMember (getAvailableOperations, "Operation.GetAvailableOperations");

            operations = new Dictionary<string, object> (StringComparer.OrdinalIgnoreCase);
            var array = getAvailableOperations.Invoke (null, null) as IEnumerable;
            if (array == null)
                return operations;
            foreach (var operation in array) {
                if (operation == null)
                    continue;
                operations [operation.GetType ().Name] = operation;
            }
            return operations;
        }

        /// <summary>
        /// The operations this MechJeb offers, by CLASS name.
        ///
        /// Class names rather than the labels MechJeb shows the player, because the labels
        /// are localised - they change with the game's language - one of them is hardcoded
        /// English while the rest are not, and another has a stray quotation mark in the
        /// English file. A script keyed on any of that would break on a French install.
        /// </summary>
        internal static IList<string> Names ()
        {
            var names = new List<string> (All ().Keys);
            names.Sort (StringComparer.Ordinal);
            return names;
        }

        internal static object Get (string name)
        {
            if (string.IsNullOrEmpty (name))
                throw new ArgumentException ("operation name is empty");
            object operation;
            if (All ().TryGetValue (name, out operation))
                return operation;
            throw new ArgumentException (
                "MechJeb has no maneuver operation called '" + name + "' - "
                + "maneuver_operations() lists what this build offers");
        }

        internal static string DisplayName (object operation)
        {
            if (getOperationName == null)
                return operation.GetType ().Name;
            try {
                var name = getOperationName.Invoke (operation, null);
                return name == null ? operation.GetType ().Name : name.ToString ();
            } catch (Exception) {
                return operation.GetType ().Name;
            }
        }

        // ==================================================================
        // The burn-time selector
        // ==================================================================

        /// <summary>
        /// An operation's TimeSelector, or null when it computes its own burn time.
        ///
        /// Held in a PRIVATE field, and on all but one operation a private STATIC one - so
        /// the chosen time reference is shared between every instance of that operation,
        /// including the one MechJeb's own window is using. A script that selects "at the
        /// next apoapsis" changes what the player sees in the Maneuver Planner, and a
        /// player fiddling with that window changes what the script gets. That coupling is
        /// MechJeb's design; it is documented rather than fought.
        /// </summary>
        static object TimeSelectorOf (object operation)
        {
            const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic
                                     | BindingFlags.Instance | BindingFlags.Static;
            for (var type = operation.GetType (); type != null; type = type.BaseType) {
                foreach (var field in type.GetFields (any | BindingFlags.DeclaredOnly)) {
                    if (field.Name.IndexOf ("timeSelector", StringComparison.OrdinalIgnoreCase) < 0)
                        continue;
                    return field.IsStatic ? field.GetValue (null) : field.GetValue (operation);
                }
            }
            return null;
        }

        static Array AllowedReferences (object selector)
        {
            if (selector == null)
                return null;
            const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
            foreach (var field in selector.GetType ().GetFields (any))
                if (field.Name.IndexOf ("allowedTimeRef", StringComparison.OrdinalIgnoreCase) >= 0)
                    return field.GetValue (selector) as Array;
            return null;
        }

        /// <summary>
        /// The burn times this operation will accept, as enum names. Empty means the
        /// operation works out its own timing and there is nothing to choose.
        /// </summary>
        internal static IList<string> TimeReferences (object operation)
        {
            var result = new List<string> ();
            var allowed = AllowedReferences (TimeSelectorOf (operation));
            if (allowed == null)
                return result;
            foreach (var value in allowed)
                if (value != null)
                    result.Add (value.ToString ());
            return result;
        }

        /// <summary>
        /// Choose the burn time. The selector stores an INDEX into its own whitelist rather
        /// than the enum value, so the whitelist is also the validation: a reference the
        /// operation does not allow simply is not in it.
        /// </summary>
        internal static void SetTimeReference (object operation, string reference)
        {
            var selector = TimeSelectorOf (operation);
            if (selector == null)
                throw new InvalidOperationException (
                    DisplayName (operation) + " computes its own burn time - there is nothing to select");

            var allowed = AllowedReferences (selector);
            if (allowed == null)
                throw new InvalidOperationException ("could not read this operation's allowed burn times");

            for (var i = 0; i < allowed.Length; i++) {
                var value = allowed.GetValue (i);
                if (value == null || !string.Equals (value.ToString (), reference, StringComparison.OrdinalIgnoreCase))
                    continue;
                var current = MechJebApi.FindField (selector.GetType (), "_currentTimeRef", "currentTimeRef");
                if (current == null) {
                    const BindingFlags any = BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance;
                    foreach (var field in selector.GetType ().GetFields (any))
                        if (field.Name.IndexOf ("currentTimeRef", StringComparison.OrdinalIgnoreCase) >= 0) {
                            current = field;
                            break;
                        }
                }
                if (current == null)
                    throw new InvalidOperationException ("could not find the burn-time index field");
                current.SetValue (selector, i);
                return;
            }

            var names = new List<string> ();
            foreach (var value in allowed)
                names.Add (value == null ? "?" : value.ToString ());
            throw new ArgumentException (
                "'" + reference + "' is not a burn time " + DisplayName (operation)
                + " accepts - it offers " + string.Join (", ", names.ToArray ()));
        }

        /// <summary>
        /// The selector's own two parameters, so lead time and altitude are reachable on
        /// the same path as everything else rather than needing procedures of their own.
        /// </summary>
        internal static MemberInfo SelectorMember (object operation, string name)
        {
            var selector = TimeSelectorOf (operation);
            if (selector == null)
                return null;
            var member = MechJebApi.FindField (selector.GetType (), name) as MemberInfo
                         ?? MechJebApi.FindProperty (selector.GetType (), name);
            return member;
        }

        internal static object SelectorOf (object operation)
        {
            return TimeSelectorOf (operation);
        }

        // ==================================================================
        // Planning
        // ==================================================================

        /// <summary>The non-fatal message from the last plan, if the operation left one.</summary>
        internal static string LastWarning {
            get { return lastWarning ?? string.Empty; }
        }

        /// <summary>
        /// Run the operation and put its nodes in the game.
        ///
        /// Mirrors what MechJeb's own window does, including the part that is easy to miss:
        /// when nodes already exist and you are appending, the plan starts from the END of
        /// the last one rather than from now, which is what makes "circularise" after
        /// "change apoapsis" mean what you expect.
        /// </summary>
        internal static int CreateNodes (string operationName, bool append)
        {
            Require ();
            var operation = Get (operationName);

            var vessel = FlightGlobals.ActiveVessel;
            if (vessel == null)
                throw new InvalidOperationException ("no active vessel (not in flight?)");

            if (patchedConicsUnlocked != null) {
                bool unlocked;
                try {
                    unlocked = Convert.ToBoolean (patchedConicsUnlocked.Invoke (null, new object[] { vessel }));
                } catch (Exception) {
                    unlocked = true;   // could not ask; do not block on a diagnostic
                }
                if (!unlocked)
                    throw new InvalidOperationException (
                        "this save cannot use maneuver nodes yet - the tracking station needs "
                        + "upgrading, and MechJeb's own planner is greyed out for the same reason");
            }

            var solver = vessel.patchedConicSolver;
            var ut = Planetarium.GetUniversalTime ();
            object orbit = vessel.orbit;

            if (append && solver != null && solver.maneuverNodes != null && solver.maneuverNodes.Count > 0) {
                var last = solver.maneuverNodes [solver.maneuverNodes.Count - 1];
                ut = last.UT;
                if (last.nextPatch != null)
                    orbit = last.nextPatch;
            }

            var target = MechJebModules.Get ("Target");

            object plan;
            try {
                plan = makeNodes.Invoke (operation, new[] { orbit, ut, target });
            } catch (Exception e) {
                throw new InvalidOperationException (
                    "MechJeb threw while planning " + DisplayName (operation) + ": "
                    + MechJebModules.Inner (e).Message);
            }

            var message = ErrorMessageOf (operation);

            if (plan == null) {
                lastWarning = string.Empty;
                throw new InvalidOperationException (
                    string.IsNullOrEmpty (message)
                        ? DisplayName (operation) + " could not be planned, and MechJeb gave no reason"
                        : DisplayName (operation) + ": " + message);
            }

            // A plan can arrive WITH a message. Three operations warn and succeed - a
            // semi-major axis large enough to go hyperbolic, an inclination too shallow to
            // shift the ascending node accurately, an approach that is not close enough to
            // fine-tune. Treating a message as failure would reject perfectly good plans;
            // ignoring it would hide a real caveat. So it is kept and readable.
            lastWarning = message ?? string.Empty;

            var placed = 0;
            foreach (var entry in (IEnumerable) plan) {
                if (entry == null)
                    continue;
                var deltaV = parametersDv.GetValue (entry);   // boxed Vector3d, never unpacked
                var burnUt = parametersUt.GetValue (entry);
                try {
                    placeManeuverNode.Invoke (null, new[] { vessel, orbit, deltaV, burnUt });
                } catch (Exception e) {
                    // PlaceManeuverNode guards against NaN itself and throws outside
                    // MakeNodes' own protection, so this is the only place it can be caught.
                    throw new InvalidOperationException (
                        "MechJeb planned " + DisplayName (operation) + " but the node was rejected: "
                        + MechJebModules.Inner (e).Message
                        + (placed > 0 ? " (" + placed + " node(s) were already placed)" : string.Empty));
                }
                placed++;
            }
            return placed;
        }

        static string ErrorMessageOf (object operation)
        {
            if (getErrorMessage == null)
                return string.Empty;
            try {
                var message = getErrorMessage.Invoke (operation, null);
                return message == null ? string.Empty : message.ToString ();
            } catch (Exception) {
                return string.Empty;
            }
        }
    }
}
