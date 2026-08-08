// Minimal stand-ins for the KSP, Unity and kRPC types this plugin touches.
//
// They exist ONLY so the sources can be type-checked without a KSP install
// (see build/verify/). They are never referenced by the real build, which
// resolves these types from GameData and KSP_x64_Data/Managed.
//
// Signatures mirror the real ones. If a stub has to change to make the build
// pass, that is a signal the real code is wrong - fix the code, not the stub.

using System;
using System.Collections.Generic;
using System.Reflection;

// ---------------------------------------------------------------- UnityEngine

namespace UnityEngine
{
    public class Object
    {
        public int GetInstanceID () { return 0; }
        public static Object FindObjectOfType (Type type) { return null; }
        public static bool operator == (Object x, Object y) { return ReferenceEquals (x, y); }
        public static bool operator != (Object x, Object y) { return !ReferenceEquals (x, y); }
        public override bool Equals (object other) { return ReferenceEquals (this, other); }
        public override int GetHashCode () { return base.GetHashCode (); }
    }

    public class GameObject : Object
    {
        public string name;
        public GameObject () { }
        public GameObject (string name) { }
    }

    public class Component : Object
    {
        public GameObject gameObject { get { return null; } }
    }
    public class Behaviour : Component { }

    public class YieldInstruction { }
    public sealed class WaitForSeconds : YieldInstruction { public WaitForSeconds (float seconds) { } }
    public sealed class WaitForEndOfFrame : YieldInstruction { }
    public sealed class WaitForFixedUpdate : YieldInstruction { }
    public sealed class Coroutine : Object { }

    public class MonoBehaviour : Behaviour
    {
        public Coroutine StartCoroutine (System.Collections.IEnumerator routine) { return null; }
        public void StopCoroutine (System.Collections.IEnumerator routine) { }
        public static void DontDestroyOnLoad (Object target) { }
        public static void Destroy (Object target) { }
        public static void Destroy (Object target, float delay) { }
    }

    public static class Debug
    {
        public static void Log (object message) { }
        public static void LogError (object message) { }
        public static void LogWarning (object message) { }
    }

    public static class Time
    {
        public static float realtimeSinceStartup { get { return 0f; } }
        public static float time { get { return 0f; } }
        public static float fixedDeltaTime { get { return 0.02f; } }
    }
}

// ------------------------------------------------- KSP (Assembly-CSharp, global namespace)

[AttributeUsage (AttributeTargets.Class)]
public sealed class KSPAddon : Attribute
{
    public enum Startup
    {
        Instantly, MainMenu, Settings, Credits, SpaceCentre, EditorAny,
        TrackingStation, Flight, EveryScene, EditorSPH, EditorVAB,
        SpaceCentreEditorSPH, SpaceCentreEditorVAB, PSystemSpawn,
        FlightAndEditor, FlightAndKSC, FlightEditorAndKSC, AllGameScenes
    }

    public KSPAddon (Startup startup, bool once) { }
}

public static class AssemblyLoader
{
    public class LoadedAssembly
    {
        public Assembly assembly;
        public string name;
    }

    public static List<LoadedAssembly> loadedAssemblies = new List<LoadedAssembly> ();
}

public class Vessel : UnityEngine.MonoBehaviour
{
    public Guid id;
    public uint persistentId;
    public bool packed;
    public bool loaded;
    public string vesselName;
    public string GetDisplayName () { return null; }

    public enum Situations
    {
        LANDED, SPLASHED, PRELAUNCH, FLYING, SUB_ORBITAL, ORBITING, ESCAPING, DOCKED
    }

    public Situations situation;
    public float totalMass;
    public List<Part> parts = new List<Part> ();
    public double missionTime;
    public Orbit orbit;
    public PatchedConicSolver patchedConicSolver;
}

// The maneuver-node side of the stock API, used by the MechJeb plugin's maneuver planner.
// MechJeb computes the burn and places the node; we only need to find where the last one
// ends so a second operation can be planned from there, exactly as MechJeb's own window
// does. The node itself is then read by stock kRPC, not by us.
public class Orbit
{
}

public class ManeuverNode
{
    public double UT;
    public Orbit nextPatch;
}

public class PatchedConicSolver
{
    public List<ManeuverNode> maneuverNodes = new List<ManeuverNode> ();
}

public class Part : UnityEngine.MonoBehaviour
{
    public Vessel vessel;
    public uint flightID;
    public AvailablePart partInfo;
    public double temperature;
    public double maxTemp;
    public double skinTemperature;
    public double skinMaxTemp;
}

public class PartModule : UnityEngine.MonoBehaviour
{
    public Part part;
    public Vessel vessel;
}

public static class FlightGlobals
{
    public static Vessel ActiveVessel { get { return null; } }
    public static bool ready { get { return false; } }
    public static List<Vessel> Vessels = new List<Vessel> ();
    public static List<Vessel> VesselsLoaded = new List<Vessel> ();
}

public class EventVoid
{
    public delegate void OnEvent ();
    public void Add (OnEvent e) { }
    public void Remove (OnEvent e) { }
    public void Fire () { }
}

public class EventData<T>
{
    public delegate void OnEvent (T data);
    public void Add (OnEvent e) { }
    public void Remove (OnEvent e) { }
    public void Fire (T data) { }
}

public class EventData<T, U>
{
    public delegate void OnEvent (T first, U second);
    public void Add (OnEvent e) { }
    public void Remove (OnEvent e) { }
    public void Fire (T first, U second) { }
}

public static class GameEvents
{
    // The two-argument shapes KSP uses for "X moved from A to B".
    public class HostedFromToAction<THost, TValue>
    {
        public THost host;
        public TValue from;
        public TValue to;
    }

    public class FromToAction<TFrom, TTo>
    {
        public TFrom from;
        public TTo to;
    }

    public static EventVoid onHideUI = new EventVoid ();
    public static EventVoid onShowUI = new EventVoid ();
    public static EventVoid onFlightReady = new EventVoid ();
    public static EventData<GameScenes> onGameSceneLoadRequested = new EventData<GameScenes> ();

    public static EventData<Vessel> onVesselDestroy = new EventData<Vessel> ();
    public static EventData<Vessel> onVesselChange = new EventData<Vessel> ();
    public static EventData<ProtoVessel, bool> onVesselRecovered = new EventData<ProtoVessel, bool> ();
    public static EventData<HostedFromToAction<Vessel, Vessel.Situations>> onVesselSituationChange
        = new EventData<HostedFromToAction<Vessel, Vessel.Situations>> ();
    public static EventData<HostedFromToAction<Vessel, CelestialBody>> onVesselSOIChanged
        = new EventData<HostedFromToAction<Vessel, CelestialBody>> ();
    public static EventData<int> onStageActivate = new EventData<int> ();
    public static EventData<Part> onPartDie = new EventData<Part> ();
    public static EventData<EventReport> onCrash = new EventData<EventReport> ();
    public static EventData<EventReport> onCrashSplashdown = new EventData<EventReport> ();
}

public enum GameScenes
{
    LOADING, LOADINGBUFFER, MAINMENU, SETTINGS, CREDITS,
    SPACECENTER, EDITOR, FLIGHT, TRACKSTATION, PSYSTEM, MISSIONBUILDER
}

public static class HighLogic
{
    public static bool LoadedSceneIsFlight { get { return false; } }
}

namespace KSP.UI
{
    public class UIMasterController
    {
        public static UIMasterController Instance { get; set; }
        public bool IsUIShowing { get { return true; } }
        public void HideUI () { }
        public void ShowUI () { }
    }
}


// KSPAssembly / KSPAssemblyDependency drive KSP's load order. They are what makes a
// plugin skip loading (with a log line) when the Core is absent, instead of loading and
// throwing TypeLoadException on first use.
[AttributeUsage (AttributeTargets.Assembly, AllowMultiple = true)]
public sealed class KSPAssembly : Attribute
{
    public KSPAssembly (string name, int major, int minor) { }
    public KSPAssembly (string name, int major, int minor, int revision) { }
}

[AttributeUsage (AttributeTargets.Assembly, AllowMultiple = true)]
public sealed class KSPAssemblyDependency : Attribute
{
    public KSPAssemblyDependency (string name, int major, int minor) { }
    public KSPAssemblyDependency (string name, int major, int minor, int revision) { }
}

public static class Planetarium
{
    public static double GetUniversalTime () { return 0.0; }
}

public class AvailablePart
{
    public string name;
    public string title;
}

public class ProtoVessel
{
    public Guid vesselID;
    public string vesselName;
}

public class CelestialBody : UnityEngine.MonoBehaviour
{
    public string bodyName;
}

public class EventReport
{
    public Part origin;
}

public class GameEventsHosted { }

// ---------------------------------------------------------------------- kRPC

namespace KRPC.Service
{
    [Flags]
    [Serializable]
    public enum GameScene
    {
        None = 0,
        Inherit = 0,
        SpaceCenter = 1 << 0,
        Flight = 1 << 1,
        TrackingStation = 1 << 2,
        EditorVAB = 1 << 3,
        EditorSPH = 1 << 4,
        Editor = EditorSPH | EditorVAB,
        MissionBuilder = 1 << 5,
        AstronautComplex = 1 << 6,
        MissionControl = 1 << 7,
        ResearchAndDevelopment = 1 << 8,
        Administration = 1 << 9,
        All = ~0
    }

    public class YieldException : Exception
    {
        public object UntypedValue { get; set; }
    }

    public sealed class YieldException<T> : YieldException
    {
        public YieldException (T value) { Value = value; }
        public T Value {
            get { return (T) UntypedValue; }
            private set { UntypedValue = value; }
        }
    }

    // kRPC DOES have an event primitive. It rides the stream channel: Trigger() latches
    // a flag the client's stream picks up on the next update, so it says THAT something
    // happened, carries no payload, and coalesces two triggers inside one tick.
    public class Event
    {
        public Event () { }
        public Event (Func<Event, bool> predicate) { }
        public Event (Func<bool> predicate) { }
        public Messages.Event Message { get { return new Messages.Event (); } }
        public void Trigger () { }
        public void Remove () { }
    }
}

namespace KRPC.Service.Messages
{
    public interface IMessage { }

    public class Event : IMessage { }
}

namespace KRPC.Service.Attributes
{
    [AttributeUsage (AttributeTargets.Class)]
    public sealed class KRPCServiceAttribute : Attribute
    {
        public string Name { get; set; }
        public uint Id { get; set; }
        public GameScene GameScene { get; set; }
        public KRPCServiceAttribute () { GameScene = GameScene.All; }
    }

    [AttributeUsage (AttributeTargets.Method)]
    public sealed class KRPCProcedureAttribute : Attribute
    {
        public bool Nullable { get; set; }
        public GameScene GameScene { get; set; }
        public KRPCProcedureAttribute () { GameScene = GameScene.Inherit; }
    }

    [AttributeUsage (AttributeTargets.Property)]
    public sealed class KRPCPropertyAttribute : Attribute
    {
        public bool Nullable { get; set; }
        public GameScene GameScene { get; set; }
        public KRPCPropertyAttribute () { GameScene = GameScene.Inherit; }
    }

    [AttributeUsage (AttributeTargets.Class)]
    public sealed class KRPCClassAttribute : Attribute
    {
        public string Service { get; set; }
        public GameScene GameScene { get; set; }
        public KRPCClassAttribute () { GameScene = GameScene.Inherit; }
    }

    [AttributeUsage (AttributeTargets.Method)]
    public sealed class KRPCMethodAttribute : Attribute
    {
        public bool Nullable { get; set; }
        public GameScene GameScene { get; set; }
        public KRPCMethodAttribute () { GameScene = GameScene.Inherit; }
    }

    [AttributeUsage (AttributeTargets.Class)]
    public sealed class KRPCExceptionAttribute : Attribute
    {
        public string Service { get; set; }
        public Type MappedException { get; set; }
    }

    [AttributeUsage (AttributeTargets.Enum)]
    public sealed class KRPCEnumAttribute : Attribute
    {
        public string Service { get; set; }
    }

    // Note the target: this moved from Method to Parameter in kRPC 0.5.2.
    [AttributeUsage (AttributeTargets.Parameter)]
    public sealed class KRPCDefaultValueAttribute : Attribute
    {
        public KRPCDefaultValueAttribute (Type valueConstructor) { }
    }

    [AttributeUsage (AttributeTargets.Parameter)]
    public sealed class KRPCNullableAttribute : Attribute { }
}
