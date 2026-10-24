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
        // Real: public int layer { get; set; } (15: Local Scenery, the ground; 0: parts).
        public int layer { get; set; }
        public GameObject () { }
        public GameObject (string name) { }
    }

    public class Component : Object
    {
        public GameObject gameObject { get { return null; } }
    }
    // Real: UnityEngine.Collider : Component (PhysicsModule).
    public class Collider : Component { }
    // Real: UnityEngine.Rigidbody : Component (PhysicsModule); Part.Rigidbody (SurfaceModelV1).
    // (the vessel's aerodynamic torque, as kRPC's Flight.AerodynamicTorque reads it): the
    // angular drag FlightIntegrator sets and the inertia it acts on (real properties).
    public class Rigidbody : Component
    {
        public float angularDrag { get; set; }
        public Quaternion rotation { get; set; }
        public Quaternion inertiaTensorRotation { get; set; }
        public Vector3 inertiaTensor { get; set; }
        public Vector3 angularVelocity { get; set; }
        // Real: UnityEngine.Rigidbody.mass, position, centerOfMass (local), worldCenterOfMass. an earlier change reads the
        // last three of every part to complete Vessel.MOI.
        public float mass { get; set; }
        public Vector3 position { get; set; }
        public Vector3 centerOfMass { get; set; }
        public Vector3 worldCenterOfMass { get { return Vector3.zero; } }
    }

    // Real: UnityEngine.WheelHit (PhysicsModule), the struct VehiclePhysics keeps in WheelState.hit.
    public struct WheelHit
    {
        public Collider collider { get; set; }
        public Vector3 point { get; set; }
        public Vector3 normal { get; set; }
        public float force { get; set; }
    }

    public class Behaviour : Component
    {
        // Real: UnityEngine.Behaviour.enabled (property). Used by the FMRS dropped-stage watch.
        public bool enabled { get; set; }
    }

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
        // Real: UnityEngine.Time.fixedTime (float) and frameCount (int).
        public static float fixedTime { get { return 0f; } }
        public static int frameCount { get { return 0; } }
    }

    // The Trajectories plugin unboxes the mod's Vector3? results. Never crosses the kRPC
    // wire: it is flattened to an IList of double first.
    public struct Vector3
    {
        public float x, y, z;
        public Vector3 (float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public static Vector3 zero { get { return new Vector3 (0f, 0f, 0f); } }
        public static Vector3 right { get { return new Vector3 (1f, 0f, 0f); } }
        public static Vector3 up { get { return new Vector3 (0f, 1f, 0f); } }
        public static Vector3 forward { get { return new Vector3 (0f, 0f, 1f); } }
        public static Vector3 operator * (float value, Vector3 vector) { return vector; }
        public static Vector3 operator + (Vector3 left, Vector3 right) { return left; }
        public static Vector3 operator - (Vector3 left, Vector3 right) { return left; }
        public static Vector3 operator / (Vector3 vector, float value) { return vector; }
        public float sqrMagnitude { get { return 1f; } }
        public Vector3 normalized { get { return this; } }
    }

    public struct Quaternion
    {
        // Real: public float x, y, z, w (read directly by the QW6 fast path).
        public float x, y, z, w;
        public static Quaternion identity { get { return new Quaternion (); } }
        public static Quaternion AngleAxis (float angle, Vector3 axis) { return new Quaternion (); }
        public static Quaternion Inverse (Quaternion rotation) { return rotation; }
        public static Quaternion operator * (Quaternion left, Quaternion right) { return left; }
        public static Vector3 operator * (Quaternion rotation, Vector3 point) { return point; }
        // Real: Dot (float32), then IsEqualUsingDot: dot > 0.999999f; != is !(==) (SurfaceModelV1).
        public static float Dot (Quaternion a, Quaternion b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }
        public static bool operator == (Quaternion lhs, Quaternion rhs) { return Dot (lhs, rhs) > 0.999999f; }
        public static bool operator != (Quaternion lhs, Quaternion rhs) { return !(lhs == rhs); }
        public override bool Equals (object other)
        {
            if (!(other is Quaternion))
                return false;
            var q = (Quaternion)other;
            return x.Equals (q.x) && y.Equals (q.y) && z.Equals (q.z) && w.Equals (q.w);
        }
        public override int GetHashCode () { return x.GetHashCode () ^ y.GetHashCode () << 2 ^ z.GetHashCode () >> 2 ^ w.GetHashCode () >> 1; }
    }

    public class Transform : Object
    {
        public Quaternion localRotation;
        public Vector3 position { get { return Vector3.zero; } }
        public Vector3 forward { get { return Vector3.forward; } }
        public Vector3 up { get { return Vector3.up; } }
        public Vector3 right { get { return Vector3.right; } }
        public Quaternion rotation { get { return new Quaternion (); } }
        public Transform parent { get { return null; } }
        public Vector3 InverseTransformDirection (Vector3 direction) { return direction; }
        public Vector3 TransformDirection (Vector3 direction) { return direction; }
        // Real (SurfaceModelV1): local <-> world points, scale included.
        public Vector3 InverseTransformPoint (Vector3 position) { return position; }
        public Vector3 TransformPoint (Vector3 position) { return position; }
        // Real: [FreeFunction("Internal_IsChildOrSameTransform")] - true for itself too.
        public bool IsChildOf (Transform parent) { return false; }
    }
}

// KSP's double-precision vector (global namespace). Only what the Trajectories plugin
// uses to turn a body-relative impact into latitude and longitude.
public struct Vector3d
{
    public double x, y, z;
    public Vector3d (double x, double y, double z) { this.x = x; this.y = y; this.z = z; }
    public static Vector3d zero { get { return new Vector3d (0, 0, 0); } }
    public static Vector3d operator + (Vector3d a, Vector3d b) { return new Vector3d (a.x + b.x, a.y + b.y, a.z + b.z); }
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
    public UnityEngine.Transform ReferenceTransform { get { return null; } }
    public UnityEngine.Vector3 CurrentCoM { get { return UnityEngine.Vector3.zero; } }
    public string GetDisplayName () { return null; }

    public enum Situations
    {
        LANDED, SPLASHED, PRELAUNCH, FLYING, SUB_ORBITAL, ORBITING, ESCAPING, DOCKED
    }

    public Situations situation;
    public float totalMass;
    public List<Part> parts = new List<Part> ();
    public double missionTime;
    // Real: public int currentStage (field, KSP 1.12.5). FMRS dropped-stage table.
    public int currentStage;
    // KSP 1.12.5 (assembly inspection, installed Assembly-CSharp): public FIELDS.
    // angularVelocity is in the vessel BODY frame, MOI is the body diagonal in t.m^2.
    public UnityEngine.Vector3 angularVelocity;
    public UnityEngine.Vector3 MOI;
    public FlightCtrlState ctrlState = new FlightCtrlState ();
    public FlightInputCallback OnFlyByWire;
    // Real: public bool Landed / Splashed (fields, KSP 1.12.5 installed Assembly-CSharp).
    public bool Landed;
    public bool Splashed;
    public Orbit orbit;
    public PatchedConicSolver patchedConicSolver;
    public CelestialBody mainBody { get { return null; } }
    // ContactV1 (refactor lot 3). Real public fields, KSP 1.12.5 (assembly inspection, installed
    // Assembly-CSharp): upAxis, verticalSpeed, horizontalSrfSpeed (UpdatePosVel, every step);
    // heightFromTerrain and terrainNormal (GetHeightFromTerrain from UpdatePosVel; terrainNormal in
    // vesselTransform's local frame); vesselTransform (the vessel's own transform).
    public Vector3d upAxis;
    public double verticalSpeed;
    public double horizontalSrfSpeed;
    public float heightFromTerrain;
    public UnityEngine.Vector3 terrainNormal;
    public UnityEngine.Transform vesselTransform;
    // (attitude_throttle_v1): real public fields, KSP 1.12.5 (assembly inspection, installed
    // Assembly-CSharp): Vector3d srf_velocity, Vector3d obt_velocity, double altitude.
    public Vector3d srf_velocity;
    public Vector3d obt_velocity;
    public double altitude;
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

// Real: public class FlightCtrlState : IConfigNode, float fields pitch/yaw/roll/mainThrottle...
public class FlightCtrlState
{
    public float pitch;
    public float yaw;
    public float roll;
    public float mainThrottle;
    // Linear inputs (RCS translation): X right, Y forward (body z), Z up (body y).
    public float X;
    public float Y;
    public float Z;
}

// Real: public delegate void FlightInputCallback(FlightCtrlState st).
public delegate void FlightInputCallback (FlightCtrlState st);

public class FlightInputHandler : UnityEngine.MonoBehaviour
{
    public static FlightInputHandler fetch;
    public bool precisionMode;
}

// Real: ITorqueProvider.GetPotentialTorque(out Vector3 pos, out Vector3 neg), kN.m.
public interface ITorqueProvider
{
    void GetPotentialTorque (out UnityEngine.Vector3 pos, out UnityEngine.Vector3 neg);
}

// Real: TimingManager (MonoBehaviour) with nested TimingStage and UpdateAction;
// FixedUpdateAdd/Remove are static and silently return while Instance is null.
public class TimingManager : UnityEngine.MonoBehaviour
{
    public enum TimingStage
    {
        ObscenelyEarly, Early, Precalc, Earlyish, Normal, FashionablyLate,
        FlightIntegrator, Late, BetterLateThanNever
    }

    public delegate void UpdateAction ();

    // Real: protected static TimingManager Instance { get; set; } - not readable from a plugin.
    protected static TimingManager Instance { get { return null; } }
    public static void FixedUpdateAdd (TimingStage stage, UpdateAction action) { }
    public static void FixedUpdateRemove (TimingStage stage, UpdateAction action) { }
}

public class Part : UnityEngine.MonoBehaviour
{
    public Vessel vessel;
    public uint flightID;
    public bool ShieldedFromAirstream { get; set; }
    public AvailablePart partInfo;
    public double temperature;
    public double maxTemp;
    public double skinTemperature;
    public double skinMaxTemp;
    public double staticPressureAtm;
    public double atmDensity;
    public List<PartModule> Modules = new List<PartModule> ();
    // ContactV1 (refactor lot 3). Real public fields, KSP 1.12.5 (assembly inspection): the part's
    // persistentId, the two flags Vessel.checkLanded reads, and the current collisions.
    public uint persistentId;
    public bool GroundContact;
    public bool PermanentGroundContact;
    // Real (assembly inspection): public float crashTolerance.
    public float crashTolerance;
    public DictionaryValueList<UnityEngine.Collider, int> currentCollisions =
        new DictionaryValueList<UnityEngine.Collider, int> ();
    // SurfaceModelV1 (lot 3, 3.1). Real, KSP 1.12.5 (assembly inspection): public Transform
    // partTransform; public Vector3 CoLOffset, CoPOffset; public double machNumber (written by
    // FlightIntegrator); public Rigidbody Rigidbody { get; } (the parent's for a physicsless part).
    public UnityEngine.Transform partTransform;
    public UnityEngine.Vector3 CoLOffset;
    public UnityEngine.Vector3 CoPOffset;
    public double machNumber;
    public UnityEngine.Rigidbody Rigidbody { get { return null; } }
    // the aerodynamic forces FlightIntegrator applied this step (KSP 1.12.5, decompiled
    // Part and FlightIntegrator.ApplyAeroDrag / ApplyAeroLift; assembly inspection): public
    // fields Rigidbody rb, Vector3 dragVectorDir, float dragScalar, Vector3 bodyLiftLocalVector,
    // bodyLiftLocalPosition, bool hasLiftModule, bodyLiftOnlyUnattachedLiftActual, ILiftProvider
    // bodyLiftOnlyProvider.
    public UnityEngine.Rigidbody rb;
    public UnityEngine.Vector3 dragVectorDir;
    public float dragScalar;
    public UnityEngine.Vector3 bodyLiftLocalVector;
    public UnityEngine.Vector3 bodyLiftLocalPosition;
    public bool hasLiftModule;
    public bool bodyLiftOnlyUnattachedLiftActual;
    public ILiftProvider bodyLiftOnlyProvider;
}

// Real: public interface ILiftProvider (KSP 1.12.5): bool IsLifting { get; } among others.
public interface ILiftProvider
{
    bool IsLifting { get; }
}

// Real: KSP's DictionaryValueList<TKey, TValue> (a dictionary and a list kept together), Count property.
public class DictionaryValueList<TKey, TValue>
{
    public int Count { get { return 0; } }
}

// Real: public enum WheelType { FREE, MOTORIZED, LEG } (KSP 1.12.5, decompiled).
public enum WheelType { FREE, MOTORIZED, LEG }

// Real: public class ModuleWheelBase : PartModule (global namespace): Wheel (property, the
// KSPWheelController, null before setup), isGrounded and wheelType (KSPFields).
public class ModuleWheelBase : PartModule
{
    public bool isGrounded;
    public WheelType wheelType;
    public KSPWheelController Wheel { get { return null; } }
    // Real (assembly inspection): public float radius (KSPField).
    public float radius;
}

// Real: public class KSPWheelController : VehiclePhysics.VehicleBase; currentState = wheelState[0]
// (null before init).
public class KSPWheelController : VehiclePhysics.VehicleBase
{
    public VehiclePhysics.VehicleBase.WheelState currentState { get { return null; } }
    // Real (assembly inspection, decompiled): public VPWheelCollider wheelCollider (field);
    // WheelCenter, WheelDown, WheelRadius (get-only properties).
    public VehiclePhysics.VPWheelCollider wheelCollider;
    public UnityEngine.Vector3 WheelCenter { get { return UnityEngine.Vector3.zero; } }
    public UnityEngine.Vector3 WheelDown { get { return UnityEngine.Vector3.zero; } }
    public float WheelRadius { get { return 0f; } }
}

namespace VehiclePhysics
{
    // Real: public class VPWheelCollider : VehicleBehaviour (Assembly-CSharp 1.12.5): public fields
    // center, radius, suspensionDistance, suspensionAnchor, springRate, damperRate; Transform cachedTransform { get; }.
    public class VPWheelCollider : UnityEngine.MonoBehaviour
    {
        public UnityEngine.Vector3 center;
        public float radius;
        public float suspensionDistance;
        public float suspensionAnchor;
        public float springRate;
        public float damperRate;
        public UnityEngine.Transform cachedTransform { get { return null; } }
    }
}

namespace VehiclePhysics
{
    // Real: public class VehicleBase : MonoBehaviour, nested public class WheelState (fields).
    public class VehicleBase : UnityEngine.MonoBehaviour
    {
        public class WheelState
        {
            public bool grounded;
            public UnityEngine.WheelHit hit;
            public float downforce;
            public float suspensionCompression;
            public float contactDepth;
            public float contactSpeed;
            public float normalizedLoad;
        }
    }
}

namespace ModuleWheels
{
    // Real: public class ModuleWheelDeployment : ModuleWheelSubmodule (a PartModule): position
    // (0..1, now) and deployedPosition (KSPFields).
    public class ModuleWheelDeployment : ModuleWheelSubmodule
    {
        public float position;
        public float deployedPosition;
    }

    // Real (assembly inspection, KSP 1.12.5): ModuleWheelSubmodule : PartModule with
    // public int baseModuleIndex (KSPField); ModuleWheelSuspension and ModuleWheelDamage derive from it.
    public class ModuleWheelSubmodule : PartModule
    {
        public int baseModuleIndex;
    }

    public class ModuleWheelSuspension : ModuleWheelSubmodule
    {
        public float suspensionDistance;
        public float springRatio;
        public float damperRatio;
        public float maximumLoad;
        public float targetPosition;
    }

    public class ModuleWheelDamage : ModuleWheelSubmodule
    {
        public float impactTolerance;
        public float stressTolerance;
        public bool isDamaged;
    }
}

// Real: public class PartJoint : MonoBehaviour; Child, Parent, Host, Target (properties).
public class PartJoint : UnityEngine.MonoBehaviour
{
    public Part Child { get { return null; } }
    public Part Parent { get { return null; } }
    public Part Host { get { return null; } }
    public Part Target { get { return null; } }
}

public class PartModule : UnityEngine.MonoBehaviour
{
    public Part part;
    public Vessel vessel;
    public bool moduleIsEnabled;
}

public class FloatCurve
{
    public float Evaluate (float time) { return 0f; }
}

public class ModuleEngines : PartModule
{
    public bool EngineIgnited;
    public float requestedThrottle;
    public float currentThrottle;
    public float finalThrust;
    public float maxThrust;
    public float thrustPercentage;
    public bool independentThrottle;
    public float independentThrottlePercentage;
    public bool useEngineResponseTime;
    public float engineAccelerationSpeed;
    public float engineDecelerationSpeed;
    // Real: public bool throttleUseAlternate (KSPField, KSP 1.12.5, assembly inspection;
    // UpdateThrottle decompiled: another rate law when true).
    public bool throttleUseAlternate;
    // KSP 1.12.5: both are float (decompiled Assembly-CSharp).
    public float requestedMassFlow;
    public float propellantReqMet;
    public float realIsp;
    public FloatCurve atmosphereCurve;
    public List<UnityEngine.Transform> thrustTransforms = new List<UnityEngine.Transform> ();
    public List<float> thrustTransformMultipliers = new List<float> ();
    // KSP 1.12.5 (assembly inspection, installed Assembly-CSharp), read by the lateral-force layer:
    // public float throttleMin; public bool throttleLocked (KSPField);
    // public bool isOperational { get; }; public bool getFlameoutState { get; }.
    public float throttleMin;
    public bool throttleLocked;
    public bool isOperational { get { return true; } }
    public bool getFlameoutState { get { return false; } }
    // Real: massFlow of the last CalculateThrust (tonnes per tick, FuelUsage included).
    public double MassFlow () { return 0.0; }
    // Real: stock max thrust at the given atmosphere (kN), no side effect.
    public virtual float MaxThrustOutputAtm (bool runningActive = false, bool useThrustLimiter = true,
        float atmPressure = 1f, double atmTemp = 310.0, double atmDensity = 1.225000023841858) { return 0f; }
}

public class ModuleRCS : PartModule, ITorqueProvider
{
    public List<UnityEngine.Transform> thrusterTransforms = new List<UnityEngine.Transform> ();
    public float[] thrustForces = new float[0];
    public bool useZaxis;
    public bool isJustForShow;
    public bool rcsEnabled;
    public bool rcs_active;
    public float thrusterPower;
    public float realISP;
    // KSP 1.12.5 fields read by AttitudeGeometry (CalculateThrust and FixedUpdate terms).
    public bool requiresFuel;
    public double flowMult;
    public double maxFuelFlow;
    public float thrustPercentage;
    public double G;
    public double ispMult;
    public bool enablePitch;
    public bool enableRoll;
    public bool enableYaw;
    // Linear axes ModuleRCS.Update reads (ctrlState.X, Z-or-throttle, Y): computed levers (bit 4).
    public bool enableX;
    public bool enableY;
    public bool enableZ;
    public bool fullThrust;
    public bool shieldedCanThrust;
    // Real KSPFields (KSP 1.12.5, assembly inspection): precision mode multiplies each nozzle's thrust
    // by precisionFactor, or divides it by its lever distance when useLever; fullThrust fires
    // at 1 above fullThrustMin. Read by the effector registry (refactor R1).
    public float precisionFactor;
    public bool useLever;
    public float fullThrustMin;
    // (KSP 1.12.5, decompiled): the dead zone of ModuleRCS.Update (KSPField, 0.05 by default) and
    // the PRIVATE world-frame command it stores there and FixedUpdate fires (read by reflection).
    public float EPSILON = 0.05f;
    private UnityEngine.Vector3 inputRot = UnityEngine.Vector3.zero;
    public void GetPotentialTorque (out UnityEngine.Vector3 pos, out UnityEngine.Vector3 neg)
    {
        pos = UnityEngine.Vector3.zero;
        neg = UnityEngine.Vector3.zero;
    }
}

public class ModuleReactionWheel : PartModule, ITorqueProvider
{
    public enum WheelState { Active, Disabled, Broken }
    public WheelState wheelState;
    public bool operational;
    public UnityEngine.Vector3 inputVector;
    public float PitchTorque;
    public float RollTorque;
    public float YawTorque;
    public float authorityLimiter;
    public int actuatorModeCycle;
    public void GetPotentialTorque (out UnityEngine.Vector3 pos, out UnityEngine.Vector3 neg)
    {
        pos = UnityEngine.Vector3.zero;
        neg = UnityEngine.Vector3.zero;
    }
}

// Real (KSP 1.12.5, decompiled; SurfaceModelV1): public class ModuleLiftingSurface : PartModule,
// ILiftProvider -- the fields the replica reads.
public class ModuleLiftingSurface : PartModule
{
    public enum TransformDir { X, Y, Z }
    public TransformDir transformDir;
    public float transformSign = 1f;
    public float deflectionLiftCoeff;
    public bool omnidirectional;
    public bool nodeEnabled;
    public bool perpendicularOnly;
    public bool displaceVelocity;
    public FloatCurve liftCurve;
    public FloatCurve liftMachCurve;
    public bool useInternalDragModel;
    public FloatCurve dragCurve;
    public FloatCurve dragMachCurve;
    public UnityEngine.Vector3 liftForce;
    public UnityEngine.Vector3 dragForce;
    // Real: protected, read by reflection.
    protected UnityEngine.Vector3 nVel;
    protected double Qlift;
    protected double Qdrag;
    protected UnityEngine.Transform baseTransform;
}

public class ModuleControlSurface : ModuleLiftingSurface, ITorqueProvider
{
    public void GetPotentialTorque (out UnityEngine.Vector3 pos, out UnityEngine.Vector3 neg)
    {
        pos = UnityEngine.Vector3.zero;
        neg = UnityEngine.Vector3.zero;
    }
    // SurfaceModelV1 (real KSPFields, KSP 1.12.5).
    public float ctrlSurfaceArea;
    public bool alwaysRecomputeLift;
    public bool deployInvert;
    public bool partDeployInvert;
    public bool usesMirrorDeploy;
    public bool mirrorDeploy;
    // Real: protected, read by reflection (deflectionDirection: 1 from OnStart, the lot-1 neutralizer).
    protected float deflectionDirection = 1f;
    protected UnityEngine.Transform ctrlSurface;
    protected UnityEngine.Quaternion neutral;
    public float ctrlSurfaceRange;
    public float authorityLimiter;
    public bool deploy;
    public float deployAngle;
    public float actuatorSpeed;
    public bool useExponentialSpeed;
    public bool ignorePitch;
    public bool ignoreYaw;
    public bool ignoreRoll;
    public float currentDeployAngle { get; private set; }
    // Read by reflection in the plugin, exactly as in KSP where they are protected.
    protected float action;
    protected float deflection;
}

public class ModuleAeroSurface : ModuleControlSurface
{
}

public class ModuleGimbal : PartModule
{
    public bool gimbalLock;
    public bool gimbalActive;
    public float gimbalLimiter;
    public float gimbalRange;
    public float gimbalRangeXN;
    public float gimbalRangeXP;
    public float gimbalRangeYN;
    public float gimbalRangeYP;
    public float xMult;
    public float yMult;
    public bool flipYZ;
    public bool useGimbalResponseSpeed;
    public float gimbalResponseSpeed;
    public UnityEngine.Vector3 actuationLocal;
    // Real: public Vector3[] oldActuationLocal, the Lerp memory of FixedUpdate (M4 restores it).
    public UnityEngine.Vector3[] oldActuationLocal;
    public List<UnityEngine.Transform> gimbalTransforms = new List<UnityEngine.Transform> ();
    public List<UnityEngine.Quaternion> initRots = new List<UnityEngine.Quaternion> ();
}

// Real: public class PhysicsGlobals : MonoBehaviour, static float properties (KSP 1.12.5).
public static class PhysicsGlobals
{
    public static float LiftMultiplier { get { return 0.036f; } }
    public static float LiftDragMultiplier { get { return 0.015f; } }
}

public static class TimeWarp
{
    public static float fixedDeltaTime { get { return 0.02f; } }
    // Real: public static float CurrentRate { get; } (1 without warp).
    public static float CurrentRate { get { return 1f; } }
}

public static class FlightGlobals
{
    public static Vessel ActiveVessel { get { return null; } }
    public static bool ready { get { return false; } }
    public static List<Vessel> Vessels = new List<Vessel> ();
    public static List<Vessel> VesselsLoaded = new List<Vessel> ();
    public static CelestialBody currentMainBody { get { return null; } }
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
    // Topology and packing events of the vessel registry (Core v2). Real: public static
    // EventData<Vessel> fields, KSP 1.12.5 (assembly inspection, installed Assembly-CSharp).
    public static EventData<Vessel> onVesselWasModified = new EventData<Vessel> ();
    public static EventData<Vessel> onVesselPartCountChanged = new EventData<Vessel> ();
    public static EventData<Vessel> onVesselCreate = new EventData<Vessel> ();
    public static EventData<Vessel> onVesselGoOnRails = new EventData<Vessel> ();
    public static EventData<Vessel> onVesselGoOffRails = new EventData<Vessel> ();
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

    // FMRS dropped-stage table (DroppedStagesV1). Real signatures, KSP 1.12.5 (assembly inspection):
    // onPartDeCoupleNewVesselComplete fires (original vessel, new vessel) at the end of
    // Part.decouple; onVesselsUndocking (old vessel, new vessel); onGameStateSaved at the end
    // of Game.Save, inside GamePersistence.SaveGame; onLaunch from StageManager.ActivateStage.
    public static EventData<Vessel, Vessel> onPartDeCoupleNewVesselComplete = new EventData<Vessel, Vessel> ();
    public static EventData<Vessel, Vessel> onVesselsUndocking = new EventData<Vessel, Vessel> ();
    public static EventData<Game> onGameStateSaved = new EventData<Game> ();
    public static EventData<EventReport> onLaunch = new EventData<EventReport> ();

    // Event bus V2 (refactor lot 3, R6). Real signatures, KSP 1.12.5 (decompiled GameEvents).
    public static EventData<PartJoint, float> onPartJointBreak = new EventData<PartJoint, float> ();
    public static EventData<Vessel> onPhysicsEaseStart = new EventData<Vessel> ();
    public static EventData<Vessel> onPhysicsEaseStop = new EventData<Vessel> ();
    public static EventData<Vessel> onVesselLoaded = new EventData<Vessel> ();
    public static EventData<Vessel> onVesselUnloaded = new EventData<Vessel> ();
    public static EventVoid onTimeWarpRateChanged = new EventVoid ();
}

public enum GameScenes
{
    LOADING, LOADINGBUFFER, MAINMENU, SETTINGS, CREDITS,
    SPACECENTER, EDITOR, FLIGHT, TRACKSTATION, PSYSTEM, MISSIONBUILDER
}

public static class HighLogic
{
    public static bool LoadedSceneIsFlight { get { return false; } }
    // Real: public static GameScenes LoadedScene (field).
    public static GameScenes LoadedScene;
}

// Real: Game, the save-game object GameEvents.onGameStateSaved carries.
public class Game { }

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

// Real: public class Planetarium : MonoBehaviour, with public static Planetarium fetch (null
// where the scene has none: main menu, editors) and GetUniversalTime().
public class Planetarium : UnityEngine.MonoBehaviour
{
    public static Planetarium fetch;
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
    // Real: public uint persistentId (KSP 1.12.5, assembly inspection).
    public uint persistentId;
}

public class CelestialBody : UnityEngine.MonoBehaviour
{
    public string bodyName;
    // World position of the body's centre, and the geographic conversions the
    // Trajectories plugin uses on a body-relative impact vector. Real signatures:
    //   public Vector3d position;
    //   public double GetLatitude (Vector3d worldPos)
    //   public double GetLongitude (Vector3d worldPos)
    //   public double TerrainAltitude (double latitude, double longitude, bool allowNegative = false)
    public Vector3d position;
    public double GetLatitude (Vector3d worldPos) { return 0.0; }
    public double GetLongitude (Vector3d worldPos) { return 0.0; }
    public double TerrainAltitude (double latitude, double longitude, bool allowNegative = false) { return 0.0; }
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

// ------------------------------------------------- kRPC SpaceCenter (KRPC.SpaceCenter.dll)
//
// Only the two classes the Ident plugin accepts as parameters, and only the members it
// reads. Everything here mirrors the real kRPC 0.6 source:
//
//   Services\Vessel.cs        public Guid Id { get; private set; }
//                             public global::Vessel InternalVessel { get; }
//   Services\Parts\Part.cs    public global::Part InternalPart { get; }
//
// Note what is NOT here and cannot be: neither class carries a [KRPCProperty] identifier.
// Id is a plain public C# property, which is exactly why a plugin can read it and a client
// cannot. If that ever changes upstream, conn.ident becomes redundant - a good outcome.
//
// The KRPCClass attribute is deliberately omitted: verify only needs these to type-check,
// and the real signature validation is build/scan against the real DLLs.

namespace KRPC.SpaceCenter.Services
{
    // Installed KRPC.SpaceCenter 0.6.0.0: public Vector3d DirectionToWorldSpace(Vector3d).
    public class ReferenceFrame
    {
        public Vector3d DirectionToWorldSpace (Vector3d direction) { return direction; }
    }

    public class Flight
    {
        public Tuple<double, double, double> AerodynamicForce { get { return Tuple.Create(0.0, 0.0, 0.0); } }
        public Tuple<double, double, double> AerodynamicTorque { get { return Tuple.Create(0.0, 0.0, 0.0); } }
        public Tuple<double, double, double> Lift { get { return Tuple.Create(0.0, 0.0, 0.0); } }
        public Tuple<double, double, double> Drag { get { return Tuple.Create(0.0, 0.0, 0.0); } }
        public float DynamicPressure { get { return 0f; } }
        public float StaticPressure { get { return 0f; } }
        public double AtmosphereDensity { get { return 0.0; } }
        public float SpeedOfSound { get { return 0f; } }
        public float TrueAirSpeed { get { return 0f; } }
        public float AngleOfAttack { get { return 0f; } }
        public float SideslipAngle { get { return 0f; } }

        // Installed KRPC.SpaceCenter 0.6.0.0 (decompiled): force (N) then torque (N.m
        // about the CoM), both in the Flight's reference frame.
        public Tuple<Tuple<double, double, double>, Tuple<double, double, double>> SimulateAerodynamicWrenchAt (
            CelestialBody body, Tuple<double, double, double> position,
            Tuple<double, double, double> velocity, Tuple<double, double, double, double> rotation,
            Tuple<double, double, double> angularVelocity, double ut)
        {
            return Tuple.Create (Tuple.Create (0.0, 0.0, 0.0), Tuple.Create (0.0, 0.0, 0.0));
        }
    }

    public class CelestialBody
    {
        public CelestialBody (global::CelestialBody body) { }
        public ReferenceFrame ReferenceFrame { get { return new ReferenceFrame (); } }
        public ReferenceFrame NonRotatingReferenceFrame { get { return new ReferenceFrame (); } }
    }

    public class Vessel
    {
        public Vessel () { }
        public Vessel (global::Vessel vessel) { }
        public Guid Id { get { return Guid.Empty; } }
        public global::Vessel InternalVessel { get { return null; } }
        public ReferenceFrame ReferenceFrame { get { return new ReferenceFrame(); } }
        public Flight Flight (ReferenceFrame referenceFrame = null) { return new Flight(); }
    }
}

namespace KRPC.SpaceCenter.Services.Parts
{
    public class Part
    {
        public global::Part InternalPart { get { return null; } }
    }
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
