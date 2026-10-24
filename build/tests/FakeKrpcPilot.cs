// -----------------------------------------------------------------------------------
// A stand-in for kRPC 0.6's KRPC.SpaceCenter.PilotAddon, for the fly-by-wire chain tests
// of AttitudeWrite (M3). The real PilotAddon.Fly registers, once per vessel:
//
//     Action<FlightCtrlState> action = delegate (FlightCtrlState s) { OnFlyByWire (vessel, s); };
//     vessel.OnFlyByWire = (FlightInputCallback)Delegate.Combine (vessel.OnFlyByWire,
//         new FlightInputCallback (action.Invoke));
//
// (decompiled from the installed KRPC.SpaceCenter.dll 0.6.0). Register reproduces that
// shape - a closure compiled inside PilotAddon, wrapped through Invoke - so that
// AttitudeWrite.IsKrpcPilot is tested on the very pattern it must recognise.
// -----------------------------------------------------------------------------------

using System;

namespace KRPC.SpaceCenter
{
    public sealed class PilotAddon
    {
        internal delegate void Callback (object state);

        internal static int Calls;

        internal static Callback Register (object vessel)
        {
            Action<object> action = delegate (object s) {
                OnFlyByWire (vessel, s);
            };
            return new Callback (action.Invoke);
        }

        /// <summary>A method group of PilotAddon itself (another plausible kRPC shape).</summary>
        internal static Callback RegisterDirect ()
        {
            return new Callback (Direct);
        }

        static void Direct (object s)
        {
            Calls++;
        }

        static void OnFlyByWire (object vessel, object s)
        {
            Calls++;
        }
    }
}
