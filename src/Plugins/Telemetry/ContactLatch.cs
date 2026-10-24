using System;

namespace KRPC.Bridge.Telemetry
{
    /// <summary>
    /// What ContactV1 latches for ONE vessel, physics step by physics step (refactor lot 3, line
    /// 3.2; audit C, R7): the master tick of the first contact of each wheel or leg and of the
    /// vessel, of every leg on the ground, of Vessel.Landed and of the LANDED situation, so a
    /// client that reads at its own rate (50 Hz at best, with holes) still learns the exact step.
    ///
    /// PURE: System only, no allocation after construction (arrays sized <see cref="MaxWheels"/>),
    /// tested by build/tests/contact. The KSP side (<see cref="ContactWatch"/>) fills the inputs
    /// of one step, calls <see cref="Step"/>, then publishes the transitions it reports.
    ///
    /// DEFINITIONS (KSP 1.12.5, decompiled; see ContactWatch for the members):
    ///   * the vessel is IN CONTACT when a wheel is grounded (VehicleBase.WheelState.grounded, the
    ///     wheel collider's ground hit) or a part has GroundContact or PermanentGroundContact (the
    ///     flags Vessel.checkLanded reads). A collision with another vessel alone (the arms of a
    ///     catch tower, debris) is not a contact: it is latched apart (<see cref="CollisionTick"/>);
    ///   * an EPISODE starts at a step in contact after a step without (a rising edge). The
    ///     per-wheel first ticks and <see cref="AllLegsTick"/> belong to the current episode and
    ///     restart at the next one; <see cref="FirstEpisodeTick"/> keeps the very first;
    ///   * every other latch is the LAST rising edge seen (-1: none seen by this watch). A watch
    ///     that starts in contact (a booster on the pad) sees no rising edge until it has flown.
    /// </summary>
    internal sealed class ContactLatch
    {
        /// <summary>Wheels (ModuleWheelBase) followed per vessel; more are ignored.</summary>
        internal const int MaxWheels = 16;

        /// <summary>Bits of <see cref="FirstContactSource"/>.</summary>
        internal const int SourceWheel = 1;            // a wheel grounded (WheelState.grounded)
        internal const int SourcePart = 2;             // a part with no wheel module in ground contact
        internal const int SourceWheelPart = 4;        // the GroundContact flag of a part that carries a wheel

        /// <summary>Transitions of one step (bits of <see cref="Transitions"/>).</summary>
        internal const int EventFirstContact = 1;
        internal const int EventContactLost = 2;
        internal const int EventAllLegs = 4;
        internal const int EventLanded = 8;
        internal const int EventSituationLanded = 16;
        internal const int EventCollision = 32;
        internal const int EventWheel = 64;           // at least one wheel's first ground of the episode

        // ------------------------------------------------------------ inputs of one step
        internal int WheelCount;
        internal readonly bool[] Grounded = new bool[MaxWheels];
        internal readonly bool[] IsLeg = new bool[MaxWheels];
        internal readonly bool[] Deployed = new bool[MaxWheels];
        internal readonly uint[] WheelPartPid = new uint[MaxWheels];
        internal readonly int[] WheelOrdinal = new int[MaxWheels];
        internal readonly double[] CompressionRate = new double[MaxWheels];

        // ------------------------------------------------------------ per-wheel latches
        internal readonly long[] FirstGroundedTick = new long[MaxWheels];
        internal readonly double[] FirstGroundedUt = new double[MaxWheels];
        internal readonly double[] FirstGroundedRate = new double[MaxWheels];
        internal readonly long[] WheelTransitions = new long[MaxWheels];
        internal readonly long[] LastChangeTick = new long[MaxWheels];
        internal readonly bool[] WasGrounded = new bool[MaxWheels];
        internal readonly bool[] WheelSeen = new bool[MaxWheels];
        internal readonly bool[] NewlyGrounded = new bool[MaxWheels];

        // Scratch of the wheel snapshot (no allocation).
        readonly uint[] oldPid = new uint[MaxWheels];
        readonly int[] oldOrdinal = new int[MaxWheels];
        readonly long[] oldFirst = new long[MaxWheels];
        readonly double[] oldFirstUt = new double[MaxWheels];
        readonly double[] oldFirstRate = new double[MaxWheels];
        readonly long[] oldTransitions = new long[MaxWheels];
        readonly long[] oldLastChange = new long[MaxWheels];
        readonly bool[] oldWas = new bool[MaxWheels];

        // ------------------------------------------------------------ vessel latches
        internal long WatchStartTick = -1;
        internal long LastStepTick = -1;
        internal bool StartedInContact;
        internal bool ContactNow;
        internal bool Stepped;
        internal long Episodes;
        internal long FirstEpisodeTick = -1;
        internal long AirborneSinceTick = -1;
        internal long FirstContactTick = -1;
        internal double FirstContactUt = double.NaN;
        internal int FirstContactSource;
        internal uint FirstContactPartPid;
        internal double FirstContactVerticalSpeed = double.NaN;
        internal double FirstContactHorizontalSpeed = double.NaN;
        internal double FirstContactTilt = double.NaN;
        internal long AllLegsTick = -1;
        internal long LandedTick = -1;
        internal long SituationLandedTick = -1;
        internal long LostTick = -1;
        internal long CollisionTick = -1;
        internal int Transitions;

        bool wasLanded;
        bool wasSituationLanded;
        bool wasColliding;

        internal ContactLatch ()
        {
            Reset ();
        }

        /// <summary>Forgets everything (a new watch).</summary>
        internal void Reset ()
        {
            WheelCount = 0;
            for (int i = 0; i < MaxWheels; i++) {
                Grounded [i] = IsLeg [i] = Deployed [i] = false;
                WheelPartPid [i] = 0;
                WheelOrdinal [i] = 0;
                CompressionRate [i] = double.NaN;
                ClearWheel (i);
                WheelTransitions [i] = 0;
                LastChangeTick [i] = -1;
                WasGrounded [i] = false;
                WheelSeen [i] = false;
            }
            WatchStartTick = LastStepTick = -1;
            StartedInContact = ContactNow = Stepped = false;
            Episodes = 0;
            FirstEpisodeTick = AirborneSinceTick = FirstContactTick = -1;
            FirstContactUt = FirstContactVerticalSpeed = FirstContactHorizontalSpeed = FirstContactTilt = double.NaN;
            FirstContactSource = 0;
            FirstContactPartPid = 0;
            AllLegsTick = LandedTick = SituationLandedTick = LostTick = CollisionTick = -1;
            Transitions = 0;
            wasLanded = wasSituationLanded = wasColliding = false;
        }

        void ClearWheel (int i)
        {
            FirstGroundedTick [i] = -1;
            FirstGroundedUt [i] = double.NaN;
            FirstGroundedRate [i] = double.NaN;
            NewlyGrounded [i] = false;
        }

        int snapshotCount;

        /// <summary>
        /// Before a new wheel scan: keeps the current wheels (identity and latches) aside. Then the
        /// caller sets <see cref="WheelCount"/>, <see cref="WheelPartPid"/>, <see cref="WheelOrdinal"/>
        /// and the static inputs of the new set, and calls <see cref="RemapFromSnapshot"/>.
        /// </summary>
        internal void SnapshotWheels ()
        {
            snapshotCount = Math.Min (WheelCount, MaxWheels);
            for (int i = 0; i < snapshotCount; i++) {
                oldPid [i] = WheelPartPid [i];
                oldOrdinal [i] = WheelOrdinal [i];
                oldFirst [i] = FirstGroundedTick [i];
                oldFirstUt [i] = FirstGroundedUt [i];
                oldFirstRate [i] = FirstGroundedRate [i];
                oldTransitions [i] = WheelTransitions [i];
                oldLastChange [i] = LastChangeTick [i];
                oldWas [i] = WasGrounded [i];
            }
        }

        /// <summary>
        /// After a new wheel scan: a wheel keeps its latches when it was in the snapshot under the
        /// same (part persistentId, module ordinal); a new wheel starts empty (no edge until it
        /// changes: its first state is taken as it is).
        /// </summary>
        internal void RemapFromSnapshot ()
        {
            for (int i = 0; i < WheelCount && i < MaxWheels; i++) {
                int from = -1;
                for (int k = 0; k < snapshotCount; k++)
                    if (oldPid [k] == WheelPartPid [i] && oldOrdinal [k] == WheelOrdinal [i]) {
                        from = k;
                        break;
                    }
                NewlyGrounded [i] = false;
                if (from < 0) {
                    ClearWheel (i);
                    WheelTransitions [i] = 0;
                    LastChangeTick [i] = -1;
                    WasGrounded [i] = false;
                    WheelSeen [i] = false;          // its first observed state is not an edge
                    continue;
                }
                FirstGroundedTick [i] = oldFirst [from];
                FirstGroundedUt [i] = oldFirstUt [from];
                FirstGroundedRate [i] = oldFirstRate [from];
                WheelTransitions [i] = oldTransitions [from];
                LastChangeTick [i] = oldLastChange [from];
                WasGrounded [i] = oldWas [from];
                WheelSeen [i] = true;
            }
            snapshotCount = 0;
        }

        /// <summary>
        /// One physics step of the vessel. The wheel inputs are set before the call; the vessel's
        /// are its arguments. Sets <see cref="Transitions"/> (and <see cref="NewlyGrounded"/> per
        /// wheel) for the caller to publish. A second call for the same tick does nothing.
        /// </summary>
        /// <param name="tick">Master tick of the step.</param>
        /// <param name="ut">UT of the step (s).</param>
        /// <param name="groundPartsNoWheel">Parts without a wheel module in ground contact.</param>
        /// <param name="groundPartsWheel">Parts carrying a wheel module whose GroundContact is set.</param>
        /// <param name="firstGroundPartPid">persistentId of the first part found in ground contact (0 none).</param>
        /// <param name="partsColliding">Parts with at least one current collision.</param>
        /// <param name="landed">Vessel.Landed.</param>
        /// <param name="situationLanded">Vessel.situation is LANDED or SPLASHED.</param>
        /// <param name="verticalSpeed">Vessel.verticalSpeed (m/s, up positive).</param>
        /// <param name="horizontalSpeed">Vessel.horizontalSrfSpeed (m/s).</param>
        /// <param name="tilt">Angle between the nose and the local vertical (rad).</param>
        internal void Step (long tick, double ut, int groundPartsNoWheel, int groundPartsWheel, uint firstGroundPartPid,
            int partsColliding, bool landed, bool situationLanded, double verticalSpeed, double horizontalSpeed,
            double tilt)
        {
            Transitions = 0;
            for (int i = 0; i < MaxWheels; i++)
                NewlyGrounded [i] = false;
            if (Stepped && tick == LastStepTick)
                return;
            int n = Math.Min (WheelCount, MaxWheels);
            int wheelsGrounded = 0;
            uint firstWheelPid = 0;
            for (int i = 0; i < n; i++)
                if (Grounded [i]) {
                    wheelsGrounded++;
                    if (firstWheelPid == 0)
                        firstWheelPid = WheelPartPid [i];
                }
            bool contact = wheelsGrounded > 0 || groundPartsNoWheel > 0 || groundPartsWheel > 0;

            if (!Stepped) {
                // First step of the watch: what it finds is its start, not a transition.
                Stepped = true;
                WatchStartTick = tick;
                StartedInContact = contact;
                if (!contact)
                    AirborneSinceTick = tick;
                for (int i = 0; i < n; i++) {
                    WasGrounded [i] = Grounded [i];
                    WheelSeen [i] = true;
                    WheelTransitions [i] = 0;
                    LastChangeTick [i] = -1;
                }
                ContactNow = contact;
                wasLanded = landed;
                wasSituationLanded = situationLanded;
                wasColliding = partsColliding > 0;
                LastStepTick = tick;
                return;
            }

            // Vessel contact: rising edge = a new episode.
            if (contact && !ContactNow) {
                Episodes++;
                if (FirstEpisodeTick < 0)
                    FirstEpisodeTick = tick;
                FirstContactTick = tick;
                FirstContactUt = ut;
                FirstContactSource = (wheelsGrounded > 0 ? SourceWheel : 0) |
                    (groundPartsNoWheel > 0 ? SourcePart : 0) | (groundPartsWheel > 0 ? SourceWheelPart : 0);
                FirstContactPartPid = firstWheelPid != 0 ? firstWheelPid : firstGroundPartPid;
                FirstContactVerticalSpeed = verticalSpeed;
                FirstContactHorizontalSpeed = horizontalSpeed;
                FirstContactTilt = tilt;
                AllLegsTick = -1;
                for (int i = 0; i < n; i++)
                    ClearWheel (i);
                Transitions |= EventFirstContact;
            } else if (!contact && ContactNow) {
                LostTick = tick;
                AirborneSinceTick = tick;
                Transitions |= EventContactLost;
            }
            ContactNow = contact;

            // Wheels: edges, and the first ground of each in the current episode.
            int legs = 0, legsGrounded = 0;
            for (int i = 0; i < n; i++) {
                if (!WheelSeen [i]) {
                    WasGrounded [i] = Grounded [i];
                    WheelSeen [i] = true;
                }
                if (Grounded [i] != WasGrounded [i]) {
                    WheelTransitions [i]++;
                    LastChangeTick [i] = tick;
                    WasGrounded [i] = Grounded [i];
                }
                if (contact && Grounded [i] && FirstGroundedTick [i] < 0 && Episodes > 0) {
                    FirstGroundedTick [i] = tick;
                    FirstGroundedUt [i] = ut;
                    FirstGroundedRate [i] = CompressionRate [i];
                    NewlyGrounded [i] = true;
                    Transitions |= EventWheel;
                }
                if (IsLeg [i] && Deployed [i]) {
                    legs++;
                    if (Grounded [i])
                        legsGrounded++;
                }
            }
            if (contact && Episodes > 0 && AllLegsTick < 0 && legs > 0 && legsGrounded == legs) {
                AllLegsTick = tick;
                Transitions |= EventAllLegs;
            }

            // Vessel.Landed, the LANDED situation and collisions: last rising edges.
            if (landed && !wasLanded) {
                LandedTick = tick;
                Transitions |= EventLanded;
            }
            wasLanded = landed;
            if (situationLanded && !wasSituationLanded) {
                SituationLandedTick = tick;
                Transitions |= EventSituationLanded;
            }
            wasSituationLanded = situationLanded;
            bool colliding = partsColliding > 0;
            if (colliding && !wasColliding) {
                CollisionTick = tick;
                Transitions |= EventCollision;
            }
            wasColliding = colliding;
            LastStepTick = tick;
        }
    }
}
