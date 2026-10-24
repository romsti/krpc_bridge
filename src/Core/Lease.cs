using System;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// One lease: who holds it (owner, token) and until when. The single implementation behind
    /// every lease of the mod (per-engine throttle and per-gimbal leases, the v2 controller,
    /// AttitudeV1's authority on the active and on a non-active vessel, the AttitudeV1 arming,
    /// the DynamicsExtV1 arming, the tracked-vessel registration).
    ///
    /// PURE: System only. The clock is passed in, never read here, so a caller keeps the exact
    /// moment it samples Time.realtimeSinceStartup, and build/tests/core can drive it.
    ///
    /// Two expiries, both optional:
    ///   * a WALL guard, Time.realtimeSinceStartup, in float with the arithmetic the leases
    ///     always had (now + seconds, stored as a float; expired when now &gt;= that), so a
    ///     migrated lease expires on exactly the same frame, bit for bit;
    ///   * a PHYSICS-STEP budget, master ticks of GameClock: expired when the tick reaches
    ///     <see cref="ExpiresAtTick"/>. <see cref="NoTickLimit"/> by default: every lease
    ///     migrated in refactor R1 keeps its wall expiry only (byte-identical behaviour); a
    ///     new contract opts in (a lease of N steps does not depend on the game/wall ratio,
    ///     the wall guard still catches a frozen game or client).
    /// </summary>
    public sealed class Lease
    {
        /// <summary>No physics-step limit.</summary>
        public const long NoTickLimit = long.MaxValue;

        /// <summary>Holder name (free text, may be null).</summary>
        public string Owner;

        /// <summary>Opaque token of the holder (null: the lease is not held).</summary>
        public string Token;

        /// <summary>Wall guard: Time.realtimeSinceStartup at which the lease ends (float, as ever).</summary>
        public float ExpiresAtReal;

        /// <summary>Master tick at which the lease ends; <see cref="NoTickLimit"/> for none.</summary>
        public long ExpiresAtTick = NoTickLimit;

        /// <summary>True while a holder is recorded (expired or not: test <see cref="Expired"/>).</summary>
        public bool Held {
            get { return Token != null; }
        }

        /// <summary>
        /// Takes the lease for a holder: owner, token, wall guard now + seconds (float), and the
        /// step budget (ticks &lt;= 0 or NoTickLimit: none).
        /// </summary>
        public void Grant (string owner, string token, float nowReal, float seconds,
            long nowTick = 0, long ticks = NoTickLimit)
        {
            Owner = owner;
            Token = token;
            RenewReal (nowReal, seconds);
            SetTicks (nowTick, ticks);
        }

        /// <summary>Wall guard = now + seconds, float arithmetic (the legacy expression).</summary>
        public void RenewReal (float nowReal, float seconds)
        {
            ExpiresAtReal = nowReal + seconds;
        }

        /// <summary>Step budget = now + ticks; ticks &lt;= 0 or NoTickLimit removes it.</summary>
        public void SetTicks (long nowTick, long ticks)
        {
            if (ticks <= 0 || ticks == NoTickLimit || nowTick > NoTickLimit - ticks)
                ExpiresAtTick = NoTickLimit;
            else
                ExpiresAtTick = nowTick + ticks;
        }

        /// <summary>The wall guard alone: now &gt;= ExpiresAtReal (the legacy test).</summary>
        public bool ExpiredReal (float nowReal)
        {
            return nowReal >= ExpiresAtReal;
        }

        /// <summary>Expired by the wall guard, or by the step budget when there is one.</summary>
        public bool Expired (float nowReal, long nowTick)
        {
            return nowReal >= ExpiresAtReal || nowTick >= ExpiresAtTick;
        }

        /// <summary>The wall guard alone: now &lt; ExpiresAtReal (the legacy test).</summary>
        public bool ValidReal (float nowReal)
        {
            return nowReal < ExpiresAtReal;
        }

        /// <summary>Wall seconds left, never negative (float, the legacy expression).</summary>
        public float RemainingReal (float nowReal)
        {
            return Math.Max (0f, ExpiresAtReal - nowReal);
        }

        /// <summary>Steps left before the budget ends (long.MaxValue: no budget).</summary>
        public long RemainingTicks (long nowTick)
        {
            if (ExpiresAtTick == NoTickLimit)
                return NoTickLimit;
            return Math.Max (0L, ExpiresAtTick - nowTick);
        }

        /// <summary>Forgets the holder: owner and token null, wall guard 0, no step budget.</summary>
        public void Clear ()
        {
            Owner = null;
            Token = null;
            ExpiresAtReal = 0f;
            ExpiresAtTick = NoTickLimit;
        }
    }
}
