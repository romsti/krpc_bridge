using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// Typed errors (refactor R1, plan 2.5): a refusal of the bridge says WHY in a code a client
    /// can branch on, at the start of the exception message: "[CODE] text". The exception TYPE
    /// is unchanged (kRPC maps it: InvalidOperationException to RuntimeError, ArgumentException
    /// to ValueError), and so is the text after the code, so a client that matched a text still
    /// does. A message without a code is an invalid argument (a value out of its documented
    /// range, a malformed list).
    ///
    /// PURE: System only (build/tests/core).
    /// </summary>
    public static class BridgeErrors
    {
        /// <summary>A mode, an option or a bit the build does not support (refused, not degraded).</summary>
        public const string UnsupportedMode = "UNSUPPORTED_MODE";

        /// <summary>The token was valid and its lease ran out (re-arm or re-acquire).</summary>
        public const string TokenExpired = "TOKEN_EXPIRED";

        /// <summary>The token was never valid, was released, or was replaced by a newer arming.</summary>
        public const string TokenInvalid = "TOKEN_INVALID";

        /// <summary>No loaded vessel to act on (no active vessel, vessel unloaded or packed).</summary>
        public const string NotLoaded = "NOT_LOADED";

        /// <summary>A state that clears by itself (topology changing, transforms not ready): retry.</summary>
        public const string Transient = "TRANSIENT";

        /// <summary>Another owner holds what is asked (v2 control, an arming, the vessel limit).</summary>
        public const string Busy = "BUSY";

        /// <summary>The named part, module, vessel or layout does not exist (or is not tracked).</summary>
        public const string NotFound = "NOT_FOUND";

        /// <summary>Every code, for the docs and the client.</summary>
        public static readonly string[] Codes = {
            UnsupportedMode, TokenExpired, TokenInvalid, NotLoaded, Transient, Busy, NotFound
        };

        /// <summary>"[CODE] message".</summary>
        public static string Tag (string code, string message)
        {
            return "[" + code + "] " + message;
        }

        /// <summary>The code at the start of a message, or "" (the parse a client does).</summary>
        public static string CodeOf (string message)
        {
            if (string.IsNullOrEmpty (message) || message [0] != '[')
                return string.Empty;
            int end = message.IndexOf (']');
            if (end < 2)
                return string.Empty;
            string code = message.Substring (1, end - 1);
            foreach (var c in Codes)
                if (c == code)
                    return code;
            return string.Empty;
        }
    }

    /// <summary>
    /// The last tokens that stopped being valid and why (TokenExpired or TokenInvalid), so a
    /// refusal can say TOKEN_EXPIRED rather than TOKEN_INVALID. Bounded (the oldest is
    /// forgotten). Pure.
    /// </summary>
    public sealed class TokenMemory
    {
        readonly string[] tokens;
        readonly string[] codes;
        int next;

        public TokenMemory (int capacity = 16)
        {
            tokens = new string[Math.Max (1, capacity)];
            codes = new string[tokens.Length];
        }

        /// <summary>Records why a token stopped being valid (a null token is ignored).</summary>
        public void Remember (string token, string code)
        {
            if (string.IsNullOrEmpty (token))
                return;
            for (int i = 0; i < tokens.Length; i++)
                if (tokens [i] == token) {
                    codes [i] = code;
                    return;
                }
            tokens [next] = token;
            codes [next] = code;
            next = (next + 1) % tokens.Length;
        }

        /// <summary>TokenExpired when this token was recorded as expired, TokenInvalid otherwise.</summary>
        public string CodeOf (string token)
        {
            if (!string.IsNullOrEmpty (token))
                for (int i = 0; i < tokens.Length; i++)
                    if (tokens [i] == token)
                        return codes [i] ?? BridgeErrors.TokenInvalid;
            return BridgeErrors.TokenInvalid;
        }
    }
}
