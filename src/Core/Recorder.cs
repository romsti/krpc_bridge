using System;
using System.Collections.Generic;

namespace KRPC.Bridge.Core
{
    /// <summary>
    /// A ring of the last <see cref="Capacity"/> frames of one transport, preallocated and
    /// reused: the single implementation behind the frame rings of the mod (DynamicsV3 of the
    /// active and of every tracked vessel, DynamicsExtV1, AeroActuatorV1, AttitudeV1 and its
    /// lateral-force frames).
    ///
    /// PURE: System only (build/tests/core tests it against a verbatim copy of the
    /// LinkedList rings it replaces).
    ///
    /// A frame is a run of doubles whose offset 2 is its tick (every transport of the mod puts
    /// it there). Writing one costs no allocation once the ring has wrapped: the writer asks
    /// for a buffer (<see cref="Reserve"/>, zeroed like a new array), fills it, and commits it.
    /// A slot's buffer is reused for every frame that lands in it, and grows only when a frame
    /// is longer than any it held before.
    ///
    /// Reading (<see cref="ReadSince"/>) returns exactly what the LinkedList rings returned:
    /// the six-value history header (protocol, schema, frame_count, oldest_tick, newest_tick,
    /// dropped_before) then each frame newer than since_tick, length-prefixed, oldest first, at
    /// most max_frames clamped to 1..128. The first frame newer than since_tick is found by
    /// binary search while the ticks of the ring never decrease (every transport: one frame per
    /// tick of its own counter); a push that breaks that order switches the ring to the
    /// legacy linear scan until it is cleared, so the answer never depends on the shortcut.
    /// </summary>
    public sealed class Recorder
    {
        /// <summary>Largest max_frames a read honours (the legacy clamp).</summary>
        public const int MaxFramesPerRead = 128;

        readonly int capacity;
        readonly double[][] buffers;
        readonly int[] lengths;
        readonly long[] ticks;
        int head;              // slot of the oldest frame
        int count;
        bool ordered = true;   // ticks never decreased since the last Clear
        int reservedLength = -1;
        int reservedSlot = -1;

        /// <param name="capacity">Frames kept (the oldest goes when a new one is committed).</param>
        public Recorder (int capacity)
        {
            if (capacity < 1)
                throw new ArgumentOutOfRangeException (nameof (capacity));
            this.capacity = capacity;
            buffers = new double[capacity][];
            lengths = new int[capacity];
            ticks = new long[capacity];
        }

        /// <summary>Frames kept at most.</summary>
        public int Capacity {
            get { return capacity; }
        }

        /// <summary>Frames in the ring now (the legacy history.Count).</summary>
        public int Count {
            get { return count; }
        }

        /// <summary>True while every committed tick was &gt;= the one before (binary search in use).</summary>
        public bool Ordered {
            get { return ordered; }
        }

        /// <summary>
        /// The tick of a frame as the legacy rings read it: offset 2 truncated to a long, 0 for a
        /// frame shorter than 3 values.
        /// </summary>
        public static long FrameTick (double[] frame, int length)
        {
            if (frame == null || length < 3)
                return 0;
            return (long)frame [2];
        }

        /// <summary>
        /// A zeroed buffer of at least <paramref name="length"/> values for the next frame
        /// (exactly what new double[length] gives). Nothing changes in the ring until
        /// <see cref="Commit"/>; the frames it holds stay readable meanwhile. A second Reserve
        /// before Commit discards the first.
        /// </summary>
        public double[] Reserve (int length)
        {
            if (length < 0)
                throw new ArgumentOutOfRangeException (nameof (length));
            int slot = count < capacity ? (head + count) % capacity : head;
            if (count == capacity) {
                // The slot about to be overwritten still holds the oldest frame, which stays
                // readable until Commit: write into a spare buffer instead.
                slot = -1;
            }
            double[] buffer;
            if (slot >= 0) {
                buffer = buffers [slot];
                if (buffer == null || buffer.Length < length) {
                    buffer = new double[RoundUp (length)];
                    buffers [slot] = buffer;
                } else {
                    Array.Clear (buffer, 0, length);
                }
            } else {
                buffer = Spare (length);
            }
            reservedSlot = slot;
            reservedLength = length;
            return buffer;
        }

        // When the ring is full, the new frame is written into a spare buffer; at Commit the
        // spare takes the oldest slot and the oldest slot's buffer becomes the next spare.
        double[] spare;

        double[] Spare (int length)
        {
            if (spare == null || spare.Length < length)
                spare = new double[RoundUp (length)];
            else
                Array.Clear (spare, 0, length);
            return spare;
        }

        /// <summary>
        /// Commits the frame of the last <see cref="Reserve"/> (its first <paramref name="length"/>
        /// values; length &lt;= the reserved length). When the ring is full, the oldest frame goes.
        /// </summary>
        public void Commit (int length)
        {
            if (reservedLength < 0)
                throw new InvalidOperationException ("Commit without Reserve");
            if (length < 0 || length > reservedLength)
                throw new ArgumentOutOfRangeException (nameof (length));
            int slot;
            if (reservedSlot >= 0) {
                slot = reservedSlot;
                count++;
            } else {
                // Full: the spare buffer becomes the newest frame, the oldest slot's buffer the spare.
                slot = head;
                var old = buffers [slot];
                buffers [slot] = spare;
                spare = old;
                head = (head + 1) % capacity;
            }
            lengths [slot] = length;
            long tick = FrameTick (buffers [slot], length);
            if (count > 1 || reservedSlot < 0) {
                int previous = (slot - 1 + capacity) % capacity;
                if (tick < ticks [previous])
                    ordered = false;
            }
            ticks [slot] = tick;
            reservedLength = -1;
            reservedSlot = -1;
        }

        /// <summary>Copies a finished frame in (Reserve + copy + Commit). The array is not kept.</summary>
        public void Push (double[] frame)
        {
            if (frame == null)
                throw new ArgumentNullException (nameof (frame));
            var buffer = Reserve (frame.Length);
            Array.Copy (frame, buffer, frame.Length);
            Commit (frame.Length);
        }

        /// <summary>Copies a list in as one frame (Reserve + CopyTo + Commit). The list is not kept.</summary>
        public void Push (List<double> frame)
        {
            if (frame == null)
                throw new ArgumentNullException (nameof (frame));
            var buffer = Reserve (frame.Count);
            frame.CopyTo (buffer, 0);
            Commit (frame.Count);
        }

        /// <summary>Forgets every frame (the buffers are kept for reuse).</summary>
        public void Clear ()
        {
            head = 0;
            count = 0;
            ordered = true;
            reservedLength = -1;
            reservedSlot = -1;
        }

        int Slot (int index)
        {
            return (head + index) % capacity;
        }

        /// <summary>Tick of the oldest frame (0 when empty, like the legacy rings).</summary>
        public long OldestTick {
            get { return count == 0 ? 0 : ticks [head]; }
        }

        /// <summary>Tick of the newest frame (0 when empty).</summary>
        public long NewestTick {
            get { return count == 0 ? 0 : ticks [Slot (count - 1)]; }
        }

        /// <summary>Length of the newest frame (0 when empty).</summary>
        public int NewestLength {
            get { return count == 0 ? 0 : lengths [Slot (count - 1)]; }
        }

        /// <summary>The newest frame as a new list (empty when the ring is), the legacy ReadLatest.</summary>
        public List<double> CopyNewest ()
        {
            if (count == 0)
                return new List<double> ();
            int slot = Slot (count - 1);
            return CopyOf (buffers [slot], lengths [slot]);
        }

        /// <summary>The newest frame as a new array (empty when the ring is).</summary>
        public double[] NewestArray ()
        {
            if (count == 0)
                return new double[0];
            int slot = Slot (count - 1);
            var copy = new double[lengths [slot]];
            Array.Copy (buffers [slot], copy, copy.Length);
            return copy;
        }

        /// <summary>Value i of the newest frame (no copy). The caller checks the length.</summary>
        public double NewestValue (int i)
        {
            int slot = Slot (count - 1);
            return buffers [slot] [i];
        }

        static List<double> CopyOf (double[] buffer, int length)
        {
            var list = new List<double> (length);
            for (int i = 0; i < length; i++)
                list.Add (buffer [i]);
            return list;
        }

        /// <summary>
        /// The legacy history read: header [protocol, schema, frame_count, oldest_tick,
        /// newest_tick, dropped_before], then frame_length and frame values for each frame newer
        /// than sinceTick, oldest first, at most maxFrames clamped to 1..128.
        /// </summary>
        public List<double> ReadSince (int protocol, int schema, int sinceTick, int maxFrames,
            int capacityHintPerFrame = 128)
        {
            int limit = Math.Min (MaxFramesPerRead, Math.Max (1, maxFrames));
            long oldestTick = OldestTick;
            long newestTick = NewestTick;
            bool droppedBefore = sinceTick > 0 && oldestTick > 0 && sinceTick < oldestTick - 1;

            int first = ordered ? FirstNewer (sinceTick) : 0;
            int selectedFirst = -1, selectedCount = 0;
            // Same selection as the legacy scan: the first `limit` frames whose tick > sinceTick.
            // Ordered: they are the run starting at `first`. Otherwise: scan.
            int[] scattered = null;
            if (ordered) {
                selectedFirst = first;
                selectedCount = Math.Min (limit, count - first);
                if (selectedCount < 0)
                    selectedCount = 0;
            } else {
                scattered = new int[Math.Min (limit, count)];
                for (int i = 0; i < count && selectedCount < limit; i++)
                    if (ticks [Slot (i)] > sinceTick)
                        scattered [selectedCount++] = i;
            }

            int total = 6;
            for (int k = 0; k < selectedCount; k++)
                total += 1 + lengths [Slot (scattered != null ? scattered [k] : selectedFirst + k)];
            var output = new List<double> (Math.Max (total, 6 + selectedCount * capacityHintPerFrame));
            output.Add (protocol);
            output.Add (schema);
            output.Add (selectedCount);
            output.Add (oldestTick);
            output.Add (newestTick);
            output.Add (droppedBefore ? 1.0 : 0.0);
            for (int k = 0; k < selectedCount; k++) {
                int slot = Slot (scattered != null ? scattered [k] : selectedFirst + k);
                int length = lengths [slot];
                var buffer = buffers [slot];
                output.Add (length);
                for (int i = 0; i < length; i++)
                    output.Add (buffer [i]);
            }
            return output;
        }

        /// <summary>Index (0 = oldest) of the first frame whose tick &gt; sinceTick; Count if none.</summary>
        int FirstNewer (long sinceTick)
        {
            int lo = 0, hi = count;
            while (lo < hi) {
                int mid = lo + ((hi - lo) >> 1);
                if (ticks [Slot (mid)] > sinceTick)
                    hi = mid;
                else
                    lo = mid + 1;
            }
            return lo;
        }

        static int RoundUp (int length)
        {
            // Room for a few more rows without a new buffer at every topology change.
            int rounded = (length + 63) & ~63;
            return rounded < length ? length : rounded;
        }
    }
}
