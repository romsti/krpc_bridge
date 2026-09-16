from __future__ import annotations

import unittest

from dynamics_v3 import (
    HEADER_STRIDE,
    STATE_STRIDE,
    decode_history,
    decode_snapshot,
)


class DynamicsV3DecoderTests(unittest.TestCase):
    def make_frame(self, tick=12):
        state = [float("nan")] * STATE_STRIDE
        state[25] = 239.5
        header = [
            3, 1, tick, 1234.5, 0.02, 42, 7,
            11, 11, tick, 2,
            (1 << 8) | (1 << 21),
            STATE_STRIDE,
            1, 20,
            1, 19,
            1, 10,
            300, 4, 0,
        ]
        engine = [1, 0, 1, .5, .49, 100, 110, 100, 1, 49,
                  1, 2, 2, .1, 1, 300, 1, 1, 0, 0]
        gimbal = [1, 0, 0, 0, 100, 5, 5, 5, 5, 5, 1, 10, 1, 2, 0, 1, 1, 1.2, 2.2]
        transform = [1, 0, 0, 1, 0, 0, -2, 0, 0, 1]
        return header + state + engine + gimbal + transform

    def test_snapshot_shape_and_rows(self):
        d = decode_snapshot(self.make_frame())
        self.assertEqual(d["physics_tick"], 12)
        self.assertEqual(d["state"]["mass_tonnes"], 239.5)
        self.assertEqual(len(d["engines"]), 1)
        self.assertEqual(len(d["gimbals"]), 1)
        self.assertEqual(len(d["thrust_transforms"]), 1)
        self.assertIn("mass", d["capabilities"])
        self.assertIn("actuators", d["capabilities"])

    def test_history_length_prefix(self):
        f1 = self.make_frame(12)
        f2 = self.make_frame(13)
        payload = [3, 1, 2, 12, 13, 0, len(f1), *f1, len(f2), *f2]
        h = decode_history(payload)
        self.assertEqual([x["physics_tick"] for x in h["frames"]], [12, 13])


if __name__ == "__main__":
    unittest.main()
