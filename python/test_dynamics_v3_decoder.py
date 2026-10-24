from __future__ import annotations

import unittest

from dynamics_v3 import (
    STATE_STRIDE_BY_SCHEMA,
    decode_history,
    decode_snapshot,
)


class DynamicsV3DecoderTests(unittest.TestCase):
    def make_frame(self, tick=12, schema=2):
        state_stride = STATE_STRIDE_BY_SCHEMA[schema]
        state = [float("nan")] * state_stride
        state[25] = 239.5
        caps = (1 << 8) | (1 << 21)
        if schema >= 2:
            state[67:70] = [1.0, 2.0, 3.0]
            state[79:82] = [4.0, 5.0, 6.0]
            state[100] = 1234.0
            state[105] = 7.5
            state[107:110] = [10.0, 20.0, 30.0]
            caps |= (1 << 22) | (1 << 23) | (1 << 24) | (1 << 27) | (1 << 28) | (1 << 30)
        header = [
            3, schema, tick, 1234.5, 0.02, 42, 7,
            11, 11, tick, 2,
            caps,
            state_stride,
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

    def test_schema2_snapshot_and_new_quantities(self):
        d = decode_snapshot(self.make_frame())
        self.assertEqual(d["schema"], 2)
        self.assertEqual(d["physics_tick"], 12)
        self.assertEqual(d["state"]["mass_tonnes"], 239.5)
        self.assertEqual(d["state"]["engine_force_body_n_x"], 1.0)
        self.assertEqual(d["state"]["aero_force_body_n_x"], 4.0)
        self.assertEqual(d["state"]["dynamic_pressure_pa"], 1234.0)
        self.assertEqual(d["state"]["aoa_deg"], 7.5)
        self.assertEqual(d["state"]["external_force_residual_world_n_z"], 30.0)
        self.assertIn("realized_engine_wrench", d["capabilities"])
        self.assertIn("krpc_live_aero_force", d["capabilities"])
        self.assertIn("krpc_live_aero_torque", d["capabilities"])
        self.assertIn("external_force_residual", d["capabilities"])
        self.assertIn("realized_engine_force_on_vessel", d["capabilities"])

    def test_schema1_backward_compatibility(self):
        d = decode_snapshot(self.make_frame(schema=1))
        self.assertEqual(d["schema"], 1)
        self.assertEqual(d["state"]["mass_tonnes"], 239.5)
        self.assertNotIn("engine_force_body_n_x", d["state"])

    def test_schema2_history_length_prefix(self):
        f1 = self.make_frame(12, 2)
        f2 = self.make_frame(13, 2)
        payload = [3, 2, 2, 12, 13, 0, len(f1), *f1, len(f2), *f2]
        h = decode_history(payload)
        self.assertEqual(h["schema"], 2)
        self.assertEqual([x["physics_tick"] for x in h["frames"]], [12, 13])

    def test_schema1_history_still_decodes(self):
        f = self.make_frame(12, 1)
        payload = [3, 1, 1, 12, 12, 0, len(f), *f]
        h = decode_history(payload)
        self.assertEqual(h["schema"], 1)
        self.assertEqual(h["frames"][0]["schema"], 1)


if __name__ == "__main__":
    unittest.main()
