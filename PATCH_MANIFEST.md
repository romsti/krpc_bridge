# RETURN_CHAIN_OBSERVABILITY A2 — krpc_bridge

Fix validated from flight `98f569b1_20260917_195605`.

- `DynamicsV3.HistoryCapacity`: 300 -> 1000 (~20 s at 50 Hz).
- `AeroActuatorV1.HistoryCapacity`: 300 -> 1000 (~20 s at 50 Hz).

Reason: the flight showed one 27-tick loss after a >6 s GNC stall, despite otherwise healthy history capture.
Rebuild the bridge DLL after applying this patch.
