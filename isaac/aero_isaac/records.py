"""The layout of the environment's per-episode records (M9): what env.py
writes for every finished episode and what the training dashboard
(train.py) reads. Kept apart from env.py so it can be read without starting
Isaac Sim (the dashboard's tests)."""
from __future__ import annotations

from aero_isaac.detector_sim import KNOBS
from aero_isaac.reward import PARTS

EPISODE_FIELDS = (
    "valid", "env", "severity", "rotor", "onset_s", "ramp_s", "termination", "outcome",
    "touchdown_speed_m_s", "max_tilt_deg", "waypoints_reached", "duration_s", "peak_hspeed_m_s",
    "mean_motor_command", "mission_progress",
    *(f"r_{k}" for k in PARTS), "return",
    "policy_landed", "detected_s", "reacted_s", "speed_cmd_before", "speed_cmd_after",
    "alt_cmd_before", "alt_cmd_after", "min_alt_cmd", "false_alarms", "healthy_flight_s",
    "mass_scale", "wind_n", "noise_scale", *(f"det_{k}" for k in KNOBS),
)
INT_FIELDS = {"env", "rotor", "termination", "outcome", "waypoints_reached", "policy_landed", "false_alarms"}
RECORD_CAPACITY = 1 << 17
# One row per 0.1 s tick of the example drones' flights.
TRACE_FIELDS = ("t", "altitude_m", "hspeed_m_s", "speed_scale", "altitude_offset_m", "land",
                "det_p_fault", "det_severity", "true_severity")
