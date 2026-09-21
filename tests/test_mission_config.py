"""M3 task 3: the shipped mission config (configs/missions/square_circuit.yaml)
and its validator (aero_bridge.mission_executor.validate_mission_config).

No simulator needed -- this is pure YAML-in, dict-out validation.
"""
from pathlib import Path

import pytest

from aero_bridge.mission_executor import (
    MissionConfigError, load_mission, validate_mission_config,
)

REPO = Path(__file__).resolve().parent.parent
SQUARE_CIRCUIT = REPO / "configs" / "missions" / "square_circuit.yaml"


def _valid_mission(**overrides):
    mission = dict(
        mission_id="test_mission", schema_version="1", altitude_m=5.0,
        acceptance_radius_m=0.75, hold_time_s=2.0, final_hover_s=3.0, timeout_s=120.0,
        waypoints=[[0.0, 0.0], [10.0, 0.0]],
        geofence=dict(x_min_m=-5.0, x_max_m=15.0, y_min_m=-5.0, y_max_m=15.0,
                       z_min_m=-10.0, z_max_m=-0.5),
    )
    mission.update(overrides)
    return mission


def test_mission_yaml_validates():
    """The shipped mission parses and its waypoints, geofence and limits are
    self-consistent (geofence contains all waypoints and the flight altitude)."""
    mission = load_mission(SQUARE_CIRCUIT)
    assert mission["mission_id"] == "square_circuit"
    assert len(mission["waypoints"]) >= 3


def test_valid_mission_passes():
    validate_mission_config(_valid_mission())


def test_missing_field_fails():
    mission = _valid_mission()
    del mission["acceptance_radius_m"]
    with pytest.raises(MissionConfigError, match="acceptance_radius_m"):
        validate_mission_config(mission)


def test_waypoint_outside_geofence_fails():
    mission = _valid_mission(waypoints=[[0.0, 0.0], [500.0, 0.0]])
    with pytest.raises(MissionConfigError, match="waypoint"):
        validate_mission_config(mission)


def test_altitude_outside_geofence_fails():
    mission = _valid_mission(altitude_m=50.0)
    with pytest.raises(MissionConfigError, match="altitude"):
        validate_mission_config(mission)


def test_inverted_geofence_fails():
    mission = _valid_mission()
    mission["geofence"]["x_min_m"] = 100.0
    with pytest.raises(MissionConfigError, match="x_min_m"):
        validate_mission_config(mission)
