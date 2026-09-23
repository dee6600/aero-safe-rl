"""Unit tests for experiments/episode_runner.py's pure logic (M4 tasks 1-3;
M6 task 6's rotor-fault driving logic). No rclpy, no simulator --
EpisodeRunner itself needs a live PX4Interface/GzSimClock and is exercised
by tests/sim/test_two_workers.py (and, for fault injection,
tests/sim/test_episode_runner_fault_injection.py) instead; this file covers
next_reset_tier(), the one piece of EpisodeRunner-adjacent logic that is a
pure function of data. Exceptions: the on_step-before-reset ordering test
and the M6 fault-driving tests below, which need a bare EpisodeRunner
(constructed via object.__new__ to skip __init__'s rclpy Node/PX4Interface/
GzSimClock setup) with everything reset/flight-related faked out, since
control-flow ordering is the whole point of what they test.
"""
import math
import types

from experiments.episode_runner import EpisodeRunner, next_reset_tier
from experiments.episode_schema import TerminationReason
from experiments.fault_schedule import FaultProfile, FaultSpec, FaultType


def test_first_episode_uses_requested_tier():
    """last_termination_reason=None means nothing to escalate from yet."""
    assert next_reset_tier("soft", None) == "soft"
    assert next_reset_tier("hard", None) == "hard"


def test_clean_completion_keeps_requested_tier():
    assert next_reset_tier("soft", TerminationReason.COMPLETED.value) == "soft"
    assert next_reset_tier("medium", "completed") == "medium"


def test_any_non_clean_termination_escalates_to_hard():
    """Every termination_reason other than 'completed' means the previous
    episode did not end in a state soft/medium reset can trust -- escalate
    regardless of which specific reason it was."""
    for reason in (TerminationReason.PREFLIGHT_FAILED.value, TerminationReason.ARM_TIMEOUT.value,
                   TerminationReason.OFFBOARD_REJECTED.value, TerminationReason.OFFBOARD_LOST.value,
                   TerminationReason.HOLD_TIMEOUT.value, TerminationReason.LAND_TIMEOUT.value,
                   TerminationReason.EPISODE_TIMEOUT.value, TerminationReason.WORKER_RESTARTED.value,
                   TerminationReason.ABORTED_ERROR.value):
        assert next_reset_tier("soft", reason) == "hard", f"{reason} should escalate to hard"


def test_already_requesting_hard_stays_hard_after_bad_episode():
    assert next_reset_tier("hard", "aborted_error") == "hard"


def test_escalation_is_independent_of_requested_tier():
    """A bad previous episode escalates to hard whether soft or medium was
    originally requested -- the requested tier only matters when the
    previous episode was clean."""
    assert next_reset_tier("soft", "hold_timeout") == "hard"
    assert next_reset_tier("medium", "hold_timeout") == "hard"


def _fake_driver():
    """Stands in for rl.policy_driver.PolicyDriver: EpisodeRunner only reads
    its provenance and hands it to fly_mission (faked here)."""
    return types.SimpleNamespace(provenance=dict(
        policy_name="nominal", policy_config_digest="none", action_spec_digest="x",
        detector_checkpoint_digest="none"))


def test_run_episode_writes_a_heartbeat_before_starting_a_reset(monkeypatch):
    """Found live (M4 task 7's soak test): a reset writes no heartbeat while
    it runs (heartbeats only happen per flight control tick), so the
    supervisor's health check can see a stale timestamp from before the
    reset started and conclude the worker died mid-reset -- piling a
    redundant restart on top of one already in progress and burning through
    the worker's restart budget on false positives. Fixed by calling
    on_step() with a fresh timestamp right before the reset starts. This
    pins down the ORDER: the heartbeat fires before hard_reset(), not after
    or not at all."""
    calls: list = []

    def fake_hard_reset(spec):
        calls.append("hard_reset")
        return types.SimpleNamespace(tier="hard", wall_duration_s=1.0)

    def fake_fly_mission(node, px4, clock, mission, on_step=None, driver=None):
        calls.append("fly_mission")
        return types.SimpleNamespace(
            termination_reason="completed", n_steps=0, waypoints_reached=0,
            position_rmse_m=0.0, final_position_error_m=0.0,
            t_sim_start_s=0.0, t_sim_end_s=1.0, steps=[])

    monkeypatch.setattr("aero_bridge.reset.hard_reset", fake_hard_reset)
    monkeypatch.setattr("aero_bridge.mission_executor.fly_mission", fake_fly_mission)

    runner = object.__new__(EpisodeRunner)  # skip __init__: no rclpy Node needed for this
    runner.spec = types.SimpleNamespace(instance=0, to_dict=lambda: {})
    runner.run_id = "run_x"
    runner.mission_id = "square_circuit"
    runner.mission = {}
    runner.mission_digest = "digest"
    runner.env_versions_json = "{}"
    runner.feature_version = "unversioned"
    runner.spec_digest = "specdigest"
    runner.enable_rotor_fault = False
    runner.node = object()
    runner.px4 = object()
    runner.clock = object()
    runner.driver = _fake_driver()
    runner.rebuild_after_hard_reset = lambda: calls.append("rebuild")
    runner.logger = types.SimpleNamespace(log_step=lambda *a, **k: None,
                                           write_episode=lambda *a, **k: None)

    def on_step(row):
        calls.append(("on_step", "t_wall_utc" in row))

    runner.run_episode(episode_id="ep_0000", reset_tier="hard", seed=0, on_step=on_step)

    assert calls[0] == ("on_step", True), \
        "the heartbeat on_step call must be the first thing that happens, before hard_reset()"
    assert calls[1] == "hard_reset"


# --------------------------------------------------- M6 task 6: fault driving

class _FakeRotorFaultController:
    """Simulates instant echo (set_rotor_fault immediately updates
    latest_*), which is what a real plugin does within about one relay tick
    -- good enough to test _drive_rotor_fault's own control flow (when it
    calls set_rotor_fault, and how it reads the echo back), which is a
    different question from tests/sim/test_rotor_fault_controller.py's
    "does a real plugin actually echo" one."""

    def __init__(self):
        self.calls: list[tuple[int, float]] = []
        self.latest_rotor_index = -1
        self.latest_severity = 0.0
        self.latest_applied = False

    def set_rotor_fault(self, rotor_index, severity):
        self.calls.append((rotor_index, severity))
        self.latest_rotor_index = rotor_index
        self.latest_severity = severity
        self.latest_applied = rotor_index >= 0

    def clear_rotor_fault(self):
        self.set_rotor_fault(-1, 0.0)


def _fake_runner(monkeypatch, *, fly_ticks: list[float]):
    """A bare EpisodeRunner (object.__new__, same pattern as the heartbeat-
    ordering test above) with enable_rotor_fault=True, a fake
    RotorFaultController, and a fake fly_mission that calls on_step once per
    t_sim_s in `fly_ticks` before returning a minimal completed result."""

    def fake_fly_mission(node, px4, clock, mission, on_step=None, driver=None):
        for t in fly_ticks:
            on_step({"t_sim_s": t, "px4_failure_detector_status": 0})
        return types.SimpleNamespace(
            termination_reason="completed", n_steps=len(fly_ticks), waypoints_reached=0,
            position_rmse_m=0.0, final_position_error_m=0.0,
            t_sim_start_s=fly_ticks[0] if fly_ticks else 0.0,
            t_sim_end_s=fly_ticks[-1] if fly_ticks else 0.0,
            steps=[{"px4_failure_detector_status": 0, "pos_x": 0.0, "pos_y": 0.0, "pos_z": 0.0}
                   for _ in fly_ticks])

    monkeypatch.setattr("aero_bridge.mission_executor.fly_mission", fake_fly_mission)

    runner = object.__new__(EpisodeRunner)
    runner.spec = types.SimpleNamespace(instance=0, to_dict=lambda: {})
    runner.run_id = "run_x"
    runner.mission_id = "square_circuit"
    runner.mission = {}
    runner.mission_digest = "digest"
    runner.env_versions_json = "{}"
    runner.feature_version = "unversioned"
    runner.spec_digest = "specdigest"
    runner.enable_rotor_fault = True
    runner.rotor_fault = _FakeRotorFaultController()
    runner.node = object()
    runner.px4 = object()
    runner.clock = object()
    runner.driver = _fake_driver()
    runner.logger = types.SimpleNamespace(log_step=lambda *a, **k: None,
                                           write_episode=lambda *a, **k: None)
    return runner


def test_step_profile_commands_once_at_onset(monkeypatch):
    runner = _fake_runner(monkeypatch, fly_ticks=[0.0, 1.0, 2.0, 3.0, 4.0])
    fault_spec = FaultSpec(
        episode_index=0, fault_applied=True, fault_type=FaultType.ROTOR_THRUST_DEGRADATION,
        rotor_index=1, severity=0.5, onset_time_s=2.0, profile=FaultProfile.STEP,
        ramp_duration_s=0.0)

    summary = runner.run_episode(episode_id="ep_0000", reset_tier="none", seed=0,
                                  fault_spec=fault_spec, fault_config_digest="cfgdigest")

    assert runner.rotor_fault.calls == [(-1, 0.0), (1, 0.5)], \
        ("must clear any stale fault from a previous episode at episode "
         "start, then command the step fault exactly once, at onset")
    assert summary["fault_confirmed_applied"] is True
    assert summary["fault_confirmed_severity_final"] == 0.5
    assert summary["fault_onset_time_s_observed"] == 2.0
    assert summary["fault_config_digest"] == "cfgdigest"


def test_ramp_profile_commands_repeatedly_until_duration_then_locks(monkeypatch):
    runner = _fake_runner(monkeypatch, fly_ticks=[0.0, 1.0, 2.0, 3.0, 4.0])
    fault_spec = FaultSpec(
        episode_index=0, fault_applied=True, fault_type=FaultType.ROTOR_THRUST_DEGRADATION,
        rotor_index=3, severity=0.8, onset_time_s=1.0, profile=FaultProfile.RAMP,
        ramp_duration_s=2.0)

    summary = runner.run_episode(episode_id="ep_0000", reset_tier="none", seed=0,
                                  fault_spec=fault_spec)

    # First call is the unconditional pre-flight clear_rotor_fault(); the
    # ramp's own sequence follows.
    assert runner.rotor_fault.calls[0] == (-1, 0.0)
    severities = [s for (_rotor, s) in runner.rotor_fault.calls[1:]]
    assert severities == [0.0, 0.4, 0.8, 0.8], (
        "ramp must rise linearly to the target over ramp_duration_s, then "
        f"lock at it for the rest of the episode; got {severities}")
    assert all(r == 3 for (r, _s) in runner.rotor_fault.calls[1:])
    assert summary["fault_confirmed_applied"] is True
    assert summary["fault_confirmed_severity_final"] == 0.8


def test_healthy_fault_spec_never_commands_anything(monkeypatch):
    runner = _fake_runner(monkeypatch, fly_ticks=[0.0, 1.0, 2.0])
    summary = runner.run_episode(episode_id="ep_0000", reset_tier="none", seed=0,
                                  fault_spec=FaultSpec.healthy(0))

    assert runner.rotor_fault.calls == [(-1, 0.0)], (
        "the only call must be run_episode's own unconditional clear_rotor_fault "
        "at episode start -- _drive_rotor_fault itself must never call "
        "set_rotor_fault for a healthy fault_spec")
    assert summary["fault_applied"] is False
    assert summary["fault_type"] == "none"
    assert summary["fault_rotor_index"] == -1
    assert math.isnan(summary["fault_onset_time_s_observed"])
    assert summary["fault_confirmed_applied"] is False


def test_no_fault_spec_at_all_writes_healthy_sentinels_too(monkeypatch):
    """fault_spec=None (every M3/M4/M5 caller, and any M6 caller not
    injecting a fault this episode) must produce the exact same healthy
    record shape as an explicit FaultSpec.healthy() -- one implementation
    of what "no fault" means, not two."""
    runner = _fake_runner(monkeypatch, fly_ticks=[0.0, 1.0])
    summary = runner.run_episode(episode_id="ep_0000", reset_tier="none", seed=0)
    assert summary["fault_applied"] is False
    assert summary["fault_rotor_index"] == -1


def test_fault_spec_without_enable_flag_raises():
    runner = object.__new__(EpisodeRunner)
    runner.enable_rotor_fault = False
    fault_spec = FaultSpec(
        episode_index=0, fault_applied=True, fault_type=FaultType.ROTOR_THRUST_DEGRADATION,
        rotor_index=0, severity=0.5, onset_time_s=1.0, profile=FaultProfile.STEP,
        ramp_duration_s=0.0)
    try:
        runner.run_episode(episode_id="ep_0000", reset_tier="none", seed=0, fault_spec=fault_spec)
        assert False, "expected ValueError"
    except ValueError as exc:
        assert "enable_rotor_fault" in str(exc)
