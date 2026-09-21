"""Unit tests for experiments/episode_runner.py's pure logic (M4 tasks 1-3).
No rclpy, no simulator -- EpisodeRunner itself needs a live PX4Interface/
GzSimClock and is exercised by tests/sim/test_two_workers.py instead; this
file covers next_reset_tier(), the one piece of EpisodeRunner-adjacent logic
that is a pure function of data. One exception: the on_step-before-reset
ordering below, which needs a bare EpisodeRunner (constructed via
object.__new__ to skip __init__'s rclpy Node/PX4Interface/GzSimClock setup)
with everything reset/flight-related faked out, since that ordering is the
whole point of the fix it tests.
"""
import types

from experiments.episode_runner import EpisodeRunner, next_reset_tier
from experiments.episode_schema import TerminationReason


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

    def fake_fly_mission(node, px4, clock, mission, on_step=None):
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
    runner.node = object()
    runner.px4 = object()
    runner.clock = object()
    runner.rebuild_after_hard_reset = lambda: calls.append("rebuild")
    runner.logger = types.SimpleNamespace(log_step=lambda *a, **k: None,
                                           write_episode=lambda *a, **k: None)

    def on_step(row):
        calls.append(("on_step", "t_wall_utc" in row))

    runner.run_episode(episode_id="ep_0000", reset_tier="hard", seed=0, on_step=on_step)

    assert calls[0] == ("on_step", True), \
        "the heartbeat on_step call must be the first thing that happens, before hard_reset()"
    assert calls[1] == "hard_reset"
