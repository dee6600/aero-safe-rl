"""Unit tests for experiments/episode_runner.py's pure logic (M4 tasks 1-3).
No rclpy, no simulator -- EpisodeRunner itself needs a live PX4Interface/
GzSimClock and is exercised by tests/sim/test_two_workers.py instead; this
file covers next_reset_tier(), the one piece of EpisodeRunner-adjacent logic
that is a pure function of data.
"""
from experiments.episode_runner import next_reset_tier
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
