"""M6 task 6 (@pytest.mark.sim): EpisodeRunner's fault-injection integration
against a real worker -- the actual "the fault was genuinely commanded,
genuinely confirmed by the plugin's own echo, during a real flight" proof,
not just the fake-controller control-flow tests in
tests/test_episode_runner.py.

Also exercises a hard-reset -> second-faulted-episode sequence explicitly
(the one place this design isn't reusing an already-proven pattern
wholesale, per milestones.md M6 task 6's own watch-out-for note):
RotorFaultController is rebuilt in rebuild_after_hard_reset() the same way
PX4Interface/GzSimClock already are, but whether that actually works against
the NEW gz sim process a hard reset spins up had not been proven live before
this test.
"""
import json
import math
import os

import pytest


def _make_runner(instance, *, run_id, results_dir):
    from aero_bridge.mission_executor import load_mission
    from experiments.episode_runner import EpisodeRunner, capture_env_versions
    from experiments.episode_schema import FEATURE_VERSION_UNSET, digest
    from experiments.fault_schedule import REPO
    from simulation.instance_spec import InstanceSpec

    spec = InstanceSpec.for_instance(instance, model="x500_aero")
    os.environ['ROS_DOMAIN_ID'] = str(spec.ros_domain_id)

    mission = load_mission(REPO / "configs" / "missions" / "square_circuit.yaml")
    mission_digest = digest(mission)
    env_versions_json = json.dumps(capture_env_versions(), sort_keys=True)

    return EpisodeRunner(
        spec, run_id=run_id, mission_id="square_circuit", mission=mission,
        mission_digest=mission_digest, env_versions_json=env_versions_json,
        results_dir=str(results_dir), feature_version=FEATURE_VERSION_UNSET,
        enable_rotor_fault=True)


def test_step_fault_is_commanded_and_confirmed_live(sim_worker_x500_aero, tmp_path):
    import rclpy

    from experiments.fault_schedule import FaultProfile, FaultSpec, FaultType

    rclpy.init()
    try:
        runner = _make_runner(0, run_id="m6_fault_injection_test", results_dir=tmp_path)
        try:
            fault_spec = FaultSpec(
                episode_index=0, fault_applied=True,
                fault_type=FaultType.ROTOR_THRUST_DEGRADATION, rotor_index=1,
                severity=0.4, onset_time_s=5.0, profile=FaultProfile.STEP,
                ramp_duration_s=0.0)
            summary = runner.run_episode(
                episode_id="ep_0000", reset_tier="none", seed=0,
                fault_spec=fault_spec, fault_config_digest="test")

            assert summary["fault_confirmed_applied"] is True, (
                "the plugin's own status echo never confirmed the commanded "
                "severity was applied")
            assert summary["fault_confirmed_severity_final"] == pytest.approx(0.4, abs=0.05)
            assert not math.isnan(summary["fault_onset_time_s_observed"])
            # Observed onset should follow shortly after the requested one --
            # generous bound, this is confirming "roughly when", not exact
            # timing (relay/echo round-trip + control-tick granularity).
            assert summary["fault_onset_time_s_observed"] >= fault_spec.onset_time_s
            assert summary["fault_onset_time_s_observed"] < fault_spec.onset_time_s + 3.0
        finally:
            runner.close()
    finally:
        rclpy.shutdown()


def test_fault_injection_survives_a_hard_reset(sim_worker_x500_aero, tmp_path):
    """Two episodes, hard reset in between, fault injected in both -- proves
    RotorFaultController's rebuild-after-hard-reset actually reconnects to
    the new gz sim process's plugin instance rather than silently going
    stale."""
    import rclpy

    from experiments.fault_schedule import FaultProfile, FaultSpec, FaultType

    rclpy.init()
    try:
        runner = _make_runner(0, run_id="m6_fault_injection_hard_reset_test",
                               results_dir=tmp_path)
        try:
            fault_spec = FaultSpec(
                episode_index=0, fault_applied=True,
                fault_type=FaultType.ROTOR_THRUST_DEGRADATION, rotor_index=2,
                severity=0.3, onset_time_s=5.0, profile=FaultProfile.STEP,
                ramp_duration_s=0.0)

            first = runner.run_episode(episode_id="ep_0000", reset_tier="none",
                                        seed=0, fault_spec=fault_spec)
            assert first["fault_confirmed_applied"] is True

            second = runner.run_episode(episode_id="ep_0001", reset_tier="hard",
                                         seed=1, fault_spec=fault_spec)
            assert second["fault_confirmed_applied"] is True, (
                "fault confirmation failed on the episode immediately after a "
                "hard reset -- RotorFaultController was not correctly rebuilt "
                "against the new gz sim process")
        finally:
            runner.close()
    finally:
        rclpy.shutdown()
