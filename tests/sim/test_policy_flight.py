"""M8 task 3 (@pytest.mark.sim): the policy-driven flight live, on two
concurrent workers, each in its own spawned process (CLAUDE.md §3.3).

  instance 0: no recovery (NominalPolicy) + the detector, healthy flight.
              Must complete like an M6 flight, log every schema-v5 field,
              record the landing, and be judged mission_success.
  instance 1: the rule-based FSM (configs/rl/fsm_v1.yaml) + the detector,
              rotor 2 at s = 0.6 (above s* -- hover impossible). The FSM must
              commit to land after onset and never before it.

Needs results/m7_detector_v1/detector.pt.
"""
from __future__ import annotations

import multiprocessing as mp
import os
from pathlib import Path

import pandas as pd
import pytest

REPO = Path(__file__).resolve().parent.parent.parent
CHECKPOINT = REPO / "results" / "m7_detector_v1" / "detector.pt"
ONSET_S = 12.0
CASES = [  # (instance, policy, policy_config, rotor, severity)
    (0, "nominal", None, None, 0.0),
    (1, "rule_based", "configs/rl/fsm_v1.yaml", 2, 0.6),
]


def _fly(instance, policy, policy_config, rotor, severity, results_dir, queue) -> None:
    import json

    from aero_bridge.mission_executor import load_mission
    from simulation.instance_spec import InstanceSpec
    spec = InstanceSpec.for_instance(instance, model="x500_aero")
    os.environ["ROS_DOMAIN_ID"] = str(spec.ros_domain_id)

    import rclpy

    from experiments.episode_runner import EpisodeRunner, capture_env_versions
    from experiments.episode_schema import FEATURE_VERSION_UNSET, digest
    from experiments.fault_schedule import FaultProfile, FaultSpec, FaultType
    from rl.policy_driver import RecoveryConfig

    rclpy.init()
    try:
        mission = load_mission(REPO / "configs" / "missions" / "square_circuit.yaml")
        runner = EpisodeRunner(
            spec, run_id="m8_policy_flight_test", mission_id="square_circuit", mission=mission,
            mission_digest=digest(mission),
            env_versions_json=json.dumps(capture_env_versions(), sort_keys=True),
            results_dir=results_dir, feature_version=FEATURE_VERSION_UNSET,
            enable_rotor_fault=True,
            recovery=RecoveryConfig(policy=policy, policy_config=policy_config,
                                    detector_checkpoint=str(CHECKPOINT)))
        fault = (FaultSpec(episode_index=0, fault_applied=True,
                           fault_type=FaultType.ROTOR_THRUST_DEGRADATION, rotor_index=rotor,
                           severity=severity, onset_time_s=ONSET_S, profile=FaultProfile.STEP,
                           ramp_duration_s=0.0)
                 if rotor is not None else FaultSpec.healthy(0))
        try:
            summary = runner.run_episode(episode_id="ep_0000", reset_tier="none", seed=instance,
                                         fault_spec=fault, fault_config_digest="test")
        finally:
            runner.close()
        queue.put({"instance": instance, "summary": summary})
    except Exception as e:  # surfaced to the parent as a failure, never a hang
        queue.put({"instance": instance, "error": repr(e)})
    finally:
        rclpy.shutdown()


@pytest.mark.timeout(900)
@pytest.mark.skipif(not CHECKPOINT.exists(), reason="train the detector first (ai/detector/train.py)")
def test_policy_driven_flight_live_on_two_workers(sim_workers_0_1_x500_aero, tmp_path):
    from experiments.metrics import Outcome, classify_outcome

    ctx = mp.get_context("spawn")
    queue = ctx.Queue()
    procs = [ctx.Process(target=_fly, args=(*case, str(tmp_path), queue)) for case in CASES]
    for p in procs:
        p.start()
    results = {}
    for _ in procs:
        r = queue.get(timeout=800)
        results[r["instance"]] = r
    for p in procs:
        p.join(timeout=60)
    for r in results.values():
        assert "error" not in r, f"instance {r['instance']}: {r.get('error')}"

    def steps(instance):
        return pd.read_parquet(tmp_path / "m8_policy_flight_test" / f"worker_{instance}"
                               / "episode_ep_0000_steps.parquet")

    # --- instance 0: nominal, healthy
    s0, st0 = results[0]["summary"], steps(0)
    print(f"nominal: {s0['termination_reason']} in {s0['t_sim_duration_s']:.1f}s, "
          f"rmse {s0['position_rmse_m']:.2f}, {len(st0)} steps "
          f"({(st0.flight_phase == 'landing').sum()} landing)")
    assert s0["termination_reason"] == "completed" and s0["waypoints_reached"] == 5
    assert s0["policy_name"] == "nominal" and s0["detector_checkpoint_digest"] != "none"
    mission = st0[st0.flight_phase == "mission"]
    assert (mission.action_speed_scale == 1.0).all() and not st0.action_land.any()
    assert (st0.flight_phase == "landing").any()
    assert st0.det_p_fault.notna().all()
    assert classify_outcome(s0["termination_reason"], st0).outcome == Outcome.MISSION_SUCCESS

    # --- instance 1: FSM, severe fault
    s1, st1 = results[1]["summary"], steps(1)
    elapsed = st1.t_sim_s - s1["t_sim_start_s"]
    landed_at = elapsed[st1.action_land.astype(bool)]
    print(f"fsm: {s1['termination_reason']}, states {st1.policy_state.unique().tolist()}, "
          f"land committed {landed_at.min() - ONSET_S if len(landed_at) else None}s after onset, "
          f"outcome {classify_outcome(s1['termination_reason'], st1)}")
    assert s1["fault_confirmed_applied"]
    assert s1["policy_name"] == "rule_based" and s1["policy_config_digest"] != "none"
    assert len(landed_at), "FSM never committed to land on a severe fault"
    assert landed_at.min() >= ONSET_S, "FSM committed to land before the fault"
    assert not st1[elapsed < ONSET_S].policy_state.isin(["RECOVERING", "ABORTED"]).any()
    assert s1["termination_reason"] in ("recovery_landed", "ground_contact")
