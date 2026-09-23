"""M7 task 5 (@pytest.mark.sim): the trained detector running live, inside
two concurrent workers, on real flights with an injected rotor fault.

Each worker flies in its own spawned process (rclpy.init only in the child,
CLAUDE.md §3.3) through the ordinary EpisodeRunner, with DetectorRuntime
attached via EpisodeRunner's existing on_step hook -- the exact wiring M8's
recovery layer will use. Different rotor and severity per worker, so a
cross-wired detector (worker 0's telemetry reaching worker 1's detector)
would show up as the wrong rotor.

Needs results/m7_detector_v1/detector.pt (ai/detector/train.py).
"""
from __future__ import annotations

import multiprocessing as mp
import os
import time
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parent.parent.parent
CHECKPOINT = REPO / "results" / "m7_detector_v1" / "detector.pt"

ONSET_S = 12.0
# (instance, rotor, severity)
CASES = [(0, 1, 0.5), (1, 3, 0.3)]
# Generous live bound; the offline median delay is reported in
# docs/detector_results.md.
MAX_DELAY_S = 3.0


def _fly_with_detector(instance: int, rotor: int, severity: float, results_dir: str, queue) -> None:
    import json

    from aero_bridge.mission_executor import load_mission
    from simulation.instance_spec import InstanceSpec
    spec = InstanceSpec.for_instance(instance, model="x500_aero")
    os.environ["ROS_DOMAIN_ID"] = str(spec.ros_domain_id)

    import rclpy

    from ai.detector.runtime import DetectorRuntime
    from experiments.episode_runner import EpisodeRunner, capture_env_versions
    from experiments.episode_schema import FEATURE_VERSION_UNSET, digest
    from experiments.fault_schedule import FaultProfile, FaultSpec, FaultType

    rclpy.init()
    try:
        mission = load_mission(REPO / "configs" / "missions" / "square_circuit.yaml")
        runner = EpisodeRunner(
            spec, run_id="m7_detector_live_test", mission_id="square_circuit", mission=mission,
            mission_digest=digest(mission),
            env_versions_json=json.dumps(capture_env_versions(), sort_keys=True),
            results_dir=results_dir, feature_version=FEATURE_VERSION_UNSET,
            enable_rotor_fault=True)
        detector = DetectorRuntime(CHECKPOINT)
        trace = []

        def on_step(row):
            t0 = time.perf_counter()
            out = detector.step(row)
            trace.append((row["t_sim_s"], out.p_fault, out.alarm, out.rotor, out.severity,
                          time.perf_counter() - t0))

        try:
            fault = FaultSpec(episode_index=0, fault_applied=True,
                              fault_type=FaultType.ROTOR_THRUST_DEGRADATION, rotor_index=rotor,
                              severity=severity, onset_time_s=ONSET_S, profile=FaultProfile.STEP,
                              ramp_duration_s=0.0)
            summary = runner.run_episode(episode_id="ep_0000", reset_tier="none", seed=instance,
                                         on_step=on_step, fault_spec=fault,
                                         fault_config_digest="test")
        finally:
            runner.close()
        queue.put({"instance": instance, "summary": {
            k: summary[k] for k in ("fault_confirmed_applied", "termination_reason", "n_steps")},
            "trace": trace})
    except Exception as e:  # surfaced to the parent as a failure, never a hang
        queue.put({"instance": instance, "error": repr(e)})
    finally:
        rclpy.shutdown()


@pytest.mark.timeout(900)
@pytest.mark.skipif(not CHECKPOINT.exists(), reason="train the detector first (ai/detector/train.py)")
def test_detector_fires_on_the_right_rotor_live_on_two_workers(sim_workers_0_1_x500_aero, tmp_path):
    ctx = mp.get_context("spawn")
    queue = ctx.Queue()
    procs = [ctx.Process(target=_fly_with_detector, args=(i, r, s, str(tmp_path), queue))
             for i, r, s in CASES]
    for p in procs:
        p.start()
    results = {}
    for _ in procs:
        r = queue.get(timeout=800)
        results[r["instance"]] = r
    for p in procs:
        p.join(timeout=60)

    for instance, rotor, severity in CASES:
        r = results[instance]
        assert "error" not in r, f"instance {instance}: {r.get('error')}"
        assert r["summary"]["fault_confirmed_applied"], f"instance {instance}: fault never confirmed"
        t, p_fault, alarm, pred_rotor, _, latency = map(np.array, zip(*r["trace"]))
        elapsed = t - t[0]
        before = elapsed < ONSET_S
        after = np.flatnonzero((elapsed >= ONSET_S) & alarm)
        print(f"instance {instance} rotor {rotor} s={severity}: {len(t)} ticks, "
              f"pre-onset alarm ticks {int(alarm[before].sum())}, "
              f"first alarm {elapsed[after[0]] - ONSET_S if len(after) else None} s after onset, "
              f"latency median {1e3 * np.median(latency):.2f} ms p99 {1e3 * np.quantile(latency, .99):.2f} ms")

        assert not alarm[before].any(), f"instance {instance}: false alarm before onset"
        assert len(after), f"instance {instance}: fault never detected"
        assert elapsed[after[0]] - ONSET_S <= MAX_DELAY_S
        # Right rotor on (nearly) every alarmed tick after detection.
        assert np.mean(pred_rotor[after] == rotor) >= 0.9, f"instance {instance}: wrong rotor"
        assert np.quantile(latency, 0.99) < 0.020
