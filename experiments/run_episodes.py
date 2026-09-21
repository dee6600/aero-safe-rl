#!/usr/bin/env python3
"""M3's episode runner: fly a mission N times against one worker, resetting
between flights, logging every episode through EpisodeLogger.

This is M3's own single-worker driver, not M4's EpisodeRunner/SimFarm (that
milestone owns parallel workers, health-checked restarts and a run manifest;
building that here would be the "second implementation" CLAUDE.md forbids).
What this script does own is exactly what M3 needs to produce the noise
floor, the reset-tier comparison and the divergence numbers: one worker,
one mission, N episodes, real resets between them.

Usage:
    python experiments/run_episodes.py --mission square_circuit --n 20 --instance 0
    python experiments/run_episodes.py --mission square_circuit --n 20 --reset-tier hard
    python experiments/run_episodes.py --mission square_circuit --n 20 --seed 7 --run-id divergence_seed7
"""
from __future__ import annotations

import argparse
import datetime
import json
import subprocess
import sys
import time
import uuid
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from experiments.episode_schema import FEATURE_VERSION_UNSET, SCHEMA_VERSION, digest


def capture_env_versions() -> dict:
    """Runs scripts/env_report.sh once per run_episodes.py invocation (not
    once per episode -- toolchain versions do not change mid-run, and the
    script itself takes a couple of seconds) and returns its parsed JSON.
    Its stdout is valid JSON even when it exits non-zero (e.g. a missing
    patch warning), so returncode is not treated as failure here."""
    proc = subprocess.run([str(REPO / "scripts" / "env_report.sh")],
                           capture_output=True, text=True, timeout=30)
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError:
        return {"error": "env_report.sh did not produce valid JSON", "stderr": proc.stderr[-500:]}


def _iso(t: float) -> str:
    return datetime.datetime.fromtimestamp(t, tz=datetime.timezone.utc).isoformat()


class _Worker:
    """Owns the ROS-side objects for one instance and knows how to rebuild
    them after a hard reset -- the one place run_episodes.py needs this,
    since soft/medium reset (aero_bridge.reset) keep the same objects valid
    but hard reset does not (see reset.py's module docstring)."""

    def __init__(self, spec):
        self.spec = spec
        self._build()

    def _build(self):
        import rclpy
        from rclpy.node import Node
        from aero_bridge.px4_clock import PX4Clock
        from aero_bridge.px4_interface import PX4Interface
        from simulation.sim_clock import GzSimClock

        self.node = Node(f'run_episodes_{self.spec.instance}')
        self.px4 = PX4Interface(self.node, self.spec)
        self.gz_clock = GzSimClock(world=self.spec.world, gz_partition=self.spec.gz_partition)
        self.clock = PX4Clock(now_us_fn=self.gz_clock.now_us,
                               pump_fn=lambda t: rclpy.spin_once(self.node, timeout_sec=t))

    def close(self):
        self.gz_clock.close()
        self.node.destroy_node()


def run_episodes(mission_id: str, n: int, instance: int, *, reset_tier: str = "soft",
                  seed: int = 0, speed_factor: float = 1.0, run_id: str | None = None,
                  results_dir: str | Path = "results") -> list[dict]:
    import os

    from simulation.instance_spec import InstanceSpec

    run_id = run_id or f"run_{datetime.datetime.now(tz=datetime.timezone.utc):%Y%m%dT%H%M%SZ}_{uuid.uuid4().hex[:6]}"
    mission_path = REPO / "configs" / "missions" / f"{mission_id}.yaml"

    spec = InstanceSpec.for_instance(instance, speed_factor=speed_factor)
    spec_digest = digest(spec.to_dict())

    # ROS_DOMAIN_ID must match the instance BEFORE rclpy (and the DDS layer
    # underneath it) initialises -- it cannot be changed afterwards. Every
    # instance's domain is its instance number (D9); getting this wrong is
    # the exact "only works on instance 0" bug class CLAUDE.md §2 warns
    # about, since instance 0's domain happens to equal the usual default.
    os.environ['ROS_DOMAIN_ID'] = str(spec.ros_domain_id)

    import rclpy

    from aero_bridge.episode_logger import EpisodeLogger
    from aero_bridge.mission_executor import fly_mission, load_mission
    from aero_bridge.reset import ResetError, hard_reset, medium_reset, soft_reset

    mission = load_mission(mission_path)
    mission_digest = digest(mission)
    env_versions_json = json.dumps(capture_env_versions(), sort_keys=True)
    logger = EpisodeLogger(run_id, worker_id=instance, results_dir=results_dir)

    rclpy.init()
    worker = _Worker(spec)
    print(f"run_id={run_id} mission={mission_id} n={n} instance={instance} "
          f"reset_tier={reset_tier} seed={seed} speed={speed_factor}x")

    # A worker-level hiccup (observed live: gz set_pose occasionally trips
    # PX4's magnetometer consistency check, "Preflight Fail: Compass 0
    # fault" -- see aero_bridge/reset.py's module docstring) can abort an
    # attempt before the mission itself is exercised at all. That is not a
    # measurement of the mission's own behaviour, so it does not count
    # toward n -- but it IS logged like any other episode (never silently
    # discarded) and retried, capped so a systemic problem still fails
    # loudly rather than looping forever.
    MAX_EXTRA_ATTEMPTS = max(5, n // 2)

    summaries: list[dict] = []
    last_termination_reason = None
    n_completed = 0
    attempt = 0
    try:
        while n_completed < n:
            if attempt >= n + MAX_EXTRA_ATTEMPTS:
                raise RuntimeError(
                    f"only {n_completed}/{n} episodes completed after {attempt} attempts "
                    f"({MAX_EXTRA_ATTEMPTS} retries exhausted) -- this is a systemic problem, "
                    f"not a fluke; stop and investigate rather than retrying further")
            i = attempt
            attempt += 1
            episode_id = f"ep_{i:04d}"
            tier_used = "none"
            reset_wall_duration_s = 0.0

            # A prior episode that did not end in a clean landing may have
            # left the vehicle somewhere soft/medium reset cannot safely
            # recover from (soft_reset itself already refuses a still-armed
            # vehicle; ResetUnsafe below already escalates that case) -- but
            # even a *safe*, completed-looking reset following an abnormal
            # episode is worth being conservative about, so escalate the
            # requested tier to hard for one cycle rather than compounding
            # whatever went wrong.
            effective_tier = reset_tier
            if i > 0 and last_termination_reason not in (None, "completed") and effective_tier != "hard":
                print(f"  episode {i}: previous episode ended '{last_termination_reason}', "
                      f"using hard reset instead of {reset_tier} for this cycle")
                effective_tier = "hard"

            if i > 0:
                try:
                    if effective_tier == "soft":
                        r = soft_reset(worker.node, worker.px4, worker.clock, worker.spec)
                    elif effective_tier == "medium":
                        r = medium_reset(worker.node, worker.px4, worker.clock, worker.spec)
                    elif effective_tier == "hard":
                        worker.close()
                        r = hard_reset(spec)
                        rclpy.shutdown()
                        rclpy.init()
                        worker = _Worker(spec)
                    else:
                        raise ValueError(f"unknown reset_tier {effective_tier!r}")
                    tier_used, reset_wall_duration_s = r.tier, r.wall_duration_s
                except ResetError as exc:
                    print(f"  episode {i}: {effective_tier} reset failed ({exc}); escalating to hard reset")
                    worker.close()
                    r = hard_reset(spec)
                    rclpy.shutdown()
                    rclpy.init()
                    worker = _Worker(spec)
                    tier_used, reset_wall_duration_s = r.tier, r.wall_duration_s

            def on_step(row, _episode_id=episode_id):
                # mission_executor's rows carry only flight data -- the
                # run/episode/worker identity a step record also requires
                # (configs/schema/episode_record.yaml) is this loop's
                # context, not something fly_mission knows about.
                logger.log_step(dict(row, schema_version=SCHEMA_VERSION, run_id=run_id,
                                      episode_id=_episode_id, worker_id=instance))

            t_wall_start = time.time()
            result = fly_mission(worker.node, worker.px4, worker.clock, mission,
                                  on_step=on_step)
            t_wall_end = time.time()

            t_sim_start = result.t_sim_start_s if result.t_sim_start_s is not None else 0.0
            t_sim_end = result.t_sim_end_s if result.t_sim_end_s is not None else t_sim_start
            last_step = result.steps[-1] if result.steps else None

            summary = dict(
                schema_version=SCHEMA_VERSION, run_id=run_id, episode_id=episode_id,
                worker_id=instance, instance=instance, seed=seed,
                instance_spec_digest=spec_digest, mission_id=mission_id,
                mission_config_digest=mission_digest, feature_version=FEATURE_VERSION_UNSET,
                env_versions=env_versions_json, reset_tier=tier_used,
                termination_reason=result.termination_reason, valid=True,
                t_sim_start_s=t_sim_start, t_sim_end_s=t_sim_end,
                t_sim_duration_s=t_sim_end - t_sim_start,
                t_wall_start_utc=_iso(t_wall_start), t_wall_end_utc=_iso(t_wall_end),
                t_wall_duration_s=t_wall_end - t_wall_start,
                n_steps=result.n_steps, waypoints_reached=result.waypoints_reached,
                position_rmse_m=result.position_rmse_m,
                final_position_error_m=result.final_position_error_m,
                # Extra fields, not part of the schema's required set (validate_episode
                # only checks required fields are present, not that no others exist):
                # the reset ladder's own wall-clock cost (M3 task 5), and the raw final
                # position (task 7 asks for "final position" divergence specifically,
                # not just the scalar error distance final_position_error_m already
                # required above).
                reset_wall_duration_s=reset_wall_duration_s,
                final_pos_x=last_step["pos_x"] if last_step else float('nan'),
                final_pos_y=last_step["pos_y"] if last_step else float('nan'),
                final_pos_z=last_step["pos_z"] if last_step else float('nan'),
            )
            logger.write_episode(summary)
            summaries.append(summary)
            last_termination_reason = result.termination_reason
            if result.termination_reason == "completed":
                n_completed += 1
            print(f"  episode {i}: {result.termination_reason:16s} reset={tier_used:6s} "
                  f"rmse={result.position_rmse_m:.3f}m final_err={result.final_position_error_m:.3f}m "
                  f"steps={result.n_steps} wall={t_wall_end - t_wall_start:.1f}s "
                  f"[{n_completed}/{n} completed]")
    finally:
        worker.close()
        rclpy.shutdown()

    n_failed_attempts = len(summaries) - n_completed
    print(f"done: {n_completed}/{n} completed over {len(summaries)} attempts "
          f"({n_failed_attempts} did not complete), results under "
          f"{Path(results_dir) / run_id / f'worker_{instance}'}")
    return summaries


def main(argv=None) -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mission", required=True, help="mission id under configs/missions/")
    ap.add_argument("--n", type=int, required=True, help="number of episodes to fly")
    ap.add_argument("--instance", type=int, default=0)
    ap.add_argument("--reset-tier", choices=["soft", "medium", "hard"], default="soft")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--run-id", default=None)
    ap.add_argument("--results-dir", default=str(REPO / "results"))
    args = ap.parse_args(argv)

    run_episodes(args.mission, args.n, args.instance, reset_tier=args.reset_tier,
                 seed=args.seed, speed_factor=args.speed, run_id=args.run_id,
                 results_dir=args.results_dir)


if __name__ == "__main__":
    main()
