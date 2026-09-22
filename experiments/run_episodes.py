#!/usr/bin/env python3
"""M3's episode runner: fly a mission N times against one worker, resetting
between flights, logging every episode through EpisodeLogger.

This is M3's own single-worker CLI driver. As of M4, the actual per-episode
work (reset -> arm -> fly -> log) is EpisodeRunner (experiments/episode_runner.py)
-- the project's one implementation of "fly one episode" (CLAUDE.md §1.4).
This script's own job is the part EpisodeRunner deliberately does NOT own:
the N-episode loop, the escalate-to-hard-reset-after-a-bad-episode retry
policy (next_reset_tier(), shared with M4's SimFarm), and this milestone's
env-version capture / run bookkeeping.

Usage:
    python experiments/run_episodes.py --mission square_circuit --n 20 --instance 0
    python experiments/run_episodes.py --mission square_circuit --n 20 --reset-tier hard
    python experiments/run_episodes.py --mission square_circuit --n 20 --seed 7 --run-id divergence_seed7
"""
from __future__ import annotations

import argparse
import datetime
import json
import sys
import uuid
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from experiments.episode_runner import EpisodeRunner, capture_env_versions, next_reset_tier
from experiments.episode_schema import FEATURE_VERSION_UNSET, digest


def run_episodes(mission_id: str, n: int, instance: int, *, reset_tier: str = "soft",
                  seed: int = 0, speed_factor: float = 1.0, model: str | None = None,
                  run_id: str | None = None,
                  results_dir: str | Path = "results") -> list[dict]:
    import os

    from simulation.instance_spec import DEFAULT_MODEL, InstanceSpec

    run_id = run_id or f"run_{datetime.datetime.now(tz=datetime.timezone.utc):%Y%m%dT%H%M%SZ}_{uuid.uuid4().hex[:6]}"
    mission_path = REPO / "configs" / "missions" / f"{mission_id}.yaml"

    # Must match whatever model the worker was actually started with
    # (scripts/sim_start.sh -m) -- this script connects to an already-running
    # worker rather than starting one, but spec.model_name (used by
    # aero_bridge/reset.py's gz-transport calls, e.g. set_pose) is still
    # derived from it, and a mismatch here fails silently in exactly that
    # kind of call, not at startup (M6, found live).
    spec = InstanceSpec.for_instance(instance, speed_factor=speed_factor,
                                      model=model or DEFAULT_MODEL)

    # ROS_DOMAIN_ID must match the instance BEFORE rclpy (and the DDS layer
    # underneath it) initialises -- it cannot be changed afterwards. Every
    # instance's domain is its instance number (D9); getting this wrong is
    # the exact "only works on instance 0" bug class CLAUDE.md §2 warns
    # about, since instance 0's domain happens to equal the usual default.
    os.environ['ROS_DOMAIN_ID'] = str(spec.ros_domain_id)

    import rclpy

    from aero_bridge.mission_executor import load_mission
    from aero_bridge.reset import ResetError

    mission = load_mission(mission_path)
    mission_digest = digest(mission)
    env_versions_json = json.dumps(capture_env_versions(), sort_keys=True)

    rclpy.init()
    runner = EpisodeRunner(spec, run_id=run_id, mission_id=mission_id, mission=mission,
                            mission_digest=mission_digest, env_versions_json=env_versions_json,
                            results_dir=results_dir, feature_version=FEATURE_VERSION_UNSET)
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

            # Nothing to reset from on a freshly started worker's very first
            # episode; every episode after that asks next_reset_tier() what
            # to actually use, given how the previous one ended (escalates to
            # hard after anything but a clean completion -- see that
            # function's docstring, experiments/episode_runner.py).
            effective_tier = "none" if i == 0 else next_reset_tier(reset_tier, last_termination_reason)
            if effective_tier == "hard" and reset_tier != "hard" and i > 0:
                print(f"  episode {i}: previous episode ended '{last_termination_reason}', "
                      f"using hard reset instead of {reset_tier} for this cycle")

            try:
                summary = runner.run_episode(episode_id=episode_id, reset_tier=effective_tier, seed=seed)
            except ResetError as exc:
                print(f"  episode {i}: {effective_tier} reset failed ({exc}); escalating to hard reset")
                # EpisodeRunner.run_episode(reset_tier="hard") both performs the
                # hard reset and rebuilds the ROS objects afterward -- retrying
                # through it (rather than calling hard_reset() here and then
                # replaying with reset_tier="none") is what keeps the summary's
                # own reset_tier/reset_wall_duration_s fields honest about what
                # actually happened.
                summary = runner.run_episode(episode_id=episode_id, reset_tier="hard", seed=seed)

            summaries.append(summary)
            last_termination_reason = summary["termination_reason"]
            if summary["termination_reason"] == "completed":
                n_completed += 1
            print(f"  episode {i}: {summary['termination_reason']:16s} reset={summary['reset_tier']:6s} "
                  f"rmse={summary['position_rmse_m']:.3f}m final_err={summary['final_position_error_m']:.3f}m "
                  f"steps={summary['n_steps']} wall={summary['t_wall_duration_s']:.1f}s "
                  f"[{n_completed}/{n} completed]")
    finally:
        runner.close()
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
    ap.add_argument("--model", default=None,
                    help="must match the model the worker was started with (sim_start.sh -m)")
    ap.add_argument("--run-id", default=None)
    ap.add_argument("--results-dir", default=str(REPO / "results"))
    args = ap.parse_args(argv)

    run_episodes(args.mission, args.n, args.instance, reset_tier=args.reset_tier,
                 seed=args.seed, speed_factor=args.speed, model=args.model,
                 run_id=args.run_id, results_dir=args.results_dir)


if __name__ == "__main__":
    main()
