"""M4 task 1: the project's one implementation of "fly one episode on one
worker" (CLAUDE.md §1.4). Everything above this layer -- M3's run_episodes.py,
M4's SimFarm, M6's dataset generator, M9's evaluation harness -- is a caller
of EpisodeRunner. None of them re-implements reset selection, flight, or
logging.

EpisodeRunner owns exactly the pipeline the milestone describes: reset (at an
already-decided tier) -> arm -> fly -> terminate -> log. It does NOT own
process lifecycle (that's WorkerSupervisor/SimFarm) and it does NOT own the
policy of *which* tier to request next after a bad episode -- that policy is
next_reset_tier() below, a pure function so it is testable without a
simulator and is exactly as reusable as EpisodeRunner itself. Two callers
(run_episodes.py's single-worker loop, sim_farm.py's per-worker loop) share
both.

Absorbs what used to be run_episodes.py's inline `_Worker` class: EpisodeRunner
owns the rclpy Node / PX4Interface / GzSimClock / PX4Clock for its one worker,
and rebuilds them after a hard reset invalidates the old PX4 process (see
aero_bridge/reset.py's module docstring for why hard reset alone has this
requirement).
"""
from __future__ import annotations

import datetime
import json
import subprocess
import time
from pathlib import Path
from typing import Optional

from experiments.episode_schema import FEATURE_VERSION_UNSET, SCHEMA_VERSION, TerminationReason, digest

REPO = Path(__file__).resolve().parent.parent

# Reset tiers that indicate the previous episode did NOT end cleanly, and
# therefore should not be trusted to leave the vehicle somewhere soft/medium
# reset can safely recover from -- escalate to hard instead of compounding
# whatever went wrong. Mirrors run_episodes.py's original inline policy
# (M3), lifted out here so SimFarm's per-worker loop shares it rather than
# re-deriving it.
_CLEAN_TERMINATIONS = frozenset({TerminationReason.COMPLETED.value})


def next_reset_tier(requested_tier: str, last_termination_reason: Optional[str]) -> str:
    """What reset tier to actually use for the next episode, given what was
    requested and how the previous episode ended.

    A pure function of two strings in, one string out -- no I/O, no
    simulator -- so it is fully covered by tests/test_episode_runner.py.
    `last_termination_reason` is None for the first episode on a freshly
    started worker (nothing to escalate from yet).
    """
    if last_termination_reason is None:
        return requested_tier
    if last_termination_reason in _CLEAN_TERMINATIONS:
        return requested_tier
    return "hard"


def _iso(t: float) -> str:
    return datetime.datetime.fromtimestamp(t, tz=datetime.timezone.utc).isoformat()


def capture_env_versions() -> dict:
    """Runs scripts/env_report.sh once (not once per episode -- toolchain
    versions do not change mid-run, and the script itself takes a couple of
    seconds) and returns its parsed JSON. Its stdout is valid JSON even when
    it exits non-zero (e.g. a missing patch warning), so returncode is not
    treated as failure here.

    Moved here from run_episodes.py (M3) so M4's sim_farm.py can share it
    without depending on the M3 script, or vice versa -- both now depend on
    this module instead of on each other."""
    proc = subprocess.run([str(REPO / "scripts" / "env_report.sh")],
                           capture_output=True, text=True, timeout=30)
    try:
        return json.loads(proc.stdout)
    except json.JSONDecodeError:
        return {"error": "env_report.sh did not produce valid JSON", "stderr": proc.stderr[-500:]}


class EpisodeRunner:
    """Bound to one worker (one InstanceSpec) for its whole lifetime. Owns
    the ROS-side objects for that worker and knows how to rebuild them after
    a hard reset invalidates the old ones.

    Constructed once per worker; run_episode() is called once per episode.
    """

    def __init__(self, spec, *, run_id: str, mission_id: str, mission: dict,
                 mission_digest: str, env_versions_json: str,
                 results_dir: str | Path = "results",
                 feature_version: str = FEATURE_VERSION_UNSET):
        self.spec = spec
        self.run_id = run_id
        self.mission_id = mission_id
        self.mission = mission
        self.mission_digest = mission_digest
        self.env_versions_json = env_versions_json
        self.feature_version = feature_version
        self.spec_digest = digest(spec.to_dict())

        from aero_bridge.episode_logger import EpisodeLogger
        self.logger = EpisodeLogger(run_id, worker_id=spec.instance, results_dir=results_dir)

        self._build_ros_objects()

    def _build_ros_objects(self) -> None:
        import rclpy
        from rclpy.node import Node

        from aero_bridge.px4_clock import PX4Clock
        from aero_bridge.px4_interface import PX4Interface
        from simulation.sim_clock import GzSimClock

        self.node = Node(f'episode_runner_{self.spec.instance}')
        self.px4 = PX4Interface(self.node, self.spec)
        self.gz_clock = GzSimClock(world=self.spec.world, gz_partition=self.spec.gz_partition)
        self.clock = PX4Clock(now_us_fn=self.gz_clock.now_us,
                               pump_fn=lambda t: rclpy.spin_once(self.node, timeout_sec=t))

    def close(self) -> None:
        """Releases this worker's ROS-side objects. Does NOT stop the
        underlying PX4/Gazebo OS processes -- that is WorkerSupervisor's job
        (simulation/worker_process.py), not this class's."""
        self.gz_clock.close()
        self.node.destroy_node()

    def rebuild_after_hard_reset(self) -> None:
        """Call after aero_bridge.reset.hard_reset() returns: the OLD node/
        px4/clock belong to a PX4 process that no longer exists. Mirrors what
        run_episodes.py's _Worker used to do inline."""
        self.close()
        self._build_ros_objects()

    def run_episode(self, *, episode_id: str, reset_tier: str, seed: int,
                     on_step=None) -> dict:
        """Runs exactly one episode at the given (already-decided) reset
        tier: apply the reset, fly the mission, log and return the summary
        record. Never raises ResetError itself for a *known-recoverable*
        soft/medium failure -- that decision (retry vs escalate) belongs to
        the caller, which is why this only takes a tier, not a policy.

        `on_step`, if given, is called with the same per-step row this
        method already logs -- an extension point for a caller that needs
        the per-tick cadence for something EpisodeRunner itself has no
        business knowing about. Its one real user is sim_farm.py's worker
        process, which writes a supervisor heartbeat file on every tick;
        EpisodeRunner stays ignorant of heartbeats, run dirs, or supervision
        entirely, which is what keeps this class a pure "fly one episode"
        primitive rather than something that also half-knows about SimFarm.

        Raises aero_bridge.reset.ResetError if the requested reset tier
        itself fails (the caller decides whether/how to escalate and retry;
        run_episodes.py and sim_farm.py both do this identically via
        next_reset_tier() above).
        """
        from aero_bridge.mission_executor import fly_mission
        from aero_bridge.reset import hard_reset, medium_reset, soft_reset

        tier_used = "none"
        reset_wall_duration_s = 0.0

        if reset_tier != "none":
            if on_step is not None:
                # A synthetic "still alive" heartbeat right before a reset
                # starts -- not a real step, not logged via the schema-
                # validated _on_step below, only the supervisor's heartbeat
                # side effect. Found live (M4 task 7's soak test): a reset
                # writes no heartbeat while it runs (heartbeats only happen
                # per flight control tick), so the LAST heartbeat before a
                # hard reset can already be seconds old by the time the
                # reset starts; under contention (worker_count>=4) a reset
                # occasionally took long enough for that stale timestamp to
                # cross heartbeat_stall_timeout_s, making WorkerSupervisor's
                # health check conclude the child was dead and restart it
                # ON TOP of its own still-in-progress reset -- burning
                # through the worker's restart budget on false positives.
                # Marking "reset just started, right now" as the heartbeat's
                # new baseline gives the health check the FULL timeout
                # window measured from the actual start of the slow
                # operation, not from whenever the last flight tick happened
                # to be.
                on_step({"t_wall_utc": time.time()})
            if reset_tier == "soft":
                r = soft_reset(self.node, self.px4, self.clock, self.spec)
            elif reset_tier == "medium":
                r = medium_reset(self.node, self.px4, self.clock, self.spec)
            elif reset_tier == "hard":
                r = hard_reset(self.spec)
                self.rebuild_after_hard_reset()
            else:
                raise ValueError(f"unknown reset_tier {reset_tier!r}")
            tier_used, reset_wall_duration_s = r.tier, r.wall_duration_s

        def _on_step(row, _episode_id=episode_id):
            self.logger.log_step(dict(row, schema_version=SCHEMA_VERSION, run_id=self.run_id,
                                       episode_id=_episode_id, worker_id=self.spec.instance))
            if on_step is not None:
                on_step(row)

        t_wall_start = time.time()
        result = fly_mission(self.node, self.px4, self.clock, self.mission, on_step=_on_step)
        t_wall_end = time.time()

        t_sim_start = result.t_sim_start_s if result.t_sim_start_s is not None else 0.0
        t_sim_end = result.t_sim_end_s if result.t_sim_end_s is not None else t_sim_start
        last_step = result.steps[-1] if result.steps else None

        summary = dict(
            schema_version=SCHEMA_VERSION, run_id=self.run_id, episode_id=episode_id,
            worker_id=self.spec.instance, instance=self.spec.instance, seed=seed,
            instance_spec_digest=self.spec_digest, mission_id=self.mission_id,
            mission_config_digest=self.mission_digest, feature_version=self.feature_version,
            env_versions=self.env_versions_json, reset_tier=tier_used,
            termination_reason=result.termination_reason,
            # Every outcome except sim_fault is a real, trustworthy measurement --
            # even offboard_lost/episode_timeout/aborted_error are genuine flight
            # data. sim_fault means the simulation state itself broke (§8), so
            # that one alone is invalidated here; worker_restarted's invalid
            # record is synthesized separately by SimFarm, never reaches here.
            valid=(result.termination_reason != TerminationReason.SIM_FAULT.value),
            t_sim_start_s=t_sim_start, t_sim_end_s=t_sim_end,
            t_sim_duration_s=t_sim_end - t_sim_start,
            t_wall_start_utc=_iso(t_wall_start), t_wall_end_utc=_iso(t_wall_end),
            t_wall_duration_s=t_wall_end - t_wall_start,
            n_steps=result.n_steps, waypoints_reached=result.waypoints_reached,
            position_rmse_m=result.position_rmse_m,
            final_position_error_m=result.final_position_error_m,
            reset_wall_duration_s=reset_wall_duration_s,
            final_pos_x=last_step["pos_x"] if last_step else float('nan'),
            final_pos_y=last_step["pos_y"] if last_step else float('nan'),
            final_pos_z=last_step["pos_z"] if last_step else float('nan'),
        )
        self.logger.write_episode(summary)
        return summary
