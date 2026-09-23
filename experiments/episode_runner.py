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
from experiments.fault_schedule import FaultProfile, FaultSpec, commanded_severity

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
                 feature_version: str = FEATURE_VERSION_UNSET,
                 enable_rotor_fault: bool = False):
        self.spec = spec
        self.run_id = run_id
        self.mission_id = mission_id
        self.mission = mission
        self.mission_digest = mission_digest
        self.env_versions_json = env_versions_json
        self.feature_version = feature_version
        self.spec_digest = digest(spec.to_dict())
        # M6: off by default, so every M3/M4/M5 caller is unaffected. Only
        # M6's dataset generator (task 9) passes True.
        self.enable_rotor_fault = enable_rotor_fault

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

        if self.enable_rotor_fault:
            from simulation.rotor_fault import RotorFaultController
            # A hard reset restarts the OS-level gz sim/px4/MicroXRCEAgent
            # processes for this worker; whether an old gz.transport13.Node
            # from the same GZ_PARTITION would silently keep working against
            # the new process is exactly the kind of thing CLAUDE.md says not
            # to assume, so this is rebuilt here unconditionally, the same as
            # every other ROS-side object _build_ros_objects owns.
            self.rotor_fault = RotorFaultController(self.spec)
            self.rotor_fault.wait_for_heartbeat()

    def close(self) -> None:
        """Releases this worker's ROS-side objects. Does NOT stop the
        underlying PX4/Gazebo OS processes -- that is WorkerSupervisor's job
        (simulation/worker_process.py), not this class's."""
        self.gz_clock.close()
        self.node.destroy_node()
        if self.enable_rotor_fault:
            self.rotor_fault.close()

    def rebuild_after_hard_reset(self) -> None:
        """Call after aero_bridge.reset.hard_reset() returns: the OLD node/
        px4/clock belong to a PX4 process that no longer exists. Mirrors what
        run_episodes.py's _Worker used to do inline."""
        self.close()
        self._build_ros_objects()

    def run_episode(self, *, episode_id: str, reset_tier: str, seed: int,
                     on_step=None, fault_spec: Optional[FaultSpec] = None,
                     fault_config_digest: str = "none") -> dict:
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

        `fault_spec` (M6), if given, drives rotor-fault injection during the
        flight via self.rotor_fault (constructed only when this runner was
        built with enable_rotor_fault=True -- passing a fault_spec without
        that raises, rather than silently flying a healthy episode when a
        fault was actually requested). A healthy FaultSpec
        (fault_applied=False) is a legitimate value: it means "this episode
        is one of the dataset's negative examples", not "no fault system in
        use" -- the requested-side fields are still recorded either way.

        Raises aero_bridge.reset.ResetError if the requested reset tier
        itself fails (the caller decides whether/how to escalate and retry;
        run_episodes.py and sim_farm.py both do this identically via
        next_reset_tier() above).
        """
        from aero_bridge.mission_executor import fly_mission
        from aero_bridge.reset import hard_reset, medium_reset, soft_reset

        if fault_spec is not None and not self.enable_rotor_fault:
            raise ValueError(
                "fault_spec given but this EpisodeRunner was built with "
                "enable_rotor_fault=False -- construct it with "
                "enable_rotor_fault=True to inject faults")

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

        if self.enable_rotor_fault:
            # A soft/medium reset (unlike hard) keeps the same gz sim
            # process alive, and with it the same RotorDegradationSystem
            # plugin instance -- a fault commanded during the PREVIOUS
            # episode would otherwise still be active when this one's
            # flight starts, regardless of this episode's own onset_time_s.
            # Idempotent and cheap, so cleared unconditionally rather than
            # only for the tiers where it would actually matter.
            self.rotor_fault.clear_rotor_fault()

        fault_state = {
            "mission_start_t_sim_s": None,
            "commanded": False,
            "ramp_start_t_sim_s": None,
            "onset_time_s_observed": None,
            "confirmed_applied": False,
            "confirmed_severity_final": 0.0,
        }

        def _drive_rotor_fault(row: dict) -> None:
            """Advances fault_spec's onset/ramp against this tick's sim
            time, and records what the plugin's own status echo confirms
            was actually applied -- the "confirm, don't assume" loop this
            whole milestone exists for. A no-op for a healthy fault_spec
            (fault_applied=False): the requested-side fields are already
            correct sentinels, and there is nothing to command or confirm.
            """
            if fault_spec is None or not fault_spec.fault_applied:
                return
            t_sim_s = row.get("t_sim_s")
            if t_sim_s is None:
                return
            if fault_state["mission_start_t_sim_s"] is None:
                fault_state["mission_start_t_sim_s"] = t_sim_s
            elapsed = t_sim_s - fault_state["mission_start_t_sim_s"]

            if elapsed >= fault_spec.onset_time_s:
                if fault_spec.profile == FaultProfile.STEP:
                    if not fault_state["commanded"]:
                        self.rotor_fault.set_rotor_fault(fault_spec.rotor_index, fault_spec.severity)
                        fault_state["commanded"] = True
                elif fault_spec.profile == FaultProfile.RAMP:
                    if fault_state["ramp_start_t_sim_s"] is None:
                        fault_state["ramp_start_t_sim_s"] = t_sim_s
                    ramp_elapsed = t_sim_s - fault_state["ramp_start_t_sim_s"]
                    self.rotor_fault.set_rotor_fault(
                        fault_spec.rotor_index,
                        commanded_severity(fault_spec.severity, fault_spec.profile,
                                           fault_spec.ramp_duration_s, ramp_elapsed))

            if (self.rotor_fault.latest_applied
                    and self.rotor_fault.latest_rotor_index == fault_spec.rotor_index):
                if fault_state["onset_time_s_observed"] is None:
                    fault_state["onset_time_s_observed"] = elapsed
                fault_state["confirmed_severity_final"] = self.rotor_fault.latest_severity
                if abs(self.rotor_fault.latest_severity - fault_spec.severity) < 1e-2:
                    fault_state["confirmed_applied"] = True

        def _on_step(row, _episode_id=episode_id):
            if self.enable_rotor_fault:
                _drive_rotor_fault(row)
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
            # (fault_spec or FaultSpec.healthy(0)).to_episode_fields() supplies
            # the REQUESTED-side fields either way (CLAUDE.md §7 -- one
            # implementation of what "no fault" means, not a second copy of
            # these literals here); fault_config_digest is the caller's
            # concern (M6 task 9's dataset generator sets it from the fault
            # schedule config actually in use -- this class has no fault
            # config of its own to digest).
            fault_config_digest=(fault_config_digest if fault_spec is not None else "none"),
            **(fault_spec or FaultSpec.healthy(0)).to_episode_fields(),
            # OBSERVED/CONFIRMED fields are genuinely runtime-only and have no
            # "no fault requested" analogue in FaultSpec -- filled in from
            # fault_state, which _drive_rotor_fault populated from the
            # plugin's own status echo (never assumed from the command alone).
            fault_onset_time_s_observed=(
                fault_state["onset_time_s_observed"]
                if fault_state["onset_time_s_observed"] is not None else float('nan')),
            fault_confirmed_applied=fault_state["confirmed_applied"],
            fault_confirmed_severity_final=fault_state["confirmed_severity_final"],
            # A real, honest computation from this episode's own telemetry --
            # not a placeholder -- since px4_failure_detector_status (schema
            # v4) is logged on every step regardless of whether a fault was
            # ever commanded (mission_executor.py's record_step()).
            px4_failure_detector_silent=all(
                s["px4_failure_detector_status"] == 0 for s in result.steps
            ) if result.steps else True,
        )
        self.logger.write_episode(summary)
        return summary
