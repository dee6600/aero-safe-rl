"""The Isaac Lab training environment (M8b task 4): N x500s flying the
square_circuit mission in parallel, each with a possible weak rotor, under a
recovery policy that sees exactly what it would see on the PX4 side.

Per physics step (250 per second, Gazebo's own step), for every drone: read
the state, run the PX4 controller port on the mission tracker's setpoint, step
the rotors (with the fault), and apply the resulting force and torque at the
centre of mass. Every 0.1 s -- the PX4 side's step-record tick -- advance the
mission tracker, the outcome rule, the landing logic and the simulated
detector. Every 0.2 s the policy decides (configs/rl/action_v1.yaml), and the
observation is assembled from the four observable blocks only
(observation.py). The true fault state lives in `self._fault_*` and is never
passed to the observation builder.

Frames: Isaac world x = north, y = west, z = up (frames.py). Everything the
tracker, controller, outcome rule and observation see is north-east-down /
forward-right-down, relative to where the drone rests at spawn.

The reward is zero: M9 freezes it. Per-episode results (termination, outcome,
touchdown speed, tilt, the fault flown) are kept in `self.episode_log` for the
agreement check (task 5) and, later, M9's logging.
"""
from __future__ import annotations

import math

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject
from isaaclab.envs import DirectRLEnv, DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sim import SimulationCfg
from isaaclab.utils import configclass

from aero_isaac.contracts import (load_action_spec, load_mission, load_observation_spec,
                                  load_outcome_spec)
from aero_isaac.controller import PX4Controller
from aero_isaac.detector_sim import DetectorSim, NoDetector
from aero_isaac.frames import euler_ned, flip, quat_ned_frd, specific_force_frd
from aero_isaac.mission import MissionTracker
from aero_isaac.observation import ObservationBuilder
from aero_isaac.outcome import (COMPLETED, GROUND_CONTACT, RECOVERY_LANDED, TIMEOUT, OutcomeTracker,
                                tilt_deg)
from aero_isaac.px4_model import load_x500, mass_properties, px4_param
from aero_isaac.rotor import RotorModel
from aero_isaac.x500_asset import rest_height, rigid_object_cfg

PHYSICS_DT = 0.004          # Gazebo's max_step_size in PX4's default world
TICK_S = 0.1                # PX4 side's step-record period (mission_executor.CONTROL_PERIOD_S)
POS_LOOP_EVERY = 5          # PX4's position loop runs at ~50 Hz


@configclass
class AeroEnvCfg(DirectRLEnvCfg):
    decimation = 50                                  # 0.2 s decisions -- checked against action_v1.yaml
    episode_length_s = 150.0                         # mission timeout (120 s) plus room to land
    action_space = 3
    observation_space = 27                           # checked against observation_v2.yaml
    state_space = 0
    sim: SimulationCfg = SimulationCfg(dt=PHYSICS_DT, render_interval=decimation)
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=16, env_spacing=40.0, replicate_physics=True)
    seed: int = 0

    # Faults: one rotor per faulted episode, as the PX4-side evaluation
    # schedule (experiments/run_recovery.py). M9 widens these for training.
    fault_probability: float = 0.8
    severity_range: tuple = (0.2, 0.9)
    onset_range_s: tuple = (5.0, 25.0)
    ramp_range_s: tuple = (1.0, 5.0)
    step_fraction: float = 0.5
    # If set, drone i always flies severity fixed_severities[i % len] (0 =
    # healthy) -- for the agreement check's severity grid.
    fixed_severities: tuple | None = None
    # "simulated": the detector stand-in fitted to the real detector
    # (detector_sim.py); "none": no detector output (scripted-policy checks).
    detector: str = "simulated"

    # Sensor noise (1-sigma), measured from the M6 flights' hover stretches
    # (sample-to-sample jitter / sqrt(2), 519 samples; an upper bound).
    noise_attitude_rad: tuple = (0.0056, 0.0051, 0.0011)
    noise_rate_rad_s: tuple = (0.0087, 0.0076, 0.0054)
    noise_accel_m_s2: tuple = (0.0055, 0.0056, 0.0186)
    noise_vel_m_s: tuple = (0.0248, 0.0287, 0.0043)


class AeroEnv(DirectRLEnv):
    cfg: AeroEnvCfg

    def __init__(self, cfg: AeroEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)
        n, dev = self.num_envs, self.device
        self.action_spec = load_action_spec()
        self.outcome_spec = load_outcome_spec()
        self.mission = load_mission()
        obs_spec = load_observation_spec()
        if abs(self.step_dt - self.action_spec.decision_period_s) > 1e-9:
            raise ValueError(f"decision period {self.step_dt} s != action_v1.yaml's "
                             f"{self.action_spec.decision_period_s} s (CLAUDE.md §4)")
        if len(obs_spec.entries) != cfg.observation_space:
            raise ValueError("observation_space does not match observation_v2.yaml")
        self.ticks_per_step = round(self.step_dt / TICK_S)
        self.substeps_per_tick = round(TICK_S / PHYSICS_DT)

        x500 = load_x500()
        com, _ = mass_properties(x500)
        self._rest = rest_height(x500)
        self._r_b = torch.tensor([r.position_flu for r in x500.rotors], device=dev) - \
            torch.tensor(com, dtype=torch.float32, device=dev)                         # (4, 3) body frame, from COM
        self.rotors = RotorModel(x500, n, device=dev)
        self.controller = PX4Controller(n, PHYSICS_DT, POS_LOOP_EVERY, device=dev)
        self.tracker = MissionTracker(self.mission, self.action_spec, n, device=dev)
        self.outcome = OutcomeTracker(self.outcome_spec, n, device=dev)
        self.observer = ObservationBuilder(obs_spec)
        self.land_speed = px4_param("MPC_LAND_SPEED")

        self._gen = torch.Generator(device=dev)
        self._gen.manual_seed(cfg.seed)
        if cfg.detector == "simulated":
            self.detector = DetectorSim(n, device=dev, generator=self._gen)
        elif cfg.detector == "none":
            self.detector = NoDetector(n, device=dev)
        else:
            raise ValueError(f"unknown detector {cfg.detector!r}")
        f = lambda *s, dtype=torch.float32: torch.zeros(*s, dtype=dtype, device=dev)  # noqa: E731
        self._substep = f(n, dtype=torch.long)                # physics steps since episode start
        self._action = self.action_spec.nominal_tensor(n, device=dev)
        self._landing = f(n, dtype=torch.bool)
        self._land_reason = f(n, dtype=torch.long)
        self._land_xy = f(n, 2)
        self._on_ground_s = f(n)
        self._termination = torch.full((n,), -1, dtype=torch.long, device=dev)
        self._last_cmd = f(n, 4)
        self._peak_speed = f(n)                               # agreement-check measures, mission phase
        self._cmd_sum = f(n)
        self._cmd_ticks = f(n)
        self._det = NoDetector(n, device=dev).step()
        # ground truth -- never passed to the observation
        self._fault_rotor = torch.full((n,), -1, dtype=torch.long, device=dev)
        self._fault_severity = f(n)
        self._fault_onset = f(n)
        self._fault_ramp = f(n)
        self.episode_log: list[dict] = []
        self._reset_idx(torch.arange(n, device=dev))

    # ------------------------------------------------------------ scene

    def _setup_scene(self):
        self.robot = RigidObject(rigid_object_cfg("/World/envs/env_.*/Robot"))
        self.scene.rigid_objects["robot"] = self.robot
        sim_utils.GroundPlaneCfg().func("/World/ground", sim_utils.GroundPlaneCfg())
        self.scene.clone_environments(copy_from_source=False)
        sim_utils.DomeLightCfg(intensity=2000.0).func("/World/Light", sim_utils.DomeLightCfg(intensity=2000.0))

    # ------------------------------------------------------------ state (PX4 frames)

    def _state(self):
        d = self.robot.data
        pos = flip(d.root_pos_w - self.scene.env_origins - torch.tensor((0.0, 0.0, self._rest), device=self.device))
        vel = flip(d.root_lin_vel_w)
        q = quat_ned_frd(d.root_quat_w)
        rates = flip(d.root_ang_vel_b)
        return pos, vel, q, rates

    def _t(self) -> torch.Tensor:
        return self._substep.to(torch.float32) * PHYSICS_DT

    def _current_severity(self, t: torch.Tensor) -> torch.Tensor:
        """experiments/fault_schedule.py:commanded_severity, per drone."""
        since = t - self._fault_onset
        ramp = torch.where(self._fault_ramp > 0, (since / self._fault_ramp.clamp(min=1e-6)).clamp(0.0, 1.0),
                           torch.ones_like(since))
        return torch.where((self._fault_rotor >= 0) & (since >= 0), self._fault_severity * ramp,
                           torch.zeros_like(since))

    # ------------------------------------------------------------ policy step

    def _pre_physics_step(self, actions: torch.Tensor):
        decoded = self.action_spec.decode(actions.to(torch.float32))
        latched = self._action[:, 2] > 0.5                   # a land decision is final
        self._action = torch.where(latched[:, None], self._action, decoded)

    def _apply_action(self):
        pos, vel, q, rates = self._state()
        t = self._t()
        if int(self._substep[0]) % self.substeps_per_tick == 0:
            self._tick(t, pos, vel, q)

        pos_sp = torch.where(self._landing[:, None],
                             torch.cat([self._land_xy, torch.full_like(pos[:, :1], float("nan"))], 1),
                             self.tracker.setpoint)
        vz_sp = torch.where(self._landing, torch.full_like(t, self.land_speed), torch.full_like(t, float("nan")))
        cmd = self.controller.step(pos_sp, vz_sp, torch.zeros_like(t), pos, vel, q, rates)
        self._last_cmd = cmd

        scale = RotorModel.fault_speed_scale(self._current_severity(t), self._fault_rotor)
        thrust, drag_tq_up = self.rotors.step(cmd, scale, PHYSICS_DT)

        # forces in the body frame (forward-left-up), about the centre of mass
        d = self.robot.data
        v_rot = d.root_lin_vel_b[:, None, :] + torch.cross(d.root_ang_vel_b[:, None, :].expand(-1, 4, -1),
                                                           self._r_b.expand(self.num_envs, -1, -1), dim=2)
        v_perp = v_rot * torch.tensor((1.0, 1.0, 0.0), device=self.device)
        f_drag, m_roll = self.rotors.air_drag(v_perp)
        f = f_drag.clone()
        f[..., 2] += thrust
        torque = torch.cross(self._r_b.expand(self.num_envs, -1, -1), f, dim=2).sum(1) + m_roll.sum(1)
        torque[:, 2] += drag_tq_up.sum(1)
        self.robot.permanent_wrench_composer.set_forces_and_torques(
            forces=f.sum(1)[:, None, :], torques=torque[:, None, :], body_ids=[0])
        self._substep += 1

    def _tick(self, t, pos, vel, q):
        """Every 0.1 s: mission tracker, outcome rule, landing, detector."""
        flying = ~self._landing
        was_done = self.tracker.done.clone()
        self.tracker.update(t, pos, self._action)
        # mission finished, or the policy committed to land: PX4's land mode from here
        start_land = flying & ((self.tracker.done & ~was_done) | (self._action[:, 2] > 0.5))
        self._land_reason = torch.where(start_land & (self._action[:, 2] > 0.5),
                                        torch.full_like(self._land_reason, RECOVERY_LANDED),
                                        torch.where(start_land, torch.full_like(self._land_reason, COMPLETED),
                                                    self._land_reason))
        self._land_xy = torch.where(start_land[:, None], pos[:, :2], self._land_xy)
        self._landing |= start_land

        euler = euler_ned(q)
        alt = -pos[:, 2]
        airborne_mission = flying & (alt > 1.0)
        self._peak_speed = torch.where(airborne_mission, torch.maximum(self._peak_speed, vel[:, :2].norm(dim=1)),
                                       self._peak_speed)
        self._cmd_sum += torch.where(airborne_mission, self._last_cmd.mean(1), torch.zeros_like(alt))
        self._cmd_ticks += airborne_mission.float()
        contact = self.outcome.update(alt, vel[:, 2], tilt_deg(euler[:, 0], euler[:, 1]))
        on_ground = alt < self.outcome_spec.ground_contact_alt_m
        self._on_ground_s = torch.where(self._landing & on_ground, self._on_ground_s + TICK_S,
                                        torch.zeros_like(self._on_ground_s))
        open_ = self._termination < 0
        term = torch.where(open_ & contact & ~self._landing, torch.full_like(self._termination, GROUND_CONTACT),
                           self._termination)
        term = torch.where(open_ & self._landing & (self._on_ground_s >= self.outcome_spec.landed_settle_s),
                           self._land_reason, term)
        mission_over = open_ & ~self._landing & (t >= float(self.mission["timeout_s"]))
        self._termination = torch.where(mission_over, torch.full_like(term, TIMEOUT), term)

        self._det = self.detector.step(t=t, severity=self._current_severity(t), offset_cmd=self._action[:, 1])

    # ------------------------------------------------------------ observation, reward, dones

    def _noisy(self, x, std):
        return x + torch.randn(x.shape, generator=self._gen, device=self.device) * \
            torch.tensor(std, device=self.device)

    def _get_observations(self) -> dict:
        pos, vel, q, _ = self._state()
        c = self.cfg
        euler = self._noisy(euler_ned(q), c.noise_attitude_rad)
        rates = self._noisy(flip(self.robot.data.root_ang_vel_b), c.noise_rate_rad_s)
        acc = self._noisy(specific_force_frd(self.robot.data.root_quat_w, self.robot.data.body_lin_acc_w[:, 0]),
                          c.noise_accel_m_s2)
        vel_n = self._noisy(vel, c.noise_vel_m_s)
        feature = dict(roll_rad=euler[:, 0], pitch_rad=euler[:, 1], yaw_rad=euler[:, 2],
                       rate_p_rad_s=rates[:, 0], rate_q_rad_s=rates[:, 1], rate_r_rad_s=rates[:, 2],
                       accel_x_m_s2=acc[:, 0], accel_y_m_s2=acc[:, 1], accel_z_m_s2=acc[:, 2],
                       vel_x=vel_n[:, 0], vel_y=vel_n[:, 1], vel_z=vel_n[:, 2],
                       position_error_m=torch.linalg.norm(pos - self.tracker.target, dim=1))
        a = self._action
        obs = self.observer.assemble({
            "feature": feature,
            "detector": self._det,
            "mission": self.tracker.progress(self._t(), pos),
            "previous_action": dict(speed_scale=a[:, 0], altitude_offset_m=a[:, 1], land=a[:, 2]),
        })
        return {"policy": obs}

    def _get_rewards(self) -> torch.Tensor:
        return torch.zeros(self.num_envs, device=self.device)   # M9 freezes the reward

    def _get_dones(self):
        ended = self._termination >= 0
        timed_out = (self._termination == TIMEOUT) | (self.episode_length_buf >= self.max_episode_length - 1)
        return ended & (self._termination != TIMEOUT), timed_out

    # ------------------------------------------------------------ reset

    def _log_finished(self, env_ids: torch.Tensor):
        if not hasattr(self, "tracker") or len(env_ids) == 0:
            return
        term_all = torch.where(self._termination >= 0, self._termination,
                               torch.full_like(self._termination, TIMEOUT))
        outcome = self.outcome.outcome(term_all)
        term = term_all[env_ids]
        for i, e in enumerate(env_ids.tolist()):
            if int(self._substep[e]) == 0:
                continue
            self.episode_log.append(dict(
                env=e, severity=float(self._fault_severity[e]), rotor=int(self._fault_rotor[e]),
                onset_s=float(self._fault_onset[e]), ramp_s=float(self._fault_ramp[e]),
                termination=int(term[i]), outcome=int(outcome[e]),
                touchdown_speed_m_s=float(self.outcome.touchdown_speed[e]),
                max_tilt_deg=float(self.outcome.max_tilt_deg[e]),
                waypoints_reached=int(self.tracker.waypoints_reached[e]),
                duration_s=float(self._substep[e]) * PHYSICS_DT,
                peak_hspeed_m_s=float(self._peak_speed[e]),
                mean_motor_command=float(self._cmd_sum[e] / self._cmd_ticks[e].clamp(min=1.0))))

    def _reset_idx(self, env_ids):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        env_ids = torch.as_tensor(env_ids, device=self.device)
        self._log_finished(env_ids)
        super()._reset_idx(env_ids)
        if not hasattr(self, "tracker"):
            return  # called by the base class before __init__ finished

        k, dev = len(env_ids), self.device
        state = self.robot.data.default_root_state[env_ids].clone()
        state[:, :3] += self.scene.env_origins[env_ids]
        self.robot.write_root_pose_to_sim(state[:, :7], env_ids)
        self.robot.write_root_velocity_to_sim(torch.zeros(k, 6, device=dev), env_ids)

        self.rotors.reset(env_ids)
        self.controller.reset(env_ids)
        self.tracker.reset(env_ids, torch.zeros(k, 2, device=dev))
        self.outcome.reset(env_ids)
        self._substep[env_ids] = 0
        self._action[env_ids] = self.action_spec.nominal_tensor(k, device=dev)
        self._landing[env_ids] = False
        self._on_ground_s[env_ids] = 0.0
        self._termination[env_ids] = -1
        self._peak_speed[env_ids] = 0.0
        self._cmd_sum[env_ids] = 0.0
        self._cmd_ticks[env_ids] = 0.0

        c = self.cfg
        u = lambda lo_hi: lo_hi[0] + (lo_hi[1] - lo_hi[0]) * torch.rand(k, generator=self._gen, device=dev)  # noqa: E731
        if c.fixed_severities:
            sev = torch.tensor(c.fixed_severities, device=dev)[env_ids % len(c.fixed_severities)]
            faulted = sev > 0
        else:
            sev = u(c.severity_range)
            faulted = torch.rand(k, generator=self._gen, device=dev) < c.fault_probability
        rotor = torch.randint(0, 4, (k,), generator=self._gen, device=dev)
        step = torch.rand(k, generator=self._gen, device=dev) < c.step_fraction
        self._fault_rotor[env_ids] = torch.where(faulted, rotor, torch.full_like(rotor, -1))
        self._fault_severity[env_ids] = torch.where(faulted, sev, torch.zeros_like(sev))
        self._fault_onset[env_ids] = u(c.onset_range_s)
        self._fault_ramp[env_ids] = torch.where(step, torch.zeros(k, device=dev), u(c.ramp_range_s))
        self.detector.reset(env_ids, target=self._fault_severity[env_ids], rotor=self._fault_rotor[env_ids],
                            onset=self._fault_onset[env_ids], ramp_s=self._fault_ramp[env_ids])
