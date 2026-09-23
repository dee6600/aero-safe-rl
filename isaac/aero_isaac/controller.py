"""PX4 v1.17's multicopter flight controller, ported to PyTorch and
vectorised over N drones -- what flies the Isaac-side x500 in place of PX4
(milestones.md M8b, "Design"). Each stage follows the pinned source it names,
line by line where it matters:

  takeoff                mc_pos_control/Takeoff/Takeoff.cpp (spool-up, ramp) and the takeoff
                         handling in MulticopterPositionControl.cpp
  position -> velocity   mc_pos_control/PositionControl/PositionControl.cpp  _positionControl
  velocity -> thrust     same file, _velocityControl / _accelerationControl
  thrust -> attitude     ControlMath.cpp  thrustToAttitude / bodyzToAttitude / limitTilt
  attitude -> rates      mc_att_control/AttitudeControl/AttitudeControl.cpp  update
  rates -> torque        lib/rate_control/rate_control.cpp  update / updateIntegral
  torque -> motors       lib/control_allocation/.../ControlAllocationPseudoInverse.cpp (matrix,
                         normalisation) and ControlAllocationSequentialDesaturation.cpp
                         (mixAirmodeDisabled, mixYaw, desaturateActuators, computeDesaturationGain)

Gains and limits are read from PX4's parameter defaults and the x500
airframe (px4_model.px4_param), not typed in.

Simplifications, each one a candidate for the M8b task 5 agreement check to
expose: PX4's state estimator is replaced by the true state; its
hover-thrust estimator by a fixed hover thrust (HOVER_THRUST); its
second-order sensor filters by first-order ones at the same cut-offs; the
land detector is not modelled. The takeoff sequence is (added after the M8b
agreement check found the takeoff 4 s faster than PX4's).

Everything is in PX4's frames: north-east-down world, forward-right-down body.
"""
from __future__ import annotations

import math

import torch

from aero_isaac.px4_model import allocation_geometry, px4_param

G = 9.80665  # CONSTANTS_ONE_G
# Hover thrust the controller assumes. PX4 estimates it online (MPC_USE_HTE=1)
# and settles at the command the x500 actually hovers at; the M6 flights
# measured a median motor command of 0.736 over whole missions. A fixed value
# replaces the estimator unless the agreement check shows it matters.
HOVER_THRUST = 0.736
MINIMUM_YAW_MARGIN = 0.15   # ControlAllocationSequentialDesaturation.hpp


# ------------------------------------------------------------ quaternions
# (w, x, y, z), Hamilton, as PX4's matrix library.

def qmul(a: torch.Tensor, b: torch.Tensor) -> torch.Tensor:
    aw, ax, ay, az = a.unbind(-1)
    bw, bx, by, bz = b.unbind(-1)
    return torch.stack([aw * bw - ax * bx - ay * by - az * bz,
                        aw * bx + ax * bw + ay * bz - az * by,
                        aw * by - ax * bz + ay * bw + az * bx,
                        aw * bz + ax * by - ay * bx + az * bw], dim=-1)


def qconj(q: torch.Tensor) -> torch.Tensor:
    return q * torch.tensor((1.0, -1.0, -1.0, -1.0), dtype=q.dtype, device=q.device)


def canonical(q: torch.Tensor) -> torch.Tensor:
    return torch.where(q[..., :1] < 0, -q, q)


def dcm_z(q: torch.Tensor) -> torch.Tensor:
    w, x, y, z = q.unbind(-1)
    return torch.stack([2 * (w * y + x * z), 2 * (y * z - w * x), w * w - x * x - y * y + z * z], dim=-1)


def q_from_two_vectors(src: torch.Tensor, dst: torch.Tensor) -> torch.Tensor:
    """matrix::Quaternion(src, dst): the shortest rotation taking src to dst
    (the opposite-vector corner case is handled by the caller, as in PX4)."""
    cr = torch.cross(src, dst, dim=-1)
    dt = (src * dst).sum(-1, keepdim=True)
    w = dt + torch.sqrt((src * src).sum(-1, keepdim=True) * (dst * dst).sum(-1, keepdim=True))
    q = torch.cat([w, cr], dim=-1)
    return q / q.norm(dim=-1, keepdim=True).clamp(min=1e-12)


def q_from_dcm(R: torch.Tensor) -> torch.Tensor:
    """Rotation matrix (N, 3, 3) -> quaternion, numerically robust."""
    m00, m11, m22 = R[:, 0, 0], R[:, 1, 1], R[:, 2, 2]
    tr = m00 + m11 + m22
    cands = torch.stack([
        torch.stack([1 + tr, R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], -1),
        torch.stack([R[:, 2, 1] - R[:, 1, 2], 1 + m00 - m11 - m22, R[:, 0, 1] + R[:, 1, 0], R[:, 0, 2] + R[:, 2, 0]], -1),
        torch.stack([R[:, 0, 2] - R[:, 2, 0], R[:, 0, 1] + R[:, 1, 0], 1 - m00 + m11 - m22, R[:, 1, 2] + R[:, 2, 1]], -1),
        torch.stack([R[:, 1, 0] - R[:, 0, 1], R[:, 0, 2] + R[:, 2, 0], R[:, 1, 2] + R[:, 2, 1], 1 - m00 - m11 + m22], -1),
    ], dim=1)                                                     # (N, 4, 4)
    pick = torch.stack([tr, m00, m11, m22], dim=-1).argmax(-1)
    q = cands[torch.arange(len(R), device=R.device), pick]
    q = q / q.norm(dim=-1, keepdim=True)
    return canonical(q)


# ------------------------------------------------------------ allocation

def allocation_matrices(dtype=torch.float32, device=None):
    """PX4's effectiveness matrix (6 x 4), its normalised pseudo-inverse
    (4 x 6), and the normalisation scale (6,) -- ActuatorEffectivenessRotors
    + ControlAllocationPseudoInverse, for the x500's CA_ROTORn geometry."""
    geo = allocation_geometry()
    B = torch.zeros(6, len(geo), dtype=torch.float64)
    for i, r in enumerate(geo):
        # axis (0, 0, -1); moment = ct * (p x axis) - ct * km * axis
        B[:, i] = torch.tensor([-r.ct * r.py, r.ct * r.px, r.ct * r.km, 0.0, 0.0, -r.ct], dtype=torch.float64)
    mix = torch.linalg.pinv(B)
    scale = torch.ones(6, dtype=torch.float64)
    nnz_r = int((mix[:, 0].abs() > 1e-3).sum())
    nnz_p = int((mix[:, 1].abs() > 1e-3).sum())
    roll_s = math.sqrt(float((mix[:, 0] ** 2).sum()) / (nnz_r / 2.0)) if nnz_r else 1.0
    pitch_s = math.sqrt(float((mix[:, 1] ** 2).sum()) / (nnz_p / 2.0)) if nnz_p else 1.0
    scale[0] = scale[1] = max(roll_s, pitch_s)
    scale[2] = float(mix[:, 2].max())
    scale[5] = 1.0
    for axis in (2, 1, 0):
        col = mix[:, 3 + axis].abs()
        nnz = int((col > 1e-7).sum())
        scale[3 + axis] = float(col.sum()) / nnz if nnz else float(scale[5])
    mix = mix / scale
    mix = torch.where(mix.abs() < 1e-3, torch.zeros_like(mix), mix)
    return B.to(dtype=dtype, device=device), mix.to(dtype=dtype, device=device), scale.to(dtype=dtype, device=device)


def _desaturation_gain(v: torch.Tensor, u: torch.Tensor, u_min: torch.Tensor, u_max: torch.Tensor) -> torch.Tensor:
    """computeDesaturationGain. v (4,) direction, u (N, 4). Returns (N,)."""
    usable = v.abs() >= 0.2
    vv = torch.where(usable, v, torch.ones_like(v))
    k_lo = torch.where(usable & (u < u_min), (u_min - u) / vv, torch.zeros_like(u))
    k_hi = torch.where(usable & (u > u_max), (u_max - u) / vv, torch.zeros_like(u))
    ks = torch.cat([k_lo, k_hi], dim=1)
    return ks.min(dim=1).values.clamp(max=0.0) + ks.max(dim=1).values.clamp(min=0.0)


def _desaturate(u, v, u_min, u_max, increase_only=False):
    """desaturateActuators."""
    gain = _desaturation_gain(v, u, u_min, u_max)
    if increase_only:
        gain = gain.clamp(min=0.0)
    u = u + gain[:, None] * v
    return u + 0.5 * _desaturation_gain(v, u, u_min, u_max)[:, None] * v


def allocate(control_sp: torch.Tensor, mix: torch.Tensor) -> torch.Tensor:
    """mixAirmodeDisabled + mixYaw (airmode off, MC_AIRMODE default 0).
    control_sp (N, 6): roll, pitch, yaw torque, thrust x, y, z (normalised).
    Returns unclipped actuator setpoints (N, 4); the caller clips to [0, 1]."""
    u_min = torch.zeros(mix.shape[0], dtype=mix.dtype, device=mix.device)
    u_max = torch.ones_like(u_min)
    u = (control_sp[:, [0, 1, 3, 4, 5]] @ mix[:, [0, 1, 3, 4, 5]].T)
    u = _desaturate(u, mix[:, 5], u_min, u_max, increase_only=True)   # only reduce thrust
    u = _desaturate(u, mix[:, 0], u_min, u_max)                       # then roll
    u = _desaturate(u, mix[:, 1], u_min, u_max)                       # then pitch
    u = u + control_sp[:, 2:3] * mix[:, 2]                             # yaw, last
    u = _desaturate(u, mix[:, 2], u_min, u_max + (u_max - u_min) * MINIMUM_YAW_MARGIN)
    return _desaturate(u, mix[:, 5], u_min, u_max, increase_only=True)


# ------------------------------------------------------------ controller

def _lpf_alpha(cutoff_hz: float, dt: float) -> float:
    return dt / (dt + 1.0 / (2.0 * math.pi * cutoff_hz))


class PX4Controller:
    """One per environment, all drones at once. Call step() every physics
    step; the position/velocity loop runs every `pos_every`-th call, as PX4's
    runs slower than its attitude and rate loops."""

    def __init__(self, num_envs: int, physics_dt: float, pos_every: int, device=None,
                 dtype=torch.float32):
        p = px4_param
        self.n, self.dt, self.pos_every = num_envs, physics_dt, pos_every
        self.dt_pos = physics_dt * pos_every
        kw = dict(dtype=dtype, device=device)
        t = lambda v: torch.tensor(v, **kw)  # noqa: E731
        self.kw = kw
        # position / velocity
        self.gain_pos = t([p("MPC_XY_P"), p("MPC_XY_P"), p("MPC_Z_P")])
        self.gain_vel_p = t([p("MPC_XY_VEL_P_ACC")] * 2 + [p("MPC_Z_VEL_P_ACC")])
        self.gain_vel_i = t([p("MPC_XY_VEL_I_ACC")] * 2 + [p("MPC_Z_VEL_I_ACC")])
        self.gain_vel_d = t([p("MPC_XY_VEL_D_ACC")] * 2 + [p("MPC_Z_VEL_D_ACC")])
        self.lim_vel_xy, self.lim_vel_up, self.lim_vel_down = (
            p("MPC_XY_VEL_MAX"), p("MPC_Z_VEL_MAX_UP"), p("MPC_Z_VEL_MAX_DN"))
        self.lim_tilt = math.radians(p("MPC_TILTMAX_AIR"))
        self.lim_tilt_takeoff = math.radians(p("MPC_TILTMAX_LND"))
        self.spoolup_s = p("COM_SPOOLUP_TIME")
        self.takeoff_ramp_s = p("MPC_TKO_RAMP_T")
        self.takeoff_ramp_init = -G / max(p("MPC_Z_VEL_P_ACC"), 0.01)   # generateInitialRampValue
        self.lim_thr_min, self.lim_thr_max = p("MPC_THR_MIN"), p("MPC_THR_MAX")
        self.lim_thr_xy_margin = p("MPC_THR_XY_MARG")
        self.hover = HOVER_THRUST
        # attitude
        yaw_w = min(max(p("MC_YAW_WEIGHT"), 0.0), 1.0)
        self.yaw_w = yaw_w
        self.gain_att = t([p("MC_ROLL_P"), p("MC_PITCH_P"), p("MC_YAW_P") / yaw_w])
        self.rate_limit = t([math.radians(p("MC_ROLLRATE_MAX")), math.radians(p("MC_PITCHRATE_MAX")),
                             math.radians(p("MC_YAWRATE_MAX"))])
        # rates (gains are K * P/I/D, as PX4's setGains)
        k = t([p("MC_ROLLRATE_K"), p("MC_PITCHRATE_K"), p("MC_YAWRATE_K")])
        self.gain_rate_p = k * t([p("MC_ROLLRATE_P"), p("MC_PITCHRATE_P"), p("MC_YAWRATE_P")])
        self.gain_rate_i = k * t([p("MC_ROLLRATE_I"), p("MC_PITCHRATE_I"), p("MC_YAWRATE_I")])
        self.gain_rate_d = k * t([p("MC_ROLLRATE_D"), p("MC_PITCHRATE_D"), p("MC_YAWRATE_D")])
        self.lim_rate_int = t([p("MC_RR_INT_LIM"), p("MC_PR_INT_LIM"), p("MC_YR_INT_LIM")])
        self.a_gyro = _lpf_alpha(p("IMU_GYRO_CUTOFF"), physics_dt)
        self.a_dgyro = _lpf_alpha(p("IMU_DGYRO_CUTOFF"), physics_dt)
        self.a_yaw_tq = _lpf_alpha(p("MC_YAW_TQ_CUTOFF"), physics_dt)
        self.a_vel_dot = _lpf_alpha(5.0, self.dt_pos)
        # allocation
        self.B, self.mix, self.scale = allocation_matrices(**kw)
        self._state = {}
        self.reset(torch.arange(num_envs, device=device))

    def _buf(self, name, shape):
        if name not in self._state:
            self._state[name] = torch.zeros((self.n, *shape), **self.kw)
        return self._state[name]

    def reset(self, env_ids: torch.Tensor) -> None:
        for name, shape in (("vel_int", (3,)), ("vel_dot", (3,)), ("vel_prev", (3,)), ("rate_int", (3,)),
                            ("rate_f", (3,)), ("rate_f_prev", (3,)), ("ang_acc", (3,)), ("yaw_tq", ()),
                            ("thrust", ()), ("sat_pos", (3,)), ("sat_neg", (3,)),
                            ("takeoff", ()), ("ramp_progress", ())):
            self._buf(name, shape)[env_ids] = 0.0
        qd = self._buf("q_d", (4,))
        qd[env_ids] = torch.tensor((1.0, 0.0, 0.0, 0.0), **self.kw)
        self._buf("tick", ())[env_ids] = 0.0
        self._buf("vel_first", ())[env_ids] = 1.0

    # -------------------------------------------------- position / velocity

    def _takeoff(self, pos_sp, vz_sp, pos, due):
        """TakeoffHandling: spool-up -> ready -> ramp -> flight, per drone.
        States 0..3 as PX4's TakeoffState from spoolup. Returns the upward
        velocity limit (may be negative, as PX4's ramp starts), and masks."""
        s = self._state
        st = s["takeoff"]
        armed_s = s["tick"] * self.dt
        want = (torch.isfinite(pos_sp[:, 2]) & (pos_sp[:, 2] < pos[:, 2])) | \
               (torch.isnan(pos_sp[:, 2]) & (torch.nan_to_num(vz_sp, nan=0.0) < 0))
        st = torch.where((st == 0) & (armed_s >= self.spoolup_s), torch.ones_like(st), st)
        start = (st == 1) & want
        s["ramp_progress"] = torch.where(start, torch.zeros_like(st), s["ramp_progress"])
        st = torch.where(start, torch.full_like(st, 2), st)
        st = torch.where((st == 2) & (s["ramp_progress"] >= 1.0), torch.full_like(st, 3), st)
        ramping = (st == 2) & due
        s["ramp_progress"] = torch.where(ramping, s["ramp_progress"] + self.dt_pos / self.takeoff_ramp_s,
                                         s["ramp_progress"])
        up = torch.where(st < 2, torch.full_like(st, self.takeoff_ramp_init),
                         torch.where((st == 2) & (s["ramp_progress"] < 1.0),
                                     self.takeoff_ramp_init + s["ramp_progress"]
                                     * (self.lim_vel_up - self.takeoff_ramp_init),
                                     torch.full_like(st, self.lim_vel_up)))
        s["takeoff"] = torch.where(due, st, s["takeoff"])
        return torch.minimum(up, torch.full_like(up, self.lim_vel_up)), st < 2, st >= 3

    def _position_velocity(self, pos_sp, vz_sp, yaw_sp, pos, vel, due):
        s = self._state
        speed_up, not_taken_off, flying = self._takeoff(pos_sp, vz_sp, pos, due)
        # velocity derivative for the D term (PX4 uses the estimator's acceleration)
        fresh = s["vel_first"] > 0
        raw_dot = torch.where(fresh[:, None], torch.zeros_like(vel), (vel - s["vel_prev"]) / self.dt_pos)
        vel_dot = s["vel_dot"] + self.a_vel_dot * (raw_dot - s["vel_dot"])

        # _positionControl (no velocity feed-forward is ever sent)
        err = pos_sp - pos
        vel_sp = torch.nan_to_num(err, nan=0.0) * self.gain_pos
        vel_sp[:, 2] = torch.where(torch.isnan(pos_sp[:, 2]), torch.nan_to_num(vz_sp, nan=0.0), vel_sp[:, 2])
        xy = vel_sp[:, :2]
        n = xy.norm(dim=1, keepdim=True)
        # math::constrain(v, -up, down) as PX4 writes it -- also when the
        # takeoff ramp's negative start makes the lower bound exceed the upper
        vz = vel_sp[:, 2]
        vz = torch.where(vz < -speed_up, -speed_up, torch.where(vz > self.lim_vel_down,
                                                                 torch.full_like(vz, self.lim_vel_down), vz))
        vel_sp = torch.cat([torch.where(n > self.lim_vel_xy, xy / n.clamp(min=1e-9) * self.lim_vel_xy, xy),
                            vz[:, None]], dim=1)

        # _velocityControl (integrators held at zero until the takeoff ramp starts)
        s["vel_int"] = torch.where(not_taken_off[:, None], torch.zeros_like(s["vel_int"]), s["vel_int"])
        s["rate_int"] = torch.where(not_taken_off[:, None], torch.zeros_like(s["rate_int"]), s["rate_int"])
        vel_int = s["vel_int"].clone()
        vel_int[:, 2] = vel_int[:, 2].clamp(-G, G)
        vel_err = vel_sp - vel
        acc_sp = vel_err * self.gain_vel_p + vel_int - vel_dot * self.gain_vel_d

        # _accelerationControl (MPC_ACC_DECOUPLE = 1: assume -g vertical specific force)
        body_z = torch.stack([-acc_sp[:, 0], -acc_sp[:, 1], torch.full_like(acc_sp[:, 0], G)], dim=1)
        body_z = body_z / body_z.norm(dim=1, keepdim=True)
        tilt = torch.where(flying, torch.full_like(body_z[:, 0], self.lim_tilt),
                           torch.full_like(body_z[:, 0], self.lim_tilt_takeoff))
        body_z = _limit_tilt(body_z, tilt)
        thr_min = torch.where(flying, torch.full_like(tilt, self.lim_thr_min), torch.zeros_like(tilt))
        thrust_ned_z = acc_sp[:, 2] * (self.hover / G) - self.hover
        collective = torch.minimum(thrust_ned_z / body_z[:, 2], -thr_min)
        # not yet taken off: PX4 commands a large downward acceleration, i.e. no thrust
        collective = torch.where(not_taken_off, torch.zeros_like(collective), collective)
        thr_sp = body_z * collective[:, None]

        # vertical integrator anti-windup
        stop = ((thr_sp[:, 2] >= -thr_min) & (vel_err[:, 2] >= 0)) | \
               ((thr_sp[:, 2] <= -self.lim_thr_max) & (vel_err[:, 2] <= 0))
        vel_err[:, 2] = torch.where(stop, torch.zeros_like(vel_err[:, 2]), vel_err[:, 2])

        # prioritise vertical control while keeping a horizontal margin
        thr_xy = thr_sp[:, :2]
        xy_norm = thr_xy.norm(dim=1)
        alloc_h = torch.minimum(xy_norm, torch.full_like(xy_norm, self.lim_thr_xy_margin))
        z_max = torch.sqrt((self.lim_thr_max ** 2 - alloc_h ** 2).clamp(min=0.0))
        thr_z = torch.maximum(thr_sp[:, 2], -z_max)
        max_xy = torch.sqrt((self.lim_thr_max ** 2 - thr_z ** 2).clamp(min=0.0))
        over = xy_norm > max_xy
        thr_xy = torch.where(over[:, None], thr_xy / xy_norm.clamp(min=1e-9)[:, None] * max_xy[:, None], thr_xy)
        thr_sp = torch.cat([thr_xy, thr_z[:, None]], dim=1)

        # horizontal tracking anti-windup
        acc_xy_prod = thr_xy * (G / self.hover)
        arw = (acc_sp[:, :2] ** 2).sum(1) > (acc_xy_prod ** 2).sum(1)
        arw_gain = 2.0 / float(self.gain_vel_p[0])
        vel_err_xy = torch.where(arw[:, None], vel_err[:, :2] - arw_gain * (acc_sp[:, :2] - acc_xy_prod),
                                 vel_err[:, :2])
        vel_err = torch.cat([vel_err_xy, vel_err[:, 2:]], dim=1)
        vel_int = vel_int + vel_err * self.gain_vel_i * self.dt_pos

        # thrustToAttitude
        q_d = _bodyz_to_attitude(-thr_sp, yaw_sp)
        thrust = thr_sp.norm(dim=1)

        d = due
        s["vel_int"] = torch.where(d[:, None], vel_int, s["vel_int"])
        s["vel_dot"] = torch.where(d[:, None], vel_dot, s["vel_dot"])
        s["vel_prev"] = torch.where(d[:, None], vel, s["vel_prev"])
        s["vel_first"] = torch.where(d, torch.zeros_like(s["vel_first"]), s["vel_first"])
        s["q_d"] = torch.where(d[:, None], q_d, s["q_d"])
        s["thrust"] = torch.where(d, thrust, s["thrust"])

    # -------------------------------------------------- attitude

    def _attitude(self, q):
        qd = self._state["q_d"]
        e_z, e_z_d = dcm_z(q), dcm_z(qd)
        qd_red = q_from_two_vectors(e_z, e_z_d)
        corner = (qd_red[:, 1].abs() > 1 - 1e-5) | (qd_red[:, 2].abs() > 1 - 1e-5)
        qd_red = torch.where(corner[:, None], qd, qmul(qd_red, q))
        qd_dyaw = canonical(qmul(qconj(qd_red), qd))
        w = qd_dyaw[:, 0].clamp(-1.0, 1.0)
        z = qd_dyaw[:, 3].clamp(-1.0, 1.0)
        zero = torch.zeros_like(w)
        qd = qmul(qd_red, torch.stack([torch.cos(self.yaw_w * torch.acos(w)), zero, zero,
                                       torch.sin(self.yaw_w * torch.asin(z))], dim=1))
        qe = canonical(qmul(qconj(q), qd))
        rate_sp = 2.0 * qe[:, 1:] * self.gain_att
        return torch.maximum(torch.minimum(rate_sp, self.rate_limit), -self.rate_limit)

    # -------------------------------------------------- rates

    def _rates(self, rate_sp, rates_raw):
        s = self._state
        s["rate_f"] = s["rate_f"] + self.a_gyro * (rates_raw - s["rate_f"])
        raw_acc = (s["rate_f"] - s["rate_f_prev"]) / self.dt
        s["rate_f_prev"] = s["rate_f"].clone()
        s["ang_acc"] = s["ang_acc"] + self.a_dgyro * (raw_acc - s["ang_acc"])
        err = rate_sp - s["rate_f"]
        torque = self.gain_rate_p * err + s["rate_int"] - self.gain_rate_d * s["ang_acc"]
        # updateIntegral, with the allocator's saturation from the previous step
        e = torch.where(s["sat_pos"] > 0, err.clamp(max=0.0), err)
        e = torch.where(s["sat_neg"] > 0, e.clamp(min=0.0), e)
        i_factor = (1.0 - (e / math.radians(400.0)) ** 2).clamp(min=0.0)
        s["rate_int"] = torch.maximum(torch.minimum(s["rate_int"] + i_factor * self.gain_rate_i * e * self.dt,
                                                    self.lim_rate_int), -self.lim_rate_int)
        s["yaw_tq"] = s["yaw_tq"] + self.a_yaw_tq * (torque[:, 2] - s["yaw_tq"])
        return torch.cat([torque[:, :2], s["yaw_tq"][:, None]], dim=1)

    # -------------------------------------------------- one physics step

    def step(self, pos_sp: torch.Tensor, vz_sp: torch.Tensor, yaw_sp: torch.Tensor, pos: torch.Tensor,
             vel: torch.Tensor, q: torch.Tensor, rates: torch.Tensor) -> torch.Tensor:
        """pos_sp (N, 3) north-east-down, z may be NaN (then vz_sp, (N,), is
        the vertical velocity setpoint -- PX4's land mode); yaw_sp (N,);
        state: pos, vel (NED), q (forward-right-down to NED), rates (body).
        Returns motor commands (N, 4) in [0, 1]."""
        s = self._state
        due = (s["tick"] % self.pos_every) == 0
        self._position_velocity(pos_sp, vz_sp, yaw_sp, pos, vel, due)
        s["tick"] = s["tick"] + 1
        torque = self._rates(self._attitude(q), rates)
        control = torch.cat([torque, torch.zeros_like(torque[:, :2]), -s["thrust"][:, None]], dim=1)
        u = allocate(control, self.mix).clamp(0.0, 1.0)
        unallocated = control - (u @ self.B.T) * self.scale
        s["sat_pos"] = (unallocated[:, :3] > 1e-6).to(u.dtype)
        s["sat_neg"] = (unallocated[:, :3] < -1e-6).to(u.dtype)
        return u


def _limit_tilt(body_z: torch.Tensor, max_angle) -> torch.Tensor:
    """ControlMath::limitTilt against world up (0, 0, 1) in NED 'z-down'
    terms. max_angle: a number or one per drone."""
    world = torch.tensor((0.0, 0.0, 1.0), dtype=body_z.dtype, device=body_z.device)
    dot = body_z[:, 2]
    angle = torch.minimum(torch.acos(dot.clamp(-1.0, 1.0)),
                          torch.as_tensor(max_angle, dtype=body_z.dtype, device=body_z.device))
    rej = body_z - dot[:, None] * world
    tiny = (rej ** 2).sum(1) < 1.1920929e-07
    rej = torch.where(tiny[:, None], torch.tensor((1.0, 0.0, 0.0), dtype=body_z.dtype, device=body_z.device), rej)
    rej = rej / rej.norm(dim=1, keepdim=True)
    return torch.cos(angle)[:, None] * world + torch.sin(angle)[:, None] * rej


def _bodyz_to_attitude(body_z: torch.Tensor, yaw_sp: torch.Tensor) -> torch.Tensor:
    """ControlMath::bodyzToAttitude -> quaternion."""
    body_z = body_z / body_z.norm(dim=1, keepdim=True).clamp(min=1e-12)
    y_c = torch.stack([-torch.sin(yaw_sp), torch.cos(yaw_sp), torch.zeros_like(yaw_sp)], dim=1)
    body_x = torch.cross(y_c, body_z, dim=1)
    body_x = torch.where((body_z[:, 2] < 0)[:, None], -body_x, body_x)
    flat = body_z[:, 2].abs() < 1e-6
    body_x = torch.where(flat[:, None], torch.tensor((0.0, 0.0, 1.0), dtype=body_z.dtype, device=body_z.device),
                         body_x)
    body_x = body_x / body_x.norm(dim=1, keepdim=True)
    body_y = torch.cross(body_z, body_x, dim=1)
    return q_from_dcm(torch.stack([body_x, body_y, body_z], dim=2))
