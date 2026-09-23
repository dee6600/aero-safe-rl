"""M8b task 4: run the Isaac environment and report.

    source scripts/activate_isaac.sh && cd isaac
    python -m aero_isaac.probe smoke --out smoke.json          # what isaac/tests/test_env.py checks
    python -m aero_isaac.probe throughput --num-envs 8192 --out ../results/m8b_throughput.json
    python -m aero_isaac.probe clip --out ../results/m8b_isaac_clip.mp4

smoke: 16 drones at severities 0 / 0.3 / 0.45 / 0.7 under the nominal action,
until every drone has finished an episode. Records the observation's shape
and finiteness, the decision period, each drone's first episode, whether a
finished drone is back on the ground, and the leak check: with physics,
the detector's last output and the noise draw held fixed, changing the true
fault must leave the observation unchanged (while changing the detector's
output must change it, so the check is not vacuous).
throughput: decisions per second, simulated drone-seconds per second, and
memory, at a given drone count (M3b measured the stock quadcopter task).
clip: 4 drones, fixed zoomed-out camera, one frame per decision, played at
4x real time.
"""
from __future__ import annotations

import argparse
import json
import os
import resource
import sys
import time
from pathlib import Path

NOMINAL = (1.0, 0.0, 0.0)


def _env(num_envs: int, fixed_severities=None, spacing=None, render=False):
    import torch  # noqa: F401  (after the app is up)
    from aero_isaac.env import AeroEnv, AeroEnvCfg
    cfg = AeroEnvCfg()
    cfg.scene.num_envs = num_envs
    if spacing:
        cfg.scene.env_spacing = spacing
    if fixed_severities:
        cfg.fixed_severities = tuple(fixed_severities)
    if render:
        cfg.viewer.eye, cfg.viewer.lookat = (-19.0, 16.0, 16.0), (8.0, -8.0, 0.0)
    return AeroEnv(cfg, render_mode="rgb_array" if render else None)


def smoke() -> dict:
    import torch
    sev = [0.0, 0.3, 0.45, 0.7] * 4
    env = _env(len(sev), fixed_severities=sev)
    obs, _ = env.reset()
    act = torch.tensor([NOMINAL], device=env.device).repeat(env.num_envs, 1)
    out = dict(num_envs=env.num_envs, obs_shape=list(obs["policy"].shape), decision_period_s=env.step_dt,
               physics_dt_s=env.physics_dt, all_finite=bool(torch.isfinite(obs["policy"]).all()),
               first_episode={}, back_on_ground={}, leak=None)
    for step in range(450):
        obs, _, terminated, truncated, _ = env.step(act)
        out["all_finite"] &= bool(torch.isfinite(obs["policy"]).all())
        for e in env.episode_log:
            if str(e["env"]) not in out["first_episode"]:
                out["first_episode"][str(e["env"])] = e
                pos = env._state()[0][e["env"]]
                out["back_on_ground"][str(e["env"])] = bool(int(env._substep[e["env"]]) == 0 and
                                                            float(pos.norm()) < 0.05)
        env.episode_log.clear()
        if step == 175 and out["leak"] is None:          # 35 s: every fault has started
            out["leak"] = _leak_check(env)
        if len(out["first_episode"]) == env.num_envs:
            break
    out["decisions"] = step + 1
    return out


def _leak_check(env) -> dict:
    """Same physics, same detector output, same noise draw; only the true
    fault changes."""
    import torch
    truth = ("_fault_rotor", "_fault_severity", "_fault_onset", "_fault_ramp")
    saved = {k: getattr(env, k).clone() for k in truth}
    g = env._gen.get_state()
    base = env._get_observations()["policy"].clone()
    env._fault_rotor[:] = (saved["_fault_rotor"] + 2) % 4
    env._fault_severity[:] = 1.0 - saved["_fault_severity"]
    env._fault_onset[:] = saved["_fault_onset"] + 7.0
    env._fault_ramp[:] = saved["_fault_ramp"] + 2.0
    env._gen.set_state(g)
    changed_truth = env._get_observations()["policy"].clone()
    for k, v in saved.items():
        getattr(env, k).copy_(v)
    det = env._det
    env._det = {k: (v + 0.25 if k == "severity" else v) for k, v in det.items()}
    env._gen.set_state(g)
    changed_detector = env._get_observations()["policy"].clone()
    env._det = det
    env._gen.set_state(g)
    return dict(max_change_true_fault=float((changed_truth - base).abs().max()),
                max_change_detector=float((changed_detector - base).abs().max()),
                faulted_drones=int((saved["_fault_rotor"] >= 0).sum()))


def throughput(num_envs: int, decisions: int) -> dict:
    import torch
    env = _env(num_envs)
    env.reset()
    act = torch.tensor([NOMINAL], device=env.device).repeat(env.num_envs, 1)
    for _ in range(10):                                   # warm-up
        env.step(act)
    torch.cuda.synchronize()
    rss0 = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    t0 = time.perf_counter()
    for _ in range(decisions):
        env.step(act)
    torch.cuda.synchronize()
    wall = time.perf_counter() - t0
    free, total = torch.cuda.mem_get_info()
    return dict(num_envs=num_envs, decisions_timed=decisions, wall_s=wall,
                decisions_per_s=num_envs * decisions / wall,
                physics_steps_per_s=num_envs * decisions * env.cfg.decimation / wall,   # M3b's measure
                sim_drone_s_per_wall_s=num_envs * decisions * env.step_dt / wall,
                physics_steps_per_decision=env.cfg.decimation,
                gpu_used_gb=(total - free) / 1e9, gpu_total_gb=total / 1e9,
                peak_rss_gb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024 ** 2,
                rss_growth_mb_while_timed=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024 - rss0,
                episodes_finished=len(env.episode_log))


def clip(out_path: str) -> dict:
    import imageio.v2 as imageio
    import torch
    env = _env(4, fixed_severities=[0.0, 0.3, 0.45, 0.7], spacing=20.0, render=True)
    env.reset()
    act = torch.tensor([NOMINAL], device=env.device).repeat(env.num_envs, 1)
    for _ in range(5):
        env.render()                                      # the renderer warms up on empty frames
    frames = []
    for _ in range(330):                                  # 66 s: takeoff, mission, landing
        env.step(act)
        frames.append(env.render())
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(out_path, frames, fps=20)
    imageio.imwrite(str(Path(out_path).with_suffix(".png")), frames[len(frames) // 3])
    return dict(out=out_path, frames=len(frames), severities=[0.0, 0.3, 0.45, 0.7], speedup=4)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("smoke", "throughput", "clip"))
    ap.add_argument("--out", required=True)
    ap.add_argument("--num-envs", type=int, default=8192)
    ap.add_argument("--decisions", type=int, default=100)
    args = ap.parse_args(argv)

    from isaaclab.app import AppLauncher
    AppLauncher(headless=True, enable_cameras=args.mode == "clip")
    if args.mode == "smoke":
        result = smoke()
    elif args.mode == "throughput":
        result = throughput(args.num_envs, args.decisions)
    else:
        result = clip(args.out)
    if args.mode != "clip":
        Path(args.out).parent.mkdir(parents=True, exist_ok=True)
        Path(args.out).write_text(json.dumps(result, indent=2))
    print(json.dumps({k: v for k, v in result.items() if k != "first_episode"}), flush=True)
    sys.stdout.flush()
    os._exit(0)   # Isaac Sim's shutdown never returns headless


if __name__ == "__main__":
    main()
