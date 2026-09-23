"""M8b task 1: record the PX4-side contract code's behaviour into a fixture
the Isaac side is tested against.

The two environments share files, never imports (CLAUDE.md §0.1), so the
Isaac side re-implements action decoding, the mission tracker, the outcome
rule and the observation layout in PyTorch. This script runs the real
PX4-side implementations on seeded synthetic inputs and writes inputs and
outputs to tests/fixtures/isaac_contract_v1.json; isaac/tests/ replays the
same inputs and must reproduce the outputs.

    python experiments/write_isaac_fixtures.py            # (re)write the fixture
    python experiments/write_isaac_fixtures.py --check    # exit 1 if it would change

Inputs are synthetic and seeded -- nothing is read from results/ -- so the
fixture is identical on any machine. Regenerating it is a deliberate act
(CLAUDE.md §6): do it only when a contract file changes, and bump the
contract's version when you do.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

FIXTURE = REPO / "tests" / "fixtures" / "isaac_contract_v1.json"
MISSION = REPO / "configs" / "missions" / "square_circuit.yaml"
SEED = 20260924
DT = 0.1  # the PX4 side's step-record period (mission_executor.CONTROL_PERIOD_S)


def _round(x, nd=9):
    if isinstance(x, dict):
        return {k: _round(v, nd) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [_round(v, nd) for v in x]
    if isinstance(x, (bool, np.bool_)):
        return bool(x)
    if isinstance(x, (int, np.integer)):
        return int(x)
    return None if not math.isfinite(float(x)) else round(float(x), nd)


def action_cases(rng) -> list[dict]:
    from rl.policies.base_policy import load_action_spec
    spec = load_action_spec()
    vectors = [[1.0, 0.0, 0.0], [0.0, -3.5, 1.0], [1.5, 2.0, 0.5], [-0.2, -9.0, 0.4999]]
    vectors += rng.uniform([-0.5, -5.0, -0.5], [1.5, 1.0, 1.5], size=(20, 3)).tolist()
    return [dict(vector=_round(v), decoded=_round(spec.decode(v).to_vector())) for v in vectors]


def tracker_cases(rng) -> list[dict]:
    """Trajectories of MissionTracker driven by a lagging vehicle under
    piecewise-constant actions. Per step: inputs (t, position, action) and
    the outputs fly_mission uses -- progress (computed BEFORE update, as
    fly_mission does), then setpoint/target/waypoints_reached/done after it."""
    from aero_bridge.mission_executor import load_mission
    from rl.mission_tracker import MissionTracker
    from rl.policies.base_policy import load_action_spec
    spec, mission = load_action_spec(), load_mission(MISSION)
    plans = [
        ("nominal_full_mission", 460, lambda i: [1.0, 0.0, 0.0]),
        ("slow_then_low", 320, lambda i: [0.3, 0.0 if i < 120 else -3.0, 0.0]),
        ("random_pieces", 320, None),
    ]
    out = []
    for name, n, plan in plans:
        tr = MissionTracker(mission, spec, start_xy=(0.0, 0.0))
        pos = np.zeros(3)
        tau = 0.6 if name != "random_pieces" else 0.9
        action_vec = [1.0, 0.0, 0.0]
        rows = {k: [] for k in ("t", "position", "action", "progress", "setpoint", "target",
                                "waypoints_reached", "done", "altitude_offset_m")}
        for i in range(n):
            t = round(i * DT, 10)
            if plan is not None:
                action_vec = plan(i)
            elif i % 40 == 0:
                action_vec = [float(rng.choice([1.0, 0.6, 0.25, 0.0])),
                              float(rng.choice([0.0, -1.5, -3.5])), 0.0]
            action = spec.decode(action_vec)
            p = tr.progress(t, tuple(pos))
            tr.update(t, tuple(pos), action)
            rows["t"].append(t)
            rows["position"].append(_round(pos.tolist()))
            rows["action"].append(_round(action.to_vector()))
            rows["progress"].append(_round([p.waypoint_index, p.n_waypoints, p.distance_to_waypoint_m,
                                            p.altitude_m, p.elapsed_s]))
            rows["setpoint"].append(_round(list(tr.setpoint)))
            rows["target"].append(_round(list(tr.target)))
            rows["waypoints_reached"].append(tr.waypoints_reached)
            rows["done"].append(tr.done)
            rows["altitude_offset_m"].append(_round(tr.altitude_offset_m))
            # A first-order vehicle chasing the setpoint (not physics -- only
            # a source of realistic positions for the tracker's inputs).
            pos = pos + (np.array(tr.setpoint) - pos) * (DT / tau)
        out.append(dict(name=name, **rows))
    return out


def outcome_cases(rng) -> list[dict]:
    from experiments.metrics import classify_outcome
    cases = []

    def add(reason, alt, vz, tilt_deg):
        alt, vz, tilt = (np.asarray(a, float) for a in (alt, vz, tilt_deg))
        steps = dict(pos_z=-alt, vel_z=vz, roll_rad=np.radians(tilt), pitch_rad=np.zeros_like(alt))
        o = classify_outcome(reason, steps)
        cases.append(dict(termination_reason=reason, alt=_round(alt.tolist()), vel_z=_round(vz.tolist()),
                          roll_deg=_round(tilt.tolist()), outcome=o.outcome.value,
                          touchdown_speed_m_s=_round(o.touchdown_speed_m_s),
                          max_tilt_deg=_round(o.max_tilt_deg)))

    takeoff = [0.0, 0.1, 0.6, 1.5, 3.0, 4.5, 5.0, 5.0]
    for reason in ("completed", "recovery_landed", "ground_contact", "hold_timeout", "episode_timeout"):
        for _ in range(4):
            speed = float(rng.uniform(0.3, 6.0))
            alt, a = list(takeoff), 5.0
            while a > 0.0:
                a = max(0.0, a - speed * DT)
                alt.append(a)
            alt += [0.0, 0.0]
            vz = [0.0] * len(takeoff) + [speed] * (len(alt) - len(takeoff) - 2) + [0.0, 0.0]
            tilt = rng.uniform(0.0, 50.0, size=len(alt))
            if rng.random() < 0.3:
                tilt[rng.integers(len(takeoff), len(alt))] = rng.uniform(55.0, 120.0)
            add(reason, alt, vz, tilt)
    add("preflight_failed", [0.0, 0.0, 0.1], [0.0, 0.0, 0.0], [0.0, 80.0, 0.0])   # never airborne
    add("offboard_lost", takeoff, [0.0] * len(takeoff), [0.0] * len(takeoff))       # airborne, no contact
    add("ground_contact", takeoff + [2.0, 0.6, 0.0], [0.0] * len(takeoff) + [3.0, 3.4, 0.0],
        [0.0] * (len(takeoff) + 3))                                                  # speed on the tick before contact
    return cases


def observation_cases(rng) -> list[dict]:
    from ai.detector.runtime import DetectorOutput
    from rl.policies.base_policy import (
        Action, MissionProgress, PolicyInput, flatten_observation, load_observation_spec)
    spec = load_observation_spec()
    names = [e["source"].split(".", 1)[1] for e in spec.entries if e["source"].startswith("feature.")]
    cases = []
    for _ in range(12):
        # Rounded before use, so the recorded inputs are exactly what produced the vector.
        features = _round({n: spec.norm[n][0] + spec.norm[n][1] * rng.normal() for n in names})
        det = dict(p_fault=_round(rng.uniform()), rotor=int(rng.integers(0, 4)),
                   severity=_round(rng.uniform(0, 0.9)), uncertainty=_round(rng.uniform(0, 0.2)),
                   alarm=bool(rng.random() < 0.5))
        mission = dict(waypoint_index=int(rng.integers(0, 6)), n_waypoints=5,
                       distance_to_waypoint_m=_round(rng.uniform(0, 20)),
                       altitude_m=_round(rng.uniform(0, 6)), elapsed_s=_round(rng.uniform(0, 120)))
        prev = dict(speed_scale=_round(rng.uniform()), altitude_offset_m=_round(rng.uniform(-3.5, 0)),
                    land=bool(rng.random() < 0.2))
        obs = PolicyInput(t_sim_s=0.0, features=features, detector=DetectorOutput(**det),
                          mission=MissionProgress(**mission), previous_action=Action(**prev))
        cases.append(dict(features=features, detector=det, mission=mission,
                          previous_action=prev,
                          vector=_round(flatten_observation(obs, spec).tolist(), 6)))
    return cases


def build() -> dict:
    from experiments.episode_schema import digest
    from rl.policies.base_policy import load_action_spec, load_observation_spec
    import yaml
    rng = np.random.default_rng(SEED)
    return dict(
        fixture_version="1",
        written_by="experiments/write_isaac_fixtures.py",
        seed=SEED,
        # Digests of the contract files this fixture was recorded under; the
        # Isaac side recomputes them and refuses a mismatch.
        contract_files={
            "configs/rl/action_v1.yaml": load_action_spec().digest,
            "configs/rl/observation_v2.yaml": load_observation_spec().digest,
            "configs/rl/outcome_v1.yaml": digest(yaml.safe_load(
                (REPO / "configs/rl/outcome_v1.yaml").read_text())),
            "configs/missions/square_circuit.yaml": digest(yaml.safe_load(MISSION.read_text())),
        },
        observation_names=list(load_observation_spec().names),
        action=action_cases(rng),
        tracker=tracker_cases(rng),
        outcome=outcome_cases(rng),
        observation=observation_cases(rng),
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true")
    args = ap.parse_args(argv)
    text = json.dumps(build(), separators=(",", ":"), sort_keys=True) + "\n"
    if args.check:
        same = FIXTURE.exists() and FIXTURE.read_text() == text
        print("fixture up to date" if same else "fixture would change")
        return 0 if same else 1
    FIXTURE.write_text(text)
    print(f"wrote {FIXTURE} ({len(text) / 1024:.0f} KiB)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
