"""M8b task 6: the simulated detector against flights held out of its fit
(tests/fixtures/detector_sim_heldout_v1.json, written by the PX4 side's
experiments/fit_detector_sim.py from M8b's confirmation run, flown after the
model was frozen). Pure PyTorch: the
held-out flights' conditions -- fault, onset, descent start -- are replayed
through the simulator, many times each, and its statistics compared with the
real detector's.

Tolerances, fixed before the first comparison:
  detection delay, median per (severity band, onset type) with >= 8 flights: within 0.3 s
  false alarms (runs of p >= 0.1) per healthy hour: within a factor of 1.5
  false alarms that matter (p >= 0.5 for >= 0.3 s) per healthy hour: within a factor of 2
  severity over-read in the first 3 s of a descent, median per 1 s bin: within 0.05
"""
import json
import math
from pathlib import Path

import pytest
import torch

from aero_isaac.detector_sim import DetectorSim, load_params

FIXTURE = Path(__file__).resolve().parent.parent.parent / "tests" / "fixtures" / "detector_sim_heldout_v1.json"
REPEATS = 20
DT = 0.1


@pytest.fixture(scope="module")
def replay():
    """Simulate every held-out flight REPEATS times; per tick record what
    the statistics need."""
    fx = json.loads(FIXTURE.read_text())
    eps = [e for e in fx["episodes"] for _ in range(REPEATS)]
    n = len(eps)
    gen = torch.Generator().manual_seed(7)
    sim = DetectorSim(n, generator=gen)
    f = lambda key, default: torch.tensor([e[key] if e[key] is not None else default for e in eps],  # noqa: E731
                                          dtype=torch.float32)
    target = f("severity", 0.0)
    onset = f("onset_s", math.inf)
    ramp_s = f("ramp_s", 0.0)
    is_ramp = torch.tensor([e["profile"] == "ramp" for e in eps])
    rotor = torch.where(target > 0, torch.zeros(n, dtype=torch.long), torch.full((n,), -1))
    descent = f("descent_start_s", math.inf)
    duration = f("duration_s", 0.0)
    sim.reset(torch.arange(n), target=target, rotor=rotor, onset=onset, ramp_s=torch.where(is_ramp, ramp_s, 0.0))
    steps = int(duration.max() / DT) + 1
    rec = {k: [] for k in ("t", "sev", "p", "est")}
    for i in range(steps):
        t = torch.full((n,), i * DT)
        since = t - onset
        frac = torch.where(is_ramp & (ramp_s > 0), (since / ramp_s.clamp(min=1e-6)).clamp(0, 1), torch.ones(n))
        sev = torch.where(since >= 0, target * frac, torch.zeros(n))
        out = sim.step(t=t, severity=sev, offset_cmd=torch.where(t >= descent, torch.full((n,), -3.0),
                                                                  torch.zeros(n)))
        for k, v in (("t", t), ("sev", sev), ("p", out["p_fault"]), ("est", out["severity"])):
            rec[k].append(v)
    rec = {k: torch.stack(v, 1) for k, v in rec.items()}                 # (n, steps)
    live = torch.arange(steps)[None, :] * DT <= duration[:, None]
    return fx, eps, rec, live, onset, descent, target, is_ramp, sim


def _bursts(mask_row):
    edges = torch.diff(torch.cat([torch.tensor([0]), mask_row.int(), torch.tensor([0])]))
    return list(zip(torch.nonzero(edges == 1).flatten().tolist(), torch.nonzero(edges == -1).flatten().tolist()))


CELLS = [c for c in json.loads(FIXTURE.read_text())["real"]["delays"]
         if c["n"] >= 8 and c["median_s"] is not None]
# The one miss on the confirmation run, recorded rather than tuned away
# (user decision 2026-09-24; docs/isaac_env.md): simulated 1.70 s, real 1.12 s.
KNOWN_GAP = ((0.5, 0.7), "ramp")


@pytest.mark.parametrize("cell", [
    pytest.param(c, id=f"{c['band'][0]}-{c['band'][1]}-{c['profile']}",
                 marks=[pytest.mark.xfail(strict=True, reason="known gap: ramps at 0.5-0.7 detected 0.58 s late")]
                 if (tuple(c["band"]), c["profile"]) == KNOWN_GAP else [])
    for c in CELLS])
def test_detection_delay_matches_held_out(replay, cell):
    fx, eps, rec, live, onset, _, target, is_ramp, sim = replay
    lo, hi = cell["band"]
    sel = (target >= lo) & (target < hi) & (is_ramp == (cell["profile"] == "ramp"))
    delays = []
    for i in torch.nonzero(sel).flatten().tolist():
        on = int(round(onset[i].item() / DT))
        hit = torch.nonzero((rec["p"][i, on:] >= 0.5) & live[i, on:]).flatten()
        if len(hit):
            delays.append(hit[0].item() * DT)
    got = float(torch.tensor(delays).median())
    assert abs(got - cell["median_s"]) <= 0.3, (cell, got)


def test_enough_delay_cells_are_checked():
    assert len(CELLS) >= 6


def test_false_alarm_rates_match_held_out(replay):
    fx, eps, rec, live, onset, descent, _, _, _ = replay
    healthy = live & (rec["sev"] <= 0) & (rec["t"] < descent[:, None])
    hours = healthy.sum().item() * DT / 3600
    n_all, n_big = 0, 0
    for i in range(len(eps)):
        for a, b in _bursts((rec["p"][i] >= 0.1) & healthy[i]):
            n_all += 1
            if rec["p"][i, a:b].max() >= 0.5 and (b - a) * DT >= 0.3:
                n_big += 1
    real = fx["real"]
    ratio_all = (n_all / hours) / real["false_alarms_per_hour"]
    ratio_big = (n_big / hours) / real["big_false_alarms_per_hour"]
    print(f"false alarms/h sim {n_all / hours:.1f} real {real['false_alarms_per_hour']:.1f}; "
          f"that matter sim {n_big / hours:.1f} real {real['big_false_alarms_per_hour']:.1f}")
    assert 1 / 1.5 <= ratio_all <= 1.5
    assert 0.5 <= ratio_big <= 2.0


def test_descent_over_read_matches_held_out(replay):
    fx, eps, rec, live, _, descent, target, _, sim = replay
    bias = sim.bias[(torch.bucketize(target, sim.bands, right=True) - 1).clamp(0, len(sim.bands) - 2)]
    tau = rec["t"] - descent[:, None]
    excess = rec["est"] - rec["sev"] - bias[:, None]
    ok = live & (rec["p"] >= 0.5) & (rec["sev"] > 0)
    for b in fx["real"]["descent"]["bins"][:3]:
        if b["n"] < 20:
            continue
        sel = ok & (tau >= b["from_s"]) & (tau < b["to_s"])
        got = float(excess[sel].median())
        print(f"descent {b['from_s']}-{b['to_s']} s: sim {got:.3f} real {b['median']:.3f}")
        assert abs(got - b["median"]) <= 0.05


def test_never_detects_before_onset_or_on_healthy_flights():
    sim = DetectorSim(4, generator=torch.Generator().manual_seed(1))
    sim.reset(torch.arange(4), target=torch.tensor([0.0, 0.5, 0.5, 0.3]), rotor=torch.tensor([-1, 2, 1, 0]),
              onset=torch.tensor([math.inf, 5.0, 5.0, 5.0]), ramp_s=torch.tensor([0.0, 0.0, 3.0, 3.0]))
    assert torch.isinf(sim.detect_at[0]) and (sim.detect_at[1:] >= 5.0).all()


def test_ramp_is_detected_when_it_reaches_the_drawn_level():
    p = load_params()
    nb = len(p["severity_bands"]) - 1
    p["detection"]["ramp"]["level_quantiles"] = [[0.1] * 21] * nb
    p["detection"]["step"]["delay_s_quantiles"] = [[0.2] * 21] * nb
    p["detection"]["ramp"]["miss_rate"] = p["detection"]["step"]["miss_rate"] = [0.0] * nb
    sim = DetectorSim(3, generator=torch.Generator().manual_seed(3), params=p)
    sim.reset(torch.arange(3), target=torch.tensor([0.4, 0.05, 0.4]), rotor=torch.tensor([0, 1, 2]),
              onset=torch.full((3,), 5.0), ramp_s=torch.tensor([4.0, 2.0, 0.0]))
    # 0.1 of 0.4 is a quarter of a 4 s ramp; a 0.05 ramp never reaches 0.1, so a step delay after it ends
    assert sim.detect_at.tolist() == pytest.approx([6.0, 7.2, 5.2])


def test_named_rotor_is_right_at_the_fitted_rate():
    n = 4000
    sim = DetectorSim(n, generator=torch.Generator().manual_seed(2))
    rotor = torch.randint(0, 4, (n,))
    sim.reset(torch.arange(n), target=torch.full((n,), 0.45), rotor=rotor, onset=torch.zeros(n),
              ramp_s=torch.zeros(n))
    acc = (sim.rotor_named == rotor).float().mean().item()
    assert acc == pytest.approx(sim.rotor_acc[2].item(), abs=0.01)


def test_parameters_record_their_source():
    p = load_params()
    assert p["fitted_from"]["held_out"] == ["m8b_detector_confirm_nominal", "m8b_detector_confirm_recovery"]
    assert not set(p["fitted_from"]["held_out"]) & set(p["fitted_from"]["live_runs"])
    assert p["fitted_from"]["detector_checkpoint_digest"]
