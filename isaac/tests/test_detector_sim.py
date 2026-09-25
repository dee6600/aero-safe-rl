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


# ---------------------------------------------------------------- M9 training knobs

def _run_knobs(knobs, n=2000, seconds=40.0, seed=11, target=0.4, ramp=0.0, descent_s=math.inf):
    """Fly n identical episodes (fault at 10 s) through the simulator; per
    tick record the estimate, probability and true severity."""
    sim = DetectorSim(n, generator=torch.Generator().manual_seed(seed))
    k = {name: torch.full((n,), float(v)) for name, v in knobs.items()} if knobs is not None else None
    sim.reset(torch.arange(n), target=torch.full((n,), target), rotor=torch.zeros(n, dtype=torch.long),
              onset=torch.full((n,), 10.0), ramp_s=torch.full((n,), ramp), knobs=k)
    rec = {"p": [], "est": [], "sev": [], "t": []}
    for i in range(int(seconds / DT)):
        t = torch.full((n,), i * DT)
        frac = ((t - 10.0) / ramp).clamp(0, 1) if ramp > 0 else torch.ones(n)
        sev = torch.where(t >= 10.0, target * frac, torch.zeros(n))
        out = sim.step(t=t, severity=sev, offset_cmd=torch.where(t >= descent_s, torch.full((n,), -3.0),
                                                                  torch.zeros(n)))
        for key, v in (("p", out["p_fault"]), ("est", out["severity"]), ("sev", sev), ("t", t)):
            rec[key].append(v)
    return sim, {key: torch.stack(v, 1) for key, v in rec.items()}


def test_neutral_knobs_change_nothing():
    """Explicit neutral knobs give bit-identical output to no knobs at all,
    from the same random stream -- the held-out check above stays valid."""
    _, a = _run_knobs(None, n=300, ramp=3.0, descent_s=20.0)
    _, b = _run_knobs(dict(delay_scale=1, false_alarm_rate_scale=1, severity_noise_scale=1,
                           severity_bias_shift=0, descent_overread_scale=1), n=300, ramp=3.0, descent_s=20.0)
    for key in a:
        assert torch.equal(a[key], b[key]), key


def test_delay_scale_stretches_detection_time():
    base, _ = _run_knobs(None, ramp=4.0, seconds=0.1)
    slow, _ = _run_knobs(dict(delay_scale=2.0), ramp=4.0, seconds=0.1)
    d0, d1 = (base.detect_at - 10.0).median(), (slow.detect_at - 10.0).median()
    assert float(d1) == pytest.approx(2 * float(d0), rel=0.05)


def test_false_alarm_rate_scale_multiplies_bursts():
    def bursts(scale):
        _, r = _run_knobs(dict(false_alarm_rate_scale=scale), target=0.0, n=400, seconds=300.0)
        return sum(len(_bursts(r["p"][i] >= 0.1)) for i in range(r["p"].shape[0]))
    b1, b3 = bursts(1.0), bursts(3.0)
    assert 2.4 <= b3 / b1 <= 3.4


def test_noise_scale_and_bias_shift_move_the_estimate_error():
    def err(knobs):
        sim, r = _run_knobs(knobs, seconds=30.0)
        detected = (r["t"] >= sim.detect_at[:, None] + 2.0)   # well after detection, flying level
        return (r["est"] - r["sev"])[detected]
    e0, e_noise, e_bias = err(None), err(dict(severity_noise_scale=2.0)), err(dict(severity_bias_shift=0.03))
    assert float(e_noise.std()) == pytest.approx(2 * float(e0.std()), rel=0.1)
    assert float(e_bias.mean() - e0.mean()) == pytest.approx(0.03, abs=0.004)


def test_descent_overread_scale_scales_the_over_read():
    def over(scale):
        _, r = _run_knobs(dict(descent_overread_scale=scale), seconds=26.0, descent_s=20.0)
        sel = r["t"] >= 21.0
        return float((r["est"] - r["sev"])[sel].mean())
    _, r = _run_knobs(None, seconds=19.9)
    level = float((r["est"] - r["sev"])[r["t"] >= 15.0].mean())    # the error before any descent
    assert (over(2.0) - level) == pytest.approx(2 * (over(1.0) - level), rel=0.1)


def test_unknown_knob_is_refused():
    sim = DetectorSim(1)
    with pytest.raises(ValueError):
        sim.reset(torch.arange(1), target=torch.zeros(1), rotor=torch.full((1,), -1), onset=torch.zeros(1),
                  ramp_s=torch.zeros(1), knobs={"latency": torch.ones(1)})
