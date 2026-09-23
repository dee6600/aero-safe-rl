"""M8b task 6: fit the simulated detector to the real detector's measured
behaviour, for the Isaac environment (CLAUDE.md §1.7 -- the policy trains on
the detector's estimate, never the true fault; in Isaac the real detector
cannot run, so a fitted stand-in produces that estimate).

    python experiments/fit_detector_sim.py

Fit data, only flights the detector never trained on (see main()):
  * the M6 test split (results/m6_dataset_v1) replayed through the M7
    detector -- level mission flight;
  * the M8 live runs with the detector attached (check, sweep, tie-break,
    validation) and M8b's low-severity recovery run -- the same detector
    running inside real flights, including recovery descents, which is where
    M8 found the severity estimate over-reads.
Held out: M8b's confirmation run (m8b_detector_confirm_*), flown after this
model was frozen. Its per-episode conditions and real statistics go to
tests/fixtures/detector_sim_heldout_v1.json (written once those flights
exist), and
the Isaac side (isaac/tests/test_detector_sim.py) simulates the same
conditions and compares -- files only, no shared code (CLAUDE.md §0.1).

The model (configs/rl/detector_sim_v1.yaml records every number):
  healthy     background probability; false-alarm bursts (runs of
              p >= 0.1) at the measured rate, duration and peak -- live
              flights only (see main()).
  fault       detection delay after onset, per severity band and onset type,
              or a miss; after detection the probability, rotor accuracy,
              and severity = true + bias + correlated noise (AR(1)).
  fault       (ramps) detection when the true severity reaches a drawn
              level (per band), so it carries over to other ramp lengths;
              a level at or above the target means detection after the
              ramp ends, one step delay later.
  descent     when a commanded altitude drop starts, an extra severity
              error with the measured shape over time, scaled per descent.
  First version (independent burst length and height, a fixed-delay ramp
  model, an exponential descent term, ramp lengths read off the trace, all
  M6 flights) failed held-out checks; see docs/isaac_env.md for each
  revision and why it was made. Every revision was refit on fit data only.
Distributions are stored as 21 quantiles and sampled by interpolation.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

CHECKPOINT = REPO / "results" / "m7_detector_v1" / "detector.pt"
# m8b_px4_descents_low: rule-based recovery at severity 0.20-0.30, flown because
# the other runs had no recovery descents below 0.35 to fit the over-read on.
# m8_validation_*: the held-out set while the model was being revised; it
# steered those revisions, so it became fit data and a fresh run the check.
FIT_RUNS = ["m8_check_nominal", "m8_fsm_sweep_*", "m8_fsm_tiebreak_*", "m8b_px4_descents_low", "m8_validation_*"]
HELDOUT_RUNS = ["m8b_detector_confirm_nominal", "m8b_detector_confirm_recovery"]
OUT = REPO / "configs" / "rl" / "detector_sim_v1.yaml"
HELDOUT = REPO / "tests" / "fixtures" / "detector_sim_heldout_v1.json"

TICK_S = 0.1
QS = np.linspace(0.0, 1.0, 21)
BUCKETS = (0.0, 0.3, 0.4, 0.5, 0.7, 1.01)       # severity bands [lo, hi)
DETECTED_P = 0.5                                # "detected": the probability's near-binary switch
BURST_P = 0.1                                   # a false-alarm burst: a healthy run above this
MISS_MIN_OBSERVED_S = 5.0                       # undetected for this long after onset counts as a miss
DESCENT_BINS_S = (0.0, 1.0, 2.0, 3.0, 4.0, 6.0)


def _q(x) -> list[float]:
    x = np.asarray(x, float)
    x = x[np.isfinite(x)]
    return [round(float(v), 6) for v in np.quantile(x, QS)] if len(x) else []


def bucket(sev: float) -> int:
    return int(np.searchsorted(BUCKETS, sev, side="right") - 1)


# ----------------------------------------------------------------- traces

def m6_ramp_lengths(run_dir) -> dict[str, float]:
    """Commanded ramp length per M6 episode key ("<worker_id>/<episode_id>")."""
    out = {}
    for sp in Path(run_dir).glob("worker_*/episode_*_summary.parquet"):
        s = pd.read_parquet(sp, columns=["worker_id", "episode_id", "fault_ramp_duration_s"]).iloc[0]
        out[f"{s.worker_id}/{s.episode_id}"] = float(s.fault_ramp_duration_s)
    return out


def trace_m6(e, tr, ramp_s: float) -> dict:
    return dict(t=e.elapsed_s, sev=e.severity.astype(float), rotor=int(e.rotor_class.max()) - 1,
                profile=e.profile, target=e.severity_commanded, ramp_s=ramp_s,
                p=tr.score, rotor_pred=tr.rotor, sev_est=tr.severity, unc=tr.uncertainty,
                descent_start=None, source="m6")


def trace_live(summary, steps) -> dict:
    from experiments.fault_schedule import commanded_severity
    m = steps[steps.flight_phase == "mission"]
    t = m.t_sim_s.to_numpy(float) - float(summary.t_sim_start_s)
    if summary.fault_applied:
        sev = np.array([commanded_severity(summary.fault_severity_commanded, summary.fault_profile,
                                           summary.fault_ramp_duration_s, x - summary.fault_onset_time_s_requested)
                        for x in t])
    else:
        sev = np.zeros(len(t))
    lower = np.flatnonzero(m.action_altitude_offset_m.to_numpy(float) < -0.05)
    return dict(t=t, sev=sev, rotor=int(summary.fault_rotor_index), profile=str(summary.fault_profile),
                target=float(summary.fault_severity_commanded), ramp_s=float(summary.fault_ramp_duration_s),
                p=m.det_p_fault.to_numpy(float), rotor_pred=m.det_rotor.to_numpy(float),
                sev_est=m.det_severity.to_numpy(float), unc=m.det_uncertainty.to_numpy(float),
                descent_start=float(t[lower[0]]) if len(lower) else None, source=str(summary.run_id))


def live_traces(patterns) -> list[dict]:
    out = []
    for pat in patterns:
        for run in sorted((REPO / "results").glob(pat)):
            for sp in sorted(run.glob("worker_*/episode_*_summary.parquet")):
                s = pd.read_parquet(sp).iloc[0]
                if not s.valid or s.n_steps == 0:
                    continue
                st = pd.read_parquet(sp.with_name(sp.name.replace("_summary", "_steps")))
                if "det_p_fault" not in st or st.det_p_fault.isna().all():
                    continue
                if s.fault_applied and not np.isfinite(s.fault_onset_time_s_observed):
                    continue          # fault never took effect
                tr = trace_live(s, st)
                if len(tr["t"]) > 5:
                    out.append(tr)
    return out


# ----------------------------------------------------------------- statistics

def runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """(start, end) index pairs of True runs, end exclusive."""
    edges = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def healthy_stats(traces) -> dict:
    hours, durations, peaks, burst_sev, bg_p, bg_sev, bg_unc = 0.0, [], [], [], [], [], []
    for tr in traces:
        h = tr["sev"] <= 0
        if tr["descent_start"] is not None:
            h &= tr["t"] < tr["descent_start"]
        hours += h.sum() * TICK_S / 3600
        high = h & (tr["p"] >= BURST_P)
        for a, b in runs(high):
            durations.append((b - a) * TICK_S)
            peaks.append(tr["p"][a:b].max())
            burst_sev.extend(tr["sev_est"][a:b])
        quiet = h & ~high
        bg_p.extend(tr["p"][quiet]); bg_sev.extend(tr["sev_est"][quiet]); bg_unc.extend(tr["unc"][quiet])
    return dict(hours=hours, n_bursts=len(durations), durations=durations, peaks=peaks, burst_sev=burst_sev,
                bg_p=bg_p, bg_sev=bg_sev, bg_unc=bg_unc)


def ramp_duration(tr) -> float | None:
    """The commanded ramp length; None for a step. Taken from the fault
    command, not from the trace: a trace can end before the ramp does (the
    recovery controller landed), and such a flight is still a ramp of that
    length. (The first version read it off the trace, recorded those flights
    as length 0, and the held-out check then replayed them as steps.)"""
    if tr["profile"] != "ramp" or tr["target"] <= 0 or not tr["ramp_s"] > 0:
        return None
    return float(tr["ramp_s"])


def fault_stats(traces) -> dict:
    """Detection delays per (band, profile), and detected-tick behaviour."""
    delays = {(b, p): [] for b in range(len(BUCKETS) - 1) for p in ("step", "ramp")}
    ramp_level = {b: [] for b in range(len(BUCKETS) - 1)}
    misses = {k: 0 for k in delays}
    det_p, det_unc, rotor_hit = [], [], {b: [] for b in range(len(BUCKETS) - 1)}
    err = {b: [] for b in range(len(BUCKETS) - 1)}
    pairs = []
    for tr in traces:
        active = np.flatnonzero(tr["sev"] > 0)
        if not len(active) or tr["profile"] not in ("step", "ramp"):
            continue
        b, on = bucket(tr["target"]), active[0]
        key = (b, tr["profile"])
        hit = np.flatnonzero(tr["p"][on:] >= DETECTED_P)
        if len(hit):
            delays[key].append(float(tr["t"][on + hit[0]] - tr["t"][on]))
            ramp = ramp_duration(tr)
            if ramp:
                # the severity reached at detection, on the same clock as the delay
                ramp_level[b].append(tr["target"] * min((tr["t"][on + hit[0]] - tr["t"][on]) / ramp, 1.0))
        elif tr["t"][-1] - tr["t"][on] >= MISS_MIN_OBSERVED_S:
            misses[key] += 1
        det = np.zeros(len(tr["t"]), bool)
        if len(hit):
            det[on + hit[0]:] = True
        det_p.extend(tr["p"][det]); det_unc.extend(tr["unc"][det])
        level = det & (tr["p"] >= DETECTED_P)
        if tr["descent_start"] is not None:
            level &= tr["t"] < tr["descent_start"]
        rotor_hit[b].extend(tr["rotor_pred"][level] == tr["rotor"])
        e = tr["sev_est"] - tr["sev"]
        err[b].extend(e[level])
        idx = np.flatnonzero(level[:-1] & level[1:])
        pairs.extend(zip(e[idx], e[idx + 1]))
    return dict(delays=delays, ramp_level=ramp_level, misses=misses, det_p=det_p, det_unc=det_unc, rotor_hit=rotor_hit, err=err,
                pairs=pairs)


def descent_excess(traces, bias: list[float]) -> list[tuple[float, float]]:
    """(time since the descent command started, severity error minus the
    band's level-flight bias) on detected ticks of faulty flights."""
    out = []
    for tr in traces:
        if tr["descent_start"] is None or tr["target"] <= 0:
            continue
        b = bucket(tr["target"])
        tau = tr["t"] - tr["descent_start"]
        sel = (tau >= 0) & (tau <= DESCENT_BINS_S[-1]) & (tr["p"] >= DETECTED_P) & (tr["sev"] > 0)
        out.extend(zip(tau[sel], (tr["sev_est"] - tr["sev"] - bias[b])[sel]))
    return out


def descent_profile(excess) -> dict:
    ex = np.array(excess) if excess else np.zeros((0, 2))
    bins = []
    for lo, hi in zip(DESCENT_BINS_S, DESCENT_BINS_S[1:]):
        v = ex[(ex[:, 0] >= lo) & (ex[:, 0] < hi), 1]
        bins.append(dict(from_s=lo, to_s=hi, n=int(len(v)),
                         median=float(np.median(v)) if len(v) else None,
                         p90=float(np.quantile(v, 0.9)) if len(v) else None))
    return dict(bins=bins)


PROFILE_BIN_S = 0.5
PROFILE_END_S = 10.0


def fit_descent(traces, bias) -> dict:
    """The over-read that follows a commanded descent builds up over about a
    second and fades slowly, so it is modelled by its measured shape: the
    median excess per 0.5 s after the descent command (`profile`), times a
    per-descent scale drawn from `scale_quantiles` (each descent's
    least-squares scale against the profile)."""
    ex = np.array(descent_excess_long(traces, bias))
    edges = np.arange(0.0, PROFILE_END_S + 1e-9, PROFILE_BIN_S)
    profile = []
    for lo, hi in zip(edges, edges[1:]):
        v = ex[(ex[:, 0] >= lo) & (ex[:, 0] < hi), 1]
        profile.append(float(np.median(v)) if len(v) >= 10 else (profile[-1] if profile else 0.0))
    g = np.array(profile)
    scales = []
    for tr in traces:
        if tr["descent_start"] is None or tr["target"] <= 0:
            continue
        tau = tr["t"] - tr["descent_start"]
        sel = (tau >= 0) & (tau < PROFILE_END_S) & (tr["p"] >= DETECTED_P) & (tr["sev"] > 0)
        if sel.sum() >= 5:
            e = tr["sev_est"][sel] - tr["sev"][sel] - bias[bucket(tr["target"])]
            gi = g[np.minimum((tau[sel] / PROFILE_BIN_S).astype(int), len(g) - 1)]
            if np.sum(gi * gi) > 1e-9:
                scales.append(float(np.sum(e * gi) / np.sum(gi * gi)))
    return dict(profile_bin_s=PROFILE_BIN_S, profile=[round(x, 5) for x in profile],
                scale_quantiles=_q(scales), n_descents=len(scales))


def descent_excess_long(traces, bias) -> list[tuple[float, float]]:
    out = []
    for tr in traces:
        if tr["descent_start"] is None or tr["target"] <= 0:
            continue
        tau = tr["t"] - tr["descent_start"]
        sel = (tau >= 0) & (tau < PROFILE_END_S) & (tr["p"] >= DETECTED_P) & (tr["sev"] > 0)
        out.extend(zip(tau[sel], (tr["sev_est"] - tr["sev"] - bias[bucket(tr["target"])])[sel]))
    return out


# ----------------------------------------------------------------- main

def summary_stats(traces, bias) -> dict:
    """What the held-out check compares: delays, misses, false alarms,
    descent excess."""
    h, f = healthy_stats(traces), fault_stats(traces)
    delay_cells = []
    for (b, prof), d in f["delays"].items():
        if d or f["misses"][(b, prof)]:
            delay_cells.append(dict(band=[BUCKETS[b], BUCKETS[b + 1]], profile=prof, n=len(d),
                                    misses=f["misses"][(b, prof)],
                                    median_s=float(np.median(d)) if d else None,
                                    p90_s=float(np.quantile(d, 0.9)) if d else None))
    big = int(np.sum((np.array(h["peaks"]) >= 0.5) & (np.array(h["durations"]) >= 0.3))) if h["peaks"] else 0
    return dict(healthy_hours=float(h["hours"]), false_alarms_per_hour=float(h["n_bursts"] / h["hours"]) if h["hours"] else None,
                big_false_alarms_per_hour=float(big / h["hours"]) if h["hours"] else None,
                delays=delay_cells, descent=descent_profile(descent_excess(traces, bias)))


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--m6", default=str(REPO / "results" / "m6_dataset_v1"))
    args = ap.parse_args(argv)

    import torch
    from ai.detector.dataset import load_episodes, split_by_episode
    from ai.detector.model import file_digest, load_checkpoint, predict_episode
    torch.set_num_threads(2)
    ensemble, meta = load_checkpoint(CHECKPOINT)
    episodes, _ = load_episodes(args.m6)
    split = split_by_episode(episodes, seed=meta["extra"]["split_seed"])
    if split.digest() != meta["split_digest"]:
        raise SystemExit("M6 split does not match the one the detector was trained on")
    ramps = m6_ramp_lengths(args.m6)
    m6_test = [trace_m6(e, predict_episode(ensemble, e.features), ramps[e.key])
               for e in split.select(episodes, "test")]
    live = live_traces(FIT_RUNS)
    fit = m6_test + live
    finished = [r for r in HELDOUT_RUNS if (REPO / "results" / r / "manifest.json").exists() and
                json.loads((REPO / "results" / r / "manifest.json").read_text()).get("end_time_utc")]
    heldout = live_traces(HELDOUT_RUNS) if finished == HELDOUT_RUNS else []   # never a run still flying
    print(f"fit traces: {len(m6_test)} M6 test split + {len(live)} live; held out: {len(heldout)}", flush=True)

    # Only flights the detector never trained on. Replayed on the flights it
    # trained on, it catches ramps faster (median 0.62 s at severity 0.3-0.4)
    # than on its own test split (0.92 s) or live (1.13 s); steps are caught
    # in one or two ticks either way. Healthy behaviour from live flights
    # only: the false alarms that matter (p >= 0.5 for >= 0.3 s) occur at the
    # same rate in replayed and live flights (10.8 vs 10.7 per hour), but the
    # live detector blips for a single tick about twice as often, and the
    # policy is evaluated live.
    h, f = healthy_stats(live), fault_stats(fit)
    nb = len(BUCKETS) - 1
    bias = [float(np.mean(f["err"][b])) if f["err"][b] else 0.0 for b in range(nb)]
    std = [float(np.std(f["err"][b])) if f["err"][b] else 0.02 for b in range(nb)]
    pr = np.array(f["pairs"])
    phi = float(np.corrcoef(pr[:, 0], pr[:, 1])[0, 1]) if len(pr) > 10 else 0.0
    detection = {}
    for prof in ("step", "ramp"):
        detection[prof] = dict(
            miss_rate=[round(f["misses"][(b, prof)] / max(1, f["misses"][(b, prof)] + len(f["delays"][(b, prof)])), 4)
                       for b in range(nb)],
            delay_s_quantiles=[_q(f["delays"][(b, prof)]) or _q(sum((f["delays"][(b, p)] for p in ("step", "ramp")), []))
                               for b in range(nb)],
            n=[len(f["delays"][(b, prof)]) for b in range(nb)])
    for prof in ("ramp",):
        detection[prof]["level_quantiles"] = [
            _q(f["ramp_level"][b]) or _q(sum(f["ramp_level"].values(), [])) for b in range(nb)]
    params = dict(
        detector_sim_version="1",
        fitted_from=dict(m6_run=Path(args.m6).name, m6_part="test split only", live_runs=FIT_RUNS,
                         held_out=HELDOUT_RUNS,
                         detector_checkpoint_digest=file_digest(CHECKPOINT),
                         n_traces=len(fit), healthy_hours=round(float(h["hours"]), 3)),
        tick_s=TICK_S,
        alarm=dict(threshold=float(meta["threshold"]), hold_ticks=5),
        severity_bands=list(BUCKETS),
        background=dict(p_quantiles=_q(h["bg_p"]), severity_quantiles=_q(h["bg_sev"]),
                        uncertainty_quantiles=_q(h["bg_unc"])),
        false_alarm=dict(burst_threshold=BURST_P, rate_per_hour=round(float(h["n_bursts"] / h["hours"]), 3),
                         # every observed burst's (duration s, peak p): drawn together,
                         # since long bursts are also high ones
                         bursts=[[round(float(d), 2), round(float(pk), 4)]
                                 for d, pk in zip(h["durations"], h["peaks"])],
                         severity_quantiles=_q(h["burst_sev"])),
        detection=dict(detected_p=DETECTED_P, p_quantiles=_q(f["det_p"]), uncertainty_quantiles=_q(f["det_unc"]),
                       rotor_accuracy=[round(float(np.mean(f["rotor_hit"][b])), 4) if f["rotor_hit"][b] else 0.9
                                       for b in range(nb)],
                       **detection),
        severity_error=dict(bias=[round(x, 5) for x in bias], std=[round(x, 5) for x in std], ar1=round(phi, 4)),
        descent=fit_descent(fit, bias),
    )
    header = ("# Simulated-detector parameters (M8b task 6), fitted by experiments/fit_detector_sim.py\n"
              "# from the real M7 detector's output -- see that script's docstring for the model and\n"
              "# docs/isaac_env.md for the fit. Regenerate, never hand-edit; a change is a new version.\n")
    OUT.write_text(header + yaml.safe_dump(params, sort_keys=False, width=120))
    print(yaml.safe_dump({"severity_error": params["severity_error"], "descent": params["descent"]}, width=120))
    if not heldout:
        print("no held-out flights yet: fixture not written", flush=True)
        return 0

    held = [dict(severity=tr["target"], profile=tr["profile"],
                 onset_s=float(tr["t"][np.flatnonzero(tr["sev"] > 0)[0]]) if (tr["sev"] > 0).any() else None,
                 ramp_s=ramp_duration(tr) or 0.0,
                 descent_start_s=tr["descent_start"], duration_s=float(tr["t"][-1]))
            for tr in heldout]
    HELDOUT.write_text(json.dumps(dict(fixture_version="1", written_by="experiments/fit_detector_sim.py",
                                       runs=HELDOUT_RUNS, episodes=held,
                                       real=summary_stats(heldout, bias)), indent=1) + "\n")
    print(json.dumps(summary_stats(heldout, bias), indent=1)[:3000])
    return 0


if __name__ == "__main__":
    sys.exit(main())
