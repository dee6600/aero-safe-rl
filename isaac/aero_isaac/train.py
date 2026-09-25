"""M9: train the recovery policy in the Isaac environment, export it for the
PX4 side, and draw the training dashboard.

    source scripts/activate_isaac.sh && cd isaac
    python -m aero_isaac.train train --seed 1                  # -> results/m9_train/seed_1/
    python -m aero_isaac.train train --seed 1 --resume         # continue from its latest checkpoint
    python -m aero_isaac.train train --seed 0 --updates 10 --num-envs 2048 --run ../results/m9_smoke
    python -m aero_isaac.train export --run ../results/m9_train/seed_1
    python -m aero_isaac.train curves --run ../results/m9_train/seed_1
    python -m aero_isaac.train fixture                         # tests/fixtures/policy_export_v1.pt

train: rsl_rl proximal policy optimisation with every setting from
the training settings in force, contracts.TRAIN_CONFIG, or --config (budget,
network, action mapping, randomisation).
A checkpoint every `checkpoint_every` updates; the final one is exported to
`policy.pt` when training ends. The final checkpoint is the reported one,
never a "best" one (milestones.md M9).

The dashboard (TensorBoard, in the run directory) is built from the
environment's per-episode records (env.py `EPISODE_FIELDS`), read once per
update. Sections are numbered so they sort in reading order: 1 Outcomes,
2 Reward parts, 3 Behaviour, 4 Detector, 5 Training health; the CUSTOM
SCALARS tab draws the per-band views as one chart each; IMAGES holds, at
every checkpoint, outcome and touchdown speed against severity and four
example flights; TEXT holds the frozen settings. Every scalar also goes to
`metrics.csv` (one row per update), which `curves` draws and the Phase 2
dashboard can read.

export: the final checkpoint's policy network with the action mapping folded
into its last layer, plus the contract fingerprints, as plain tensors, lists
and strings (loads with torch.load(weights_only=True)). The PX4 side
(rl/policies/learned.py) runs the layers, then action_v1's decode, and
refuses a checkpoint whose action, observation or normalisation fingerprints
differ from its own files (CLAUDE.md §0.1).

fixture: a small random-weight policy exported the same way, plus its
outputs on the 12 recorded observation vectors of
tests/fixtures/isaac_contract_v1.json. The PX4 side must reproduce them.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import math
import os
import re
import resource
import subprocess
import sys
from pathlib import Path

import torch
import yaml

from aero_isaac.contracts import CONFIGS, REPO, file_digest_of_yaml, load_train_config

POLICY_FORMAT = "aero_safe_rl_policy_v1"
BANDS = (("healthy", 0.0, 0.0), ("0.1-0.3", 0.1, 0.3), ("0.3-0.35", 0.3, 0.35), ("0.35-0.45", 0.35, 0.45),
         ("0.45-0.5", 0.45, 0.5), ("0.5-1.0", 0.5, 1.01))
OUTCOMES = ("mission_success", "safe_landing", "crash", "incomplete")
PICTURE_WINDOW = 60000          # most recent finished episodes the pictures are drawn from


# ---------------------------------------------------------------- contract fingerprints

def fingerprints(train: dict) -> dict:
    """The checked ones first (the PX4 side refuses a mismatch), then the
    ones recorded for provenance."""
    rl = CONFIGS / "rl"
    obs = yaml.safe_load((rl / "observation_v2.yaml").read_text())
    return dict(
        action_v1=file_digest_of_yaml(rl / "action_v1.yaml"),
        observation_v2=file_digest_of_yaml(rl / "observation_v2.yaml"),
        normalization=file_digest_of_yaml(REPO / obs["normalization"]),
        reward=train["reward_spec"].digest,
        train=train["digest"],
        detector_sim_v1=file_digest_of_yaml(rl / "detector_sim_v1.yaml"),
        outcome_v1=file_digest_of_yaml(rl / "outcome_v1.yaml"))


CHECKED_FINGERPRINTS = ("action_v1", "observation_v2", "normalization")


def _git_commit() -> str:
    try:
        return subprocess.run(["git", "-C", str(REPO), "rev-parse", "HEAD"], capture_output=True, text=True,
                              check=True).stdout.strip()
    except Exception:  # noqa: BLE001
        return "unknown"


# ---------------------------------------------------------------- export

def mlp_layers(actor_state: dict) -> list[tuple[torch.Tensor, torch.Tensor]]:
    """(weight, bias) of every linear layer of an rsl_rl MLPModel's MLP, in order."""
    idx = sorted({int(m.group(1)) for k in actor_state if (m := re.fullmatch(r"mlp\.(\d+)\.weight", k))})
    if not idx:
        raise ValueError("no mlp layers in the actor state")
    return [(actor_state[f"mlp.{i}.weight"].detach().float().cpu(), actor_state[f"mlp.{i}.bias"].detach().float().cpu())
            for i in idx]


def export_policy(actor_state: dict, train: dict, *, seed: int, update: int) -> dict:
    """The exported policy: the actor's mean network with the training settings' action
    mapping (offset + scale * u) folded into its last layer, so its output
    is already an action_v1 vector (before action_v1's own clip and land
    threshold)."""
    layers = mlp_layers(actor_state)
    act_dim = len(train["action_map"]["offset"])
    obs_dim = int(layers[0][0].shape[1])
    if layers[-1][0].shape[0] != act_dim:
        raise ValueError(f"actor outputs {layers[-1][0].shape[0]} values, action_v1 has {act_dim}")
    if obs_dim != 27:
        raise ValueError(f"actor takes {obs_dim} inputs, observation_v2 has 27")
    offset = torch.tensor(train["action_map"]["offset"], dtype=torch.float32)
    scale = torch.tensor(train["action_map"]["scale"], dtype=torch.float32)
    w, b = layers[-1]
    layers[-1] = (scale[:, None] * w, scale * b + offset)
    return dict(format=POLICY_FORMAT, obs_dim=obs_dim, act_dim=act_dim,
                activation=str(train["network"]["activation"]),
                layers=[dict(weight=w.contiguous(), bias=b.contiguous()) for w, b in layers],
                fingerprints=fingerprints(train), checked_fingerprints=list(CHECKED_FINGERPRINTS),
                seed=int(seed), update=int(update), code_commit=_git_commit(),
                created_utc=dt.datetime.now(dt.timezone.utc).isoformat())


def run_exported(policy: dict, obs: torch.Tensor) -> torch.Tensor:
    """What the PX4 side computes: the layers with the activation between
    them. (N, 27) -> (N, 3) action_v1 vectors, before clip and threshold."""
    act = {"elu": torch.nn.functional.elu, "relu": torch.relu, "tanh": torch.tanh}[policy["activation"]]
    x = obs
    for i, layer in enumerate(policy["layers"]):
        x = x @ layer["weight"].T + layer["bias"]
        if i < len(policy["layers"]) - 1:
            x = act(x)
    return x


def latest_checkpoint(run: Path) -> tuple[Path, int] | None:
    found = [(int(m.group(1)), p) for p in run.glob("model_*.pt") if (m := re.fullmatch(r"model_(\d+)\.pt", p.name))]
    if not found:
        return None
    it, path = max(found)
    return path, it


def export_run(run: Path, train: dict, seed: int) -> Path:
    ck = latest_checkpoint(run)
    if ck is None:
        raise FileNotFoundError(f"no model_*.pt in {run}")
    path, it = ck
    state = torch.load(path, map_location="cpu", weights_only=False)["actor_state_dict"]
    policy = export_policy(state, train, seed=seed, update=it)
    out = run / "policy.pt"
    torch.save(policy, out)
    return out


# ---------------------------------------------------------------- fixture for the PX4 side

FIXTURE_PATH = REPO / "tests" / "fixtures" / "policy_export_v1.pt"


def write_fixture() -> Path:
    """A random-weight network of the trained shape (seeded), exported the
    same way as a trained one, with its outputs on the recorded
    observation vectors."""
    train = load_train_config()
    contract = json.loads((REPO / "tests" / "fixtures" / "isaac_contract_v1.json").read_text())
    gen = torch.Generator().manual_seed(20260924)
    dims = [27, *train["network"]["hidden"], 3]
    state = {}
    for i, (a, b) in enumerate(zip(dims, dims[1:])):
        state[f"mlp.{2 * i}.weight"] = torch.randn(b, a, generator=gen) / math.sqrt(a)
        state[f"mlp.{2 * i}.bias"] = torch.randn(b, generator=gen) * 0.1
    policy = export_policy(state, train, seed=0, update=0)
    policy["code_commit"], policy["created_utc"] = "fixture", "fixture"     # regenerates identically
    obs = torch.tensor([c["vector"] for c in contract["observation"]], dtype=torch.float32)
    policy["fixture"] = dict(observations=obs, outputs=run_exported(policy, obs),
                             source="tests/fixtures/isaac_contract_v1.json observation cases",
                             written_by="isaac/aero_isaac/train.py fixture")
    torch.save(policy, FIXTURE_PATH)
    return FIXTURE_PATH


# ---------------------------------------------------------------- dashboard

class Dashboard:
    """Reads the environment's per-episode records once per update and
    writes the numbered TensorBoard sections, the pictures and metrics.csv."""

    def __init__(self, env, writer, run: Path, train: dict, checkpoint_every: int):
        self.env, self.writer, self.run, self.train = env, writer, run, train
        self.every = checkpoint_every
        self.read = 0                        # the environment is fresh: every valid row is this run's
        from aero_isaac.records import EPISODE_FIELDS, RECORD_CAPACITY
        self.fields = {k: i for i, k in enumerate(EPISODE_FIELDS)}
        self.capacity = RECORD_CAPACITY
        self.window: list[torch.Tensor] = []
        self.csv_path = run / "metrics.csv"
        self.metrics: list[dict] = []
        if self.csv_path.exists():           # resuming: keep the earlier updates' rows
            with self.csv_path.open() as f:
                self.metrics = [{k: float(v) if v not in ("", "nan") else math.nan for k, v in r.items()}
                                for r in csv.DictReader(f)]
        self._layout_written = False

    def _new_rows(self) -> torch.Tensor:
        end = self.env.records_written
        n = min(end - self.read, self.capacity)
        self.read = end
        if n <= 0:
            return torch.empty(0, len(self.fields))
        slots = (end - n + torch.arange(n, device=self.env.records.device)) % self.capacity
        rows = self.env.records[slots]
        return rows[rows[:, 0] > 0].cpu()

    def col(self, rows, name):
        return rows[:, self.fields[name]]

    def _band_mask(self, rows, lo, hi):
        sev, faulted = self.col(rows, "severity"), self.col(rows, "rotor") >= 0
        if hi == 0.0:
            return ~faulted
        return faulted & (sev >= lo) & (sev < hi)

    def scalars(self, rows: torch.Tensor) -> dict[str, float]:
        s: dict[str, float] = {}
        if len(rows) == 0:
            return s
        c = lambda k: self.col(rows, k)                                  # noqa: E731
        mean = lambda x: float(x[torch.isfinite(x)].mean()) if torch.isfinite(x).any() else float("nan")  # noqa: E731
        # a true median (torch.median returns the lower middle value of an even count)
        med = lambda x: float(torch.quantile(x[torch.isfinite(x)], 0.5)) if torch.isfinite(x).any() \
            else float("nan")  # noqa: E731
        out = c("outcome")
        s["1 Outcomes/episodes finished"] = float(len(rows))
        for i, o in enumerate(OUTCOMES):
            s[f"1 Outcomes/all/{o}"] = float((out == i).float().mean())
        for name, lo, hi in BANDS:
            m = self._band_mask(rows, lo, hi)
            for i, o in enumerate(OUTCOMES):
                s[f"1 Outcomes/{o} by band/{name}"] = float((out[m] == i).float().mean()) if m.any() else math.nan
            s[f"3 Behaviour/median touchdown speed by band/{name}"] = med(c("touchdown_speed_m_s")[m]) if m.any() \
                else math.nan
        healthy = c("rotor") < 0
        s["1 Outcomes/healthy flights landed by the policy"] = mean(c("policy_landed")[healthy])
        for k in ("mission_success", "safe_landing", "crash", "incomplete", "touchdown_speed", "progress"):
            s[f"2 Reward parts/{k}"] = mean(c(f"r_{k}"))
        s["2 Reward parts/total"] = mean(c("return"))
        s["3 Behaviour/speed command before detection"] = mean(c("speed_cmd_before"))
        s["3 Behaviour/speed command after detection"] = mean(c("speed_cmd_after"))
        s["3 Behaviour/altitude offset before detection (m)"] = mean(c("alt_cmd_before"))
        s["3 Behaviour/altitude offset after detection (m)"] = mean(c("alt_cmd_after"))
        s["3 Behaviour/reaction time after detection (s, median)"] = med(c("reacted_s"))
        s["3 Behaviour/faulty flights landed by the policy"] = mean(c("policy_landed")[~healthy])
        s["3 Behaviour/healthy flights that descended"] = mean((c("min_alt_cmd")[healthy] < -0.5).float())
        s["3 Behaviour/share of mission flown"] = mean(c("mission_progress"))
        s["3 Behaviour/episode length (s)"] = mean(c("duration_s"))
        hs = float(c("healthy_flight_s").sum())
        s["4 Detector/false alarms (p at least 0.5) per healthy hour"] = float(c("false_alarms").sum()) / hs * 3600 if hs > 0 \
            else math.nan
        s["4 Detector/detection delay (s, median)"] = med(c("detected_s"))
        for k in ("delay_scale", "false_alarm_rate_scale", "severity_noise_scale", "severity_bias_shift",
                  "descent_overread_scale"):
            s[f"4 Detector/randomisation drawn/{k}"] = mean(c(f"det_{k}"))
        return s

    def _layout(self):
        """CUSTOM SCALARS: the per-band views as one multi-line chart each."""
        layout = {
            "1 Outcomes": {
                f"{o.replace('_', ' ')} rate by severity band": ["Multiline", [f"1 Outcomes/{o} by band/.*"]]
                for o in OUTCOMES},
            "2 Reward parts": {"every part per episode": ["Multiline", [r"2 Reward parts/.*"]]},
            "3 Behaviour": {
                "commands before and after detection": ["Multiline", [r"3 Behaviour/(speed|altitude).*"]],
                "median touchdown speed by severity band": ["Multiline", [r"3 Behaviour/median touchdown.*"]]},
        }
        self.writer.add_custom_scalars(layout)

    def write_settings(self):
        for path in (REPO / self.train["path"], REPO / self.train["reward"]):
            self.writer.add_text(f"Settings/{path.name}", "```\n" + path.read_text() + "\n```", 0)
        fp = fingerprints(self.train)
        self.writer.add_text("Settings/fingerprints", "\n".join(f"- {k}: `{v}`" for k, v in fp.items()), 0)

    def health(self, runner, loss_dict: dict, learning_rate: float) -> dict[str, float]:
        s = {f"5 Training health/loss/{k}": float(v) for k, v in loss_dict.items()}
        s["5 Training health/learning rate"] = float(learning_rate)
        dist = runner.alg.get_policy().distribution
        std = dist.std_param.detach().cpu() if hasattr(dist, "std_param") else torch.exp(dist.log_std_param.detach()).cpu()
        for name, v in zip(("speed", "altitude", "land"), std.tolist()):
            s[f"5 Training health/action noise/{name}"] = v
        free, total = torch.cuda.mem_get_info()
        s["5 Training health/graphics memory used (GB)"] = (total - free) / 1e9
        s["5 Training health/computer memory peak (GB)"] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024 ** 2
        return s

    def log(self, it: int, runner, loss_dict: dict, learning_rate: float, collect_time: float, learn_time: float):
        if not self._layout_written:
            self._layout()
            self.write_settings()
            self._layout_written = True
        rows = self._new_rows()
        if len(rows):
            self.window.append(rows)
            total = sum(len(r) for r in self.window)
            while total - len(self.window[0]) > PICTURE_WINDOW:
                total -= len(self.window.pop(0))
        s = self.scalars(rows)
        s.update(self.health(runner, loss_dict, learning_rate))
        s["5 Training health/update time (s)"] = collect_time + learn_time
        for k, v in s.items():
            if not math.isnan(v):
                self.writer.add_scalar(k, v, it)
        self._csv(it, s)
        if it % self.every == 0:
            self.pictures(it)

    def _csv(self, it: int, s: dict):
        """One row per update. Rewritten whole each time (a few hundred rows),
        so a column that first appears later -- outcome rates only exist once
        episodes finish -- is never lost."""
        self.metrics = [m for m in self.metrics if int(m["update"]) < it] + [dict(update=it, **s)]
        cols = sorted({k for m in self.metrics for k in m} - {"update"})
        tmp = self.csv_path.with_suffix(".tmp")
        with tmp.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["update", *cols])
            for m in self.metrics:
                w.writerow([int(m["update"]), *(m.get(k, math.nan) for k in cols)])
        tmp.replace(self.csv_path)

    def pictures(self, it: int):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        if self.window:
            rows = torch.cat(self.window)
            self.writer.add_figure("Pictures/1 outcome against severity", outcome_figure(self, rows, plt), it)
            self.writer.add_figure("Pictures/2 touchdown speed against severity", touchdown_figure(self, rows, plt), it)
        fig = example_figure(self.env, self.train, plt)
        if fig is not None:
            self.writer.add_figure("Pictures/3 example flights", fig, it)
        self.writer.flush()


COLOURS = dict(mission_success="#2e7d32", safe_landing="#1565c0", crash="#c62828", incomplete="#9e9e9e")


def _bins(dash, rows):
    sev, faulted = dash.col(rows, "severity"), dash.col(rows, "rotor") >= 0
    edges = torch.arange(0.1, 1.0001, 0.05)
    groups = [("healthy", ~faulted)]
    for lo, hi in zip(edges[:-1].tolist(), edges[1:].tolist()):
        groups.append((f"{lo:.2f}", faulted & (sev >= lo) & (sev < hi + (1e-6 if hi >= 1.0 else 0.0))))
    return groups


def outcome_figure(dash, rows, plt):
    """Stacked outcome shares per 0.05-wide severity bin (healthy on the left)."""
    out = dash.col(rows, "outcome")
    groups = _bins(dash, rows)
    fig, ax = plt.subplots(figsize=(11, 4.2))
    bottom = [0.0] * len(groups)
    for i, o in enumerate(OUTCOMES):
        vals = [float((out[m] == i).float().mean()) if m.any() else 0.0 for _, m in groups]
        ax.bar(range(len(groups)), vals, bottom=bottom, color=COLOURS[o], label=o.replace("_", " "), width=0.85)
        bottom = [b + v for b, v in zip(bottom, vals)]
    ax.set_xticks(range(len(groups)), [g for g, _ in groups], rotation=60, fontsize=8)
    lo = next(i for i, (g, _) in enumerate(groups) if g == "0.35")
    ax.add_patch(plt.Rectangle((lo - 0.5, 0.0), 2.0, 1.0, fill=False, edgecolor="#f9a825", lw=2.5, zorder=5,
                               label="where decisions can matter (0.35-0.45)"))
    ax.set_ylim(0, 1)
    ax.set_xlabel("weak rotor's severity (bin start)")
    ax.set_ylabel("share of episodes")
    ax.set_title(f"How episodes end, by fault severity ({len(rows):,} most recent training episodes)")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.3), fontsize=8, ncol=5, frameon=False)
    fig.tight_layout()
    return fig


def touchdown_figure(dash, rows, plt):
    """Median and quartiles of touchdown speed per severity bin, with the crash line."""
    td = dash.col(rows, "touchdown_speed_m_s")
    groups = _bins(dash, rows)
    fig, ax = plt.subplots(figsize=(11, 4.2))
    for x, (_, m) in enumerate(groups):
        v = td[m & torch.isfinite(td)]
        if len(v) < 5:
            continue
        q = torch.quantile(v, torch.tensor([0.25, 0.5, 0.75]))
        ax.plot([x, x], [float(q[0]), float(q[2])], color="#455a64", lw=3, solid_capstyle="round")
        ax.plot(x, float(q[1]), "o", color="#263238")
    ax.axhline(2.0, color=COLOURS["crash"], ls="--", lw=1.2, label="crash line (outcome_v1: 2.0 m/s)")
    ax.set_xticks(range(len(groups)), [g for g, _ in groups], rotation=60, fontsize=8)
    ax.set_xlabel("weak rotor's severity (bin start)")
    ax.set_ylabel("touchdown speed (m/s)")
    ax.set_title("Touchdown speed by fault severity: median and middle half")
    ax.legend(loc="upper left", fontsize=8)
    fig.tight_layout()
    return fig


def example_figure(env, train, plt):
    """The four example drones' latest finished flights, one row each."""
    from aero_isaac.records import TRACE_FIELDS
    done, live_now = env.trace_done.cpu(), env.trace.cpu()
    if done.shape[0] == 0:
        return None
    # the latest finished flight of each example drone, or its flight so far if none has finished yet
    finished = torch.isfinite(done[:, 0, 0])
    tr = torch.where(finished[:, None, None], done, live_now)
    if not torch.isfinite(tr[:, 0, 0]).any():
        return None
    f = {k: i for i, k in enumerate(TRACE_FIELDS)}
    sevs = train["example_flights"]["severities"]
    onset = float(train["example_flights"]["onset_s"])
    fig, axes = plt.subplots(len(sevs), 3, figsize=(13, 2.4 * len(sevs)), sharex=True, squeeze=False)
    for r, sev in enumerate(sevs):
        live = torch.isfinite(tr[r, :, f["t"]])
        if not live.any():
            continue
        t = tr[r, live, f["t"]]
        g = lambda k: tr[r, live, f[k]]                                   # noqa: E731
        a0, a1, a2 = axes[r]
        a0.plot(t, g("altitude_m"), color="#1565c0", label="altitude (m)")
        a0.plot(t, g("hspeed_m_s"), color="#ef6c00", label="horizontal speed (m/s)")
        a1.plot(t, g("speed_scale"), color="#6a1b9a", label="speed command (0-1)")
        a1.plot(t, g("altitude_offset_m"), color="#00838f", label="altitude offset command (m)")
        a1.plot(t, g("land"), color="#c62828", label="land committed")
        a2.plot(t, g("true_severity"), color="#000000", ls="--", label="true severity")
        a2.plot(t, g("det_severity"), color="#2e7d32", lw=0.8, alpha=0.55, label="detector's severity estimate")
        a2.plot(t, g("det_p_fault"), color="#9e9d24", lw=1.2, label="detector's fault probability")
        for ax in (a0, a1, a2):
            if sev > 0:
                ax.axvline(onset, color="#c62828", lw=0.8, alpha=0.6)
            ax.grid(alpha=0.25)
        a0.set_ylabel(("healthy" if sev == 0 else f"severity {sev}") + ("" if finished[r] else "\n(still flying)"),
                      fontsize=10)
        if r == 0:
            a0.set_title("the flight")
            a1.set_title("what the policy commanded (sampled, as in training)")
            a2.set_title("what the detector said vs the truth")
            for ax in (a0, a1, a2):
                ax.legend(fontsize=7, loc="upper right")
    for ax in axes[-1]:
        ax.set_xlabel("time (s)")
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------- train

def runner_cfg(train: dict, run_name: str) -> dict:
    a, net = train["algorithm"], train["network"]
    return dict(
        num_steps_per_env=int(a["decisions_per_update"]), save_interval=int(a["checkpoint_every"]),
        obs_groups={"actor": ["policy"], "critic": ["policy"]}, logger="tensorboard", run_name=run_name,
        check_for_nan=True,
        actor=dict(class_name="MLPModel", hidden_dims=list(net["hidden"]), activation=net["activation"],
                   obs_normalization=False,
                   distribution_cfg=dict(class_name="GaussianDistribution", init_std=float(net["initial_action_noise"]),
                                         std_type="scalar")),
        critic=dict(class_name="MLPModel", hidden_dims=list(net["hidden"]), activation=net["activation"],
                    obs_normalization=False),
        algorithm=dict(class_name="PPO", num_learning_epochs=int(a["epochs"]), num_mini_batches=int(a["mini_batches"]),
                       clip_param=float(a["clip"]), gamma=float(a["discount"]), lam=float(a["gae_lambda"]),
                       value_loss_coef=float(a["value_loss_coef"]), entropy_coef=float(a["entropy_coef"]),
                       learning_rate=float(a["learning_rate"]), max_grad_norm=float(a["max_grad_norm"]),
                       schedule=a["schedule"], desired_kl=float(a["desired_kl"]), rnd_cfg=None, symmetry_cfg=None))


def train_run(seed: int, run: Path, updates: int | None, num_envs: int | None, resume: bool,
              config: Path | None = None) -> dict:
    from isaaclab.app import AppLauncher
    AppLauncher(headless=True)
    from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper
    from rsl_rl.runners import OnPolicyRunner

    from aero_isaac.env import AeroEnv, training_cfg

    train = load_train_config(config) if config else load_train_config()
    updates = int(updates or train["algorithm"]["updates"])
    run.mkdir(parents=True, exist_ok=True)
    if resume and (run / "run.json").exists():
        recorded = json.loads((run / "run.json").read_text()).get("config")
        if recorded and recorded != train["path"]:
            raise ValueError(f"{run} was trained under {recorded}; resume with --config {recorded}")
    start = latest_checkpoint(run) if resume else None
    if not resume and latest_checkpoint(run) is not None:
        raise FileExistsError(f"{run} already has checkpoints; pass --resume or use a new run directory")
    (run / "run.json").write_text(json.dumps(dict(
        seed=seed, updates=updates, config=train["path"], num_envs=int(num_envs or train["num_envs"]), fingerprints=fingerprints(train),
        code_commit=_git_commit(), resumed_from=str(start[0]) if start else None,
        started_utc=dt.datetime.now(dt.timezone.utc).isoformat()), indent=2))

    snapshot = run / "code"                  # exactly what trained this run, committed or not
    for src in [*(REPO / "isaac" / "aero_isaac").glob("*.py"), *(CONFIGS / "rl").glob("*.yaml")]:
        (snapshot / src.parent.name).mkdir(parents=True, exist_ok=True)
        (snapshot / src.parent.name / src.name).write_bytes(src.read_bytes())

    env = AeroEnv(training_cfg(train, num_envs=num_envs, seed=seed))
    torch.manual_seed(seed)
    vec = RslRlVecEnvWrapper(env)
    runner = OnPolicyRunner(vec, runner_cfg(train, run.name), log_dir=str(run), device=str(env.device))
    runner.add_git_repo_to_log(str(REPO))     # this checkout's uncommitted changes go into <run>/git/ too
    done = 0
    if start is not None:
        runner.load(str(start[0]))
        runner.current_learning_iteration = start[1] + 1
        done = start[1] + 1
        print(f"resumed from {start[0]} (update {start[1]})", flush=True)

    dash_holder: dict = {}
    original_log = runner.logger.log

    def log(**kw):
        original_log(**kw)
        if runner.logger.writer is None:
            return
        if "dash" not in dash_holder:
            dash_holder["dash"] = Dashboard(env, runner.logger.writer, run, train, int(train["algorithm"]["checkpoint_every"]))
        dash_holder["dash"].log(kw["it"], runner, kw["loss_dict"], kw["learning_rate"], kw["collect_time"],
                                kw["learn_time"])

    runner.logger.log = log
    if updates - done > 0:
        runner.learn(num_learning_iterations=updates - done)
    if "dash" in dash_holder:
        dash_holder["dash"].pictures(runner.current_learning_iteration)
    policy = export_run(run, train, seed)
    summary = dict(run=str(run), seed=seed, updates=updates, final_checkpoint=str(latest_checkpoint(run)[0]),
                   policy=str(policy), finished_utc=dt.datetime.now(dt.timezone.utc).isoformat())
    (run / "finished.json").write_text(json.dumps(summary, indent=2))
    return summary


# ---------------------------------------------------------------- curves

CURVE_PANELS = (
    ("How episodes end (all)", [f"1 Outcomes/all/{o}" for o in OUTCOMES]),
    ("Crash rate by severity band", [f"1 Outcomes/crash by band/{b}" for b, _, _ in BANDS]),
    ("Mission success by severity band", [f"1 Outcomes/mission_success by band/{b}" for b, _, _ in BANDS]),
    ("Reward per episode", ["2 Reward parts/total"]),
    ("Commands before / after detection", ["3 Behaviour/speed command before detection",
                                           "3 Behaviour/speed command after detection",
                                           "3 Behaviour/altitude offset after detection (m)"]),
    ("Healthy flights landed by the policy", ["1 Outcomes/healthy flights landed by the policy"]),
)


def curves(run: Path) -> Path:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    with (run / "metrics.csv").open() as f:
        rows = list(csv.DictReader(f))
    x = [int(r["update"]) for r in rows]
    fig, axes = plt.subplots(2, 3, figsize=(15, 7.5))
    for ax, (title, keys) in zip(axes.flat, CURVE_PANELS):
        for k in keys:
            if k in rows[0]:
                ax.plot(x, [float(r[k]) if r[k] not in ("", "nan") else math.nan for r in rows],
                        label=k.split("/")[-1].replace("_", " "))
        ax.set_title(title)
        ax.set_xlabel("update")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    fig.suptitle(f"Training curves (Isaac, training episodes): {run.name}")
    fig.tight_layout()
    out = run / "curves.png"
    fig.savefig(out, dpi=110)
    return out


# ---------------------------------------------------------------- main

def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("train", "export", "curves", "fixture"))
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--run", default=None, help="run directory (default results/m9_train/seed_<seed>)")
    ap.add_argument("--updates", type=int, default=None, help="override for smoke runs only")
    ap.add_argument("--num-envs", type=int, default=None, help="override for smoke runs only")
    ap.add_argument("--resume", action="store_true")
    ap.add_argument("--config", default=None, help="training settings file (default: the one in force, "
                    "contracts.TRAIN_CONFIG)")
    args = ap.parse_args(argv)
    run = Path(args.run) if args.run else REPO / "results" / "m9_train" / f"seed_{args.seed}"
    if args.mode == "fixture":
        print(write_fixture())
        return 0
    if args.mode == "curves":
        print(curves(run))
        return 0
    if args.mode == "export":
        meta = json.loads((run / "run.json").read_text())
        print(export_run(run, load_train_config(REPO / meta.get("config", "configs/rl/train_v2.yaml")), meta["seed"]))
        return 0
    config = Path(args.config) if args.config else None
    if config and not config.is_absolute() and not config.exists():
        config = REPO / config                 # repo-relative, whichever directory this runs from
    summary = train_run(args.seed, run, args.updates, args.num_envs, args.resume, config)
    print(json.dumps(summary), flush=True)
    sys.stdout.flush()
    os._exit(0)   # Isaac Sim's shutdown never returns headless


if __name__ == "__main__":
    main()
