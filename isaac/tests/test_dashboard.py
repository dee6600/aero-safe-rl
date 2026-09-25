"""M9 task 3: the training dashboard (isaac/aero_isaac/train.py Dashboard),
on hand-built episode records -- no simulator. Rates per severity band must
be the plain shares of the episodes finished in that update, the metrics
table must keep columns that only appear once episodes finish, and the
pictures must draw."""
import csv
import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import torch

from aero_isaac.contracts import load_train_config
from aero_isaac.records import EPISODE_FIELDS, RECORD_CAPACITY, TRACE_FIELDS
from aero_isaac.train import Dashboard

F = {k: i for i, k in enumerate(EPISODE_FIELDS)}
TRAIN = load_train_config()


class FakeWriter:
    def __init__(self):
        self.scalars, self.figures, self.texts = {}, {}, {}

    def add_scalar(self, tag, value, step):
        self.scalars.setdefault(tag, {})[step] = value

    def add_figure(self, tag, fig, step):
        self.figures[tag] = step

    def add_text(self, tag, text, step):
        self.texts[tag] = text

    def add_custom_scalars(self, layout):
        self.layout = layout

    def flush(self):
        pass


def _env():
    return SimpleNamespace(records=torch.full((RECORD_CAPACITY, len(F)), float("nan")), records_written=0,
                           trace_done=torch.full((4, 20, len(TRACE_FIELDS)), float("nan")),
                           trace=torch.full((4, 20, len(TRACE_FIELDS)), float("nan")))


def _add(env, *rows):
    for r in rows:
        row = torch.full((len(F),), float("nan"))
        for k, v in dict(dict(valid=1.0, false_alarms=0.0, healthy_flight_s=0.0, policy_landed=0.0), **r).items():
            row[F[k]] = v
        env.records[env.records_written % RECORD_CAPACITY] = row
        env.records_written += 1


def _runner():
    dist = SimpleNamespace(std_param=torch.tensor([0.9, 0.8, 0.7]))
    return SimpleNamespace(alg=SimpleNamespace(get_policy=lambda: SimpleNamespace(distribution=dist)))


def test_band_rates_are_shares_of_the_episodes_finished_this_update(tmp_path):
    env, w = _env(), FakeWriter()
    dash = Dashboard(env, w, tmp_path, TRAIN, checkpoint_every=25)
    dash.log(0, _runner(), {"value": 1.0}, 1e-3, 1.0, 1.0)             # nothing finished yet
    _add(env,
         dict(severity=0.0, rotor=-1, outcome=0, touchdown_speed_m_s=0.6, healthy_flight_s=3600.0, false_alarms=10),
         dict(severity=0.0, rotor=-1, outcome=1, policy_landed=1.0, touchdown_speed_m_s=0.7),
         dict(severity=0.4, rotor=2, outcome=2, touchdown_speed_m_s=2.6, detected_s=0.5, reacted_s=0.4),
         dict(severity=0.42, rotor=1, outcome=1, touchdown_speed_m_s=1.2, detected_s=0.3, reacted_s=0.2),
         dict(severity=0.8, rotor=0, outcome=2, touchdown_speed_m_s=6.0),
         dict(valid=0.0, severity=0.0, rotor=-1, outcome=0))                # a reset that never flew
    dash.log(1, _runner(), {"value": 0.5}, 1e-3, 1.0, 1.0)
    at = lambda tag: w.scalars[tag][1]                                      # noqa: E731
    assert at("1 Outcomes/episodes finished") == 5
    assert at("1 Outcomes/crash by band/0.35-0.45") == 0.5
    assert at("1 Outcomes/safe_landing by band/0.35-0.45") == 0.5
    assert at("1 Outcomes/mission_success by band/healthy") == 0.5
    assert at("1 Outcomes/healthy flights landed by the policy") == 0.5
    assert at("1 Outcomes/crash by band/0.5-1.0") == 1.0
    assert at("3 Behaviour/median touchdown speed by band/0.35-0.45") == pytest.approx(1.9)
    assert at("4 Detector/false alarms (p at least 0.5) per healthy hour") == pytest.approx(10.0)
    assert at("5 Training health/action noise/land") == pytest.approx(0.7)
    assert "1 Outcomes/crash by band/0.1-0.3" not in w.scalars           # no episodes there: not plotted
    assert set(w.texts) >= {f"Settings/{Path(TRAIN['path']).name}", f"Settings/{Path(TRAIN['reward']).name}", "Settings/fingerprints"}


def test_metrics_table_keeps_columns_that_appear_later(tmp_path):
    env = _env()
    dash = Dashboard(env, FakeWriter(), tmp_path, TRAIN, checkpoint_every=25)
    dash.log(0, _runner(), {"value": 1.0}, 1e-3, 1.0, 1.0)
    _add(env, dict(severity=0.3, rotor=0, outcome=0, touchdown_speed_m_s=0.5))
    dash.log(1, _runner(), {"value": 1.0}, 1e-3, 1.0, 1.0)
    with (tmp_path / "metrics.csv").open() as f:
        rows = list(csv.DictReader(f))
    assert [r["update"] for r in rows] == ["0", "1"]
    assert math.isnan(float(rows[0]["1 Outcomes/all/mission_success"]))
    assert float(rows[1]["1 Outcomes/all/mission_success"]) == 1.0
    # resuming re-reads the table and replaces rows from the resumed update on
    again = Dashboard(env, FakeWriter(), tmp_path, TRAIN, checkpoint_every=25)
    again.log(1, _runner(), {"value": 2.0}, 1e-3, 1.0, 1.0)
    with (tmp_path / "metrics.csv").open() as f:
        rows = list(csv.DictReader(f))
    assert [r["update"] for r in rows] == ["0", "1"] and float(rows[1]["5 Training health/loss/value"]) == 2.0


def test_pictures_draw_at_checkpoints(tmp_path):
    env, w = _env(), FakeWriter()
    for i in range(4):                     # drone 2 has not finished a flight yet: its live trace is drawn
        tr = env.trace if i == 2 else env.trace_done
        tr[i, :, 0] = torch.arange(20) * 0.1
        tr[i, :, 1:] = torch.rand(20, len(TRACE_FIELDS) - 1)
    dash = Dashboard(env, w, tmp_path, TRAIN, checkpoint_every=2)
    rng = torch.Generator().manual_seed(0)
    for i in range(300):
        sev = float(torch.rand(1, generator=rng)) if i % 5 else 0.0
        _add(env, dict(severity=sev, rotor=0 if sev else -1, outcome=int(sev > 0.42) * 2,
                       touchdown_speed_m_s=0.5 + 6 * max(0.0, sev - 0.35)))
    dash.log(2, _runner(), {"value": 1.0}, 1e-3, 1.0, 1.0)
    assert set(w.figures) == {"Pictures/1 outcome against severity", "Pictures/2 touchdown speed against severity",
                              "Pictures/3 example flights"}
