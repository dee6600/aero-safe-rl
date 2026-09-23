"""M7 task 3: ai/detector/model.py -- the rotor symmetry the architecture
claims, causality, the checkpoint guard, and the geometry's provenance.
Untrained (randomly initialised) networks: every property here must hold for
any weights, not just trained ones."""
from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest
import torch

from ai.detector import model as dm
from ai.features.feature_extractor import all_feature_names

NAMES = all_feature_names()
I = {n: i for i, n in enumerate(NAMES)}


def _raw(T=30, seed=0) -> torch.Tensor:
    """Random but plausibly-scaled raw features, (1, T, F)."""
    rng = np.random.default_rng(seed)
    x = rng.normal(size=(T, len(NAMES))).astype(np.float32)
    for i in range(4):
        x[:, I[f"motor_{i}_output"]] = 0.6 + 0.1 * rng.normal(size=T)
    return torch.tensor(x)[None]


def _mirror(x: torch.Tensor) -> torch.Tensor:
    """Left-right mirror of the telemetry (y -> -y): roll, yaw and the p/r
    rates flip sign (pseudo-vectors), a_y and v_y flip (vectors), and each
    rotor's command moves to its mirror partner (0 <-> 2, 1 <-> 3)."""
    y = x.clone()
    for n in ("roll_rad", "yaw_rad", "rate_p_rad_s", "rate_r_rad_s", "accel_y_m_s2", "vel_y"):
        y[..., I[n]] = -x[..., I[n]]
    for a, b in ((0, 2), (1, 3), (2, 0), (3, 1)):
        y[..., I[f"motor_{b}_output"]] = x[..., I[f"motor_{a}_output"]]
    return y


MIRROR_PERM = [2, 3, 0, 1]   # output rotor j of the mirrored input == rotor MIRROR_PERM[j]


@pytest.fixture(scope="module")
def norm():
    return dm.load_normalization()


@pytest.mark.parametrize("seed", [0, 1, 2])
def test_mirrored_input_permutes_rotor_outputs(norm, seed):
    torch.manual_seed(seed)
    net = dm.RotorGRU(dm.RotorFrameTransform(norm)).eval()
    x = _raw(seed=seed)
    with torch.no_grad():
        logits, sev, _ = net(x)
        logits_m, sev_m, _ = net(_mirror(x))
    torch.testing.assert_close(logits_m[..., 0], logits[..., 0], atol=1e-5, rtol=1e-5)
    torch.testing.assert_close(logits_m[..., 1:], logits[..., 1:][..., MIRROR_PERM],
                               atol=1e-5, rtol=1e-5)
    torch.testing.assert_close(sev_m, sev[..., MIRROR_PERM], atol=1e-5, rtol=1e-5)


def test_mirror_check_is_not_vacuous(norm):
    # The rotor outputs must actually differ between rotors, or the test
    # above would pass for a model that ignores which rotor is which.
    torch.manual_seed(0)
    net = dm.RotorGRU(dm.RotorFrameTransform(norm)).eval()
    with torch.no_grad():
        logits, _, _ = net(_raw())
    assert logits[..., 1:].std(-1).mean() > 1e-3


def test_ensemble_outputs_are_causal(norm):
    ens = dm.build_ensemble(3, norm).eval()
    x = _raw(T=80)
    y = x.clone()
    y[:, 50:] += 3.0
    with torch.no_grad():
        a, _ = ens(x)
        b, _ = ens(y)
    for k in ("p_fault", "severity", "uncertainty"):
        torch.testing.assert_close(a[k][:, :50], b[k][:, :50])
    assert not torch.allclose(a["p_fault"][:, 50:], b["p_fault"][:, 50:])


def test_ensemble_output_ranges(norm):
    ens = dm.build_ensemble(3, norm).eval()
    with torch.no_grad():
        out, hidden = ens(_raw(T=20))
    assert out["probs"].shape == (1, 20, 5)
    torch.testing.assert_close(out["probs"].sum(-1), torch.ones(1, 20))
    assert ((out["p_fault"] >= 0) & (out["p_fault"] <= 1)).all()
    assert ((out["severity"] >= 0) & (out["severity"] <= 1)).all()
    assert (out["uncertainty"] >= 0).all()
    assert out["rotor"].min() >= 0 and out["rotor"].max() <= 3
    assert len(hidden) == 3


def test_nan_battery_does_not_poison_outputs(norm):
    x = _raw(T=20)
    x[:, 5:8, I["battery_remaining"]] = float("nan")
    with torch.no_grad():
        out, _ = dm.build_ensemble(2, norm).eval()(x)
    assert torch.isfinite(out["p_fault"]).all()


def test_checkpoint_round_trip_and_guards(norm, tmp_path):
    ens = dm.build_ensemble(2, norm, seeds=[5, 6]).eval()
    ens.temperature = 1.7
    path = tmp_path / "d.pt"
    dm.save_checkpoint(path, ens, seeds=[5, 6], split_digest="abc", threshold=0.4)
    loaded, meta = dm.load_checkpoint(path)
    assert meta["threshold"] == 0.4 and meta["split_digest"] == "abc"
    assert loaded.temperature == 1.7
    x = _raw(T=15)
    with torch.no_grad():
        torch.testing.assert_close(loaded(x)[0]["p_fault"], ens(x)[0]["p_fault"])

    ckpt = torch.load(path, weights_only=False)
    ckpt["feature_version"] = "999"
    torch.save(ckpt, tmp_path / "bad_fv.pt")
    with pytest.raises(dm.CheckpointMismatch):
        dm.load_checkpoint(tmp_path / "bad_fv.pt")

    ckpt = torch.load(path, weights_only=False)
    ckpt["norm_digest"] = "0" * 16
    torch.save(ckpt, tmp_path / "bad_norm.pt")
    with pytest.raises(dm.CheckpointMismatch):
        dm.load_checkpoint(tmp_path / "bad_norm.pt")


AIRFRAME = (Path.home() / "projects/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/"
            "airframes/4001_gz_x500")


@pytest.mark.skipif(not AIRFRAME.exists(), reason="PX4 tree not present")
def test_geometry_matches_px4_airframe():
    params = {}
    for line in AIRFRAME.read_text().splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[:2] == ["param", "set-default"] and "CA_ROTOR" in parts[2]:
            params[parts[2]] = float(parts[3])
    for i in range(4):
        assert params[f"CA_ROTOR{i}_PX"] == dm.X500_ROTOR_GEOMETRY["px"][i]
        assert params[f"CA_ROTOR{i}_PY"] == dm.X500_ROTOR_GEOMETRY["py"][i]
        assert params[f"CA_ROTOR{i}_KM"] == dm.X500_ROTOR_GEOMETRY["km"][i]
