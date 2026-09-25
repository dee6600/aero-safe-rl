"""M9 task 4: the trained recovery policy, flown on the PX4 side.

Loads the file the Isaac side exports (isaac/aero_isaac/train.py export,
`policy.pt`): the policy network's layers, with train_v1's action mapping
already folded into the last one, and the fingerprints of the contract files
it was trained under. The two sides share this file, never code
(CLAUDE.md §0.1).

Loading refuses a file whose action, observation or normalisation
fingerprints differ from this checkout's files. That check is the only thing
standing between "the policy transferred" and "the two sides quietly
disagreed about what input 12 means".

Per decision: flatten_observation (the one PX4-side implementation of the
observation vector), then the layers, then ActionSpec.decode (action_v1's
clip and land threshold). It uses the mean action, with no sampling. The
network sees exactly the PolicyInput the rule-based controller sees, and
returns exactly an Action.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import yaml

from rl.policies.base_policy import (
    REPO, Action, ActionSpec, BasePolicy, ObservationSpec, PolicyInput, flatten_observation, load_action_spec,
    load_observation_spec)

POLICY_FORMAT = "aero_safe_rl_policy_v1"
CHECKED = ("action_v1", "observation_v2", "normalization")
ACTIVATIONS = ("elu", "relu", "tanh")


def contract_fingerprints(spec: ActionSpec, obs_spec: ObservationSpec, obs_path: Path | None = None) -> dict:
    """This checkout's fingerprints of the three files a policy's inputs and
    outputs depend on, computed the way the Isaac side computes them
    (sorted JSON of the parsed file)."""
    from experiments.episode_schema import digest
    obs_raw = yaml.safe_load((obs_path or REPO / "configs" / "rl" / "observation_v2.yaml").read_text())
    return dict(action_v1=spec.digest, observation_v2=obs_spec.digest,
                normalization=digest(yaml.safe_load((REPO / obs_raw["normalization"]).read_text())))


class LearnedPolicy(BasePolicy):

    def __init__(self, path: str | Path, spec: ActionSpec | None = None, obs_spec: ObservationSpec | None = None,
                 *, torch_threads: int = 1):
        import torch
        from ai.detector.model import CheckpointMismatch
        torch.set_num_threads(torch_threads)
        self.path = Path(path)
        self.spec = spec or load_action_spec()
        self.obs_spec = obs_spec or load_observation_spec()
        ck = torch.load(self.path, map_location="cpu", weights_only=True)

        problems = []
        if ck.get("format") != POLICY_FORMAT:
            problems.append(f"format {ck.get('format')!r} != {POLICY_FORMAT!r}")
        if ck.get("obs_dim") != len(self.obs_spec.entries):
            problems.append(f"takes {ck.get('obs_dim')} inputs, observation_v2 has {len(self.obs_spec.entries)}")
        if ck.get("act_dim") != len(self.spec.names):
            problems.append(f"gives {ck.get('act_dim')} outputs, action_v1 has {len(self.spec.names)}")
        if ck.get("activation") not in ACTIVATIONS:
            problems.append(f"unknown activation {ck.get('activation')!r}")
        ours = contract_fingerprints(self.spec, self.obs_spec)
        theirs = ck.get("fingerprints", {})
        for k in CHECKED:
            if theirs.get(k) != ours[k]:
                problems.append(f"{k}: trained under {theirs.get(k)!r}, this checkout has {ours[k]!r}")
        if problems:
            raise CheckpointMismatch(f"{self.path}: " + "; ".join(problems))

        self.fingerprints = dict(theirs)
        self.meta = {k: ck.get(k) for k in ("seed", "update", "code_commit", "created_utc")}
        self._torch = torch
        self._layers = [(layer["weight"].float(), layer["bias"].float()) for layer in ck["layers"]]
        self._act = {"elu": torch.nn.functional.elu, "relu": torch.relu, "tanh": torch.tanh}[ck["activation"]]

    def vector(self, x: np.ndarray) -> np.ndarray:
        """The network's output, (…, 27) -> (…, 3) action_v1 vectors, before
        the clip and land threshold."""
        torch = self._torch
        with torch.no_grad():
            h = torch.as_tensor(np.asarray(x, dtype=np.float32))
            for i, (w, b) in enumerate(self._layers):
                h = h @ w.T + b
                if i < len(self._layers) - 1:
                    h = self._act(h)
        return h.numpy()

    def act(self, obs: PolicyInput) -> Action:
        return self.spec.decode(self.vector(flatten_observation(obs, self.obs_spec)).tolist())

    def describe(self) -> str:
        return json.dumps(dict(path=str(self.path), **self.meta, fingerprints=self.fingerprints))
