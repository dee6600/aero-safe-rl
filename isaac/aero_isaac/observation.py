"""The Isaac side's policy observation (configs/rl/observation_v2.yaml),
built in PyTorch for N drones at once. The PX4 side's builder is
rl/policies/base_policy.py:flatten_observation; isaac/tests/test_observation.py
holds this one to it through the recorded fixture.

Ground truth cannot get in: `assemble` accepts exactly the four observable
blocks the spec names -- feature, detector (the *simulated detector's*
output), mission, previous_action -- and raises on anything else. The true
fault state lives in the environment and is simply never passed here
(CLAUDE.md §1.7, milestones.md M8b "watch out for").
"""
from __future__ import annotations

import torch

from aero_isaac.contracts import ObservationSpec

BLOCKS = ("feature", "detector", "mission", "previous_action")


class ObservationBuilder:

    def __init__(self, spec: ObservationSpec):
        self.spec = spec
        self.size = len(spec.entries)

    def assemble(self, blocks: dict[str, dict[str, torch.Tensor]]) -> torch.Tensor:
        unknown = set(blocks) - set(BLOCKS)
        if unknown:
            raise KeyError(f"not an observation block: {sorted(unknown)} -- only {BLOCKS} may be observed")
        cols = []
        for e in self.spec.entries:
            block, field = e["source"].split(".", 1)
            x = blocks[block][field].to(torch.float32)
            t = e.get("transform")
            if t == "normalize":
                mean, std = self.spec.norm[field]
                x = (x - mean) / std
            elif isinstance(t, dict) and "scale" in t:
                x = x / float(t["scale"])
            elif isinstance(t, dict) and "per" in t:
                x = x / blocks[block][t["per"]].to(torch.float32)
            elif isinstance(t, dict) and "one_hot" in t:
                x = (x.round().long() == int(t["one_hot"])).to(torch.float32)
            elif t is not None:
                raise ValueError(f"unknown transform {t!r}")
            cols.append(x)
        return torch.stack(cols, dim=1)
