"""Loads tests/fixtures/isaac_contract_v1.json, written by the PX4 side
(experiments/write_isaac_fixtures.py)."""
import json
from functools import lru_cache
from pathlib import Path

PATH = Path(__file__).resolve().parent.parent.parent / "tests" / "fixtures" / "isaac_contract_v1.json"


@lru_cache(maxsize=1)
def contract() -> dict:
    return json.loads(PATH.read_text())
