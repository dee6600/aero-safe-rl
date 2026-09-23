"""CLAUDE.md §0.1: nothing under isaac/ imports the PX4 side. The two sides
exchange files only. A static scan, so it holds without either environment
installed."""
import re
from pathlib import Path

ISAAC = Path(__file__).resolve().parent.parent
FORBIDDEN = ("rclpy", "px4_msgs", "aero_bridge", "rl", "experiments", "ai", "simulation")
PATTERN = re.compile(r"^\s*(?:from|import)\s+(" + "|".join(FORBIDDEN) + r")(?:\.|\s|$)", re.M)


def test_no_import_crosses_the_environment_boundary():
    offenders = []
    for path in ISAAC.rglob("*.py"):
        for m in PATTERN.finditer(path.read_text()):
            offenders.append(f"{path.relative_to(ISAAC)}: {m.group(0).strip()}")
    assert not offenders, offenders
