"""M6 task 8 (@pytest.mark.sim): independently reproduces
scripts/measure_rotor_fault_thrust.py's measurement live -- this test does
NOT read the committed tests/fixtures/rotor_fault_thrust_curve.json (that
fixture is for M8b, not built yet, to compare its own Isaac-side curve
against), it re-derives the same numbers against a real worker, so this
milestone's own test suite doesn't depend on M8b existing to prove
anything.
"""
import importlib.util
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

# Loaded by file path, not `import scripts.measure_rotor_fault_thrust`:
# /opt/ros/humble's own dist-packages ships a REAL (has __init__.py)
# top-level package also named `scripts`, which Python's import system
# prefers over this project's `scripts/` directory (a namespace-package
# portion only) regardless of sys.path order -- confirmed live, not
# guessed. Sidesteps the name collision entirely rather than turning
# scripts/ into a package project-wide just to work around one test.
_SPEC = importlib.util.spec_from_file_location(
    "aero_measure_rotor_fault_thrust", REPO / "scripts" / "measure_rotor_fault_thrust.py")
_measure_module = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_measure_module)
SEVERITIES = _measure_module.SEVERITIES
measure = _measure_module.measure


def test_measured_thrust_ratio_matches_one_minus_severity(sim_worker_x500_aero):
    results = measure(instance=0)
    assert len(results) == len(SEVERITIES)
    for row in results:
        # Exact, not approximate -- this is deterministic relay arithmetic
        # (velocity_ratio = sqrt(1-s), thrust_ratio = velocity_ratio**2 = 1-s),
        # confirmed to float precision by test_graded_severity_scales_relayed_velocity.
        assert row["derived_thrust_ratio"] == pytest.approx(1.0 - row["severity"], abs=1e-9)
