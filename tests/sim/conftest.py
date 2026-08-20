"""Fixtures shared by tests/sim/*.

Every test in this directory is @pytest.mark.sim: it needs one or more real
SITL workers, started via scripts/sim_start.sh (M1b), running headless. These
tests are the milestone gate, not the everyday suite -- run them explicitly
with `pytest tests/sim -q`, after the sim toolchain is sourced
(source scripts/activate.sh).
"""
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent.parent


def pytest_collection_modifyitems(items):
    # pytest calls a conftest's pytest_collection_modifyitems with the FULL
    # session item list, not just items under this directory -- an unfiltered
    # version of this hook marks the entire suite as `sim`, silently
    # deselecting everything under "-m 'not sim'" (confirmed: it did, during
    # M2 development). Only mark items actually collected from this directory.
    here = Path(__file__).parent
    for item in items:
        if here in Path(str(item.fspath)).parents:
            item.add_marker(pytest.mark.sim)


def _sim_start(instance: int, speed: float = 4) -> None:
    r = subprocess.run(
        [str(REPO / 'scripts/sim_start.sh'), '-i', str(instance), '-s', str(speed)],
        capture_output=True, text=True, timeout=180,
    )
    if r.returncode != 0:
        pytest.fail(f"sim_start.sh -i {instance} failed:\n{r.stdout}\n{r.stderr}")


def _sim_stop_all() -> None:
    subprocess.run([str(REPO / 'scripts/sim_stop.sh'), '--all'],
                   capture_output=True, text=True, timeout=120)


@pytest.fixture
def sim_worker():
    """One headless worker at instance 0, torn down afterward regardless of
    test outcome."""
    _sim_stop_all()
    _sim_start(0)
    yield 0
    _sim_stop_all()


@pytest.fixture
def sim_worker_1():
    """One headless worker at instance 1, not 0 -- so a test that only cares
    about telemetry content (not concurrency, which sim_workers_0_1 already
    covers) still exercises the namespaced topic path instance 0 does not
    take by default (docs/parallelism.md §2.2), rather than the easy case."""
    _sim_stop_all()
    _sim_start(1)
    yield 1
    _sim_stop_all()


@pytest.fixture
def sim_workers_0_1():
    """Two concurrent headless workers -- the actual M2 gate. Instance 0 and
    1 deliberately, since PX4 treats instance 0 as a special case internally
    (docs/parallelism.md §2.2) and testing only instance 0 would prove
    nothing about instance 1.
    """
    _sim_stop_all()
    _sim_start(0)
    _sim_start(1)
    yield (0, 1)
    _sim_stop_all()
