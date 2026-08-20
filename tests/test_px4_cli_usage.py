"""Guard against PX4 shell-client argument-order bugs (found in M1b).

PX4's posix shell client only accepts ``--instance N`` as ``argv[1]``:

    platforms/posix/src/px4/common/main.cpp:154
        if (argc >= 3 && strcmp(argv[1], "--instance") == 0) {

Put it anywhere else and it is **silently ignored**: the command is delivered to
instance 0, and ``--instance``/``N`` are additionally passed through as stray
arguments to the command itself. Nothing errors. The client prints instance 0's
reply, so a status check "passes" while the vehicle you meant to command never
moves.

That bug shipped in two places at once (``scripts/fly_demo.py`` and, briefly,
``scripts/sim_start.sh``) and cost a debugging cycle, because the symptom --
"instance 1 arms but never takes off" -- looks like a flight problem rather than
a CLI problem.

It is a purely static property of the source, so it can be checked in
milliseconds with no simulator. This test is the cheap permanent version of that
debugging cycle.
"""

import re
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent

#: Where PX4 shell-client calls could plausibly live.
SEARCH_DIRS = ("scripts", "simulation", "experiments", "ros2_ws/src/aero_bridge", "rl", "ai")

SEARCH_SUFFIXES = (".py", ".sh")

PX4_CLIENTS = ("px4-param",)
# px4-commander had the same argv[1] trap, but its only call site was
# scripts/fly_demo.py (removed -- superseded by the ROS 2 flight path). If a
# px4-commander call is ever added back, add it here so it's covered again.

#: Sub-commands that must never appear before --instance.
SUBCOMMANDS = (
    "set", "show", "status", "arm", "disarm", "takeoff", "land",
    "mode", "compare", "save", "load", "reset",
)

# How far past the binary name to look for the instance flag.
WINDOW = 160


def _source_files():
    for rel in SEARCH_DIRS:
        base = REPO / rel
        if not base.is_dir():
            continue
        for path in base.rglob("*"):
            if path.suffix in SEARCH_SUFFIXES and "__pycache__" not in path.parts:
                yield path


def _strip_comments(text: str) -> str:
    """Drop whole-line comments.

    Both languages here use `#`, and the docs deliberately quote the *wrong*
    form as an example -- so scanning comments would flag the very warning that
    exists to prevent the bug.
    """
    return "\n".join(
        line for line in text.splitlines() if not line.lstrip().startswith("#")
    )


def _display(path: Path) -> str:
    """Repo-relative path when possible; the scanner is also run on tmp files."""
    try:
        return str(path.relative_to(REPO))
    except ValueError:
        return str(path)


def _violations(path: Path) -> list[str]:
    text = _strip_comments(path.read_text(errors="replace"))
    found = []
    for client in PX4_CLIENTS:
        for match in re.finditer(re.escape(client), text):
            window = text[match.end(): match.end() + WINDOW]
            # Only judge calls that use --instance at all. A call that omits it
            # entirely targets instance 0, which is legitimate for single-worker
            # tooling and is not what this guard is about.
            flag = window.find("--instance")
            if flag == -1:
                continue
            before = window[:flag]
            for sub in SUBCOMMANDS:
                if re.search(rf"\b{sub}\b", before):
                    line = text[: match.start()].count("\n") + 1
                    found.append(
                        f"{_display(path)}:{line}: '{sub}' appears before "
                        f"--instance in a {client} call; --instance must be argv[1] "
                        f"or it is silently ignored and hits instance 0"
                    )
                    break
    return found


def test_instance_flag_comes_first():
    """--instance must immediately follow the px4 client binary, everywhere."""
    violations = [v for path in _source_files() for v in _violations(path)]
    assert not violations, "PX4 shell client argument order is wrong:\n  " + "\n  ".join(
        violations
    )


def test_scanner_would_catch_the_original_bug(tmp_path):
    """The guard must actually detect the form that shipped.

    A static check that cannot fail is worse than no check, because it reads as
    coverage. This pins the scanner against the exact broken line from
    scripts/fly_demo.py as it was before the fix.
    """
    bad = tmp_path / "bad.sh"
    bad.write_text('"$PX4_BUILD/bin/px4-param" set NAV_DLL_ACT 0 --instance "$INSTANCE"\n')
    assert _violations(bad), "scanner failed to flag the known-bad argument order"


def test_scanner_accepts_the_fixed_form(tmp_path):
    good = tmp_path / "good.sh"
    good.write_text('"$PX4_BUILD/bin/px4-param" --instance "$INSTANCE" set NAV_DLL_ACT 0\n')
    assert not _violations(good)


def test_scanner_ignores_comments(tmp_path):
    """The wrong form is quoted in comments as a warning; that must not trip."""
    commented = tmp_path / "doc.sh"
    commented.write_text(
        "# the obvious `px4-param set NAV_DLL_ACT 0 --instance 1` targets the\n"
        "# WRONG vehicle and reports success\n"
    )
    assert not _violations(commented)


@pytest.mark.parametrize("client", PX4_CLIENTS)
def test_repo_uses_known_clients(client):
    """Sanity: the scan is actually looking at something.

    If every call site disappears (or moves outside SEARCH_DIRS) this test fails
    rather than letting test_instance_flag_comes_first pass vacuously.
    """
    hits = [p for p in _source_files() if client in _strip_comments(p.read_text(errors="replace"))]
    assert hits, f"no {client} call sites found -- has the code moved out of SEARCH_DIRS?"
