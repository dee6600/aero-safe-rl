"""Isaac-side tests. Run in the isaacsim environment from the repo root:

    source scripts/activate_isaac.sh
    python -m pytest isaac/tests -m "not isaac and not slow"   # pure PyTorch, seconds
    python -m pytest isaac/tests -m "slow"        # closed-loop controller checks, ~1 min
    python -m pytest isaac/tests -m isaac         # starts Isaac Sim (headless)
"""
import sys
from pathlib import Path

ISAAC_DIR = Path(__file__).resolve().parent.parent
REPO = ISAAC_DIR.parent
if str(ISAAC_DIR) not in sys.path:
    sys.path.insert(0, str(ISAAC_DIR))
