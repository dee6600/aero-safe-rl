"""Make the repository root importable so tests can `import simulation...`
without the project needing to be pip-installed."""
import sys
from pathlib import Path

ROOT = Path(__file__).parent.resolve()
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
