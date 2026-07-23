"""Test configuration that selects the repository's vendored dependencies."""

from pathlib import Path
import sys


ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(ROOT / "vendor"))
