#!/usr/bin/env python3
"""Compatibility wrapper for the shared CFD sweep recorder."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "shared"))
sys.path.insert(0, str(ROOT))

from shared.cfd.record import main

if __name__ == "__main__":
    sys.exit(main())
