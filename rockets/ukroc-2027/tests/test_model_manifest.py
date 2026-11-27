#!/usr/bin/env python3
"""Check that model.yaml stays consistent with the profile and its assets.

No third-party YAML parser is used: this is a small guard against a manifest
silently drifting away from the compiled constants or losing a referenced
model/CFD input.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    manifest = (ROOT / "model.yaml").read_text()
    header = (ROOT / "include" / "ukroc_profile.h").read_text()
    errors: list[str] = []

    def require(text: str, token: str, where: str) -> None:
        if token not in text:
            errors.append(f"{where}: missing {token!r}")

    for token in ("apogee_m: 229.0", "apogee_tolerance_m: 5.0",
                  "duration_min_s: 34.0", "duration_max_s: 37.0",
                  "337c2384aa7c96644eed28153bd00ac8"):
        require(manifest, token, "model.yaml")

    match = re.search(r"#define\s+UKROC_TARGET_APOGEE_M\s+([0-9.]+)f", header)
    if not match or float(match.group(1)) != 229.0:
        errors.append("ukroc_profile.h: UKROC_TARGET_APOGEE_M must be 229.0f")

    assets = [
        "cad/fin_plate.stl",
        "cad/models/GOOP1-UKROC.ork",
        "cad/models/GOOP1-UKROC-ESP32C3.ork",
        "cad/models/GOOP1-UKROC-STM32F103.ork",
        "cfd/fin_states.yaml",
        "sim/data/reference_flight.csv",
        "sim/data/cfd/fluidx3d/configs/manifest.json",
        "sim/data/cfd/fluidx3d/logs/neutral_force.csv",
        "sim/data/cfd/fluidx3d/logs/xmix_p15_force.csv",
    ]
    for asset in assets:
        if not (ROOT / asset).is_file():
            errors.append(f"missing asset: {asset}")

    if errors:
        print("model.yaml check FAILED:\n  " + "\n  ".join(errors))
        return 1
    print("model.yaml check PASSED")
    return 0


if __name__ == "__main__":
    sys.exit(main())
