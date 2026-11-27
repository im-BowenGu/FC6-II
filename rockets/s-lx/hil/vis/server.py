#!/usr/bin/env python3
"""Tiny live GUI backend for the S-LX HIL harness.

Serves a single-page dashboard plus a JSON telemetry endpoint. It only tails
the CSV/JSONL files that `run.sh` writes -- it never attaches to shared memory
itself, so it is safe to run in a separate shell while a flight is in progress.

    python3 vis/server.py --port 8765

Open http://127.0.0.1:8765 in a browser. No third-party packages are used.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

VIS = Path(__file__).resolve().parent
HIL = VIS.parent
DEFAULT_LOGS = HIL / "logs"

# Fields charted from the firmware (bridge) log. The browser receives one
# downsampled array per field, keyed by time in milliseconds.
BRIDGE_FIELDS = (
    "alt_m", "vv_ms", "pred_apo_m", "fin0", "fin1", "fin2", "fin3",
    "pitch_deg", "roll_deg", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps",
    "state", "tilt_cmd_deg",
)
PLANT_FIELDS = (
    "alt_m", "vz_mps", "pitch_deg", "roll_deg", "az_g",
    "drag_n", "thrust_n", "M_pitch_nm", "q_ratio", "fw_state",
)


def _number(value: str | None) -> float:
    """Parse a finite float, mapping bad/blank values to 0.0 for the charts."""
    if value is None:
        return 0.0
    try:
        result = float(value)
    except (TypeError, ValueError):
        return 0.0
    return result if math.isfinite(result) else 0.0


def read_csv(path: Path, fields: tuple[str, ...], stride: int,
             t_key: str) -> tuple[dict, dict]:
    """Return (latest row, {field: [values]}) with downsampled series."""
    latest: dict = {}
    series: dict[str, list] = {f: [] for f in fields}
    times: list[float] = []
    if not path.exists():
        return latest, {"t": times, **series}

    with path.open(newline="", errors="replace") as fh:
        for index, row in enumerate(csv.DictReader(fh)):
            if not row or not row.get(t_key):
                continue
            latest = {key: _number(row.get(key)) for key in fields}
            latest[t_key] = _number(row.get(t_key))
            if index % stride == 0:
                times.append(latest[t_key])
                for field in fields:
                    series[field].append(latest[field])
    return latest, {"t": times, **series}


def load_state(log_dir: Path, run: str, stride: int) -> dict:
    bridge_latest, bridge_series = read_csv(
        log_dir / "bridge.csv", BRIDGE_FIELDS, stride, "t_ms")
    plant_latest, plant_series = read_csv(
        log_dir / run / "plant.csv", PLANT_FIELDS, stride, "t_s")

    events: list[dict] = []
    events_path = log_dir / run / "events.jsonl"
    if events_path.exists():
        for line in events_path.read_text(errors="replace").splitlines():
            if line.strip():
                try:
                    events.append(json.loads(line))
                except json.JSONDecodeError:
                    pass

    summary: dict = {}
    run_json = log_dir / run / "run.json"
    if run_json.exists():
        try:
            summary = json.loads(run_json.read_text())
        except json.JSONDecodeError:
            pass

    bridge_path = log_dir / "bridge.csv"
    meta = {
        "run": run,
        "bridge_rows": len(bridge_series["t"]),
        "plant_rows": len(plant_series["t"]),
        "bridge_mtime": bridge_path.stat().st_mtime if bridge_path.exists() else 0,
        "server_time": time.time(),
        "stride": stride,
    }
    return {
        "bridge": {"latest": bridge_latest, "series": bridge_series},
        "plant": {"latest": plant_latest, "series": plant_series},
        "events": events,
        "run": summary,
        "meta": meta,
    }


class Handler(BaseHTTPRequestHandler):
    log_dir = DEFAULT_LOGS
    stride = 4

    def _send(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            body = (VIS / "index.html").read_bytes()
            self._send(200, body, "text/html; charset=utf-8")
            return
        if parsed.path == "/api/state":
            query = parse_qs(parsed.query)
            run = query.get("run", ["cfd"])[0]
            # Defend against path traversal from the query string.
            if "/" in run or "\\" in run or run.startswith("."):
                self._send(400, b"bad run", "text/plain")
                return
            try:
                payload = load_state(self.log_dir, run, self.stride)
                body = json.dumps(payload, separators=(",", ":")).encode()
                self._send(200, body, "application/json")
            except OSError as exc:
                self._send(500, str(exc).encode(), "text/plain")
            return
        self._send(404, b"not found", "text/plain")

    def log_message(self, fmt: str, *args) -> None:
        # Keep the terminal quiet while the browser polls at 5 Hz.
        return


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--log-dir", type=Path, default=DEFAULT_LOGS)
    parser.add_argument("--stride", type=int, default=4,
                        help="chart downsample factor (1 = every frame)")
    args = parser.parse_args()

    Handler.log_dir = args.log_dir.resolve()
    Handler.stride = max(1, args.stride)
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"S-LX HIL GUI: http://{args.host}:{args.port} "
          f"(logs: {Handler.log_dir})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
