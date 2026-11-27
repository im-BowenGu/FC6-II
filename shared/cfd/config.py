"""Per-rocket CFD configuration loaded from JSON."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any


def _require(mapping: dict[str, Any], key: str) -> Any:
    if key not in mapping:
        raise ValueError(f"CFD config missing {key!r}")
    return mapping[key]


def _resolve(root: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else (root / path).resolve()


@dataclass(frozen=True)
class SolverConfig:
    nx: int
    ny: int
    nz: int
    u_lbm: float
    tau: float
    arch: str
    steps: int


@dataclass(frozen=True)
class GeometryConfig:
    domain_x_m: float
    body_radius_m: float
    body_nose_m: float
    body_tail_m: float
    fin_hinge_m: float
    fin_chord_m: float
    fin_span_m: float
    fin_thick_m: float
    cg_m: float


@dataclass(frozen=True)
class FlowConfig:
    si_u_mps: float
    si_rho_kgm3: float


@dataclass(frozen=True)
class CalibrationConfig:
    cd0: float
    kd: float
    linear: dict[str, tuple[float, float, float]]


@dataclass(frozen=True)
class StateConfig:
    fin0: float
    fin1: float
    fin2: float
    fin3: float
    fit: bool
    cfd: bool = True

    def __getitem__(self, key: str) -> float | bool:
        return getattr(self, key)


@dataclass(frozen=True)
class PathsConfig:
    sim: Path
    out: Path
    stl: Path
    fields: Path


@dataclass(frozen=True)
class TelemetryConfig:
    csv: Path


@dataclass(frozen=True)
class CfdConfig:
    rocket: str
    solver: SolverConfig
    geometry: GeometryConfig
    flow: FlowConfig
    calibration: CalibrationConfig
    states: dict[str, StateConfig]
    paths: PathsConfig
    telemetry: TelemetryConfig
    source: Path

    @classmethod
    def load(cls, path: str | Path) -> CfdConfig:
        source = Path(path).expanduser().resolve()
        raw = json.loads(source.read_text())
        root = source.parent.parent
        solver_raw = _require(raw, "solver")
        geometry_raw = _require(raw, "geometry")
        flow_raw = _require(raw, "flow")
        calibration_raw = _require(raw, "calibration")
        paths_raw = _require(raw, "paths")
        telemetry_raw = _require(raw, "telemetry")
        linear = {
            key: tuple(float(v) for v in values)
            for key, values in _require(calibration_raw, "linear").items()
        }
        states = {
            name: StateConfig(
                fin0=float(_require(state, "fin0")),
                fin1=float(_require(state, "fin1")),
                fin2=float(_require(state, "fin2")),
                fin3=float(_require(state, "fin3")),
                fit=bool(state.get("fit", True)),
                cfd=bool(state.get("cfd", True)),
            )
            for name, state in _require(raw, "states").items()
        }
        return cls(
            rocket=str(_require(raw, "rocket")),
            solver=SolverConfig(
                nx=int(_require(solver_raw, "nx")),
                ny=int(_require(solver_raw, "ny")),
                nz=int(_require(solver_raw, "nz")),
                u_lbm=float(_require(solver_raw, "u_lbm")),
                tau=float(_require(solver_raw, "tau")),
                arch=str(_require(solver_raw, "arch")),
                steps=int(_require(solver_raw, "steps")),
            ),
            geometry=GeometryConfig(**{
                key: float(_require(geometry_raw, key))
                for key in (
                    "domain_x_m", "body_radius_m", "body_nose_m", "body_tail_m",
                    "fin_hinge_m", "fin_chord_m", "fin_span_m", "fin_thick_m", "cg_m",
                )
            }),
            flow=FlowConfig(
                si_u_mps=float(_require(flow_raw, "si_u_mps")),
                si_rho_kgm3=float(_require(flow_raw, "si_rho_kgm3")),
            ),
            calibration=CalibrationConfig(
                cd0=float(_require(calibration_raw, "cd0")),
                kd=float(_require(calibration_raw, "kd")),
                linear=linear,
            ),
            states=states,
            paths=PathsConfig(
                sim=_resolve(root, _require(paths_raw, "sim")),
                out=_resolve(root, _require(paths_raw, "out")),
                stl=_resolve(root, _require(paths_raw, "stl")),
                fields=_resolve(root, _require(paths_raw, "fields")),
            ),
            telemetry=TelemetryConfig(
                csv=_resolve(root, _require(telemetry_raw, "csv")),
            ),
            source=source,
        )

    @classmethod
    def from_file(cls, path: str | Path) -> CfdConfig:
        return cls.load(path)

    @property
    def rocket_root(self) -> Path:
        return self.source.parent.parent

    @property
    def dx_m(self) -> float:
        return self.geometry.domain_x_m / self.solver.nx

    def angles(self, name: str) -> list[float]:
        state = self.states[name]
        return [state.fin0, state.fin1, state.fin2, state.fin3]

    def state_names(self, fit_only: bool = False) -> list[str]:
        if fit_only:
            return [name for name, state in self.states.items() if state.fit]
        return list(self.states)

    def write_configs(
        self,
        out: str | Path,
        *,
        arch: str | None = None,
        nx: int | None = None,
        ny: int | None = None,
        nz: int | None = None,
        steps: int | None = None,
        u_lbm: float | None = None,
        tau: float | None = None,
    ) -> None:
        out = Path(out).resolve()
        configs_dir = out / "configs"
        configs_dir.mkdir(parents=True, exist_ok=True)
        manifest = {
            name: {"main_fin_deflection_deg": {
                "0": state.fin0, "90": state.fin1,
                "180": state.fin2, "270": state.fin3,
            }}
            for name, state in self.states.items()
        }
        (configs_dir / "manifest.json").write_text(
            json.dumps(manifest, indent=2) + "\n"
        )
        solver = {
            "solver": "taichi-d3q19-lbm",
            "arch": self.solver.arch if arch is None else arch,
            "nx": self.solver.nx if nx is None else nx,
            "ny": self.solver.ny if ny is None else ny,
            "nz": self.solver.nz if nz is None else nz,
            "steps": self.solver.steps if steps is None else steps,
            "u_lbm": self.solver.u_lbm if u_lbm is None else u_lbm,
            "tau": self.solver.tau if tau is None else tau,
            "domain_x_m": self.geometry.domain_x_m,
            "body_radius_m": self.geometry.body_radius_m,
            "fin_hinge_m": self.geometry.fin_hinge_m,
            "fin_chord_m": self.geometry.fin_chord_m,
            "fin_span_m": self.geometry.fin_span_m,
            "cg_m": self.geometry.cg_m,
            "si_u_mps": self.flow.si_u_mps,
            "si_rho_kgm3": self.flow.si_rho_kgm3,
            "calibration": {
                "Cd0": self.calibration.cd0,
                "Kd": self.calibration.kd,
                "linear": self.calibration.linear,
            },
        }
        (configs_dir / "solver.json").write_text(json.dumps(solver, indent=2) + "\n")
