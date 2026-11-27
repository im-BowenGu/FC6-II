"""MuJoCo attitude engine configured per rocket, without a hardcoded model."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass
from typing import Any

from .engine import AttitudeEngine


@dataclass(frozen=True)
class AttitudeConfig:
    model_name: str = "rocket"
    gravity: str = "0 0 0"
    timestep: float = 0.01
    joint_axes: tuple[tuple[float, float, float], ...] = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    capsule_fromto: tuple[float, float, float, float, float, float] = (
        0.0, 0.0, -0.3, 0.0, 0.0, 0.3,
    )
    capsule_size: float = 0.033
    capsule_density: float = 227.0


def _num(value: float) -> str:
    return format(value, "g")


def _vec(values: Any) -> str:
    return " ".join(_num(value) for value in values)


class MujocoAttitudeEngine(AttitudeEngine):
    """MuJoCo rigid-body attitude integration driven by external moments."""

    def __init__(self, config: AttitudeConfig | Mapping[str, Any] | None = None) -> None:
        if config is None or isinstance(config, AttitudeConfig):
            self.config = config or AttitudeConfig()
        else:
            self.config = AttitudeConfig(**config)
        import mujoco
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_string(self.mjcf())
        self.data = mujoco.MjData(self.model)

    @classmethod
    def from_config(cls, config: AttitudeConfig | Mapping[str, Any]) -> MujocoAttitudeEngine:
        return cls(config)

    def mjcf(self) -> str:
        config = self.config
        joints = "\n".join(
            f'      <joint name="{name}" type="hinge" axis="{_vec(axis)}"/>'
            for name, axis in zip(("pitch", "roll", "yaw"), config.joint_axes)
        )
        return (
            f'<mujoco model="{config.model_name}">\n'
            f'  <option gravity="{config.gravity}" timestep="{_num(config.timestep)}"/>\n'
            '  <worldbody>\n'
            '    <body name="rocket" pos="0 0 0">\n'
            f'{joints}\n'
            f'      <geom name="body" type="capsule" fromto="{_vec(config.capsule_fromto)}"\n'
            f'            size="{_num(config.capsule_size)}" density="{_num(config.capsule_density)}"/>\n'
            '    </body>\n'
            '  </worldbody>\n'
            '</mujoco>'
        )

    def reset(self) -> None:
        self.mj.mj_resetData(self.model, self.data)

    def step(self, moments: tuple[float, float, float]) -> tuple[float, float, float, float, float, float]:
        for i, value in enumerate(moments):
            self.data.qfrc_applied[i] = value
        self.mj.mj_step(self.model, self.data)
        return (self.data.qpos[0], self.data.qpos[1], self.data.qpos[2],
                self.data.qvel[0], self.data.qvel[1], self.data.qvel[2])
