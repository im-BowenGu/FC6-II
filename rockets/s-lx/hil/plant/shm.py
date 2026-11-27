"""POSIX shared-memory endpoint mirroring hil/include/slx_hil_shm.h.

Linux shm_open("/slx_hil_sensors") maps to /dev/shm/slx_hil_sensors, so a plain
file-backed mmap is ABI-compatible with the C++ bridge. The layouts are packed
and must stay in sync with the C header.
"""

from __future__ import annotations

import mmap
import struct
from pathlib import Path

SENSOR_FMT = "<Q19dII"          # seq + 19 doubles + flags + reserved
FINS_FMT = "<Q9dII"             # seq + 9 doubles + state + flags
SENSOR_FIELDS = (
    "t_s", "alt_m", "pressure_pa", "vx_mps", "vy_mps", "vz_mps",
    "pitch_deg", "roll_deg", "yaw_deg", "p_rate_dps", "r_rate_dps",
    "y_rate_dps", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps",
    "q_ratio",
)
FINS_FIELDS = ("t_s",) + tuple(f"fin_cmd_deg{i}" for i in range(4)) + \
              tuple(f"fin_act_deg{i}" for i in range(4))

FLAG_RUNNING = 0x1
FLAG_DONE = 0x2
FLAG_FAULT = 0x4


class Segment:
    def __init__(self, name: str, fmt: str) -> None:
        self.fmt = fmt
        self.size = struct.calcsize(fmt)
        path = Path("/dev/shm") / name
        self.file = path.open("w+b")
        self.file.truncate(self.size)
        self.mm = mmap.mmap(self.file.fileno(), self.size)
        self._zero()

    def _zero(self) -> None:
        self.mm[:] = b"\x00" * self.size

    def write(self, values: tuple) -> None:
        seq = struct.unpack_from("<Q", self.mm, 0)[0]
        struct.pack_into("<Q", self.mm, 0, seq + 1)   # writer active
        struct.pack_into(self.fmt, self.mm, 0, *(seq + 1,) + tuple(values))
        struct.pack_into("<Q", self.mm, 0, seq + 2)   # stable

    def read(self) -> dict:
        while True:
            a = struct.unpack_from("<Q", self.mm, 0)[0]
            if a & 1:
                continue
            payload = struct.unpack_from(self.fmt, self.mm, 0)
            b = struct.unpack_from("<Q", self.mm, 0)[0]
            if a == b:
                break
        values = payload[1:]
        names = SENSOR_FIELDS if self.fmt == SENSOR_FMT else FINS_FIELDS
        out = dict(zip(names, values[:len(names)]))
        if self.fmt == FINS_FMT:
            out["state"] = int(values[len(names)])
            out["flags"] = int(values[len(names) + 1])
        else:
            out["flags"] = int(values[len(names)])
        return out

    def set_flags(self, flags: int) -> None:
        cur = self.read()
        seq = struct.unpack_from("<Q", self.mm, 0)[0]
        struct.pack_into("<Q", self.mm, 0, seq + 1)
        names = SENSOR_FIELDS if self.fmt == SENSOR_FMT else FINS_FIELDS
        if self.fmt == SENSOR_FMT:
            vals = [cur[k] for k in names] + [flags, 0]
        else:
            vals = [cur[k] for k in names] + [cur["state"], flags]
        struct.pack_into(self.fmt, self.mm, 0, *(seq + 1,) + tuple(vals))
        struct.pack_into("<Q", self.mm, 0, seq + 2)

    def close(self) -> None:
        self.mm.close()
        self.file.close()
