#!/usr/bin/env python3
"""S-LX HIL physics plant.

Steps a 100 Hz 6-DOF-lite rocket plant, publishes BMP280/ICM20689-equivalent
sensor values into POSIX shared memory, and reads fin commands published by
the Verilated peripheral + real STM32 guidance core. Two backends:

  cfd     - OpenRocket F32 plant + recorded static fin lookup (validated surrogate)
  mujoco  - MuJoCo integrates the attitude joints with the same CFD moments

This is the *plant*, not an avionics port: the guidance loop is the actual C
core in the C++ bridge.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
HIL = HERE.parent
SIM = HIL.parent / "sim"

spec = importlib.util.spec_from_file_location("slxsim", SIM / "run_dynamic_sim.py")
slxsim = importlib.util.module_from_spec(spec)
sys.modules["slxsim"] = slxsim
spec.loader.exec_module(slxsim)

from shm import FINS_FMT, SENSOR_FMT, FLAG_DONE, FLAG_RUNNING, Segment  # noqa: E402

DT = slxsim.DT
G = slxsim.G
SEA_LEVEL_PA = 101325.0


class MujocoAttitude:
    """MuJoCo rigid-body attitude integration driven by CFD moments."""

    XML = """<mujoco model="s-lx">
      <option gravity="0 0 0" timestep="0.01"/>
      <worldbody>
        <body name="rocket" pos="0 0 0">
          <joint name="pitch" type="hinge" axis="1 0 0"/>
          <joint name="roll" type="hinge" axis="0 1 0"/>
          <joint name="yaw" type="hinge" axis="0 0 1"/>
          <geom name="body" type="capsule" fromto="0 0 -0.3 0 0 0.3"
                size="0.033" density="227"/>
        </body>
      </worldbody>
    </mujoco>"""

    def __init__(self) -> None:
        import mujoco  # imported lazily so the cfd backend stays dependency-free
        self.mj = mujoco
        self.model = mujoco.MjModel.from_xml_string(self.XML)
        self.data = mujoco.MjData(self.model)

    def step(self, moments: tuple[float, float, float]) -> tuple[float, float, float, float, float, float]:
        for i, value in enumerate(moments):
            self.data.qfrc_applied[i] = value
        self.mj.mj_step(self.model, self.data)
        return (self.data.qpos[0], self.data.qpos[1], self.data.qpos[2],
                self.data.qvel[0], self.data.qvel[1], self.data.qvel[2])


def plant_step(state: dict, cmd: list[float], or_data: dict, aero: dict,
               backend, wind_mps: float) -> dict:
    t = state["t"]
    mass = max(0.35, or_data["Mass"].at(t))
    thrust = or_data["Thrust"].at(t) if t < 6.0 else 0.0
    D0 = or_data["Drag_force"].at(t)
    rho = max(0.5, or_data["Air_density"].at(t))
    speed = math.sqrt(sum(v * v for v in (state["vx"], state["vy"], state["vz"])))

    if state["tilt_deg"] and not state["tilt_applied"] and \
            t >= slxsim.RAIL_CLEARANCE_MS / 1000.0:
        state["p"] = math.radians(state["tilt_deg"])
        state["tilt_applied"] = True

    p, r, psi = state["p"], state["r"], state["psi"]
    p_rate, r_rate, psi_rate = state["p_rate"], state["r_rate"], state["psi_rate"]

    bx = (math.sin(p), math.sin(r), math.cos(p) * math.cos(r))
    bn = math.sqrt(sum(c * c for c in bx)) or 1.0
    bx = tuple(c / bn for c in bx)
    by = (math.cos(p), 0.0, -math.sin(p))
    bz = (-math.sin(p) * math.sin(r), math.cos(r), -math.cos(p) * math.sin(r))

    # Servo lag, same first-order model as the recorded-motion surrogate.
    fin = state["fin_act"]
    for i in range(4):
        fin[i] += (cmd[i] - fin[i]) * (DT / slxsim.SERVO_TAU_S)
    pitch_modal = (fin[0] - fin[2]) / 2.0
    yaw_modal = (fin[1] - fin[3]) / 2.0
    roll_modal = sum(fin) / 4.0

    q_ratio = (0.5 * rho * speed * speed) / slxsim.Q_REF
    sin2 = sum(math.sin(math.radians(d)) ** 2 for d in fin) / \
        math.sin(math.radians(slxsim.FIN_REF_DEG)) ** 2
    dCd = aero["kd"] * sin2
    drag = (0.5 * rho * speed * speed * slxsim.CDA_PARA if state["recovered"]
            else D0 * (aero["Cd0"] + dCd) / aero["Cd0"])
    drag_vec = tuple(-drag * c / speed for c in
                     (state["vx"], state["vy"], state["vz"])) if speed > 0.05 else (0.0, 0.0, 0.0)

    lin = aero["linear"]

    def modal_force(key: str) -> float:
        return (lin[key][0] * pitch_modal + lin[key][1] * yaw_modal +
                lin[key][2] * roll_modal) * q_ratio

    Fy_body, Fz_body = modal_force("Fy"), modal_force("Fz")
    Tp, Ttilt_n, Troll = modal_force("Ty"), modal_force("Tz"), modal_force("Tx")
    lat_vec = (Fy_body * by[0] + Fz_body * bz[0],
               Fy_body * by[1] + Fz_body * bz[1],
               Fy_body * by[2] + Fz_body * bz[2])
    thrust_vec = tuple(thrust * c for c in bx)
    ax = (thrust_vec[0] + drag_vec[0] + lat_vec[0]) / mass
    ay = (thrust_vec[1] + drag_vec[1] + lat_vec[1]) / mass
    az = (thrust_vec[2] + drag_vec[2] + lat_vec[2]) / mass - G
    if wind_mps:
        ax += (drag / mass) * math.copysign(min(1.0, abs(wind_mps) / max(speed, 1.0)), wind_mps)

    alpha_p = p - math.atan2(state["vx"], state["vz"]) if state["vz"] > 1.0 else p
    alpha_r = r - math.atan2(state["vy"], state["vz"]) if state["vz"] > 1.0 else r
    k_static = 2.40 * q_ratio
    c_damp = 0.24 * q_ratio
    c_roll = 0.10 * q_ratio
    M_p = Tp - k_static * alpha_p - c_damp * p_rate
    M_r = Ttilt_n - k_static * alpha_r - c_damp * r_rate
    M_roll = Troll - c_roll * psi_rate

    if t < slxsim.RAIL_CLEARANCE_MS / 1000.0:
        p = r = psi = p_rate = r_rate = psi_rate = 0.0
        state["vx"] = state["vy"] = 0.0
        state["vz"] = max(state["vz"], 0.0)
        M_p = M_r = M_roll = 0.0
    elif backend is not None:
        p, r, psi, p_rate, r_rate, psi_rate = backend.step((M_p, M_r, M_roll))
    else:
        I_pitch = I_yaw = 0.038
        I_roll = 0.00048
        p_rate += M_p / I_pitch * DT
        r_rate += M_r / I_yaw * DT
        psi_rate = (psi_rate + M_roll / I_roll * DT) / (1.0 + c_roll / I_roll * DT)
        p += p_rate * DT
        r += r_rate * DT
        psi += psi_rate * DT

    state["vx"] += ax * DT
    state["vy"] += ay * DT
    state["vz"] += az * DT
    state["z"] += state["vz"] * DT
    state["x"] += state["vx"] * DT
    state["y"] += state["vy"] * DT
    state["p"], state["r"], state["psi"] = p, r, psi
    state["p_rate"], state["r_rate"], state["psi_rate"] = p_rate, r_rate, psi_rate
    state["t"] += DT

    if not state["recovered"] and state["vz"] < 0.0 and state["z"] > 20.0:
        state["recovered"] = True
    state["max_z"] = max(state["max_z"], state["z"])

    # IMU convention used by slx_guidance_tick: specific force with gravity
    # projected out, including the 1 g rest offset. Matches run_dynamic_sim.py.
    f_up = (state["vz"] * 0.0 + az + G) / G
    accel_body = (-f_up * math.sin(p), f_up * math.sin(r), f_up * math.cos(p) * math.cos(r))
    pressure = SEA_LEVEL_PA * max(0.05, 1.0 - state["z"] / 44330.0) ** 5.255
    return {
        "t_s": state["t"], "alt_m": state["z"], "pressure_pa": pressure,
        "vx_mps": state["vx"], "vy_mps": state["vy"], "vz_mps": state["vz"],
        "pitch_deg": math.degrees(p), "roll_deg": math.degrees(r), "yaw_deg": math.degrees(psi),
        "p_rate_dps": math.degrees(p_rate), "r_rate_dps": math.degrees(r_rate),
        "y_rate_dps": math.degrees(psi_rate),
        "ax_g": accel_body[0], "ay_g": accel_body[1], "az_g": accel_body[2],
        "gx_dps": math.degrees(p_rate), "gy_dps": math.degrees(r_rate),
        "gz_dps": math.degrees(psi_rate), "q_ratio": q_ratio,
        "fin_act_deg": list(fin), "drag_n": drag, "mass_kg": mass,
        "thrust_n": thrust, "dCd_cfd": dCd, "M_pitch_nm": M_p,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", choices=("cfd", "mujoco"), default="cfd")
    ap.add_argument("--tilt", type=float, default=0.0, help="rail-exit tilt [deg]")
    ap.add_argument("--wind", type=float, default=0.0, help="steady wind [m/s]")
    ap.add_argument("--max-t", type=float, default=40.0)
    ap.add_argument("--out", default=str(HIL / "logs"))
    args = ap.parse_args()

    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    or_data = slxsim.read_or_flight(slxsim.OR_CSV)
    aero = slxsim.read_cfd_lookup()
    sensors = Segment("slx_hil_sensors", SENSOR_FMT)
    fins = Segment("slx_hil_fins", FINS_FMT)

    backend = MujocoAttitude() if args.backend == "mujoco" else None
    state = {
        "t": 0.0, "z": 0.0, "x": 0.0, "y": 0.0,
        "vx": 0.0, "vy": 0.0, "vz": 0.0,
        "p": 0.0, "r": 0.0, "psi": 0.0,
        "p_rate": 0.0, "r_rate": 0.0, "psi_rate": 0.0,
        "fin_act": [0.0] * 4, "recovered": False, "max_z": 0.0,
        "tilt_deg": args.tilt, "tilt_applied": False,
    }
    events: list[dict] = []
    last_state = None
    rows: list[dict] = []

    while state["t"] <= args.max_t:
        fin_frame = fins.read()
        cmd = [fin_frame[f"fin_cmd_deg{i}"] for i in range(4)] \
            if fin_frame["flags"] else [0.0] * 4
        published = plant_step(state, cmd, or_data, aero, backend, args.wind)
        sensors.write(tuple(published[k] for k in (
            "t_s", "alt_m", "pressure_pa", "vx_mps", "vy_mps", "vz_mps",
            "pitch_deg", "roll_deg", "yaw_deg", "p_rate_dps", "r_rate_dps",
            "y_rate_dps", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps",
            "gz_dps", "q_ratio")) + (FLAG_RUNNING, 0))
        # Lockstep: the bridge echoes the sensor timestamp in the fins segment
        # after the real C guidance core has consumed the frame. Waiting for
        # that echo guarantees every simulated tick is processed even when the
        # plant and bridge share a single CPU (wall-clock pacing let the faster
        # process starve the other and truncate the flight).
        ack_deadline = time.monotonic() + 5.0
        while time.monotonic() < ack_deadline:
            f = fins.read()
            if f["t_s"] >= published["t_s"] - 1e-9 or f["flags"] & FLAG_DONE:
                break
            time.sleep(0.0005)
        if f["state"] != last_state:
            events.append({"t_s": round(state["t"], 3), "from": last_state,
                           "to": f["state"], "alt_m": round(state["z"], 2)})
            last_state = f["state"]
        rows.append({**published, "fin_cmd_deg": [f[f"fin_cmd_deg{i}"] for i in range(4)],
                     "fw_state": f["state"]})
        if f["flags"] & FLAG_DONE:
            break

    # The plant stops publishing at max_t, but the bridge may still be behind.
    # The bridge echoes the sensor timestamp in the fins segment, so wait until
    # it has consumed the final frame before raising DONE; otherwise it would
    # exit early and miss part of the flight.
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        ack = fins.read()
        if ack["flags"] & FLAG_DONE or ack["t_s"] >= state["t"] - 1e-9:
            break
        time.sleep(DT)
    sensors.set_flags(FLAG_RUNNING | FLAG_DONE)
    with (out / "plant.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    (out / "events.jsonl").write_text("".join(json.dumps(e) + "\n" for e in events))
    (out / "run.json").write_text(json.dumps({
        "backend": args.backend, "tilt_deg": args.tilt, "wind_mps": args.wind,
        "apogee_m": round(state["max_z"], 2), "duration_s": round(state["t"], 2),
        "aero": {"kd": aero["kd"], "cd0": aero["Cd0"], "rms": aero["drag_rms"]},
        "note": "guidance is the real C core in cpp/bridge.cpp, not a Python port",
    }, indent=2) + "\n")
    print(f"plant: backend={args.backend} apogee={state['max_z']:.2f} m "
          f"duration={state['t']:.2f} s logs={out}")
    sensors.close()
    fins.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
