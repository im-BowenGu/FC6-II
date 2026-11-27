#!/usr/bin/env python3
"""S-LX HIL physics plant.

Steps a 100 Hz 6-DOF-lite rocket plant, publishes BMP280/ICM20689-equivalent
sensor values into POSIX shared memory, and reads fin commands published by
the Verilated peripheral + real STM32 guidance core. Two backends:

  cfd     - OpenRocket F32 plant + recorded static fin lookup (validated surrogate)
  mujoco  - MuJoCo integrates the attitude joints with the same CFD moments

Both backends use the same transient actuator model: a 10 ms command transport
delay, a 50 Hz PCA9685 latch, 300 deg/s slew limit and 35 ms first-order servo
lag. Each latched move records its from/to angles and completion percentage,
and the recorded CFD lookup is evaluated with the actual fin angles through a
20 ms reduced-order response lag.

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
ROCKET = SIM.parent
PHYSICS_CONFIG = ROCKET / "config" / "physics.json"
SHARED = Path(__file__).resolve().parents[4] / "shared"
if str(SHARED) not in sys.path:
    sys.path.insert(0, str(SHARED))

from physics.mujoco_engine import AttitudeConfig, MujocoAttitudeEngine  # noqa: E402

spec = importlib.util.spec_from_file_location("slxsim", SIM / "run_dynamic_sim.py")
slxsim = importlib.util.module_from_spec(spec)
sys.modules["slxsim"] = slxsim
spec.loader.exec_module(slxsim)

from shm import FINS_FMT, SENSOR_FMT, FLAG_DONE, FLAG_RUNNING, Segment  # noqa: E402

DT = slxsim.DT
G = slxsim.G
SEA_LEVEL_PA = 101325.0
SERVO_TAU_S = slxsim.SERVO_TAU_S
SERVO_PWM_HZ = slxsim.SERVO_PWM_HZ
SERVO_RATE_DPS = slxsim.SERVO_RATE_DPS
SERVO_PWM_TICKS = slxsim.SERVO_PWM_TICKS
SERVO_LATENCY_MS = slxsim.SERVO_LATENCY_MS
SERVO_LATENCY_TICKS = slxsim.SERVO_LATENCY_TICKS
AERO_RESPONSE_TAU_S = slxsim.AERO_RESPONSE_TAU_S


def _attitude_config() -> AttitudeConfig:
    if PHYSICS_CONFIG.exists():
        return AttitudeConfig(**json.loads(PHYSICS_CONFIG.read_text()))
    return AttitudeConfig(model_name=ROCKET.name)


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

    # Same transient actuator model as the 100 Hz surrogate: a transport delay
    # before the PCA9685 input, a 50 Hz latch, finite slew and a first-order
    # mechanical lag. Each latched target change records where the move began
    # so the GUI can show a real completion percentage.
    for i in range(4):
        pending = state["pending_cmds"][i]
        while pending and pending[0][0] <= state["pwm_tick"]:
            state["fin_delayed"][i] = pending.pop(0)[1]
        pending.append((state["pwm_tick"] + SERVO_LATENCY_TICKS, cmd[i]))
    if state["pwm_tick"] % SERVO_PWM_TICKS == 0 and \
            any(abs(state["fin_delayed"][i] - state["pwm_cmd"][i]) > 1e-9
                for i in range(4)):
        for i in range(4):
            state["fin_move_from"][i] = state["fin_act"][i]
            state["fin_move_to"][i] = state["fin_delayed"][i]
            state["fin_move_start_s"][i] = t
        state["pwm_cmd"][:] = state["fin_delayed"]
    state["pwm_tick"] += 1
    fin = state["fin_act"]
    for i in range(4):
        target = fin[i] + (state["pwm_cmd"][i] - fin[i]) * (DT / SERVO_TAU_S)
        delta = max(-SERVO_RATE_DPS * DT,
                    min(SERVO_RATE_DPS * DT, target - fin[i]))
        fin[i] += delta
        state["fin_rate_dps"][i] = delta / DT
        state["fin_progress_pct"][i] = slxsim.move_progress(
            fin[i], state["fin_move_from"][i], state["fin_move_to"][i])
    pitch_modal = (fin[0] - fin[2]) / 2.0
    yaw_modal = (fin[1] - fin[3]) / 2.0
    roll_modal = sum(fin) / 4.0

    q_ratio = (0.5 * rho * speed * speed) / slxsim.Q_REF
    sin2 = sum(math.sin(math.radians(d)) ** 2 for d in fin) / \
        math.sin(math.radians(slxsim.FIN_REF_DEG)) ** 2
    dCd_raw = aero["kd"] * sin2
    lp = state["aero_lp"]
    lp["dCd"] += state["aero_alpha"] * (dCd_raw - lp["dCd"])
    dCd = lp["dCd"]
    drag = (0.5 * rho * speed * speed * slxsim.CDA_PARA if state["recovered"]
            else D0 * (aero["Cd0"] + dCd) / aero["Cd0"])
    drag_vec = tuple(-drag * c / speed for c in
                     (state["vx"], state["vy"], state["vz"])) if speed > 0.05 else (0.0, 0.0, 0.0)

    lin = aero["linear"]

    def modal_force(key: str) -> float:
        return (lin[key][0] * pitch_modal + lin[key][1] * yaw_modal +
                lin[key][2] * roll_modal) * q_ratio

    raw_forces = {key: modal_force(key) for key in ("Fy", "Fz", "Ty", "Tz", "Tx")}
    for key, value in raw_forces.items():
        lp[key] += state["aero_alpha"] * (value - lp[key])
    Fy_body, Fz_body = lp["Fy"], lp["Fz"]
    Tp, Ttilt_n, Troll = lp["Ty"], lp["Tz"], lp["Tx"]
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
        "fin_act_deg": list(fin), "fin_cmd_deg": list(state["pwm_cmd"]),
        "fin_delay_deg": list(state["fin_delayed"]),
        "fin_move_from_deg": list(state["fin_move_from"]),
        "fin_move_to_deg": list(state["fin_move_to"]),
        "fin_move_start_s": list(state["fin_move_start_s"]),
        "fin_progress_pct": list(state["fin_progress_pct"]),
        "fin_rate_dps": list(state["fin_rate_dps"]),
        "drag_n": drag, "mass_kg": mass,
        "thrust_n": thrust, "dCd_cfd_raw": dCd_raw, "dCd_cfd": dCd,
        "M_pitch_cfd_raw_nm": raw_forces["Ty"], "M_pitch_nm": M_p,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--backend", choices=("cfd", "mujoco"), default="cfd")
    ap.add_argument("--motion", default=None,
                    help="recorded Taichi/CUDA motion JSON "
                         "(default: ../../build/cfd/motion.json if present)")
    ap.add_argument("--tilt", type=float, default=0.0, help="rail-exit tilt [deg]")
    ap.add_argument("--wind", type=float, default=0.0, help="steady wind [m/s]")
    ap.add_argument("--max-t", type=float, default=40.0)
    ap.add_argument("--out", default=str(HIL / "logs"))
    args = ap.parse_args()

    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=True)
    or_data = slxsim.read_or_flight(slxsim.OR_CSV)
    # Prefer the recorded Taichi/CUDA sweep produced by `make ci-live`. The
    # committed fluidx3d CSV fit remains the offline fallback. The HIL plant
    # never solves CFD itself; it replays whichever recorded lookup is chosen.
    motion_path = Path(args.motion).resolve() if args.motion else \
        (HIL.parent / "build" / "cfd" / "motion.json").resolve()
    if motion_path.exists():
        aero = json.loads(motion_path.read_text())
        aero_source = str(motion_path.relative_to(HIL.parent))
    else:
        aero = slxsim.read_cfd_lookup()
        aero_source = "committed sim/data/cfd/fluidx3d logs"
    sensors = Segment("slx_hil_sensors", SENSOR_FMT)
    fins = Segment("slx_hil_fins", FINS_FMT)

    backend = MujocoAttitudeEngine.from_config(_attitude_config()) \
        if args.backend == "mujoco" else None
    state = {
        "t": 0.0, "z": 0.0, "x": 0.0, "y": 0.0,
        "vx": 0.0, "vy": 0.0, "vz": 0.0,
        "p": 0.0, "r": 0.0, "psi": 0.0,
        "p_rate": 0.0, "r_rate": 0.0, "psi_rate": 0.0,
        "fin_act": [0.0] * 4, "pwm_cmd": [0.0] * 4,
        "fin_delayed": [0.0] * 4,
        "pending_cmds": [[] for _ in range(4)],
        "fin_move_from": [0.0] * 4, "fin_move_to": [0.0] * 4,
        "fin_move_start_s": [0.0] * 4, "fin_progress_pct": [100.0] * 4,
        "fin_rate_dps": [0.0] * 4, "pwm_tick": 0,
        "aero_lp": {key: 0.0 for key in ("dCd", "Fy", "Fz", "Ty", "Tz", "Tx")},
        "aero_alpha": slxsim.first_order_alpha(AERO_RESPONSE_TAU_S),
        "recovered": False, "max_z": 0.0,
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
        list_fields = ("fin_act_deg", "fin_cmd_deg", "fin_delay_deg",
                       "fin_move_from_deg", "fin_move_to_deg",
                       "fin_move_start_s", "fin_progress_pct", "fin_rate_dps")
        row = {k: v for k, v in published.items() if k not in list_fields}
        for i in range(4):
            for key in list_fields:
                row[f"{key}{i}"] = published[key][i]
            row[f"fw_fin_deg{i}"] = f[f"fin_cmd_deg{i}"]
        row["fw_state"] = f["state"]
        rows.append(row)
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
        "aero": {"source": aero_source, "kd": aero["kd"],
                 "cd0": aero["Cd0"], "rms": aero["drag_rms"],
                 "eval_hz": 1.0 / DT, "response_tau_s": AERO_RESPONSE_TAU_S,
                 "model": "recorded steady-state lookup + reduced-order response lag"},
        "actuator": {
            "pwm_hz": SERVO_PWM_HZ, "tau_s": SERVO_TAU_S,
            "transport_latency_ms": SERVO_LATENCY_MS,
            "transport_latency_ticks": SERVO_LATENCY_TICKS,
            "rate_limit_dps": SERVO_RATE_DPS,
            "model": "10 ms transport delay + 50 Hz latch + slew-limited "
                     "first-order servo; CFD lookup uses actual fin angles",
        },
        "note": "guidance is the real C core in cpp/bridge.cpp, not a Python port",
    }, indent=2) + "\n")
    print(f"plant: backend={args.backend} aero={aero_source} "
          f"apogee={state['max_z']:.2f} m "
          f"duration={state['t']:.2f} s logs={out}")
    sensors.close()
    fins.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
