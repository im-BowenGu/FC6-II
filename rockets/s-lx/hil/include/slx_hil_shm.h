/*
 * S-LX HIL shared-memory ABI. Used by both the C++ bridge (Verilated RTL +
 * real guidance core) and the Python physics plant. Layouts are packed and
 * mirrored exactly in plant/shm.py -- keep both in sync.
 */
#ifndef SLX_HIL_SHM_H
#define SLX_HIL_SHM_H

#include <stdint.h>

#define SLX_HIL_SENSORS_NAME "/slx_hil_sensors"
#define SLX_HIL_FINS_NAME    "/slx_hil_fins"

#define SLX_HIL_FLAG_RUNNING 0x1u
#define SLX_HIL_FLAG_DONE    0x2u
#define SLX_HIL_FLAG_FAULT   0x4u

/* 8 + 19*8 + 4 + 4 = 168 bytes. <Q19dII in Python. */
typedef struct __attribute__((packed)) {
    uint64_t seq;          /* seqlock: even = stable, odd = writer active */
    double t_s;
    double alt_m;
    double pressure_pa;
    double vx_mps, vy_mps, vz_mps;
    double pitch_deg, roll_deg, yaw_deg;
    double p_rate_dps, r_rate_dps, y_rate_dps;
    double ax_g, ay_g, az_g;
    double gx_dps, gy_dps, gz_dps;
    double q_ratio;
    uint32_t flags;
    uint32_t reserved;
} slx_hil_sensors_t;

/* 8 + 9*8 + 4 + 4 = 88 bytes. <Q9dII in Python. */
typedef struct __attribute__((packed)) {
    uint64_t seq;
    double t_s;
    double fin_cmd_deg[4];
    double fin_act_deg[4];
    uint32_t state;
    uint32_t flags;
} slx_hil_fins_t;

#endif /* SLX_HIL_SHM_H */
