/*
 * S-LX rocket profile (UKROC 2027 competition) — hardware-independent guidance core.
 *
 * Ported from rocket/firmware/esp32_rust/guidance-core/src/lib.rs, then tuned
 * against the coupled FP16S/OpenRocket 100 Hz surrogate. The ESP32-C3 Rust
 * firmware is deliberately unchanged; the tuning divergence is STM32-only.
 * The host test in tests/test_slx_profile.c checks the constants, reference
 * table, boost schedule, mixer, state machine and failsafe-neutral behaviour.
 */

#include "slx_profile.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct {
    float t;
    float alt;
    float vv;
} slx_ref_point_t;

/* Reference flight from the saved fixed-fin OpenRocket design (AeroTech F32). */
static const slx_ref_point_t slx_ref_traj[] = {
    { 0.0f,   0.00f,  0.00f },
    { 0.5f,   7.58f, 31.05f },
    { 1.0f,  29.21f, 55.30f },
    { 1.5f,  60.79f, 68.70f },
    { 2.0f,  94.29f, 62.09f },
    { 2.5f, 123.34f, 54.25f },
    { 3.0f, 148.66f, 47.12f },
    { 3.5f, 170.56f, 40.55f },
    { 4.0f, 189.29f, 34.43f },
    { 4.5f, 205.05f, 28.67f },
    { 5.0f, 218.00f, 23.18f },
    { 5.5f, 228.27f, 17.91f },
    { 6.0f, 235.64f,  8.86f },
    { 6.5f, 237.89f,  2.03f },
    { 7.0f, 237.60f, -2.98f }
};

#define SLX_REF_N (sizeof(slx_ref_traj) / sizeof(slx_ref_traj[0]))

static float wrap_to_nearest(float angle, float current)
{
    float wrapped = angle;

    while (wrapped - current > 180.0f) {
        wrapped -= 360.0f;
    }
    while (wrapped - current < -180.0f) {
        wrapped += 360.0f;
    }
    return wrapped;
}

float slx_clampf(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

float slx_altitude_from_pressure(float pressure_pa, float sea_level_pa)
{
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 1.0f / 5.255f));
}

void slx_reference_at(float t, float *alt, float *vv)
{
    size_t i;
    const slx_ref_point_t *last;

    if (alt == NULL || vv == NULL) {
        return;
    }
    if (t <= slx_ref_traj[0].t) {
        *alt = slx_ref_traj[0].alt;
        *vv = slx_ref_traj[0].vv;
        return;
    }
    for (i = 1; i < SLX_REF_N; i++) {
        if (t <= slx_ref_traj[i].t) {
            const slx_ref_point_t *a = &slx_ref_traj[i - 1];
            const slx_ref_point_t *b = &slx_ref_traj[i];
            float f = (t - a->t) / (b->t - a->t);

            *alt = a->alt + f * (b->alt - a->alt);
            *vv = a->vv + f * (b->vv - a->vv);
            return;
        }
    }
    last = &slx_ref_traj[SLX_REF_N - 1U];
    *alt = last->alt;
    *vv = last->vv;
}

float slx_scheduled_tilt_deg(float t)
{
    if (t < SLX_BOOST_START_S) {
        return 0.0f;
    }
    if (t < SLX_BOOST_END_S) {
        return SLX_BOOST_TILT_DEG;
    }
    return 0.0f;
}

void slx_mix(float pitch_cmd, float roll_cmd, float yaw_damp, float out[4])
{
    if (out == NULL) {
        return;
    }
    out[0] =  pitch_cmd + roll_cmd + yaw_damp;
    out[1] = -pitch_cmd + roll_cmd - yaw_damp;
    out[2] = -pitch_cmd - roll_cmd + yaw_damp;
    out[3] =  pitch_cmd - roll_cmd - yaw_damp;
}

const char *slx_state_name(slx_state_t state)
{
    switch (state) {
    case SLX_STATE_BOOT:    return "BOOT";
    case SLX_STATE_ARMED:   return "ARMED";
    case SLX_STATE_READY:   return "READY";
    case SLX_STATE_LAUNCH:  return "LAUNCH";
    case SLX_STATE_GUIDED:  return "GUIDED";
    case SLX_STATE_DESCENT: return "DESCENT";
    case SLX_STATE_LANDED:  return "LANDED";
    case SLX_STATE_FAULT:   return "FAULT";
    default:                  return "UNKNOWN";
    }
}

const char *slx_fault_name(slx_fault_t fault)
{
    switch (fault) {
    case SLX_FAULT_NONE:           return "none";
    case SLX_FAULT_SD_INIT:        return "SD init";
    case SLX_FAULT_LOG_INIT:       return "log open";
    case SLX_FAULT_PCA9685_INIT:   return "PCA9685 init";
    case SLX_FAULT_BMP280_INIT:    return "BMP280 init";
    case SLX_FAULT_ICM20689_INIT:  return "ICM20689 init";
    case SLX_FAULT_SENSOR_READ:    return "sensor read failures";
    default:                         return "unknown";
    }
}

void slx_guidance_init(slx_guidance_t *g)
{
    if (g == NULL) {
        return;
    }
    memset(g, 0, sizeof(*g));
    g->state = SLX_STATE_BOOT;
    g->fault = SLX_FAULT_NONE;
    g->sea_level_pa = 101325.0f;
}

void slx_guidance_set_sea_level_pressure(slx_guidance_t *g, float pressure_pa)
{
    if (g != NULL && pressure_pa > 10000.0f && pressure_pa < 200000.0f) {
        g->sea_level_pa = pressure_pa;
    }
}

void slx_guidance_enter_armed(slx_guidance_t *g)
{
    if (g != NULL && g->state == SLX_STATE_BOOT) {
        g->state = SLX_STATE_ARMED;
    }
}

void slx_guidance_force_fault(slx_guidance_t *g, slx_fault_t reason)
{
    if (g == NULL) {
        return;
    }
    g->fault = reason;
    memset(g->fins_deg, 0, sizeof(g->fins_deg));
    g->state = SLX_STATE_FAULT;
}

bool slx_guidance_note_sensor_failure(slx_guidance_t *g)
{
    if (g == NULL) {
        return false;
    }
    if (g->sensor_faults < UINT8_MAX) {
        g->sensor_faults++;
    }
    if (g->sensor_faults >= 10U) {
        slx_guidance_force_fault(g, SLX_FAULT_SENSOR_READ);
        return true;
    }
    return false;
}

static slx_snapshot_t slx_snapshot(const slx_guidance_t *g,
                                       uint32_t now_ms,
                                       float pressure_pa,
                                       slx_imu_t imu,
                                       bool close_log)
{
    slx_snapshot_t snap;

    memset(&snap, 0, sizeof(snap));
    snap.t_ms = now_ms;
    snap.state = g->state;
    snap.pressure_pa = pressure_pa;
    snap.alt_m = g->smoothed_alt;
    snap.imu = imu;
    snap.pitch_deg = g->pitch_deg;
    snap.roll_deg = g->roll_deg;
    memcpy(snap.fins_deg, g->fins_deg, sizeof(snap.fins_deg));
    snap.vv_ms = g->vv_est;
    snap.pred_apo_m = g->pred_apogee;
    snap.ref_alt_m = g->ref_alt;
    snap.ref_vv_ms = g->ref_vv;
    snap.tilt_cmd_deg = g->tilt_cmd_deg;
    snap.close_log = close_log;
    return snap;
}

static void slx_write_fins(slx_guidance_t *g, const float cmd[4])
{
    size_t i;

    for (i = 0; i < 4U; i++) {
        g->fins_deg[i] = slx_clampf(cmd[i], -SLX_MAX_FIN_DEG, SLX_MAX_FIN_DEG);
    }
}

slx_snapshot_t slx_guidance_tick(slx_guidance_t *g,
                                     uint32_t now_ms,
                                     bool arm_closed,
                                     float pressure_pa,
                                     slx_imu_t imu)
{
    float alt;
    float v_baro;
    float accel_g;
    bool close_log = false;
    float cmd[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

    if (g == NULL) {
        slx_snapshot_t snap;
        memset(&snap, 0, sizeof(snap));
        snap.state = SLX_STATE_FAULT;
        snap.close_log = false;
        return snap;
    }

    if (g->state == SLX_STATE_FAULT) {
        return slx_snapshot(g, now_ms, pressure_pa, imu, false);
    }

    alt = slx_altitude_from_pressure(pressure_pa, g->sea_level_pa);
    g->smoothed_alt += 0.20f * (alt - g->smoothed_alt);
    v_baro = (g->smoothed_alt - g->prev_alt_for_vv) / SLX_DT;
    g->prev_alt_for_vv = g->smoothed_alt;

    accel_g = sqrtf(imu.ax_g * imu.ax_g + imu.ay_g * imu.ay_g + imu.az_g * imu.az_g);

    /*
     * Gate the accelerometer gravity proxy on az > 0.5 g. After F32 burnout
     * the specific-force reversal makes atan2() snap by 180 degrees; the fins
     * must stay on gyro-only propagation in that condition.
     */
    if (imu.az_g > SLX_ACCEL_TILT_MIN_AZ) {
        float accel_pitch = atan2f(-imu.ax_g, imu.az_g) * 180.0f / (float)M_PI;
        float accel_roll = atan2f(imu.ay_g, imu.az_g) * 180.0f / (float)M_PI;

        accel_pitch = wrap_to_nearest(accel_pitch, g->pitch_deg);
        accel_roll = wrap_to_nearest(accel_roll, g->roll_deg);
        g->pitch_deg = (1.0f - SLX_COMP_ALPHA) * (g->pitch_deg + imu.gx_dps * SLX_DT)
                     + SLX_COMP_ALPHA * accel_pitch;
        g->roll_deg = (1.0f - SLX_COMP_ALPHA) * (g->roll_deg + imu.gy_dps * SLX_DT)
                    + SLX_COMP_ALPHA * accel_roll;
    } else {
        g->pitch_deg += imu.gx_dps * SLX_DT;
        g->roll_deg += imu.gy_dps * SLX_DT;
    }

    switch (g->state) {
    case SLX_STATE_ARMED:
        if (arm_closed) {
            memset(g->fins_deg, 0, sizeof(g->fins_deg));
            g->pitch_deg = 0.0f;
            g->roll_deg = 0.0f;
            g->prev_alt_for_vv = g->smoothed_alt;
            g->state = SLX_STATE_READY;
        }
        break;

    case SLX_STATE_READY:
        if (!arm_closed) {
            memset(g->fins_deg, 0, sizeof(g->fins_deg));
            g->state = SLX_STATE_ARMED;
        } else if (accel_g > SLX_ARM_THRESHOLD_G) {
            g->launch_ms = now_ms;
            g->peak_alt = g->smoothed_alt;
            g->pitch_deg = 0.0f;
            g->roll_deg = 0.0f;
            g->vv_est = 0.0f;
            g->prev_alt_for_vv = g->smoothed_alt;
            g->pred_apogee = 0.0f;
            g->ref_alt = 0.0f;
            g->ref_vv = 0.0f;
            g->tilt_cmd_deg = 0.0f;
            g->state = SLX_STATE_LAUNCH;
        }
        break;

    case SLX_STATE_LAUNCH:
        if ((uint32_t)(now_ms - g->launch_ms) > SLX_RAIL_CLEARANCE_MS) {
            g->vv_est = v_baro;
            g->state = SLX_STATE_GUIDED;
        }
        break;

    case SLX_STATE_GUIDED: {
        float t_from_launch = (float)(uint32_t)(now_ms - g->launch_ms) / 1000.0f;
        float pitch_rad;
        float roll_rad;
        float f_up;
        float trim;
        float target;
        float pitch_cmd;
        float roll_cmd;
        float yaw_damp;

        slx_reference_at(t_from_launch, &g->ref_alt, &g->ref_vv);

        pitch_rad = g->pitch_deg * (float)M_PI / 180.0f;
        roll_rad = g->roll_deg * (float)M_PI / 180.0f;
        f_up = -imu.ax_g * sinf(pitch_rad)
             + imu.ay_g * sinf(roll_rad)
             + imu.az_g * cosf(pitch_rad) * cosf(roll_rad);
        g->vv_est += (f_up - 1.0f) * SLX_G_M_S2 * SLX_DT;
        g->vv_est += SLX_VV_BARO_BLEND * (v_baro - g->vv_est);
        g->vv_est = slx_clampf(g->vv_est, 0.0f, 120.0f);

        if (g->vv_est > 0.0f) {
            g->pred_apogee = g->smoothed_alt + g->vv_est * g->vv_est
                           / (2.0f * SLX_G_M_S2 * SLX_APOGEE_DRAG_G);
        } else {
            g->pred_apogee = g->smoothed_alt;
        }

        trim = SLX_K_ALT * (g->smoothed_alt - g->ref_alt)
             + SLX_K_VEL * (g->vv_est - g->ref_vv);
        if (t_from_launch >= SLX_BURNOUT_T_S) {
            trim += SLX_K_APOGEE * (g->pred_apogee - SLX_TARGET_APOGEE_M);
        }
        target = slx_clampf(slx_scheduled_tilt_deg(t_from_launch) + trim,
                              0.0f, SLX_MAX_TILT_DEG);
        g->tilt_cmd_deg = target;

        pitch_cmd = SLX_KP_TILT * (target - g->pitch_deg) - SLX_KD_RATE * imu.gx_dps;
        roll_cmd = SLX_KP_TILT * (0.0f - g->roll_deg) - SLX_KD_RATE * imu.gy_dps;
        yaw_damp = SLX_KD_YAW * imu.gz_dps;
        slx_mix(pitch_cmd, roll_cmd, yaw_damp, cmd);
        slx_write_fins(g, cmd);

        /*
         * Coast weathervane. Once thrust has ended the fins no longer change
         * the trajectory. Holding an absolute attitude only opposes static
         * stability, saturates the servos and adds drag; if the airframe has
         * left the near-vertical corridor, centre the fins and let the
         * restoring moment follow the flight path. This is a fail-safe
         * neutral output, so a bogus tilt estimate cannot drive a fin hard over.
         */
        if (t_from_launch >= SLX_BURNOUT_T_S &&
            sqrtf(g->pitch_deg * g->pitch_deg + g->roll_deg * g->roll_deg)
                > SLX_COAST_WEATHERVANE_TILT_DEG) {
            memset(g->fins_deg, 0, sizeof(g->fins_deg));
        }

        if (g->smoothed_alt > g->peak_alt) {
            g->peak_alt = g->smoothed_alt;
        }
        if (g->smoothed_alt < g->peak_alt - 3.0f) {
            memset(g->fins_deg, 0, sizeof(g->fins_deg));
            g->state = SLX_STATE_DESCENT;
        }
        break;
    }

    case SLX_STATE_DESCENT:
        if (accel_g < SLX_LAND_THRESHOLD_G) {
            if (g->land_ms == 0U) {
                g->land_ms = now_ms;
            } else if ((uint32_t)(now_ms - g->land_ms) > 2000U) {
                memset(g->fins_deg, 0, sizeof(g->fins_deg));
                close_log = true;
                g->state = SLX_STATE_LANDED;
            }
        } else {
            g->land_ms = 0U;
        }
        break;

    case SLX_STATE_BOOT:
    case SLX_STATE_LANDED:
    case SLX_STATE_FAULT:
    default:
        break;
    }

    g->sensor_faults = 0U;
    return slx_snapshot(g, now_ms, pressure_pa, imu, close_log);
}

static char *slx_append_u32(char *p, char *end, uint32_t value)
{
    char tmp[10U];
    uint8_t n = 0U;

    do {
        tmp[n++] = (char) ('0' + (uint8_t) (value % 10U));
        value /= 10U;
    } while (value != 0U && n < sizeof(tmp));

    while (n != 0U && p < end) {
        *p++ = tmp[--n];
    }
    return p;
}

static char *slx_append_fixed(char *p, char *end, float value, uint8_t decimals)
{
    uint32_t scale = 1U;
    uint8_t i;
    float scaled;

    if (p >= end) {
        return p;
    }
    if (value < 0.0f) {
        *p++ = '-';
        value = -value;
    }
    for (i = 0U; i < decimals; i++) {
        scale *= 10U;
    }
    scaled = value * (float) scale + 0.5f;
    if (scaled >= 4294967295.0f) {
        scaled = 4294967295.0f;
    }
    p = slx_append_u32(p, end, (uint32_t) scaled / scale);
    if (decimals != 0U && p < end) {
        uint32_t fraction = (uint32_t) scaled % scale;

        *p++ = '.';
        for (i = 0U; i < decimals && p < end; i++) {
            uint32_t divisor = scale / 10U;
            uint8_t digit;

            if (i != 0U) {
                divisor /= 10U;
            }
            digit = (uint8_t) (fraction / divisor);
            fraction %= divisor;
            *p++ = (char) ('0' + digit);
        }
    }
    return p;
}

size_t slx_format_csv_row(char *buf, size_t len,
                            const slx_snapshot_t *snap, float vbat_v)
{
    char *p;
    char *end;

    if (buf == NULL || len == 0U || snap == NULL) {
        return 0U;
    }
    end = buf + len - 1U;
    p = buf;

    p = slx_append_u32(p, end, snap->t_ms);
    if (p < end) { *p++ = ','; }
    p = slx_append_u32(p, end, (uint32_t) snap->state);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->pressure_pa, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->alt_m, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->imu.ax_g, 3U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->imu.ay_g, 3U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->imu.az_g, 3U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->imu.gx_dps, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->imu.gy_dps, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->imu.gz_dps, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->pitch_deg, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->roll_deg, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->fins_deg[0], 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->fins_deg[1], 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->fins_deg[2], 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->fins_deg[3], 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, vbat_v, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->vv_ms, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->pred_apo_m, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->ref_alt_m, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->ref_vv_ms, 2U);
    if (p < end) { *p++ = ','; }
    p = slx_append_fixed(p, end, snap->tilt_cmd_deg, 2U);
    if (p < end) { *p++ = '\r'; }
    if (p < end) { *p++ = '\n'; }
    *p = '\0';
    return (size_t) (p - buf);
}
