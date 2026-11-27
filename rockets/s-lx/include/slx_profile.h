/*
 * S-LX rocket profile for the STM32 FC6-II fork (UKROC 2027 competition).
 *
 * Hardware-independent guidance constants and control law. The 100 Hz state
 * machine, reference trajectory and failsafe posture were ported from the
 * authoritative Rust guidance-core in rocket/firmware/esp32_rust/. The trim
 * constants below were then tuned against the coupled static-CFD/OpenRocket
 * surrogate; the ESP32-C3 Rust firmware is unchanged, so the two options now
 * intentionally differ in apogee-trim tuning only.
 *
 * No radio, GPS, ground-control or pyrotechnic interfaces are declared here.
 */

#ifndef SLX_PROFILE_H
#define SLX_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define SLX_LOOP_HZ                100U
#define SLX_DT                     (1.0f / (float)SLX_LOOP_HZ)
#define SLX_ARM_THRESHOLD_G        2.0f
#define SLX_LAND_THRESHOLD_G       0.3f
#define SLX_RAIL_CLEARANCE_MS      300U
#define SLX_MAX_FIN_DEG            15.0f
#define SLX_KP_TILT                2.0f
#define SLX_KD_RATE                0.35f
#define SLX_KD_YAW                 0.25f
#define SLX_COMP_ALPHA             0.03f
#define SLX_ACCEL_TILT_MIN_AZ      0.5f

#define SLX_TARGET_APOGEE_M        229.0f
#define SLX_K_ALT                  0.010f
#define SLX_K_VEL                  0.080f
#define SLX_K_APOGEE               0.200f
#define SLX_MAX_TILT_DEG           4.0f
#define SLX_BURNOUT_T_S            2.0f
/* After burnout the fins cannot steer and their authority collapses with q.
 * Past this body-tilt threshold, centring them lets static stability align the
 * airframe with its flight path instead of fighting it into fin saturation.
 */
#define SLX_COAST_WEATHERVANE_TILT_DEG 3.0f
#define SLX_APOGEE_DRAG_G          1.55f
#define SLX_VV_BARO_BLEND          0.05f
#define SLX_G_M_S2                 9.80665f

/* Tuned boost tilt programme after the coupled 100 Hz surrogate sweep. */
#define SLX_BOOST_TILT_DEG         1.9f
#define SLX_BOOST_START_S          0.30f
#define SLX_BOOST_END_S            2.20f

#define SLX_SERVO_CENTER_US        1500.0f
#define SLX_SERVO_US_PER_DEG       (600.0f / SLX_MAX_FIN_DEG)
#define SLX_SERVO_MIN_US           900.0f
#define SLX_SERVO_MAX_US           2100.0f

/* Rule-13 safety rails: these are compile-time statements of intent. */
#define SLX_HAS_RADIO              0
#define SLX_HAS_GPS                0
#define SLX_HAS_PYRO               0
#define SLX_HAS_GROUND_CONTROL     0

#define SLX_CSV_HEADER \
    "t_ms,state,p_pa,alt_m,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps," \
    "pitch_deg,roll_deg,fin0,fin1,fin2,fin3,vbat_v," \
    "vv_ms,pred_apo_m,ref_alt_m,ref_vv_ms,tilt_cmd_deg"

typedef enum {
    SLX_STATE_BOOT = 0,
    SLX_STATE_ARMED = 1,
    SLX_STATE_READY = 2,
    SLX_STATE_LAUNCH = 3,
    SLX_STATE_GUIDED = 4,
    SLX_STATE_DESCENT = 5,
    SLX_STATE_LANDED = 6,
    SLX_STATE_FAULT = 7
} slx_state_t;

typedef enum {
    SLX_FAULT_NONE = 0,
    SLX_FAULT_SD_INIT = 1,
    SLX_FAULT_LOG_INIT = 2,
    SLX_FAULT_PCA9685_INIT = 3,
    SLX_FAULT_BMP280_INIT = 4,
    SLX_FAULT_ICM20689_INIT = 5,
    SLX_FAULT_SENSOR_READ = 6
} slx_fault_t;

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} slx_imu_t;

typedef struct {
    uint32_t t_ms;
    slx_state_t state;
    float pressure_pa;
    float alt_m;
    slx_imu_t imu;
    float pitch_deg;
    float roll_deg;
    float fins_deg[4];
    float vv_ms;
    float pred_apo_m;
    float ref_alt_m;
    float ref_vv_ms;
    float tilt_cmd_deg;
    bool close_log;
} slx_snapshot_t;

typedef struct {
    slx_state_t state;
    slx_fault_t fault;
    uint32_t launch_ms;
    uint32_t land_ms;
    float sea_level_pa;
    float smoothed_alt;
    float peak_alt;
    float pitch_deg;
    float roll_deg;
    float fins_deg[4];
    uint8_t sensor_faults;
    float vv_est;
    float prev_alt_for_vv;
    float pred_apogee;
    float ref_alt;
    float ref_vv;
    float tilt_cmd_deg;
} slx_guidance_t;

void slx_guidance_init(slx_guidance_t *g);
void slx_guidance_set_sea_level_pressure(slx_guidance_t *g, float pressure_pa);
void slx_guidance_enter_armed(slx_guidance_t *g);
bool slx_guidance_note_sensor_failure(slx_guidance_t *g);
void slx_guidance_force_fault(slx_guidance_t *g, slx_fault_t reason);
slx_snapshot_t slx_guidance_tick(slx_guidance_t *g,
                                     uint32_t now_ms,
                                     bool arm_closed,
                                     float pressure_pa,
                                     slx_imu_t imu);

float slx_clampf(float value, float lo, float hi);
float slx_altitude_from_pressure(float pressure_pa, float sea_level_pa);
void slx_reference_at(float t, float *alt, float *vv);
float slx_scheduled_tilt_deg(float t);
void slx_mix(float pitch_cmd, float roll_cmd, float yaw_damp, float out[4]);
const char *slx_state_name(slx_state_t state);
const char *slx_fault_name(slx_fault_t fault);
size_t slx_format_csv_row(char *buf, size_t len,
                            const slx_snapshot_t *snap, float vbat_v);

#endif /* SLX_PROFILE_H */
