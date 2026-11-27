/*
 * UKROC 2027 rocket profile for the STM32 FC6-II fork.
 *
 * Hardware-independent guidance constants and control law. The 100 Hz state
 * machine, reference trajectory and failsafe posture were ported from the
 * authoritative Rust guidance-core in rocket/firmware/esp32_rust/. The trim
 * constants below were then tuned against the coupled FP16S/OpenRocket
 * surrogate; the ESP32-C3 Rust firmware is unchanged, so the two options now
 * intentionally differ in apogee-trim tuning only.
 *
 * No radio, GPS, ground-control or pyrotechnic interfaces are declared here.
 */

#ifndef UKROC_PROFILE_H
#define UKROC_PROFILE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define UKROC_LOOP_HZ                100U
#define UKROC_DT                     (1.0f / (float)UKROC_LOOP_HZ)
#define UKROC_ARM_THRESHOLD_G        2.0f
#define UKROC_LAND_THRESHOLD_G       0.3f
#define UKROC_RAIL_CLEARANCE_MS      300U
#define UKROC_MAX_FIN_DEG            15.0f
#define UKROC_KP_TILT                2.0f
#define UKROC_KD_RATE                0.35f
#define UKROC_KD_YAW                 0.25f
#define UKROC_COMP_ALPHA             0.03f
#define UKROC_ACCEL_TILT_MIN_AZ      0.5f

#define UKROC_TARGET_APOGEE_M        229.0f
#define UKROC_K_ALT                  0.010f
#define UKROC_K_VEL                  0.080f
#define UKROC_K_APOGEE               0.200f
#define UKROC_MAX_TILT_DEG           4.0f
#define UKROC_BURNOUT_T_S            2.0f
/* After burnout the fins cannot steer and their authority collapses with q.
 * Past this body-tilt threshold, centring them lets static stability align the
 * airframe with its flight path instead of fighting it into fin saturation.
 */
#define UKROC_COAST_WEATHERVANE_TILT_DEG 3.0f
#define UKROC_APOGEE_DRAG_G          1.55f
#define UKROC_VV_BARO_BLEND          0.05f
#define UKROC_G_M_S2                 9.80665f

/* Tuned boost tilt programme after the coupled 100 Hz surrogate sweep. */
#define UKROC_BOOST_TILT_DEG         1.9f
#define UKROC_BOOST_START_S          0.30f
#define UKROC_BOOST_END_S            2.20f

#define UKROC_SERVO_CENTER_US        1500.0f
#define UKROC_SERVO_US_PER_DEG       (600.0f / UKROC_MAX_FIN_DEG)
#define UKROC_SERVO_MIN_US           900.0f
#define UKROC_SERVO_MAX_US           2100.0f

/* Rule-13 safety rails: these are compile-time statements of intent. */
#define UKROC_HAS_RADIO              0
#define UKROC_HAS_GPS                0
#define UKROC_HAS_PYRO               0
#define UKROC_HAS_GROUND_CONTROL     0

#define UKROC_CSV_HEADER \
    "t_ms,state,p_pa,alt_m,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps," \
    "pitch_deg,roll_deg,fin0,fin1,fin2,fin3,vbat_v," \
    "vv_ms,pred_apo_m,ref_alt_m,ref_vv_ms,tilt_cmd_deg"

typedef enum {
    UKROC_STATE_BOOT = 0,
    UKROC_STATE_ARMED = 1,
    UKROC_STATE_READY = 2,
    UKROC_STATE_LAUNCH = 3,
    UKROC_STATE_GUIDED = 4,
    UKROC_STATE_DESCENT = 5,
    UKROC_STATE_LANDED = 6,
    UKROC_STATE_FAULT = 7
} ukroc_state_t;

typedef enum {
    UKROC_FAULT_NONE = 0,
    UKROC_FAULT_SD_INIT = 1,
    UKROC_FAULT_LOG_INIT = 2,
    UKROC_FAULT_PCA9685_INIT = 3,
    UKROC_FAULT_BMP280_INIT = 4,
    UKROC_FAULT_ICM20689_INIT = 5,
    UKROC_FAULT_SENSOR_READ = 6
} ukroc_fault_t;

typedef struct {
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
} ukroc_imu_t;

typedef struct {
    uint32_t t_ms;
    ukroc_state_t state;
    float pressure_pa;
    float alt_m;
    ukroc_imu_t imu;
    float pitch_deg;
    float roll_deg;
    float fins_deg[4];
    float vv_ms;
    float pred_apo_m;
    float ref_alt_m;
    float ref_vv_ms;
    float tilt_cmd_deg;
    bool close_log;
} ukroc_snapshot_t;

typedef struct {
    ukroc_state_t state;
    ukroc_fault_t fault;
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
} ukroc_guidance_t;

void ukroc_guidance_init(ukroc_guidance_t *g);
void ukroc_guidance_set_sea_level_pressure(ukroc_guidance_t *g, float pressure_pa);
void ukroc_guidance_enter_armed(ukroc_guidance_t *g);
bool ukroc_guidance_note_sensor_failure(ukroc_guidance_t *g);
void ukroc_guidance_force_fault(ukroc_guidance_t *g, ukroc_fault_t reason);
ukroc_snapshot_t ukroc_guidance_tick(ukroc_guidance_t *g,
                                     uint32_t now_ms,
                                     bool arm_closed,
                                     float pressure_pa,
                                     ukroc_imu_t imu);

float ukroc_clampf(float value, float lo, float hi);
float ukroc_altitude_from_pressure(float pressure_pa, float sea_level_pa);
void ukroc_reference_at(float t, float *alt, float *vv);
float ukroc_scheduled_tilt_deg(float t);
void ukroc_mix(float pitch_cmd, float roll_cmd, float yaw_damp, float out[4]);
const char *ukroc_state_name(ukroc_state_t state);
const char *ukroc_fault_name(ukroc_fault_t fault);
size_t ukroc_format_csv_row(char *buf, size_t len,
                            const ukroc_snapshot_t *snap, float vbat_v);

#endif /* UKROC_PROFILE_H */
