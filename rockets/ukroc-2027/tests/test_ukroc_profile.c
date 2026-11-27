#include "ukroc_profile.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void assert_near(float a, float b, float tol)
{
    if (fabsf(a - b) > tol) {
        fprintf(stderr, "expected %.6f near %.6f (tol %.6f)\n", a, b, tol);
        assert(0);
    }
}

static size_t comma_count(const char *s)
{
    size_t n = 0U;

    while (*s != '\0') {
        if (*s == ',') {
            n++;
        }
        s++;
    }
    return n;
}

static void test_header_and_csv_row(void)
{
    ukroc_guidance_t g;
    ukroc_snapshot_t snap;
    char row[512];

    assert(strcmp(UKROC_CSV_HEADER,
                  "t_ms,state,p_pa,alt_m,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,"
                  "pitch_deg,roll_deg,fin0,fin1,fin2,fin3,vbat_v,"
                  "vv_ms,pred_apo_m,ref_alt_m,ref_vv_ms,tilt_cmd_deg") == 0);
    assert(comma_count(UKROC_CSV_HEADER) == 21U);

    ukroc_guidance_init(&g);
    snap = ukroc_guidance_tick(&g, 0U, false, 101325.0f,
                               (ukroc_imu_t){ 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f });
    assert(ukroc_format_csv_row(row, sizeof(row), &snap, 3.9f) > 0U);
    assert(comma_count(row) == 21U);
    assert(strstr(row, "\r\n") != NULL);
}

static void test_reference_table_and_schedule(void)
{
    float alt;
    float vv;

    assert_near(UKROC_K_ALT, 0.010f, 1e-6f);
    assert_near(UKROC_K_VEL, 0.080f, 1e-6f);
    assert_near(UKROC_K_APOGEE, 0.200f, 1e-6f);
    assert_near(UKROC_APOGEE_DRAG_G, 1.55f, 1e-6f);
    assert_near(UKROC_BOOST_TILT_DEG, 1.9f, 1e-6f);
    assert_near(UKROC_COAST_WEATHERVANE_TILT_DEG, 3.0f, 1e-6f);

    ukroc_reference_at(-1.0f, &alt, &vv);
    assert_near(alt, 0.0f, 1e-6f);
    assert_near(vv, 0.0f, 1e-6f);

    ukroc_reference_at(0.25f, &alt, &vv);
    assert_near(alt, 3.79f, 1e-6f);
    assert_near(vv, 15.525f, 1e-6f);

    ukroc_reference_at(2.25f, &alt, &vv);
    assert_near(alt, 108.815f, 1e-3f);
    assert_near(vv, 58.17f, 1e-3f);

    ukroc_reference_at(99.0f, &alt, &vv);
    assert_near(alt, 237.60f, 1e-6f);
    assert_near(vv, -2.98f, 1e-6f);

    assert_near(ukroc_scheduled_tilt_deg(0.0f), 0.0f, 1e-6f);
    assert_near(ukroc_scheduled_tilt_deg(0.299f), 0.0f, 1e-6f);
    assert_near(ukroc_scheduled_tilt_deg(0.300f), 1.9f, 1e-6f);
    assert_near(ukroc_scheduled_tilt_deg(2.199f), 1.9f, 1e-6f);
    assert_near(ukroc_scheduled_tilt_deg(2.200f), 0.0f, 1e-6f);
}

static void test_mixer_is_x_pattern(void)
{
    float out[4];

    ukroc_mix(1.0f, 2.0f, 3.0f, out);
    assert_near(out[0], 6.0f, 1e-6f);
    assert_near(out[1], -2.0f, 1e-6f);
    assert_near(out[2], 0.0f, 1e-6f);
    assert_near(out[3], -4.0f, 1e-6f);
}

static void test_state_machine_and_rail_hold(void)
{
    ukroc_guidance_t g;
    ukroc_snapshot_t snap;
    const ukroc_imu_t rest = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
    const ukroc_imu_t launch = { 0.0f, 0.0f, 3.0f, 0.0f, 0.0f, 0.0f };

    ukroc_guidance_init(&g);
    assert(g.state == UKROC_STATE_BOOT);
    ukroc_guidance_enter_armed(&g);
    assert(g.state == UKROC_STATE_ARMED);

    snap = ukroc_guidance_tick(&g, 0U, true, 101325.0f, rest);
    assert(snap.state == UKROC_STATE_READY);

    snap = ukroc_guidance_tick(&g, 100U, true, 101325.0f, launch);
    assert(snap.state == UKROC_STATE_LAUNCH);

    snap = ukroc_guidance_tick(&g, 450U, true, 101325.0f, rest);
    assert(snap.state == UKROC_STATE_GUIDED);
}

static void test_low_apogee_commands_vertical_only(void)
{
    ukroc_guidance_t g;
    ukroc_snapshot_t snap;
    const ukroc_imu_t rest = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
    size_t i;

    ukroc_guidance_init(&g);
    g.state = UKROC_STATE_GUIDED;
    g.launch_ms = 0U;
    g.smoothed_alt = 10.0f;
    g.prev_alt_for_vv = 10.0f;
    g.peak_alt = 10.0f;
    g.vv_est = 0.0f;

    snap = ukroc_guidance_tick(&g, 4000U, true, 101325.0f, rest);
    assert_near(snap.tilt_cmd_deg, 0.0f, 1e-6f);
    for (i = 0U; i < 4U; i++) {
        assert_near(snap.fins_deg[i], 0.0f, 1e-6f);
    }
}

static void test_accel_gate_avoids_burnout_flip(void)
{
    ukroc_guidance_t g;
    ukroc_snapshot_t snap;
    const ukroc_imu_t coast = { 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f };

    ukroc_guidance_init(&g);
    g.state = UKROC_STATE_GUIDED;
    g.launch_ms = 0U;
    g.pitch_deg = 10.0f;

    snap = ukroc_guidance_tick(&g, 3000U, true, 101325.0f, coast);
    assert_near(snap.pitch_deg, 10.0f, 1e-6f);
}

static void test_coast_weathervane(void)
{
    ukroc_guidance_t g;
    ukroc_snapshot_t snap;
    const ukroc_imu_t coast = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
    size_t i;

    /* Under power the same tilt must still drive the fins. */
    ukroc_guidance_init(&g);
    g.state = UKROC_STATE_GUIDED;
    g.launch_ms = 0U;
    g.pitch_deg = 5.0f;
    snap = ukroc_guidance_tick(&g, 1000U, true, 101325.0f, coast);
    assert(fabsf(snap.fins_deg[0]) > 1.0f);

    /* After burnout and past the tilt threshold the fins must be centred. */
    ukroc_guidance_init(&g);
    g.state = UKROC_STATE_GUIDED;
    g.launch_ms = 0U;
    g.pitch_deg = 5.0f;
    snap = ukroc_guidance_tick(&g, 3000U, true, 101325.0f, coast);
    for (i = 0U; i < 4U; i++) {
        assert_near(snap.fins_deg[i], 0.0f, 1e-6f);
    }
}

static void test_failsafe_neutral(void)
{
    ukroc_guidance_t g;
    ukroc_snapshot_t snap;
    const ukroc_imu_t rest = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f };
    size_t i;

    ukroc_guidance_init(&g);
    ukroc_guidance_enter_armed(&g);
    for (i = 0U; i < 9U; i++) {
        assert(!ukroc_guidance_note_sensor_failure(&g));
    }
    assert(ukroc_guidance_note_sensor_failure(&g));
    assert(g.state == UKROC_STATE_FAULT);
    assert(g.fault == UKROC_FAULT_SENSOR_READ);

    snap = ukroc_guidance_tick(&g, 1000U, true, 101325.0f, rest);
    assert(snap.state == UKROC_STATE_FAULT);
    for (i = 0U; i < 4U; i++) {
        assert_near(snap.fins_deg[i], 0.0f, 1e-6f);
    }

    ukroc_guidance_init(&g);
    g.state = UKROC_STATE_GUIDED;
    g.fins_deg[0] = 12.0f;
    ukroc_guidance_force_fault(&g, UKROC_FAULT_PCA9685_INIT);
    assert(g.state == UKROC_STATE_FAULT);
    for (i = 0U; i < 4U; i++) {
        assert_near(g.fins_deg[i], 0.0f, 1e-6f);
    }
}

int main(void)
{
    test_header_and_csv_row();
    test_reference_table_and_schedule();
    test_mixer_is_x_pattern();
    test_state_machine_and_rail_hold();
    test_low_apogee_commands_vertical_only();
    test_accel_gate_avoids_burnout_flip();
    test_coast_weathervane();
    test_failsafe_neutral();

    puts("ukroc_profile: all tests passed");
    return 0;
}
