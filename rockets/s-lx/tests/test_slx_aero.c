#include "slx_aero.h"
#include "slx_profile.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void assert_near(float a, float b, float tol)
{
    if (fabsf(a - b) > tol) {
        fprintf(stderr, "expected %.6f near %.6f (tol %.6f)\n", a, b, tol);
        assert(0);
    }
}

static void test_q_ratio(void)
{
    assert_near(slx_aero_q_ratio(26.0f, 0.0f), 1.0f, 0.01f);
    assert(slx_aero_q_ratio(13.0f, 0.0f) < 0.3f);
    assert(slx_aero_q_ratio(26.0f, 5000.0f) < slx_aero_q_ratio(26.0f, 0.0f));
    assert_near(slx_aero_clamp_q(-1.0f), SLX_AERO_Q_MIN, 1e-6f);
    assert_near(slx_aero_clamp_q(99.0f), SLX_AERO_Q_MAX, 1e-6f);
}

static void test_forward_moments(void)
{
    const float fins[4] = { 5.0f, 0.0f, -5.0f, 0.0f };
    float moments[3] = { 0.0f, 0.0f, 0.0f };

    slx_aero_predict_moments(fins, 1.0f, moments);
    assert_near(moments[0], SLX_AERO_TY_PITCH * 5.0f, 1e-6f);
    assert_near(moments[1], SLX_AERO_TZ_PITCH * 5.0f, 1e-6f);
    assert_near(moments[2], SLX_AERO_TX_PITCH * 5.0f, 1e-6f);

    slx_aero_predict_moments(fins, 2.0f, moments);
    assert_near(moments[0], 2.0f * SLX_AERO_TY_PITCH * 5.0f, 1e-6f);
}

static void test_inverse_mixer(void)
{
    const float modal[3] = { 4.0f, -2.5f, 1.5f };
    float desired[3];
    float fins[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float recovered[3];
    float moments[3];

    slx_aero_predict_moments((const float[4]) {
        modal[0] + modal[2], modal[1] + modal[2],
        -modal[0] + modal[2], -modal[1] + modal[2]
    }, 1.0f, desired);

    assert(slx_aero_mix_from_moments(desired, 1.0f, fins));
    assert_near(fins[0], 5.5f, 0.02f);
    assert_near(fins[1], -1.0f, 0.02f);
    assert_near(fins[2], -2.5f, 0.02f);
    assert_near(fins[3], 4.0f, 0.02f);

    recovered[0] = (fins[0] - fins[2]) * 0.5f;
    recovered[1] = (fins[1] - fins[3]) * 0.5f;
    recovered[2] = (fins[0] + fins[1] + fins[2] + fins[3]) * 0.25f;
    assert_near(recovered[0], modal[0], 0.02f);
    assert_near(recovered[1], modal[1], 0.02f);
    assert_near(recovered[2], modal[2], 0.02f);

    slx_aero_predict_moments(fins, 1.0f, moments);
    for (int i = 0; i < 3; i++) {
        assert_near(moments[i], desired[i], 1e-4f);
    }

    /* Four times the dynamic pressure needs a quarter of the deflection. */
    assert(slx_aero_mix_from_moments(desired, 4.0f, fins));
    recovered[0] = (fins[0] - fins[2]) * 0.5f;
    recovered[1] = (fins[1] - fins[3]) * 0.5f;
    recovered[2] = (fins[0] + fins[1] + fins[2] + fins[3]) * 0.25f;
    assert_near(recovered[0], modal[0] / 4.0f, 0.02f);
    assert_near(recovered[1], modal[1] / 4.0f, 0.02f);
    assert_near(recovered[2], modal[2] / 4.0f, 0.02f);
    slx_aero_predict_moments(fins, 4.0f, moments);
    for (int i = 0; i < 3; i++) {
        assert_near(moments[i], desired[i], 1e-4f);
    }
}

static void test_drag_model(void)
{
    const float neutral[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    const float full[4] = { 15.0f, 15.0f, 15.0f, 15.0f };

    assert_near(slx_aero_predict_drag_cd(neutral), 0.0f, 1e-6f);
    assert_near(slx_aero_predict_drag_cd(full), 4.0f * SLX_AERO_KD, 1e-5f);
}

int main(void)
{
    test_q_ratio();
    test_forward_moments();
    test_inverse_mixer();
    test_drag_model();
    puts("slx_aero: ok");
    return 0;
}
