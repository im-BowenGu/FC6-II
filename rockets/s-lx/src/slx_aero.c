/*
 * S-LX fin-to-aerodynamics model. See include/slx_aero.h for provenance.
 *
 * Modal deflections follow the same convention as the physics plant:
 *   pitch = (fin0 - fin2) / 2
 *   yaw   = (fin1 - fin3) / 2
 *   roll  = (fin0 + fin1 + fin2 + fin3) / 4
 * The moment vector is (Ty, Tz, Tx), scaled linearly by dynamic pressure ratio.
 */
#include "slx_aero.h"
#include "slx_profile.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float dq_clamp(float value, float lo, float hi)
{
    if (value < lo) {
        return lo;
    }
    if (value > hi) {
        return hi;
    }
    return value;
}

float slx_aero_clamp_q(float q_ratio)
{
    if (!isfinite(q_ratio)) {
        return SLX_AERO_Q_MIN;
    }
    return dq_clamp(q_ratio, SLX_AERO_Q_MIN, SLX_AERO_Q_MAX);
}

float slx_aero_q_ratio(float speed_mps, float altitude_m)
{
    float altitude_fraction;
    float density;
    float q_pa;

    speed_mps = fabsf(speed_mps);
    if (!isfinite(speed_mps) || !isfinite(altitude_m)) {
        return SLX_AERO_Q_MIN;
    }
    altitude_fraction = dq_clamp(altitude_m / 44330.0f, 0.0f, 0.95f);
    density = SLX_AERO_RHO_REF_KGM3 *
              powf(1.0f - altitude_fraction, 4.255f);
    q_pa = 0.5f * density * speed_mps * speed_mps;
    return slx_aero_clamp_q(q_pa / SLX_AERO_Q_REF_PA);
}

static void modal_from_fins(const float fins_deg[4], float modal[3])
{
    modal[0] = (fins_deg[0] - fins_deg[2]) * 0.5f;
    modal[1] = (fins_deg[1] - fins_deg[3]) * 0.5f;
    modal[2] = (fins_deg[0] + fins_deg[1] + fins_deg[2] + fins_deg[3]) * 0.25f;
}

void slx_aero_predict_moments(const float fins_deg[4], float q_ratio,
                              float moments_nm[3])
{
    float modal[3];
    float q = slx_aero_clamp_q(q_ratio);

    if (fins_deg == NULL || moments_nm == NULL) {
        return;
    }
    modal_from_fins(fins_deg, modal);
    moments_nm[0] = q * (SLX_AERO_TY_PITCH * modal[0] +
                         SLX_AERO_TY_YAW * modal[1] +
                         SLX_AERO_TY_ROLL * modal[2]);
    moments_nm[1] = q * (SLX_AERO_TZ_PITCH * modal[0] +
                         SLX_AERO_TZ_YAW * modal[1] +
                         SLX_AERO_TZ_ROLL * modal[2]);
    moments_nm[2] = q * (SLX_AERO_TX_PITCH * modal[0] +
                         SLX_AERO_TX_YAW * modal[1] +
                         SLX_AERO_TX_ROLL * modal[2]);
}

float slx_aero_predict_drag_cd(const float fins_deg[4])
{
    float sin2;
    float deg;

    if (fins_deg == NULL) {
        return 0.0f;
    }
    sin2 = 0.0f;
    for (int i = 0; i < 4; i++) {
        deg = fins_deg[i] * (float) M_PI / 180.0f;
        sin2 += sinf(deg) * sinf(deg);
    }
    sin2 /= sinf(SLX_MAX_FIN_DEG * (float) M_PI / 180.0f) *
            sinf(SLX_MAX_FIN_DEG * (float) M_PI / 180.0f);
    return SLX_AERO_KD * sin2;
}

/* Gaussian elimination with partial pivoting for the 3x3 aero map. */
static bool solve3(float a[3][4], float solution[3])
{
    for (int col = 0; col < 3; col++) {
        int pivot = col;
        for (int row = col + 1; row < 3; row++) {
            if (fabsf(a[row][col]) > fabsf(a[pivot][col])) {
                pivot = row;
            }
        }
        if (fabsf(a[pivot][col]) < 1e-12f) {
            return false;
        }
        if (pivot != col) {
            for (int j = col; j < 4; j++) {
                float tmp = a[col][j];
                a[col][j] = a[pivot][j];
                a[pivot][j] = tmp;
            }
        }
        for (int row = 0; row < 3; row++) {
            if (row == col) {
                continue;
            }
            float factor = a[row][col] / a[col][col];
            for (int j = col; j < 4; j++) {
                a[row][j] -= factor * a[col][j];
            }
        }
    }
    for (int i = 0; i < 3; i++) {
        solution[i] = a[i][3] / a[i][i];
    }
    return true;
}

bool slx_aero_mix_from_moments(const float moments_nm[3], float q_ratio,
                               float fins_deg[4])
{
    float q = slx_aero_clamp_q(q_ratio);
    float a[3][4];
    float modal[3];

    if (moments_nm == NULL || fins_deg == NULL) {
        return false;
    }
    a[0][0] = q * SLX_AERO_TY_PITCH;
    a[0][1] = q * SLX_AERO_TY_YAW;
    a[0][2] = q * SLX_AERO_TY_ROLL;
    a[0][3] = moments_nm[0];
    a[1][0] = q * SLX_AERO_TZ_PITCH;
    a[1][1] = q * SLX_AERO_TZ_YAW;
    a[1][2] = q * SLX_AERO_TZ_ROLL;
    a[1][3] = moments_nm[1];
    a[2][0] = q * SLX_AERO_TX_PITCH;
    a[2][1] = q * SLX_AERO_TX_YAW;
    a[2][2] = q * SLX_AERO_TX_ROLL;
    a[2][3] = moments_nm[2];

    if (!solve3(a, modal)) {
        memset(fins_deg, 0, 4 * sizeof(fins_deg[0]));
        return false;
    }
    for (int i = 0; i < 3; i++) {
        modal[i] = slx_clampf(modal[i], -SLX_MAX_FIN_DEG, SLX_MAX_FIN_DEG);
    }

    /* Inverse of the modal extraction above; pairs are anti-symmetric and the
     * common component supplies roll without changing pitch/yaw. */
    fins_deg[0] = modal[0] + modal[2];
    fins_deg[1] = modal[1] + modal[2];
    fins_deg[2] = -modal[0] + modal[2];
    fins_deg[3] = -modal[1] + modal[2];
    for (int i = 0; i < 4; i++) {
        fins_deg[i] = slx_clampf(fins_deg[i], -SLX_MAX_FIN_DEG, SLX_MAX_FIN_DEG);
    }
    return true;
}
