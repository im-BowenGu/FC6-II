/*
 * S-LX fin-to-aerodynamics model qualified by the live fin sweep (now driven
 * by Taichi/CUDA, calibrated to the validated coefficients).
 *
 * The guidance loop does not treat fin deflections as abstract control numbers:
 * it uses this model to ask "what moment will these fins actually make at the
 * current dynamic pressure?" before committing to a command. Coefficients are
 * moments per degree of modal fin deflection at Q_REF (26 m/s, 1.225 kg/m^3).
 */
#ifndef SLX_AERO_H
#define SLX_AERO_H

#include <stdbool.h>

#define SLX_AERO_Q_REF_PA        414.05f
#define SLX_AERO_RHO_REF_KGM3    1.225f
#define SLX_AERO_Q_MIN           0.05f
#define SLX_AERO_Q_MAX           8.00f

/* dTy/d(pitch, yaw, roll modal deflection) [N.m/deg at Q_REF]. */
#define SLX_AERO_TY_PITCH        0.00972f
#define SLX_AERO_TY_YAW          0.00005f
#define SLX_AERO_TY_ROLL        -0.00066f
/* dTz/d(pitch, yaw, roll modal deflection) [N.m/deg at Q_REF]. */
#define SLX_AERO_TZ_PITCH        0.00048f
#define SLX_AERO_TZ_YAW          0.00944f
#define SLX_AERO_TZ_ROLL        -0.00042f
/* dTx/d(pitch, yaw, roll modal deflection) [N.m/deg at Q_REF]. */
#define SLX_AERO_TX_PITCH        0.00008f
#define SLX_AERO_TX_YAW          0.00003f
#define SLX_AERO_TX_ROLL        -0.00376f

/* CFD quadratic drag increment at 15 deg modal deflection. */
#define SLX_AERO_KD              0.8439f
#define SLX_AERO_CD0             1.01068f

float slx_aero_q_ratio(float speed_mps, float altitude_m);
float slx_aero_clamp_q(float q_ratio);
void slx_aero_predict_moments(const float fins_deg[4], float q_ratio,
                              float moments_nm[3]);
float slx_aero_predict_drag_cd(const float fins_deg[4]);
bool slx_aero_mix_from_moments(const float moments_nm[3], float q_ratio,
                               float fins_deg[4]);

#endif /* SLX_AERO_H */
