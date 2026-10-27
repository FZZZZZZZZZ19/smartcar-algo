#include "sc/lqr.h"

#include "sc/kinematics.h"
#include "sc/math_utils.h"

int sc_lqr_track_init(sc_lqr_track_t *lqr, float wheelbase, float v, float dt,
                      float q_y, float q_psi, float r_delta, float max_steer) {
  memset(lqr, 0, sizeof(*lqr));
  lqr->wheelbase = wheelbase;
  lqr->v = v;
  lqr->dt = (dt > 0.0f) ? dt : 0.005f;
  lqr->q_y = q_y;
  lqr->q_psi = q_psi;
  lqr->r_delta = r_delta;
  lqr->max_steer = max_steer;

  return sc_lqr_track_rebuild(lqr, v);
}

int sc_lqr_track_rebuild(sc_lqr_track_t *lqr, float v) {
  float L = lqr->wheelbase;
  float dt = lqr->dt;

  lqr->v = v;

  /* 连续模型 A = [0 v; 0 0], B = [0; v/L] */
  sc_mat_t A, B, Q, R, K;
  sc_mat_zero(&A, 2, 2);
  sc_mat_zero(&B, 2, 1);
  sc_mat_zero(&Q, 2, 2);
  sc_mat_zero(&R, 1, 1);

  /* 前向欧拉离散：Ad = I + A dt, Bd = B dt */
  A.a[0][0] = 1.0f;
  A.a[0][1] = v * dt;
  A.a[1][0] = 0.0f;
  A.a[1][1] = 1.0f;

  B.a[1][0] = v * dt / L;

  Q.a[0][0] = lqr->q_y;
  Q.a[1][1] = lqr->q_psi;
  R.a[0][0] = lqr->r_delta;

  int iter = sc_lqr_discrete(&A, &B, &Q, &R, &K, 5000, 1e-7f);
  if (iter < 0) {
    return -1;
  }

  lqr->K = K;
  lqr->k_y = K.a[0][0];
  lqr->k_psi = K.a[0][1];
  lqr->iterations = iter;

  return 0;
}

float sc_lqr_track_update(const sc_lqr_track_t *lqr, float e_y, float e_psi,
                          float kappa_ref) {
  /* 前馈：直接用曲率对应的转角抵消路径弯曲 */
  float ff = atanf(lqr->wheelbase * kappa_ref);

  float delta = -lqr->k_y * e_y - lqr->k_psi * e_psi + ff;
  return sc_clampf(delta, -lqr->max_steer, lqr->max_steer);
}

int sc_lqr_balance(float cart_mass, float pole_mass, float pole_half,
                   float q_pos, float q_vel, float q_angle, float q_angle_rate,
                   float r_force, float K[4]) {
  sc_balance_t b;
  b.cart_mass = cart_mass;
  b.pole_mass = pole_mass;
  b.pole_half = pole_half;
  b.g = 9.81f;

  float A4[4][4];
  float B4[4];
  sc_balance_state_space(&b, A4, B4);

  /* 平衡车一般用 5~10 ms 的控制周期 */
  float dt = 0.005f;

  sc_mat_t A, B, Q, R, Kmat;
  sc_mat_zero(&A, 4, 4);
  sc_mat_zero(&B, 4, 1);
  sc_mat_zero(&Q, 4, 4);
  sc_mat_zero(&R, 1, 1);

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      A.a[i][j] = (i == j ? 1.0f : 0.0f) + A4[i][j] * dt;
    }
    B.a[i][0] = B4[i] * dt;
  }

  Q.a[0][0] = q_pos;
  Q.a[1][1] = q_vel;
  Q.a[2][2] = q_angle;
  Q.a[3][3] = q_angle_rate;
  R.a[0][0] = r_force;

  int iter = sc_lqr_discrete(&A, &B, &Q, &R, &Kmat, 5000, 1e-7f);
  if (iter < 0) {
    return -1;
  }

  for (int i = 0; i < 4; i++) {
    K[i] = Kmat.a[0][i];
  }
  return 0;
}
