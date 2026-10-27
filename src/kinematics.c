#include "sc/kinematics.h"

#include "sc/math_utils.h"

/* ============================================================
 * 差速驱动
 * ============================================================ */

void sc_diff_init(sc_diff_drive_t *d, float wheel_radius, float track_width,
                  float gear_ratio, float encoder_ppr) {
  d->wheel_radius = wheel_radius;
  d->track_width = track_width;
  d->gear_ratio = (gear_ratio > 0.0f) ? gear_ratio : 1.0f;
  d->encoder_ppr = encoder_ppr;
}

void sc_diff_forward(const sc_diff_drive_t *d, float v_left, float v_right,
                     float *v, float *w) {
  *v = 0.5f * (v_left + v_right);
  *w = (v_right - v_left) / d->track_width;
}

void sc_diff_inverse(const sc_diff_drive_t *d, float v, float w, float *v_left,
                     float *v_right) {
  float half = 0.5f * w * d->track_width;
  *v_left = v - half;
  *v_right = v + half;
}

float sc_diff_wheel_speed(const sc_diff_drive_t *d, int32_t delta_counts,
                          float dt) {
  if (dt <= 0.0f || d->encoder_ppr <= 0.0f) {
    return 0.0f;
  }
  /* 电机轴转数 = 计数 / PPR；车轮转数 = 电机转数 / 减速比 */
  float motor_rev = (float)delta_counts / d->encoder_ppr;
  float wheel_rev = motor_rev / d->gear_ratio;
  float dist = wheel_rev * SC_TWO_PI * d->wheel_radius;
  return dist / dt;
}

void sc_diff_odometry(sc_pose2_t *pose, const sc_diff_drive_t *d, float v_left,
                      float v_right, float dt) {
  float v = 0.0f;
  float w = 0.0f;
  sc_diff_forward(d, v_left, v_right, &v, &w);

  if (fabsf(w) < 1e-5f) {
    /* 直线：直接平移 */
    pose->x += v * cosf(pose->theta) * dt;
    pose->y += v * sinf(pose->theta) * dt;
  } else {
    /* 圆弧：R = v/w，走过的圆心角 dtheta = w*dt */
    float r = v / w;
    float dtheta = w * dt;
    float theta0 = pose->theta;
    float theta1 = theta0 + dtheta;
    pose->x += r * (sinf(theta1) - sinf(theta0));
    pose->y += -r * (cosf(theta1) - cosf(theta0));
    pose->theta = sc_wrap_pi(theta1);
    return;
  }
  pose->theta = sc_wrap_pi(pose->theta);
}

/* ============================================================
 * 阿克曼 / 自行车模型
 * ============================================================ */

void sc_ackermann_init(sc_ackermann_t *a, float wheelbase, float track_width,
                       float max_steer) {
  a->wheelbase = wheelbase;
  a->track_width = track_width;
  a->max_steer = max_steer;
}

float sc_ackermann_steer_from_curvature(const sc_ackermann_t *a, float kappa) {
  float delta = atanf(a->wheelbase * kappa);
  return sc_clampf(delta, -a->max_steer, a->max_steer);
}

float sc_ackermann_curvature(const sc_ackermann_t *a, float delta) {
  return tanf(delta) / a->wheelbase;
}

void sc_ackermann_wheel_angles(const sc_ackermann_t *a, float delta,
                               float *delta_inner, float *delta_outer) {
  float half_w = 0.5f * a->track_width;

  if (fabsf(delta) < 1e-4f) {
    *delta_inner = delta;
    *delta_outer = delta;
    return;
  }

  /* 用 |delta| 算半径，最后恢复符号，避免左右转对称性写错 */
  float s = (delta > 0.0f) ? 1.0f : -1.0f;
  float d = fabsf(delta);
  float r = a->wheelbase / tanf(d);

  float r_in = r - half_w;
  float r_out = r + half_w;

  /* 半径小于半轮距 = 理论上前轮已经不可能纯滚动，做保护 */
  if (r_in < 0.05f) {
    r_in = 0.05f;
  }

  *delta_inner = s * atanf(a->wheelbase / r_in);
  *delta_outer = s * atanf(a->wheelbase / r_out);
}

float sc_ackermann_steer_from_radius(const sc_ackermann_t *a, float radius) {
  if (fabsf(radius) < 1e-4f) {
    return (radius >= 0.0f) ? a->max_steer : -a->max_steer;
  }
  float delta = atanf(a->wheelbase / radius);
  return sc_clampf(delta, -a->max_steer, a->max_steer);
}

void sc_ackermann_integrate(sc_pose2_t *pose, const sc_ackermann_t *a, float v,
                            float delta, float dt) {
  float w = v * tanf(delta) / a->wheelbase;
  float theta_mid = pose->theta + 0.5f * w * dt;

  pose->x += v * cosf(theta_mid) * dt;
  pose->y += v * sinf(theta_mid) * dt;
  pose->theta = sc_wrap_pi(pose->theta + w * dt);
}

void sc_ackermann_integrate_front(sc_pose2_t *pose, const sc_ackermann_t *a,
                                  float v, float delta, float dt) {
  /* 前轴参考：beta 为车体速度方向相对车头的侧偏角
   *   beta = atan(0.5 * tan(delta))
   *   x_dot = v cos(theta + beta)
   *   y_dot = v sin(theta + beta)
   *   theta_dot = v sin(beta) / (L/2) = v tan(delta)/L
   */
  float beta = atanf(0.5f * tanf(delta));
  float w = v * tanf(delta) / a->wheelbase;
  float theta_mid = pose->theta + 0.5f * w * dt + beta;

  pose->x += v * cosf(theta_mid) * dt;
  pose->y += v * sinf(theta_mid) * dt;
  pose->theta = sc_wrap_pi(pose->theta + w * dt);
}

void sc_ackermann_forward(const sc_ackermann_t *a, float v_left, float v_right,
                          float *v, float *w) {
  /* 左右轮速差 -> 绕后轴中心的角速度（小转角近似） */
  *v = 0.5f * (v_left + v_right);
  *w = (v_right - v_left) / a->track_width;
}

/* ============================================================
 * 麦克纳姆轮
 * ============================================================ */

void sc_mecanum_init(sc_mecanum_t *m, float wheel_radius, float lx, float ly) {
  m->wheel_radius = wheel_radius;
  m->lx = lx;
  m->ly = ly;
}

void sc_mecanum_inverse(const sc_mecanum_t *m, float vx, float vy, float wz,
                        float wheel[4]) {
  float k = (m->lx + m->ly) * wz;
  float r = m->wheel_radius;

  wheel[0] = (vx - vy - k) / r; /* FL */
  wheel[1] = (vx + vy + k) / r; /* FR */
  wheel[2] = (vx + vy - k) / r; /* RL */
  wheel[3] = (vx - vy + k) / r; /* RR */
}

void sc_mecanum_forward(const sc_mecanum_t *m, const float wheel[4], float *vx,
                        float *vy, float *wz) {
  float r = m->wheel_radius;
  float w0 = wheel[0] * r;
  float w1 = wheel[1] * r;
  float w2 = wheel[2] * r;
  float w3 = wheel[3] * r;

  *vx = 0.25f * (w0 + w1 + w2 + w3);
  *vy = 0.25f * (-w0 + w1 + w2 - w3);
  *wz = (-w0 + w1 - w2 + w3) / (4.0f * (m->lx + m->ly));
}

/* ============================================================
 * 三轮全向
 * ============================================================ */

void sc_omni3_init_default(sc_omni3_t *o, float wheel_radius) {
  o->wheel_radius = wheel_radius;
  o->angle[0] = SC_HALF_PI;                    /* 90°  */
  o->angle[1] = SC_HALF_PI + SC_TWO_PI / 3.0f; /* 210° */
  o->angle[2] = SC_HALF_PI - SC_TWO_PI / 3.0f; /* 330° */
}

void sc_omni3_inverse(const sc_omni3_t *o, float vx, float vy, float wz,
                      float wheel[3]) {
  for (int i = 0; i < 3; i++) {
    float s = sinf(o->angle[i]);
    float c = cosf(o->angle[i]);
    float ground = -s * vx + c * vy;
    wheel[i] = (ground + o->wheel_radius * wz) / o->wheel_radius;
  }
}

void sc_omni3_forward(const sc_omni3_t *o, const float wheel[3], float *vx,
                      float *vy, float *wz) {
  float sx = 0.0f, sy = 0.0f, sv = 0.0f;

  for (int i = 0; i < 3; i++) {
    float v = wheel[i] * o->wheel_radius;
    sx += -sinf(o->angle[i]) * v;
    sy += cosf(o->angle[i]) * v;
    sv += v;
  }

  *vx = (2.0f / 3.0f) * sx;
  *vy = (2.0f / 3.0f) * sy;
  *wz = sv / (3.0f * o->wheel_radius);
}

/* ============================================================
 * 平衡车
 * ============================================================ */

void sc_balance_state_space(const sc_balance_t *b, float A[4][4], float B[4]) {
  float M = b->cart_mass;
  float m = b->pole_mass;
  float l = b->pole_half;
  float g = (b->g > 0.0f) ? b->g : 9.81f;

  memset(A, 0, sizeof(float) * 16);
  memset(B, 0, sizeof(float) * 4);

  A[0][1] = 1.0f;
  A[1][2] = -m * g / M;
  A[2][3] = 1.0f;
  A[3][2] = (M + m) * g / (M * l);

  B[1] = 1.0f / M;
  B[3] = -1.0f / (M * l);
}

float sc_balance_force_from_cmd(const sc_balance_t *b, float cmd,
                                float max_force) {
  (void)b;
  return sc_clampf(cmd, -1.0f, 1.0f) * max_force;
}
