#include "sc/attitude.h"

#include "sc/math_utils.h"

/* ============================================================
 * 加速度倾角
 * ============================================================ */

float sc_accel_pitch(float ax, float ay, float az) {
  /* 车头抬起时 ax 的分量变化：pitch = atan2(-ax, sqrt(ay²+az²)) */
  return atan2f(-ax, sqrtf(ay * ay + az * az));
}

float sc_accel_roll(float ax, float ay, float az) {
  return atan2f(ay, sqrtf(ax * ax + az * az));
}

/* ============================================================
 * 一阶互补滤波
 * ============================================================ */

void sc_comp_init(sc_comp_t *f, float angle, float alpha) {
  f->angle = angle;
  f->alpha = sc_clampf(alpha, 0.0f, 0.999f);
}

void sc_comp_reset(sc_comp_t *f, float angle) { f->angle = angle; }

float sc_comp_update(sc_comp_t *f, float gyro_rate, float acc_angle, float dt) {
  /* 陀螺积分负责短期动态，加速度计负责长期不漂 */
  float predicted = f->angle + gyro_rate * dt;
  f->angle = f->alpha * predicted + (1.0f - f->alpha) * acc_angle;
  return f->angle;
}

/* ============================================================
 * 四元数
 * ============================================================ */

void sc_quat_normalize(float q[4]) {
  float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (n < 1e-8f) {
    q[0] = 1.0f;
    q[1] = q[2] = q[3] = 0.0f;
    return;
  }
  float inv = 1.0f / n;
  for (int i = 0; i < 4; i++) {
    q[i] *= inv;
  }
}

void sc_quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw) {
  float w = q[0], x = q[1], y = q[2], z = q[3];

  *roll = atan2f(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
  *pitch = asinf(sc_clampf(2.0f * (w * y - z * x), -1.0f, 1.0f));
  *yaw = atan2f(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void sc_quat_to_rot(const float q[4], float R[3][3]) {
  float w = q[0], x = q[1], y = q[2], z = q[3];

  R[0][0] = 1.0f - 2.0f * (y * y + z * z);
  R[0][1] = 2.0f * (x * y - w * z);
  R[0][2] = 2.0f * (x * z + w * y);
  R[1][0] = 2.0f * (x * y + w * z);
  R[1][1] = 1.0f - 2.0f * (x * x + z * z);
  R[1][2] = 2.0f * (y * z - w * x);
  R[2][0] = 2.0f * (x * z - w * y);
  R[2][1] = 2.0f * (y * z + w * x);
  R[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

/* ============================================================
 * Mahony
 * ============================================================ */

void sc_mahony_init(sc_mahony_t *m, float kp, float ki) {
  memset(m, 0, sizeof(*m));
  m->q[0] = 1.0f;
  m->kp = kp;
  m->ki = ki;
  m->init = 1;
}

void sc_mahony_reset(sc_mahony_t *m, float roll, float pitch, float yaw) {
  float cr = cosf(0.5f * roll);
  float sr = sinf(0.5f * roll);
  float cp = cosf(0.5f * pitch);
  float sp = sinf(0.5f * pitch);
  float cy = cosf(0.5f * yaw);
  float sy = sinf(0.5f * yaw);

  /* ZYX 欧拉角 -> 四元数 */
  m->q[0] = cr * cp * cy + sr * sp * sy;
  m->q[1] = sr * cp * cy - cr * sp * sy;
  m->q[2] = cr * sp * cy + sr * cp * sy;
  m->q[3] = cr * cp * sy - sr * sp * cy;

  memset(m->integral, 0, sizeof(m->integral));
}

void sc_mahony_update(sc_mahony_t *m, float gx, float gy, float gz, float ax,
                      float ay, float az, float dt) {
  if (!m->init) {
    sc_mahony_init(m, 0.5f, 0.0f);
  }

  float an = sqrtf(ax * ax + ay * ay + az * az);
  if (an < 1e-6f || dt <= 0.0f) {
    /* 加速度无效（自由落体/剧烈振动），只积分陀螺 */
    float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];
    m->q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
    m->q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
    m->q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
    m->q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;
    sc_quat_normalize(m->q);
    return;
  }

  ax /= an;
  ay /= an;
  az /= an;

  float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];

  /* 当前四元数估计出的重力方向（机体系） */
  float vx = 2.0f * (q1 * q3 - q0 * q2);
  float vy = 2.0f * (q0 * q1 + q2 * q3);
  float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

  /* 测量重力与估计重力的叉积 = 姿态误差 */
  float ex = ay * vz - az * vy;
  float ey = az * vx - ax * vz;
  float ez = ax * vy - ay * vx;

  if (m->ki > 0.0f) {
    m->integral[0] += m->ki * ex * dt;
    m->integral[1] += m->ki * ey * dt;
    m->integral[2] += m->ki * ez * dt;
  }

  gx += m->kp * ex + m->integral[0];
  gy += m->kp * ey + m->integral[1];
  gz += m->kp * ez + m->integral[2];

  m->q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * dt;
  m->q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
  m->q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
  m->q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;

  sc_quat_normalize(m->q);
}

void sc_mahony_euler(const sc_mahony_t *m, float *roll, float *pitch,
                     float *yaw) {
  sc_quat_to_euler(m->q, roll, pitch, yaw);
}

/* ============================================================
 * 陀螺零漂校准
 * ============================================================ */

void sc_gyro_cal_init(sc_gyro_cal_t *c, int samples) {
  memset(c, 0, sizeof(*c));
  c->target = (samples > 0) ? samples : 500;
}

int sc_gyro_cal_feed(sc_gyro_cal_t *c, float gx, float gy, float gz,
                     float acc_norm, float g_ref) {
  if (c->done) {
    return 1;
  }

  /* 车必须静止：加速度模长要接近 1g（单位任意，传入 g_ref 即可） */
  if (fabsf(acc_norm - g_ref) > 0.35f * g_ref) {
    c->count = 0;
    c->sum[0] = c->sum[1] = c->sum[2] = 0.0;
    return 0;
  }

  c->sum[0] += gx;
  c->sum[1] += gy;
  c->sum[2] += gz;
  c->count++;

  if (c->count >= c->target) {
    c->bias[0] = (float)(c->sum[0] / c->count);
    c->bias[1] = (float)(c->sum[1] / c->count);
    c->bias[2] = (float)(c->sum[2] / c->count);
    c->done = 1;
    return 1;
  }

  return 0;
}

void sc_gyro_cal_apply(const sc_gyro_cal_t *c, float *gx, float *gy,
                       float *gz) {
  if (!c->done) {
    return;
  }
  *gx -= c->bias[0];
  *gy -= c->bias[1];
  *gz -= c->bias[2];
}
