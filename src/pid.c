#include "sc/pid.h"

#include "sc/math_utils.h"

void sc_pid_init(sc_pid_t *p, float kp, float ki, float kd) {
  memset(p, 0, sizeof(*p));
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
  p->out_min = -1.0e9f;
  p->out_max = 1.0e9f;
  p->integ_min = -1.0e9f;
  p->integ_max = 1.0e9f;
  p->d_alpha = 1.0f;
  p->deadband = 0.0f;
  p->integral_sep = 0.0f;
  p->form = SC_PID_POSITION;
  p->init = 0;
}

void sc_pid_set_gains(sc_pid_t *p, float kp, float ki, float kd) {
  p->kp = kp;
  p->ki = ki;
  p->kd = kd;
}

void sc_pid_set_limits(sc_pid_t *p, float out_min, float out_max) {
  p->out_min = out_min;
  p->out_max = out_max;
}

void sc_pid_set_integral_limits(sc_pid_t *p, float min, float max) {
  p->integ_min = min;
  p->integ_max = max;
}

void sc_pid_set_deadband(sc_pid_t *p, float db) {
  p->deadband = (db > 0.0f) ? db : 0.0f;
}

void sc_pid_set_derivative_filter(sc_pid_t *p, float alpha) {
  p->d_alpha = sc_clampf(alpha, 0.0f, 1.0f);
}

void sc_pid_set_integral_separation(sc_pid_t *p, float e_thresh) {
  p->integral_sep = (e_thresh > 0.0f) ? e_thresh : 0.0f;
}

void sc_pid_set_derivative_on_measurement(sc_pid_t *p, int enable) {
  p->derivative_on_measurement = enable ? 1 : 0;
}

void sc_pid_set_form(sc_pid_t *p, sc_pid_form_t form) {
  if (p->form != (uint8_t)form) {
    p->form = (uint8_t)form;
    sc_pid_reset(p);
  }
}

void sc_pid_reset(sc_pid_t *p) {
  p->integ = 0.0f;
  p->e1 = 0.0f;
  p->e2 = 0.0f;
  p->meas1 = 0.0f;
  p->d_filt = 0.0f;
  p->out = 0.0f;
  p->init = 0;
}

static float pid_position_update(sc_pid_t *p, float e, float meas, float dt) {
  float p_term = p->kp * e;

  /* 积分分离：误差很大时先靠 P 把系统拉回来，避免积分爆掉 */
  int allow_integral = 1;
  if (p->integral_sep > 0.0f && fabsf(e) > p->integral_sep) {
    allow_integral = 0;
  }

  float integ_new = p->integ;
  if (allow_integral) {
    integ_new += p->ki * e * dt;
    integ_new = sc_clampf(integ_new, p->integ_min, p->integ_max);
  }

  /* 微分：默认对误差微分；微分先行时对测量值微分。
   * 微分先行可以避免设定值阶跃时产生巨大的微分尖峰。 */
  float d_raw;
  if (p->derivative_on_measurement) {
    d_raw = -p->kd * (meas - p->meas1) / dt;
  } else {
    d_raw = p->kd * (e - p->e1) / dt;
  }

  p->d_filt = p->d_alpha * p->d_filt + (1.0f - p->d_alpha) * d_raw;

  float u = p_term + integ_new + p->d_filt;

  int sat_hi = (u > p->out_max);
  int sat_lo = (u < p->out_min);
  if (sat_hi) {
    u = p->out_max;
  }
  if (sat_lo) {
    u = p->out_min;
  }

  /* 条件积分抗饱和：输出已经顶到限幅，且误差还在往同方向推，
   * 这次积分就撤销，防止退饱和时"回绕"超调。 */
  if ((sat_hi && e > 0.0f) || (sat_lo && e < 0.0f)) {
    integ_new = p->integ;
  }
  p->integ = sc_clampf(integ_new, p->integ_min, p->integ_max);

  return u;
}

static float pid_incremental_update(sc_pid_t *p, float e, float dt) {
  /* Δu = kp*(e-e1) + ki*e*dt + kd*(e-2e1+e2)/dt */
  float du = p->kp * (e - p->e1) + p->ki * e * dt +
             p->kd * (e - 2.0f * p->e1 + p->e2) / dt;

  float u = p->out + du;
  u = sc_clampf(u, p->out_min, p->out_max);

  /* 增量式没有显式积分器，但输出本身会饱和；
   * 饱和时把增量清零，效果等价于抗积分饱和。 */
  return u;
}

float sc_pid_update(sc_pid_t *p, float setpoint, float measurement, float dt) {
  if (dt <= 0.0f) {
    dt = 1e-3f;
  }

  float e = setpoint - measurement;

  if (!p->init) {
    /* 增量式第一次更新要让 P 项产生阶跃，e1/e2 从 0 开始；
     * 位置式从当前误差开始，避免设定值突变带来的微分冲击。 */
    p->e1 = (p->form == SC_PID_INCREMENTAL) ? 0.0f : e;
    p->e2 = p->e1;
    p->meas1 = measurement;
    p->d_filt = 0.0f;
    p->init = 1;
  }

  /* 死区：小误差不动作，抑制舵机/电机抖动 */
  if (p->deadband > 0.0f && fabsf(e) <= p->deadband) {
    e = 0.0f;
  }

  float u;

  if (p->form == SC_PID_INCREMENTAL) {
    u = pid_incremental_update(p, e, dt);
    p->out = u;
  } else {
    u = pid_position_update(p, e, measurement, dt);
    p->out = u;
  }

  p->e2 = p->e1;
  p->e1 = e;
  p->meas1 = measurement;

  return u;
}

float sc_pid_update_ff(sc_pid_t *p, float setpoint, float measurement, float dt,
                       float ff) {
  float u = sc_pid_update(p, setpoint, measurement, dt) + ff;
  u = sc_clampf(u, p->out_min, p->out_max);
  p->out = u;
  return u;
}
