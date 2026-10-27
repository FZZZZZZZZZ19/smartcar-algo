/* PID 家族：位置式、增量式、积分分离、抗饱和、不完全微分、微分先行
 *
 * 单位约定：ki、kd 都按"秒"计，内部乘 dt，所以换采样率不用改参数。
 *   u = kp*e + ki*∫e dt + kd*de/dt
 */
#ifndef SC_PID_H
#define SC_PID_H

#include "sc/common.h"

typedef enum {
  SC_PID_POSITION = 0,   /* 位置式：直接算输出 */
  SC_PID_INCREMENTAL = 1 /* 增量式：算输出增量 */
} sc_pid_form_t;

typedef struct {
  float kp;
  float ki;
  float kd;

  float out_min;
  float out_max;
  float integ_min;
  float integ_max;

  float deadband;     /* 误差死区，|e|<=db 时按 0 处理，0=关闭 */
  float d_alpha;      /* 微分低通系数，1=不滤波，越小越平滑 */
  float integral_sep; /* 积分分离阈值，<=0 关闭 */

  uint8_t derivative_on_measurement; /* 1=微分先行（对测量值微分，避免设定值跳变冲击）
                                      */
  uint8_t form;

  /* 内部状态 */
  float integ;
  float e1;
  float e2;
  float meas1;
  float d_filt;
  float out;
  int init;
} sc_pid_t;

void sc_pid_init(sc_pid_t *p, float kp, float ki, float kd);
void sc_pid_set_gains(sc_pid_t *p, float kp, float ki, float kd);
void sc_pid_set_limits(sc_pid_t *p, float out_min, float out_max);
void sc_pid_set_integral_limits(sc_pid_t *p, float min, float max);
void sc_pid_set_deadband(sc_pid_t *p, float db);
void sc_pid_set_derivative_filter(sc_pid_t *p, float alpha);
void sc_pid_set_integral_separation(sc_pid_t *p, float e_thresh);
void sc_pid_set_derivative_on_measurement(sc_pid_t *p, int enable);
void sc_pid_set_form(sc_pid_t *p, sc_pid_form_t form);
void sc_pid_reset(sc_pid_t *p);

/* 更新一次，返回控制量。dt 为本次采样间隔（秒）。 */
float sc_pid_update(sc_pid_t *p, float setpoint, float measurement, float dt);

/* 带前馈的更新：u = PID + ff，仍然受输出限幅约束。
 * 速度环常用：ff 来自目标速度/加速度的前馈。 */
float sc_pid_update_ff(sc_pid_t *p, float setpoint, float measurement, float dt,
                       float ff);

#endif
