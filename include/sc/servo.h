/* 舵机模型：角度 -> PWM，含中值、非线性标定、限幅、斜率限制
 *
 * 标定方法：
 *   1) 找到车轮正前方时舵机 PWM（center）
 *   2) 分别记录左/右打满时的 PWM，拟合二次曲线
 *   pwm = center + k1*delta + k2*delta^2
 */
#ifndef SC_SERVO_H
#define SC_SERVO_H

#include "sc/common.h"

typedef struct {
  float pwm_center; /* 中值 PWM（单位随意，us 或 0..1 占空比） */
  float k1;         /* 一次项：pwm / rad */
  float k2;         /* 二次项：pwm / rad^2，用于补偿连杆非线性 */
  float pwm_min;
  float pwm_max;
  float trim;     /* 安装偏差（rad），正值表示机械中值偏左 */
  float max_rate; /* 舵机最大角速度 rad/s，用于斜率限制 */

  float last_angle;
  int init;
} sc_servo_t;

void sc_servo_init(sc_servo_t *s, float pwm_center, float k1, float k2,
                   float pwm_min, float pwm_max, float max_rate);

void sc_servo_set_trim(sc_servo_t *s, float trim);

/* 纯映射，不做斜率限制 */
float sc_servo_pwm_from_angle(const sc_servo_t *s, float delta);

/* 带斜率限制的更新，返回本周期应输出的 PWM */
float sc_servo_update(sc_servo_t *s, float delta_cmd, float dt);

void sc_servo_reset(sc_servo_t *s, float delta);

#endif
