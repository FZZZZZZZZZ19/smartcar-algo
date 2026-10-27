#include "sc/servo.h"

#include "sc/math_utils.h"

void sc_servo_init(sc_servo_t *s, float pwm_center, float k1, float k2,
                   float pwm_min, float pwm_max, float max_rate) {
  memset(s, 0, sizeof(*s));
  s->pwm_center = pwm_center;
  s->k1 = k1;
  s->k2 = k2;
  s->pwm_min = pwm_min;
  s->pwm_max = pwm_max;
  s->max_rate = (max_rate > 0.0f) ? max_rate : 10.0f;
  s->init = 0;
}

void sc_servo_set_trim(sc_servo_t *s, float trim) { s->trim = trim; }

float sc_servo_pwm_from_angle(const sc_servo_t *s, float delta) {
  float a = delta + s->trim;
  float pwm = s->pwm_center + s->k1 * a + s->k2 * a * a;
  return sc_clampf(pwm, s->pwm_min, s->pwm_max);
}

float sc_servo_update(sc_servo_t *s, float delta_cmd, float dt) {
  if (!s->init) {
    s->last_angle = delta_cmd;
    s->init = 1;
  }

  /* 先限斜率，再映射 PWM：避免舵机被阶跃指令打坏 */
  s->last_angle = sc_slew_limit(s->last_angle, delta_cmd, s->max_rate, dt);
  return sc_servo_pwm_from_angle(s, s->last_angle);
}

void sc_servo_reset(sc_servo_t *s, float delta) {
  s->last_angle = delta;
  s->init = 1;
}
