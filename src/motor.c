#include "sc/motor.h"

#include "sc/math_utils.h"

void sc_motor_init(sc_motor_t *m, float deadzone, float max_duty, float slew,
                   float battery_nominal) {
  memset(m, 0, sizeof(*m));
  m->deadzone = sc_clampf(deadzone, 0.0f, 0.45f);
  m->max_duty = sc_clampf(max_duty, 0.05f, 1.0f);
  m->slew = (slew > 0.0f) ? slew : 5.0f;
  m->battery_nominal = (battery_nominal > 1.0f) ? battery_nominal : 7.4f;
  m->battery_now = m->battery_nominal;
  m->init = 0;
}

void sc_motor_set_battery(sc_motor_t *m, float voltage) {
  if (voltage > 1.0f) {
    m->battery_now = voltage;
  }
}

void sc_motor_reset(sc_motor_t *m, float duty) {
  m->last_duty = duty;
  m->init = 1;
}

float sc_motor_update(sc_motor_t *m, float cmd, float dt) {
  cmd = sc_clampf(cmd, -1.0f, 1.0f);

  /* 死区补偿：越过死区后线性映射到剩余行程 */
  float span = m->max_duty - m->deadzone;
  float duty;

  if (cmd > 0.0f) {
    duty = m->deadzone + cmd * span;
  } else if (cmd < 0.0f) {
    duty = -m->deadzone + cmd * span;
  } else {
    duty = 0.0f;
  }

  /* 电池掉压补偿：电压低时给更大的占空比 */
  if (m->battery_now > 1.0f) {
    duty *= m->battery_nominal / m->battery_now;
  }

  duty = sc_clampf(duty, -m->max_duty, m->max_duty);

  if (!m->init) {
    m->last_duty = duty;
    m->init = 1;
    return duty;
  }

  m->last_duty = sc_slew_limit(m->last_duty, duty, m->slew, dt);
  return m->last_duty;
}
