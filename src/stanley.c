#include "sc/stanley.h"

#include "sc/math_utils.h"

void sc_stanley_init(sc_stanley_t *s, float k_gain, float soft_speed,
                     float max_steer) {
  s->k_gain = (k_gain > 0.0f) ? k_gain : 1.0f;
  s->soft_speed = (soft_speed > 0.0f) ? soft_speed : 0.5f;
  s->max_steer = max_steer;
  s->wheelbase = 0.0f;
}

void sc_stanley_set_wheelbase(sc_stanley_t *s, float wheelbase) {
  s->wheelbase = wheelbase;
}

float sc_stanley_steer(const sc_stanley_t *s, float v, float e_y, float e_psi) {
  float speed = fabsf(v) + s->soft_speed;

  /* 车在左侧(e_y>0)要右打，航向偏左(e_psi>0)也要右打，所以两项都取负 */
  float delta = -e_psi - atanf(s->k_gain * e_y / speed);

  return sc_clampf(delta, -s->max_steer, s->max_steer);
}

float sc_stanley_steer_ff(const sc_stanley_t *s, float v, float e_y,
                          float e_psi, float kappa) {
  float delta = sc_stanley_steer(s, v, e_y, e_psi);

  if (s->wheelbase > 1e-6f) {
    delta += atanf(s->wheelbase * kappa);
  }

  return sc_clampf(delta, -s->max_steer, s->max_steer);
}
