#include "sc/pure_pursuit.h"

#include "sc/math_utils.h"

void sc_pp_init(sc_pure_pursuit_t *pp, float wheelbase, float ld_base,
                float ld_gain, float max_steer) {
  pp->wheelbase = wheelbase;
  pp->ld_base = ld_base;
  pp->ld_gain = ld_gain;
  pp->ld_min = 0.15f;
  pp->ld_max = 3.0f;
  pp->max_steer = max_steer;
}

float sc_pp_lookahead(const sc_pure_pursuit_t *pp, float v) {
  float ld = pp->ld_base + pp->ld_gain * fabsf(v);
  return sc_clampf(ld, pp->ld_min, pp->ld_max);
}

float sc_pp_steer_alpha(const sc_pure_pursuit_t *pp, float alpha, float ld) {
  if (ld < 0.05f) {
    ld = 0.05f;
  }

  /* 纯跟踪的几何解：R = ld / (2 sin(alpha))，delta = atan(L/R) */
  float delta = atan2f(2.0f * pp->wheelbase * sinf(alpha), ld);
  return sc_clampf(delta, -pp->max_steer, pp->max_steer);
}

float sc_pp_steer(const sc_pure_pursuit_t *pp, const sc_pose2_t *pose,
                  float target_x, float target_y) {
  float dx = target_x - pose->x;
  float dy = target_y - pose->y;
  float ld = sqrtf(dx * dx + dy * dy);

  float alpha = sc_wrap_pi(atan2f(dy, dx) - pose->theta);
  return sc_pp_steer_alpha(pp, alpha, ld);
}
