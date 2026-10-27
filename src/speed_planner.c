#include "sc/speed_planner.h"

#include "sc/math_utils.h"

float sc_speed_limit_from_curvature(float kappa, float a_lat_max, float v_max) {
  float k = fabsf(kappa);
  if (k < 1e-6f) {
    return v_max;
  }
  float v = sqrtf(a_lat_max / k);
  return fminf(v, v_max);
}

void sc_speed_profile_build(const float *kappa, int n, float ds, float v_max,
                            float a_lat, float a_acc, float a_dec, int closed,
                            float *v_out, int passes) {
  if (n <= 0 || ds <= 0.0f) {
    return;
  }

  for (int i = 0; i < n; i++) {
    v_out[i] = sc_speed_limit_from_curvature(kappa[i], a_lat, v_max);
  }

  if (passes <= 0) {
    passes = closed ? 3 : 1;
  }

  for (int p = 0; p < passes; p++) {
    /* 反向：从后往前，保证能刹得住 */
    for (int i = n - 2; i >= 0; i--) {
      float limit = sqrtf(v_out[i + 1] * v_out[i + 1] + 2.0f * a_dec * ds);
      if (v_out[i] > limit) {
        v_out[i] = limit;
      }
    }
    if (closed && n > 1) {
      float limit = sqrtf(v_out[0] * v_out[0] + 2.0f * a_dec * ds);
      if (v_out[n - 1] > limit) {
        v_out[n - 1] = limit;
      }
    }

    /* 正向：从前往后，保证加速能力够 */
    for (int i = 1; i < n; i++) {
      float limit = sqrtf(v_out[i - 1] * v_out[i - 1] + 2.0f * a_acc * ds);
      if (v_out[i] > limit) {
        v_out[i] = limit;
      }
    }
    if (closed && n > 1) {
      float limit = sqrtf(v_out[n - 1] * v_out[n - 1] + 2.0f * a_acc * ds);
      if (v_out[0] > limit) {
        v_out[0] = limit;
      }
    }
  }

  for (int i = 0; i < n; i++) {
    if (v_out[i] < 0.0f) {
      v_out[i] = 0.0f;
    }
  }
}

void sc_trap_plan(sc_trap_t *t, float distance, float v0, float v1, float v_max,
                  float a_acc, float a_dec) {
  memset(t, 0, sizeof(*t));
  t->distance = distance;

  if (a_acc <= 0.0f) {
    a_acc = 1.0f;
  }
  if (a_dec <= 0.0f) {
    a_dec = 1.0f;
  }
  if (distance <= 0.0f) {
    t->v_peak = fmaxf(v0, v1);
    t->feasible = 0;
    return;
  }

  /* 先假设能达到 v_max，看距离够不够 */
  float v_peak = v_max;

  float s_acc = 0.0f;
  if (v_peak > v0) {
    s_acc = (v_peak * v_peak - v0 * v0) / (2.0f * a_acc);
  }
  float s_dec = 0.0f;
  if (v_peak > v1) {
    s_dec = (v_peak * v_peak - v1 * v1) / (2.0f * a_dec);
  }

  if (s_acc + s_dec > distance) {
    /* 距离不够：解方程求实际峰值速度 */
    float inv_sum = 1.0f / a_acc + 1.0f / a_dec;
    float v2 = (2.0f * distance + v0 * v0 / a_acc + v1 * v1 / a_dec) / inv_sum;

    if (v2 < 0.0f) {
      v2 = 0.0f;
    }
    v_peak = sqrtf(v2);

    if (v_peak > v_max) {
      v_peak = v_max;
    }

    s_acc = (v_peak > v0) ? (v_peak * v_peak - v0 * v0) / (2.0f * a_acc) : 0.0f;
    s_dec = (v_peak > v1) ? (v_peak * v_peak - v1 * v1) / (2.0f * a_dec) : 0.0f;

    t->feasible = (s_acc + s_dec <= distance + 1e-4f) ? 1 : 0;
  } else {
    t->feasible = 1;
  }

  t->v_peak = v_peak;
  t->s_acc = s_acc;
  t->s_dec = s_dec;

  float s_cruise = distance - s_acc - s_dec;
  t->s_cruise = (s_cruise > 0.0f) ? s_cruise : 0.0f;
}

float sc_trap_speed_at_distance(const sc_trap_t *t, float s, float v0, float v1,
                                float a_acc, float a_dec) {
  if (a_acc <= 0.0f) {
    a_acc = 1.0f;
  }
  if (a_dec <= 0.0f) {
    a_dec = 1.0f;
  }

  s = sc_clampf(s, 0.0f, t->distance);

  if (s < t->s_acc) {
    float v2 = v0 * v0 + 2.0f * a_acc * s;
    return (v2 > 0.0f) ? sqrtf(v2) : 0.0f;
  }

  if (s < t->s_acc + t->s_cruise) {
    return t->v_peak;
  }

  float v2 = v1 * v1 + 2.0f * a_dec * (t->distance - s);
  return (v2 > 0.0f) ? sqrtf(v2) : 0.0f;
}

void sc_scurve_init(sc_scurve_t *s, float a_max, float j_max, float v0) {
  memset(s, 0, sizeof(*s));
  s->a_max = (a_max > 0.0f) ? a_max : 1.0f;
  s->j_max = (j_max > 0.0f) ? j_max : 10.0f;
  s->v = v0;
  s->a = 0.0f;
  s->init = 1;
}

void sc_scurve_reset(sc_scurve_t *s, float v0) {
  s->v = v0;
  s->a = 0.0f;
}

float sc_scurve_step(sc_scurve_t *s, float v_target, float dt) {
  if (dt <= 0.0f) {
    return s->v;
  }
  if (!s->init) {
    sc_scurve_init(s, s->a_max, s->j_max, s->v);
  }

  /* 期望加速度：比例逼近，限制在 ±a_max */
  float a_des = sc_clampf(3.0f * (v_target - s->v), -s->a_max, s->a_max);

  /* 加速度变化率受 j_max 限制 → 速度曲线呈 S 形 */
  s->a = sc_move_towards(s->a, a_des, s->j_max * dt);

  s->v += s->a * dt;
  if (s->v < 0.0f) {
    s->v = 0.0f;
    s->a = 0.0f;
  }

  return s->v;
}
