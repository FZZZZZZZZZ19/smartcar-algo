#include "sc/math_utils.h"

#include <float.h>

float sc_wrap_pi(float a) {
  /* 用 fmodf 做长角度也能收敛的归一化，避免大角度时反复加减 */
  a = fmodf(a + SC_PI, SC_TWO_PI);
  if (a < 0.0f) {
    a += SC_TWO_PI;
  }
  return a - SC_PI;
}

float sc_wrap_2pi(float a) {
  a = fmodf(a, SC_TWO_PI);
  if (a < 0.0f) {
    a += SC_TWO_PI;
  }
  return a;
}

float sc_angle_diff(float a, float b) { return sc_wrap_pi(a - b); }

float sc_deadband(float x, float db) {
  if (db <= 0.0f) {
    return x;
  }
  if (x > db) {
    return x - db;
  }
  if (x < -db) {
    return x + db;
  }
  return 0.0f;
}

float sc_lpf_alpha_tau(float tau, float dt) {
  if (tau <= 0.0f || dt <= 0.0f) {
    return 0.0f; /* alpha=0 表示完全跟随新值 */
  }
  return tau / (tau + dt);
}

float sc_lpf_alpha_fc(float fc, float dt) {
  if (fc <= 0.0f || dt <= 0.0f) {
    return 0.0f;
  }
  /* 精确的双线性/指数离散：alpha = exp(-2*pi*fc*dt) */
  return expf(-SC_TWO_PI * fc * dt);
}

float sc_slew_limit(float current, float target, float max_rate, float dt) {
  if (max_rate <= 0.0f || dt <= 0.0f) {
    return target;
  }
  return sc_move_towards(current, target, max_rate * dt);
}

float sc_move_towards(float current, float target, float max_step) {
  float d = target - current;
  if (fabsf(d) <= max_step) {
    return target;
  }
  return current + (d > 0.0f ? max_step : -max_step);
}

float sc_lerp(float a, float b, float t) { return a + (b - a) * t; }

float sc_smoothstep(float t) {
  t = sc_clampf(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

int sc_almost_equal(float a, float b, float tol) { return fabsf(a - b) <= tol; }

float sc_norm01(float x, float lo, float hi) {
  float d = hi - lo;
  if (fabsf(d) < SC_EPS) {
    return 0.5f;
  }
  return sc_clampf((x - lo) / d, 0.0f, 1.0f);
}

float sc_curvature_3pt(float x0, float y0, float x1, float y1, float x2,
                       float y2) {
  /* 有向曲率 kappa = 2 * cross(v1, v2) / (|v1||v2||v3|)
   * 半径 r = 1/kappa。逆时针弯曲时 kappa > 0。 */
  float ax = x1 - x0, ay = y1 - y0;
  float bx = x2 - x0, by = y2 - y0;
  float cross = ax * by - ay * bx;

  float la = sqrtf(ax * ax + ay * ay);
  float lb = sqrtf(bx * bx + by * by);
  float lc = sqrtf((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  float denom = la * lb * lc;

  if (denom < 1e-9f) {
    return FLT_MAX;
  }

  float kappa = 2.0f * cross / denom;
  if (fabsf(kappa) < 1e-9f) {
    return FLT_MAX; /* 直线 */
  }
  return 1.0f / kappa;
}
