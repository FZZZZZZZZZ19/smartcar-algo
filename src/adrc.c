#include "sc/adrc.h"

#include "sc/math_utils.h"

static float signf(float x) {
  return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

/* 最速跟踪函数 fhan，用于 TD */
static float fhan(float x1, float x2, float r, float h) {
  float d = r * h;
  float d0 = d * h;
  float y = x1 + h * x2;
  float a0 = sqrtf(d * d + 8.0f * r * fabsf(y));

  float a;
  if (fabsf(y) > d0) {
    a = x2 + 0.5f * (a0 - d) * signf(y);
  } else {
    a = x2 + y / h;
  }

  if (fabsf(a) > d) {
    return -r * signf(a);
  }
  return -r * a / d;
}

/* 非线性函数 fal：小误差线性、大误差幂次，兼顾平滑与快速 */
static float fal(float e, float a, float delta) {
  if (delta < 1e-6f) {
    delta = 1e-6f;
  }
  if (fabsf(e) > delta) {
    return powf(fabsf(e), a) * signf(e);
  }
  return e / powf(delta, 1.0f - a);
}

void sc_adrc_init(sc_adrc_t *a, float b0, float dt) {
  memset(a, 0, sizeof(*a));

  a->r = 100.0f;
  a->b0 = (fabsf(b0) > 1e-6f) ? b0 : 1.0f;
  a->a1 = 0.75f;
  a->a2 = 1.5f;
  a->delta = (dt > 0.0f) ? dt : 0.005f;
  a->out_min = -1.0e9f;
  a->out_max = 1.0e9f;

  /* 默认 wc=3, wo=8：注意 wo*dt 过大会让显式欧拉 ESO 发散，
   * 经验上 wo*dt <= 0.15 比较稳。 */
  sc_adrc_set_bandwidth(a, 3.0f, 8.0f, a->b0);
  a->init = 1;
}

void sc_adrc_set_bandwidth(sc_adrc_t *a, float wc, float wo, float b0) {
  if (fabsf(b0) > 1e-6f) {
    a->b0 = b0;
  }

  a->kp = wc * wc;
  a->kd = 2.0f * wc;

  a->beta1 = 3.0f * wo;
  a->beta2 = 3.0f * wo * wo;
  a->beta3 = wo * wo * wo;
}

void sc_adrc_reset(sc_adrc_t *a) {
  a->v1 = 0.0f;
  a->v2 = 0.0f;
  a->z1 = 0.0f;
  a->z2 = 0.0f;
  a->z3 = 0.0f;
  a->u_prev = 0.0f;
}

void sc_adrc_set_limits(sc_adrc_t *a, float out_min, float out_max) {
  a->out_min = out_min;
  a->out_max = out_max;
}

float sc_adrc_update(sc_adrc_t *a, float ref, float measurement, float dt) {
  if (dt <= 0.0f) {
    dt = 1e-3f;
  }
  if (!a->init) {
    sc_adrc_init(a, a->b0, dt);
  }
  if (fabsf(a->b0) < 1e-6f) {
    a->b0 = 1.0f;
  }

  /* ---------- 1. TD：安排过渡过程 ---------- */
  float f = fhan(a->v1 - ref, a->v2, a->r, dt);
  a->v1 += dt * a->v2;
  a->v2 += dt * f;

  /* ---------- 2. ESO：观测状态与总扰动 ---------- */
  float e = a->z1 - measurement;
  float fe = fal(e, 0.5f, dt);
  float fe1 = fal(e, 0.25f, dt);

  a->z1 += dt * (a->z2 - a->beta1 * e);
  a->z2 += dt * (a->z3 - a->beta2 * fe + a->b0 * a->u_prev);
  a->z3 += dt * (-a->beta3 * fe1);

  /* ---------- 3. NLSEF：误差反馈 + 扰动补偿 ---------- */
  float e1 = a->v1 - a->z1;
  float e2 = a->v2 - a->z2;

  float u0 =
      a->kp * fal(e1, a->a1, a->delta) + a->kd * fal(e2, a->a2, a->delta);

  float u = (u0 - a->z3) / a->b0;
  u = sc_clampf(u, a->out_min, a->out_max);

  /* ESO 用实际施加的输入，饱和时估计才准 */
  a->u_prev = u;
  return u;
}
