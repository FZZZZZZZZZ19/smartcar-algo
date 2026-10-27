#include "sc/fuzzy_pid.h"

#include "sc/math_utils.h"

/* 语言变量索引：0=NB 1=NM 2=NS 3=ZO 4=PS 5=PM 6=PB */
#define FUZZY_N 7
#define FUZZY_UNIVERSE 61 /* -3.0 .. 3.0，步长 0.1 */

/* 经典模糊 PID 规则表（行 = e，列 = ec） */
static const uint8_t RULE_KP[FUZZY_N][FUZZY_N] = {
    {6, 6, 5, 5, 4, 3, 3}, {6, 6, 5, 4, 4, 3, 2}, {5, 5, 5, 4, 3, 2, 2},
    {5, 5, 4, 3, 2, 1, 1}, {4, 4, 3, 2, 2, 1, 1}, {4, 3, 2, 1, 1, 1, 0},
    {3, 3, 1, 1, 1, 0, 0},
};

static const uint8_t RULE_KI[FUZZY_N][FUZZY_N] = {
    {0, 0, 1, 1, 2, 3, 3}, {0, 0, 1, 2, 2, 3, 3}, {0, 1, 2, 2, 3, 4, 4},
    {1, 1, 2, 3, 4, 5, 5}, {1, 2, 3, 4, 4, 5, 6}, {3, 3, 4, 4, 5, 6, 6},
    {3, 3, 4, 5, 5, 6, 6},
};

static const uint8_t RULE_KD[FUZZY_N][FUZZY_N] = {
    {4, 2, 0, 0, 0, 1, 4}, {4, 2, 0, 1, 1, 2, 3}, {3, 2, 1, 1, 2, 2, 3},
    {3, 2, 2, 2, 2, 2, 3}, {3, 3, 3, 3, 3, 3, 3}, {6, 4, 4, 4, 4, 4, 6},
    {6, 5, 5, 5, 4, 4, 6},
};

/* 三角形隶属度，中心在 -3..3，半宽 1 */
static float mu_set(int set, float x) {
  float center = (float)(set - 3);
  float d = fabsf(x - center);
  if (d >= 1.0f) {
    return 0.0f;
  }
  return 1.0f - d;
}

static float fuzzy_reason(const uint8_t rule[FUZZY_N][FUZZY_N], float e_norm,
                          float ec_norm) {
  float mu_e[FUZZY_N];
  float mu_ec[FUZZY_N];

  for (int i = 0; i < FUZZY_N; i++) {
    mu_e[i] = mu_set(i, e_norm);
    mu_ec[i] = mu_set(i, ec_norm);
  }

  /* 每条规则取 min，相同后件的规则取 max */
  float agg[FUZZY_N] = {0.0f};
  for (int i = 0; i < FUZZY_N; i++) {
    if (mu_e[i] <= 0.0f) {
      continue;
    }
    for (int j = 0; j < FUZZY_N; j++) {
      if (mu_ec[j] <= 0.0f) {
        continue;
      }
      float w = fminf(mu_e[i], mu_ec[j]);
      int out = rule[i][j];
      if (w > agg[out]) {
        agg[out] = w;
      }
    }
  }

  /* 重心法解模糊 */
  float num = 0.0f;
  float den = 0.0f;
  for (int k = 0; k < FUZZY_UNIVERSE; k++) {
    float x = -3.0f + 0.1f * (float)k;
    float m = 0.0f;
    for (int s = 0; s < FUZZY_N; s++) {
      float v = fminf(agg[s], mu_set(s, x));
      if (v > m) {
        m = v;
      }
    }
    num += x * m;
    den += m;
  }

  if (den < 1e-6f) {
    return 0.0f;
  }
  return num / den;
}

float sc_fuzzy_dkp(float e_norm, float ec_norm) {
  return fuzzy_reason(RULE_KP, e_norm, ec_norm);
}

float sc_fuzzy_dki(float e_norm, float ec_norm) {
  return fuzzy_reason(RULE_KI, e_norm, ec_norm);
}

float sc_fuzzy_dkd(float e_norm, float ec_norm) {
  return fuzzy_reason(RULE_KD, e_norm, ec_norm);
}

/* ============================================================ */

void sc_fuzzy_pid_init(sc_fuzzy_pid_t *f, float kp0, float ki0, float kd0,
                       float e_range, float ec_range) {
  memset(f, 0, sizeof(*f));
  sc_pid_init(&f->pid, kp0, ki0, kd0);

  f->kp0 = kp0;
  f->ki0 = ki0;
  f->kd0 = kd0;
  f->e_range = (e_range > 1e-6f) ? e_range : 1.0f;
  f->ec_range = (ec_range > 1e-6f) ? ec_range : 1.0f;

  /* 默认：模糊修正量是基础参数的 30% 量级 */
  f->kp_scale = 0.3f * kp0;
  f->ki_scale = 0.3f * ki0;
  f->kd_scale = 0.3f * kd0;
  f->init = 1;
}

void sc_fuzzy_pid_set_scales(sc_fuzzy_pid_t *f, float kp_scale, float ki_scale,
                             float kd_scale) {
  f->kp_scale = kp_scale;
  f->ki_scale = ki_scale;
  f->kd_scale = kd_scale;
}

void sc_fuzzy_pid_set_limits(sc_fuzzy_pid_t *f, float out_min, float out_max) {
  sc_pid_set_limits(&f->pid, out_min, out_max);
}

void sc_fuzzy_pid_reset(sc_fuzzy_pid_t *f) {
  sc_pid_reset(&f->pid);
  f->last_e = 0.0f;
  f->init = 1;
}

float sc_fuzzy_pid_update(sc_fuzzy_pid_t *f, float setpoint, float measurement,
                          float dt) {
  if (dt <= 0.0f) {
    dt = 1e-3f;
  }

  float e = setpoint - measurement;
  float ec = 0.0f;

  if (!f->init) {
    f->last_e = e;
    f->init = 1;
  } else {
    ec = (e - f->last_e) / dt;
  }

  float ne = sc_clampf(3.0f * e / f->e_range, -3.0f, 3.0f);
  float nec = sc_clampf(3.0f * ec / f->ec_range, -3.0f, 3.0f);

  float dkp = sc_fuzzy_dkp(ne, nec) * f->kp_scale;
  float dki = sc_fuzzy_dki(ne, nec) * f->ki_scale;
  float dkd = sc_fuzzy_dkd(ne, nec) * f->kd_scale;

  /* 模糊输出叠加到基础参数，并保证非负 */
  f->pid.kp = fmaxf(0.0f, f->kp0 + dkp);
  f->pid.ki = fmaxf(0.0f, f->ki0 + dki);
  f->pid.kd = fmaxf(0.0f, f->kd0 + dkd);

  float u = sc_pid_update(&f->pid, setpoint, measurement, dt);

  f->last_e = e;
  return u;
}
