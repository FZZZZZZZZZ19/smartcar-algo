#include "sc/filter.h"

#include <stdlib.h>

#include "sc/math_utils.h"

/* ================= 一阶低通 ================= */

void sc_lpf1_reset(sc_lpf1_t *f, float y0) {
  f->y = y0;
  f->init = 1;
}

float sc_lpf1_step(sc_lpf1_t *f, float x, float alpha) {
  alpha = sc_clampf(alpha, 0.0f, 1.0f);
  if (!f->init) {
    f->y = x;
    f->init = 1;
    return x;
  }
  /* alpha 越接近 1，越相信历史；alpha=0 表示完全跟随 */
  f->y = alpha * f->y + (1.0f - alpha) * x;
  return f->y;
}

/* ================= 滑动平均 ================= */

void sc_ma_init(sc_ma_t *f, int n) {
  if (n < 1)
    n = 1;
  if (n > SC_FILTER_MAX)
    n = SC_FILTER_MAX;
  memset(f, 0, sizeof(*f));
  f->n = n;
}

void sc_ma_reset(sc_ma_t *f, float y0) {
  for (int i = 0; i < f->n; i++) {
    f->buf[i] = y0;
  }
  f->idx = 0;
  f->count = f->n;
  f->sum = y0 * (float)f->n;
}

float sc_ma_step(sc_ma_t *f, float x) {
  if (f->count < f->n) {
    f->count++;
    f->sum += x;
  } else {
    f->sum -= f->buf[f->idx];
    f->sum += x;
  }
  f->buf[f->idx] = x;
  f->idx = (f->idx + 1) % f->n;
  return f->sum / (float)f->count;
}

/* ================= 中值滤波 ================= */

void sc_median_init(sc_median_t *f, int n) {
  if (n < 1)
    n = 1;
  if (n > SC_FILTER_MAX)
    n = SC_FILTER_MAX;
  memset(f, 0, sizeof(*f));
  f->n = n;
}

void sc_median_reset(sc_median_t *f, float y0) {
  for (int i = 0; i < f->n; i++) {
    f->buf[i] = y0;
  }
  f->idx = 0;
  f->count = f->n;
}

static int cmp_float(const void *a, const void *b) {
  float fa = *(const float *)a;
  float fb = *(const float *)b;
  return (fa > fb) - (fa < fb);
}

float sc_median_step(sc_median_t *f, float x) {
  f->buf[f->idx] = x;
  f->idx = (f->idx + 1) % f->n;
  if (f->count < f->n) {
    f->count++;
  }

  /* n <= 32，直接拷贝排序足够快，也避免维护堆的复杂度 */
  float tmp[SC_FILTER_MAX];
  memcpy(tmp, f->buf, sizeof(float) * (size_t)f->count);
  qsort(tmp, (size_t)f->count, sizeof(float), cmp_float);

  if (f->count & 1) {
    return tmp[f->count / 2];
  }
  return 0.5f * (tmp[f->count / 2 - 1] + tmp[f->count / 2]);
}

/* ================= 斜率限制 ================= */

void sc_rate_lim_init(sc_rate_lim_t *f, float max_rate, float y0) {
  f->y = y0;
  f->max_rate = max_rate;
  f->init = 1;
}

float sc_rate_lim_step(sc_rate_lim_t *f, float target, float dt) {
  if (!f->init) {
    f->y = target;
    f->init = 1;
    return target;
  }
  f->y = sc_slew_limit(f->y, target, f->max_rate, dt);
  return f->y;
}

/* ================= 一维卡尔曼 ================= */

void sc_kf1_init(sc_kf1_t *f, float q, float r, float x0, float p0) {
  f->q = q;
  f->r = r;
  f->x = x0;
  f->p = p0;
  f->init = 1;
}

float sc_kf1_step(sc_kf1_t *f, float z) {
  if (!f->init) {
    sc_kf1_init(f, f->q, f->r, z, 1.0f);
    return z;
  }

  /* 预测：状态不变，方差增加 */
  f->p += f->q;

  /* 更新 */
  float k = f->p / (f->p + f->r);
  f->x += k * (z - f->x);
  f->p *= (1.0f - k);

  return f->x;
}

/* ================= 二维卡尔曼 ================= */

void sc_kf2_init(sc_kf2_t *f, float q, float r, float z0) {
  memset(f, 0, sizeof(*f));
  f->x[0] = z0;
  f->x[1] = 0.0f;
  f->P[0][0] = 1.0f;
  f->P[1][1] = 1.0f;
  f->q = q;
  f->r = r;
  f->init = 1;
}

void sc_kf2_predict(sc_kf2_t *f, float dt) {
  if (dt <= 0.0f) {
    return;
  }

  /* 状态转移 [1 dt; 0 1] */
  float p00 = f->P[0][0], p01 = f->P[0][1];
  float p10 = f->P[1][0], p11 = f->P[1][1];

  f->x[0] += f->x[1] * dt;

  /* P = F P F^T，先算 FP */
  float fp00 = p00 + dt * p10;
  float fp01 = p01 + dt * p11;
  float fp10 = p10;
  float fp11 = p11;

  /* 再乘 F^T */
  f->P[0][0] = fp00 + dt * fp01;
  f->P[0][1] = fp01;
  f->P[1][0] = fp10 + dt * fp11;
  f->P[1][1] = fp11;

  /* 过程噪声：连续白噪声加速度模型的离散化 */
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;
  float q = f->q;
  f->P[0][0] += q * dt4 * 0.25f;
  f->P[0][1] += q * dt3 * 0.5f;
  f->P[1][0] += q * dt3 * 0.5f;
  f->P[1][1] += q * dt2;
}

float sc_kf2_update(sc_kf2_t *f, float z) {
  if (!f->init) {
    sc_kf2_init(f, f->q, f->r, z);
    return z;
  }

  float s = f->P[0][0] + f->r;
  float k0 = f->P[0][0] / s;
  float k1 = f->P[1][0] / s;
  float y = z - f->x[0];

  f->x[0] += k0 * y;
  f->x[1] += k1 * y;

  /* P = (I - K H) P */
  float p00 = f->P[0][0], p01 = f->P[0][1];
  float p10 = f->P[1][0], p11 = f->P[1][1];

  f->P[0][0] = (1.0f - k0) * p00;
  f->P[0][1] = (1.0f - k0) * p01;
  f->P[1][0] = p10 - k1 * p00;
  f->P[1][1] = p11 - k1 * p01;

  return f->x[0];
}

float sc_kf2_pos(const sc_kf2_t *f) { return f->x[0]; }

float sc_kf2_vel(const sc_kf2_t *f) { return f->x[1]; }
