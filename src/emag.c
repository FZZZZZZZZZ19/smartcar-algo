#include "sc/emag.h"

#include "sc/math_utils.h"

void sc_emag_init(sc_emag_t *e, int n, const float *pos, float initial_max) {
  if (n > SC_EMAG_MAX) {
    n = SC_EMAG_MAX;
  }
  memset(e, 0, sizeof(*e));
  e->n = n;
  e->max_decay = 0.999f;
  e->min_rise = 0.999f;
  e->sharpness = 1.5f;

  for (int i = 0; i < n; i++) {
    e->pos[i] = pos ? pos[i] : 0.0f;
    e->vmax[i] = (initial_max > 0.0f) ? initial_max : 1.0f;
    e->vmin[i] = 0.0f;
  }
}

void sc_emag_update(sc_emag_t *e, const float *raw) {
  for (int i = 0; i < e->n; i++) {
    float v = raw[i];
    e->raw[i] = v;

    /* 最大值慢衰减、最小值慢回升：适应赛道电压/温度漂移，
     * 同时保留"曾经到过的峰值"，避免归一化忽大忽小。 */
    if (v > e->vmax[i]) {
      e->vmax[i] = v;
    } else {
      e->vmax[i] = e->max_decay * e->vmax[i] + (1.0f - e->max_decay) * v;
    }

    if (v < e->vmin[i]) {
      e->vmin[i] = v;
    } else {
      e->vmin[i] = e->min_rise * e->vmin[i] + (1.0f - e->min_rise) * v;
    }

    float span = e->vmax[i] - e->vmin[i];
    if (span < 1e-6f) {
      e->norm[i] = 0.0f;
    } else {
      e->norm[i] = sc_clampf((v - e->vmin[i]) / span, 0.0f, 1.0f);
    }
  }
}

void sc_emag_set_range(sc_emag_t *e, const float *vmin, const float *vmax) {
  for (int i = 0; i < e->n; i++) {
    e->vmin[i] = vmin[i];
    e->vmax[i] = vmax[i];
    float span = e->vmax[i] - e->vmin[i];
    if (span > 1e-6f) {
      e->norm[i] = sc_clampf((e->raw[i] - e->vmin[i]) / span, 0.0f, 1.0f);
    }
  }
}

float sc_emag_diff_ratio(float left, float right) {
  float s = left + right;
  if (fabsf(s) < 1e-6f) {
    return 0.0f;
  }
  return sc_clampf((left - right) / s, -1.0f, 1.0f);
}

float sc_emag_quadratic_offset(const float pos[3], const float val[3]) {
  float x0 = pos[0], x1 = pos[1], x2 = pos[2];
  float y0 = val[0], y1 = val[1], y2 = val[2];

  float den = (x0 - x1) * (x0 - x2) * (x1 - x2);
  if (fabsf(den) < 1e-9f) {
    return sc_emag_weighted_centroid(pos, val, 3, 1.0f);
  }

  float a = (y0 * (x1 - x2) + y1 * (x2 - x0) + y2 * (x0 - x1)) / den;
  float b = (y0 * (x2 * x2 - x1 * x1) + y1 * (x0 * x0 - x2 * x2) +
             y2 * (x1 * x1 - x0 * x0)) /
            den;

  if (fabsf(a) < 1e-6f) {
    return sc_emag_weighted_centroid(pos, val, 3, 1.0f);
  }

  return -b / (2.0f * a);
}

float sc_emag_weighted_centroid(const float *pos, const float *val, int n,
                                float sharpness) {
  float num = 0.0f;
  float den = 0.0f;

  for (int i = 0; i < n; i++) {
    float w = (val[i] > 0.0f) ? powf(val[i], sharpness) : 0.0f;
    num += pos[i] * w;
    den += w;
  }

  if (den < 1e-9f) {
    return 0.0f;
  }
  return num / den;
}

float sc_emag_offset_5(const sc_emag_t *e, float half_span) {
  if (e->n < 5) {
    return 0.0f;
  }

  float left = e->norm[0] + e->norm[1];
  float right = e->norm[3] + e->norm[4];
  float ratio = sc_emag_diff_ratio(left, right);

  /* 差分比在 ±0.5 附近接近线性，超出后压缩，避免饱和 */
  return half_span * tanhf(2.0f * ratio);
}

float sc_emag_direction(float up, float down) {
  return sc_emag_diff_ratio(up, down);
}

float sc_emag_confidence(const sc_emag_t *e) {
  if (e->n <= 0) {
    return 0.0f;
  }

  float sum = 0.0f;
  for (int i = 0; i < e->n; i++) {
    sum += e->norm[i];
  }
  return sc_clampf(sum / (float)e->n, 0.0f, 1.0f);
}

float sc_emag_sim_field(float sensor_y, float wire_y, float h) {
  float dy = sensor_y - wire_y;
  float den = dy * dy + h * h;
  if (den < 1e-9f) {
    return 1.0f;
  }
  return (h * h) / den;
}
