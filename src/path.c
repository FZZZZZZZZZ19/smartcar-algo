#include "sc/path.h"

#include "sc/math_utils.h"

static int seg_start_count(const sc_path_t *p) {
  return p->closed ? p->n : p->n - 1;
}

/* 第 i 条线段的起点、终点索引与长度 */
static void seg_info(const sc_path_t *p, int i, int *i0, int *i1, float *len) {
  *i0 = i;
  *i1 = (i + 1) % p->n;

  if (i + 1 < p->n) {
    *len = p->s[i + 1] - p->s[i];
  } else {
    /* 闭环的收尾段：从最后一个点回到第 0 个点 */
    *len = p->length - p->s[i];
  }
}

void sc_path_build(sc_path_t *p, const float *x, const float *y, float *s,
                   int n, int closed) {
  p->x = x;
  p->y = y;
  p->s = s;
  p->n = n;
  p->closed = closed;

  if (n <= 0) {
    p->length = 0.0f;
    return;
  }

  s[0] = 0.0f;
  for (int i = 1; i < n; i++) {
    float dx = x[i] - x[i - 1];
    float dy = y[i] - y[i - 1];
    s[i] = s[i - 1] + sqrtf(dx * dx + dy * dy);
  }

  if (closed && n > 1) {
    float dx = x[0] - x[n - 1];
    float dy = y[0] - y[n - 1];
    p->length = s[n - 1] + sqrtf(dx * dx + dy * dy);
  } else {
    p->length = s[n - 1];
  }
}

float sc_path_s_delta(const sc_path_t *p, float s_from, float s_to) {
  float d = s_to - s_from;
  if (p->closed && p->length > 1e-6f) {
    d = fmodf(d, p->length);
    if (d < 0.0f) {
      d += p->length;
    }
    /* 取最短方向 */
    if (d > 0.5f * p->length) {
      d -= p->length;
    }
  }
  return d;
}

void sc_path_point_at(const sc_path_t *p, float s, float *x, float *y) {
  if (p->n <= 0) {
    *x = 0.0f;
    *y = 0.0f;
    return;
  }
  if (p->n == 1) {
    *x = p->x[0];
    *y = p->y[0];
    return;
  }

  if (p->closed && p->length > 1e-6f) {
    s = fmodf(s, p->length);
    if (s < 0.0f) {
      s += p->length;
    }
  } else {
    s = sc_clampf(s, 0.0f, p->length);
  }

  /* 二分查找所在线段 */
  int lo = 0;
  int hi = p->n - 1;
  while (lo + 1 < hi) {
    int mid = (lo + hi) / 2;
    if (p->s[mid] <= s) {
      lo = mid;
    } else {
      hi = mid;
    }
  }

  int i0 = lo;
  int i1 = (lo + 1) % p->n;
  float len = (i1 == 0) ? (p->length - p->s[i0]) : (p->s[i1] - p->s[i0]);

  float t = (len > 1e-9f) ? (s - p->s[i0]) / len : 0.0f;
  t = sc_clampf(t, 0.0f, 1.0f);

  *x = sc_lerp(p->x[i0], p->x[i1], t);
  *y = sc_lerp(p->y[i0], p->y[i1], t);
}

float sc_path_heading_at(const sc_path_t *p, float s) {
  if (p->n < 2) {
    return 0.0f;
  }

  float s1 = s + 0.05f;
  float s2 = s - 0.05f;
  float x1, y1, x2, y2;
  sc_path_point_at(p, s1, &x1, &y1);
  sc_path_point_at(p, s2, &x2, &y2);

  return atan2f(y1 - y2, x1 - x2);
}

float sc_path_curvature_at(const sc_path_t *p, float s, float ds) {
  if (ds < 0.02f) {
    ds = 0.02f;
  }

  float x0, y0, x1, y1, x2, y2;
  sc_path_point_at(p, s - ds, &x0, &y0);
  sc_path_point_at(p, s, &x1, &y1);
  sc_path_point_at(p, s + ds, &x2, &y2);

  float r = sc_curvature_3pt(x0, y0, x1, y1, x2, y2);
  if (r > 1e6f) {
    return 0.0f;
  }
  return 1.0f / r;
}

int sc_path_project(const sc_path_t *p, float x, float y, float theta, int hint,
                    int window, float *s_out, float *ey_out, float *epsi_out,
                    int *idx_out) {
  if (p->n < 2) {
    return -1;
  }

  int segs = seg_start_count(p);
  int first = 0;
  int last = segs - 1;

  if (hint >= 0 && window > 0) {
    first = hint - window;
    last = hint + window;
  }

  float best_d2 = 1e30f;
  float best_s = 0.0f;
  float best_tx = 1.0f;
  float best_ty = 0.0f;
  int best_i = -1;

  for (int k = first; k <= last; k++) {
    int i = k;
    if (p->closed) {
      i = ((k % segs) + segs) % segs;
    } else if (i < 0 || i >= segs) {
      continue;
    }

    int i0, i1;
    float len;
    seg_info(p, i, &i0, &i1, &len);
    if (len < 1e-9f) {
      continue;
    }

    float dx = p->x[i1] - p->x[i0];
    float dy = p->y[i1] - p->y[i0];
    float inv = 1.0f / len;
    float tx = dx * inv;
    float ty = dy * inv;

    float t = ((x - p->x[i0]) * dx + (y - p->y[i0]) * dy) / (len * len);
    t = sc_clampf(t, 0.0f, 1.0f);

    float px = p->x[i0] + t * dx;
    float py = p->y[i0] + t * dy;
    float ex = x - px;
    float ey = y - py;
    float d2 = ex * ex + ey * ey;

    if (d2 < best_d2) {
      best_d2 = d2;
      best_s = p->s[i0] + t * len;
      best_tx = tx;
      best_ty = ty;
      best_i = i;
    }
  }

  if (best_i < 0) {
    return -1;
  }

  float px, py;
  sc_path_point_at(p, best_s, &px, &py);

  /* 叉积 t x (car - p)：正表示车在路径左侧 */
  float ey = best_tx * (y - py) - best_ty * (x - px);
  float path_heading = atan2f(best_ty, best_tx);

  if (s_out) {
    *s_out = best_s;
  }
  if (ey_out) {
    *ey_out = ey;
  }
  if (epsi_out) {
    *epsi_out = sc_wrap_pi(theta - path_heading);
  }
  if (idx_out) {
    *idx_out = best_i;
  }

  return 0;
}

void sc_path_lookahead(const sc_path_t *p, float s_car, float ld, float *x,
                       float *y, float *s_out) {
  float s = s_car + ld;

  if (p->closed && p->length > 1e-6f) {
    s = fmodf(s, p->length);
    if (s < 0.0f) {
      s += p->length;
    }
  } else {
    s = sc_clampf(s, 0.0f, p->length);
  }

  sc_path_point_at(p, s, x, y);
  if (s_out) {
    *s_out = s;
  }
}

/* ============================================================
 * Catmull-Rom 路径平滑
 * ============================================================ */

int sc_catmull_rom(const float *cx, const float *cy, int cn, int closed,
                   int samples_per_seg, float *out_x, float *out_y) {
  if (cn < 2 || samples_per_seg < 1) {
    return 0;
  }

  int segs = closed ? cn : (cn - 1);
  int n = 0;

  for (int i = 0; i < segs; i++) {
    int i0, i1, i2, i3;

    if (closed) {
      i0 = (i - 1 + cn) % cn;
      i1 = i % cn;
      i2 = (i + 1) % cn;
      i3 = (i + 2) % cn;
    } else {
      i0 = (i > 0) ? i - 1 : 0;
      i1 = i;
      i2 = (i + 1 < cn) ? i + 1 : cn - 1;
      i3 = (i + 2 < cn) ? i + 2 : cn - 1;
    }

    float x0 = cx[i0], y0 = cy[i0];
    float x1 = cx[i1], y1 = cy[i1];
    float x2 = cx[i2], y2 = cy[i2];
    float x3 = cx[i3], y3 = cy[i3];

    for (int k = 0; k < samples_per_seg; k++) {
      float t = (float)k / (float)samples_per_seg;
      float t2 = t * t;
      float t3 = t2 * t;

      /* Catmull-Rom 基函数 */
      float b0 = -0.5f * t3 + t2 - 0.5f * t;
      float b1 = 1.5f * t3 - 2.5f * t2 + 1.0f;
      float b2 = -1.5f * t3 + 2.0f * t2 + 0.5f * t;
      float b3 = 0.5f * t3 - 0.5f * t2;

      out_x[n] = b0 * x0 + b1 * x1 + b2 * x2 + b3 * x3;
      out_y[n] = b0 * y0 + b1 * y1 + b2 * y2 + b3 * y3;
      n++;
    }
  }

  if (!closed) {
    /* 开环补上最后一个点 */
    out_x[n] = cx[cn - 1];
    out_y[n] = cy[cn - 1];
    n++;
  }

  return n;
}
