#include "sc/camera_line.h"

#include "sc/math_utils.h"

/* ============================================================
 * 二值化
 * ============================================================ */

uint8_t sc_otsu(const sc_image_t *img) {
  int hist[256] = {0};
  int total = img->width * img->height;

  if (total <= 0) {
    return 128;
  }

  for (int i = 0; i < total; i++) {
    hist[img->data[i]]++;
  }

  float sum_all = 0.0f;
  for (int i = 0; i < 256; i++) {
    sum_all += (float)i * (float)hist[i];
  }

  int w_bg = 0;
  float sum_bg = 0.0f;
  float best_var = -1.0f;
  int best = 128;

  for (int t = 0; t < 256; t++) {
    w_bg += hist[t];
    if (w_bg == 0) {
      continue;
    }
    int w_fg = total - w_bg;
    if (w_fg == 0) {
      break;
    }
    sum_bg += (float)t * (float)hist[t];

    float m_bg = sum_bg / (float)w_bg;
    float m_fg = (sum_all - sum_bg) / (float)w_fg;
    float diff = m_bg - m_fg;
    float var = (float)w_bg * (float)w_fg * diff * diff;

    if (var > best_var) {
      best_var = var;
      best = t;
    }
  }

  return (uint8_t)best;
}

void sc_binarize(const sc_image_t *src, uint8_t *dst, uint8_t threshold,
                 int line_is_dark) {
  int total = src->width * src->height;

  for (int i = 0; i < total; i++) {
    /* 用 <=：Otsu 在两峰之间任意阈值都等价，但阈值正好等于暗峰时
     * 必须把暗峰算作"暗"，否则整幅图会翻转。 */
    int is_dark = src->data[i] <= threshold;
    dst[i] = (uint8_t)(line_is_dark ? is_dark : !is_dark);
  }
}

/* ============================================================
 * 八邻域爬线
 * ============================================================ */

/* 方向 0=上，顺时针：上、右上、右、右下、下、左下、左、左上 */
static const int DX8[8] = {0, 1, 1, 1, 0, -1, -1, -1};
static const int DY8[8] = {-1, -1, 0, 1, 1, 1, 0, -1};

int sc_trace_8(const uint8_t *bin, uint8_t *visited, int width, int height,
               int x0, int y0, sc_point_t *pts, int max_pts) {
  if (!bin || !visited || !pts || max_pts <= 0) {
    return 0;
  }
  if (x0 < 0 || x0 >= width || y0 < 0 || y0 >= height) {
    return 0;
  }
  if (!bin[y0 * width + x0]) {
    return 0;
  }

  int x = x0;
  int y = y0;
  int dir = 0; /* 初始向上 */
  int n = 0;

  pts[n].x = x;
  pts[n].y = y;
  n++;
  visited[y * width + x] = 1;

  while (n < max_pts) {
    int found = 0;

    /* 搜索顺序：先直行，再左右偏 1，再偏 2，最后回头。
     * 这样 1 像素细线不会因为浮点/噪声左右抖动。 */
    static const int order[8] = {0, 1, -1, 2, -2, 3, -3, 4};

    for (int k = 0; k < 8; k++) {
      int d = (dir + order[k] + 8) % 8;
      int nx = x + DX8[d];
      int ny = y + DY8[d];

      if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
        continue;
      }
      if (!bin[ny * width + nx]) {
        continue;
      }
      if (visited[ny * width + nx]) {
        continue;
      }

      x = nx;
      y = ny;
      dir = d;
      found = 1;
      break;
    }

    if (!found) {
      break;
    }

    pts[n].x = x;
    pts[n].y = y;
    n++;
    visited[y * width + x] = 1;

    if (y == 0) {
      break;
    }
  }

  return n;
}

/* ============================================================
 * 边线扫描
 * ============================================================ */

static int find_run(const uint8_t *bin, int width, int y, int seed,
                    int max_jump, int *out_l, int *out_r) {
  const uint8_t *row = bin + (size_t)y * (size_t)width;

  /* 从 seed 向外找最近的赛道像素 */
  int start = -1;
  if (seed >= 0 && seed < width && row[seed]) {
    start = seed;
  } else {
    for (int k = 1; k <= max_jump; k++) {
      int xl = seed - k;
      int xr = seed + k;
      if (xl >= 0 && row[xl]) {
        start = xl;
        break;
      }
      if (xr < width && row[xr]) {
        start = xr;
        break;
      }
    }
  }

  if (start < 0) {
    return 0;
  }

  int l = start;
  int r = start;
  while (l - 1 >= 0 && row[l - 1]) {
    l--;
  }
  while (r + 1 < width && row[r + 1]) {
    r++;
  }

  *out_l = l;
  *out_r = r;
  return 1;
}

int sc_scan_edges(const uint8_t *bin, int width, int height, int y_bottom,
                  int y_step, int rows, int center_x, int max_jump,
                  int min_width, sc_edge_t *e) {
  if (!bin || !e || width <= 0 || height <= 0) {
    return 0;
  }
  if (rows > SC_EDGE_MAX_ROWS) {
    rows = SC_EDGE_MAX_ROWS;
  }
  if (y_step == 0) {
    y_step = -1;
  }

  memset(e, 0, sizeof(*e));
  e->y_bottom = y_bottom;
  e->y_step = y_step;
  e->img_width = width;

  int seed = center_x;

  for (int i = 0; i < rows; i++) {
    int y = y_bottom + i * y_step;
    if (y < 0 || y >= height) {
      break;
    }

    e->left[i] = -1;
    e->right[i] = -1;
    e->center[i] = -1;
    e->width[i] = 0;

    int l = -1;
    int r = -1;
    if (find_run(bin, width, y, seed, max_jump, &l, &r)) {
      int w = r - l + 1;
      if (w >= min_width) {
        e->left[i] = l;
        e->right[i] = r;
        e->width[i] = w;
        e->center[i] = (l + r) / 2;
        seed = e->center[i];
      }
    }

    e->rows = i + 1;
  }

  return e->rows;
}

void sc_edge_smooth_center(sc_edge_t *e, int window) {
  if (window < 1) {
    window = 1;
  }

  int tmp[SC_EDGE_MAX_ROWS];

  for (int i = 0; i < e->rows; i++) {
    tmp[i] = e->center[i];
  }

  for (int i = 0; i < e->rows; i++) {
    if (tmp[i] < 0) {
      continue;
    }
    int sum = 0;
    int cnt = 0;
    int lo = i - window;
    int hi = i + window;
    if (lo < 0)
      lo = 0;
    if (hi >= e->rows)
      hi = e->rows - 1;

    for (int j = lo; j <= hi; j++) {
      if (tmp[j] >= 0) {
        sum += tmp[j];
        cnt++;
      }
    }
    if (cnt >= 2) {
      e->center[i] = sum / cnt;
    }
  }
}

float sc_centerline_curvature_px(const sc_edge_t *e) {
  if (e->rows < 3) {
    return 0.0f;
  }

  /* 从底部找三个有效中心点 */
  int idx[3] = {-1, -1, -1};
  int cnt = 0;
  int picks[3] = {0, e->rows / 2, e->rows - 1};

  for (int k = 0; k < 3; k++) {
    int p = picks[k];
    for (int off = 0; off < e->rows; off++) {
      int a = p - off;
      int b = p + off;
      if (a >= 0 && a < e->rows && e->center[a] >= 0) {
        idx[k] = a;
        break;
      }
      if (b >= 0 && b < e->rows && e->center[b] >= 0) {
        idx[k] = b;
        break;
      }
    }
    if (idx[k] >= 0) {
      cnt++;
    }
  }

  if (cnt < 3) {
    return 0.0f;
  }

  float x0 = (float)e->center[idx[0]];
  float y0 = (float)idx[0];
  float x1 = (float)e->center[idx[1]];
  float y1 = (float)idx[1];
  float x2 = (float)e->center[idx[2]];
  float y2 = (float)idx[2];

  /* 图像 y 向下，转成数学坐标：Y = -y */
  float r = sc_curvature_3pt(x0, -y0, x1, -y1, x2, -y2);
  return (r > 1e6f) ? 0.0f : 1.0f / r;
}

/* ============================================================
 * 元素识别
 * ============================================================ */

void sc_track_features(const sc_edge_t *e, sc_track_feat_t *f) {
  memset(f, 0, sizeof(*f));
  f->rows = e->rows;

  int width_sum = 0;
  int width_cnt = 0;
  int prev_w = 0;
  float max_jump = 0.0f;

  for (int i = 0; i < e->rows; i++) {
    if (e->left[i] < 0) {
      f->both_lost++;
      continue;
    }
    if (e->width[i] <= 0) {
      f->both_lost++;
      continue;
    }

    /* 边线贴到画面边缘：在环岛/大弯里，内侧边线常常整条消失 */
    if (e->left[i] <= 0) {
      f->left_lost++;
    }
    if (e->img_width > 0 && e->right[i] >= e->img_width - 1) {
      f->right_lost++;
    }

    width_sum += e->width[i];
    width_cnt++;

    if (prev_w > 0) {
      float jump = fabsf((float)e->width[i] - (float)prev_w) / (float)prev_w;
      if (jump > max_jump) {
        max_jump = jump;
      }
    }
    prev_w = e->width[i];
  }

  f->width_mean = (width_cnt > 0) ? (float)width_sum / (float)width_cnt : 0.0f;
  f->width_jump = max_jump;
  f->curvature_px = sc_centerline_curvature_px(e);
}

sc_element_t sc_classify_element(const sc_track_feat_t *f, float curve_thresh,
                                 float width_jump_thresh) {
  if (f->rows <= 0) {
    return SC_ELEM_UNKNOWN;
  }

  float lost_ratio = (float)f->both_lost / (float)f->rows;
  float half = 0.5f * (float)f->rows;

  if (lost_ratio > 0.45f) {
    return SC_ELEM_CROSS; /* 大面积丢线：十字/出赛道，需结合上一状态 */
  }
  /* 只有一侧长期贴边：环岛的典型特征（内侧边线整条消失） */
  if (f->left_lost > half && f->right_lost < 0.2f * (float)f->rows) {
    return SC_ELEM_ROUNDABOUT;
  }
  if (f->right_lost > half && f->left_lost < 0.2f * (float)f->rows) {
    return SC_ELEM_ROUNDABOUT;
  }
  if (f->width_jump > width_jump_thresh) {
    return SC_ELEM_CROSS; /* 宽度突变：十字路口或环岛入口 */
  }
  if (f->width_mean < 1.0f) {
    return SC_ELEM_UNKNOWN;
  }
  if (f->curvature_px > curve_thresh) {
    return SC_ELEM_CURVE_LEFT;
  }
  if (f->curvature_px < -curve_thresh) {
    return SC_ELEM_CURVE_RIGHT;
  }
  return SC_ELEM_STRAIGHT;
}

const char *sc_element_name(sc_element_t e) {
  switch (e) {
  case SC_ELEM_STRAIGHT:
    return "直道";
  case SC_ELEM_CURVE_LEFT:
    return "左弯";
  case SC_ELEM_CURVE_RIGHT:
    return "右弯";
  case SC_ELEM_CROSS:
    return "十字/交叉";
  case SC_ELEM_ROUNDABOUT:
    return "环岛";
  case SC_ELEM_RAMP:
    return "坡道";
  case SC_ELEM_OBSTACLE:
    return "障碍";
  case SC_ELEM_UNKNOWN:
    return "未知";
  default:
    return "无";
  }
}

/* ============================================================
 * 相机模型 / IPM
 * ============================================================ */

void sc_camera_init(sc_camera_t *c, float height, float pitch, float focal,
                    float cx, float cy) {
  c->height = height;
  c->pitch = pitch;
  c->focal = focal;
  c->cx = cx;
  c->cy = cy;
}

int sc_ipm_pixel_to_ground(const sc_camera_t *c, float u, float v, float *x,
                           float *y) {
  float xc = u - c->cx;
  float yc = v - c->cy;
  float zc = c->focal;

  float sp = sinf(c->pitch);
  float cp = cosf(c->pitch);

  /* 相机坐标 -> 世界坐标（前、左、上），推导见 docs/02 */
  float fwd = -yc * sp + zc * cp;
  float left = -xc;
  float up = -yc * cp - zc * sp;

  if (up >= -1e-5f) {
    return -1; /* 射线朝上，打不到地面 */
  }

  float t = c->height / (-up);
  *x = t * fwd;
  *y = t * left;
  return 0;
}

int sc_ipm_ground_to_pixel(const sc_camera_t *c, float x, float y, float *u,
                           float *v) {
  /* 反解：世界点 (x, y, z=-height) 投影到像素 */
  float sp = sinf(c->pitch);
  float cp = cosf(c->pitch);

  /* 世界(前,左,上) -> 相机(x右,y下,z前)
   * 由正变换 [fwd,left,up] = R [xc,yc,zc] 反推：
   *   fwd = -yc*sp + zc*cp
   *   left = -xc
   *   up = -yc*cp - zc*sp
   */
  float fwd = x;
  float left = y;
  float up = -c->height;

  float xc = -left;

  /* 解 [ -sp  cp ] [yc]   [fwd]
   *     [ -cp -sp ] [zc] = [up ]
   * 行列式 = sp*sp + cp*cp = 1
   */
  float yc = -sp * fwd - cp * up;
  float zc = cp * fwd - sp * up;

  if (zc <= 1e-5f) {
    return -1;
  }

  *u = c->cx + c->focal * (xc / zc);
  *v = c->cy + c->focal * (yc / zc);
  return 0;
}

int sc_centerline_to_ground(const sc_edge_t *e, const sc_camera_t *c,
                            sc_point2f_t *out, int max_out) {
  int n = 0;

  for (int i = 0; i < e->rows && n < max_out; i++) {
    if (e->center[i] < 0) {
      continue;
    }
    int y = e->y_bottom + i * e->y_step;
    float gx = 0.0f;
    float gy = 0.0f;
    if (sc_ipm_pixel_to_ground(c, (float)e->center[i], (float)y, &gx, &gy) ==
        0) {
      out[n].x = gx;
      out[n].y = gy;
      n++;
    }
  }

  return n;
}

float sc_ground_curvature(const sc_point2f_t *pts, int n) {
  if (n < 3) {
    return 0.0f;
  }

  /* 取近端三点（前三个有效点）估计当前曲率 */
  float r = sc_curvature_3pt(pts[0].x, pts[0].y, pts[1].x, pts[1].y, pts[2].x,
                             pts[2].y);
  if (r > 1e6f) {
    return 0.0f;
  }
  return 1.0f / r;
}
