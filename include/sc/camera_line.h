/* 摄像头巡线：二值化、八邻域爬线、边线扫描、中线、曲率、元素识别、逆透视
 *
 * 图像约定：灰度图，data[0] 为左上角，行优先。
 * 二值图里 1 = 赛道（或黑线），0 = 背景。
 */
#ifndef SC_CAMERA_LINE_H
#define SC_CAMERA_LINE_H

#include "sc/common.h"

typedef struct {
  uint8_t *data;
  int width;
  int height;
} sc_image_t;

typedef struct {
  int x;
  int y;
} sc_point_t;

typedef struct {
  float x;
  float y;
} sc_point2f_t;

/* ---------------- 二值化 ---------------- */

/* 大津法（Otsu）自动阈值。返回 0..255。 */
uint8_t sc_otsu(const sc_image_t *img);

/* 二值化。line_is_dark=1 表示赛道/线是暗的（白底黑线）；
 * line_is_dark=0 表示赛道是亮的（黑底白线 / 白赛道黑边线场景选这个再取反）。 */
void sc_binarize(const sc_image_t *src, uint8_t *dst, uint8_t threshold,
                 int line_is_dark);

/* ---------------- 八邻域爬线 ---------------- */

/* 从 (x0,y0) 出发，沿二值图的 1 像素向上爬。
 * visited 必须是 width*height 的缓冲区并已清零（函数内部不负责分配）。
 * 返回写入 pts 的点数；到图像顶部、断线或达到 max_pts 时停止。
 * 适合单根黑线（电磁/摄像头巡线的基础）。 */
int sc_trace_8(const uint8_t *bin, uint8_t *visited, int width, int height,
               int x0, int y0, sc_point_t *pts, int max_pts);

/* ---------------- 边线扫描 ---------------- */

#define SC_EDGE_MAX_ROWS 128

typedef struct {
  int left[SC_EDGE_MAX_ROWS];   /* 左边界列，-1 表示本行丢失 */
  int right[SC_EDGE_MAX_ROWS];  /* 右边界列 */
  int center[SC_EDGE_MAX_ROWS]; /* (left+right)/2 */
  int width[SC_EDGE_MAX_ROWS];  /* right-left */
  int rows;                     /* 实际扫描行数 */
  int y_bottom;
  int y_step;    /* 通常 -1，从下往上 */
  int img_width; /* 图像宽度，用于判断边线是否贴到画面边缘 */
} sc_edge_t;

/* 从底部向上逐行扫描赛道边线。
 * center_x：底部种子列（一般图像宽度/2）。
 * max_jump：每行相对上一行允许的最大横向跳变，防止跨元素误连。
 * min_width：认为有效的最小赛道宽度（像素）。
 * 返回有效行数。 */
int sc_scan_edges(const uint8_t *bin, int width, int height, int y_bottom,
                  int y_step, int rows, int center_x, int max_jump,
                  int min_width, sc_edge_t *e);

/* 由边线求中线（行内已算好，这里做一次孤立点剔除 + 平滑） */
void sc_edge_smooth_center(sc_edge_t *e, int window);

/* 像素空间曲率（只反映形状，不是物理曲率）。取底部、中部、顶部三点。
 * 正=左弯。 */
float sc_centerline_curvature_px(const sc_edge_t *e);

/* ---------------- 元素识别（骨架，阈值需按赛道调） ---------------- */

typedef enum {
  SC_ELEM_NONE = 0,
  SC_ELEM_STRAIGHT,
  SC_ELEM_CURVE_LEFT,
  SC_ELEM_CURVE_RIGHT,
  SC_ELEM_CROSS,
  SC_ELEM_ROUNDABOUT,
  SC_ELEM_RAMP,
  SC_ELEM_OBSTACLE,
  SC_ELEM_UNKNOWN
} sc_element_t;

typedef struct {
  float curvature_px; /* 近端像素曲率 */
  float width_mean;   /* 平均赛道宽度（像素） */
  float width_jump;   /* 相邻行宽度的最大相对跳变 */
  int left_lost;      /* 左边界丢失行数 */
  int right_lost;
  int both_lost;
  int rows;
} sc_track_feat_t;

void sc_track_features(const sc_edge_t *e, sc_track_feat_t *f);

sc_element_t sc_classify_element(const sc_track_feat_t *f, float curve_thresh,
                                 float width_jump_thresh);

const char *sc_element_name(sc_element_t e);

/* ---------------- 相机模型与逆透视（IPM） ---------------- */

typedef struct {
  float height; /* 相机光心离地高度 m */
  float pitch;  /* 俯仰角，镜头向下为正，rad */
  float focal;  /* 焦距（像素） */
  float cx;     /* 主点列 */
  float cy;     /* 主点行 */
} sc_camera_t;

void sc_camera_init(sc_camera_t *c, float height, float pitch, float focal,
                    float cx, float cy);

/* 像素 (u,v) -> 地面坐标 (x 向前, y 向左)，单位 m。
 * 返回 0 成功，-1 表示射线朝上打不到地面。 */
int sc_ipm_pixel_to_ground(const sc_camera_t *c, float u, float v, float *x,
                           float *y);

/* 地面坐标 -> 像素（用于在图上画预测轨迹） */
int sc_ipm_ground_to_pixel(const sc_camera_t *c, float x, float y, float *u,
                           float *v);

/* 把整条中线转换到地面坐标，返回有效点数 */
int sc_centerline_to_ground(const sc_edge_t *e, const sc_camera_t *c,
                            sc_point2f_t *out, int max_out);

/* 地面坐标下的物理曲率（1/m），用近端三点计算 */
float sc_ground_curvature(const sc_point2f_t *pts, int n);

#endif
