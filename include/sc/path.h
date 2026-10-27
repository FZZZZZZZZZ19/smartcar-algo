/* 折线路径工具：弧长参数化、最近点投影、横向/航向误差、预览点
 *
 * 路径由 x[]、y[] 两个数组给出，s[] 为调用方提供的累计弧长缓冲区。
 * 不做动态内存分配，方便直接移植到 MCU。
 */
#ifndef SC_PATH_H
#define SC_PATH_H

#include "sc/common.h"

typedef struct {
  const float *x;
  const float *y;
  float *s; /* 长度 >= n 的缓冲区 */
  int n;
  int closed;
  float length;
} sc_path_t;

void sc_path_build(sc_path_t *p, const float *x, const float *y, float *s,
                   int n, int closed);

/* 按弧长取点。closed 时会自动环绕。 */
void sc_path_point_at(const sc_path_t *p, float s, float *x, float *y);

/* 按弧长取切向航向角 */
float sc_path_heading_at(const sc_path_t *p, float s);

/* 按弧长取曲率（1/m），用前后各 ds 的三点估算 */
float sc_path_curvature_at(const sc_path_t *p, float s, float ds);

/* 把点 (x,y) 投影到路径上。
 * 输出：
 *   s_out   投影点弧长
 *   ey_out  横向误差（正 = 车在路径左侧）
 *   epsi_out 航向误差 theta - path_heading，已归一化
 *   idx_out 投影所在线段起点索引（可为 NULL）
 * hint/window：从 hint 附近 window 条线段开始找，hint<0 时全局搜索。
 * 返回 0 成功。 */
int sc_path_project(const sc_path_t *p, float x, float y, float theta, int hint,
                    int window, float *s_out, float *ey_out, float *epsi_out,
                    int *idx_out);

/* 从 s_car 沿路径前进 ld 得到的预览点 */
void sc_path_lookahead(const sc_path_t *p, float s_car, float ld, float *x,
                       float *y, float *s_out);

/* 两个弧长之间的最短有向距离（处理闭环环绕） */
float sc_path_s_delta(const sc_path_t *p, float s_from, float s_to);

/* Catmull-Rom 样条平滑：把少量控制点插值成密集路径点。
 * out_x/out_y 至少能容纳 cn * samples_per_seg 个点（闭环）。
 * 返回生成的点数。 */
int sc_catmull_rom(const float *cx, const float *cy, int cn, int closed,
                   int samples_per_seg, float *out_x, float *out_y);

#endif
