/* 通用滤波器：一阶低通、滑动平均、中值、斜率限制、卡尔曼 */
#ifndef SC_FILTER_H
#define SC_FILTER_H

#include "sc/common.h"

/* ---------- 一阶低通 ---------- */
typedef struct {
  float y;
  int init;
} sc_lpf1_t;

void sc_lpf1_reset(sc_lpf1_t *f, float y0);
float sc_lpf1_step(sc_lpf1_t *f, float x, float alpha);

/* ---------- 滑动平均 ---------- */
#define SC_FILTER_MAX 32

typedef struct {
  float buf[SC_FILTER_MAX];
  int n;
  int idx;
  int count;
  float sum;
} sc_ma_t;

void sc_ma_init(sc_ma_t *f, int n);
void sc_ma_reset(sc_ma_t *f, float y0);
float sc_ma_step(sc_ma_t *f, float x);

/* ---------- 中值滤波 ---------- */
typedef struct {
  float buf[SC_FILTER_MAX];
  int n;
  int idx;
  int count;
} sc_median_t;

void sc_median_init(sc_median_t *f, int n);
void sc_median_reset(sc_median_t *f, float y0);
float sc_median_step(sc_median_t *f, float x);

/* ---------- 斜率限制 ---------- */
typedef struct {
  float y;
  float max_rate; /* 单位/秒 */
  int init;
} sc_rate_lim_t;

void sc_rate_lim_init(sc_rate_lim_t *f, float max_rate, float y0);
float sc_rate_lim_step(sc_rate_lim_t *f, float target, float dt);

/* ---------- 一维卡尔曼 ---------- */
/* 模型：x_{k+1} = x_k + w,  z = x + v
 * 适合温度、静止零漂、单传感器测距等。 */
typedef struct {
  float q; /* 过程噪声 */
  float r; /* 测量噪声 */
  float p; /* 估计方差 */
  float x; /* 状态 */
  int init;
} sc_kf1_t;

void sc_kf1_init(sc_kf1_t *f, float q, float r, float x0, float p0);
float sc_kf1_step(sc_kf1_t *f, float z);

/* ---------- 二维卡尔曼（位置 + 速度，匀加速噪声模型） ---------- */
typedef struct {
  float x[2]; /* [位置, 速度] */
  float P[2][2];
  float q; /* 加速度功率谱密度 */
  float r; /* 位置测量噪声 */
  int init;
} sc_kf2_t;

void sc_kf2_init(sc_kf2_t *f, float q, float r, float z0);
void sc_kf2_predict(sc_kf2_t *f, float dt);
float sc_kf2_update(sc_kf2_t *f, float z);
float sc_kf2_pos(const sc_kf2_t *f);
float sc_kf2_vel(const sc_kf2_t *f);

#endif
