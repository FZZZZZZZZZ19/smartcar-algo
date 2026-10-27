/* 电磁巡线：归一化、差比和、二次函数拟合、加权质心、方向判断
 *
 * 约定：水平电感按位置从左到右排列，pos 为相对赛道中心的横向坐标（左正）。
 * 电感读数随距离增大而减小，所以"离导线越近读数越大"。
 */
#ifndef SC_EMAG_H
#define SC_EMAG_H

#include "sc/common.h"

#define SC_EMAG_MAX 8

typedef struct {
  int n;
  float pos[SC_EMAG_MAX];  /* 传感器横向位置 m，左正右负 */
  float raw[SC_EMAG_MAX];  /* 原始 ADC */
  float norm[SC_EMAG_MAX]; /* 归一化 0..1 */

  float vmax[SC_EMAG_MAX]; /* 归一化用动态最大值 */
  float vmin[SC_EMAG_MAX]; /* 归一化用动态最小值 */
  float max_decay;         /* 最大值慢衰减系数，0.99~0.999 */
  float min_rise;          /* 最小值回升系数 */

  float sharpness; /* 加权质心的幂次，1=线性，2=更尖锐 */
} sc_emag_t;

void sc_emag_init(sc_emag_t *e, int n, const float *pos, float initial_max);

/* 每帧调用：更新归一化 */
void sc_emag_update(sc_emag_t *e, const float *raw);

/* 手动设置归一化上下限（已知标定值时用） */
void sc_emag_set_range(sc_emag_t *e, const float *vmin, const float *vmax);

/* 差比和：(l-r)/(l+r)，范围 [-1,1]，正=导线在左 */
float sc_emag_diff_ratio(float left, float right);

/* 二次函数拟合三电感：
 * 用 (pos_i, val_i) 拟合 y = a x^2 + b x + c，返回顶点 x* = -b/(2a)。
 * a 接近 0（三点近似共线）时退化为加权质心。 */
float sc_emag_quadratic_offset(const float pos[3], const float val[3]);

/* 加权质心，val^sharpness 作为权重 */
float sc_emag_weighted_centroid(const float *pos, const float *val, int n,
                                float sharpness);

/* 五电感常用解算：左右两组求和后差比和，再乘以 half_span 转成米。
 * 索引约定：0,1 左；2 中；3,4 右。 */
float sc_emag_offset_5(const sc_emag_t *e, float half_span);

/* 竖直电感方向判断：(up-down)/(up+down)，正=导线在前方 */
float sc_emag_direction(float up, float down);

/* 置信度：总能量越大越可信，用于元素判断时加权 */
float sc_emag_confidence(const sc_emag_t *e);

/* 仿真用：导线在 wire_y、传感器在 sensor_y 时的归一化场强。
 * 模型 E = h^2 / ((dy)^2 + h^2)，h 为电感离导线高度。 */
float sc_emag_sim_field(float sensor_y, float wire_y, float h);

#endif
