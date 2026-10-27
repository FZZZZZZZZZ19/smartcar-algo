/* 自抗扰控制 ADRC：跟踪微分器 TD + 扩张状态观测器 ESO + 非线性状态误差反馈
 *
 * 韩京清经典结构：
 *   TD   ：安排过渡过程，给出光滑的 v1 和微分 v2
 *   ESO  ：用 z1,z2 观测输出及其导数，z3 估计"总扰动"
 *   NLSEF：u0 = kp*fal(e1) + kd*fal(e2)，再用 -z3/b0 补偿扰动
 */
#ifndef SC_ADRC_H
#define SC_ADRC_H

#include "sc/common.h"

typedef struct {
  float r; /* TD 跟踪速度因子，越大过渡越快 */
  float beta1;
  float beta2;
  float beta3;
  float b0; /* 控制增益估计（输入到输出的静态增益） */
  float kp;
  float kd;
  float a1;    /* fal 指数，通常 0.75 */
  float a2;    /* 通常 1.5 */
  float delta; /* fal 线性区宽度 */
  float out_min;
  float out_max;

  /* 状态 */
  float v1;
  float v2;
  float z1;
  float z2;
  float z3;
  float u_prev;

  int init;
} sc_adrc_t;

void sc_adrc_init(sc_adrc_t *a, float b0, float dt);

/* 用带宽法整定：wc 为控制器带宽，wo 为观测器带宽（通常 3~5 倍 wc） */
void sc_adrc_set_bandwidth(sc_adrc_t *a, float wc, float wo, float b0);

void sc_adrc_reset(sc_adrc_t *a);
void sc_adrc_set_limits(sc_adrc_t *a, float out_min, float out_max);

float sc_adrc_update(sc_adrc_t *a, float ref, float measurement, float dt);

#endif
