/* 路径跟踪 LQR
 *
 * 误差模型（后轴参考，小角度线性化）：
 *   e_y_dot   = v * e_psi
 *   e_psi_dot = v * delta / L - v * kappa_ref
 *
 * 离散化后对 [e_y, e_psi] 做 LQR，得到 delta = -K x + 前馈。
 * K 与速度有关：速度变化大时要重新计算（或做增益调度）。
 */
#ifndef SC_LQR_H
#define SC_LQR_H

#include "sc/common.h"
#include "sc/linalg.h"

typedef struct {
  float wheelbase;
  float v;       /* 线性化用的速度 */
  float dt;      /* 离散化周期 */
  float q_y;     /* 横向误差权重 */
  float q_psi;   /* 航向误差权重 */
  float r_delta; /* 转角代价 */

  float k_y;
  float k_psi;
  float max_steer;
  int iterations;

  sc_mat_t K;
} sc_lqr_track_t;

/* 计算 LQR 增益。返回 0 成功。 */
int sc_lqr_track_init(sc_lqr_track_t *lqr, float wheelbase, float v, float dt,
                      float q_y, float q_psi, float r_delta, float max_steer);

/* 速度改变后重新求解增益（增益调度） */
int sc_lqr_track_rebuild(sc_lqr_track_t *lqr, float v);

/* e_y 正=车在路径左侧，e_psi = theta - path_heading，kappa_ref 为路径曲率 */
float sc_lqr_track_update(const sc_lqr_track_t *lqr, float e_y, float e_psi,
                          float kappa_ref);

/* 平衡车 LQR：A、B 来自 sc_balance_state_space，返回 4 维反馈增益 */
int sc_lqr_balance(float cart_mass, float pole_mass, float pole_half,
                   float q_pos, float q_vel, float q_angle, float q_angle_rate,
                   float r_force, float K[4]);

#endif
