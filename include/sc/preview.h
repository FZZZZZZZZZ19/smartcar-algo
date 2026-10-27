/* 预瞄控制（Preview / Look-ahead PD）
 *
 * 思路：先按当前速度预测“再过 preview_time 秒车会到哪”，
 * 把预测点投影到路径上得到预瞄横向误差 e_y_p，
 * 再用 PD 把预瞄误差和当前航向误差压到 0：
 *
 *   delta = -k_y * e_y_p - k_psi * e_psi + atan(L * kappa)
 *
 * 和纯跟踪的区别：纯跟踪是几何圆弧，前视点在路径上；
 * 预瞄控制是线性 PD，参数物理意义更直观，很多高速队伍在用。
 */
#ifndef SC_PREVIEW_H
#define SC_PREVIEW_H

#include "sc/common.h"

typedef struct {
  float wheelbase;
  float preview_time; /* 预瞄时间 s，典型 0.3~0.6 */
  float k_y;          /* 预瞄横向误差增益 */
  float k_psi;        /* 航向误差增益 */
  float max_steer;
} sc_preview_t;

void sc_preview_init(sc_preview_t *p, float wheelbase, float preview_time,
                     float k_y, float k_psi, float max_steer);

/* 按当前速度预测 preview_time 后的位姿（假设车保持当前航向直行） */
void sc_preview_predict(const sc_preview_t *p, const sc_pose2_t *pose, float v,
                        sc_pose2_t *predicted);

/* e_y_preview 为正表示预测点在路径左侧；kappa_ref 用于曲率前馈 */
float sc_preview_steer(const sc_preview_t *p, float e_y_preview, float e_psi,
                       float kappa_ref);

#endif
