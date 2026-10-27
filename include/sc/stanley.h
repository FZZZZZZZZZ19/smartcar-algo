/* Stanley 横向控制
 *
 *   delta = -e_psi - atan(k * e_y / (|v| + soft))
 *
 * 第一项对准路径切向（航向误差），第二项把车拉回中心线（横向误差）。
 * 与纯跟踪的区别：Stanley 直接用横向误差，没有前视距离，
 * 低速贴线更好，但对噪声更敏感，需要滤波。
 *
 * 符号：e_y > 0 表示车在路径左侧；e_psi = theta - path_heading。
 */
#ifndef SC_STANLEY_H
#define SC_STANLEY_H

#include "sc/common.h"

typedef struct {
  float k_gain;     /* 横向误差增益，典型 1~3 */
  float soft_speed; /* 低速软化，避免 v→0 时 atan 爆炸，典型 0.3~1.0 */
  float max_steer;
  float wheelbase;  /* 用于曲率前馈；<=0 时关闭前馈 */
} sc_stanley_t;

void sc_stanley_init(sc_stanley_t *s, float k_gain, float soft_speed,
                     float max_steer);

/* 经典 Stanley（无曲率前馈） */
float sc_stanley_steer(const sc_stanley_t *s, float v, float e_y, float e_psi);

/* 带曲率前馈：delta += atan(L*kappa)，弯道上消除稳态误差。
 * 前提是 sc_stanley_set_wheelbase 设置过 wheelbase。 */
void sc_stanley_set_wheelbase(sc_stanley_t *s, float wheelbase);
float sc_stanley_steer_ff(const sc_stanley_t *s, float v, float e_y,
                          float e_psi, float kappa);

#endif
