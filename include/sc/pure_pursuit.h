/* 纯跟踪 Pure Pursuit
 *
 * 在路径上取一个前视点，用车体到前视点的圆弧去逼近：
 *   alpha = atan2(dy, dx) - theta
 *   delta = atan2(2 * L * sin(alpha), ld)
 *
 * 前视距离 ld 是唯一关键参数：
 *   ld 小 → 贴线紧、容易抖；ld 大 → 平滑、切弯。
 * 常用自适应：ld = ld_base + ld_gain * v。
 */
#ifndef SC_PURE_PURSUIT_H
#define SC_PURE_PURSUIT_H

#include "sc/common.h"

typedef struct {
  float wheelbase;
  float ld_base;
  float ld_gain;
  float ld_min;
  float ld_max;
  float max_steer;
} sc_pure_pursuit_t;

void sc_pp_init(sc_pure_pursuit_t *pp, float wheelbase, float ld_base,
                float ld_gain, float max_steer);

/* 自适应前视距离 */
float sc_pp_lookahead(const sc_pure_pursuit_t *pp, float v);

/* 由车体位姿和前视点求前轮转角 */
float sc_pp_steer(const sc_pure_pursuit_t *pp, const sc_pose2_t *pose,
                  float target_x, float target_y);

/* 直接用 alpha、ld 计算，便于单元测试和自定义前视点 */
float sc_pp_steer_alpha(const sc_pure_pursuit_t *pp, float alpha, float ld);

#endif
