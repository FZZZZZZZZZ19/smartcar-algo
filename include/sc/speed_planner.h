/* 速度规划：曲率限速、全局速度剖面、梯形规划、在线 S 型（加加速度限制）
 */
#ifndef SC_SPEED_PLANNER_H
#define SC_SPEED_PLANNER_H

#include "sc/common.h"

/* 曲率限速：v = sqrt(a_lat / |kappa|)，直线时返回 v_max */
float sc_speed_limit_from_curvature(float kappa, float a_lat_max, float v_max);

/* 沿路径生成速度剖面。
 * kappa  : 每点的曲率数组（1/m）
 * ds     : 相邻点弧长间隔
 * closed : 是否闭环（闭环会多迭代几次让首尾约束收敛）
 * v_out  : 输出速度数组（与 kappa 等长）
 *
 * 做法：
 *   1) 每点按横向加速度限速
 *   2) 反向扫描，保证减速度不超过 a_dec
 *   3) 正向扫描，保证加速度不超过 a_acc
 */
void sc_speed_profile_build(const float *kappa, int n, float ds, float v_max,
                            float a_lat, float a_acc, float a_dec, int closed,
                            float *v_out, int passes);

/* ---------------- 梯形速度规划（已知距离） ---------------- */

typedef struct {
  float distance;
  float v_peak;
  float s_acc;
  float s_cruise;
  float s_dec;
  int feasible; /* 0 表示距离太短，无法在 v0/v1 之间完成规划 */
} sc_trap_t;

void sc_trap_plan(sc_trap_t *t, float distance, float v0, float v1, float v_max,
                  float a_acc, float a_dec);

/* 按已行驶距离取速度 */
float sc_trap_speed_at_distance(const sc_trap_t *t, float s, float v0, float v1,
                                float a_acc, float a_dec);

/* ---------------- 在线 S 型速度规划 ---------------- */

/* 不预先知道距离时，用加加速度限制把速度平滑地推向目标：
 *   a_des 由速度误差得到，a 受 j_max 限制，v 再积分。
 * 好处是起步/刹车没有加速度突变，轮胎不容易打滑。 */
typedef struct {
  float v;
  float a;
  float a_max;
  float j_max;
  int init;
} sc_scurve_t;

void sc_scurve_init(sc_scurve_t *s, float a_max, float j_max, float v0);
void sc_scurve_reset(sc_scurve_t *s, float v0);
float sc_scurve_step(sc_scurve_t *s, float v_target, float dt);

#endif
