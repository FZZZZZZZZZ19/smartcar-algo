/* 数学小工具：角度归一化、限幅、斜率限制、低通系数换算 */
#ifndef SC_MATH_UTILS_H
#define SC_MATH_UTILS_H

#include "sc/common.h"

/* 把角度归一化到 [-pi, pi) */
float sc_wrap_pi(float a);

/* 把角度归一化到 [0, 2pi) */
float sc_wrap_2pi(float a);

/* 两个角度之间的最短差值 a-b，结果在 [-pi, pi) */
float sc_angle_diff(float a, float b);

/* 带死区的符号处理：|x| <= db 时输出 0，否则保留符号并减去死区 */
float sc_deadband(float x, float db);

/* 一阶低通系数：给定时间常数 tau 和采样周期 dt */
float sc_lpf_alpha_tau(float tau, float dt);

/* 一阶低通系数：给定截止频率 fc(Hz) 和采样周期 dt */
float sc_lpf_alpha_fc(float fc, float dt);

/* 斜率限制：把 current 朝 target 移动，最多移动 max_rate*dt。
 * 用于舵机、电机、目标速度的平滑。 */
float sc_slew_limit(float current, float target, float max_rate, float dt);

/* 与 sc_slew_limit 等价，但不受 dt 影响（每步最多 max_step） */
float sc_move_towards(float current, float target, float max_step);

/* 线性插值 */
float sc_lerp(float a, float b, float t);

/* 三次平滑插值（t 在 [0,1]） */
float sc_smoothstep(float t);

/* 浮点近似相等 */
int sc_almost_equal(float a, float b, float tol);

/* 归一化到 [0,1]，分母过小时返回 0.5 */
float sc_norm01(float x, float lo, float hi);

/* 由三点求曲率（有向，逆时针为正）。
 * 返回 1/kappa 的半径；三点共线时返回 FLT_MAX。 */
float sc_curvature_3pt(float x0, float y0, float x1, float y1, float x2,
                       float y2);

#endif
