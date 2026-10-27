/* 智能车电控算法学习工程 —— 公共定义
 *
 * 约定：
 *   长度 m，角度 rad，时间 s，速度 m/s，角速度 rad/s。
 *   车体坐标系：x 轴向前，y 轴向左，theta 为车头相对世界 x
 * 轴的转角（逆时针为正）。 舵机转角
 * delta：左转为正。所有模块统一使用这个符号约定。
 */
#ifndef SC_COMMON_H
#define SC_COMMON_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifndef SC_PI
#define SC_PI 3.14159265358979323846f
#endif
#define SC_TWO_PI (2.0f * SC_PI)
#define SC_HALF_PI (0.5f * SC_PI)
#define SC_DEG2RAD (SC_PI / 180.0f)
#define SC_RAD2DEG (180.0f / SC_PI)
#define SC_EPS 1e-6f

#define SC_ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

/* 平面位姿 */
typedef struct {
  float x;
  float y;
  float theta;
} sc_pose2_t;

typedef struct {
  float x;
  float y;
} sc_vec2_t;

/* 常用限幅宏（避免到处写 fminf/fmaxf） */
static inline float sc_clampf(float x, float lo, float hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

#endif /* SC_COMMON_H */
