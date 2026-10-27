/* 姿态解算：加速度倾角、互补滤波、Mahony、四元数、零漂校准
 *
 * 坐标系：右手系，x 前，y 左，z 上（机体系）。
 * 陀螺仪单位 rad/s，加速度单位任意（内部会归一化）。
 */
#ifndef SC_ATTITUDE_H
#define SC_ATTITUDE_H

#include "sc/common.h"

/* ---------------- 加速度计直接求倾角 ---------------- */

/* 俯仰角：车头抬起为正。ax 前，ay 左，az 上 */
float sc_accel_pitch(float ax, float ay, float az);

/* 横滚角：向左倾为正 */
float sc_accel_roll(float ax, float ay, float az);

/* ---------------- 一阶互补滤波 ---------------- */

typedef struct {
  float angle; /* 融合后的角度 rad */
  float alpha; /* 0..1，越接近 1 越相信陀螺 */
} sc_comp_t;

void sc_comp_init(sc_comp_t *f, float angle, float alpha);
void sc_comp_reset(sc_comp_t *f, float angle);
float sc_comp_update(sc_comp_t *f, float gyro_rate, float acc_angle, float dt);

/* ---------------- 四元数工具 ---------------- */

void sc_quat_normalize(float q[4]); /* q = [w, x, y, z] */
void sc_quat_to_euler(const float q[4], float *roll, float *pitch, float *yaw);
void sc_quat_to_rot(const float q[4], float R[3][3]);

/* ---------------- Mahony 互补滤波 ---------------- */

typedef struct {
  float q[4]; /* [w, x, y, z] */
  float kp;
  float ki;
  float integral[3]; /* 陀螺零漂积分 */
  int init;
} sc_mahony_t;

void sc_mahony_init(sc_mahony_t *m, float kp, float ki);
void sc_mahony_reset(sc_mahony_t *m, float roll, float pitch, float yaw);
void sc_mahony_update(sc_mahony_t *m, float gx, float gy, float gz, float ax,
                      float ay, float az, float dt);
void sc_mahony_euler(const sc_mahony_t *m, float *roll, float *pitch,
                     float *yaw);

/* ---------------- 陀螺零漂校准 ---------------- */

typedef struct {
  double sum[3];
  int count;
  int target;
  float bias[3];
  int done;
} sc_gyro_cal_t;

void sc_gyro_cal_init(sc_gyro_cal_t *c, int samples);
/* 静止时持续喂数据，返回 1 表示校准完成 */
int sc_gyro_cal_feed(sc_gyro_cal_t *c, float gx, float gy, float gz,
                     float acc_norm, float g_ref);
void sc_gyro_cal_apply(const sc_gyro_cal_t *c, float *gx, float *gy, float *gz);

#endif
