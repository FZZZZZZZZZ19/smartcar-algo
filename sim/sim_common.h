/* 闭环仿真公共部分：赛道生成 + 三种横向控制器对比
 *
 * 车辆模型：运动学自行车模型（后轴参考）
 * 纵向：速度剖面 + 加速度限制
 * 横向：纯跟踪 / Stanley / LQR
 */
#ifndef SIM_COMMON_H
#define SIM_COMMON_H

#include "sc/common.h"
#include "sc/path.h"

#define SIM_TRACK_MAX 2048

typedef struct {
  float x[SIM_TRACK_MAX];
  float y[SIM_TRACK_MAX];
  float s[SIM_TRACK_MAX];
  float kappa[SIM_TRACK_MAX];
  float vprof[SIM_TRACK_MAX];
  int n;
  float length;
  sc_path_t path;
} sim_track_t;

typedef enum { SIM_CTRL_PP = 0, SIM_CTRL_STANLEY, SIM_CTRL_LQR } sim_ctrl_t;

typedef struct {
  float rms_ey;
  float max_ey;
  float rms_epsi;
  float mean_v;
  float max_v;
  float lap_time;
  int steps;
} sim_metrics_t;

/* 生成示例赛道（Catmull-Rom 控制点 -> 均匀重采样 -> 曲率 -> 速度剖面） */
void sim_track_build(sim_track_t *t);

/* 按弧长取目标速度（线性插值，闭环环绕） */
float sim_track_speed(const sim_track_t *t, float s);

/* 跑一圈仿真，写 CSV（csv_path 可为 NULL），填 metrics */
void sim_run(const sim_track_t *t, sim_ctrl_t ctrl, const char *csv_path,
             sim_metrics_t *m);

const char *sim_ctrl_name(sim_ctrl_t ctrl);

#endif
