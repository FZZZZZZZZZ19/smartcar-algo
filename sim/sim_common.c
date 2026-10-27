#include "sim_common.h"

#include <stdio.h>

#include "sc/kinematics.h"
#include "sc/lqr.h"
#include "sc/math_utils.h"
#include "sc/pure_pursuit.h"
#include "sc/speed_planner.h"
#include "sc/stanley.h"

/* 赛道控制点：一个左右不对称的闭环，包含大小弯和一段近似直线 */
static const float CTRL_X[8] = {-6.0f, -1.0f, 4.0f,  6.0f,
                                5.0f,  1.0f,  -3.0f, -6.5f};
static const float CTRL_Y[8] = {-2.0f, -3.5f, -3.0f, -0.5f,
                                2.5f,  3.5f,  3.0f,  1.0f};

#define SIM_V_MAX 3.2f
#define SIM_A_LAT 3.5f
#define SIM_A_ACC 1.6f
#define SIM_A_DEC 2.2f
#define SIM_DT 0.005f
#define SIM_STEPS 4800 /* 24 s */
#define SIM_WARMUP 400 /* 前 2 s 不计指标 */

void sim_track_build(sim_track_t *t) {
  enum { SPLINE_N = 512, UNIFORM_N = 600 };

  static float raw_x[SPLINE_N];
  static float raw_y[SPLINE_N];
  static float raw_s[SPLINE_N];

  float *ux = t->x;
  float *uy = t->y;

  int sn = sc_catmull_rom(CTRL_X, CTRL_Y, 8, 1, 64, raw_x, raw_y);
  if (sn > SPLINE_N) {
    sn = SPLINE_N;
  }

  sc_path_t raw;
  sc_path_build(&raw, raw_x, raw_y, raw_s, sn, 1);

  /* 按等弧长重采样：速度剖面的差分假设 ds 均匀 */
  int n = UNIFORM_N;
  float ds = raw.length / (float)n;

  for (int i = 0; i < n; i++) {
    sc_path_point_at(&raw, (float)i * ds, &ux[i], &uy[i]);
  }

  sc_path_build(&t->path, ux, uy, t->s, n, 1);
  t->n = n;
  t->length = t->path.length;

  /* 每个点的曲率，窗口 0.25 m：足够稳定又不至于把弯道抹平 */
  for (int i = 0; i < n; i++) {
    t->kappa[i] = sc_path_curvature_at(&t->path, t->s[i], 0.25f);
  }

  sc_speed_profile_build(t->kappa, n, t->length / (float)n, SIM_V_MAX,
                         SIM_A_LAT, SIM_A_ACC, SIM_A_DEC, 1, t->vprof, 4);
}

float sim_track_speed(const sim_track_t *t, float s) {
  if (t->n <= 0) {
    return 0.0f;
  }

  float ds = t->length / (float)t->n;
  if (ds < 1e-9f) {
    return t->vprof[0];
  }

  float pos = s / ds;
  int i0 = (int)floorf(pos);
  float frac = pos - (float)i0;

  i0 = ((i0 % t->n) + t->n) % t->n;
  int i1 = (i0 + 1) % t->n;

  return sc_lerp(t->vprof[i0], t->vprof[i1], frac);
}

const char *sim_ctrl_name(sim_ctrl_t ctrl) {
  switch (ctrl) {
  case SIM_CTRL_PP:
    return "pure-pursuit";
  case SIM_CTRL_STANLEY:
    return "stanley";
  case SIM_CTRL_LQR:
    return "lqr";
  default:
    return "?";
  }
}

void sim_run(const sim_track_t *t, sim_ctrl_t ctrl, const char *csv_path,
             sim_metrics_t *m) {
  memset(m, 0, sizeof(*m));

  /* ---------- 车辆与控制器 ---------- */
  sc_ackermann_t car;
  sc_ackermann_init(&car, 0.33f, 0.20f, 0.55f);

  sc_pure_pursuit_t pp;
  sc_pp_init(&pp, 0.33f, 0.55f, 0.30f, 0.55f);
  pp.ld_min = 0.30f;
  pp.ld_max = 1.60f;

  sc_stanley_t st;
  sc_stanley_init(&st, 2.0f, 0.45f, 0.55f);
  sc_stanley_set_wheelbase(&st, 0.33f);

  sc_lqr_track_t lqr;
  sc_lqr_track_init(&lqr, 0.33f, 1.0f, SIM_DT, 1.0f, 1.5f, 0.15f, 0.55f);

  /* ---------- 初始状态 ---------- */
  sc_pose2_t pose;
  sc_path_point_at(&t->path, 0.0f, &pose.x, &pose.y);
  pose.theta = sc_path_heading_at(&t->path, 0.0f);

  float v = 1.0f;
  float delta = 0.0f;
  float delta_prev = 0.0f;
  float s = 0.0f;
  float s_prev = 0.0f;
  float traveled = 0.0f;
  int hint = 0;

  FILE *csv = NULL;
  if (csv_path) {
    csv = fopen(csv_path, "w");
    if (csv) {
      fprintf(csv, "t,x,y,theta,v,delta,ey,epsi,s,v_target,kappa\n");
    }
  }

  double sum_ey2 = 0.0;
  double sum_epsi2 = 0.0;
  double sum_v = 0.0;
  int count = 0;
  int lap_done = 0;

  for (int step = 0; step < SIM_STEPS; step++) {
    float tsec = (float)step * SIM_DT;

    /* 用上一帧索引做局部搜索（窗口 60 段 ≈ 3 m） */
    int idx = 0;
    float ey = 0.0f;
    float epsi = 0.0f;
    if (sc_path_project(&t->path, pose.x, pose.y, pose.theta, hint, 60, &s, &ey,
                        &epsi, &idx) != 0) {
      continue;
    }
    hint = idx;

    traveled += sc_path_s_delta(&t->path, s_prev, s);
    s_prev = s;

    if (!lap_done && traveled >= t->length) {
      m->lap_time = tsec;
      lap_done = 1;
    }

    float kappa = sc_path_curvature_at(&t->path, s, 0.25f);
    float v_target = sim_track_speed(t, s) * 0.95f;

    /* ---------- 纵向：P 控制 + 加速度限制 ---------- */
    float a_cmd = sc_clampf(4.0f * (v_target - v), -SIM_A_DEC, SIM_A_ACC);
    v += a_cmd * SIM_DT;
    if (v < 0.15f) {
      v = 0.15f;
    }

    /* ---------- 横向 ---------- */
    float delta_cmd = 0.0f;

    switch (ctrl) {
    case SIM_CTRL_PP: {
      float ld = sc_pp_lookahead(&pp, v);
      float tx, ty;
      sc_path_lookahead(&t->path, s, ld, &tx, &ty, NULL);
      delta_cmd = sc_pp_steer(&pp, &pose, tx, ty);
      break;
    }
    case SIM_CTRL_STANLEY:
      /* 用带曲率前馈的版本，否则弯道会有稳态误差 */
      delta_cmd = sc_stanley_steer_ff(&st, v, ey, epsi, kappa);
      break;
    case SIM_CTRL_LQR:
      /* 增益调度：速度变化超过 0.4 m/s 就重算 K */
      if (fabsf(v - lqr.v) > 0.4f) {
        sc_lqr_track_rebuild(&lqr, v);
      }
      delta_cmd = sc_lqr_track_update(&lqr, ey, epsi, kappa);
      break;
    default:
      break;
    }

    /* 舵机斜率限制 + 机械限位 */
    delta = sc_slew_limit(delta_prev, delta_cmd, 7.0f, SIM_DT);
    delta = sc_clampf(delta, -car.max_steer, car.max_steer);
    delta_prev = delta;

    /* ---------- 运动学积分 ---------- */
    sc_ackermann_integrate(&pose, &car, v, delta, SIM_DT);

    /* ---------- 指标 ---------- */
    if (step > SIM_WARMUP) {
      sum_ey2 += (double)ey * (double)ey;
      sum_epsi2 += (double)epsi * (double)epsi;
      sum_v += (double)v;
      count++;
      float aey = fabsf(ey);
      if (aey > m->max_ey) {
        m->max_ey = aey;
      }
      if (v > m->max_v) {
        m->max_v = v;
      }
    }

    if (csv) {
      fprintf(csv,
              "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.5f,%.5f,%.4f,%.4f,"
              "%.5f\n",
              (double)tsec, (double)pose.x, (double)pose.y, (double)pose.theta,
              (double)v, (double)delta, (double)ey, (double)epsi, (double)s,
              (double)v_target, (double)kappa);
    }
  }

  if (csv) {
    fclose(csv);
  }

  m->steps = count;
  if (count > 0) {
    m->rms_ey = sqrtf((float)(sum_ey2 / (double)count));
    m->rms_epsi = sqrtf((float)(sum_epsi2 / (double)count));
    m->mean_v = (float)(sum_v / (double)count);
  }
}
