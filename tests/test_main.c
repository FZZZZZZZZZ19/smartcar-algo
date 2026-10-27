/* 单元测试：把每个算法模块单独跑一遍
 * 运行：make test
 */
#include <stdio.h>
#include <string.h>

#include "sc/adrc.h"
#include "sc/attitude.h"
#include "sc/camera_line.h"
#include "sc/emag.h"
#include "sc/filter.h"
#include "sc/fuzzy_pid.h"
#include "sc/kinematics.h"
#include "sc/linalg.h"
#include "sc/lqr.h"
#include "sc/math_utils.h"
#include "sc/motor.h"
#include "sc/path.h"
#include "sc/pid.h"
#include "sc/preview.h"
#include "sc/pure_pursuit.h"
#include "sc/servo.h"
#include "sc/speed_planner.h"
#include "sc/stanley.h"

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (cond) {                                                                \
      g_pass++;                                                                \
    } else {                                                                   \
      g_fail++;                                                                \
      printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);                   \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(a, b, tol)                                                  \
  do {                                                                         \
    float _a = (a);                                                            \
    float _b = (b);                                                            \
    if (fabsf(_a - _b) <= (tol)) {                                             \
      g_pass++;                                                                \
    } else {                                                                   \
      g_fail++;                                                                \
      printf("FAIL %s:%d  |%.6f - %.6f| > %.6f\n", __FILE__, __LINE__,         \
             (double)_a, (double)_b, (double)(tol));                           \
    }                                                                          \
  } while (0)

static void test_math(void) {
  printf("-- math_utils\n");

  CHECK_NEAR(sc_wrap_pi(3.0f * SC_PI), -SC_PI, 1e-4f);
  CHECK_NEAR(sc_wrap_pi(SC_PI + 0.1f), -SC_PI + 0.1f, 1e-4f);
  CHECK_NEAR(sc_angle_diff(0.1f, 6.2f), 0.1f - 6.2f + SC_TWO_PI, 1e-3f);
  CHECK_NEAR(sc_deadband(0.05f, 0.1f), 0.0f, 1e-6f);
  CHECK_NEAR(sc_deadband(0.3f, 0.1f), 0.2f, 1e-6f);
  CHECK_NEAR(sc_move_towards(0.0f, 1.0f, 0.3f), 0.3f, 1e-6f);
  CHECK_NEAR(sc_lerp(0.0f, 2.0f, 0.25f), 0.5f, 1e-6f);

  /* 单位圆上三点，曲率半径应为 1 */
  float r = sc_curvature_3pt(1.0f, 0.0f, 0.0f, 1.0f, -1.0f, 0.0f);
  CHECK_NEAR(r, 1.0f, 1e-3f);
  CHECK(r > 0.0f);
}

static void test_filter(void) {
  printf("-- filter\n");

  sc_lpf1_t lpf;
  sc_lpf1_reset(&lpf, 0.0f);
  for (int i = 0; i < 200; i++) {
    sc_lpf1_step(&lpf, 5.0f, 0.9f);
  }
  CHECK_NEAR(lpf.y, 5.0f, 1e-3f);

  sc_ma_t ma;
  sc_ma_init(&ma, 3);
  sc_ma_step(&ma, 1.0f);
  sc_ma_step(&ma, 1.0f);
  CHECK_NEAR(sc_ma_step(&ma, 4.0f), 2.0f, 1e-6f);

  sc_median_t med;
  sc_median_init(&med, 3);
  sc_median_step(&med, 1.0f);
  sc_median_step(&med, 1.0f);
  CHECK_NEAR(sc_median_step(&med, 100.0f), 1.0f, 1e-6f);

  sc_rate_lim_t rl;
  sc_rate_lim_init(&rl, 2.0f, 0.0f);
  CHECK_NEAR(sc_rate_lim_step(&rl, 10.0f, 0.1f), 0.2f, 1e-6f);

  sc_kf1_t kf;
  sc_kf1_init(&kf, 0.01f, 1.0f, 0.0f, 1.0f);
  for (int i = 0; i < 200; i++) {
    sc_kf1_step(&kf, 5.0f);
  }
  CHECK_NEAR(kf.x, 5.0f, 0.05f);

  sc_kf2_t kf2;
  sc_kf2_init(&kf2, 0.5f, 0.05f, 0.0f);
  float t = 0.0f;
  for (int i = 0; i < 500; i++) {
    t += 0.01f;
    sc_kf2_predict(&kf2, 0.01f);
    sc_kf2_update(&kf2, 2.0f * t);
  }
  CHECK_NEAR(sc_kf2_vel(&kf2), 2.0f, 0.2f);
}

static void test_kinematics(void) {
  printf("-- kinematics\n");

  sc_diff_drive_t d;
  sc_diff_init(&d, 0.05f, 1.0f, 1.0f, 1000.0f);

  float vl, vr;
  sc_diff_inverse(&d, 1.0f, 0.0f, &vl, &vr);
  CHECK_NEAR(vl, 1.0f, 1e-6f);
  CHECK_NEAR(vr, 1.0f, 1e-6f);

  float v, w;
  sc_diff_forward(&d, 0.5f, 1.5f, &v, &w);
  CHECK_NEAR(v, 1.0f, 1e-6f);
  CHECK_NEAR(w, 1.0f, 1e-6f);

  /* 编码器：转一圈 2πr，1 秒 */
  float ws = sc_diff_wheel_speed(&d, 1000, 1.0f);
  CHECK_NEAR(ws, SC_TWO_PI * 0.05f, 1e-4f);

  /* 差速原地画圆：v=1, w=1, 2π 秒后回到原点 */
  sc_pose2_t pose = {0.0f, 0.0f, 0.0f};
  for (int i = 0; i < 6284; i++) {
    sc_diff_odometry(&pose, &d, 0.5f, 1.5f, 0.001f);
  }
  CHECK_NEAR(pose.x, 0.0f, 0.02f);
  CHECK_NEAR(pose.y, 0.0f, 0.02f);

  /* 阿克曼内外轮转角 */
  sc_ackermann_t a;
  sc_ackermann_init(&a, 0.3f, 0.2f, 0.6f);
  float inner, outer;
  sc_ackermann_wheel_angles(&a, 0.3f, &inner, &outer);
  CHECK(inner > 0.3f);
  CHECK(outer > 0.0f && outer < 0.3f);
  CHECK(inner > outer);

  sc_ackermann_wheel_angles(&a, -0.3f, &inner, &outer);
  CHECK(inner < -0.3f);
  CHECK(outer > -0.3f && outer < 0.0f);

  sc_pose2_t p2 = {0.0f, 0.0f, 0.0f};
  sc_ackermann_integrate(&p2, &a, 1.0f, 0.0f, 1.0f);
  CHECK_NEAR(p2.x, 1.0f, 1e-5f);
  CHECK_NEAR(p2.y, 0.0f, 1e-5f);

  /* 麦克纳姆轮正逆解往返 */
  sc_mecanum_t m;
  sc_mecanum_init(&m, 0.05f, 0.15f, 0.20f);
  float wheel[4];
  sc_mecanum_inverse(&m, 0.5f, 0.3f, 0.4f, wheel);
  float vx, vy, wz;
  sc_mecanum_forward(&m, wheel, &vx, &vy, &wz);
  CHECK_NEAR(vx, 0.5f, 1e-5f);
  CHECK_NEAR(vy, 0.3f, 1e-5f);
  CHECK_NEAR(wz, 0.4f, 1e-5f);

  /* 三轮全向正逆解往返 */
  sc_omni3_t o;
  sc_omni3_init_default(&o, 0.05f);
  float ow[3];
  sc_omni3_inverse(&o, 0.4f, -0.2f, 0.3f, ow);
  sc_omni3_forward(&o, ow, &vx, &vy, &wz);
  CHECK_NEAR(vx, 0.4f, 1e-5f);
  CHECK_NEAR(vy, -0.2f, 1e-5f);
  CHECK_NEAR(wz, 0.3f, 1e-5f);

  /* 平衡车模型：倾角项应为正 */
  sc_balance_t b = {1.0f, 0.5f, 0.3f, 9.81f};
  float A[4][4];
  float B[4];
  sc_balance_state_space(&b, A, B);
  CHECK(A[3][2] > 0.0f);
  CHECK(B[1] > 0.0f);
}

static void test_camera(void) {
  printf("-- camera_line\n");

  enum { W = 160, H = 120 };
  static uint8_t img[W * H];
  static uint8_t bin[W * H];
  static uint8_t visited[W * H];

  /* 合成一张"近宽远窄"的直道图 */
  for (int y = 0; y < H; y++) {
    float half = 12.0f + 18.0f * (float)y / (float)(H - 1);
    for (int x = 0; x < W; x++) {
      img[y * W + x] = (fabsf((float)x - 80.0f) <= half) ? 220 : 30;
    }
  }

  sc_image_t im = {img, W, H};
  uint8_t th = sc_otsu(&im);
  CHECK(th >= 30 && th < 220);

  sc_binarize(&im, bin, th, 0);

  sc_edge_t e;
  int rows = sc_scan_edges(bin, W, H, H - 1, -1, 80, W / 2, 5, 5, &e);
  CHECK(rows > 50);
  CHECK_NEAR((float)e.center[0], 80.0f, 2.0f);
  CHECK(e.width[0] > e.width[rows - 1]);

  sc_track_feat_t f;
  sc_track_features(&e, &f);
  CHECK(fabsf(f.curvature_px) < 0.002f);
  CHECK(sc_classify_element(&f, 0.002f, 0.5f) == SC_ELEM_STRAIGHT);

  /* 弯道：中线随行号二次偏移 */
  for (int y = 0; y < H; y++) {
    float shift = 0.003f * (float)((H - 1 - y) * (H - 1 - y));
    float half = 12.0f + 18.0f * (float)y / (float)(H - 1);
    for (int x = 0; x < W; x++) {
      img[y * W + x] = (fabsf((float)x - (80.0f + shift)) <= half) ? 220 : 30;
    }
  }
  sc_binarize(&im, bin, th, 0);
  sc_scan_edges(bin, W, H, H - 1, -1, 60, W / 2, 5, 5, &e);
  sc_track_features(&e, &f);
  CHECK(fabsf(f.curvature_px) > 0.001f);
  CHECK(sc_classify_element(&f, 0.001f, 0.5f) != SC_ELEM_STRAIGHT);

  /* 八邻域爬线：画一条从底部到顶部的斜线 */
  memset(bin, 0, sizeof(bin));
  memset(visited, 0, sizeof(visited));
  for (int y = H - 1; y >= 0; y--) {
    int x = 80 + (H - 1 - y) / 2;
    for (int dx = -1; dx <= 1; dx++) {
      if (x + dx >= 0 && x + dx < W) {
        bin[y * W + x + dx] = 1;
      }
    }
  }
  sc_point_t pts[200];
  int n = sc_trace_8(bin, visited, W, H, 80, H - 1, pts, 200);
  CHECK(n > 100);
  CHECK(pts[n - 1].y < 20);
  CHECK(pts[n - 1].x > pts[0].x);

  /* IPM：同一列，越靠下（v 越大）离车越近 */
  sc_camera_t cam;
  sc_camera_init(&cam, 0.5f, 10.0f * SC_DEG2RAD, 200.0f, 80.0f, 60.0f);

  float x_near, y_near, x_far, y_far, x_mid, y_mid;
  CHECK(sc_ipm_pixel_to_ground(&cam, 80, 110, &x_near, &y_near) == 0);
  CHECK(sc_ipm_pixel_to_ground(&cam, 80, 50, &x_far, &y_far) == 0);
  CHECK(sc_ipm_pixel_to_ground(&cam, 80, 60, &x_mid, &y_mid) == 0);

  CHECK(x_near > 0.0f);
  CHECK(x_near < x_mid);
  CHECK(x_mid < x_far);
  CHECK_NEAR(fabsf(y_near), 0.0f, 1e-4f);
  CHECK_NEAR(x_mid, 0.5f / tanf(10.0f * SC_DEG2RAD), 1e-3f);

  /* 地面 -> 像素 -> 地面 往返 */
  float u, vv, gx, gy;
  CHECK(sc_ipm_ground_to_pixel(&cam, 2.0f, 0.5f, &u, &vv) == 0);
  CHECK(sc_ipm_pixel_to_ground(&cam, u, vv, &gx, &gy) == 0);
  CHECK_NEAR(gx, 2.0f, 1e-3f);
  CHECK_NEAR(gy, 0.5f, 1e-3f);
}

static void test_emag(void) {
  printf("-- emag\n");

  CHECK_NEAR(sc_emag_sim_field(0.0f, 0.0f, 0.05f), 1.0f, 1e-6f);
  CHECK_NEAR(sc_emag_diff_ratio(3.0f, 1.0f), 0.5f, 1e-6f);
  CHECK_NEAR(sc_emag_diff_ratio(1.0f, 3.0f), -0.5f, 1e-6f);
  CHECK_NEAR(sc_emag_diff_ratio(0.0f, 0.0f), 0.0f, 1e-6f);

  /* 二次拟合：导线在 0.02 m 处 */
  float pos[3] = {-0.05f, 0.0f, 0.05f};
  float val[3];
  for (int i = 0; i < 3; i++) {
    val[i] = sc_emag_sim_field(pos[i], 0.02f, 0.05f);
  }
  CHECK_NEAR(sc_emag_quadratic_offset(pos, val), 0.02f, 0.005f);

  /* 加权质心 */
  float v2[3] = {0.2f, 1.0f, 0.2f};
  CHECK_NEAR(sc_emag_weighted_centroid(pos, v2, 3, 1.0f), 0.0f, 1e-5f);

  /* 归一化 */
  sc_emag_t em;
  sc_emag_init(&em, 3, pos, 100.0f);
  float raw[3] = {100.0f, 50.0f, 0.0f};
  sc_emag_update(&em, raw);
  CHECK_NEAR(em.norm[0], 1.0f, 1e-3f);
  CHECK_NEAR(em.norm[1], 0.5f, 1e-3f);
  CHECK_NEAR(em.norm[2], 0.0f, 1e-3f);

  /* 五电感：导线偏左 -> 偏移为正 */
  float p5[5] = {0.16f, 0.08f, 0.0f, -0.08f, -0.16f};
  sc_emag_t e5;
  sc_emag_init(&e5, 5, p5, 1.0f);
  float r5[5];
  for (int i = 0; i < 5; i++) {
    r5[i] = sc_emag_sim_field(p5[i], 0.06f, 0.05f);
  }
  sc_emag_update(&e5, r5);
  CHECK(sc_emag_offset_5(&e5, 0.2f) > 0.0f);
}

static void test_attitude(void) {
  printf("-- attitude\n");

  CHECK_NEAR(sc_accel_pitch(0.0f, 0.0f, 1.0f), 0.0f, 1e-6f);
  CHECK_NEAR(sc_accel_roll(0.0f, 0.0f, 1.0f), 0.0f, 1e-6f);
  CHECK(sc_accel_pitch(-0.5f, 0.0f, 1.0f) > 0.0f);

  sc_comp_t c;
  sc_comp_init(&c, 0.0f, 0.98f);
  for (int i = 0; i < 1000; i++) {
    sc_comp_update(&c, 0.0f, 0.3f, 0.01f);
  }
  CHECK_NEAR(c.angle, 0.3f, 0.01f);

  sc_mahony_t m;
  sc_mahony_init(&m, 0.5f, 0.05f);
  for (int i = 0; i < 2000; i++) {
    sc_mahony_update(&m, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.005f);
  }
  float roll, pitch, yaw;
  sc_mahony_euler(&m, &roll, &pitch, &yaw);
  CHECK_NEAR(roll, 0.0f, 0.01f);
  CHECK_NEAR(pitch, 0.0f, 0.01f);

  sc_gyro_cal_t cal;
  sc_gyro_cal_init(&cal, 100);
  int done = 0;
  for (int i = 0; i < 100; i++) {
    done = sc_gyro_cal_feed(&cal, 0.01f, -0.02f, 0.003f, 1.0f, 1.0f);
  }
  CHECK(done == 1);
  CHECK_NEAR(cal.bias[0], 0.01f, 1e-6f);
  CHECK_NEAR(cal.bias[1], -0.02f, 1e-6f);
}

static void test_pid(void) {
  printf("-- pid / fuzzy / adrc\n");

  /* 一阶被控对象 y' = (u-y)/tau，PID 应无静差 */
  sc_pid_t pid;
  sc_pid_init(&pid, 2.0f, 4.0f, 0.02f);
  sc_pid_set_limits(&pid, -10.0f, 10.0f);

  float y = 0.0f;
  float dt = 0.01f;
  for (int i = 0; i < 800; i++) {
    float u = sc_pid_update(&pid, 1.0f, y, dt);
    y += dt * (u - y) / 0.2f;
  }
  CHECK_NEAR(y, 1.0f, 0.02f);

  /* 抗积分饱和：输出不应超过限幅 */
  sc_pid_t pid2;
  sc_pid_init(&pid2, 0.0f, 10.0f, 0.0f);
  sc_pid_set_limits(&pid2, 0.0f, 1.0f);
  float out = 0.0f;
  for (int i = 0; i < 200; i++) {
    out = sc_pid_update(&pid2, 10.0f, 0.0f, 0.01f);
  }
  CHECK(out <= 1.0f + 1e-5f);
  CHECK(pid2.integ <= 1.0f + 1e-5f);

  /* 增量式：误差不变时输出保持不变 */
  sc_pid_t pid3;
  sc_pid_init(&pid3, 1.0f, 0.0f, 0.0f);
  sc_pid_set_form(&pid3, SC_PID_INCREMENTAL);
  sc_pid_set_limits(&pid3, -100.0f, 100.0f);
  float u1 = sc_pid_update(&pid3, 1.0f, 0.0f, 0.01f);
  float u2 = sc_pid_update(&pid3, 1.0f, 0.0f, 0.01f);
  CHECK_NEAR(u1, 1.0f, 1e-4f);
  CHECK_NEAR(u2, u1, 1e-4f);

  /* 死区 */
  sc_pid_t pid4;
  sc_pid_init(&pid4, 100.0f, 0.0f, 0.0f);
  sc_pid_set_deadband(&pid4, 0.1f);
  CHECK_NEAR(sc_pid_update(&pid4, 0.05f, 0.0f, 0.01f), 0.0f, 1e-6f);

  /* 模糊推理：零点输出为 0，且输出有界 */
  CHECK_NEAR(sc_fuzzy_dkp(0.0f, 0.0f), 0.0f, 1e-4f);
  CHECK_NEAR(sc_fuzzy_dki(0.0f, 0.0f), 0.0f, 1e-4f);
  /* 经典 Kd 规则表在 (ZO,ZO) 处取 NS，所以零点有一个 -1 的偏置 */
  CHECK_NEAR(sc_fuzzy_dkd(0.0f, 0.0f), -1.0f, 0.2f);

  for (float e = -3.0f; e <= 3.0f; e += 0.5f) {
    for (float ec = -3.0f; ec <= 3.0f; ec += 0.5f) {
      float a = sc_fuzzy_dkp(e, ec);
      float b = sc_fuzzy_dki(e, ec);
      float cc = sc_fuzzy_dkd(e, ec);
      CHECK(a >= -3.1f && a <= 3.1f);
      CHECK(b >= -3.1f && b <= 3.1f);
      CHECK(cc >= -3.1f && cc <= 3.1f);
    }
  }

  /* 模糊 PID 闭环 */
  sc_fuzzy_pid_t fp;
  sc_fuzzy_pid_init(&fp, 2.0f, 3.0f, 0.02f, 1.0f, 5.0f);
  sc_fuzzy_pid_set_limits(&fp, -10.0f, 10.0f);
  y = 0.0f;
  for (int i = 0; i < 1000; i++) {
    float u = sc_fuzzy_pid_update(&fp, 1.0f, y, dt);
    y += dt * (u - y) / 0.2f;
  }
  CHECK_NEAR(y, 1.0f, 0.05f);

  /* ADRC：一阶对象 y' = -y + u */
  sc_adrc_t adrc;
  sc_adrc_init(&adrc, 1.0f, dt);
  sc_adrc_set_bandwidth(&adrc, 3.0f, 8.0f, 1.0f);
  sc_adrc_set_limits(&adrc, -20.0f, 20.0f);

  y = 0.0f;
  for (int i = 0; i < 1000; i++) {
    float u = sc_adrc_update(&adrc, 1.0f, y, dt);
    y += dt * (-y + u);
  }
  CHECK_NEAR(y, 1.0f, 0.05f);
}

static void test_path_tracking(void) {
  printf("-- path / pure_pursuit / stanley / lqr\n");

  float px[3] = {0.0f, 5.0f, 10.0f};
  float py[3] = {0.0f, 0.0f, 0.0f};
  float ps[3];
  sc_path_t path;
  sc_path_build(&path, px, py, ps, 3, 0);

  CHECK_NEAR(path.length, 10.0f, 1e-5f);

  float s, ey, epsi;
  int idx;
  CHECK(sc_path_project(&path, 5.0f, 1.0f, 0.0f, -1, 0, &s, &ey, &epsi, &idx) ==
        0);
  CHECK_NEAR(s, 5.0f, 0.01f);
  CHECK_NEAR(ey, 1.0f, 0.01f);
  CHECK_NEAR(epsi, 0.0f, 0.01f);

  CHECK(sc_path_project(&path, 5.0f, -1.0f, 0.0f, -1, 0, &s, &ey, &epsi,
                        &idx) == 0);
  CHECK_NEAR(ey, -1.0f, 0.01f);

  float x, yy;
  sc_path_point_at(&path, 3.0f, &x, &yy);
  CHECK_NEAR(x, 3.0f, 0.01f);
  CHECK_NEAR(yy, 0.0f, 0.01f);
  CHECK_NEAR(sc_path_heading_at(&path, 3.0f), 0.0f, 0.01f);
  CHECK_NEAR(sc_path_curvature_at(&path, 3.0f, 0.1f), 0.0f, 0.01f);

  /* 圆路径曲率：采样要足够密，三点也不能离得太近，
   * 否则近共线时曲率对浮点误差极敏感。 */
  enum { N = 256 };
  float cx[N], cy[N], cs[N];
  for (int i = 0; i < N; i++) {
    float a = SC_TWO_PI * (float)i / (float)N;
    cx[i] = 2.0f * cosf(a);
    cy[i] = 2.0f * sinf(a);
  }
  sc_path_t circle;
  sc_path_build(&circle, cx, cy, cs, N, 1);
  CHECK_NEAR(sc_path_curvature_at(&circle, cs[64], 0.3f), 0.5f, 0.05f);

  /* Catmull-Rom 平滑点数：用 8 个控制点单独测 */
  float ccx[8] = {0.0f, 2.0f, 4.0f, 5.0f, 4.0f, 2.0f, 0.0f, -1.0f};
  float ccy[8] = {0.0f, -1.0f, 0.0f, 2.0f, 4.0f, 5.0f, 4.0f, 2.0f};
  float sx[64], sy[64];
  int sn = sc_catmull_rom(ccx, ccy, 8, 1, 4, sx, sy);
  CHECK(sn == 32);

  /* 纯跟踪 */
  sc_pure_pursuit_t pp;
  sc_pp_init(&pp, 0.3f, 0.5f, 0.5f, 0.6f);
  sc_pose2_t pose = {0.0f, 0.0f, 0.0f};
  CHECK_NEAR(sc_pp_steer(&pp, &pose, 1.0f, 0.0f), 0.0f, 1e-5f);
  CHECK(sc_pp_steer(&pp, &pose, 1.0f, 1.0f) > 0.0f);
  CHECK(sc_pp_lookahead(&pp, 2.0f) > sc_pp_lookahead(&pp, 0.5f));

  /* Stanley 符号 */
  sc_stanley_t st;
  sc_stanley_init(&st, 1.5f, 0.5f, 0.6f);
  CHECK(sc_stanley_steer(&st, 1.0f, 0.1f, 0.0f) < 0.0f);
  CHECK(sc_stanley_steer(&st, 1.0f, 0.0f, 0.1f) < 0.0f);
  CHECK(sc_stanley_steer(&st, 1.0f, -0.1f, 0.0f) > 0.0f);

  /* 预瞄控制 */
  sc_preview_t pv;
  sc_preview_init(&pv, 0.3f, 0.4f, 2.0f, 1.0f, 0.6f);
  CHECK_NEAR(sc_preview_steer(&pv, 0.0f, 0.0f, 0.0f), 0.0f, 1e-6f);
  CHECK(sc_preview_steer(&pv, 0.1f, 0.0f, 0.0f) < 0.0f);
  CHECK(sc_preview_steer(&pv, 0.0f, 0.1f, 0.0f) < 0.0f);
  CHECK(sc_preview_steer(&pv, 0.0f, 0.0f, 0.5f) > 0.0f);

  sc_pose2_t pp0 = {0.0f, 0.0f, 0.0f};
  sc_pose2_t pp1;
  sc_preview_predict(&pv, &pp0, 2.0f, &pp1);
  CHECK_NEAR(pp1.x, 0.8f, 1e-5f);

  /* LQR */
  sc_lqr_track_t lqr;
  CHECK(sc_lqr_track_init(&lqr, 0.3f, 1.0f, 0.005f, 1.0f, 1.0f, 0.1f, 0.6f) ==
        0);
  CHECK(lqr.k_y > 0.0f);
  CHECK(lqr.k_psi > 0.0f);
  CHECK(lqr.iterations > 0 && lqr.iterations < 5000);

  float e_y = 0.2f;
  float e_psi = 0.0f;
  for (int i = 0; i < 5000; i++) {
    float delta = sc_lqr_track_update(&lqr, e_y, e_psi, 0.0f);
    float ey_dot = 1.0f * e_psi;
    float epsi_dot = 1.0f * delta / 0.3f;
    e_y += ey_dot * 0.005f;
    e_psi += epsi_dot * 0.005f;
  }

  CHECK_NEAR(e_y, 0.0f, 0.02f);

  /* 平衡车 LQR */
  float K[4];
  CHECK(sc_lqr_balance(1.0f, 0.5f, 0.3f, 1.0f, 1.0f, 10.0f, 1.0f, 0.1f, K) ==
        0);
  for (int i = 0; i < 4; i++) {
    CHECK(isfinite(K[i]));
  }
}

static void test_speed_and_actuators(void) {
  printf("-- speed / servo / motor\n");

  CHECK_NEAR(sc_speed_limit_from_curvature(0.5f, 4.0f, 10.0f), 2.8284f, 1e-3f);
  CHECK_NEAR(sc_speed_limit_from_curvature(0.0f, 4.0f, 10.0f), 10.0f, 1e-6f);

  /* 速度剖面：弯道前必须减速 */
  float kappa[7] = {0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f};
  float vprof[7];
  sc_speed_profile_build(kappa, 7, 0.1f, 5.0f, 4.0f, 2.0f, 2.0f, 0, vprof, 1);
  CHECK(vprof[3] < 1.5f);
  CHECK(vprof[2] < 1.7f);
  CHECK(vprof[0] < 1.9f);
  CHECK(vprof[6] > vprof[5]);

  /* 梯形规划 */
  sc_trap_t trap;
  sc_trap_plan(&trap, 10.0f, 0.0f, 0.0f, 2.0f, 1.0f, 1.0f);
  CHECK_NEAR(trap.v_peak, 2.0f, 1e-4f);
  CHECK_NEAR(trap.s_acc, 2.0f, 1e-4f);
  CHECK_NEAR(trap.s_cruise, 6.0f, 1e-4f);
  CHECK_NEAR(trap.s_dec, 2.0f, 1e-4f);
  CHECK(trap.feasible == 1);
  CHECK_NEAR(sc_trap_speed_at_distance(&trap, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f),
             1.4142f, 1e-3f);
  CHECK_NEAR(sc_trap_speed_at_distance(&trap, 5.0f, 0.0f, 0.0f, 1.0f, 1.0f),
             2.0f, 1e-3f);

  /* 距离太短 */
  sc_trap_plan(&trap, 1.0f, 3.0f, 0.0f, 5.0f, 1.0f, 1.0f);
  CHECK(trap.feasible == 0);

  /* S 型：速度平滑上升，加速度不超限 */
  sc_scurve_t sc;
  sc_scurve_init(&sc, 2.0f, 10.0f, 0.0f);
  float max_a = 0.0f;
  for (int i = 0; i < 300; i++) {
    sc_scurve_step(&sc, 3.0f, 0.01f);
    if (fabsf(sc.a) > max_a) {
      max_a = fabsf(sc.a);
    }
  }
  CHECK(sc.v > 2.5f);
  CHECK(max_a <= 2.05f);

  /* 舵机 */
  sc_servo_t servo;
  sc_servo_init(&servo, 1500.0f, 500.0f, 0.0f, 1000.0f, 2000.0f, 5.0f);
  CHECK_NEAR(sc_servo_pwm_from_angle(&servo, 0.0f), 1500.0f, 1e-3f);
  CHECK_NEAR(sc_servo_pwm_from_angle(&servo, 0.5f), 1750.0f, 1e-3f);
  CHECK_NEAR(sc_servo_pwm_from_angle(&servo, 2.0f), 2000.0f, 1e-3f);

  sc_servo_reset(&servo, 0.0f);
  float pwm = sc_servo_update(&servo, 1.0f, 0.01f);
  CHECK_NEAR(pwm, 1500.0f + 500.0f * 0.05f, 1e-3f);

  /* 电机 */
  sc_motor_t motor;
  sc_motor_init(&motor, 0.1f, 1.0f, 1000.0f, 8.0f);
  sc_motor_reset(&motor, 0.0f);
  CHECK_NEAR(sc_motor_update(&motor, 0.5f, 0.01f), 0.55f, 1e-4f);
  sc_motor_reset(&motor, 0.0f);
  CHECK_NEAR(sc_motor_update(&motor, -0.5f, 0.01f), -0.55f, 1e-4f);
  sc_motor_reset(&motor, 0.0f);
  CHECK_NEAR(sc_motor_update(&motor, 2.0f, 0.01f), 1.0f, 1e-4f);

  /* 掉压补偿：电压 6V、标称 8V，占空比应放大 */
  sc_motor_set_battery(&motor, 6.0f);
  sc_motor_reset(&motor, 0.0f);
  CHECK(sc_motor_update(&motor, 0.5f, 0.01f) > 0.55f);
}

int main(void) {
  printf("=== smartcar-algo unit tests ===\n");

  test_math();
  test_filter();
  test_kinematics();
  test_camera();
  test_emag();
  test_attitude();
  test_pid();
  test_path_tracking();
  test_speed_and_actuators();

  printf("\n%d passed, %d failed\n", g_pass, g_fail);
  return (g_fail == 0) ? 0 : 1;
}
