/* 车辆运动学：差速、阿克曼/自行车、麦克纳姆、三轮全向、平衡车
 *
 * 坐标约定（全工程统一）：
 *   x 向前，y 向左，theta 逆时针为正。
 *   角速度 w > 0 表示左转。
 *   阿克曼前轮转角 delta > 0 表示左转。
 */
#ifndef SC_KINEMATICS_H
#define SC_KINEMATICS_H

#include "sc/common.h"

/* ============================================================
 * 1. 差速驱动（两轮差速 / 履带 / 双电机）
 * ============================================================ */

typedef struct {
  float wheel_radius; /* 驱动轮半径 m */
  float track_width;  /* 左右轮中心距 m（不是车宽！） */
  float gear_ratio;   /* 电机转数 / 车轮转数，直驱填 1 */
  float encoder_ppr;  /* 车轮转一圈的编码器计数（四倍频后） */
} sc_diff_drive_t;

void sc_diff_init(sc_diff_drive_t *d, float wheel_radius, float track_width,
                  float gear_ratio, float encoder_ppr);

/* 正解：左右轮线速度 -> 车体线速度、角速度 */
void sc_diff_forward(const sc_diff_drive_t *d, float v_left, float v_right,
                     float *v, float *w);

/* 逆解：车体线速度、角速度 -> 左右轮线速度 */
void sc_diff_inverse(const sc_diff_drive_t *d, float v, float w, float *v_left,
                     float *v_right);

/* 编码器增量 -> 车轮线速度（m/s） */
float sc_diff_wheel_speed(const sc_diff_drive_t *d, int32_t delta_counts,
                          float dt);

/* 航迹推算：用左右轮速积分位姿。
 * 采用精确圆弧积分（不是简单的一阶欧拉），原地旋转时不丢位置。 */
void sc_diff_odometry(sc_pose2_t *pose, const sc_diff_drive_t *d, float v_left,
                      float v_right, float dt);

/* ============================================================
 * 2. 阿克曼转向（自行车模型）
 * ============================================================ */

typedef struct {
  float wheelbase;   /* 前后轴距 L m */
  float track_width; /* 前轮轮距 W m */
  float max_steer;   /* 前轮最大转角 rad（内侧轮） */
} sc_ackermann_t;

void sc_ackermann_init(sc_ackermann_t *a, float wheelbase, float track_width,
                       float max_steer);

/* 曲率 -> 等效前轮转角：kappa = tan(delta)/L */
float sc_ackermann_steer_from_curvature(const sc_ackermann_t *a, float kappa);

/* 等效前轮转角 -> 曲率 */
float sc_ackermann_curvature(const sc_ackermann_t *a, float delta);

/* 由等效转角求内外前轮实际转角（阿克曼几何）。
 * delta > 0 左转，此时左轮为内轮。 */
void sc_ackermann_wheel_angles(const sc_ackermann_t *a, float delta,
                               float *delta_inner, float *delta_outer);

/* 由转向半径求等效转角 */
float sc_ackermann_steer_from_radius(const sc_ackermann_t *a, float radius);

/* 自行车模型积分（后轴参考点）。
 * 使用中点法：theta_mid = theta + 0.5*w*dt，精度优于一阶欧拉。 */
void sc_ackermann_integrate(sc_pose2_t *pose, const sc_ackermann_t *a, float v,
                            float delta, float dt);

/* 前轴参考点的运动学（有些队伍把摄像头装在车头，用前轴建模） */
void sc_ackermann_integrate_front(sc_pose2_t *pose, const sc_ackermann_t *a,
                                  float v, float delta, float dt);

/* 由左右轮速求车辆线速度与角速度（阿克曼车没有差速器时的滑移估计） */
void sc_ackermann_forward(const sc_ackermann_t *a, float v_left, float v_right,
                          float *v, float *w);

/* ============================================================
 * 3. 麦克纳姆轮（四轮）
 * ============================================================ */

typedef struct {
  float wheel_radius; /* 轮半径 m */
  float lx;           /* 轮轴到车体中心的纵向距离（半轴距）m */
  float ly;           /* 轮轴到车体中心的横向距离（半轮距）m */
} sc_mecanum_t;

/* 轮序：0=左前 FL, 1=右前 FR, 2=左后 RL, 3=右后 RR
 *
 * 逆解（车体速度 -> 四个轮子线速度）：
 *   FL =  vx - vy - (lx+ly)*wz
 *   FR =  vx + vy + (lx+ly)*wz
 *   RL =  vx + vy - (lx+ly)*wz
 *   RR =  vx - vy + (lx+ly)*wz
 *
 * 注意：不同厂家的辊子旋向相反，如果发现平移方向反了，
 * 把该轮公式里的 vy 符号取反即可（参数 roller_flip）。
 */
void sc_mecanum_init(sc_mecanum_t *m, float wheel_radius, float lx, float ly);

void sc_mecanum_inverse(const sc_mecanum_t *m, float vx, float vy, float wz,
                        float wheel[4]);

/* 正解（四个轮子角速度 -> 车体速度），伪逆形式 */
void sc_mecanum_forward(const sc_mecanum_t *m, const float wheel[4], float *vx,
                        float *vy, float *wz);

/* ============================================================
 * 4. 三轮全向轮（120° 分布）
 * ============================================================ */

typedef struct {
  float wheel_radius;
  float angle[3]; /* 轮子安装方位角 rad，默认 90°, 210°, 330° */
} sc_omni3_t;

void sc_omni3_init_default(sc_omni3_t *o, float wheel_radius);

/* 每个轮子的角速度：w_i = (-sin(a_i)*vx + cos(a_i)*vy)/R + wz */
void sc_omni3_inverse(const sc_omni3_t *o, float vx, float vy, float wz,
                      float wheel[3]);

void sc_omni3_forward(const sc_omni3_t *o, const float wheel[3], float *vx,
                      float *vy, float *wz);

/* ============================================================
 * 5. 平衡车 / 独轮车（直线倒立摆线性化模型）
 * ============================================================ */

typedef struct {
  float cart_mass; /* 轮组/底盘质量 M */
  float pole_mass; /* 车身质量 m */
  float pole_half; /* 质心到轮轴距离 l（半摆长） */
  float g;         /* 重力加速度，默认 9.81 */
} sc_balance_t;

/* 线性化连续状态空间：
 *   状态 x = [位置, 速度, 倾角, 角速度]
 *   输入 u = 水平推力 F
 * 经典小车倒立摆模型：
 *   A = [0 1 0 0; 0 0 -m*g/M 0; 0 0 0 1; 0 0 (M+m)*g/(M*l) 0]
 *   B = [0; 1/M; 0; -1/(M*l)]
 * 可以直接喂给 LQR 求反馈增益 K。 */
void sc_balance_state_space(const sc_balance_t *b, float A[4][4], float B[4]);

/* 电机电压/占空比 -> 推力（简化线性模型），用于仿真 */
float sc_balance_force_from_cmd(const sc_balance_t *b, float cmd,
                                float max_force);

#endif
