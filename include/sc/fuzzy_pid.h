/* 模糊 PID：Mamdani 推理，7 个语言变量，49 条规则，重心法解模糊
 *
 * 输入：误差 e、误差变化率 ec
 * 输出：ΔKp、ΔKi、ΔKd，叠加到基础 PID 参数上
 */
#ifndef SC_FUZZY_PID_H
#define SC_FUZZY_PID_H

#include "sc/common.h"
#include "sc/pid.h"

typedef struct {
  sc_pid_t pid;

  float e_range;  /* 误差量化范围：|e| <= e_range 映射到 [-3,3] */
  float ec_range; /* 误差变化率量化范围 */

  float kp0, ki0, kd0;                /* 基础参数 */
  float kp_scale, ki_scale, kd_scale; /* 模糊输出缩放 */

  float last_e;
  int init;
} sc_fuzzy_pid_t;

void sc_fuzzy_pid_init(sc_fuzzy_pid_t *f, float kp0, float ki0, float kd0,
                       float e_range, float ec_range);

void sc_fuzzy_pid_set_scales(sc_fuzzy_pid_t *f, float kp_scale, float ki_scale,
                             float kd_scale);
void sc_fuzzy_pid_set_limits(sc_fuzzy_pid_t *f, float out_min, float out_max);
void sc_fuzzy_pid_reset(sc_fuzzy_pid_t *f);

float sc_fuzzy_pid_update(sc_fuzzy_pid_t *f, float setpoint, float measurement,
                          float dt);

/* 单独暴露模糊推理，便于教学和单元测试：
 * 输入已归一化到 [-3,3] 的 e、ec，返回 [-3,3] 的 ΔK 输出。 */
float sc_fuzzy_dkp(float e_norm, float ec_norm);
float sc_fuzzy_dki(float e_norm, float ec_norm);
float sc_fuzzy_dkd(float e_norm, float ec_norm);

#endif
