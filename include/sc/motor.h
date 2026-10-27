/* 直流电机执行器模型：死区补偿、电压补偿、斜率限制
 *
 * 输入 cmd ∈ [-1, 1]，输出实际占空比 ∈ [-max_duty, max_duty]。
 */
#ifndef SC_MOTOR_H
#define SC_MOTOR_H

#include "sc/common.h"

typedef struct {
  float deadzone;        /* 启动死区占空比，0~0.3 */
  float max_duty;        /* 最大占空比，0~1 */
  float slew;            /* 占空比变化率上限 1/s */
  float battery_nominal; /* 标称电压 */
  float battery_now;     /* 当前电压（用于补偿掉压） */
  float last_duty;
  int init;
} sc_motor_t;

void sc_motor_init(sc_motor_t *m, float deadzone, float max_duty, float slew,
                   float battery_nominal);

void sc_motor_set_battery(sc_motor_t *m, float voltage);
void sc_motor_reset(sc_motor_t *m, float duty);

float sc_motor_update(sc_motor_t *m, float cmd, float dt);

#endif
