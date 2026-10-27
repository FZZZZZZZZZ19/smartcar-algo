/* 第一课：PID 阶跃响应
 *
 * 被控对象：一阶惯性 + 100 ms 纯延迟
 *
 *     y' = (u_delayed - y) / 0.2
 *
 * 为什么加延迟：真实的舵机、电机、摄像头都有延迟。没有延迟时 PID
 * 太"好调"，加了延迟你才能看到超调、振荡、积分饱和这些真问题。
 *
 * 运行：
 *   make exp
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "sc/pid.h"

#define DELAY_N 10 /* 10 × 10 ms = 100 ms */

typedef struct {
  float buf[DELAY_N];
  int idx;
} delay_t;

static void delay_reset(delay_t *d) { memset(d, 0, sizeof(*d)); }

static float delay_step(delay_t *d, float u) {
  float out = d->buf[d->idx];
  d->buf[d->idx] = u;
  d->idx = (d->idx + 1) % DELAY_N;
  return out;
}

static void run(const char *name, float kp, float ki, float kd, int limited) {
  sc_pid_t pid;
  sc_pid_init(&pid, kp, ki, kd);
  if (limited) {
    sc_pid_set_limits(&pid, 0.0f, 1.0f);
  } else {
    sc_pid_set_limits(&pid, -10.0f, 10.0f);
  }

  delay_t dly;
  delay_reset(&dly);

  float y = 0.0f;
  const float dt = 0.01f;
  float peak = 0.0f;
  float last_outside = 0.0f;

  for (int i = 0; i < 1000; i++) {
    float u = sc_pid_update(&pid, 1.0f, y, dt);
    float ud = delay_step(&dly, u);
    y += dt * (ud - y) / 0.2f;

    if (y > peak) {
      peak = y;
    }
    if (fabsf(y - 1.0f) > 0.02f) {
      last_outside = (float)(i + 1) * dt;
    }
  }

  if (!isfinite(y) || fabsf(y) > 5.0f) {
    printf("%-30s 发散\n", name);
    return;
  }

  float overshoot = (peak > 1.0f) ? (peak - 1.0f) * 100.0f : 0.0f;

  printf("%-30s peak=%.3f  超调=%5.1f%%  final=%.3f  settle=%.2fs\n", name,
         (double)peak, (double)overshoot, (double)y, (double)last_outside);
}

int main(void) {
  printf("=== 第一课：PID 阶跃响应 ===\n");
  printf("目标 1.0，对象 y'=(u-y)/0.2，延迟 100 ms，周期 10 ms\n\n");

  run("P      kp=2", 2.0f, 0.0f, 0.0f, 0);
  run("PI     kp=2 ki=4", 2.0f, 4.0f, 0.0f, 0);
  run("PID    kp=2 ki=4 kd=0.05", 2.0f, 4.0f, 0.05f, 0);
  run("PID    kp=2 ki=4 kd=0.2", 2.0f, 4.0f, 0.2f, 0);
  run("PI     kp=3 ki=6", 3.0f, 6.0f, 0.0f, 0);
  run("PI     kp=4 ki=8", 4.0f, 8.0f, 0.0f, 0);
  run("PI     kp=2 ki=20", 2.0f, 20.0f, 0.0f, 0);
  run("PI     limit[0,1] kp=2 ki=20", 2.0f, 20.0f, 0.0f, 1);

  return 0;
}
