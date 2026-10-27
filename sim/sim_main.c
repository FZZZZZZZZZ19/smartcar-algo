/* 单场景仿真：纯跟踪跑一圈，输出完整遥测 CSV
 * 运行：make sim  ->  out/sim_single.csv
 */
#include <stdio.h>

#include "sim_common.h"

int main(void) {
  static sim_track_t track;
  sim_track_build(&track);

  float kappa_max = 0.0f;
  float v_min = 1e9f;
  float v_max = 0.0f;

  for (int i = 0; i < track.n; i++) {
    float ak = fabsf(track.kappa[i]);
    if (ak > kappa_max) {
      kappa_max = ak;
    }
    if (track.vprof[i] < v_min) {
      v_min = track.vprof[i];
    }
    if (track.vprof[i] > v_max) {
      v_max = track.vprof[i];
    }
  }

  printf("=== 单场景仿真（纯跟踪）===\n");
  printf("赛道: %d 点, 周长 %.2f m\n", track.n, (double)track.length);
  printf("最大曲率 %.3f 1/m (最小半径 %.2f m)\n", (double)kappa_max,
         (double)(1.0f / (kappa_max + 1e-6f)));
  printf("速度剖面 %.2f ~ %.2f m/s\n", (double)v_min, (double)v_max);

  sim_metrics_t m;
  sim_run(&track, SIM_CTRL_PP, "out/sim_single.csv", &m);

  printf("\n结果:\n");
  printf("  横向误差 RMS   %.4f m\n", (double)m.rms_ey);
  printf("  横向误差 最大  %.4f m\n", (double)m.max_ey);
  printf("  航向误差 RMS   %.4f rad\n", (double)m.rms_epsi);
  printf("  平均速度       %.3f m/s\n", (double)m.mean_v);
  printf("  最高速度       %.3f m/s\n", (double)m.max_v);
  printf("  首圈用时       %.2f s\n", (double)m.lap_time);

  printf("\n遥测已写入 out/sim_single.csv（%d 行）\n", m.steps);
  return 0;
}
