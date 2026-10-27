/* 三种横向控制器对比：纯跟踪 / Stanley / LQR
 * 同一辆车、同一条赛道、同一速度剖面，只换横向控制器。
 * 运行：make sim  ->  out/{pp,stanley,lqr}.csv, out/track.csv, out/metrics.txt
 */
#include <stdio.h>

#include "sim_common.h"

static void write_track(const sim_track_t *t) {
  FILE *f = fopen("out/track.csv", "w");
  if (!f) {
    return;
  }

  fprintf(f, "x,y,s,kappa,vprof\n");
  for (int i = 0; i < t->n; i++) {
    fprintf(f, "%.4f,%.4f,%.4f,%.6f,%.4f\n", (double)t->x[i], (double)t->y[i],
            (double)t->s[i], (double)t->kappa[i], (double)t->vprof[i]);
  }
  fclose(f);
}

int main(void) {
  static sim_track_t track;
  sim_track_build(&track);

  write_track(&track);

  static const sim_ctrl_t ctrls[3] = {SIM_CTRL_PP, SIM_CTRL_STANLEY,
                                      SIM_CTRL_LQR};
  static const char *files[3] = {"out/pp.csv", "out/stanley.csv",
                                 "out/lqr.csv"};

  sim_metrics_t metrics[3];

  for (int i = 0; i < 3; i++) {
    sim_run(&track, ctrls[i], files[i], &metrics[i]);
  }

  FILE *mf = fopen("out/metrics.txt", "w");
  if (mf) {
    fprintf(mf, "controller,rms_ey,max_ey,rms_epsi,mean_v,max_v,lap_time\n");
    for (int i = 0; i < 3; i++) {
      fprintf(mf, "%s,%.5f,%.5f,%.5f,%.4f,%.4f,%.3f\n", sim_ctrl_name(ctrls[i]),
              (double)metrics[i].rms_ey, (double)metrics[i].max_ey,
              (double)metrics[i].rms_epsi, (double)metrics[i].mean_v,
              (double)metrics[i].max_v, (double)metrics[i].lap_time);
    }
    fclose(mf);
  }

  printf("=== 横向控制器对比 ===\n");
  printf("赛道周长 %.2f m，最高目标速度 3.2 m/s，控制周期 5 ms\n\n",
         (double)track.length);

  printf("%-14s %10s %10s %10s %10s %9s\n", "控制器", "RMS ey", "max |ey|",
         "RMS epsi", "平均速度", "首圈");
  printf("%-14s %10s %10s %10s %10s %9s\n", "", "(m)", "(m)", "(rad)", "(m/s)",
         "(s)");

  for (int i = 0; i < 3; i++) {
    printf("%-14s %10.4f %10.4f %10.4f %10.3f %9.2f\n", sim_ctrl_name(ctrls[i]),
           (double)metrics[i].rms_ey, (double)metrics[i].max_ey,
           (double)metrics[i].rms_epsi, (double)metrics[i].mean_v,
           (double)metrics[i].lap_time);
  }

  printf("\nCSV: out/pp.csv out/stanley.csv out/lqr.csv out/track.csv\n");
  return 0;
}
