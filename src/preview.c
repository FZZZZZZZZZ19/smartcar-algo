#include "sc/preview.h"

#include "sc/math_utils.h"

void sc_preview_init(sc_preview_t *p, float wheelbase, float preview_time,
                     float k_y, float k_psi, float max_steer) {
  p->wheelbase = wheelbase;
  p->preview_time = (preview_time > 0.0f) ? preview_time : 0.4f;
  p->k_y = k_y;
  p->k_psi = k_psi;
  p->max_steer = max_steer;
}

void sc_preview_predict(const sc_preview_t *p, const sc_pose2_t *pose, float v,
                        sc_pose2_t *predicted) {
  /* 最简单的预测：保持当前航向直行。速度越快，预瞄点越远。 */
  predicted->x = pose->x + v * cosf(pose->theta) * p->preview_time;
  predicted->y = pose->y + v * sinf(pose->theta) * p->preview_time;
  predicted->theta = pose->theta;
}

float sc_preview_steer(const sc_preview_t *p, float e_y_preview, float e_psi,
                       float kappa_ref) {
  /* 预测点在左侧(e_y>0)要右打，航向偏左(e_psi>0)也要右打 */
  float delta = -p->k_y * e_y_preview - p->k_psi * e_psi;

  /* 曲率前馈：弯道上不需要等误差出现才打角 */
  delta += atanf(p->wheelbase * kappa_ref);

  return sc_clampf(delta, -p->max_steer, p->max_steer);
}
