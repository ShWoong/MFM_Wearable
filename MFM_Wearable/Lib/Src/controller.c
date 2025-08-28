#include "controller.h"
#include <math.h>
#include <stddef.h>

/* 안전한 소수 처리 */
#ifndef FLT_EPSILON
#define FLT_EPSILON (1.19209290e-7F)
#endif

static inline float clampf(float x, float lo, float hi){
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void Controller_Init(const ControllerParams* p, ControllerState* s)
{
  if (!p || !s) return;

  s->i_term_W       = 0.0f;
  s->Tamb_hat       = 25.0f;
  s->hard_off_latch = false;
  s->u_noaw_W       = 0.0f;
  s->u_cmd_W        = 0.0f;
  s->duty_last      = 0.0f;

  /* 참조 성형 & 지연 보상 초기화 */
  s->Td_f = s->Td_prev = 25.0f;
  s->Pff_W = 0.0f;

  float Ts = (p->Ts > 1e-6f) ? p->Ts : 1e-3f;
  int th_samp = (int)lroundf(p->theta / Ts);
  if (th_samp < 0) th_samp = 0;
  if (th_samp > (int)(sizeof(s->Td_rb)/sizeof(s->Td_rb[0])) - 1)
      th_samp = (int)(sizeof(s->Td_rb)/sizeof(s->Td_rb[0])) - 1;
  s->theta_samp = th_samp;
  s->rb_head = 0;
  for (size_t i=0;i<sizeof(s->Td_rb)/sizeof(s->Td_rb[0]);++i) s->Td_rb[i] = s->Td_f;
}

/* Vrms, R_est 기반의 단순 W→duty 매핑
 * P ≈ duty * V^2 / R  →  duty = P*R / V^2
 */
static inline float power_to_duty(float P_W, float Vrms, float R_ohm)
{
  const float Vmin = 0.1f;
  const float Rmin = 0.05f;
  if (Vrms < Vmin || R_ohm < Rmin) return 0.0f;
  float duty = (P_W * R_ohm) / (Vrms * Vrms + 1e-6f);
  return clampf(duty, 0.0f, 0.9999f);
}

void Controller_Step(const ControllerParams* p, ControllerState* s,
                     float Tset, float Tmeas, float P_meas_W,
                     float Vrms, float R_est,
                     float* duty_out, float* u_des_W)
{
  if (!p || !s || !duty_out || !u_des_W) return;

  /* ----- 0) 파라미터 가드 ----- */
  const float Ts      = (p->Ts   > 1e-6f) ? p->Ts   : 1e-3f;
  const float Ti      = (p->Ti   > 1e-6f) ? p->Ti   : 1e-3f;
  const float Tt      = (p->Tt   > 1e-6f) ? p->Tt   : 1e-3f;
  const float Kp_proc = (fabsf(p->Kp_proc) > 1e-6f) ? p->Kp_proc : 1.0f;
  const float Kc      = p->Kc;  /* λ 바뀌면 메인에서 갱신 */
  const float lam_r   = (p->lam_ref > 1e-6f) ? p->lam_ref : p->lambda;

  /* ----- 1) Safety ----- */
  if (Tmeas >= p->T_hard_off) {
    s->hard_off_latch = true;
  } else if (Tmeas <= p->T_hard_rst) {
    s->hard_off_latch = false;
  }

  float Tset_eff = Tset;
  if (Tset_eff > p->T_soft_max) Tset_eff = p->T_soft_max;

  /* ----- 2) 주변온도 추정 (저전력/저듀티) ----- */
  if ((P_meas_W < p->p_off_w) && (s->duty_last < p->duty_off) && (p->track_tau_s > 1e-3f)) {
    float alpha_tr = Ts / p->track_tau_s; if (alpha_tr > 1.0f) alpha_tr = 1.0f;
    s->Tamb_hat += alpha_tr * (Tmeas - s->Tamb_hat);
  }

  /* ----- 3) 참조 성형 ----- */
  float alpha = Ts / lam_r; if (alpha > 1.0f) alpha = 1.0f;
  s->Td_prev = s->Td_f;
  s->Td_f   += alpha * (Tset_eff - s->Td_f);

  /* dead-time 보정(선택): use_dyn_ff=true면 θ샘플만큼 미리 당긴 참조 사용 */
  s->Td_rb[s->rb_head] = s->Td_f;
  int idx = s->rb_head - s->theta_samp; if (idx < 0) idx += (int)(sizeof(s->Td_rb)/sizeof(s->Td_rb[0]));
  float Td_used = p->use_dyn_ff ? s->Td_rb[idx] : s->Td_f;
  s->rb_head++; if (s->rb_head >= (int)(sizeof(s->Td_rb)/sizeof(s->Td_rb[0]))) s->rb_head = 0;

  /* ----- 4) 역모델 Feedforward -----
   * dT/dt ≈ (Td_f - Td_prev)/Ts
   * P_ff = (Td_used - Tamb_hat)/K + (tau/K)*dT
   * 냉각 구간(목표가 주변 이하)에서는 FF 가열 금지
   */
  float dTd  = (s->Td_f - s->Td_prev) / Ts;
  float Pff  = (Td_used - s->Tamb_hat)/Kp_proc + (p->tau / Kp_proc) * dTd;
  if ( (Td_used - s->Tamb_hat) < 0.0f ) Pff = 0.0f; /* cooling FF 금지 */
  if (p->Kff > 0.0f) Pff *= p->Kff;
  Pff = clampf(Pff, p->u_min_W, p->u_max_W);
  s->Pff_W = Pff;

  /* ----- 5) PI + Anti-windup(back-calculation) ----- */
  float e = Tset_eff - Tmeas;
  float u_pi_noaw = Kc * e + s->i_term_W;
  float u_noaw    = Pff + u_pi_noaw;

  /* 출력 제한 */
  float u_sat = clampf(u_noaw, p->u_min_W, p->u_max_W);

  /* Back-calculation: di/dt = Kc/Ti * e + (u_sat - u_noaw)/Tt */
  float i_dot = (Kc / Ti) * e + (u_sat - u_noaw) / Tt;

  /* 조건부 적분: 0W에서 냉각(e<0)으로 더 내려가려는 상황은 적분을 붙잡아 과발열 펄스 방지 */
  bool block_int = (u_sat <= p->u_min_W + 1e-6f) && (e < 0.0f);
  if (!block_int) s->i_term_W += Ts * i_dot;

  /* integrator clamp */
  const float I_MAX = p->u_max_W;
  s->i_term_W = clampf(s->i_term_W, -I_MAX, I_MAX);

  /* ----- 6) 하드오프 ----- */
  float u = u_sat;
  if (s->hard_off_latch) u = 0.0f;

  /* ----- 7) W→duty ----- */
  float duty = power_to_duty(u, Vrms, R_est);

  /* 결과/텔레메트리 */
  s->u_noaw_W  = u_noaw;
  s->u_cmd_W   = u;
  s->duty_last = duty;

  *u_des_W  = u;
  *duty_out = duty;
}
