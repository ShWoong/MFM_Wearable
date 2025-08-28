#include "controller.h"
#include <math.h>
#include <stddef.h>

static inline float clampf(float x, float lo, float hi){
  if (x < lo) return lo; if (x > hi) return hi; return x;
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

  s->Td_f = s->Td_prev = 25.0f;
  s->Pff_W = 0.0f;

  float Ts = (p->Ts > 1e-6f) ? p->Ts : 1e-2f;
  int th_samp = (int)lroundf(p->theta / Ts);
  if (th_samp < 0) th_samp = 0;
  int rbmax = (int)(sizeof(s->Td_rb)/sizeof(s->Td_rb[0])) - 1;
  if (p->theta_samp_max > 0 && p->theta_samp_max < rbmax) rbmax = p->theta_samp_max;
  if (th_samp > rbmax) th_samp = rbmax;
  s->theta_samp = th_samp;
  s->rb_head = 0;
  for (size_t i=0;i<sizeof(s->Td_rb)/sizeof(s->Td_rb[0]);++i) s->Td_rb[i] = s->Td_f;
}

void Controller_SetLambda(ControllerParams* p, ControllerState* s, float lambda)
{
  if (!p) return;
  p->lambda = clampf(lambda, 0.05f, 60.0f);
  p->Kc = p->tau / (p->Kp_proc * (p->lambda + p->theta));
  p->Ti = p->tau;
  p->Tt = clampf(0.2f * p->lambda, 0.05f, 5.0f);
  if (p->lam_ref <= 1e-6f) p->lam_ref = 0.0f;   // 참조성형은 lambda와 동일
  if (s) Controller_ResetIntegrator(s);
}

void Controller_UpdatePlant(ControllerParams* p, ControllerState* s,
                            float Kp_proc, float tau, float theta)
{
  if (!p) return;
  p->Kp_proc = Kp_proc;
  p->tau     = tau;
  p->theta   = theta;
  Controller_SetLambda(p, s, p->lambda);
}

/* 외부루프에선 실제 듀티를 결정하지 않으므로 힌트만 0 반환 */
static inline float duty_hint_from_power(float P_W, float Vrms, float R_ohm){
  (void)P_W; (void)Vrms; (void)R_ohm;
  return 0.0f;
}

void Controller_Step(const ControllerParams* p, ControllerState* s,
                     float Tset, float Tmeas, float P_meas_W,
                     float Vrms, float R_est,
                     float* duty_out, float* u_des_W)
{
  if (!p || !s || !duty_out || !u_des_W) return;

  const float Ts      = (p->Ts   > 1e-6f) ? p->Ts   : 1e-2f;
  const float Ti      = (p->Ti   > 1e-6f) ? p->Ti   : 1e-2f;
  const float Tt      = (p->Tt   > 1e-6f) ? p->Tt   : 1e-1f;
  const float Kp_proc = (fabsf(p->Kp_proc) > 1e-6f) ? p->Kp_proc : 1.0f;
  const float Kc      = p->Kc;
  const float lam_r   = (p->lam_ref > 1e-6f) ? p->lam_ref : p->lambda;

  /* Safety latch */
  if (Tmeas >= p->T_hard_off)       s->hard_off_latch = true;
  else if (Tmeas <= p->T_hard_rst)  s->hard_off_latch = false;

  float Tset_eff = (Tset > p->T_soft_max) ? p->T_soft_max : Tset;

  /* 주변온도 추정 (저전력/저듀티일 때만) */
  if ((P_meas_W < p->p_off_w) && (s->duty_last < p->duty_off) && (p->track_tau_s > 1e-3f)) {
    float alpha_tr = Ts / p->track_tau_s; if (alpha_tr > 1.0f) alpha_tr = 1.0f;
    s->Tamb_hat += alpha_tr * (Tmeas - s->Tamb_hat);
  }

  /* 참조 성형 */
  float alpha = Ts / lam_r; if (alpha > 1.0f) alpha = 1.0f;
  s->Td_prev = s->Td_f;
  s->Td_f   += alpha * (Tset_eff - s->Td_f);

  /* dead-time 보상(선택) */
  s->Td_rb[s->rb_head] = s->Td_f;
  int idx = s->rb_head - s->theta_samp; if (idx < 0) idx += (int)(sizeof(s->Td_rb)/sizeof(s->Td_rb[0]));
  float Td_used = p->use_dyn_ff ? s->Td_rb[idx] : s->Td_f;
  s->rb_head++; if (s->rb_head >= (int)(sizeof(s->Td_rb)/sizeof(s->Td_rb[0]))) s->rb_head = 0;

  /* 역모델 FF (냉각구간 FF 차단) */
  float dTd  = (s->Td_f - s->Td_prev) / Ts;
  float Pff  = (Td_used - s->Tamb_hat)/Kp_proc + (p->tau / Kp_proc) * dTd;
  if (p->Kff > 0.0f) Pff *= p->Kff;
  Pff = clampf(Pff, p->u_min_W, p->u_max_W);
  if ( (Td_used - s->Tamb_hat) < 0.0f ) Pff = 0.0f;
  if (p->Kff > 0.0f) Pff *= p->Kff;
  Pff = clampf(Pff, p->u_min_W, p->u_max_W);
  s->Pff_W = Pff;

  /* PI + AW(back-calculation) */
  float e = Tset_eff - Tmeas;
  if (e <= 0.0f || (Td_used - s->Tamb_hat) < 0.0f)  Pff = 0.0f;
  float u_pi_noaw = Kc * e + s->i_term_W;
  float u_noaw    = Pff + u_pi_noaw;

  float u_sat = clampf(u_noaw, p->u_min_W, p->u_max_W);

  float i_dot = (Kc / Ti) * e + (u_sat - u_noaw) / Tt;
  bool block_int = (u_sat <= p->u_min_W + 1e-6f) && (e < 0.0f);
  if (!block_int) s->i_term_W += Ts * i_dot;

  const float I_MAX = p->u_max_W;
  s->i_term_W = clampf(s->i_term_W, -I_MAX, I_MAX);

  float u = u_sat;
  if (s->hard_off_latch) u = 0.0f;

  /* 결과 */
  s->u_noaw_W = u_noaw;
  s->u_cmd_W  = u;

  *u_des_W  = u;                                     // 전력 목표[W]
  *duty_out = duty_hint_from_power(u, Vrms, R_est);  // 텔레메트리 힌트
}
