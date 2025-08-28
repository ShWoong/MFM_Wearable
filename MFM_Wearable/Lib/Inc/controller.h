#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ---------------------------
 * Plant/Controller parameters
 * ---------------------------
 * 단위:
 *  - Kp_proc : [K/W] (정상상태 온도상승 / 전력)
 *  - tau     : [s]
 *  - theta   : [s] (dead-time)
 *  - lambda  : [s] (IMC 튜닝 → Kc 계산)
 *  - Ti      : [s] (적분시간, 권장 τ)
 *  - Tt      : [s] (anti-windup tracking time, back-calculation)
 *  - Ts      : [s] (샘플링)
 *  - u_min_W, u_max_W : [W]
 *  - Kff     : [-] (FF 스케일)
 *
 *  - lam_ref : [s] (참조 성형용; 0이면 lambda 사용)
 *  - use_dyn_ff : 동적 역모델 FF 사용 여부
 *
 * Safety/추정:
 *  - T_soft_max / T_hard_off / T_hard_rst : [°C]
 *  - track_tau_s : [s] (주변온도 추정 LPF)
 *  - p_off_w : [W] (주변 추정 허용 전력)
 *  - duty_off : [0..1]
 *  - latch_samples : (예비)
 */
typedef struct {
  float Kp_proc;
  float tau;
  float theta;

  float lambda;
  float Kc;     // = tau / (Kp_proc*(lambda+theta))
  float Ti;     // = tau
  float Tt;     // anti-windup tracking time
  float Ts;

  float u_min_W;
  float u_max_W;

  float Kff;        // feedforward gain (scale)

  /* NEW: 모델기반 옵션 */
  float lam_ref;    // 0 → lambda 사용
  bool  use_dyn_ff; // true: 역모델 FF 활성

  /* Safety */
  float T_soft_max;
  float T_hard_off;
  float T_hard_rst;

  /* Ambient estimation */
  float track_tau_s;
  float p_off_w;
  float duty_off;
  uint16_t latch_samples;
} ControllerParams;

/* ---------------------------
 * Controller runtime states
 * --------------------------- */
typedef struct {
  /* PI integrator state */
  float i_term_W;

  /* Ambient temperature estimate */
  float Tamb_hat;

  /* Latches / flags */
  bool  hard_off_latch;

  /* Telemetry */
  float u_noaw_W;   // FF+PI (AW 전)
  float u_cmd_W;    // 최종 전력 명령
  float duty_last;

  /* NEW: 역모델 FF/참조 성형 상태 */
  float Td_f;       // 성형된 참조
  float Td_prev;    // 직전 Td_f
  float Pff_W;      // FF 전력(전송/로그용)

  int   theta_samp; // dead-time 샘플 수
  int   rb_head;    // 링버퍼 인덱스
  /* 최대 8 s @ 100 Hz = 800 샘플 */
  float Td_rb[800];
} ControllerState;

/* 초기화 */
void Controller_Init(const ControllerParams* p, ControllerState* s);

/* (옵션) 적분기 리셋 */
static inline void Controller_ResetIntegrator(ControllerState* s){
  if (s) s->i_term_W = 0.0f;
}

/* 1스텝 업데이트
 * 입력 : Tset, Tmeas, P_meas_W, Vrms, R_est
 * 출력 : duty_out(0..1), u_des_W(전력명령)
 */
void Controller_Step(const ControllerParams* p, ControllerState* s,
                     float Tset, float Tmeas, float P_meas_W,
                     float Vrms, float R_est,
                     float* duty_out, float* u_des_W);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_H */
