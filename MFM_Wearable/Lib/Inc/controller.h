#ifndef CONTROLLER_H
#define CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 외부(온도) 루프 파라미터 — 대표 튜닝은 lambda 하나만 */
typedef struct {
  /* 공정 모델 */
  float Kp_proc;      // [K/W]
  float tau;          // [s]
  float theta;        // [s]

  /* IMC/PI 파라미터 (lambda만 외부 입력, 나머지는 자동계산) */
  float lambda;       // [s]  ★ 대표 파라미터
  float Kc;           // [W/K]  = tau / (Kp_proc*(lambda+theta))
  float Ti;           // [s]    = tau
  float Tt;           // [s]    ≈ 0.2*lambda (anti-windup back-calculation)
  float Ts;           // [s]    샘플링(외루프, 0.01s)

  /* 전력 제한 */
  float u_min_W;      // [W]
  float u_max_W;      // [W]

  /* 역모델 Feedforward */
  float Kff;          // [-]  (0이면 미사용)

  /* 참조 성형/옵션 */
  float lam_ref;      // [s]  (0 → lambda 사용)
  bool  use_dyn_ff;   // true면 dead-time 보상(선택)
  int   theta_samp_max; // dead-time 버퍼 한도 (samples)

  /* Safety */
  float T_soft_max;   // [°C] 소프트 리밋(목표 클램프)
  float T_hard_off;   // [°C] 하드오프 래치 온
  float T_hard_rst;   // [°C] 하드오프 래치 해제

  /* 주변온도 추정 */
  float track_tau_s;  // [s]
  float p_off_w;      // [W]
  float duty_off;     // [0..1]
} ControllerParams;

/* 외부(온도) 루프 상태 */
typedef struct {
  float i_term_W;     // PI 적분 상태(전력 명령 단위)
  float Tamb_hat;     // 주변온도 추정
  bool  hard_off_latch;

  /* 텔레메트리 */
  float u_noaw_W;
  float u_cmd_W;
  float duty_last;

  /* 참조 성형/지연 보상 */
  float Td_f;
  float Td_prev;
  float Pff_W;

  int   theta_samp;
  int   rb_head;
  float Td_rb[800];   // 최대 8s @100Hz
} ControllerState;

/* 초기화 */
void Controller_Init(const ControllerParams* p, ControllerState* s);

/* 대표 파라미터 재튜닝(λ만 입력, 종속 파라미터 자동 계산) */
void Controller_SetLambda(ControllerParams* p, ControllerState* s, float lambda);

/* 공정모델 변경(드물게) */
void Controller_UpdatePlant(ControllerParams* p, ControllerState* s,
                            float Kp_proc, float tau, float theta);

/* (옵션) FF 게인 변경 */
static inline void Controller_SetKff(ControllerParams* p, float kff){ if (p && kff >= 0.0f) p->Kff = kff; }

/* 적분기 리셋 */
static inline void Controller_ResetIntegrator(ControllerState* s){ if (s) s->i_term_W = 0.0f; }

/* 1스텝(외부 루프): Tset/Tmeas -> u_des_W(전력 목표)
 * duty_out은 텔레메트리 힌트일 뿐, 실제 듀티는 내부 전력루프가 결정
 */
void Controller_Step(const ControllerParams* p, ControllerState* s,
                     float Tset, float Tmeas, float P_meas_W,
                     float Vrms, float R_est,
                     float* duty_out, float* u_des_W);

#ifdef __cplusplus
}
#endif
#endif /* CONTROLLER_H */
