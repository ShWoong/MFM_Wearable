#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif

/* 전력→듀티 피드포워드 맵 모드 */
typedef enum {
    CTRL_PWRMAP_ONOFF = 0,      /* P ≈ d·Vs²/R  → d = P·R / Vs² */
    CTRL_PWRMAP_BUCK  = 1       /* P ≈ d²·Vs²/R → d = sqrt(P·R)/Vs */
} CtrlPowermapMode;

typedef struct {
    /* ── 식별/IMC 파라미터 ───────────────────────────── */
    float Kp_proc;      /* 열 플랜트 이득 Kp */
    float tau;          /* 플랜트 시정수 τ */
    float theta;        /* 지연 θ */
    float lambda;       /* IMC λ (PI 유도용; 외부 FF는 미사용) */

    /* ── 외부 루프(Temp→Power) PI ────────────────────── */
    float Kc;           /* Kc (0이면 IMC로 산출) */
    float Ti;           /* Ti (0이면 τ로 설정) */
    float Tt;           /* AW 시간상수(표준/트래킹 공통) */
    float Ts;           /* 샘플링 주기 */
    float u_min_W;      /* 전력 명령 최소 */
    float u_max_W;      /* 전력 명령 최대 */

    /* ── 내부 루프(Power→Duty) PI ───────────────────── */
    float KcP;
    float TiP;
    float TtP;          /* 내부 AW 시간상수 */
    float duty_min;     /* 듀티 하한 */
    float duty_max;     /* 듀티 상한 */
    float du_max;       /* 듀티 변화율 제한 [1/s] */
    int   map_mode;     /* CtrlPowermapMode */

    /* ── 안전/환경 추정 ─────────────────────────────── */
    float T_soft_max;   /* 소프트 상한: 가열방향 적분 억제 등(옵션) */
    float T_hard_off;   /* 하드 컷오프 온도 (래치 ON) */
    float T_hard_rst;   /* 하드 래치 해제 온도 */
    float track_tau_s;  /* 환경온도 추정 저역 시정수 */
    float p_off_w;      /* 오프로 간주할 전력 [W] */
    float duty_off;     /* 오프로 간주할 듀티 */
    int   latch_samples;/* Ta 래치 샘플 수(예: 50=0.5s@100Hz) */

    /* ── 호환 필드(미사용) ──────────────────────────── */
    float lam_ff;       /* 외부 FF용이었으나 현재 미사용(0 취급) */
} ControllerParams;

typedef struct {
    /* 외부 루프 상태 */
    float I_T;          /* 외부 PI 적분 */
    float u_unsat;      /* 외부 비포화 명령(디버그용) */
    float u_sat;        /* 외부 포화 명령(=u_desW) */

    /* 내부 루프 상태 */
    float I_P;          /* 내부 PI 적분 */
    float duty_last;    /* 마지막 최종 듀티(레이트리밋 이후) */

    /* 환경온도 추정/안전 래치 */
    float Ta_est;
    bool  Ta_locked;
    float Ta_accum;
    int   Ta_n;
    bool  overtemp_latch;

    /* 호환 필드(외부 FF 잔재; 항상 0 유지) */
    float uff_f;
} ControllerState;

/* 초기화: 기본값/유도치 채움 */
void Controller_Init(ControllerParams *p, ControllerState *s);

/* 한 틱 실행: 외부(Temp→Power) → 내부(Power→Duty)
   - 외부 FF 없음
   - 외부 표준 AW + 트래킹 AW(내부 포화/레이트리밋 반영)
   - 내부 AW는 최종 duty(레이트리밋 이후) 기준
   - 전력→듀티 FF 맵 모드 선택 가능 */
void Controller_Step(ControllerParams *p, ControllerState *s,
                     float T_set_C, float T_meas_C,
                     float P_meas_W, float Vbus_V, float R_est_ohm,
                     float *duty_out, float *u_des_W);

#ifdef __cplusplus
}
#endif
#endif /* CONTROLLER_H */
