#include "controller.h"
#include <math.h>

static inline float clampf(float x, float lo, float hi){
    return (x<lo)?lo:((x>hi)?hi:x);
}
static inline float posf(float x, float eps){ return (x>eps)?x:eps; }

void Controller_Init(ControllerParams *p, ControllerState *s){
    /* 외부 PI를 IMC로 유도(필요 시) */
    if (p->Kc <= 0.0f || p->Ti <= 0.0f){
        /* IMC→PI: Ti=τ, Kc=τ/[Kp(λ+θ)] */
        const float denom = p->Kp_proc * (p->lambda + p->theta);
        p->Ti = (p->tau > 1e-9f) ? p->tau : 1.0f;
        p->Kc = (denom > 1e-9f) ? (p->tau / denom) : 0.0f;
    }
    if (p->Tt   <= 0.0f) p->Tt   = p->Ti * 0.5f;

    /* 내부 PI 기본 */
    if (p->TiP  <= 0.0f) p->TiP  = 1.0f;
    if (p->TtP  <= 0.0f) p->TtP  = p->TiP * 0.5f;

    /* 안전/환경 기본 */
    if (p->track_tau_s   <= 0.0f) p->track_tau_s = 30.0f;
    if (p->p_off_w       <= 0.0f) p->p_off_w     = 0.2f;
    if (p->duty_off      <= 0.0f) p->duty_off    = 0.02f;
    if (p->latch_samples <= 0)    p->latch_samples = 50;

    /* 맵 모드 기본 (기존 구현과 호환: BUCK) */
    if (p->map_mode != CTRL_PWRMAP_ONOFF && p->map_mode != CTRL_PWRMAP_BUCK)
        p->map_mode = CTRL_PWRMAP_BUCK;

    /* 상태 초기화 */
    s->I_T = 0.0f; s->I_P = 0.0f;
    s->u_unsat = 0.0f; s->u_sat = 0.0f;
    s->duty_last = 0.0f;
    s->Ta_est = 25.0f; s->Ta_locked = false;
    s->Ta_accum = 0.0f; s->Ta_n = 0;
    s->overtemp_latch = false;
    s->uff_f = 0.0f; /* 외부 FF 미사용 */
}

void Controller_Step(ControllerParams *p, ControllerState *s,
                     float T_set_C, float T_meas_C,
                     float P_meas_W, float Vbus_V, float R_est_ohm,
                     float *duty_out, float *u_des_W)
{
    float duty = 0.0f;
    float u_des = 0.0f;

    /* 0) 세트포인트 소프트 클램프(필요 시) */
    const float Tsp = (p->T_soft_max > 0.0f) ? fminf(T_set_C, p->T_soft_max) : T_set_C;

    /* A) 환경온도 래치: 완전 오프일 때만 누적 */
    if (!s->Ta_locked){
        if (P_meas_W < p->p_off_w && s->duty_last < p->duty_off){
            s->Ta_accum += T_meas_C;
            s->Ta_n++;
            if (s->Ta_n >= p->latch_samples){
                s->Ta_est = s->Ta_accum / (float)s->Ta_n;
                s->Ta_locked = true;
                s->Ta_accum = 0.0f; s->Ta_n = 0;
            }
        } else {
            s->Ta_accum = 0.0f; s->Ta_n = 0;
        }
        *duty_out = 0.0f; *u_des_W = 0.0f;
        return;
    }

    /* B) 오프 시 서서히 환경온도 트래킹 */
    if (P_meas_W < p->p_off_w && s->duty_last < p->duty_off){
        const float a = p->Ts / (p->track_tau_s + p->Ts);
        s->Ta_est += a * (T_meas_C - s->Ta_est);
    }

    /* C) 하드 과열 래치 */
    if (!s->overtemp_latch && T_meas_C >= p->T_hard_off) s->overtemp_latch = true;
    if (s->overtemp_latch  && T_meas_C <= p->T_hard_rst) s->overtemp_latch = false;
    if (s->overtemp_latch){
        s->I_T = 0.0f; s->I_P = 0.0f;
        s->duty_last = 0.0f; s->u_unsat = 0.0f; s->u_sat = 0.0f;
        *duty_out = 0.0f; *u_des_W = 0.0f; return;
    }

    /* D) 외부 루프: ΔT PI (외부 FF 없음, 세트포인트 필터 없음) */
    const float r = Tsp - s->Ta_est;      /* 목표 ΔT */
    const float y = T_meas_C - s->Ta_est; /* 실제 ΔT */
    const float e_T = r - y;

    const float Kc = p->Kc;
    const float Ki = (p->Ti > 1e-9f) ? (Kc / p->Ti) : 0.0f;

    const float u_fb  = Kc * e_T + s->I_T;
    s->u_unsat = u_fb;                    /* 외부 FF=0 */
    const float u_raw = s->u_unsat;

    /* 외부 포화 + 표준 AW */
    s->u_sat = clampf(u_raw, p->u_min_W, p->u_max_W);
    s->I_T   += Ki * p->Ts * e_T;
    if (p->Tt > 1e-9f){
        const float bc = (s->u_sat - u_raw);          /* 표준 back-calculation */
        s->I_T += (bc / p->Tt) * p->Ts;
    }
    u_des = s->u_sat; /* 전력 명령 */

    /* E) 내부 루프: 전력 PI + 파워맵 FF */
    const float e_P = u_des - P_meas_W;
    const float Pp  = p->KcP * e_P;
    const float KiP = (p->TiP > 1e-9f) ? (p->KcP / p->TiP) : 0.0f;
    float I2 = s->I_P + KiP * p->Ts * e_P;

    const float Rff = posf(R_est_ohm, 1e-3f);
    const float Vs  = posf(Vbus_V,    1e-3f);
    const float Vs2_over_R = (Vs*Vs) / Rff;

    float duty_ff = 0.0f;
    if (u_des > 0.0f){
        if (p->map_mode == CTRL_PWRMAP_ONOFF){
            duty_ff = (u_des / Vs2_over_R);           /* d = P·R/Vs² */
        } else {
            duty_ff = sqrtf(u_des / Vs2_over_R);      /* d = sqrt(P·R)/Vs */
        }
    }

    const float duty_raw = duty_ff + Pp + I2;
    const float duty_sat = clampf(duty_raw, p->duty_min, p->duty_max);

    /* 레이트리밋 */
    const float dmax = p->du_max * p->Ts;
    const float step = duty_sat - s->duty_last;
    if      (step >  dmax) duty = s->duty_last + dmax;
    else if (step < -dmax) duty = s->duty_last - dmax;
    else                   duty = duty_sat;
    s->duty_last = duty;

    /* 내부 AW는 최종 duty(레이트리밋 이후) 기준 */
    if (p->TtP > 1e-9f){
        s->I_P = I2 + ( (duty - duty_raw) * (p->Ts / p->TtP) );
    } else {
        s->I_P = I2;
    }

    /* F) 외부 트래킹 AW: 내부 포화/레이트리밋을 전력 등가로 환산하여 외부 적분 보정 */
    if (p->Tt > 1e-9f){
        float P_sat_hat;
        if (p->map_mode == CTRL_PWRMAP_ONOFF)   P_sat_hat = duty * Vs2_over_R;   /* P ≈ d·Vs²/R */
        else                                    P_sat_hat = duty*duty * Vs2_over_R; /* P ≈ d²·Vs²/R */
        const float delta = (P_sat_hat - u_des);    /* 내부 제약으로 실제 가능한 전력 - 외부 명령 */
        s->I_T += (delta / p->Tt) * p->Ts;          /* tracking back-calc */
    }

    *duty_out = duty;
    *u_des_W  = u_des;
}
