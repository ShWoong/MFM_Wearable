#include <filters.h>
#include <main.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>

/************************************************BUTTERWORTH HIGH PASS FILTER*************************************************/

/*INITIALIZING*/
/*ADC1(EMG)*/
float hpf_x_buffer1[SECTIONS][2] = {0};
float hpf_y_buffer1[SECTIONS][2] = {0};

float hpf_x_buffer20[SECTIONS][2] = {0};
float hpf_y_buffer20[SECTIONS][2] = {0};



/*FILTER COEFFICIENTS*/
float hpf_sos_40[SECTIONS][6] = {
    {8.3720e-01, -1.6744e+00, 8.3720e-01, 1.0000e+00, -1.6475e+00, 7.0090e-01},
};//40Hz
float hpf_sos_35[SECTIONS][6] = {
    {9.82385e-01, -1.96477e+00, 9.82385e-01, 1.00000e+00, -1.96451e+00, 9.65090e-01},
};//35Hz
float hpf_sos_20[SECTIONS][6] = {
    {9.3910e-01, -1.8782e+00, 9.3910e-01, 1.0000e+00, -1.8783e+00, 8.8100e-01},
};//20Hz
float hpf_sos_10[SECTIONS][6] = {
    {9.84106e-01, -1.96821e+00, 9.84106e-01, 1.00000e+00, -1.96922e+00, 9.70982e-01},
};//10Hz
float hpf_sos_1[SECTIONS][6] = {
    {9.98404e-01, -1.99681e+00, 9.98404e-01, 1.00000e+00, -1.99679e+00, 9.96810e-01},
};//1Hz


float EMGBWHPF(float input) {
	float output = 0;
	float xn = 0;


	for (int i = 0; i < SECTIONS; i++) {
		xn = (i == 0) ? input : output;

		output = hpf_sos_20[i][0] * xn + hpf_sos_20[i][1] * hpf_x_buffer20[i][0] + hpf_sos_20[i][2] * hpf_x_buffer20[i][1]
		- hpf_sos_20[i][4] * hpf_y_buffer20[i][0] - hpf_sos_20[i][5] * hpf_y_buffer20[i][1];

		hpf_x_buffer20[i][1] = hpf_x_buffer20[i][0];
		hpf_x_buffer20[i][0] = xn;
		hpf_y_buffer20[i][1] = hpf_y_buffer20[i][0];
		hpf_y_buffer20[i][0] = output;

	}
	return output;
}

/*******************************************INTEGRAL FILTER*******************************************/
/**
 * @brief 적분 필터: 입력 신호를 적분하여 DC 성분을 평활화.
 * @param input: 현재 입력 신호.
 * @param prev_output: 이전 출력값 (필터 상태 유지용).
 * @param alpha: 필터 계수 (0.0 ~ 1.0).
 * @return 필터링된 출력값.
 */
float IntegralFilter(float input, float *prev_output, float alpha) {
    // 적분 계산
    *prev_output = alpha * (*prev_output) + (1 - alpha) * input;
    return *prev_output;
}

/*************************************************BUTTERWORTH LOW PASS FILTER*************************************************/
/*INITILAIZING*/
/*ADC1(EMG)*/
float lpf_x_buffer10[SECTIONS][2] = {0};
float lpf_y_buffer10[SECTIONS][2] = {0};
/*ADC2(STRETCH)*/
float lpf_x_buffer200[SECTIONS][2] = {0};
float lpf_y_buffer200[SECTIONS][2] = {0};

float lpf_x_buffer14[SECTIONS][2] = {0};
float lpf_y_buffer14[SECTIONS][2] = {0};


float lpf_sos_2[SECTIONS][6] = {
		{1.55148423e-04, 3.10296847e-04, 1.55148423e-04, 1.00000000e+00, -1.96446058e+00, 9.65081174e-01},
		}; // 2 Hz

float lpf_sos_3[SECTIONS][6] = {
    {3.46041338e-04, 6.92082675e-04, 3.46041338e-04, 1.00000000e+00, -1.94669754e+00, 9.48081706e-01},
}; // 3 Hz

float lpf_sos_4[SECTIONS][6] = {
    {6.09854719e-04, 1.21970944e-03, 6.09854719e-04, 1.00000000e+00, -1.92894226e+00, 9.31381682e-01},
}; // 4 Hz
float lpf_sos_14[SECTIONS][6] = {
    {2.50000e-01, 5.00000e-01, 2.50000e-01, 1.00000e+00, -1.75100e+00, 7.80000e-01},
};//14Hz
float lpf_sos_50[SECTIONS][6] = {
    {2.5460e-03, 5.0920e-03, 2.5460e-03, 1.00000e+00, -1.85290e+00, 8.6280e-01},
};//50Hz
float lpf_sos_53[SECTIONS][6] = {
    {4.8240e-03, 9.6480e-03, 4.8240e-03, 1.0000e+00, -1.7990e+00, 8.1800e-01},
};//53Hz
float lpf_sos_200[SECTIONS][6] = {
    {6.7500e-02, 1.3490e-01, 6.7500e-02, 1.0000e+00, -1.1429e+00, 4.1280e-01},
};//200Hz
float lpf_sos_3_2700[SECTIONS][6] = {
    {1.21e-05, 2.42e-05, 1.21e-05, 1.00000000e+00, -1.99000000e+00, 9.90000000e-01},
}; // 3 Hz 컷오프, fs=2700Hz
float lpf_sos_4_2700[SECTIONS][6] = {
    {2.16e-05, 4.32e-05, 2.16e-05, 1.00000000e+00, -1.98600000e+00, 9.87000000e-01},
}; // 4 Hz 컷오프, fs = 2700 Hz



float BWLPF(float input, int8_t cf) {
	float output = 0;
	float xn = 0;

	if (cf == 4){

		for (int i = 0; i < SECTIONS; i++) {
			xn = (i == 0) ? input : output;

			output = lpf_sos_4[i][0] * xn + lpf_sos_4[i][1] * lpf_x_buffer10[i][0] + lpf_sos_4[i][2] * lpf_x_buffer10[i][1]
			- lpf_sos_4[i][4] * lpf_y_buffer10[i][0] - lpf_sos_4[i][5] * lpf_y_buffer10[i][1];

			lpf_x_buffer10[i][1] = lpf_x_buffer10[i][0];
			lpf_x_buffer10[i][0] = xn;
			lpf_y_buffer10[i][1] = lpf_y_buffer10[i][0];
			lpf_y_buffer10[i][0] = output;
		}
	}
	else if (cf == 3){

	    for (int i = 0; i < SECTIONS; i++) {
	        xn = (i == 0) ? input : output;

	        output = lpf_sos_3[i][0] * xn + lpf_sos_3[i][1] * lpf_x_buffer10[i][0] + lpf_sos_3[i][2] * lpf_x_buffer10[i][1]
	        - lpf_sos_3[i][4] * lpf_y_buffer10[i][0] - lpf_sos_3[i][5] * lpf_y_buffer10[i][1];

	        lpf_x_buffer10[i][1] = lpf_x_buffer10[i][0];
	        lpf_x_buffer10[i][0] = xn;
	        lpf_y_buffer10[i][1] = lpf_y_buffer10[i][0];
	        lpf_y_buffer10[i][0] = output;
	    }
	}
    return output;
}

float EMGBWLPF(float input) {
	float output = 0;
	float xn = 0;

	for (int i = 0; i < SECTIONS; i++) {
		xn = (i == 0) ? input : output;

		output = lpf_sos_4_2700[i][0] * xn + lpf_sos_4_2700[i][1] * lpf_x_buffer200[i][0] + lpf_sos_4_2700[i][2] * lpf_x_buffer200[i][1]
		- lpf_sos_4_2700[i][4] * lpf_y_buffer200[i][0] - lpf_sos_4_2700[i][5] * lpf_y_buffer200[i][1];

		lpf_x_buffer200[i][1] = lpf_x_buffer200[i][0];
		lpf_x_buffer200[i][0] = xn;
		lpf_y_buffer200[i][1] = lpf_y_buffer200[i][0];
		lpf_y_buffer200[i][0] = output;
	}
    return output;
}

/*************************************************FIRST-ORDER BUTTERWORTH LOW PASS FILTER*************************************************/
float lpf1_sos_1Hz[1][6] = {
		{6.24403505e-03, 6.24403505e-03, 0.00000000e+00, 1.00000000e+00, -9.87511930e-01, 0.00000000e+00},
		}; // 1 Hz
float lpf1_sos_2Hz[1][6] = {
		{1.24110619e-02, 1.24110619e-02, 0.00000000e+00, 1.00000000e+00, -9.75177876e-01, 0.00000000e+00},
		}; // 2 Hz
/* 2Hz 1차 필터 계수 */
float lpf1_sos_3Hz[1][6] = {
		{1.85029745e-02, 1.85029745e-02, 0.00000000e+00, 1.00000000e+00, -9.62994051e-01, 0.00000000e+00},
		}; // 3 Hz

/* 입력 및 출력 버퍼 */
float lpf1_x_buffer[1] = {0};
float lpf1_y_buffer[1] = {0};

float BWLPF_1st(float input, int8_t cf) {
    float output = 0;
    float xn = input; // 현재 입력 신호

    if (cf == 1) { // 1Hz 1차 필터
            output = lpf1_sos_1Hz[0][0] * xn
                   + lpf1_sos_1Hz[0][1] * lpf1_x_buffer[0]
                   - lpf1_sos_1Hz[0][4] * lpf1_y_buffer[0];

            lpf1_x_buffer[0] = xn;         // 입력 버퍼 업데이트
            lpf1_y_buffer[0] = output;    // 출력 버퍼 업데이트
        }
    else if (cf == 2) { // 2Hz 1차 필터
        output = lpf1_sos_2Hz[0][0] * xn
               + lpf1_sos_2Hz[0][1] * lpf1_x_buffer[0]
               - lpf1_sos_2Hz[0][4] * lpf1_y_buffer[0];

        lpf1_x_buffer[0] = xn;         // 입력 버퍼 업데이트
        lpf1_y_buffer[0] = output;    // 출력 버퍼 업데이트

    } else if (cf == 3) { // 3Hz 1차 필터
        output = lpf1_sos_3Hz[0][0] * xn
               + lpf1_sos_3Hz[0][1] * lpf1_x_buffer[0]
               - lpf1_sos_3Hz[0][4] * lpf1_y_buffer[0];

        lpf1_x_buffer[0] = xn;         // 입력 버퍼 업데이트
        lpf1_y_buffer[0] = output;    // 출력 버퍼 업데이트
    } else {
        // 지원하지 않는 채널
        output = xn;
    }

    return output;
}

/*******************************************FINITE IMPULSE RESPONSE LOW PASS FILTER*******************************************/
/* FIR 필터 계수 및 상태 */
static float *fir_coeff = NULL; // 필터 계수
static float *fir_state = NULL; // 상태 저장
static int fir_num_taps = 0;    // 탭 수

/* FIR 필터 초기화 */
void FIR_Init(int num_taps, float cutoff_freq, float fs) {
    if (fir_coeff != NULL) {
        free(fir_coeff); // 이전 메모리 해제
    }
    if (fir_state != NULL) {
        free(fir_state);
    }

    fir_num_taps = num_taps;
    fir_coeff = (float *)malloc(num_taps * sizeof(float));
    fir_state = (float *)calloc(num_taps, sizeof(float));

    if (fir_coeff == NULL || fir_state == NULL) {
        // 메모리 할당 실패 시 처리
        exit(1);
    }

    // FIR 계수 계산 (Hamming 윈도우 사용)
    float nyquist = fs / 2.0;
    float normalized_cutoff = cutoff_freq / nyquist;
    for (int i = 0; i < num_taps; i++) {
        if (i == (num_taps - 1) / 2) {
            fir_coeff[i] = normalized_cutoff;
        } else {
            float n = i - (num_taps - 1) / 2.0;
            fir_coeff[i] = sinf(M_PI * normalized_cutoff * n) / (M_PI * n);
            fir_coeff[i] *= 0.54 - 0.46 * cosf(2.0 * M_PI * i / (num_taps - 1)); // Hamming 윈도우 적용
        }
    }
}

/* FIR 필터 처리 */
float FIR_Process(float input) {
    float output = 0.0;

    // 상태 배열 업데이트
    for (int i = fir_num_taps - 1; i > 0; i--) {
        fir_state[i] = fir_state[i - 1];
    }
    fir_state[0] = input;

    // FIR 필터링 수행
    for (int i = 0; i < fir_num_taps; i++) {
        output += fir_state[i] * fir_coeff[i];
    }

    return output;
}

/*******************************************FINITE IMPULSE RESPONSE HIGH PASS FILTER******************************************/
/*static float highPassFilterCoeffs[FILTER_TAP_NUM] = {
    -3.19998918e-06, -3.23629810e-06, -3.34507992e-06, -3.52590537e-06, -3.77806088e-06,
    -4.10055137e-06, -4.49210415e-06, -4.95117398e-06, -5.47594919e-06, -6.06435875e-06,
    -6.71408052e-06, -7.42255037e-06, -8.18697234e-06, -9.00432962e-06, -9.87139651e-06,
    -1.07847511e-05, -1.17407888e-05, -1.27357366e-05, -1.37656680e-05, -1.48265182e-05,
    -1.59141005e-05, -1.70241229e-05, -1.81522044e-05, -1.92938932e-05, -2.04446835e-05,
    -2.16000336e-05, -2.27553839e-05, -2.39061747e-05, -2.50478644e-05, -2.61759473e-05,
    -2.72859713e-05, -2.83735556e-05, -2.94344081e-05, -3.04643419e-05, -3.14592925e-05,
    -3.24153332e-05, -3.33286909e-05, -3.41957610e-05, -3.50131215e-05, -3.57775468e-05,
    -3.64860198e-05, -3.71357447e-05, -3.77241573e-05, -3.82489352e-05, -3.87080075e-05,
    -3.90995624e-05, -3.94220547e-05, -3.96742114e-05, -3.98550376e-05, -3.99638196e-05,
     9.99963200e-01, -3.99638196e-05, -3.98550376e-05, -3.96742114e-05, -3.94220547e-05,
    -3.90995624e-05, -3.87080075e-05, -3.82489352e-05, -3.77241573e-05, -3.71357447e-05,
    -3.64860198e-05, -3.57775468e-05, -3.50131215e-05, -3.41957610e-05, -3.33286909e-05,
    -3.24153332e-05, -3.14592925e-05, -3.04643419e-05, -2.94344081e-05, -2.83735556e-05,
    -2.72859713e-05, -2.61759473e-05, -2.50478644e-05, -2.39061747e-05, -2.27553839e-05,
    -2.16000336e-05, -2.04446835e-05, -1.92938932e-05, -1.81522044e-05, -1.70241229e-05,
    -1.59141005e-05, -1.48265182e-05, -1.37656680e-05, -1.27357366e-05, -1.17407888e-05,
    -1.07847511e-05, -9.87139651e-06, -9.00432962e-06, -8.18697234e-06, -7.42255037e-06,
    -6.71408052e-06, -6.06435875e-06, -5.47594919e-06, -4.95117398e-06, -4.49210415e-06,
    -4.10055137e-06, -3.77806088e-06, -3.52590537e-06, -3.34507992e-06, -3.23629810e-06,
    -3.19998918e-06
};

// 필터 상태
static float highPassFilterState[HPFILTER_TAP_NUM] = {0};

// FIR 고역 통과 필터 초기화
void HighPassFilter_Init(void) {
    for (int i = 0; i < HPFILTER_TAP_NUM; ++i) {
        highPassFilterState[i] = 0;
    }
}

// FIR 고역 통과 필터 처리 함수
float HighPassFilter_Process(float input) {
    for (int i = HPFILTER_TAP_NUM - 1; i > 0; --i) {
        highPassFilterState[i] = highPassFilterState[i - 1];
    }
    highPassFilterState[0] = input;

    float output = 0;
    for (int i = 0; i < HPFILTER_TAP_NUM; ++i) {
        output += highPassFilterState[i] * highPassFilterCoeffs[i];
    }

    return output;
}*/

/****************************************************MOVING AVERAGE FILTER****************************************************/
float MAFTemp(float new_sample) {
    static float samples[SAMPLE_SIZE] = {0};
    static int index = 0;
    static float sum = 0;
    float average = 0;

    // 이전 합계에서 가장 오래된 샘플 제거
    sum -= samples[index];
    // 새 샘플 추가
    samples[index] = new_sample;
    sum += new_sample;
    // 다음 샘플을 위한 인덱스 업데이트
    index = (index + 1) % SAMPLE_SIZE;

    // 평균 계산
    average = sum / SAMPLE_SIZE;

    return average;
}

double MAFPower(double new_sample) {
    static float samples[EMG_SAMPLE_SIZE] = {0};
    static int index = 0;
    static float sum = 0;
    float average = 0;

    // 이전 합계에서 가장 오래된 샘플 제거
    sum -= samples[index];
    // 새 샘플 추가
    samples[index] = new_sample;
    sum += new_sample;
    // 다음 샘플을 위한 인덱스 업데이트
    index = (index + 1) % EMG_SAMPLE_SIZE;

    // 평균 계산
    average = sum / EMG_SAMPLE_SIZE;

    return average;
}

/*****************************************EXPONENTIAL WEIGHTED MOVING AVERAGE FILTER******************************************/
float EWMAF(float new_measurement, float prev_ewma, float alpha) {
    return alpha * new_measurement + (1 - alpha) * prev_ewma;
}

/*void KMF_Init(KMF *kf, float init_estimate, float init_error_estimate, float error_measure) {
    kf->estimate = init_estimate;
    kf->error_estimate = init_error_estimate;
    kf->error_measure = error_measure;
}

void KMF_Update(KMF *kf, float measurement) {
    kf->kalman_gain = kf->error_estimate / (kf->error_estimate + kf->error_measure);
    kf->estimate = kf->estimate + kf->kalman_gain * (measurement - kf->estimate);
    kf->error_estimate = (1 - kf->kalman_gain) * kf->error_estimate;
}*/

/************************************************NEURAL ACTIVATION CALCULATION************************************************/
float na = 0, nat1 = 0, nat2 = 0;
float estimated_angle_emg = 0.0f;
float angle_velocity_emg = 0.0f;
float estimated_angle_fused = 0.0f;

float NEURAL_ACTIVATION(float emg){
	float gma1 = -0.75, gma2 = -0.125;
	float bet1 = gma1 + gma2, bet2 = gma1*gma2, alp = 1 + bet1 + bet2;

	nat2 = nat1;
	nat1 = na;
	na = (alp*emg - bet1*nat1 - bet2*nat2);
	if(na<0){
	  na=0;
	}

	return na;
}

/************************************************MUSCLE ACTIVATION CALCULATION************************************************/
float MUSCLE_ACTIVATION(float neural_activation){
	float normalized = neural_activation/2;
	//float A = -0.03, max_value = 33.81404;
	float A = -0.2;
	float ma = (exp(A*normalized) - 1)/(exp(A) - 1);
	//float ma_cal = ma/max_value*100;
	float ma_cal = ma*100.0-40;
	if (ma_cal < 0){
		ma_cal = 0;
	}

	return (int32_t)round(ma_cal);
}

/******************************************************TORQUE GENERATION******************************************************/
float FORCE_GENERATION(float muscle_activation) {
    float q0 = -2.06f, q1 = 6.16f, q2 = -3.13f;
    float pa = 13.9f * PI / 180.0f; // pennation angle (radian)
    float lt = 34.6f;   // 힘줄 길이
    float lm0 = 7.6f;   // 최적 근섬유 길이
    float Fm0 = 848.8f; // 최대 등척성 근력

    // 여기서는 근섬유 길이를 최적 길이의 50%로 가정
    float lm = lm0 * 0.5f;
    float l = lm / lm0;
    float fA = 0.0f;
    // 유효 범위 검사: 0.5 <= l <= 1.5
    if (l >= 0.5f && l <= 1.5f) {
        fA = q0 + q1 * l + q2 * (l * l);
    } else {
        fA = 0.0f;
    }
    float fP = expf(10.0f * l - 15.0f);
    float fV = 1.0f; // 수축 속도 계수 (상수로 가정)

    float FmA = fA * fV * muscle_activation * Fm0;
    float FmP = fP * Fm0;
    float Fmt = (FmA + FmP) * cosf(pa);
    return Fmt;
}


/***********************************************STRETCH SENSOR CAP CALCULATION************************************************/
/*float STRETCH_SENSOR(void){
	const float C_stray = 7, adc_max = 4096;
	float adc;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
	sConfig.Channel = ADC_CHANNEL_12;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK){
		adc = HAL_ADC_GetValue(&hadc1);
	}

	float C = adc*C_stray/(adc_max-adc);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);

	return C;
}*/


