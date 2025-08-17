
#ifndef FILTERS_H
#define FILTERS_H

#include <stdint.h>
#include <math.h>

#define FILTER_ORDER 2
#define SECTIONS 1
//#define FILTER_TAP_NUM 100
#define SAMPLE_SIZE 5
#define EMG_SAMPLE_SIZE 5000
#define FILTER_TAP_NUM 100
#define HPFILTER_TAP_NUM 101
#define PI 3.141592f

float EMGBWHPF(float input);
float EMGBWLPF(float input);
float BWLPF(float input, int8_t ch);
float BWLPF_1st(float input, int8_t ch);
float NEURAL_ACTIVATION(float emg);
float MUSCLE_ACTIVATION(float neural_activation);
float FORCE_GENERATION(float muscle_activation);
float STRETCH_SENSOR(void);
float EMG_SENSOR(void);
void FIR_Init(int num_taps, float cutoff_freq, float fs);
float FIR_Process(float input);
float MAF(float new_sample);
double MAFEMG(double new_sample);
float EWMAF(float new_measurement, float prev_ewma, float alpha);
float IntegralFilter(float input, float *prev_output, float alpha);

/*void HighPassFilter_Init(void);
float HighPassFilter_Process(float input);
float applyLowPassFilter(float input);

typedef struct {
    float estimate;
    float error_estimate;
    float error_measure;
    float kalman_gain;
} KMF;

void KMF_Init(KMF *kf, float init_estimate, float init_error_estimate, float error_measure);
void KMF_Update(KMF *kf, float measurement);*/

#endif /* FILTERS_H */
