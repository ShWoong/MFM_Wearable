#pragma once
#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    I2C_HandleTypeDef *hi2c;  // 예: &hi2c2
    uint16_t addr8;           // 8-bit I2C 주소 (7-bit << 1). 기본 0x60<<1
    uint32_t timeout_ms;      // HAL I2C 타임아웃 (ms)

    // 정수계산용 계수(Pololu 라이브러리와 동일)
    uint16_t vcodesMult, icodesMult, pinstantMult;
    uint8_t  vcodesShift, icodesShift, pinstantShift;

    // 최근 읽은 값들 (mV/mA/mW)
    int32_t instVoltageMillivolts;
    int32_t instCurrentMilliamps;
    int32_t instPowerMilliwatts;

    int32_t rmsVoltageMillivolts;
    int32_t rmsCurrentMilliamps;
    int32_t activePowerMilliwatts;
    int32_t reactivePowerMilliwatts;
    int32_t apparentPowerMilliwatts;

    uint8_t lastError;        // 0=OK, 그 외=오류
} ACS37800_Handle;

// ===== 초기화/설정 =====
void ACS37800_Init(ACS37800_Handle *h, I2C_HandleTypeDef *hi2c, uint8_t addr7);
void ACS37800_SetTimeout(ACS37800_Handle *h, uint32_t timeout_ms);

// Pololu 캐리어 보드용(점퍼에 따른 Rsense kΩ: 1/2/4)
void ACS37800_SetBoardPololu(ACS37800_Handle *h, uint8_t rsense_kohm);

// 범용 보드 파라미터 세팅 (isense_range[A], riso[Ω], rsense[Ω])
void ACS37800_SetBoardParameters(ACS37800_Handle *h, uint8_t isense_range,
                                 uint32_t riso, uint32_t rsense);

// 샘플 수 설정 (0~1023). 0=제로크로싱-제한 방식
HAL_StatusTypeDef ACS37800_SetSampleCount(ACS37800_Handle *h, uint16_t count);

// 쓰기 언락
HAL_StatusTypeDef ACS37800_EnableWriteAccess(ACS37800_Handle *h);

// EEPROM에 I2C 7-bit 주소 기록 (적용은 전원 재인가 후)
HAL_StatusTypeDef ACS37800_WriteEepromI2CAddress(ACS37800_Handle *h, uint8_t addr7);

// ===== 측정/읽기 =====
HAL_StatusTypeDef ACS37800_ReadRMSVoltageAndCurrent(ACS37800_Handle *h);
HAL_StatusTypeDef ACS37800_ReadActiveAndReactivePower(ACS37800_Handle *h);
HAL_StatusTypeDef ACS37800_ReadApparentPower(ACS37800_Handle *h);
HAL_StatusTypeDef ACS37800_ReadInstVoltageAndCurrent(ACS37800_Handle *h);
HAL_StatusTypeDef ACS37800_ReadInstPower(ACS37800_Handle *h);

// ===== 레지스터 I/O (32비트, LSB->MSB) =====
HAL_StatusTypeDef ACS37800_ReadReg(ACS37800_Handle *h, uint8_t reg, uint32_t *out);
HAL_StatusTypeDef ACS37800_WriteReg(ACS37800_Handle *h, uint8_t reg, uint32_t val);

// 마지막 에러 코드 반환(0=OK)
static inline uint8_t ACS37800_GetLastError(ACS37800_Handle *h) { return h->lastError; }

#ifdef __cplusplus
}
#endif
