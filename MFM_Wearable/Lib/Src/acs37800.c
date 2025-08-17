#include <acs37800.h>
#include <math.h>

// 내부: 정수 근사 계수 계산 (Pololu 방식과 동일한 로직)
static void calculateApproximation(uint64_t numerator, uint64_t denominator,
                                   uint16_t *outMult, uint8_t *outShift)
{
    float k = (float)numerator / (float)denominator;
    uint16_t mult = 0;
    uint8_t  shift = 0;

    for (uint8_t sc = 0; sc < 32; sc++)
    {
        uint32_t mc = (uint32_t)lroundf(k * (float)(1u << sc));
        if (mc > 0x7FFFu) break;
        mult = (uint16_t)mc;
        shift = sc;
    }
    while ((mult & 1u) == 0u && shift > 0u)
    {
        mult >>= 1;
        shift--;
    }
    *outMult = mult;
    *outShift = shift;
}

void ACS37800_Init(ACS37800_Handle *h, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    h->hi2c = hi2c;
    h->addr8 = (uint16_t)(addr7 << 1);  // HAL은 8-bit 주소 사용
    h->timeout_ms = 10;

    // 기본 계수(=1)로 초기화
    h->vcodesMult = 1; h->icodesMult = 1; h->pinstantMult = 1;
    h->vcodesShift = 0; h->icodesShift = 0; h->pinstantShift = 0;

    h->instVoltageMillivolts = 0;
    h->instCurrentMilliamps  = 0;
    h->instPowerMilliwatts   = 0;

    h->rmsVoltageMillivolts  = 0;
    h->rmsCurrentMilliamps   = 0;
    h->activePowerMilliwatts = 0;
    h->reactivePowerMilliwatts = 0;
    h->apparentPowerMilliwatts = 0;

    h->lastError = 0;
}

void ACS37800_SetTimeout(ACS37800_Handle *h, uint32_t timeout_ms)
{
    h->timeout_ms = timeout_ms;
}

void ACS37800_SetBoardPololu(ACS37800_Handle *h, uint8_t rsense_kohm)
{
    // 아두이노 라이브러리와 동일
    h->icodesMult = 17873;
    h->icodesShift = 14;

    switch (rsense_kohm)
    {
    case 1:
        h->vcodesMult = 18623;
        h->vcodesShift = 9;
        h->pinstantMult = 1299;
        h->pinstantShift = 0;
        break;
    case 2:
        h->vcodesMult = 18627;
        h->vcodesShift = 10;
        h->pinstantMult = 10395;
        h->pinstantShift = 4;
        break;
    default:
    case 4:
        h->vcodesMult = 18637;
        h->vcodesShift = 11;
        h->pinstantMult = 325;
        h->pinstantShift = 0;
        break;
    }
}

void ACS37800_SetBoardParameters(ACS37800_Handle *h, uint8_t isense_range,
                                 uint32_t riso, uint32_t rsense)
{
    // Pololu generic 계산식 포팅
    uint64_t numV = (uint64_t)(riso + rsense);
    uint64_t denV = (uint64_t)(110ull * rsense);
    calculateApproximation(numV, denV, &h->vcodesMult, &h->vcodesShift);

    uint64_t numI = (uint64_t)(2ull * isense_range);
    uint64_t denI = 55ull;
    calculateApproximation(numI, denI, &h->icodesMult, &h->icodesShift);

    uint64_t numP = (uint64_t)isense_range * (uint64_t)(riso + rsense) * 5ull;
    uint64_t denP = (uint64_t)rsense * 462ull;
    calculateApproximation(numP, denP, &h->pinstantMult, &h->pinstantShift);
}

HAL_StatusTypeDef ACS37800_ReadReg(ACS37800_Handle *h, uint8_t reg, uint32_t *out)
{
    HAL_StatusTypeDef s;
    s = HAL_I2C_Master_Transmit(h->hi2c, h->addr8, &reg, 1, h->timeout_ms);
    if (s != HAL_OK) { h->lastError = 1; return s; }

    uint8_t b[4];
    s = HAL_I2C_Master_Receive(h->hi2c, h->addr8, b, 4, h->timeout_ms);
    if (s != HAL_OK) { h->lastError = 2; return s; }

    // LSB -> MSB
    *out =  (uint32_t)b[0]
          | ((uint32_t)b[1] << 8)
          | ((uint32_t)b[2] << 16)
          | ((uint32_t)b[3] << 24);
    h->lastError = 0;
    return HAL_OK;
}

HAL_StatusTypeDef ACS37800_WriteReg(ACS37800_Handle *h, uint8_t reg, uint32_t val)
{
    uint8_t tx[5];
    tx[0] = reg;
    tx[1] = (uint8_t)(val & 0xFF);
    tx[2] = (uint8_t)((val >> 8) & 0xFF);
    tx[3] = (uint8_t)((val >> 16) & 0xFF);
    tx[4] = (uint8_t)((val >> 24) & 0xFF);

    HAL_StatusTypeDef s = HAL_I2C_Master_Transmit(h->hi2c, h->addr8, tx, 5, h->timeout_ms);
    if (s != HAL_OK) { h->lastError = 3; return s; }
    h->lastError = 0;
    return HAL_OK;
}

HAL_StatusTypeDef ACS37800_EnableWriteAccess(ACS37800_Handle *h)
{
    // ACCESS_CODE = 0x4F70656E 를 0x2F에 기록
    return ACS37800_WriteReg(h, 0x2F, 0x4F70656E);
}

HAL_StatusTypeDef ACS37800_SetSampleCount(ACS37800_Handle *h, uint16_t count)
{
    HAL_StatusTypeDef s;
    s = ACS37800_EnableWriteAccess(h);
    if (s != HAL_OK) return s;

    uint32_t reg;
    s = ACS37800_ReadReg(h, 0x1F, &reg);
    if (s != HAL_OK) return s;

    if (count > 1023) count = 1023;

    // Clear N(23:14) & BYPASS_N_EN(24) : 0b 0000 0001 1111 1111 1100 0000 0000 -> 반전 = 0xFE003FFF
    reg &= 0xFE003FFFu;
    if (count)
    {
        reg |= (1u << 24);           // BYPASS_N_EN = 1
        reg |= ((uint32_t)count << 14); // N = count
    }

    s = ACS37800_WriteReg(h, 0x1F, reg);
    return s;
}

HAL_StatusTypeDef ACS37800_WriteEepromI2CAddress(ACS37800_Handle *h, uint8_t addr7)
{
    HAL_StatusTypeDef s;
    s = ACS37800_EnableWriteAccess(h);
    if (s != HAL_OK) return s;

    uint32_t reg;
    s = ACS37800_ReadReg(h, 0x0F, &reg);
    if (s != HAL_OK) return s;

    // clear bits [9] and [8:2], then set
    reg = (reg & ~(uint32_t)0x3FCu) | (1u << 9) | (((uint32_t)(addr7 & 0x7F)) << 2);

    s = ACS37800_WriteReg(h, 0x0F, reg);
    if (s == HAL_OK)
    {
        // EEPROM write latency ~25ms
        HAL_Delay(30);
    }
    return s;
}

HAL_StatusTypeDef ACS37800_ReadRMSVoltageAndCurrent(ACS37800_Handle *h)
{
    uint32_t reg;
    HAL_StatusTypeDef s = ACS37800_ReadReg(h, 0x20, &reg);
    if (s != HAL_OK) return s;

    uint16_t vrms = (uint16_t)(reg & 0xFFFFu);
    uint16_t irms = (uint16_t)((reg >> 16) & 0xFFFFu);

    // 아두이노 라이브러리와 동일한 스케일: (x * mult >> shift >> 1)
    h->rmsVoltageMillivolts = ((int32_t)vrms * (int32_t)h->vcodesMult) >> h->vcodesShift;
    h->rmsVoltageMillivolts >>= 1;

    h->rmsCurrentMilliamps  = ((int32_t)irms * (int32_t)h->icodesMult) >> h->icodesShift;
    h->rmsCurrentMilliamps  >>= 1;

    return HAL_OK;
}

HAL_StatusTypeDef ACS37800_ReadActiveAndReactivePower(ACS37800_Handle *h)
{
    uint32_t reg;
    HAL_StatusTypeDef s = ACS37800_ReadReg(h, 0x21, &reg);
    if (s != HAL_OK) return s;

    int16_t  pactive = (int16_t)(reg & 0xFFFFu);
    uint16_t pimag   = (uint16_t)((reg >> 16) & 0xFFFFu);

    h->activePowerMilliwatts   = ((int32_t)pactive * (int32_t)h->pinstantMult) >> h->pinstantShift;
    h->reactivePowerMilliwatts = ((int32_t)pimag   * (int32_t)h->pinstantMult) >> h->pinstantShift;
    h->reactivePowerMilliwatts >>= 1;

    return HAL_OK;
}

HAL_StatusTypeDef ACS37800_ReadApparentPower(ACS37800_Handle *h)
{
    uint32_t reg;
    HAL_StatusTypeDef s = ACS37800_ReadReg(h, 0x22, &reg);
    if (s != HAL_OK) return s;

    uint16_t papparent = (uint16_t)(reg & 0xFFFFu);
    h->apparentPowerMilliwatts = ((int32_t)papparent * (int32_t)h->pinstantMult) >> h->pinstantShift;
    h->apparentPowerMilliwatts >>= 1;

    return HAL_OK;
}

HAL_StatusTypeDef ACS37800_ReadInstVoltageAndCurrent(ACS37800_Handle *h)
{
    uint32_t reg;
    HAL_StatusTypeDef s = ACS37800_ReadReg(h, 0x2A, &reg);
    if (s != HAL_OK) return s;

    int16_t vcodes = (int16_t)(reg & 0xFFFFu);
    int16_t icodes = (int16_t)((reg >> 16) & 0xFFFFu);

    h->instVoltageMillivolts = ((int32_t)vcodes * (int32_t)h->vcodesMult) >> h->vcodesShift;
    h->instCurrentMilliamps  = ((int32_t)icodes * (int32_t)h->icodesMult) >> h->icodesShift;

    return HAL_OK;
}

HAL_StatusTypeDef ACS37800_ReadInstPower(ACS37800_Handle *h)
{
    uint32_t reg;
    HAL_StatusTypeDef s = ACS37800_ReadReg(h, 0x2C, &reg);
    if (s != HAL_OK) return s;

    int16_t pinstant = (int16_t)(reg & 0xFFFFu);
    h->instPowerMilliwatts = ((int32_t)pinstant * (int32_t)h->pinstantMult) >> h->pinstantShift;

    return HAL_OK;
}
