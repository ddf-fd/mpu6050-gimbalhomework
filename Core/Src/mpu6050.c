#include "mpu6050.h"

extern I2C_HandleTypeDef hi2c1;

static uint8_t mpu_addr = 0xD0;

static void wr(uint8_t reg, uint8_t val)
{
    HAL_I2C_Mem_Write(&hi2c1, mpu_addr, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, 10);
}

static void rd(uint8_t reg, uint8_t *buf, uint16_t n)
{
    HAL_I2C_Mem_Read(&hi2c1, mpu_addr, reg, I2C_MEMADD_SIZE_8BIT, buf, n, 10);
}

uint8_t MPU6050_Init(void)
{
    wr(0x6B, 0x00);
    HAL_Delay(50);
    wr(0x19, 0x07);
    wr(0x1A, 0x03);
    wr(0x1B, 0x08);
    wr(0x1C, 0x08);
    return 0;
}

uint8_t MPU6050_Read(MPU6050_Data *d)
{
    uint8_t buf[14];

    rd(0x3B, buf, 14);

    d->ax = (int16_t)((buf[0] << 8) | buf[1]) / 8192.0f;
    d->ay = (int16_t)((buf[2] << 8) | buf[3]) / 8192.0f;
    d->az = (int16_t)((buf[4] << 8) | buf[5]) / 8192.0f;
    d->gx = (int16_t)((buf[8]  << 8) | buf[9])  / 65.5f;
    d->gy = (int16_t)((buf[10] << 8) | buf[11]) / 65.5f;
    d->gz = (int16_t)((buf[12] << 8) | buf[13]) / 65.5f;
    return 0;
}
