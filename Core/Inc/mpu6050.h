#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

typedef struct {
    float ax, ay, az;    // 加速度，单位 g
    float gx, gy, gz;    // 角速度，单位 deg/s
} MPU6050_Data;


uint8_t MPU6050_Init(void);
uint8_t MPU6050_Read(MPU6050_Data *d);

#endif
