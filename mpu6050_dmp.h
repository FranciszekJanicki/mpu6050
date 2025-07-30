#ifndef MPU6050_MPU6050_DMP_H
#define MPU6050_MPU6050_DMP_H

#include "mpu6050.h"
#include "mpu6050_config.h"
#include "mpu6050_dmp.h"
#include "mpu6050_registers.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    mpu6050_t mpu6050;
} mpu6050_dmp_t;

#ifdef __cplusplus
}
#endif

#endif // MPU6050_MPU6050_DMP_H