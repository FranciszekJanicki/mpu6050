#ifndef MPU6050_MPU6050_DMP_CONFIG_H
#define MPU6050_MPU6050_DMP_CONFIG_H

#include "mpu6050_config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MPU6050_DMP_MEMORY_BANKS 8
#define MPU6050_DMP_MEMORY_BANK_SIZE 256U
#define MPU6050_DMP_MEMORY_CHUNK_SIZE 16U
#define MPU6050_DMP_QUAT_SCALE (1.0F / (float32_t)(1ULL << 30ULL))

typedef union {
    uint8_t data[42];
    struct {
        uint8_t quat_data[16];
        uint8_t accel_data[6];
        uint8_t temp_data[2];
        uint8_t gyro_data[6];
    };
} mpu6050_dmp_packet_t;

#ifdef __cplusplus
}
#endif

#endif // MPU6050_MPU6050_DMP_CONFIG_H