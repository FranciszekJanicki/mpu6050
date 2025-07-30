#ifndef MPU6050_MPU6050_UTILITY_H
#define MPU6050_MPU6050_UTILITY_H

#include "mpu6050_config.h"

#ifdef __cplusplus
extern "C" {
#endif

inline float32_t mpu6050_gyro_range_to_scale(mpu6050_gyro_range_t range)
{
    switch (range) {
        case MPU6050_GYRO_RANGE_250DPS:
            return 131.0F;
        case MPU6050_GYRO_RANGE_500DPS:
            return 65.5F;
        case MPU6050_GYRO_RANGE_1000DPS:
            return 32.8F;
        case MPU6050_GYRO_RANGE_2000DPS:
            return 16.4F;
        default:
            return 0.0F;
    }
}

inline float32_t mpu6050_accel_range_to_scale(mpu6050_accel_range_t range)
{
    switch (range) {
        case MPU6050_ACCEL_RANGE_2G:
            return 16384.0F;
        case MPU6050_ACCEL_RANGE_4G:
            return 8192.0F;
        case MPU6050_ACCEL_RANGE_8G:
            return 4096.0F;
        case MPU6050_ACCEL_RANGE_16G:
            return 2048.0F;
        default:
            return 0.0F;
    }
}

inline uint8_t mpu6050_sampling_rate_to_divider(uint32_t sampling_rate,
                                                mpu6050_dlpf_t dlpf)
{
    return dlpf == MPU6050_DLPF_BW_256
               ? (uint8_t)((MPU6050_GYRO_OUTPUT_RATE_DLPF_DIS_HZ /
                            sampling_rate) -
                           1U)
               : (uint8_t)((MPU6050_GYRO_OUTPUT_RATE_DLPF_EN_HZ /
                            sampling_rate) -
                           1U);
}

#ifdef __cplusplus
}
#endif

#endif // MPU6050_MPU6050_UTILITY_H