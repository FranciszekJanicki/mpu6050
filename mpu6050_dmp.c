// #include "mpu6050.h"
// #include "mpu6050_config.h"
// #include "mpu6050_img.h"

// mpu6050_err_t mpu6050_initialize(mpu6050_t const* mpu6050)
// {
//     mpu6050_set_x_gyro_offset(mpu6050, 220);
//     mpu6050_set_y_gyro_offset(mpu6050, 76);
//     mpu6050_set_z_gyro_offset(mpu6050, -85);
//     mpu6050_set_z_accel_offset(mpu6050, 1788);

//     mpu6050_device_wake_up(mpu6050);
//     mpu6050_set_sleep_enabled(mpu6050, false);
//     mpu6050_set_memory_bank(mpu6050, 0x10, true, true);
//     mpu6050_set_memory_start_address(mpu6050, 0x06);
//     mpu6050_set_memory_bank(mpu6050, 0, false, false);
//     mpu6050_get_otp_bank_valid(mpu6050);

//     mpu6050_set_slave_address(mpu6050, 0, 0x7F);
//     mpu6050_set_i2c_master_mode_enabled(mpu6050, false);
//     mpu6050_set_slave_address(mpu6050, 0, 0x68);
//     mpu6050_reset_i2c_master(mpu6050);
//     mpu6050_set_clock_source(mpu6050, Clock::PLL_ZGYRO);
//     mpu6050_set_int_dmp_enabled(mpu6050, true);
//     mpu6050_set_int_fifo_overflow_enabled(mpu6050, true);
//     mpu6050_set_sampling_rate(mpu6050, 200, DLPF::BW_42); // 1 / (1 + 4) =
//     200 Hz mpu6050_set_external_frame_sync(mpu6050, ExtSync::TEMP_OUT_L);
//     mpu6050_set_dlpf_mode(mpu6050, DLPF::BW_42);
//     mpu6050_set_full_scale_gyro_range(mpu6050, GYRO_RANGE);
//     mpu6050_write_memory_block(mpu6050, mpu6050_img, sizeof(mpu6050_img),
//     0x00, 0x00);

//     uint8_t mpu6050_update[] = {0x00, 0x01};
//     mpu6050_write_memory_block(mpu6050, dmp_update, sizeof(dmp_update), 0x02,
//     0x16); mpu6050_set_dmp_config1(mpu6050, 0x03);
//     mpu6050_set_dmp_config2(mpu6050, 0x00);
//     mpu6050_set_otp_bank_valid(mpu6050, false);

//     mpu6050_set_motion_detection_threshold(mpu6050, 2);
//     mpu6050_set_zero_motion_detection_threshold(mpu6050, 156);
//     mpu6050_set_motion_detection_duration(mpu6050, 80);
//     mpu6050_set_zero_motion_detection_duration(mpu6050, 0);
//     mpu6050_set_fifo_enabled(mpu6050, true);
//     mpu6050_reset_dmp(mpu6050);
//     mpu6050_set_dmp_enabled(mpu6050, false);
//     mpu6050_get_int_status(mpu6050);
//     mpu6050_reset_fifo(mpu6050);
//     mpu6050_set_dmp_enabled(mpu6050, true);
// }

// mpu6050_err_t mpu6050_get_dmp_packet(mpu6050_t const* mpu6050,
// mpu6050_dmp_packet_t* packet)
// {
//     mpu6050_dmp_packet_t packet;

//     if (!mpu6050_get_int_dmp_status(mpu6050)) {
//         return packet;
//     }

//     uint16_t fifo_count = mpu6050_get_fifo_count(mpu6050);
//     if (fifo_count >= MPU6050_FIFO_MAX_COUNT) {
//         mpu6050_reset_fifo(mpu6050);
//         return packet;
//     }

//     while (fifo_count < sizeof(packet.data)) {
//         fifo_count = mpu6050_get_fifo_count(mpu6050);
//     }

//     mpu6050_get_fifo_bytes(mpu6050, packet.data, sizeof(packet.data));

//     return packet;
// }

// mpu6050_err_t mpu6050_get_dmp_accel_data_raw(mpu6050_t const* mpu6050)
// {}

// mpu6050_err_t mpu6050_get_dmp_accel_data_scaled(mpu6050_t const* mpu6050)
// {}

// mpu6050_err_t mpu6050_get_dmp_temp_data_raw(mpu6050_t const* mpu6050)
// {}

// mpu6050_err_t mpu6050_get_dmp_temp_data_scaled(mpu6050_t const* mpu6050)
// {}

// mpu6050_err_t mpu6050_get_dmp_gyro_data_raw(mpu6050_t const* mpu6050)
// {}

// mpu6050_err_t mpu6050_get_dmp_gyro_data_scaled(mpu6050_t const* mpu6050)
// {}

// mpu6050_err_t mpu6050_get_dmp_quat_data_raw(mpu6050_t const* mpu6050)
// {}

// mpu6050_err_t mpu6050_get_dmp_quat_data_scaled(mpu6050_t const* mpu6050)
// {}

// mpu6050_err_t mpu6050_set_dmp_memory_bank(mpu6050_t const* mpu6050,
//                                           uint8_t bank,
//                                           bool prefetch_enabled,
//                                           bool user_bank)
// {
//     uint8_t data = bank & 0x1F;
//     if (user_bank)
//         data |= 0x20;
//     if (prefetch_enabled)
//         data |= 0x40;
//     mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_BANK_SEL, data);
// }

// mpu6050_err_t mpu6050_read_dmp_memory_block(mpu6050_t const* mpu6050,
//                                             uint8_t* data,
//                                             size_t data_size,
//                                             uint8_t bank,
//                                             uint8_t address)
// {
//     mpu6050_set_memory_bank(mpu6050, bank);
//     mpu6050_set_memory_start_address(mpu6050, address);

//     for (uint32_t i = 0; i < data_size;) {
//         uint8_t chunk_size = mpu6050_MEMORY_CHUNK_SIZE;

//         if (i + chunk_size > data_size) {
//             chunk_size = data_size - i;
//         }
//         if (chunk_size > 256 - address) {
//             chunk_size = 256 - address;
//         }

//         mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_MEM_R_W, data + i,
//         chunk_size); i += chunk_size; address += chunk_size;

//         if (i < data_size) {
//             if (address == 0) {
//                 bank++;
//             }
//             mpu6050_set_memory_bank(mpu6050, bank, false, false);
//             mpu6050_set_memory_start_address(mpu6050, address);
//         }
//     }
// }

// mpu6050_err_t mpu6050_write_dmp_memory_block(mpu6050_t const* mpu6050,
//                                              uint8_t* data,
//                                              size_t data_size,
//                                              uint8_t bank,
//                                              uint8_t address)
// {
//     mpu6050_set_memory_bank(mpu6050, bank);
//     mpu6050_set_memory_start_address(mpu6050, address);

//     for (uint32_t i = 0; i < data_size;) {
//         uint8_t chunk_size = MPU6050_DMP_MEMORY_CHUNK_SIZE;

//         if (i + chunk_size > data_size) {
//             chunk_size = data_size - i;
//         }
//         if (chunk_size > 256 - address) {
//             chunk_size = 256 - address;
//         }

//         mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_MEM_R_W, data +
//         i, chunk_size); i += chunk_size; address += chunk_size;

//         if (i < data_size) {
//             if (address == 0) {
//                 bank++;
//             }
//             mpu6050_set_memory_bank(mpu6050, bank);
//             mpu6050_set_memory_start_address(mpu6050, address);
//         }
//     }
// }

// mpu6050_err_t mpu6050_write_dmp_configuration_set(mpu6050_t const* mpu6050,
//                                                   uint8_t* data,
//                                                   size_t data_size)
// {
//     for (uint32_t i = 0; i < data_size;) {
//         uint8_t bank = data[i++];
//         uint8_t offset = data[i++];
//         uint8_t length = data[i++];

//         if (length > 0) {
//             mpu6050_write_memory_block(mpu6050, data + i, length, bank,
//             offset); i += length;
//         } else {
//             if (data[i++] == 0x01) {
//                 mpu6050_bus_write_data(mpu6050,
//                 MPU6050_REG_ADDRESS_INT_ENABLE, 0x32);
//             }
//         }
//     }
// }
