#ifndef MPU6050_MPU6050_H
#define MPU6050_MPU6050_H

#include "mpu6050_config.h"
#include "mpu6050_registers.h"

typedef struct {
    mpu6050_config_t config;
    mpu6050_interface_t interface;
} mpu6050_t;

mpu6050_err_t mpu6050_initialize(mpu6050_t* mpu6050,
                                 mpu6050_config_t const* config,
                                 mpu6050_interface_t const* interface);
mpu6050_err_t mpu6050_deinitialize(mpu6050_t* mpu6050);

mpu6050_err_t mpu6050_get_accel_data_x_scaled(mpu6050_t const* mpu6050, float32_t* scaled);
mpu6050_err_t mpu6050_get_accel_data_y_scaled(mpu6050_t const* mpu6050, float32_t* scaled);
mpu6050_err_t mpu6050_get_accel_data_z_scaled(mpu6050_t const* mpu6050, float32_t* scaled);
mpu6050_err_t mpu6050_get_accel_data_scaled(mpu6050_t const* mpu6050, vec3_float32_t* scaled);

mpu6050_err_t mpu6050_get_temp_data_scaled(mpu6050_t const* mpu6050, float32_t* scaled);

mpu6050_err_t mpu6050_get_gyro_data_x_scaled(mpu6050_t const* mpu6050, float32_t* scaled);
mpu6050_err_t mpu6050_get_gyro_data_y_scaled(mpu6050_t const* mpu6050, float32_t* scaled);
mpu6050_err_t mpu6050_get_gyro_data_z_scaled(mpu6050_t const* mpu6050, float32_t* scaled);
mpu6050_err_t mpu6050_get_gyro_data_scaled(mpu6050_t const* mpu6050, vec3_float32_t* scaled);

mpu6050_err_t mpu6050_get_accel_data_x_raw(mpu6050_t const* mpu6050, int16_t* raw);
mpu6050_err_t mpu6050_get_accel_data_y_raw(mpu6050_t const* mpu6050, int16_t* raw);
mpu6050_err_t mpu6050_get_accel_data_z_raw(mpu6050_t const* mpu6050, int16_t* raw);
mpu6050_err_t mpu6050_get_accel_data_raw(mpu6050_t const* mpu6050, vec3_int16_t* raw);

mpu6050_err_t mpu6050_get_temp_data_raw(mpu6050_t const* mpu6050, int16_t* raw);

mpu6050_err_t mpu6050_get_gyro_data_x_raw(mpu6050_t const* mpu6050, int16_t* raw);
mpu6050_err_t mpu6050_get_gyro_data_y_raw(mpu6050_t const* mpu6050, int16_t* raw);
mpu6050_err_t mpu6050_get_gyro_data_z_raw(mpu6050_t const* mpu6050, int16_t* raw);
mpu6050_err_t mpu6050_get_gyro_data_raw(mpu6050_t const* mpu6050, vec3_int16_t* raw);

mpu6050_err_t mpu6050_ext_slv_read(mpu6050_t const* mpu6050,
                                   mpu6050_slave_num_t slave_num,
                                   uint8_t address,
                                   uint8_t* data,
                                   uint8_t data_size);

mpu6050_err_t mpu6050_ext_slv_write(mpu6050_t const* mpu6050,
                                    mpu6050_slave_num_t slave_num,
                                    uint8_t address,
                                    uint8_t const* data,
                                    uint8_t data_size);

mpu6050_err_t mpu6050_fifo_read(mpu6050_t const* mpu6050, uint8_t* data, uint8_t data_size);

mpu6050_err_t mpu6050_fifo_write(mpu6050_t const* mpu6050, uint8_t const* data, uint8_t data_size);

mpu6050_err_t mpu6050_get_xg_offs_tc_reg(mpu6050_t const* mpu6050, mpu6050_xg_offs_tc_reg_t* reg);
mpu6050_err_t mpu6050_set_xg_offs_tc_reg(mpu6050_t const* mpu6050,
                                         mpu6050_xg_offs_tc_reg_t const* reg);

mpu6050_err_t mpu6050_get_yg_offs_tc_reg(mpu6050_t const* mpu6050, mpu6050_yg_offs_tc_reg_t* reg);
mpu6050_err_t mpu6050_set_yg_offs_tc_reg(mpu6050_t const* mpu6050,
                                         mpu6050_yg_offs_tc_reg_t const* reg);

mpu6050_err_t mpu6050_get_zg_offs_tc_reg(mpu6050_t const* mpu6050, mpu6050_zg_offs_tc_reg_t* reg);
mpu6050_err_t mpu6050_set_zg_offs_tc_reg(mpu6050_t const* mpu6050,
                                         mpu6050_zg_offs_tc_reg_t const* reg);

mpu6050_err_t mpu6050_get_x_fine_gain_reg(mpu6050_t const* mpu6050, mpu6050_x_fine_gain_reg_t* reg);
mpu6050_err_t mpu6050_set_x_fine_gain_reg(mpu6050_t const* mpu6050,
                                          mpu6050_x_fine_gain_reg_t const* reg);

mpu6050_err_t mpu6050_get_y_fine_gain_reg(mpu6050_t const* mpu6050, mpu6050_y_fine_gain_reg_t* reg);
mpu6050_err_t mpu6050_set_y_fine_gain_reg(mpu6050_t const* mpu6050,
                                          mpu6050_y_fine_gain_reg_t const* reg);

mpu6050_err_t mpu6050_get_z_fine_gain_reg(mpu6050_t const* mpu6050, mpu6050_z_fine_gain_reg_t* reg);
mpu6050_err_t mpu6050_set_z_fine_gain_reg(mpu6050_t const* mpu6050,
                                          mpu6050_z_fine_gain_reg_t const* reg);

mpu6050_err_t mpu6050_get_xa_offs_reg(mpu6050_t const* mpu6050, mpu6050_xa_offs_reg_t* reg);
mpu6050_err_t mpu6050_set_xa_offs_reg(mpu6050_t const* mpu6050, mpu6050_xa_offs_reg_t const* reg);

mpu6050_err_t mpu6050_get_ya_offs_reg(mpu6050_t const* mpu6050, mpu6050_ya_offs_reg_t* reg);
mpu6050_err_t mpu6050_set_ya_offs_reg(mpu6050_t const* mpu6050, mpu6050_ya_offs_reg_t const* reg);

mpu6050_err_t mpu6050_get_za_offs_reg(mpu6050_t const* mpu6050, mpu6050_za_offs_reg_t* reg);
mpu6050_err_t mpu6050_set_za_offs_reg(mpu6050_t const* mpu6050, mpu6050_za_offs_reg_t const* reg);

mpu6050_err_t mpu6050_get_self_test_x_reg(mpu6050_t const* mpu6050, mpu6050_self_test_x_reg_t* reg);
mpu6050_err_t mpu6050_set_self_test_x_reg(mpu6050_t const* mpu6050,
                                          mpu6050_self_test_x_reg_t const* reg);

mpu6050_err_t mpu6050_get_self_test_y_reg(mpu6050_t const* mpu6050, mpu6050_self_test_y_reg_t* reg);
mpu6050_err_t mpu6050_set_self_test_y_reg(mpu6050_t const* mpu6050,
                                          mpu6050_self_test_y_reg_t const* reg);

mpu6050_err_t mpu6050_get_self_test_z_reg(mpu6050_t const* mpu6050, mpu6050_self_test_z_reg_t* reg);
mpu6050_err_t mpu6050_set_self_test_z_reg(mpu6050_t const* mpu6050,
                                          mpu6050_self_test_z_reg_t const* reg);

mpu6050_err_t mpu6050_get_self_test_a_reg(mpu6050_t const* mpu6050, mpu6050_self_test_a_reg_t* reg);
mpu6050_err_t mpu6050_set_self_test_a_reg(mpu6050_t const* mpu6050,
                                          mpu6050_self_test_a_reg_t const* reg);

mpu6050_err_t mpu6050_get_xg_offs_usr_reg(mpu6050_t const* mpu6050, mpu6050_xg_offs_usr_reg_t* reg);
mpu6050_err_t mpu6050_set_xg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_xg_offs_usr_reg_t const* reg);

mpu6050_err_t mpu6050_get_yg_offs_usr_reg(mpu6050_t const* mpu6050, mpu6050_yg_offs_usr_reg_t* reg);
mpu6050_err_t mpu6050_set_yg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_yg_offs_usr_reg_t const* reg);

mpu6050_err_t mpu6050_get_zg_offs_usr_reg(mpu6050_t const* mpu6050, mpu6050_zg_offs_usr_reg_t* reg);
mpu6050_err_t mpu6050_set_zg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_zg_offs_usr_reg_t const* reg);

mpu6050_err_t mpu6050_get_smplrt_div_reg(mpu6050_t const* mpu6050, mpu6050_smplrt_div_reg_t* reg);
mpu6050_err_t mpu6050_set_smplrt_div_reg(mpu6050_t const* mpu6050,
                                         mpu6050_smplrt_div_reg_t const* reg);

mpu6050_err_t mpu6050_get_config_reg(mpu6050_t const* mpu6050, mpu6050_config_reg_t* reg);
mpu6050_err_t mpu6050_set_config_reg(mpu6050_t const* mpu6050, mpu6050_config_reg_t const* reg);

mpu6050_err_t mpu6050_get_gyro_config_reg(mpu6050_t const* mpu6050, mpu6050_gyro_config_reg_t* reg);
mpu6050_err_t mpu6050_set_gyro_config_reg(mpu6050_t const* mpu6050,
                                          mpu6050_gyro_config_reg_t const* reg);

mpu6050_err_t mpu6050_get_accel_config_reg(mpu6050_t const* mpu6050,
                                           mpu6050_accel_config_reg_t* reg);
mpu6050_err_t mpu6050_set_accel_config_reg(mpu6050_t const* mpu6050,
                                           mpu6050_accel_config_reg_t const* reg);

mpu6050_err_t mpu6050_get_ff_thr_reg(mpu6050_t const* mpu6050, mpu6050_ff_thr_reg_t* reg);
mpu6050_err_t mpu6050_set_ff_dur_reg(mpu6050_t const* mpu6050, mpu6050_ff_dur_reg_t const* reg);

mpu6050_err_t mpu6050_get_mot_thr_reg(mpu6050_t const* mpu6050, mpu6050_mot_thr_reg_t* reg);
mpu6050_err_t mpu6050_set_mot_dur_reg(mpu6050_t const* mpu6050, mpu6050_mot_dur_reg_t const* reg);

mpu6050_err_t mpu6050_get_zrmot_thr_reg(mpu6050_t const* mpu6050, mpu6050_zrmot_thr_reg_t* reg);
mpu6050_err_t mpu6050_set_zrmot_dur_reg(mpu6050_t const* mpu6050,
                                        mpu6050_zrmot_dur_reg_t const* reg);

mpu6050_err_t mpu6050_get_fifo_en_reg(mpu6050_t const* mpu6050, mpu6050_fifo_en_reg_t* reg);
mpu6050_err_t mpu6050_set_fifo_en_reg(mpu6050_t const* mpu6050, mpu6050_fifo_en_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_mst_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_i2c_mst_ctrl_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_mst_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_i2c_mst_ctrl_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_slv_addr_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_addr_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv_addr_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_addr_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_slv_reg(mpu6050_t const* mpu6050,
                                      mpu6050_slave_num_t slave_num,
                                      mpu6050_i2c_slv_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv_reg(mpu6050_t const* mpu6050,
                                      mpu6050_slave_num_t slave_num,
                                      mpu6050_i2c_slv_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_slv_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_ctrl_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_ctrl_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_slv4_addr_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_addr_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv4_addr_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_addr_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_slv4_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv4_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_slv4_do_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv4_do_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv4_do_reg(mpu6050_t const* mpu6050,
                                          mpu6050_i2c_slv4_do_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_slv4_ctrl_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_ctrl_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv4_ctrl_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_ctrl_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_slv4_di_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv4_di_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv4_di_reg(mpu6050_t const* mpu6050,
                                          mpu6050_i2c_slv4_di_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_mst_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_i2c_mst_status_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_mst_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_i2c_mst_status_reg_t const* reg);

mpu6050_err_t mpu6050_get_int_pin_cfg_reg(mpu6050_t const* mpu6050, mpu6050_int_pin_cfg_reg_t* reg);
mpu6050_err_t mpu6050_set_int_pin_cfg_reg(mpu6050_t const* mpu6050,
                                          mpu6050_int_pin_cfg_reg_t const* reg);

mpu6050_err_t mpu6050_get_int_enable_reg(mpu6050_t const* mpu6050, mpu6050_int_enable_reg_t* reg);
mpu6050_err_t mpu6050_set_int_enable_reg(mpu6050_t const* mpu6050,
                                         mpu6050_int_enable_reg_t const* reg);

mpu6050_err_t mpu6050_get_dmp_int_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_dmp_int_status_reg_t* reg);

mpu6050_err_t mpu6050_get_int_status_reg(mpu6050_t const* mpu6050, mpu6050_int_status_reg_t* reg);

mpu6050_err_t mpu6050_get_accel_xout_reg(mpu6050_t const* mpu6050, mpu6050_accel_xout_reg_t* reg);

mpu6050_err_t mpu6050_get_accel_yout_reg(mpu6050_t const* mpu6050, mpu6050_accel_yout_reg_t* reg);

mpu6050_err_t mpu6050_get_accel_zout_reg(mpu6050_t const* mpu6050, mpu6050_accel_zout_reg_t* reg);

mpu6050_err_t mpu6050_get_accel_out_reg(mpu6050_t const* mpu6050, mpu6050_accel_out_reg_t* reg);

mpu6050_err_t mpu6050_get_temp_out_reg(mpu6050_t const* mpu6050, mpu6050_temp_out_reg_t* reg);

mpu6050_err_t mpu6050_get_gyro_xout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_xout_reg_t* reg);

mpu6050_err_t mpu6050_get_gyro_yout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_yout_reg_t* reg);

mpu6050_err_t mpu6050_get_gyro_zout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_zout_reg_t* reg);

mpu6050_err_t mpu6050_get_gyro_out_reg(mpu6050_t const* mpu6050, mpu6050_gyro_out_reg_t* reg);

mpu6050_err_t mpu6050_get_ext_sens_data_reg(mpu6050_t const* mpu6050,
                                            uint8_t reg_num,
                                            mpu6050_ext_sens_data_reg_t* reg);

mpu6050_err_t mpu6050_get_mot_detect_status_reg(mpu6050_t const* mpu6050,
                                                mpu6050_mot_detect_status_reg_t* reg);

mpu6050_err_t mpu6050_get_i2c_slv_do_reg(mpu6050_t const* mpu6050,
                                         mpu6050_slave_num_t slave_num,
                                         mpu6050_i2c_slv_do_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_slv_do_reg(mpu6050_t const* mpu6050,
                                         mpu6050_slave_num_t slave_num,
                                         mpu6050_i2c_slv_do_reg_t const* reg);

mpu6050_err_t mpu6050_get_i2c_mst_delay_ctrl_reg(mpu6050_t const* mpu6050,
                                                 mpu6050_i2c_mst_delay_ctrl_reg_t* reg);
mpu6050_err_t mpu6050_set_i2c_mst_delay_ctrl_reg(mpu6050_t const* mpu6050,
                                                 mpu6050_i2c_mst_delay_ctrl_reg_t const* reg);

mpu6050_err_t mpu6050_get_signal_path_reset_reg(mpu6050_t const* mpu6050,
                                                mpu6050_signal_path_reset_reg_t* reg);
mpu6050_err_t mpu6050_set_signal_path_reset_reg(mpu6050_t const* mpu6050,
                                                mpu6050_signal_path_reset_reg_t const* reg);

mpu6050_err_t mpu6050_get_mot_detect_ctrl_reg(mpu6050_t const* mpu6050,
                                              mpu6050_mot_detect_ctrl_reg_t* reg);
mpu6050_err_t mpu6050_set_mot_detect_ctrl_reg(mpu6050_t const* mpu6050,
                                              mpu6050_mot_detect_ctrl_reg_t const* reg);

mpu6050_err_t mpu6050_get_user_ctrl_reg(mpu6050_t const* mpu6050, mpu6050_user_ctrl_reg_t* reg);
mpu6050_err_t mpu6050_set_user_ctrl_reg(mpu6050_t const* mpu6050,
                                        mpu6050_user_ctrl_reg_t const* reg);

mpu6050_err_t mpu6050_get_pwr_mgmt_1_reg(mpu6050_t const* mpu6050, mpu6050_pwr_mgmt_1_reg_t* reg);
mpu6050_err_t mpu6050_set_pwr_mgmt_1_reg(mpu6050_t const* mpu6050,
                                         mpu6050_pwr_mgmt_1_reg_t const* reg);

mpu6050_err_t mpu6050_get_pwr_mgmt_2_reg(mpu6050_t const* mpu6050, mpu6050_pwr_mgmt_2_reg_t* reg);
mpu6050_err_t mpu6050_set_pwr_mgmt_2_reg(mpu6050_t const* mpu6050,
                                         mpu6050_pwr_mgmt_2_reg_t const* reg);

mpu6050_err_t mpu6050_get_bank_sel_reg(mpu6050_t const* mpu6050, mpu6050_bank_sel_reg_t* reg);
mpu6050_err_t mpu6050_set_bank_sel_reg(mpu6050_t const* mpu6050, mpu6050_bank_sel_reg_t const* reg);

mpu6050_err_t mpu6050_get_mem_start_addr_reg(mpu6050_t const* mpu6050,
                                             mpu6050_mem_start_addr_reg_t* reg);
mpu6050_err_t mpu6050_set_mem_start_addr_reg(mpu6050_t const* mpu6050,
                                             mpu6050_mem_start_addr_reg_t const* reg);

mpu6050_err_t mpu6050_get_mem_r_w_reg(mpu6050_t const* mpu6050, mpu6050_mem_r_w_reg_t* reg);
mpu6050_err_t mpu6050_set_mem_r_w_reg(mpu6050_t const* mpu6050, mpu6050_mem_r_w_reg_t const* reg);

mpu6050_err_t mpu6050_get_dmp_cfg_1_reg(mpu6050_t const* mpu6050, mpu6050_dmp_cfg_1_reg_t* reg);
mpu6050_err_t mpu6050_set_dmp_cfg_1_reg(mpu6050_t const* mpu6050,
                                        mpu6050_dmp_cfg_1_reg_t const* reg);

mpu6050_err_t mpu6050_get_dmp_cfg_2_reg(mpu6050_t const* mpu6050, mpu6050_dmp_cfg_2_reg_t* reg);
mpu6050_err_t mpu6050_set_dmp_cfg_2_reg(mpu6050_t const* mpu6050,
                                        mpu6050_dmp_cfg_2_reg_t const* reg);

mpu6050_err_t mpu6050_get_fifo_count_reg(mpu6050_t const* mpu6050, mpu6050_fifo_count_reg_t* reg);
mpu6050_err_t mpu6050_set_fifo_count_reg(mpu6050_t const* mpu6050,
                                         mpu6050_fifo_count_reg_t const* reg);

mpu6050_err_t mpu6050_get_fifo_r_w_reg(mpu6050_t const* mpu6050, mpu6050_fifo_r_w_reg_t* reg);
mpu6050_err_t mpu6050_set_fifo_r_w_reg(mpu6050_t const* mpu6050, mpu6050_fifo_r_w_reg_t const* reg);

mpu6050_err_t mpu6050_get_who_am_i_reg(mpu6050_t const* mpu6050, mpu6050_who_am_i_reg_t* reg);

#endif // MPU6050_MPU6050_H