#ifndef MPU6050_MPU6050_REGISTERS_H
#define MPU6050_MPU6050_REGISTERS_H

#include <stdint.h>

typedef struct {
    uint8_t aux_vddio : 1;
    int8_t xg_offs_tc : 6;
    uint8_t otp_bnk_vld : 1;
} mpu6050_xg_offs_tc_reg_t;

typedef struct {
    uint8_t yg_offs_tc : 6;
} mpu6050_yg_offs_tc_reg_t;

typedef struct {
    uint8_t zg_offs_tc : 6;
} mpu6050_zg_offs_tc_reg_t;

typedef struct {
    uint8_t x_fine_gain : 8;
} mpu6050_x_fine_gain_reg_t;

typedef struct {
    uint8_t y_fine_gain : 8;
} mpu6050_y_fine_gain_reg_t;

typedef struct {
    uint8_t z_fine_gain : 8;
} mpu6050_z_fine_gain_reg_t;

typedef struct {
    int16_t xa_offs : 16;
} mpu6050_xa_offs_reg_t;

typedef struct {
    int16_t ya_offs : 16;
} mpu6050_ya_offs_reg_t;

typedef struct {
    int16_t za_offs : 16;
} mpu6050_za_offs_reg_t;

typedef struct {
    uint8_t xa_test : 3;
    uint8_t xg_test : 5;
} mpu6050_self_test_x_reg_t;

typedef struct {
    uint8_t ya_test : 3;
    uint8_t yg_test : 5;
} mpu6050_self_test_y_reg_t;

typedef struct {
    uint8_t za_test : 3;
    uint8_t zg_test : 5;
} mpu6050_self_test_z_reg_t;

typedef struct {
    uint8_t xa_test : 2;
    uint8_t ya_test : 2;
    uint8_t za_test : 2;
} mpu6050_self_test_a_reg_t;

typedef struct {
    int16_t xg_offs_usr : 16;
} mpu6050_xg_offs_usr_reg_t;

typedef struct {
    int16_t yg_offs_usr : 16;
} mpu6050_yg_offs_usr_reg_t;

typedef struct {
    int16_t zg_offs_usr : 16;
} mpu6050_zg_offs_usr_reg_t;

typedef struct {
    uint8_t smplrt_div : 8;
} mpu6050_smplrt_div_reg_t;

typedef struct {
    uint8_t ext_sync_set : 3;
    uint8_t dlpf_cfg : 3;
} mpu6050_config_reg_t;

typedef struct {
    uint8_t xg_st : 1;
    uint8_t yg_st : 1;
    uint8_t zg_st : 1;
    uint8_t fs_sel : 2;
} mpu6050_gyro_config_reg_t;

typedef struct {
    uint8_t xa_st : 1;
    uint8_t ya_st : 1;
    uint8_t za_st : 1;
    uint8_t afs_sel : 2;
    uint8_t accel_hpf : 3;
} mpu6050_accel_config_reg_t;

typedef struct {
    uint8_t ff_thr : 8;
} mpu6050_ff_thr_reg_t;

typedef struct {
    uint8_t ff_dur : 8;
} mpu6050_ff_dur_reg_t;

typedef struct {
    uint8_t mot_thr : 8;
} mpu6050_mot_thr_reg_t;

typedef struct {
    uint8_t mot_dur : 8;
} mpu6050_mot_dur_reg_t;

typedef struct {
    uint8_t zrmot_thr : 8;
} mpu6050_zrmot_thr_reg_t;

typedef struct {
    uint8_t zrmot_dur : 8;
} mpu6050_zrmot_dur_reg_t;

typedef struct {
    uint8_t temp_fifo_en : 1;
    uint8_t xg_fifo_en : 1;
    uint8_t yg_fifo_en : 1;
    uint8_t zg_fifo_en : 1;
    uint8_t accel_fifo_en : 1;
    uint8_t slv2_fifo_en : 1;
    uint8_t slv1_fifo_en : 1;
    uint8_t slv0_fifo_en : 1;
} mpu6050_fifo_en_reg_t;

typedef struct {
    uint8_t mult_mst_en : 1;
    uint8_t wait_for_es : 1;
    uint8_t slv3_fifo_en : 1;
    uint8_t i2c_mst_p_nsr : 1;
    uint8_t i2c_mst_clk : 4;
} mpu6050_i2c_mst_ctrl_reg_t;

typedef struct {
    uint8_t i2c_slv_rw : 1;
    uint8_t i2c_slv_addr : 7;
} mpu6050_i2c_slv_addr_reg_t;

typedef struct {
    uint8_t i2c_slv_reg : 8;
} mpu6050_i2c_slv_reg_t;

typedef struct {
    uint8_t i2c_slv_en : 1;
    uint8_t i2c_slv_byte_sw : 1;
    uint8_t i2c_slv_reg_dis : 1;
    uint8_t i2c_slv_grp : 1;
    uint8_t i2c_slv_len : 4;
} mpu6050_i2c_slv_ctrl_reg_t;

typedef struct {
    uint8_t i2c_slv4_rw : 1;
    uint8_t i2c_slv4_addr : 7;
} mpu6050_i2c_slv4_addr_reg_t;

typedef struct {
    uint8_t i2c_slv4 : 8;
} mpu6050_i2c_slv4_reg_t;

typedef struct {
    uint8_t i2c_slv4_do : 8;
} mpu6050_i2c_slv4_do_reg_t;

typedef struct {
    uint8_t i2c_slv4_en : 1;
    uint8_t i2c_slv4_int_en : 1;
    uint8_t i2c_slv4_reg_dis : 1;
    uint8_t i2c_slv4_mst_dly : 5;
} mpu6050_i2c_slv4_ctrl_reg_t;

typedef struct {
    uint8_t i2c_slv4_di : 8;
} mpu6050_i2c_slv4_di_reg_t;

typedef struct {
    uint8_t pass_through : 1;
    uint8_t i2c_slv4_done : 1;
    uint8_t i2c_lost_arv : 1;
    uint8_t i2c_slv4_nack : 1;
    uint8_t i2c_slv3_nack : 1;
    uint8_t i2c_slv2_nack : 1;
    uint8_t i2c_slv1_nack : 1;
    uint8_t i2c_slv0_nack : 1;
} mpu6050_i2c_mst_status_reg_t;

typedef struct {
    uint8_t int_level : 1;
    uint8_t int_open : 1;
    uint8_t latch_int_en : 1;
    uint8_t int_rd_clear : 1;
    uint8_t fsync_int_level : 1;
    uint8_t fsync_int_en : 1;
    uint8_t i2c_bypass_en : 1;
    uint8_t clkout_en : 1;
} mpu6050_int_pin_cfg_reg_t;

typedef struct {
    uint8_t ff_en : 1;
    uint8_t mot_en : 1;
    uint8_t zmot_en : 1;
    uint8_t fifo_oflow_en : 1;
    uint8_t i2c_mst_int_en : 1;
    uint8_t pll_rdy_int_en : 1;
    uint8_t dmp_int_en : 1;
    uint8_t raw_rdy_en : 1;
} mpu6050_int_enable_reg_t;

typedef struct {
    uint8_t dmp_int_5 : 1;
    uint8_t dmp_int_4 : 1;
    uint8_t dmp_int_3 : 1;
    uint8_t dmp_int_2 : 1;
    uint8_t dmp_int_1 : 1;
    uint8_t dmp_int_0 : 1;
} mpu6050_dmp_int_status_reg_t;

typedef struct {
    uint8_t ff_int : 1;
    uint8_t mot_int : 1;
    uint8_t zmot_int : 1;
    uint8_t fifo_oflow_int : 1;
    uint8_t i2c_mst_int_en : 1;
    uint8_t pll_rdy_int_en : 1;
    uint8_t dmp_int_en : 1;
    uint8_t raw_rdy_en : 1;
} mpu6050_int_status_reg_t;

typedef struct {
    int16_t accel_xout : 16;
} mpu6050_accel_xout_reg_t;

typedef struct {
    int16_t accel_yout : 16;
} mpu6050_accel_yout_reg_t;

typedef struct {
    int16_t accel_zout : 16;
} mpu6050_accel_zout_reg_t;

typedef struct {
    int16_t accel_xout : 16;
    int16_t accel_yout : 16;
    int16_t accel_zout : 16;
} mpu6050_accel_out_reg_t;

typedef struct {
    int16_t temp_out : 16;
} mpu6050_temp_out_reg_t;

typedef struct {
    int16_t gyro_xout : 16;
} mpu6050_gyro_xout_reg_t;

typedef struct {
    int16_t gyro_yout : 16;
} mpu6050_gyro_yout_reg_t;

typedef struct {
    int16_t gyro_zout : 16;
} mpu6050_gyro_zout_reg_t;

typedef struct {
    int16_t gyro_xout : 16;
    int16_t gyro_yout : 16;
    int16_t gyro_zout : 16;
} mpu6050_gyro_out_reg_t;

typedef struct {
    int16_t ext_sens_data : 16;
} mpu6050_ext_sens_data_reg_t;

typedef struct {
    uint8_t mot_xneg : 1;
    uint8_t mot_xpos : 1;
    uint8_t mot_ynef : 1;
    uint8_t mot_ypos : 1;
    uint8_t mot_znef : 1;
    uint8_t mot_zpos : 1;
    uint8_t mot_zrmot : 1;
} mpu6050_mot_detect_status_reg_t;

typedef struct {
    uint8_t i2c_slv_do : 8;
} mpu6050_i2c_slv_do_reg_t;

typedef struct {
    uint8_t delay_es_shadow : 8;
    uint8_t i2c_slv4_dly_en : 1;
    uint8_t i2c_slv3_dly_en : 1;
    uint8_t i2c_slv2_dly_en : 1;
    uint8_t i2c_slv1_dly_en : 1;
    uint8_t i2c_slv0_dly_en : 1;
} mpu6050_i2c_mst_delay_ctrl_reg_t;

typedef struct {
    uint8_t gyro_reset : 1;
    uint8_t accel_reset : 1;
    uint8_t temp_reset : 1;
} mpu6050_signal_path_reset_reg_t;

typedef struct {
    uint8_t accel_on_delay : 2;
    uint8_t ff_count : 2;
    uint8_t mot_count : 2;
} mpu6050_mot_detect_ctrl_reg_t;

typedef struct {
    uint8_t dmp_en : 1;
    uint8_t fifo_en : 1;
    uint8_t i2c_mst_en : 1;
    uint8_t i2c_if_dis : 1;
    uint8_t dmp_reset : 1;
    uint8_t fifo_reset : 1;
    uint8_t i2c_mst_reset : 1;
    uint8_t sig_cond_reset : 1;
} mpu6050_user_ctrl_reg_t;

typedef struct {
    uint8_t device_reset : 1;
    uint8_t sleep : 1;
    uint8_t cycle : 1;
    uint8_t temp_dis : 1;
    uint8_t clk_sel : 3;
} mpu6050_pwr_mgmt_1_reg_t;

typedef struct {
    uint8_t lp_wake_ctrl : 1;
    uint8_t stby_xa : 1;
    uint8_t stby_ya : 1;
    uint8_t stby_za : 1;
    uint8_t stby_xg : 1;
    uint8_t stby_yg : 1;
    uint8_t stby_zg : 1;
} mpu6050_pwr_mgmt_2_reg_t;

typedef struct {
    uint8_t prftch_en : 1;
    uint8_t cfg_user_bank : 1;
    uint8_t mem_sel : 5;
} mpu6050_bank_sel_reg_t;

typedef struct {
    uint8_t start_addr : 8;
} mpu6050_mem_start_addr_reg_t;

typedef struct {
    uint8_t mem_r_w : 8;
} mpu6050_mem_r_w_reg_t;

typedef struct {
    uint8_t dmp_cfg_1 : 8;
} mpu6050_dmp_cfg_1_reg_t;

typedef struct {
    uint8_t dmp_cfg_2 : 8;
} mpu6050_dmp_cfg_2_reg_t;

typedef struct {
    uint16_t fifo_count : 16;
} mpu6050_fifo_count_reg_t;

typedef struct {
    uint8_t fifo_r_w : 8;
} mpu6050_fifo_r_w_reg_t;

typedef struct {
    uint8_t who_am_i : 6;
} mpu6050_who_am_i_reg_t;

#endif // MPU6050_MPU6050_REGISTERS_H