#include "mpu6050.h"
#include "mpu6050_config.h"
#include <assert.h>
#include <string.h>

static mpu6050_err_t mpu6050_bus_initialize(mpu6050_t const* mpu6050)
{
    return mpu6050->interface.bus_initialize
               ? mpu6050->interface.bus_initialize(mpu6050->interface.bus_user)
               : MPU6050_ERR_NULL;
}

static mpu6050_err_t mpu6050_bus_deinitialize(mpu6050_t const* mpu6050)
{
    return mpu6050->interface.bus_deinitialize
               ? mpu6050->interface.bus_deinitialize(mpu6050->interface.bus_user)
               : MPU6050_ERR_NULL;
}

static mpu6050_err_t mpu6050_bus_write_data(mpu6050_t const* mpu6050,
                                            uint8_t address,
                                            uint8_t const* data,
                                            size_t data_size)
{
    return mpu6050->interface.bus_write_data
               ? mpu6050->interface.bus_write_data(mpu6050->interface.bus_user,
                                                   address,
                                                   data,
                                                   data_size)
               : MPU6050_ERR_NULL;
}

static mpu6050_err_t mpu6050_bus_read_data(mpu6050_t const* mpu6050,
                                           uint8_t address,
                                           uint8_t* data,
                                           size_t data_size)
{
    return mpu6050->interface.bus_read_data
               ? mpu6050->interface.bus_read_data(mpu6050->interface.bus_user,
                                                  address,
                                                  data,
                                                  data_size)
               : MPU6050_ERR_NULL;
}

mpu6050_err_t mpu6050_initialize(mpu6050_t* mpu6050,
                                 mpu6050_config_t const* config,
                                 mpu6050_interface_t const* interface)
{
    assert(mpu6050 && config && interface);

    memset(mpu6050, 0, sizeof(*mpu6050));
    memcpy(&mpu6050->config, config, sizeof(*config));
    memcpy(&mpu6050->interface, interface, sizeof(*interface));

    return mpu6050_bus_initialize(mpu6050);
}

mpu6050_err_t mpu6050_deinitialize(mpu6050_t* mpu6050)
{
    assert(mpu6050);

    mpu6050_err_t err = mpu6050_bus_deinitialize(mpu6050);

    memset(mpu6050, 0, sizeof(*mpu6050));

    return err;
}

mpu6050_err_t mpu6050_get_accel_data_x_scaled(mpu6050_t const* mpu6050, float32_t* scaled)
{
    assert(mpu6050 && scaled);

    int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_accel_data_x_raw(mpu6050, &raw);

    *scaled = (float32_t)raw * mpu6050->config.accel_scale;

    return err;
}

mpu6050_err_t mpu6050_get_accel_data_y_scaled(mpu6050_t const* mpu6050, float32_t* scaled)
{
    assert(mpu6050 && scaled);

    int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_accel_data_y_raw(mpu6050, &raw);

    *scaled = (float32_t)raw * mpu6050->config.accel_scale;

    return err;
}

mpu6050_err_t mpu6050_get_accel_data_z_scaled(mpu6050_t const* mpu6050, float32_t* scaled)
{
    assert(mpu6050 && scaled);

    int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_accel_data_z_raw(mpu6050, &raw);

    *scaled = (float32_t)raw * mpu6050->config.accel_scale;

    return err;
}

mpu6050_err_t mpu6050_get_accel_data_scaled(mpu6050_t const* mpu6050, vec3_float32_t* scaled)
{
    assert(mpu6050 && scaled);

    vec3_int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_accel_data_raw(mpu6050, &raw);

    scaled->x = (float32_t)raw.x * mpu6050->config.accel_scale;
    scaled->y = (float32_t)raw.y * mpu6050->config.accel_scale;
    scaled->z = (float32_t)raw.z * mpu6050->config.accel_scale;

    return err;
}

mpu6050_err_t mpu6050_get_temp_data_scaled(mpu6050_t const* mpu6050, float32_t* scaled)
{
    assert(mpu6050 && scaled);

    int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_temp_data_raw(mpu6050, &raw);

    *scaled = (float32_t)raw / 340.0F + 36.53F;

    return err;
}

mpu6050_err_t mpu6050_get_gyro_data_x_scaled(mpu6050_t const* mpu6050, float32_t* scaled)
{
    assert(mpu6050 && scaled);

    int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_gyro_data_x_raw(mpu6050, &raw);

    *scaled = (float32_t)raw * mpu6050->config.gyro_scale;

    return err;
}

mpu6050_err_t mpu6050_get_gyro_data_y_scaled(mpu6050_t const* mpu6050, float32_t* scaled)
{
    assert(mpu6050 && scaled);

    int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_gyro_data_y_raw(mpu6050, &raw);

    *scaled = (float32_t)raw * mpu6050->config.gyro_scale;

    return err;
}

mpu6050_err_t mpu6050_get_gyro_data_z_scaled(mpu6050_t const* mpu6050, float32_t* scaled)
{
    assert(mpu6050 && scaled);

    int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_gyro_data_z_raw(mpu6050, &raw);

    *scaled = (float32_t)raw * mpu6050->config.gyro_scale;

    return err;
}

mpu6050_err_t mpu6050_get_gyro_data_scaled(mpu6050_t const* mpu6050, vec3_float32_t* scaled)
{
    assert(mpu6050 && scaled);

    vec3_int16_t raw = {};

    mpu6050_err_t err = mpu6050_get_gyro_data_raw(mpu6050, &raw);

    scaled->x = (float32_t)raw.x * mpu6050->config.gyro_scale;
    scaled->y = (float32_t)raw.y * mpu6050->config.gyro_scale;
    scaled->z = (float32_t)raw.z * mpu6050->config.gyro_scale;

    return err;
}

mpu6050_err_t mpu6050_get_accel_data_x_raw(mpu6050_t const* mpu6050, int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_accel_xout_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_accel_xout_reg(mpu6050, &reg);

    *raw = reg.accel_xout;

    return err;
}

mpu6050_err_t mpu6050_get_accel_data_y_raw(mpu6050_t const* mpu6050, int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_accel_yout_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_accel_yout_reg(mpu6050, &reg);

    *raw = reg.accel_yout;

    return err;
}

mpu6050_err_t mpu6050_get_accel_data_z_raw(mpu6050_t const* mpu6050, int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_accel_zout_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_accel_zout_reg(mpu6050, &reg);

    *raw = reg.accel_zout;

    return err;
}

mpu6050_err_t mpu6050_get_accel_data_raw(mpu6050_t const* mpu6050, vec3_int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_accel_out_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_accel_out_reg(mpu6050, &reg);

    raw->x = reg.accel_xout;
    raw->y = reg.accel_yout;
    raw->z = reg.accel_zout;

    return err;
}

mpu6050_err_t mpu6050_get_temp_data_raw(mpu6050_t const* mpu6050, int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_temp_out_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_temp_out_reg(mpu6050, &reg);

    *raw = reg.temp_out;

    return err;
}

mpu6050_err_t mpu6050_get_gyro_data_x_raw(mpu6050_t const* mpu6050, int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_gyro_xout_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_gyro_xout_reg(mpu6050, &reg);

    *raw = reg.gyro_xout;

    return err;
}

mpu6050_err_t mpu6050_get_gyro_data_y_raw(mpu6050_t const* mpu6050, int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_gyro_yout_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_gyro_yout_reg(mpu6050, &reg);

    *raw = reg.gyro_yout;

    return err;
}

mpu6050_err_t mpu6050_get_gyro_data_z_raw(mpu6050_t const* mpu6050, int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_gyro_zout_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_gyro_zout_reg(mpu6050, &reg);

    *raw = reg.gyro_zout;

    return err;
}

mpu6050_err_t mpu6050_get_gyro_data_raw(mpu6050_t const* mpu6050, vec3_int16_t* raw)
{
    assert(mpu6050 && raw);

    mpu6050_gyro_out_reg_t reg = {};

    mpu6050_err_t err = mpu6050_get_gyro_out_reg(mpu6050, &reg);

    raw->x = reg.gyro_xout;
    raw->y = reg.gyro_yout;
    raw->z = reg.gyro_zout;

    return err;
}

mpu6050_err_t mpu6050_ext_slv_read(mpu6050_t const* mpu6050,
                                   mpu6050_slave_num_t slave_num,
                                   uint8_t address,
                                   uint8_t* data,
                                   uint8_t data_size)
{
    assert(mpu6050 && data);

    mpu6050_i2c_slv_addr_reg_t slv_addr_reg = {};
    mpu6050_err_t err = mpu6050_get_i2c_slv_addr_reg(mpu6050, slave_num, &slv_addr_reg);
    slv_addr_reg.i2c_slv_rw = true;
    err |= mpu6050_set_i2c_slv_addr_reg(mpu6050, slave_num, &slv_addr_reg);

    mpu6050_i2c_slv_reg_t slv_reg = {};
    err |= mpu6050_get_i2c_slv_reg(mpu6050, slave_num, &slv_reg);
    slv_reg.i2c_slv_reg = address;
    err |= mpu6050_set_i2c_slv_reg(mpu6050, slave_num, &slv_reg);

    mpu6050_i2c_slv_ctrl_reg_t slv_ctrl_reg = {};
    err |= mpu6050_get_i2c_slv_ctrl_reg(mpu6050, slave_num, &slv_ctrl_reg);
    slv_ctrl_reg.i2c_slv_len = data_size & 0x0FU;
    err |= mpu6050_set_i2c_slv_ctrl_reg(mpu6050, slave_num, &slv_ctrl_reg);

    err |= mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_EXT_SENS_DATA_00, data, data_size);

    return err;
}

mpu6050_err_t mpu6050_ext_slv_write(mpu6050_t const* mpu6050,
                                    mpu6050_slave_num_t slave_num,
                                    uint8_t address,
                                    uint8_t const* data,
                                    uint8_t data_size)
{
    assert(mpu6050 && data);

    mpu6050_i2c_slv_addr_reg_t slv_addr_reg = {};
    mpu6050_err_t err = mpu6050_get_i2c_slv_addr_reg(mpu6050, slave_num, &slv_addr_reg);
    slv_addr_reg.i2c_slv_rw = false;
    err |= mpu6050_set_i2c_slv_addr_reg(mpu6050, slave_num, &slv_addr_reg);

    mpu6050_i2c_slv_reg_t slv_reg = {};
    err |= mpu6050_get_i2c_slv_reg(mpu6050, slave_num, &slv_reg);
    slv_reg.i2c_slv_reg = address;
    err |= mpu6050_set_i2c_slv_reg(mpu6050, slave_num, &slv_reg);

    mpu6050_i2c_slv_ctrl_reg_t slv_ctrl_reg = {};
    err |= mpu6050_get_i2c_slv_ctrl_reg(mpu6050, slave_num, &slv_ctrl_reg);
    slv_ctrl_reg.i2c_slv_len = data_size & 0x0FU;
    err |= mpu6050_set_i2c_slv_ctrl_reg(mpu6050, slave_num, &slv_ctrl_reg);

    err |= mpu6050_bus_write_data(mpu6050, slave_num, data, data_size);

    return err;
}

mpu6050_err_t mpu6050_fifo_read(mpu6050_t const* mpu6050, uint8_t* data, uint8_t data_size)
{
    assert(mpu6050 && data);

    return mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_FIFO_R_W, data, data_size);
}

mpu6050_err_t mpu6050_fifo_write(mpu6050_t const* mpu6050, uint8_t const* data, uint8_t data_size)
{
    assert(mpu6050 && data);

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_FIFO_R_W, data, data_size);
}

mpu6050_err_t mpu6050_get_xg_offs_tc_reg(mpu6050_t const* mpu6050, mpu6050_xg_offs_tc_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_XG_OFFS_TC, &data, sizeof(data));

    reg->aux_vddio = (data >> 7U) & 0x01U;
    reg->xg_offs_tc = (int8_t)((data >> 1) & 0x3F);
    reg->otp_bnk_vld = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_xg_offs_tc_reg(mpu6050_t const* mpu6050,
                                         mpu6050_xg_offs_tc_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->aux_vddio & 0x01U) << 7U;
    data |= (uint8_t)((reg->xg_offs_tc & 0x3F) << 1);
    data |= reg->otp_bnk_vld & 0x01U;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_XG_OFFS_TC, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_yg_offs_tc_reg(mpu6050_t const* mpu6050, mpu6050_yg_offs_tc_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_YG_OFFS_TC, &data, sizeof(data));

    reg->yg_offs_tc = (int8_t)((data >> 1) & 0x3F);

    return err;
}

mpu6050_err_t mpu6050_set_yg_offs_tc_reg(mpu6050_t const* mpu6050,
                                         mpu6050_yg_offs_tc_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_YG_OFFS_TC, &data, sizeof(data));

    data &= ~(0x3F << 1);

    data |= (uint8_t)((reg->yg_offs_tc & 0x3F) << 1);

    err |= mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_YG_OFFS_TC, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_zg_offs_tc_reg(mpu6050_t const* mpu6050, mpu6050_zg_offs_tc_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ZG_OFFS_TC, &data, sizeof(data));

    reg->zg_offs_tc = (int8_t)((data >> 1) & 0x3F);

    return err;
}

mpu6050_err_t mpu6050_set_zg_offs_tc_reg(mpu6050_t const* mpu6050,
                                         mpu6050_zg_offs_tc_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ZG_OFFS_TC, &data, sizeof(data));

    data &= ~(0x3F << 1);

    data |= (uint8_t)((reg->zg_offs_tc & 0x3F) << 1);

    err |= mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_ZG_OFFS_TC, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_x_fine_gain_reg(mpu6050_t const* mpu6050, mpu6050_x_fine_gain_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_X_FINE_GAIN, &data, sizeof(data));

    reg->x_fine_gain = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_x_fine_gain_reg(mpu6050_t const* mpu6050,
                                          mpu6050_x_fine_gain_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->x_fine_gain & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_X_FINE_GAIN, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_y_fine_gain_reg(mpu6050_t const* mpu6050, mpu6050_y_fine_gain_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_Y_FINE_GAIN, &data, sizeof(data));

    reg->y_fine_gain = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_y_fine_gain_reg(mpu6050_t const* mpu6050,
                                          mpu6050_y_fine_gain_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->y_fine_gain & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_Y_FINE_GAIN, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_z_fine_gain_reg(mpu6050_t const* mpu6050, mpu6050_z_fine_gain_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_Z_FINE_GAIN, &data, sizeof(data));

    reg->z_fine_gain = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_z_fine_gain_reg(mpu6050_t const* mpu6050,
                                          mpu6050_z_fine_gain_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->z_fine_gain & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_Z_FINE_GAIN, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_xa_offs_reg(mpu6050_t const* mpu6050, mpu6050_xa_offs_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_XA_OFFS_H, data, sizeof(data));

    reg->xa_offs = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_set_xa_offs_reg(mpu6050_t const* mpu6050, mpu6050_xa_offs_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    data[0] = (uint8_t)((reg->xa_offs >> 8) & 0xFF);
    data[1] = (uint8_t)(reg->xa_offs & 0xFF);

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_XA_OFFS_H, data, sizeof(data));
}

mpu6050_err_t mpu6050_get_ya_offs_reg(mpu6050_t const* mpu6050, mpu6050_ya_offs_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_YA_OFFS_H, data, sizeof(data));

    reg->ya_offs = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_set_ya_offs_reg(mpu6050_t const* mpu6050, mpu6050_ya_offs_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    data[0] = (uint8_t)((reg->ya_offs >> 8) & 0xFF);
    data[1] = (uint8_t)(reg->ya_offs & 0xFF);

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_YA_OFFS_H, data, sizeof(data));
}

mpu6050_err_t mpu6050_get_za_offs_reg(mpu6050_t const* mpu6050, mpu6050_za_offs_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ZA_OFFS_H, data, sizeof(data));

    reg->za_offs = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_set_za_offs_reg(mpu6050_t const* mpu6050, mpu6050_za_offs_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    data[0] = (uint8_t)((reg->za_offs >> 8) & 0xFF);
    data[1] = (uint8_t)(reg->za_offs & 0xFF);

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_ZA_OFFS_H, data, sizeof(data));
}

mpu6050_err_t mpu6050_get_self_test_x_reg(mpu6050_t const* mpu6050, mpu6050_self_test_x_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SELF_TEST_X, &data, sizeof(data));

    reg->xa_test = (data >> 5U) & 0x7U;
    reg->xg_test = data & 0x1FU;

    return err;
}

mpu6050_err_t mpu6050_set_self_test_x_reg(mpu6050_t const* mpu6050,
                                          mpu6050_self_test_x_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->xa_test & 0x03U) << 5U;
    data |= reg->xg_test & 0x1FU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_SELF_TEST_X, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_self_test_y_reg(mpu6050_t const* mpu6050, mpu6050_self_test_y_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SELF_TEST_Y, &data, sizeof(data));

    reg->ya_test = (data >> 5U) & 0x7U;
    reg->yg_test = data & 0x1FU;

    return err;
}

mpu6050_err_t mpu6050_set_self_test_y_reg(mpu6050_t const* mpu6050,
                                          mpu6050_self_test_y_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->ya_test & 0x03U) << 5U;
    data |= reg->yg_test & 0x1FU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_SELF_TEST_Y, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_self_test_z_reg(mpu6050_t const* mpu6050, mpu6050_self_test_z_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SELF_TEST_Z, &data, sizeof(data));

    reg->za_test = (data >> 5U) & 0x7U;
    reg->zg_test = data & 0x1FU;

    return err;
}

mpu6050_err_t mpu6050_set_self_test_z_reg(mpu6050_t const* mpu6050,
                                          mpu6050_self_test_z_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->za_test & 0x03U) << 5U;
    data |= reg->zg_test & 0x1FU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_SELF_TEST_Z, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_self_test_a_reg(mpu6050_t const* mpu6050, mpu6050_self_test_a_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = 0U;

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SELF_TEST_A, &data, sizeof(data));

    reg->xa_test = (data >> 4U) & 0x3U;
    reg->ya_test = (data >> 2U) & 0x03U;
    reg->za_test = data & 0x03U;

    return err;
}

mpu6050_err_t mpu6050_set_self_test_a_reg(mpu6050_t const* mpu6050,
                                          mpu6050_self_test_a_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->xa_test & 0x03U) << 4U;
    data |= (reg->ya_test & 0x03U) << 2U;
    data |= reg->xa_test & 0x03U;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_SELF_TEST_X, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_xg_offs_usr_reg(mpu6050_t const* mpu6050, mpu6050_xg_offs_usr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_XG_OFFS_USR_H, data, sizeof(data));

    reg->xg_offs_usr = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_set_xg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_xg_offs_usr_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    data[0] = (uint8_t)((reg->xg_offs_usr >> 8) & 0xFF);
    data[1] = (uint8_t)(reg->xg_offs_usr & 0xFF);

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_XG_OFFS_USR_H, data, sizeof(data));
}

mpu6050_err_t mpu6050_get_yg_offs_usr_reg(mpu6050_t const* mpu6050, mpu6050_yg_offs_usr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_YG_OFFS_USR_H, data, sizeof(data));

    reg->yg_offs_usr = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_set_yg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_yg_offs_usr_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    data[0] = (uint8_t)((reg->yg_offs_usr >> 8) & 0xFF);
    data[1] = (uint8_t)(reg->yg_offs_usr & 0xFF);

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_YG_OFFS_USR_H, data, sizeof(data));
}

mpu6050_err_t mpu6050_get_zg_offs_usr_reg(mpu6050_t const* mpu6050, mpu6050_zg_offs_usr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ZG_OFFS_USR_H, data, sizeof(data));

    reg->zg_offs_usr = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_set_zg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_zg_offs_usr_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    data[0] = (uint8_t)((reg->zg_offs_usr >> 8) & 0xFF);
    data[1] = (uint8_t)(reg->zg_offs_usr & 0xFF);

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_ZG_OFFS_USR_H, data, sizeof(data));
}

mpu6050_err_t mpu6050_get_smplrt_div_reg(mpu6050_t const* mpu6050, mpu6050_smplrt_div_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SMPLRT_DIV, &data, sizeof(data));

    reg->smplrt_div = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_smplrt_div_reg(mpu6050_t const* mpu6050,
                                         mpu6050_smplrt_div_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SMPLRT_DIV, &data, sizeof(data));

    reg->smplrt_div = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_get_config_reg(mpu6050_t const* mpu6050, mpu6050_config_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_CONFIG, &data, sizeof(data));

    reg->ext_sync_set = (data >> 3U) & 0x07U;
    reg->dlpf_cfg = data & 0x07U;

    return err;
}

mpu6050_err_t mpu6050_set_config_reg(mpu6050_t const* mpu6050, mpu6050_config_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_CONFIG, &data, sizeof(data));

    data &= ~((0x07U << 3U) | 0x07U);

    err |= mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_CONFIG, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_gyro_config_reg(mpu6050_t const* mpu6050, mpu6050_gyro_config_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_GYRO_CONFIG, &data, sizeof(data));

    reg->xg_st = (data >> 7U) & 0x01U;
    reg->yg_st = (data >> 6U) & 0x01U;
    reg->zg_st = (data >> 5U) & 0x01U;
    reg->fs_sel = (data >> 3U) & 0x03U;

    return err;
}

mpu6050_err_t mpu6050_set_gyro_config_reg(mpu6050_t const* mpu6050,
                                          mpu6050_gyro_config_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_GYRO_CONFIG, &data, sizeof(data));

    data &= ~((0x01U << 7U) | (0x01U << 6U) | (0x01U << 5U) | (0x03U << 3U));

    data |= (reg->xg_st & 0x01U) << 7U;
    data |= (reg->yg_st & 0x01U) << 6U;
    data |= (reg->zg_st & 0x01U) << 5U;
    data |= (reg->fs_sel & 0x03U) << 3U;

    err |= mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_GYRO_CONFIG, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_accel_config_reg(mpu6050_t const* mpu6050,
                                           mpu6050_accel_config_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ACCEL_CONFIG, &data, sizeof(data));

    reg->xa_st = (data >> 7U) & 0x01U;
    reg->ya_st = (data >> 6U) & 0x01U;
    reg->za_st = (data >> 5U) & 0x01U;
    reg->afs_sel = (data >> 3U) & 0x03U;
    reg->accel_hpf = data & 0x07U;

    return err;
}

mpu6050_err_t mpu6050_set_accel_config_reg(mpu6050_t const* mpu6050,
                                           mpu6050_accel_config_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->xa_st & 0x01U) << 7U;
    data |= (reg->ya_st & 0x01U) << 6U;
    data |= (reg->za_st & 0x01U) << 5U;
    data |= (reg->afs_sel & 0x03U) << 3U;
    data |= reg->accel_hpf & 0x07U;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_ACCEL_CONFIG, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_ff_thr_reg(mpu6050_t const* mpu6050, mpu6050_ff_thr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_FF_THR, &data, sizeof(data));

    reg->ff_thr = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_ff_dur_reg(mpu6050_t const* mpu6050, mpu6050_ff_dur_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->ff_dur & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_FF_DUR, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_mot_thr_reg(mpu6050_t const* mpu6050, mpu6050_mot_thr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_MOT_THR, &data, sizeof(data));

    reg->mot_thr = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_mot_dur_reg(mpu6050_t const* mpu6050, mpu6050_mot_dur_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->mot_dur & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_MOT_DUR, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_zrmot_thr_reg(mpu6050_t const* mpu6050, mpu6050_zrmot_thr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ZRMOT_THR, &data, sizeof(data));

    reg->zrmot_thr = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_zrmot_dur_reg(mpu6050_t const* mpu6050,
                                        mpu6050_zrmot_dur_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_ZRMOT_DUR, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_fifo_en_reg(mpu6050_t const* mpu6050, mpu6050_fifo_en_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_FIFO_EN, &data, sizeof(data));

    reg->temp_fifo_en = (data >> 7U) & 0x01U;
    reg->xg_fifo_en = (data >> 6U) & 0x01U;
    reg->yg_fifo_en = (data >> 5U) & 0x01U;
    reg->zg_fifo_en = (data >> 4U) & 0x01U;
    reg->accel_fifo_en = (data >> 3U) & 0x01U;
    reg->slv2_fifo_en = (data >> 2U) & 0x01U;
    reg->slv1_fifo_en = (data >> 1U) & 0x01U;
    reg->slv0_fifo_en = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_fifo_en_reg(mpu6050_t const* mpu6050, mpu6050_fifo_en_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->temp_fifo_en & 0x01U) << 7U;
    data |= (reg->xg_fifo_en & 0x01U) << 6U;
    data |= (reg->yg_fifo_en & 0x01U) << 5U;
    data |= (reg->zg_fifo_en & 0x01U) << 4U;
    data |= (reg->accel_fifo_en & 0x01U) << 3U;
    data |= (reg->slv2_fifo_en & 0x01U) << 2U;
    data |= (reg->slv1_fifo_en & 0x01U) << 1U;
    data |= reg->slv0_fifo_en & 0x01U;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_FIFO_EN, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_mst_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_i2c_mst_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_MST_CTRL, &data, sizeof(data));

    reg->mult_mst_en = (data >> 7U) & 0x01U;
    reg->wait_for_es = (data >> 6U) & 0x01U;
    reg->slv3_fifo_en = (data >> 5U) & 0x01U;
    reg->i2c_mst_p_nsr = (data >> 4U) & 0x01U;
    reg->i2c_mst_clk = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_mst_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_i2c_mst_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->mult_mst_en & 0x01U) << 7U;
    data |= (reg->wait_for_es & 0x01U) << 6U;
    data |= (reg->slv3_fifo_en & 0x01U) << 5U;
    data |= (reg->i2c_mst_p_nsr & 0x01U) << 4U;
    data |= reg->i2c_mst_clk & 0x0FU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_I2C_MST_CTRL, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_slv_addr_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_addr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV0_ADDR, &data, sizeof(data));

    reg->i2c_slv_rw = (data >> 7U) & 0x01U;
    reg->i2c_slv_addr = data & 0x7FU;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv_addr_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_addr_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->i2c_slv_rw & 0x01U) << 7U;
    data |= reg->i2c_slv_addr & 0x7FU;

    return mpu6050_bus_write_data(mpu6050,
                                  MPU6050_REG_ADDRESS_I2C_SLV0_ADDR + slave_num,
                                  &data,
                                  sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_slv_reg(mpu6050_t const* mpu6050,
                                      mpu6050_slave_num_t slave_num,
                                      mpu6050_i2c_slv_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err = mpu6050_bus_read_data(mpu6050,
                                              MPU6050_REG_ADDRESS_I2C_SLV0_REG + slave_num,
                                              &data,
                                              sizeof(data));

    reg->i2c_slv_reg = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv_reg(mpu6050_t const* mpu6050,
                                      mpu6050_slave_num_t slave_num,
                                      mpu6050_i2c_slv_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->i2c_slv_reg & 0xFFU;

    return mpu6050_bus_write_data(mpu6050,
                                  MPU6050_REG_ADDRESS_I2C_SLV0_REG + slave_num,
                                  &data,
                                  sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_slv_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err = mpu6050_bus_read_data(mpu6050,
                                              MPU6050_REG_ADDRESS_I2C_SLV0_CTRL + slave_num,
                                              &data,
                                              sizeof(data));

    reg->i2c_slv_en = (data >> 7U) & 0x01U;
    reg->i2c_slv_byte_sw = (data >> 6U) & 0x01U;
    reg->i2c_slv_reg_dis = (data >> 5U) & 0x01U;
    reg->i2c_slv_grp = (data >> 4U) & 0x01U;
    reg->i2c_slv_len = data & 0x0FU;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->i2c_slv_en & 0x01U) << 7U;
    data |= (reg->i2c_slv_byte_sw & 0x01U) << 6U;
    data |= (reg->i2c_slv_reg_dis & 0x01U) << 5U;
    data |= (reg->i2c_slv_grp & 0x01U) << 4U;
    data |= reg->i2c_slv_len & 0x0FU;

    return mpu6050_bus_write_data(mpu6050,
                                  MPU6050_REG_ADDRESS_I2C_SLV0_CTRL + slave_num,
                                  &data,
                                  sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_slv4_addr_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_addr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_ADDR, &data, sizeof(data));

    reg->i2c_slv4_rw = (data >> 7U) & 0x01U;
    reg->i2c_slv4_addr = data & 0x7FU;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv4_addr_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_addr_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->i2c_slv4_rw & 0x01U) << 7U;
    data |= reg->i2c_slv4_addr & 0x7FU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_ADDR, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_slv4_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv4_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_REG, &data, sizeof(data));

    reg->i2c_slv4_reg = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv4_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv4_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->i2c_slv_reg & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_REG, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_slv4_do_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv4_do_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_DO, &data, sizeof(data));

    reg->i2c_slv4_do = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv4_do_reg(mpu6050_t const* mpu6050,
                                          mpu6050_i2c_slv4_do_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->i2c_slv4_do & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_DO, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_slv4_ctrl_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_CTRL, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv4_ctrl_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->i2c_slv4_en & 0x01U) << 7U;
    data |= (reg->i2c_slv4_int_en & 0x01U) << 6U;
    data |= (reg->i2c_slv4_reg_dis & 0x01U) << 5U;
    data |= reg->i2c_slv4_mst_dly & 0x1FU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_CTRL, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_slv4_di_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv4_di_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->i2c_slv4_di & 0xFFU;

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_DI, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv4_di_reg(mpu6050_t const* mpu6050,
                                          mpu6050_i2c_slv4_di_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->i2c_slv4_di & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_I2C_SLV4_DI, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_mst_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_i2c_mst_status_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_MST_STATUS, &data, sizeof(data));

    reg->pass_through = (data >> 7U) & 0x01U;
    reg->i2c_slv4_done = (data >> 6U) & 0x01U;
    reg->i2c_lost_arb = (data >> 5U) & 0x01U;
    reg->i2c_slv4_nack = (data >> 4U) & 0x01U;
    reg->i2c_slv3_nack = (data >> 3U) & 0x01U;
    reg->i2c_slv2_nack = (data >> 2U) & 0x01U;
    reg->i2c_slv1_nack = (data >> 1U) & 0x01U;
    reg->i2c_slv0_nack = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_mst_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_i2c_mst_status_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->pass_through & 0x01U) << 7U;
    data |= (reg->i2c_slv4_done & 0x01U) << 6U;
    data |= (reg->i2c_lost_arb & 0x01U) << 5U;
    data |= (reg->i2c_slv4_nack & 0x01U) << 4U;
    data |= (reg->i2c_slv3_nack & 0x01U) << 3U;
    data |= (reg->i2c_slv2_nack & 0x01U) << 2U;
    data |= (reg->i2c_slv1_nack & 0x01U) << 1U;
    data |= reg->i2c_slv0_nack & 0x01U;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_I2C_MST_STATUS, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_int_pin_cfg_reg(mpu6050_t const* mpu6050, mpu6050_int_pin_cfg_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_INT_PIN_CFG, &data, sizeof(data));

    reg->int_level = (data >> 7U) & 0x01U;
    reg->int_open = (data >> 6U) & 0x01U;
    reg->latch_int_en = (data >> 5U) & 0x01U;
    reg->int_rd_clear = (data >> 4U) & 0x01U;
    reg->fsync_int_level = (data >> 3U) & 0x01U;
    reg->fsync_int_en = (data >> 2U) & 0x01U;
    reg->i2c_bypass_en = (data >> 1U) & 0x01U;
    reg->clkout_en = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_int_pin_cfg_reg(mpu6050_t const* mpu6050,
                                          mpu6050_int_pin_cfg_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->int_level & 0x01U) << 7U;
    data |= (reg->int_open & 0x01U) << 6U;
    data |= (reg->latch_int_en & 0x01U) << 5U;
    data |= (reg->int_rd_clear & 0x01U) << 4U;
    data |= (reg->fsync_int_level & 0x01U) << 3U;
    data |= (reg->fsync_int_en & 0x01U) << 2U;
    data |= (reg->i2c_bypass_en & 0x01U) << 1U;
    data |= reg->clkout_en & 0x01U;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_INT_PIN_CFG, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_int_enable_reg(mpu6050_t const* mpu6050, mpu6050_int_enable_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SMPLRT_DIV, &data, sizeof(data));

    reg->ff_en = (data >> 7U) & 0x01U;
    reg->mot_en = (data >> 6U) & 0x01U;
    reg->zmot_en = (data >> 5U) & 0x01U;
    reg->fifo_oflow_en = (data >> 4U) & 0x01U;
    reg->i2c_mst_int_en = (data >> 3U) & 0x01U;
    reg->pll_rdy_int_en = (data >> 2U) & 0x01U;
    reg->dmp_int_en = (data >> 1U) & 0x01U;
    reg->raw_rdy_en = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_int_enable_reg(mpu6050_t const* mpu6050,
                                         mpu6050_int_enable_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->ff_en & 0x01U) << 7U;
    data |= (reg->mot_en & 0x01U) << 6U;
    data |= (reg->zmot_en & 0x01U) << 5U;
    data |= (reg->fifo_oflow_en & 0x01U) << 4U;
    data |= (reg->i2c_mst_int_en & 0x01U) << 3U;
    data |= (reg->pll_rdy_int_en & 0x01U) << 2U;
    data |= (reg->dmp_int_en & 0x01U) << 1U;
    data |= reg->raw_rdy_en & 0x01U;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_INT_ENABLE, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_dmp_int_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_dmp_int_status_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_DMP_INT_STATUS, &data, sizeof(data));

    reg->dmp_int_5 = (data >> 5U) & 0x01U;
    reg->dmp_int_4 = (data >> 4U) & 0x01U;
    reg->dmp_int_3 = (data >> 3U) & 0x01U;
    reg->dmp_int_2 = (data >> 2U) & 0x01U;
    reg->dmp_int_1 = (data >> 1U) & 0x01U;
    reg->dmp_int_0 = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_get_int_status_reg(mpu6050_t const* mpu6050, mpu6050_int_status_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_INT_STATUS, &data, sizeof(data));

    reg->ff_int = (data >> 7U) & 0x01U;
    reg->mot_int = (data >> 6U) & 0x01U;
    reg->zmot_int = (data >> 5U) & 0x01U;
    reg->fifo_oflow_int = (data >> 4U) & 0x01U;
    reg->i2c_mst_int = (data >> 3U) & 0x01U;
    reg->pll_rdy_int = (data >> 2U) & 0x01U;
    reg->dmp_int = (data >> 1U) & 0x01U;
    reg->raw_rdy_int = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_get_accel_xout_reg(mpu6050_t const* mpu6050, mpu6050_accel_xout_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ACCEL_XOUT_H, data, sizeof(data));

    reg->accel_xout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_accel_yout_reg(mpu6050_t const* mpu6050, mpu6050_accel_yout_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ACCEL_YOUT_H, data, sizeof(data));

    reg->accel_yout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_accel_zout_reg(mpu6050_t const* mpu6050, mpu6050_accel_zout_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ACCEL_ZOUT_H, data, sizeof(data));

    reg->accel_zout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_accel_out_reg(mpu6050_t const* mpu6050, mpu6050_accel_out_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[6] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_ACCEL_XOUT_H, data, sizeof(data));

    reg->accel_xout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));
    reg->accel_yout = (int16_t)(((data[2] & 0xFF) << 8) | (data[3] & 0xFF));
    reg->accel_zout = (int16_t)(((data[4] & 0xFF) << 8) | (data[5] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_temp_out_reg(mpu6050_t const* mpu6050, mpu6050_temp_out_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_TEMP_OUT_H, data, sizeof(data));

    reg->temp_out = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_gyro_xout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_xout_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_GYRO_XOUT_H, data, sizeof(data));

    reg->gyro_xout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_gyro_yout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_yout_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_GYRO_YOUT_H, data, sizeof(data));

    reg->gyro_yout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_gyro_zout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_zout_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_GYRO_ZOUT_H, data, sizeof(data));

    reg->gyro_zout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_gyro_out_reg(mpu6050_t const* mpu6050, mpu6050_gyro_out_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[6] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_GYRO_XOUT_H, data, sizeof(data));

    reg->gyro_xout = (int16_t)(((data[0] & 0xFF) << 8) | (data[1] & 0xFF));
    reg->gyro_yout = (int16_t)(((data[2] & 0xFF) << 8) | (data[3] & 0xFF));
    reg->gyro_zout = (int16_t)(((data[4] & 0xFF) << 8) | (data[5] & 0xFF));

    return err;
}

mpu6050_err_t mpu6050_get_ext_sens_data_reg(mpu6050_t const* mpu6050,
                                            uint8_t reg_num,
                                            mpu6050_ext_sens_data_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err = mpu6050_bus_read_data(mpu6050,
                                              MPU6050_REG_ADDRESS_EXT_SENS_DATA_00 + reg_num,
                                              &data,
                                              sizeof(data));

    reg->ext_sens_data = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_get_mot_detect_status_reg(mpu6050_t const* mpu6050,
                                                mpu6050_mot_detect_status_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_MOT_DETECT_STATUS, &data, sizeof(data));

    reg->mot_xneg = (data >> 7U) & 0x01U;
    reg->mot_xpos = (data >> 6U) & 0x01U;
    reg->mot_yneg = (data >> 5U) & 0x01U;
    reg->mot_ypos = (data >> 4U) & 0x01U;
    reg->mot_zneg = (data >> 3U) & 0x01U;
    reg->mot_zpos = (data >> 2U) & 0x01U;
    reg->mot_zrmot = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_get_i2c_slv_do_reg(mpu6050_t const* mpu6050,
                                         mpu6050_slave_num_t slave_num,
                                         mpu6050_i2c_slv_do_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err = mpu6050_bus_read_data(mpu6050,
                                              MPU6050_REG_ADDRESS_I2C_SLV0_DO + slave_num,
                                              &data,
                                              sizeof(data));

    reg->i2c_slv_do = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_slv_do_reg(mpu6050_t const* mpu6050,
                                         mpu6050_slave_num_t slave_num,
                                         mpu6050_i2c_slv_do_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->i2c_slv_do & 0xFFU;

    return mpu6050_bus_write_data(mpu6050,
                                  MPU6050_REG_ADDRESS_I2C_SLV0_DO + slave_num,
                                  &data,
                                  sizeof(data));
}

mpu6050_err_t mpu6050_get_i2c_mst_delay_ctrl_reg(mpu6050_t const* mpu6050,
                                                 mpu6050_i2c_mst_delay_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_MST_DELAY_CTRL, &data, sizeof(data));

    reg->delay_es_shadow = (data >> 7U) & 0x01U;
    reg->i2c_slv4_dly_en = (data >> 4U) & 0x01U;
    reg->i2c_slv3_dly_en = (data >> 3U) & 0x01U;
    reg->i2c_slv2_dly_en = (data >> 2U) & 0x01U;
    reg->i2c_slv1_dly_en = (data >> 1U) & 0x01U;
    reg->i2c_slv0_dly_en = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_i2c_mst_delay_ctrl_reg(mpu6050_t const* mpu6050,
                                                 mpu6050_i2c_mst_delay_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_I2C_MST_DELAY_CTRL, &data, sizeof(data));

    data &=
        ~((0x01U << 7U) | (0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) | (0x01U << 1U) | 0x01U);

    data |= (reg->delay_es_shadow & 0x01U) << 7U;
    data |= (reg->i2c_slv4_dly_en & 0x01U) << 4U;
    data |= (reg->i2c_slv3_dly_en & 0x01U) << 3U;
    data |= (reg->i2c_slv2_dly_en & 0x01U) << 2U;
    data |= (reg->i2c_slv1_dly_en & 0x01U) << 1U;
    data |= reg->i2c_slv0_dly_en & 0x01U;

    err |= mpu6050_bus_write_data(mpu6050,
                                  MPU6050_REG_ADDRESS_I2C_MST_DELAY_CTRL,
                                  &data,
                                  sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_signal_path_reset_reg(mpu6050_t const* mpu6050,
                                                mpu6050_signal_path_reset_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SIGNAL_PATH_RESET, &data, sizeof(data));

    reg->gyro_reset = (data >> 2U) & 0x01U;
    reg->accel_reset = (data >> 1U) & 0x01U;
    reg->temp_reset = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_signal_path_reset_reg(mpu6050_t const* mpu6050,
                                                mpu6050_signal_path_reset_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_SIGNAL_PATH_RESET, &data, sizeof(data));

    data &= ~((0x01U << 2U) | (0x01U << 1U) | 0x01U);

    data |= (reg->gyro_reset & 0x01U) << 2U;
    data |= (reg->accel_reset & 0x01U) << 1U;
    data |= reg->temp_reset & 0x01U;

    err |=
        mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_SIGNAL_PATH_RESET, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_mot_detect_ctrl_reg(mpu6050_t const* mpu6050,
                                              mpu6050_mot_detect_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_MOT_DETECT_CTRL, &data, sizeof(data));

    reg->accel_on_delay = (data >> 4U) & 0x03U;
    reg->ff_count = (data >> 2U) & 0x03U;
    reg->ff_count = data & 0x03U;

    return err;
}

mpu6050_err_t mpu6050_set_mot_detect_ctrl_reg(mpu6050_t const* mpu6050,
                                              mpu6050_mot_detect_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_MOT_DETECT_CTRL, &data, sizeof(data));

    data &= ~((0x03U << 4U) | (0x03U << 2U) | 0x03U);

    err |=
        mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_MOT_DETECT_CTRL, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_user_ctrl_reg(mpu6050_t const* mpu6050, mpu6050_user_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_USER_CTRL, &data, sizeof(data));

    reg->dmp_en = (data >> 7U) & 0x01U;
    reg->fifo_en = (data >> 6U) & 0x01U;
    reg->i2c_mst_en = (data >> 5U) & 0x01U;
    reg->i2c_if_dis = (data >> 4U) & 0x01U;
    reg->dmp_reset = (data >> 3U) & 0x01U;
    reg->fifo_reset = (data >> 2U) & 0x01U;
    reg->i2c_mst_reset = (data >> 1U) & 0x01U;
    reg->sig_cond_reset = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_user_ctrl_reg(mpu6050_t const* mpu6050,
                                        mpu6050_user_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data |= (reg->dmp_en & 0x01U) << 7U;
    data |= (reg->fifo_en & 0x01U) << 6U;
    data |= (reg->i2c_mst_en & 0x01U) << 5U;
    data |= (reg->i2c_if_dis & 0x01U) << 4U;
    data |= (reg->dmp_reset & 0x01U) << 3U;
    data |= (reg->fifo_reset & 0x01U) << 2U;
    data |= (reg->i2c_mst_reset & 0x01U) << 1U;
    data |= reg->sig_cond_reset & 0x01U;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_USER_CTRL, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_pwr_mgmt_1_reg(mpu6050_t const* mpu6050, mpu6050_pwr_mgmt_1_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_PWR_MGMT_1, &data, sizeof(data));

    reg->device_reset = (data >> 7U) & 0x01U;
    reg->sleep = (data >> 6U) & 0x01U;
    reg->cycle = (data >> 5U) & 0x01U;
    reg->temp_dis = (data >> 3U) & 0x01U;
    reg->clk_sel = data & 0x07U;

    return err;
}

mpu6050_err_t mpu6050_set_pwr_mgmt_1_reg(mpu6050_t const* mpu6050,
                                         mpu6050_pwr_mgmt_1_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_PWR_MGMT_1, &data, sizeof(data));

    data &= ~((0x01U << 7U) | (0x01U << 6U) | (0x01U << 5U) | (0x01U << 3U) | (0x01U << 2U) |
              (0x01U << 1U | 0x01U));

    data |= (reg->device_reset & 0x01U) << 7U;
    data |= (reg->sleep & 0x01U) << 6U;
    data |= (reg->cycle & 0x01U) << 5U;
    data |= (reg->temp_dis & 0x01U) << 3U;
    data |= reg->clk_sel & 0x07U;

    err |= mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_PWR_MGMT_1, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_pwr_mgmt_2_reg(mpu6050_t const* mpu6050, mpu6050_pwr_mgmt_2_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_PWR_MGMT_2, &data, sizeof(data));

    reg->lp_wake_ctrl = (data >> 7U) & 0x01U;
    reg->stby_xa = (data >> 5U) & 0x01U;
    reg->stby_ya = (data >> 4U) & 0x01U;
    reg->stby_za = (data >> 3U) & 0x01U;
    reg->stby_xg = (data >> 2U) & 0x01U;
    reg->stby_yg = (data >> 1U) & 0x01U;
    reg->stby_zg = data & 0x01U;

    return err;
}

mpu6050_err_t mpu6050_set_pwr_mgmt_2_reg(mpu6050_t const* mpu6050,
                                         mpu6050_pwr_mgmt_2_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_PWR_MGMT_2, &data, sizeof(data));

    data &= ~((0x01U << 7U) | (0x01U << 5U) | (0x01U << 4U) | (0x01U << 3U) | (0x01U << 2U) |
              (0x01U << 1U | 0x01U));

    data |= (reg->lp_wake_ctrl & 0x01U) << 7U;
    data |= (reg->stby_xa & 0x01U) << 5U;
    data |= (reg->stby_ya & 0x01U) << 4U;
    data |= (reg->stby_za & 0x01U) << 3U;
    data |= (reg->stby_xg & 0x01U) << 2U;
    data |= (reg->stby_yg & 0x01U) << 1U;
    data |= reg->stby_zg & 0x01U;

    err |= mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_PWR_MGMT_2, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_bank_sel_reg(mpu6050_t const* mpu6050, mpu6050_bank_sel_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_BANK_SEL, &data, sizeof(data));

    reg->prftch_en = (data >> 6U) & 0x01U;
    reg->cfg_user_bank = (data >> 5U) & 0x01U;
    reg->mem_sel = data & 0x1FU;

    return err;
}

mpu6050_err_t mpu6050_set_bank_sel_reg(mpu6050_t const* mpu6050, mpu6050_bank_sel_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_BANK_SEL, &data, sizeof(data));

    data &= ~((0x01U << 6U) | (0x01U << 5U) | (0x1FU));

    data |= (reg->prftch_en & 0x01U) << 6U;
    data |= (reg->cfg_user_bank & 0x01U) << 5U;
    data |= reg->mem_sel & 0x1FU;

    err |= mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_BANK_SEL, &data, sizeof(data));

    return err;
}

mpu6050_err_t mpu6050_get_mem_start_addr_reg(mpu6050_t const* mpu6050,
                                             mpu6050_mem_start_addr_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_MEM_START_ADDR, &data, sizeof(data));

    reg->start_addr = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_mem_start_addr_reg(mpu6050_t const* mpu6050,
                                             mpu6050_mem_start_addr_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->start_addr & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_MEM_START_ADDR, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_mem_r_w_reg(mpu6050_t const* mpu6050, mpu6050_mem_r_w_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_MEM_R_W, &data, sizeof(data));

    reg->mem_r_w = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_mem_r_w_reg(mpu6050_t const* mpu6050, mpu6050_mem_r_w_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->mem_r_w & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_MEM_R_W, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_dmp_cfg_1_reg(mpu6050_t const* mpu6050, mpu6050_dmp_cfg_1_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_DMP_CFG_1, &data, sizeof(data));

    reg->dmp_cfg_1 = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_dmp_cfg_1_reg(mpu6050_t const* mpu6050,
                                        mpu6050_dmp_cfg_1_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->dmp_cfg_1 & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_DMP_CFG_1, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_dmp_cfg_2_reg(mpu6050_t const* mpu6050, mpu6050_dmp_cfg_2_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_DMP_CFG_2, &data, sizeof(data));

    reg->dmp_cfg_2 = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_dmp_cfg_2_reg(mpu6050_t const* mpu6050,
                                        mpu6050_dmp_cfg_2_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->dmp_cfg_2 & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_DMP_CFG_2, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_fifo_count_reg(mpu6050_t const* mpu6050, mpu6050_fifo_count_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_FIFO_COUNTH, data, sizeof(data));

    reg->fifo_count = (uint16_t)(((data[0] & 0xFFU) << 8U) | (data[1] & 0xFFU));

    return err;
}

mpu6050_err_t mpu6050_set_fifo_count_reg(mpu6050_t const* mpu6050,
                                         mpu6050_fifo_count_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data[2] = {};

    data[0] = (reg->fifo_count >> 8U) & 0xFFU;
    data[1] = reg->fifo_count & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_FIFO_COUNTH, data, sizeof(data));
}

mpu6050_err_t mpu6050_get_fifo_r_w_reg(mpu6050_t const* mpu6050, mpu6050_fifo_r_w_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_FIFO_R_W, &data, sizeof(data));

    reg->fifo_r_w = data & 0xFFU;

    return err;
}

mpu6050_err_t mpu6050_set_fifo_r_w_reg(mpu6050_t const* mpu6050, mpu6050_fifo_r_w_reg_t const* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    data = reg->fifo_r_w & 0xFFU;

    return mpu6050_bus_write_data(mpu6050, MPU6050_REG_ADDRESS_FIFO_R_W, &data, sizeof(data));
}

mpu6050_err_t mpu6050_get_who_am_i_reg(mpu6050_t const* mpu6050, mpu6050_who_am_i_reg_t* reg)
{
    assert(mpu6050 && reg);

    uint8_t data = {};

    mpu6050_err_t err =
        mpu6050_bus_read_data(mpu6050, MPU6050_REG_ADDRESS_WHO_AM_I, &data, sizeof(data));

    reg->who_am_i = (data >> 1U) & 0x3FU;

    return err;
}
