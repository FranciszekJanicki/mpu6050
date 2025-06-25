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
}

mpu6050_err_t mpu6050_set_xg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_xg_offs_usr_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_yg_offs_usr_reg(mpu6050_t const* mpu6050, mpu6050_yg_offs_usr_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_yg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_yg_offs_usr_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_zg_offs_usr_reg(mpu6050_t const* mpu6050, mpu6050_zg_offs_usr_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_zg_offs_usr_reg(mpu6050_t const* mpu6050,
                                          mpu6050_zg_offs_usr_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_smplrt_div_reg(mpu6050_t const* mpu6050, mpu6050_smplrt_div_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_smplrt_div_reg(mpu6050_t const* mpu6050,
                                         mpu6050_smplrt_div_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_config_reg(mpu6050_t const* mpu6050, mpu6050_config_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_config_reg(mpu6050_t const* mpu6050, mpu6050_config_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_gyro_config_reg(mpu6050_t const* mpu6050, mpu6050_gyro_config_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_gyro_config_reg(mpu6050_t const* mpu6050,
                                          mpu6050_gyro_config_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_accel_config_reg(mpu6050_t const* mpu6050,
                                           mpu6050_accel_config_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_accel_config_reg(mpu6050_t const* mpu6050,
                                           mpu6050_accel_config_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_ff_thr_reg(mpu6050_t const* mpu6050, mpu6050_ff_thr_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_ff_dur_reg(mpu6050_t const* mpu6050, mpu6050_ff_dur_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_mot_thr_reg(mpu6050_t const* mpu6050, mpu6050_mot_thr_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_mot_dur_reg(mpu6050_t const* mpu6050, mpu6050_mot_dur_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_zrmot_thr_reg(mpu6050_t const* mpu6050, mpu6050_zrmot_thr_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_zrmot_dur_reg(mpu6050_t const* mpu6050,
                                        mpu6050_zrmot_dur_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_fifo_en_reg(mpu6050_t const* mpu6050, mpu6050_fifo_en_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_fifo_en_reg(mpu6050_t const* mpu6050, mpu6050_fifo_en_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_mst_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_i2c_mst_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_mst_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_i2c_mst_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv_addr_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_addr_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv_addr_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_addr_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv_reg(mpu6050_t const* mpu6050,
                                      mpu6050_slave_num_t slave_num,
                                      mpu6050_i2c_slv_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv_reg(mpu6050_t const* mpu6050,
                                      mpu6050_slave_num_t slave_num,
                                      mpu6050_i2c_slv_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv_ctrl_reg(mpu6050_t const* mpu6050,
                                           mpu6050_slave_num_t slave_num,
                                           mpu6050_i2c_slv_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv4_addr_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_addr_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv4_addr_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_addr_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv4_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv4_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv4_do_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv4_do_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv4_do_reg(mpu6050_t const* mpu6050,
                                          mpu6050_i2c_slv4_do_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv4_ctrl_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv4_ctrl_reg(mpu6050_t const* mpu6050,
                                            mpu6050_i2c_slv4_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv4_di_reg(mpu6050_t const* mpu6050, mpu6050_i2c_slv4_di_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv4_di_reg(mpu6050_t const* mpu6050,
                                          mpu6050_i2c_slv4_di_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_mst_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_i2c_mst_status_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_mst_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_i2c_mst_status_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_int_pin_cfg_reg(mpu6050_t const* mpu6050, mpu6050_int_pin_cfg_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_int_pin_cfg_reg(mpu6050_t const* mpu6050,
                                          mpu6050_int_pin_cfg_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_int_enable_reg(mpu6050_t const* mpu6050, mpu6050_int_enable_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_int_enable_reg(mpu6050_t const* mpu6050,
                                         mpu6050_int_enable_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_dmp_int_status_reg(mpu6050_t const* mpu6050,
                                             mpu6050_dmp_int_status_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_int_status_reg(mpu6050_t const* mpu6050, mpu6050_int_status_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_accel_xout_reg(mpu6050_t const* mpu6050, mpu6050_accel_xout_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_accel_yout_reg(mpu6050_t const* mpu6050, mpu6050_accel_yout_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_accel_zout_reg(mpu6050_t const* mpu6050, mpu6050_accel_zout_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_accel_out_reg(mpu6050_t const* mpu6050, mpu6050_accel_out_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_temp_out_reg(mpu6050_t const* mpu6050, mpu6050_temp_out_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_gyro_xout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_xout_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_gyro_yout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_yout_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_gyro_zout_reg(mpu6050_t const* mpu6050, mpu6050_gyro_zout_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_gyro_out_reg(mpu6050_t const* mpu6050, mpu6050_gyro_out_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_ext_sens_data_reg(mpu6050_t const* mpu6050,
                                            uint8_t reg_num,
                                            mpu6050_ext_sens_data_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_mot_detect_status_reg(mpu6050_t const* mpu6050,
                                                mpu6050_mot_detect_status_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_slv_do_reg(mpu6050_t const* mpu6050,
                                         mpu6050_slave_num_t slave_num,
                                         mpu6050_i2c_slv_do_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_slv_do_reg(mpu6050_t const* mpu6050,
                                         mpu6050_slave_num_t slave_num,
                                         mpu6050_i2c_slv_do_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_i2c_mst_delay_ctrl_reg(mpu6050_t const* mpu6050,
                                                 mpu6050_i2c_mst_delay_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_i2c_mst_delay_ctrl_reg(mpu6050_t const* mpu6050,
                                                 mpu6050_i2c_mst_delay_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_signal_path_reset_reg(mpu6050_t const* mpu6050,
                                                mpu6050_signal_path_reset_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_signal_path_reset_reg(mpu6050_t const* mpu6050,
                                                mpu6050_signal_path_reset_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_mot_detect_ctrl_reg(mpu6050_t const* mpu6050,
                                              mpu6050_mot_detect_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_mot_detect_ctrl_reg(mpu6050_t const* mpu6050,
                                              mpu6050_mot_detect_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_user_ctrl_reg(mpu6050_t const* mpu6050, mpu6050_user_ctrl_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_user_ctrl_reg(mpu6050_t const* mpu6050,
                                        mpu6050_user_ctrl_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_pwr_mgmt_1_reg(mpu6050_t const* mpu6050, mpu6050_pwr_mgmt_1_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_pwr_mgmt_1_reg(mpu6050_t const* mpu6050,
                                         mpu6050_pwr_mgmt_1_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_pwr_mgmt_2_reg(mpu6050_t const* mpu6050, mpu6050_pwr_mgmt_2_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_pwr_mgmt_2_reg(mpu6050_t const* mpu6050,
                                         mpu6050_pwr_mgmt_2_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_bank_sel_reg(mpu6050_t const* mpu6050, mpu6050_bank_sel_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_bank_sel_reg(mpu6050_t const* mpu6050, mpu6050_bank_sel_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_mem_start_addr_reg(mpu6050_t const* mpu6050,
                                             mpu6050_mem_start_addr_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_mem_start_addr_reg(mpu6050_t const* mpu6050,
                                             mpu6050_mem_start_addr_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_mem_r_w_reg(mpu6050_t const* mpu6050, mpu6050_mem_r_w_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_mem_r_w_reg(mpu6050_t const* mpu6050, mpu6050_mem_r_w_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_dmp_cfg_1_reg(mpu6050_t const* mpu6050, mpu6050_dmp_cfg_1_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_dmp_cfg_1_reg(mpu6050_t const* mpu6050,
                                        mpu6050_dmp_cfg_1_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_dmp_cfg_2_reg(mpu6050_t const* mpu6050, mpu6050_dmp_cfg_2_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_dmp_cfg_2_reg(mpu6050_t const* mpu6050,
                                        mpu6050_dmp_cfg_2_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_fifo_count_reg(mpu6050_t const* mpu6050, mpu6050_fifo_count_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_fifo_count_reg(mpu6050_t const* mpu6050,
                                         mpu6050_fifo_count_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_fifo_r_w_reg(mpu6050_t const* mpu6050, mpu6050_fifo_r_w_reg_t* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_set_fifo_r_w_reg(mpu6050_t const* mpu6050, mpu6050_fifo_r_w_reg_t const* reg)
{
    assert(mpu6050 && reg);
}

mpu6050_err_t mpu6050_get_who_am_i_reg(mpu6050_t const* mpu6050, mpu6050_who_am_i_reg_t* reg)
{
    assert(mpu6050 && reg);
}
