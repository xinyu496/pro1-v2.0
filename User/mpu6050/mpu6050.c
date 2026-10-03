/**
 * @file    mpu6050.c
 * @brief   MPU6050 寄存器操作与单位换算
 *
 * 只依赖 mpu6050.h 里的总线回调，不包含芯片厂商头文件。
 */
#include "mpu6050.h"

/* 数据手册 Table 6.2 / 6.1 的典型灵敏度，单位 LSB/g 与 LSB/(°/s) */
static const float s_accel_lsb_per_g[4] = {
    16384.0f, 8192.0f, 4096.0f, 2048.0f
};

static const float s_gyro_lsb_per_dps[4] = {
    131.0f, 65.5f, 32.8f, 16.4f
};

static int16_t mpu6050_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static mpu6050_status_t mpu6050_write_u8(mpu6050_t *dev, uint8_t reg, uint8_t value)
{
    if (dev->io.write(dev->io.bus_ctx, dev->addr, reg, &value, 1U) != 0) {
        return MPU6050_ERR_BUS;
    }
    return MPU6050_OK;
}

static int mpu6050_range_ok(const mpu6050_config_t *cfg)
{
    if ((unsigned)cfg->accel_range > (unsigned)MPU6050_ACCEL_16G) {
        return 0;
    }
    if ((unsigned)cfg->gyro_range > (unsigned)MPU6050_GYRO_2000DPS) {
        return 0;
    }
    if ((unsigned)cfg->dlpf > (unsigned)MPU6050_DLPF_GYRO_5HZ) {
        return 0;
    }
    return 1;
}

void mpu6050_config_default(mpu6050_config_t *cfg)
{
    if (cfg == 0) {
        return;
    }

    /* ±2 g 对 1 g 重力的分辨率最高；±500 °/s 能盖住平衡车倾倒时的角速度 */
    cfg->accel_range = MPU6050_ACCEL_2G;
    cfg->gyro_range = MPU6050_GYRO_500DPS;
    cfg->dlpf = MPU6050_DLPF_GYRO_42HZ;
    cfg->sample_rate_div = 4U;
}

mpu6050_status_t mpu6050_init(mpu6050_t *dev, const mpu6050_io_t *io,
                              uint8_t addr_7bit, const mpu6050_config_t *cfg)
{
    mpu6050_config_t local;
    mpu6050_status_t status;
    uint8_t who = 0U;

    if ((dev == 0) || (io == 0) || (io->write == 0) || (io->read == 0) || (io->delay_ms == 0)) {
        return MPU6050_ERR_PARAM;
    }
    if ((addr_7bit != MPU6050_ADDR_AD0_LOW) && (addr_7bit != MPU6050_ADDR_AD0_HIGH)) {
        return MPU6050_ERR_PARAM;
    }

    if (cfg == 0) {
        mpu6050_config_default(&local);
        cfg = &local;
    }
    if (mpu6050_range_ok(cfg) == 0) {
        return MPU6050_ERR_PARAM;
    }

    dev->io = *io;
    dev->addr = addr_7bit;
    dev->accel_lsb_per_g = s_accel_lsb_per_g[cfg->accel_range];
    dev->gyro_lsb_per_dps = s_gyro_lsb_per_dps[cfg->gyro_range];
    dev->gyro_bias_dps[0] = 0.0f;
    dev->gyro_bias_dps[1] = 0.0f;
    dev->gyro_bias_dps[2] = 0.0f;

    /* 上电后内部时钟稳定再访问；复位会让芯片重新进入睡眠 */
    dev->io.delay_ms(dev->io.bus_ctx, 100U);

    status = mpu6050_write_u8(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_DEVICE_RESET);
    if (status != MPU6050_OK) {
        return status;
    }
    dev->io.delay_ms(dev->io.bus_ctx, 100U);

    /*
     * CLKSEL = 1：用 X 轴陀螺锁相环做时钟，比内部 8 MHz 振荡器稳。
     * 该写同时清除 SLEEP。PWR_MGMT_2 = 0 打开全部加速度和陀螺轴。
     */
    status = mpu6050_write_u8(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_PLL_X);
    if (status != MPU6050_OK) {
        return status;
    }
    status = mpu6050_write_u8(dev, MPU6050_REG_PWR_MGMT_2, 0x00U);
    if (status != MPU6050_OK) {
        return status;
    }
    dev->io.delay_ms(dev->io.bus_ctx, 50U);

    status = mpu6050_whoami(dev, &who);
    if (status != MPU6050_OK) {
        return status;
    }
    if (who != MPU6050_WHO_AM_I_VALUE) {
        return MPU6050_ERR_ID;
    }

    status = mpu6050_write_u8(dev, MPU6050_REG_SMPLRT_DIV, cfg->sample_rate_div);
    if (status != MPU6050_OK) {
        return status;
    }
    status = mpu6050_write_u8(dev, MPU6050_REG_CONFIG, (uint8_t)cfg->dlpf);
    if (status != MPU6050_OK) {
        return status;
    }
    /* FS_SEL / AFS_SEL 在 bit[4:3]，自检位保持 0 */
    status = mpu6050_write_u8(dev, MPU6050_REG_GYRO_CONFIG, (uint8_t)((uint8_t)cfg->gyro_range << 3));
    if (status != MPU6050_OK) {
        return status;
    }
    status = mpu6050_write_u8(dev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)((uint8_t)cfg->accel_range << 3));
    return status;
}

mpu6050_status_t mpu6050_whoami(mpu6050_t *dev, uint8_t *id)
{
    if ((dev == 0) || (id == 0) || (dev->io.read == 0)) {
        return MPU6050_ERR_PARAM;
    }
    if (dev->io.read(dev->io.bus_ctx, dev->addr, MPU6050_REG_WHO_AM_I, id, 1U) != 0) {
        return MPU6050_ERR_BUS;
    }
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw)
{
    uint8_t buf[MPU6050_BURST_LEN];

    if ((dev == 0) || (raw == 0) || (dev->io.read == 0)) {
        return MPU6050_ERR_PARAM;
    }

    if (dev->io.read(dev->io.bus_ctx, dev->addr, MPU6050_REG_ACCEL_XOUT_H,
                     buf, MPU6050_BURST_LEN) != 0) {
        return MPU6050_ERR_BUS;
    }

    raw->accel_x = mpu6050_be16(&buf[0]);
    raw->accel_y = mpu6050_be16(&buf[2]);
    raw->accel_z = mpu6050_be16(&buf[4]);
    raw->temp_raw = mpu6050_be16(&buf[6]);
    raw->gyro_x = mpu6050_be16(&buf[8]);
    raw->gyro_y = mpu6050_be16(&buf[10]);
    raw->gyro_z = mpu6050_be16(&buf[12]);
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_read(mpu6050_t *dev, mpu6050_data_t *data)
{
    mpu6050_raw_t raw;
    mpu6050_status_t status;
    float gyro_raw[3];
    uint8_t i;

    if ((dev == 0) || (data == 0)) {
        return MPU6050_ERR_PARAM;
    }
    if ((dev->accel_lsb_per_g == 0.0f) || (dev->gyro_lsb_per_dps == 0.0f)) {
        return MPU6050_ERR_PARAM;
    }

    status = mpu6050_read_raw(dev, &raw);
    if (status != MPU6050_OK) {
        return status;
    }

    data->accel_g[0] = (float)raw.accel_x / dev->accel_lsb_per_g;
    data->accel_g[1] = (float)raw.accel_y / dev->accel_lsb_per_g;
    data->accel_g[2] = (float)raw.accel_z / dev->accel_lsb_per_g;

    gyro_raw[0] = (float)raw.gyro_x / dev->gyro_lsb_per_dps;
    gyro_raw[1] = (float)raw.gyro_y / dev->gyro_lsb_per_dps;
    gyro_raw[2] = (float)raw.gyro_z / dev->gyro_lsb_per_dps;
    for (i = 0U; i < 3U; i++) {
        data->gyro_dps[i] = gyro_raw[i] - dev->gyro_bias_dps[i];
    }

    /* 数据手册：Temperature in degrees C = TEMP_OUT / 340 + 36.53 */
    data->temp_c = ((float)raw.temp_raw / 340.0f) + 36.53f;
    return MPU6050_OK;
}

mpu6050_status_t mpu6050_calibrate_gyro(mpu6050_t *dev, uint16_t samples)
{
    int32_t sum[3] = {0, 0, 0};
    uint16_t i;

    if ((dev == 0) || (samples == 0U) || (dev->io.delay_ms == 0)) {
        return MPU6050_ERR_PARAM;
    }
    if (dev->gyro_lsb_per_dps == 0.0f) {
        return MPU6050_ERR_PARAM;
    }

    /* 标定过程不要把正在计算的零偏减进去 */
    dev->gyro_bias_dps[0] = 0.0f;
    dev->gyro_bias_dps[1] = 0.0f;
    dev->gyro_bias_dps[2] = 0.0f;

    for (i = 0U; i < samples; i++) {
        mpu6050_raw_t raw;
        mpu6050_status_t status = mpu6050_read_raw(dev, &raw);
        if (status != MPU6050_OK) {
            return status;
        }
        sum[0] += raw.gyro_x;
        sum[1] += raw.gyro_y;
        sum[2] += raw.gyro_z;
        dev->io.delay_ms(dev->io.bus_ctx, 5U);
    }

    dev->gyro_bias_dps[0] = ((float)sum[0] / (float)samples) / dev->gyro_lsb_per_dps;
    dev->gyro_bias_dps[1] = ((float)sum[1] / (float)samples) / dev->gyro_lsb_per_dps;
    dev->gyro_bias_dps[2] = ((float)sum[2] / (float)samples) / dev->gyro_lsb_per_dps;
    return MPU6050_OK;
}

void mpu6050_set_gyro_bias(mpu6050_t *dev, const float bias_dps[3])
{
    if ((dev == 0) || (bias_dps == 0)) {
        return;
    }
    dev->gyro_bias_dps[0] = bias_dps[0];
    dev->gyro_bias_dps[1] = bias_dps[1];
    dev->gyro_bias_dps[2] = bias_dps[2];
}

void mpu6050_get_gyro_bias(const mpu6050_t *dev, float bias_dps[3])
{
    if ((dev == 0) || (bias_dps == 0)) {
        return;
    }
    bias_dps[0] = dev->gyro_bias_dps[0];
    bias_dps[1] = dev->gyro_bias_dps[1];
    bias_dps[2] = dev->gyro_bias_dps[2];
}

mpu6050_status_t mpu6050_enable_data_ready(mpu6050_t *dev)
{
    mpu6050_status_t status;

    if (dev == 0) {
        return MPU6050_ERR_PARAM;
    }

    /* INT 引脚推挽、高电平有效，数据就绪时置位 */
    status = mpu6050_write_u8(dev, MPU6050_REG_INT_PIN_CFG, 0x00U);
    if (status != MPU6050_OK) {
        return status;
    }
    return mpu6050_write_u8(dev, MPU6050_REG_INT_ENABLE, MPU6050_INT_DATA_RDY_EN);
}
