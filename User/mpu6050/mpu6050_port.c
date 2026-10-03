/**
 * @file    mpu6050_port.c
 * @brief   STM32 HAL 上的 MPU6050 总线适配
 *
 * HAL 的设备地址是 8 位（7 位地址左移 1）。这里做这一次转换，
 * 驱动层继续使用 7 位地址，换到软件 I2C 或其他厂家库时不用改调用方。
 */
#include "mpu6050_port.h"

#define MPU6050_I2C_TIMEOUT_MS  100U

static int mpu6050_hal_write(void *bus_ctx, uint8_t dev_addr, uint8_t reg,
                             const uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)bus_ctx;
    HAL_StatusTypeDef hal_status;

    if ((hi2c == 0) || (data == 0) || (len == 0U)) {
        return -1;
    }

    /*
     * HAL 接口的缓冲区不是 const。数据只被发送，不会被改写。
     */
    hal_status = HAL_I2C_Mem_Write(hi2c,
                                   (uint16_t)((uint16_t)dev_addr << 1),
                                   reg,
                                   I2C_MEMADD_SIZE_8BIT,
                                   (uint8_t *)data,
                                   len,
                                   MPU6050_I2C_TIMEOUT_MS);
    return (hal_status == HAL_OK) ? 0 : -1;
}

static int mpu6050_hal_read(void *bus_ctx, uint8_t dev_addr, uint8_t reg,
                            uint8_t *data, uint16_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)bus_ctx;
    HAL_StatusTypeDef hal_status;

    if ((hi2c == 0) || (data == 0) || (len == 0U)) {
        return -1;
    }

    /* Mem_Read 在写寄存器地址后发重复起始，符合 MPU6050 的读时序 */
    hal_status = HAL_I2C_Mem_Read(hi2c,
                                  (uint16_t)((uint16_t)dev_addr << 1),
                                  reg,
                                  I2C_MEMADD_SIZE_8BIT,
                                  data,
                                  len,
                                  MPU6050_I2C_TIMEOUT_MS);
    return (hal_status == HAL_OK) ? 0 : -1;
}

static void mpu6050_hal_delay(void *bus_ctx, uint32_t ms)
{
    (void)bus_ctx;
    HAL_Delay(ms);
}

mpu6050_status_t mpu6050_port_init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c,
                                   uint8_t addr_7bit, const mpu6050_config_t *cfg)
{
    mpu6050_io_t io;

    if (hi2c == 0) {
        return MPU6050_ERR_PARAM;
    }

    io.write = mpu6050_hal_write;
    io.read = mpu6050_hal_read;
    io.delay_ms = mpu6050_hal_delay;
    io.bus_ctx = hi2c;

    return mpu6050_init(dev, &io, addr_7bit, cfg);
}
