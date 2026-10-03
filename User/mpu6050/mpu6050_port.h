/**
 * @file    mpu6050_port.h
 * @brief   把可移植的 MPU6050 驱动接到 STM32 HAL I2C
 *
 * 换平台时替换本文件和 mpu6050_port.c，mpu6050.c 保持不动。
 * 当前工程使用 I2C1，SCL = PB6，SDA = PB7，100 kHz，AD0 默认接地。
 */
#ifndef MPU6050_PORT_H
#define MPU6050_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "mpu6050.h"
#include "i2c.h"

/**
 * 用 HAL I2C 初始化 MPU6050。
 * hi2c 传 &hi2c1。addr_7bit 用 MPU6050_ADDR_AD0_LOW 或 MPU6050_ADDR_AD0_HIGH。
 * cfg 传 NULL 时用平衡车默认配置。
 *
 * 静止标定（开机且车体放稳之后再调用）：
 *   mpu6050_calibrate_gyro(dev, 200);
 */
mpu6050_status_t mpu6050_port_init(mpu6050_t *dev, I2C_HandleTypeDef *hi2c,
                                   uint8_t addr_7bit, const mpu6050_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_PORT_H */
