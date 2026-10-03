/**
 * @file    mpu6050.h
 * @brief   MPU6050 驱动（与 MCU、I2C 外设无关）
 *
 * 移植时只需要实现三个总线回调，见 mpu6050_io_t：
 *   - write：从寄存器地址起连续写
 *   - read ：从寄存器地址起连续读（需支持重复起始）
 *   - delay_ms：毫秒延时
 * 回调返回 0 表示成功，负数表示失败。dev_addr 一律使用 7 位地址
 * （AD0 接地为 0x68，接高为 0x69）。本文件不包含任何 HAL 头文件。
 *
 * 本驱动只负责传感器本身：唤醒、量程、采样、原始值和物理量。
 * 倾角解算属于控制层，不要放进驱动。
 */
#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** AD0 接 GND 时的 7 位地址 */
#define MPU6050_ADDR_AD0_LOW          0x68U
/** AD0 接 VCC 时的 7 位地址 */
#define MPU6050_ADDR_AD0_HIGH         0x69U

/** WHO_AM_I 复位值。部分兼容芯片不是这个值，初始化会因此失败。 */
#define MPU6050_WHO_AM_I_VALUE        0x68U

/* 寄存器地址，与 MPU-6000/MPU-6050 寄存器映射一致 */
#define MPU6050_REG_SMPLRT_DIV        0x19U
#define MPU6050_REG_CONFIG            0x1AU
#define MPU6050_REG_GYRO_CONFIG       0x1BU
#define MPU6050_REG_ACCEL_CONFIG      0x1CU
#define MPU6050_REG_INT_PIN_CFG       0x37U
#define MPU6050_REG_INT_ENABLE        0x38U
#define MPU6050_REG_INT_STATUS        0x3AU
#define MPU6050_REG_ACCEL_XOUT_H      0x3BU
#define MPU6050_REG_TEMP_OUT_H        0x41U
#define MPU6050_REG_GYRO_XOUT_H       0x43U
#define MPU6050_REG_PWR_MGMT_1        0x6BU
#define MPU6050_REG_PWR_MGMT_2        0x6CU
#define MPU6050_REG_WHO_AM_I          0x75U

/** 一次突发读取的字节数：加速度 6 + 温度 2 + 角速度 6 */
#define MPU6050_BURST_LEN             14U

#define MPU6050_PWR1_DEVICE_RESET     0x80U
#define MPU6050_PWR1_CLKSEL_PLL_X     0x01U
#define MPU6050_INT_DATA_RDY_EN       0x01U

typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_PARAM = -1,   /**< 空指针、地址或量程非法 */
    MPU6050_ERR_BUS = -2,     /**< I2C 回调返回失败 */
    MPU6050_ERR_ID = -3,      /**< WHO_AM_I 不是 0x68 */
} mpu6050_status_t;

/** 加速度满量程，枚举值与 ACCEL_CONFIG.AFS_SEL 一致 */
typedef enum {
    MPU6050_ACCEL_2G = 0,
    MPU6050_ACCEL_4G = 1,
    MPU6050_ACCEL_8G = 2,
    MPU6050_ACCEL_16G = 3,
} mpu6050_accel_range_t;

/** 角速度满量程，枚举值与 GYRO_CONFIG.FS_SEL 一致 */
typedef enum {
    MPU6050_GYRO_250DPS = 0,
    MPU6050_GYRO_500DPS = 1,
    MPU6050_GYRO_1000DPS = 2,
    MPU6050_GYRO_2000DPS = 3,
} mpu6050_gyro_range_t;

/**
 * 数字低通。枚举值等于 CONFIG.DLPF_CFG。
 * 名称取陀螺仪带宽。DLPF 不为 0 时，内部输出率为 1 kHz。
 */
typedef enum {
    MPU6050_DLPF_GYRO_256HZ = 0,
    MPU6050_DLPF_GYRO_188HZ = 1,
    MPU6050_DLPF_GYRO_98HZ = 2,
    MPU6050_DLPF_GYRO_42HZ = 3,
    MPU6050_DLPF_GYRO_20HZ = 4,
    MPU6050_DLPF_GYRO_10HZ = 5,
    MPU6050_DLPF_GYRO_5HZ = 6,
} mpu6050_dlpf_t;

typedef struct {
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t gyro_range;
    mpu6050_dlpf_t dlpf;
    /**
     * 采样率分频。DLPF 开启时：
     *   sample_rate = 1000 / (1 + sample_rate_div)  Hz
     * 平衡车常用 div = 4，即 200 Hz。
     */
    uint8_t sample_rate_div;
} mpu6050_config_t;

/**
 * 平台相关的总线接口。换 MCU 时实现这三个函数，驱动本体不用改。
 * bus_ctx 原样传回，可放 I2C 句柄或软件 I2C 的引脚描述。
 */
typedef struct {
    int (*write)(void *bus_ctx, uint8_t dev_addr, uint8_t reg,
                 const uint8_t *data, uint16_t len);
    int (*read)(void *bus_ctx, uint8_t dev_addr, uint8_t reg,
                uint8_t *data, uint16_t len);
    void (*delay_ms)(void *bus_ctx, uint32_t ms);
    void *bus_ctx;
} mpu6050_io_t;

/** 寄存器原始值，大端已转成有符号整数 */
typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t temp_raw;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} mpu6050_raw_t;

/**
 * 物理量。
 * accel_g 单位 g，gyro_dps 单位 °/s（已减去陀螺零偏），temp_c 单位 °C。
 * 轴顺序与芯片丝印一致：x、y、z。安装方向由上层做坐标变换。
 */
typedef struct {
    float accel_g[3];
    float gyro_dps[3];
    float temp_c;
} mpu6050_data_t;

typedef struct {
    mpu6050_io_t io;
    uint8_t addr;
    float accel_lsb_per_g;
    float gyro_lsb_per_dps;
    float gyro_bias_dps[3];
} mpu6050_t;

/** 平衡车默认：±2 g、±500 °/s、陀螺带宽 42 Hz、200 Hz 采样 */
void mpu6050_config_default(mpu6050_config_t *cfg);

/**
 * 绑定总线并初始化芯片。cfg 传 NULL 时使用 mpu6050_config_default()。
 * 返回 MPU6050_ERR_ID 时，dev 仍已绑定，可用 mpu6050_whoami() 看实际 ID。
 */
mpu6050_status_t mpu6050_init(mpu6050_t *dev, const mpu6050_io_t *io,
                              uint8_t addr_7bit, const mpu6050_config_t *cfg);

mpu6050_status_t mpu6050_whoami(mpu6050_t *dev, uint8_t *id);

/** 读 0x3B 起的 14 字节，一次总线事务，避免加速度和角速度不是同一时刻 */
mpu6050_status_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw);

/** 转成物理量，并从角速度中减去 gyro_bias_dps */
mpu6050_status_t mpu6050_read(mpu6050_t *dev, mpu6050_data_t *data);

/**
 * 静止状态下采集陀螺零偏。采集期间设备不要动。
 * samples 建议 200 以上；采样间隔 5 ms，与 200 Hz 输出率匹配。
 */
mpu6050_status_t mpu6050_calibrate_gyro(mpu6050_t *dev, uint16_t samples);

void mpu6050_set_gyro_bias(mpu6050_t *dev, const float bias_dps[3]);
void mpu6050_get_gyro_bias(const mpu6050_t *dev, float bias_dps[3]);

/**
 * 打开数据就绪中断（INT 引脚）。本工程 CubeMX 尚未配置 INT 引脚，
 * 需要时再调用。读走传感器数据后，INT 状态由芯片自动清除。
 */
mpu6050_status_t mpu6050_enable_data_ready(mpu6050_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
