/**
 * @file    balance.h
 * @brief   直立环：互补滤波倾角 + 角度 PD
 *
 * 本文件不包含 HAL，换 MCU 时不用改。
 * 输入是加速度（单位 g）和角速度（单位 °/s），输出是 -1..1 的力矩。
 * motor_port 负责把这个力矩变成 PWM。两轮差速属于转向环，不要写进这里。
 *
 * 上车调试只改下面四个宏，计算过程不用动：
 *   1. 车体竖直时 g_balance_angle.angle_deg 应接近 0，前倾时应变大。
 *      前倾反而变负，把 BALANCE_ANGLE_SIGN 改成 -1。
 *   2. 慢慢前倾时，角速度应和角度同号（角度在增加，角速度为正）。
 *      两者相反就只改 BALANCE_GYRO_SIGN。符号不一致时，微分项会把车推倒。
 *   3. 打开 g_balance_enable 后，车若往倒下去的方向加速，
 *      只把 BALANCE_OUTPUT_SIGN 改成 -1。
 *   4. 俯仰不是绕丝印 Y 轴时，改 BALANCE_GYRO_AXIS。
 */
#ifndef BALANCE_H
#define BALANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** 与 mpu6050_data_t 的 accel_g / gyro_dps 下标一致 */
#define BALANCE_AXIS_X  0
#define BALANCE_AXIS_Y  1
#define BALANCE_AXIS_Z  2

/**
 * 车体俯仰所绕的轴，默认丝印 Y。
 * Y：角度用 accel_x、accel_z，角速度用 gyro_dps[1]。
 * X：角度用 accel_y、accel_z，角速度用 gyro_dps[0]。
 */
#ifndef BALANCE_GYRO_AXIS
#define BALANCE_GYRO_AXIS  BALANCE_AXIS_Y
#endif

/** 加速度计俯仰角的符号。前倾变负时改为 -1 */
#ifndef BALANCE_ANGLE_SIGN
#define BALANCE_ANGLE_SIGN  1.0f
#endif

/**
 * 陀螺角速度的符号，必须和角度变化方向一致。
 * 前倾时角度增加、角速度却为负，改为 -1。
 */
#ifndef BALANCE_GYRO_SIGN
#define BALANCE_GYRO_SIGN  1.0f
#endif

/**
 * 力矩方向。PD 算出的正力矩应把车往直立方向推。
 * 车往倒的方向跑时改为 -1。不要用它去修正角度符号。
 */
#ifndef BALANCE_OUTPUT_SIGN
#define BALANCE_OUTPUT_SIGN  1.0f
#endif

/**
 * 互补滤波状态。
 * angle_deg 是当前俯仰角，直立附近为 0，单位度。
 * alpha 是陀螺权重：0.98 表示 98% 用陀螺积分，2% 用加速度计把漂移拉回来。
 * inited 为 0 时，下一次更新直接采用加速度计角度，不从 0° 开始积分。
 */
typedef struct {
    float angle_deg;
    float alpha;
    uint8_t inited;
} balance_angle_t;

/**
 * 直立环参数，调试器里可以直接改这些成员。
 *
 *   力矩 = kp * (angle_deg - angle_zero) + kd * gyro_dps
 *   再乘 BALANCE_OUTPUT_SIGN，并限制在 -1..1。
 *
 * kp           角度增益。偏差 1° 产生 kp 的占空比。
 * kd           角速度阻尼。单位是 1/(°/s)，正值抑制倾倒速度。
 * angle_zero   机械中值，单位度。MPU 没装正时，竖直停稳的角度写到这里。
 * fall_deg     相对中值超过该角度视为倒地，力矩强制为 0。小于等于 0 表示关闭。
 */
typedef struct {
    float kp;
    float kd;
    float angle_zero;
    float fall_deg;
} balance_pd_t;

/** alpha 置 0.98，角度清零，并标记尚未用加速度计对准 */
void balance_angle_init(balance_angle_t *st);

/** kp=0.04，kd=0.004，中值 0°，倒地阈值 45°。这是小增益起点，上车后再加大 */
void balance_pd_init(balance_pd_t *pd);

/**
 * 用加速度计算俯仰角，单位度，已乘 BALANCE_ANGLE_SIGN。
 * axis 是旋转轴。静止直立、该轴水平时，结果接近 0。
 * 加速度计有振动噪声，只用来修正陀螺漂移，不直接拿去控电机。
 */
float balance_accel_angle_deg(const float accel_g[3], int axis);

/** 取出旋转轴上的角速度，单位 °/s，已乘 BALANCE_GYRO_SIGN */
float balance_gyro_dps(const float gyro_dps[3], int axis);

/**
 * 互补滤波更新 angle_deg。
 * dt_s 是距上次调用的时间，单位秒；超过 20 ms 会截断，避免一次卡顿把角度积分飞掉。
 * 第一次调用只用加速度计角度完成对准。
 */
void balance_angle_update(balance_angle_t *st, float accel_angle_deg,
                          float gyro_dps, float dt_s);

/**
 * 计算直立环力矩，范围 -1..1。
 * 相对机械中值的倾角超过 fall_deg 时返回 0，避免倒地后电机继续顶。
 */
float balance_pd_step(const balance_pd_t *pd, float angle_deg, float gyro_dps);

/**
 * 一次完整直立环：按 axis 取轴、更新互补滤波、计算 PD。
 * dt_s 用主循环的实际间隔。返回值含义与 balance_pd_step() 相同。
 * st 或 pd 为空时返回 0。
 */
float balance_step(balance_angle_t *st, const balance_pd_t *pd,
                   const float accel_g[3], const float gyro_dps[3],
                   int axis, float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* BALANCE_H */
