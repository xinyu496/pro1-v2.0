/**
 * @file    motor_port.h
 * @brief   把直立环力矩变成两路相同的 PWM
 *
 * 当前电路只有一个方向脚 PC1，两路 PWM 必须同占空比、同方向。
 * 这只够直立环。以后做转向环，需要给左右轮各自一个方向脚。
 *
 * 两个轮子面对面安装时，对调其中一个电机的两根电源线，
 * 这样同一路 PWM 会让两轮把车往同一侧推。
 * 正力矩时 PC1 为高。整车推的方向反了，改 balance.h 的 BALANCE_OUTPUT_SIGN。
 */
#ifndef MOTOR_PORT_H
#define MOTOR_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 启动 TIM3 通道 1（PA6）和通道 2（PA7），并把占空比、PC1 清成停止。
 * 调用前需要已经执行 MX_TIM3_Init() 和 MX_GPIO_Init()。
 */
void motor_port_init(void);

/**
 * 设置左右轮力矩。duty 取值 -1..1，超出部分截断。
 * 绝对值写成两路相同的比较值，符号写到 PC1：正为高，负或 0 为低。
 * duty 为 0 时比较值是 0，电机不应出力。
 */
void motor_port_set(float duty);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PORT_H */
