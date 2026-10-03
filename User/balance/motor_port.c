/**
 * @file    motor_port.c
 * @brief   STM32 电机适配
 *
 * TIM3 通道 1 = PA6，通道 2 = PA7，方向 = PC1。
 * 比较值按定时器当前自动重装值换算，CubeMX 里改 Period 不用改本文件。
 * 现配置 Prescaler = 0、Period = 65535，TIM3 时钟 84 MHz，PWM 约 1.28 kHz。
 */
#include "motor_port.h"

#include "gpio.h"
#include "tim.h"

#include <math.h>

void motor_port_init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    motor_port_set(0.0f);
}

void motor_port_set(float duty)
{
    uint32_t arr;
    uint32_t ccr;
    float mag;
    GPIO_PinState dir;

    if (duty > 1.0f) {
        duty = 1.0f;
    } else if (duty < -1.0f) {
        duty = -1.0f;
    }

    /*
     * CCR = |duty| * ARR，满幅时比较值等于重装值，占空比 100%。
     * 不用 ARR+1，是因为 65535+1 已经超出 16 位比较寄存器。
     */
    arr = __HAL_TIM_GET_AUTORELOAD(&htim3);
    mag = fabsf(duty);
    ccr = (uint32_t)(mag * (float)arr);
    if (ccr > arr) {
        ccr = arr;
    }

    /* 方向只看符号。两路 PWM 幅值相同，差速不在这一层做 */
    dir = (duty > 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, dir);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ccr);
}
