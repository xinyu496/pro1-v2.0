/**
 * @file    balance.c
 * @brief   直立环计算
 *
 * 只依赖 math.h，不含芯片头文件。移植时保持本文件不动，
 * 换一套电机 PWM 实现即可。
 */
#include "balance.h"

#include <math.h>

/** 弧度转角度：180/π */
#define BALANCE_RAD_TO_DEG  57.2957795f
/** 互补滤波允许的最大积分步长。主循环正常是 5 ms */
#define BALANCE_DT_MAX_S    0.020f
/** 陀螺权重。剩下的 0.02 用加速度计缓慢修正零偏和安装误差 */
#define BALANCE_ALPHA       0.98f
/** 相对机械中值超过该角度则认为已经倒地 */
#define BALANCE_FALL_DEG    45.0f

/**
 * 初始增益故意取小，上电误打开使能时轮子不会猛转。
 * 5° 偏差约对应 0.2 占空比，50 °/s 约对应 0.2 的阻尼。
 * 车扶不住时再加大 kp，振荡时再加大 kd。
 */
#define BALANCE_KP_INIT     0.04f
#define BALANCE_KD_INIT     0.004f

/** 把 value 限制在 [-limit, limit]。limit 应为正数 */
static float balance_clampf(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

void balance_angle_init(balance_angle_t *st)
{
    if (st == 0) {
        return;
    }
    st->angle_deg = 0.0f;
    st->alpha = BALANCE_ALPHA;
    st->inited = 0U;
}

void balance_pd_init(balance_pd_t *pd)
{
    if (pd == 0) {
        return;
    }
    pd->kp = BALANCE_KP_INIT;
    pd->kd = BALANCE_KD_INIT;
    pd->angle_zero = 0.0f;
    pd->fall_deg = BALANCE_FALL_DEG;
}

float balance_accel_angle_deg(const float accel_g[3], int axis)
{
    float a;
    float b;

    if (accel_g == 0) {
        return 0.0f;
    }

    /*
     * 俯仰角只看垂直于旋转轴的两个加速度分量。
     * 直立且该轴水平时，重力几乎全在 b 上，atan2(a, b) 接近 0。
     * 绕 Y 倾时 a=ax、b=az；绕 X 倾时 a=ay、b=az。
     */
    if (axis == BALANCE_AXIS_X) {
        a = accel_g[1];
        b = accel_g[2];
    } else if (axis == BALANCE_AXIS_Z) {
        a = accel_g[0];
        b = accel_g[1];
    } else {
        a = accel_g[0];
        b = accel_g[2];
    }

    return atan2f(a, b) * BALANCE_RAD_TO_DEG * BALANCE_ANGLE_SIGN;
}

float balance_gyro_dps(const float gyro_dps[3], int axis)
{
    if ((gyro_dps == 0) || (axis < BALANCE_AXIS_X) || (axis > BALANCE_AXIS_Z)) {
        return 0.0f;
    }
    return gyro_dps[axis] * BALANCE_GYRO_SIGN;
}

void balance_angle_update(balance_angle_t *st, float accel_angle_deg,
                          float gyro_dps, float dt_s)
{
    float gyro_angle;
    float alpha;

    if (st == 0) {
        return;
    }

    /* 上电第一帧没有历史角度，直接用加速度计，避免从 0° 积分出假倾角 */
    if (st->inited == 0U) {
        st->angle_deg = accel_angle_deg;
        st->inited = 1U;
        return;
    }

    if (dt_s < 0.0f) {
        dt_s = 0.0f;
    } else if (dt_s > BALANCE_DT_MAX_S) {
        dt_s = BALANCE_DT_MAX_S;
    }

    alpha = st->alpha;
    if (alpha < 0.0f) {
        alpha = 0.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }

    /*
     * 陀螺积分响应快，但会漂移；加速度计长期准，但电机一振就抖。
     * angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel_angle
     */
    gyro_angle = st->angle_deg + (gyro_dps * dt_s);
    st->angle_deg = (alpha * gyro_angle) + ((1.0f - alpha) * accel_angle_deg);
}

float balance_pd_step(const balance_pd_t *pd, float angle_deg, float gyro_dps)
{
    float err;
    float out;

    if (pd == 0) {
        return 0.0f;
    }

    err = angle_deg - pd->angle_zero;

    /* 倒地后继续输出会让轮子空转，这里直接卸力 */
    if (pd->fall_deg > 0.0f) {
        float abs_err = (err >= 0.0f) ? err : -err;
        if (abs_err > pd->fall_deg) {
            return 0.0f;
        }
    }

    /* kd 项阻尼倾倒速度。gyro 与 d(angle)/dt 同号时，这项才会往回拉 */
    out = (pd->kp * err) + (pd->kd * gyro_dps);
    out *= BALANCE_OUTPUT_SIGN;
    return balance_clampf(out, 1.0f);
}

float balance_step(balance_angle_t *st, const balance_pd_t *pd,
                   const float accel_g[3], const float gyro_dps[3],
                   int axis, float dt_s)
{
    float accel_angle;
    float gyro;

    accel_angle = balance_accel_angle_deg(accel_g, axis);
    gyro = balance_gyro_dps(gyro_dps, axis);
    balance_angle_update(st, accel_angle, gyro, dt_s);
    if (st == 0) {
        return 0.0f;
    }
    return balance_pd_step(pd, st->angle_deg, gyro);
}
