#include "motor.h"

#include <stdbool.h>
#include <stdint.h>

#include "tim.h"

/*
 * 这个文件负责两件事：
 * 1) 把 LADRC 输出的有符号控制量，转换成 H 桥两路 PWM
 * 2) 读取四路编码器，得到实时 RPM，并累加成供 CAN 上传的里程计增量
 *
 * 约定：
 * - 上层统一把“车体前进”视为正方向
 * - 由于左右电机安装方向镜像，左侧电机的硬件方向需要在底层翻转
 */

/* ================== 全局静态变量 ================== */
static uint16_t s_last_count[4] = {0U, 0U, 0U, 0U};
static volatile float s_current_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static volatile int32_t s_odom_acc_ticks[4] = {0, 0, 0, 0};

/* ================== 内部辅助函数声明 ================== */
static inline void Motor_WriteBridge(TIM_HandleTypeDef *htim,
                                     uint32_t channel_in1,
                                     uint32_t channel_in2,
                                     uint16_t pwm_in1,
                                     uint16_t pwm_in2);
static void Motor_PrimeEncoderCounters(void);
static uint16_t Motor_ClampAbsToPwm(int32_t value);
static int16_t Motor_SaturateToI16(int32_t value);

/**
  * @brief  给 H 桥两路输入分别写入 PWM。
  * @note   AT8236 方向约定：
  *         - 正转：IN1 = PWM，IN2 = 0
  *         - 反转：IN1 = 0，  IN2 = PWM
  */
static inline void Motor_WriteBridge(TIM_HandleTypeDef *htim,
                                     uint32_t channel_in1,
                                     uint32_t channel_in2,
                                     uint16_t pwm_in1,
                                     uint16_t pwm_in2)
{
    __HAL_TIM_SET_COMPARE(htim, channel_in1, pwm_in1);
    __HAL_TIM_SET_COMPARE(htim, channel_in2, pwm_in2);
}

/**
  * @brief  把任意有符号数限幅并取绝对值，转换成 PWM 占空比。
  */
static uint16_t Motor_ClampAbsToPwm(int32_t value)
{
    if (value > PWM_LIMIT)
    {
        value = PWM_LIMIT;
    }
    else if (value < -PWM_LIMIT)
    {
        value = -PWM_LIMIT;
    }

    if (value < 0)
    {
        value = -value;
    }

    return (uint16_t)value;
}

/**
  * @brief  int32 饱和到 int16，防止 CAN 打包时溢出回绕。
  */
static int16_t Motor_SaturateToI16(int32_t value)
{
    if (value > 32767)
    {
        return 32767;
    }
    if (value < -32768)
    {
        return -32768;
    }
    return (int16_t)value;
}

/**
  * @brief  在编码器启动后，用当前计数值作为初值。
  * @note   这样可以避免“last_count 初始为 0”导致首个控制周期出现巨大假速度。
  */
static void Motor_PrimeEncoderCounters(void)
{
    s_last_count[MOTOR_FL] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
    s_last_count[MOTOR_RL] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    s_last_count[MOTOR_FR] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    s_last_count[MOTOR_RR] = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);
}

/**
  * @brief  启动所有 PWM 和编码器接口。
  */
void Motor_Init(void)
{
    uint32_t primask = __get_PRIMASK();

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4) != HAL_OK) { Error_Handler(); }

    if (HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK) { Error_Handler(); }
    if (HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK) { Error_Handler(); }

    /* 启动完成后，把输出和测速状态清零。 */
    Motor_Brake_All();
    Motor_PrimeEncoderCounters();

    __disable_irq();
    s_current_rpm[MOTOR_FL] = 0.0f;
    s_current_rpm[MOTOR_RL] = 0.0f;
    s_current_rpm[MOTOR_FR] = 0.0f;
    s_current_rpm[MOTOR_RR] = 0.0f;
    s_odom_acc_ticks[MOTOR_FL] = 0;
    s_odom_acc_ticks[MOTOR_RL] = 0;
    s_odom_acc_ticks[MOTOR_FR] = 0;
    s_odom_acc_ticks[MOTOR_RR] = 0;
    __set_PRIMASK(primask);
}

/**
  * @brief  把有符号控制量转换为电机驱动输出。
  * @param  id          电机编号
  * @param  control_out 上层控制量，正值表示“希望车体前进”
  * @note
  * 1) 这里先按“车体坐标系”理解 control_out。
  * 2) 左侧电机由于安装镜像，硬件方向要翻转一次。
  * 3) 变量 forward 的语义始终保持一致：
  *    true  -> IN1 = PWM, IN2 = 0
  *    false -> IN1 = 0,   IN2 = PWM
  */
void Set_Motor_Output(Motor_ID_t id, int16_t control_out)
{
    int32_t hw_command = (int32_t)control_out;
    bool forward;
    uint16_t pwm_val;

    /* 左侧硬件方向镜像：同样的“车体前进”命令，左轮需要反向转。 */
    if ((id == MOTOR_FL) || (id == MOTOR_RL))
    {
        hw_command = -hw_command;
    }

    forward = (hw_command >= 0);
    pwm_val = Motor_ClampAbsToPwm(hw_command);

    switch (id)
    {
        case MOTOR_FL:
            if (forward)
            {
                Motor_WriteBridge(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_val, 0U);
            }
            else
            {
                Motor_WriteBridge(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 0U, pwm_val);
            }
            break;

        case MOTOR_RL:
            if (forward)
            {
                Motor_WriteBridge(&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2, pwm_val, 0U);
            }
            else
            {
                Motor_WriteBridge(&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2, 0U, pwm_val);
            }
            break;

        case MOTOR_FR:
            if (forward)
            {
                Motor_WriteBridge(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, pwm_val, 0U);
            }
            else
            {
                Motor_WriteBridge(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, 0U, pwm_val);
            }
            break;

        case MOTOR_RR:
            if (forward)
            {
                Motor_WriteBridge(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, pwm_val, 0U);
            }
            else
            {
                Motor_WriteBridge(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, 0U, pwm_val);
            }
            break;

        default:
            break;
    }
}

/**
  * @brief  停止四个电机输出。
  * @note   这里使用“两个方向脚都置 0”的空转停车方式。
  *         如果你的驱动板支持真正的短刹车，并且你确认接线与驱动逻辑，
  *         可以单独再加一个 Brake 模式。
  */
void Motor_Brake_All(void)
{
    Motor_WriteBridge(&htim2, TIM_CHANNEL_3, TIM_CHANNEL_4, 0U, 0U);
    Motor_WriteBridge(&htim9, TIM_CHANNEL_1, TIM_CHANNEL_2, 0U, 0U);
    Motor_WriteBridge(&htim8, TIM_CHANNEL_1, TIM_CHANNEL_2, 0U, 0U);
    Motor_WriteBridge(&htim8, TIM_CHANNEL_3, TIM_CHANNEL_4, 0U, 0U);
}

/**
  * @brief  读取编码器脉冲并更新实时转速。
  * @param  dt_s 控制周期，单位秒，例如 10ms 就传 0.01f
  * @note
  * - 利用 int16_t 差分天然处理 16 位编码器计数器回绕
  * - 左侧编码器输入在这里反相，保证上层统一把“车体前进”看成正 RPM
  */
void Motor_Update_RPM(float dt_s)
{
    uint16_t curr_count_fl;
    uint16_t curr_count_rl;
    uint16_t curr_count_fr;
    uint16_t curr_count_rr;
    int16_t delta[4];
    float rpm_scale;

    if (dt_s <= 0.0f)
    {
        return;
    }

    curr_count_fl = (uint16_t)__HAL_TIM_GET_COUNTER(&htim5);
    curr_count_rl = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    curr_count_fr = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    curr_count_rr = (uint16_t)__HAL_TIM_GET_COUNTER(&htim1);

    delta[MOTOR_FL] = (int16_t)(curr_count_fl - s_last_count[MOTOR_FL]);
    delta[MOTOR_RL] = (int16_t)(curr_count_rl - s_last_count[MOTOR_RL]);
    delta[MOTOR_FR] = (int16_t)(curr_count_fr - s_last_count[MOTOR_FR]);
    delta[MOTOR_RR] = (int16_t)(curr_count_rr - s_last_count[MOTOR_RR]);

    s_last_count[MOTOR_FL] = curr_count_fl;
    s_last_count[MOTOR_RL] = curr_count_rl;
    s_last_count[MOTOR_FR] = curr_count_fr;
    s_last_count[MOTOR_RR] = curr_count_rr;

    /* 左侧输入镜像修正：让“车体前进”统一表现为正脉冲 / 正 RPM。 */
    delta[MOTOR_FL] = (int16_t)(-delta[MOTOR_FL]);
    delta[MOTOR_RL] = (int16_t)(-delta[MOTOR_RL]);

    /* 用 32 位内部累加，避免偶发高速度时 16 位回绕。 */
    s_odom_acc_ticks[MOTOR_FL] += delta[MOTOR_FL];
    s_odom_acc_ticks[MOTOR_RL] += delta[MOTOR_RL];
    s_odom_acc_ticks[MOTOR_FR] += delta[MOTOR_FR];
    s_odom_acc_ticks[MOTOR_RR] += delta[MOTOR_RR];

    rpm_scale = 60.0f / ((float)PULSES_PER_REV * dt_s);
    s_current_rpm[MOTOR_FL] = (float)delta[MOTOR_FL] * rpm_scale;
    s_current_rpm[MOTOR_RL] = (float)delta[MOTOR_RL] * rpm_scale;
    s_current_rpm[MOTOR_FR] = (float)delta[MOTOR_FR] * rpm_scale;
    s_current_rpm[MOTOR_RR] = (float)delta[MOTOR_RR] * rpm_scale;
}

/**
  * @brief  获取某个电机的实时 RPM。
  */
float Get_Motor_RPM(Motor_ID_t id)
{
    if ((id < MOTOR_FL) || (id > MOTOR_RR))
    {
        return 0.0f;
    }

    return s_current_rpm[id];
}

/**
  * @brief  读取并清空供 CAN 上传的里程计增量。
  * @note   这个函数会被主循环调用，而累加动作发生在 10ms 中断里，
  *         所以“读取 + 清零”必须放在同一个临界区，避免丢计数。
  */
void Motor_Get_And_Clear_Delta_Ticks(int16_t *d_fl, int16_t *d_rl, int16_t *d_fr, int16_t *d_rr)
{
    int32_t fl;
    int32_t rl;
    int32_t fr;
    int32_t rr;
    uint32_t primask;

    if ((d_fl == NULL) || (d_rl == NULL) || (d_fr == NULL) || (d_rr == NULL))
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    fl = s_odom_acc_ticks[MOTOR_FL];
    rl = s_odom_acc_ticks[MOTOR_RL];
    fr = s_odom_acc_ticks[MOTOR_FR];
    rr = s_odom_acc_ticks[MOTOR_RR];

    s_odom_acc_ticks[MOTOR_FL] = 0;
    s_odom_acc_ticks[MOTOR_RL] = 0;
    s_odom_acc_ticks[MOTOR_FR] = 0;
    s_odom_acc_ticks[MOTOR_RR] = 0;

    __set_PRIMASK(primask);

    *d_fl = Motor_SaturateToI16(fl);
    *d_rl = Motor_SaturateToI16(rl);
    *d_fr = Motor_SaturateToI16(fr);
    *d_rr = Motor_SaturateToI16(rr);
}
