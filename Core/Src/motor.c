#include "motor.h"
#include "tim.h"  // 必须包含此文件以使用 htim1, htim2 等外部句柄

/* ================== 全局静态变量 ================== */
// 存储四个电机上一次的定时器计数值
static uint16_t last_count[4] = {0, 0, 0, 0};
// 存储四个电机的实时转速 (RPM)
static float current_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
// 存储供 CAN 上传的里程计脉冲累加值
static int16_t can_delta_ticks[4] = {0, 0, 0, 0};
/* ================== 函 数 实 现 ================== */

/**
  * @brief  启动所有的底层定时器通道
  */
void Motor_Init(void)
{
    // 1. 启动左侧电机 PWM (FL使用TIM2, RL使用TIM9)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // FL_IN1 (PA2)
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4); // FL_IN2 (PA3)
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); // RL_IN1 (PE5)
    HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); // RL_IN2 (PE6)

    // 2. 启动右侧电机 PWM (全部使用TIM8)
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); // FR_IN1 (PC6)
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2); // FR_IN2 (PC7)
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3); // RR_IN1 (PC8)
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4); // RR_IN2 (PC9)

    // 3. 启动所有编码器接口
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL); // 左上 FL (PA0, PA1)
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); // 左下 RL (PA6, PA7)
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 右上 FR (PD12, PD13)
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 右下 RR (PE9, PE11)
}

/**
  * @brief  将有符号的控制量转化为 AT8236 的双路输入占空比
  * @param  id: 电机编号 (MOTOR_FL, MOTOR_RL, MOTOR_FR, MOTOR_RR)
  * @param  control_out: 闭环算法的输出控制量 (-PWM_LIMIT ~ +PWM_LIMIT)
  */
void Set_Motor_Output(Motor_ID_t id, int16_t control_out)
{
    /* ======================================================================
     * 【物理镜像校准 - 输出层】
     * 现象：右侧电机正转（顺时针）时车体向前；左侧电机必须反转（逆时针）车体才向前。
     * 处理：当上层算法要求车体"向前"（即 control_out 为正）时，
     * 我们需要在底层强行把左侧电机的指令反相，让它驱动电机逆时针转。
     * ====================================================================== */
    if (id == MOTOR_FL || id == MOTOR_RL) {
        control_out = -control_out;
    }

    // 1. 限幅保护（防止算法计算溢出导致小车失控疯狂加速）
    if(control_out > PWM_LIMIT)  control_out = PWM_LIMIT;
    if(control_out < -PWM_LIMIT) control_out = -PWM_LIMIT;

    // 2. 解析正反转与实际需要设置的 PWM 数值
    // AT8236 逻辑: 正转 -> IN1 = PWM, IN2 = 0 ; 反转 -> IN1 = 0, IN2 = PWM
    uint16_t pwm_val = 0;
    uint8_t  dir = 0; // 1为正转，0为反转

    if(control_out >= 0) {
        dir = 0;
        pwm_val = control_out;
    } else {
        dir = 1;
        pwm_val = -control_out; // 取绝对值
    }

    // 3. 执行输出 (使用直接写寄存器宏，高频调用效率极高)
    switch(id)
    {
        case MOTOR_FL: // 左上: TIM2_CH3, TIM2_CH4
            if(dir) {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pwm_val);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
            } else {
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, pwm_val);
            }
            break;

        case MOTOR_RL: // 左下: TIM9_CH1, TIM9_CH2
            if(dir) {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, pwm_val);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, 0);
            } else {
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, pwm_val);
            }
            break;

        case MOTOR_FR: // 右上: TIM8_CH1, TIM8_CH2
            if(dir) {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, pwm_val);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, 0);
            } else {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, pwm_val);
            }
            break;

        case MOTOR_RR: // 右下: TIM8_CH3, TIM8_CH4
            if(dir) {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, pwm_val);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 0);
            } else {
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
                __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, pwm_val);
            }
            break;
    }
}

/**
  * @brief  紧急制动所有电机
  */
void Motor_Brake_All(void)
{
    Set_Motor_Output(MOTOR_FL, 0);
    Set_Motor_Output(MOTOR_RL, 0);
    Set_Motor_Output(MOTOR_FR, 0);
    Set_Motor_Output(MOTOR_RR, 0);
}

/**
  * @brief  读取编码器脉冲并计算实时转速 (RPM)
  * @param  dt_s: 两次调用此函数的时间间隔 (单位：秒)
  */
void Motor_Update_RPM(float dt_s)
{
    // 如果传入的时间间隔非法（比如0），直接返回防止除以0导致硬件错误
    if(dt_s <= 0.0f) return;

    // 1. 获取当前四个编码器定时器的原始计数值
    uint16_t curr_count_FL = __HAL_TIM_GET_COUNTER(&htim5);
    uint16_t curr_count_RL = __HAL_TIM_GET_COUNTER(&htim3);
    uint16_t curr_count_FR = __HAL_TIM_GET_COUNTER(&htim4);
    uint16_t curr_count_RR = __HAL_TIM_GET_COUNTER(&htim1);

    // 2. 计算脉冲增量 (int16_t 强转自动处理 0~65535 溢出跳变)
    int16_t delta[4];
    delta[MOTOR_FL] = (int16_t)(curr_count_FL - last_count[MOTOR_FL]);
    delta[MOTOR_RL] = (int16_t)(curr_count_RL - last_count[MOTOR_RL]);
    delta[MOTOR_FR] = (int16_t)(curr_count_FR - last_count[MOTOR_FR]);
    delta[MOTOR_RR] = (int16_t)(curr_count_RR - last_count[MOTOR_RR]);

    // 3. 更新上次计数值，为下一次计算做准备
    last_count[MOTOR_FL] = curr_count_FL;
    last_count[MOTOR_RL] = curr_count_RL;
    last_count[MOTOR_FR] = curr_count_FR;
    last_count[MOTOR_RR] = curr_count_RR;

    /* ======================================================================
     * 【物理镜像校准 - 输入层】
     * 现象：当车体整体向前移动时，右侧产生正脉冲，而左侧产生负脉冲。
     * 处理：为了让算法层统一认为 "向前走算出的 RPM 就是正数"，
     * 我们必须在这里将左侧电机的脉冲增量强制取反！
     * ====================================================================== */
    delta[MOTOR_FL] = -delta[MOTOR_FL];
    delta[MOTOR_RL] = -delta[MOTOR_RL];
    // 将校准后的增量累加进 CAN 发送缓冲区
    can_delta_ticks[MOTOR_FL] += delta[MOTOR_FL];
    can_delta_ticks[MOTOR_RL] += delta[MOTOR_RL];
    can_delta_ticks[MOTOR_FR] += delta[MOTOR_FR];
    can_delta_ticks[MOTOR_RR] += delta[MOTOR_RR];
    // 4. 计算转速 RPM (转/分钟)
    // 公式: (脉冲增量 / 一圈总脉冲) / 时间周期(秒) * 60 = RPM
    for(int i = 0; i < 4; i++)
    {
        current_rpm[i] = ((float)delta[i] / PULSES_PER_REV) / dt_s * 60.0f;
    }
}

/**
  * @brief  对外提供的统一数据读取接口
  * @param  id: 电机编号
  * @retval 实际物理转速
  */
float Get_Motor_RPM(Motor_ID_t id)
{
    return current_rpm[id];
}
/**
  * @brief  供 CAN 任务提取里程计增量，提取后自动清零
  */
void Motor_Get_And_Clear_Delta_Ticks(int16_t* d_fl, int16_t* d_rl, int16_t* d_fr, int16_t* d_rr)
{
    *d_fl = can_delta_ticks[MOTOR_FL];
    *d_rl = can_delta_ticks[MOTOR_RL];
    *d_fr = can_delta_ticks[MOTOR_FR];
    *d_rr = can_delta_ticks[MOTOR_RR];

    // 清零，为下一个周期重新累加
    can_delta_ticks[MOTOR_FL] = 0;
    can_delta_ticks[MOTOR_RL] = 0;
    can_delta_ticks[MOTOR_FR] = 0;
    can_delta_ticks[MOTOR_RR] = 0;
}