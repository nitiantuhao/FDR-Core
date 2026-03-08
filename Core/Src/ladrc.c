#include "ladrc.h"

/**
 * @brief  初始化单体 LADRC 参数
 */
void LADRC_Init(LADRC_TypeDef *ladrc, float wc, float wo, float b0, float h, float max) {
    // 状态初始化
    ladrc->v = 0.0f;
    ladrc->y = 0.0f;
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->u = 0.0f;
    
    // 参数赋值
    ladrc->wc = wc;
    ladrc->wo = wo;
    ladrc->b0 = b0;
    ladrc->h = h;
    ladrc->out_max = max;
    
    // 计算观测器增益
    ladrc->beta1 = 2.0f * wo;
    ladrc->beta2 = wo * wo;
}

/**
 * @brief  单体 LADRC 核心运算
 */
float LADRC_Calc(LADRC_TypeDef *ladrc, float actual_val) {
    ladrc->y = actual_val;
    
    // 第一部分：LESO
    float e = ladrc->y - ladrc->z1; 
    ladrc->z1 += (ladrc->z2 + ladrc->b0 * ladrc->u + ladrc->beta1 * e) * ladrc->h;
    ladrc->z2 += (ladrc->beta2 * e) * ladrc->h;
    
    // 第二部分：LSEF & 扰动补偿
    float u0 = ladrc->wc * (ladrc->v - ladrc->z1);
    float out = (u0 - ladrc->z2) / ladrc->b0;
    
    // 第三部分：输出限幅保护
    if (out > ladrc->out_max) out = ladrc->out_max;
    if (out < -ladrc->out_max) out = -ladrc->out_max;
    
    ladrc->u = out; 
    
    return out;
}


/* =====================================================================
 * 针对四轮底盘的 LADRC 扩展调度层
 * ===================================================================== */

// 实例化 4 个控制器的实体
LADRC_TypeDef ladrc_motors[4];

/**
  * @brief 一键初始化底层硬件与四个 LADRC
  */
void FourWheel_LADRC_Init(void)
{
    // 1. 唤醒底层电机与编码器硬件
    Motor_Init();

    // 2. 初始化 4 个控制器的参数 (注意这里的限幅是 1000，匹配底层的 PWM_LIMIT)
    for(int i = 0; i < 4; i++) {
        LADRC_Init(&ladrc_motors[i], 25.0f, 60.0f, 0.2f, 0.01f, 1000.0f);
    }
}

/**
  * @brief 一键下发四个轮子的目标转速
  */
void FourWheel_Set_Target_RPM(float fl_rpm, float rl_rpm, float fr_rpm, float rr_rpm)
{
    ladrc_motors[MOTOR_FL].v = fl_rpm;
    ladrc_motors[MOTOR_RL].v = rl_rpm;
    ladrc_motors[MOTOR_FR].v = fr_rpm;
    ladrc_motors[MOTOR_RR].v = rr_rpm;
}

/**
  * @brief 四轮闭环大循环 (强烈建议放在 10ms 的定时器中断内执行)
  */
void FourWheel_LADRC_Control_Loop(void)
{
    Motor_Update_RPM(0.01f);

    // 静态数组，用来记住上一次的平滑速度
    static float filtered_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    // 滤波系数 alpha (0.0 ~ 1.0)。越小越平滑，越大越敏捷。
    // 0.3 的意思是：相信 30% 的最新数据，保留 70% 的历史数据
    const float alpha = 0.3f;

    for(int i = 0; i < 4; i++)
    {
        // 1. 拿到底层的原始狂野转速
        float raw_rpm = Get_Motor_RPM((Motor_ID_t)i);

        // 2. 软件减震器：一阶低通滤波！
        filtered_rpm[i] = (1.0f - alpha) * filtered_rpm[i] + alpha * raw_rpm;

        // 3. 把过滤掉毛刺、极其顺滑的速度扔给 LADRC
        float pwm_out = LADRC_Calc(&ladrc_motors[i], filtered_rpm[i]);

        Set_Motor_Output((Motor_ID_t)i, (int16_t)pwm_out);
    }
}