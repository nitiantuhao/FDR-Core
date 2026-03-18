#include "ladrc.h"
#include "f4_can_app.h"

/*
 * 这个文件实现两层内容：
 * 1) 单个电机的 LADRC 控制器。
 * 2) 四轮底盘的调度层（读取编码器 -> 滤波 -> LADRC -> PWM 输出）。
 *
 * 额外升级点：
 * - 提供目标 RPM / 控制输出 getter，供上层故障诊断与状态上报使用。
 */

#define LADRC_CONTROL_DT_S       0.01f
#define LADRC_RPM_FILTER_ALPHA   0.30f

#define LADRC_DEFAULT_WC         25.0f
#define LADRC_DEFAULT_WO         60.0f
#define LADRC_DEFAULT_B0         0.20f
#define LADRC_DEFAULT_OUT_MAX    1000.0f

/* ================== 内部辅助函数 ================== */
static float LADRC_Abs(float x);
static float LADRC_Clamp(float x, float min_value, float max_value);
static void LADRC_ResetStates(LADRC_TypeDef *ladrc);

static float s_filtered_rpm[4] = {0.0f, 0.0f, 0.0f, 0.0f};

static float LADRC_Abs(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float LADRC_Clamp(float x, float min_value, float max_value)
{
    if (x > max_value)
    {
        return max_value;
    }
    if (x < min_value)
    {
        return min_value;
    }
    return x;
}

static void LADRC_ResetStates(LADRC_TypeDef *ladrc)
{
    if (ladrc == NULL)
    {
        return;
    }

    ladrc->v = 0.0f;
    ladrc->y = 0.0f;
    ladrc->z1 = 0.0f;
    ladrc->z2 = 0.0f;
    ladrc->u = 0.0f;
}

/**
 * @brief  初始化单个 LADRC 控制器。
 * @note   顺手做基础参数保护，避免 b0/h/out_max 为 0 或负值时出问题。
 */
void LADRC_Init(LADRC_TypeDef *ladrc, float wc, float wo, float b0, float h, float max)
{
    if (ladrc == NULL)
    {
        return;
    }

    if (h <= 0.0f)
    {
        h = LADRC_CONTROL_DT_S;
    }
    if (LADRC_Abs(b0) < 1e-6f)
    {
        b0 = LADRC_DEFAULT_B0;
    }
    if (wc < 0.0f)
    {
        wc = -wc;
    }
    if (wo < 0.0f)
    {
        wo = -wo;
    }
    if (max <= 0.0f)
    {
        max = LADRC_DEFAULT_OUT_MAX;
    }

    ladrc->v = 0.0f;   /* 目标值 */
    ladrc->y = 0.0f;   /* 测量值 */
    ladrc->z1 = 0.0f;  /* 状态观测量 */
    ladrc->z2 = 0.0f;  /* 总扰动观测量 */
    ladrc->u = 0.0f;   /* 上一拍控制输出 */

    ladrc->wc = wc;
    ladrc->wo = wo;
    ladrc->b0 = b0;
    ladrc->h = h;
    ladrc->out_max = max;

    ladrc->beta1 = 2.0f * wo;
    ladrc->beta2 = wo * wo;
}

/**
 * @brief  单个 LADRC 周期运算。
 * @param  ladrc      控制器实例
 * @param  actual_val 当前测量速度（RPM）
 * @retval 限幅后的控制输出（通常可直接映射到 PWM）
 */
float LADRC_Calc(LADRC_TypeDef *ladrc, float actual_val)
{
    float e;
    float u0;
    float out;

    if (ladrc == NULL)
    {
        return 0.0f;
    }

    ladrc->y = actual_val;

    /* 第 1 部分：LESO（线性扩张状态观测器） */
    e = ladrc->y - ladrc->z1;
    ladrc->z1 += (ladrc->z2 + ladrc->b0 * ladrc->u + ladrc->beta1 * e) * ladrc->h;
    ladrc->z2 += (ladrc->beta2 * e) * ladrc->h;

    /* 第 2 部分：LSEF + 扰动补偿 */
    u0 = ladrc->wc * (ladrc->v - ladrc->z1);
    out = (u0 - ladrc->z2) / ladrc->b0;

    /* 第 3 部分：输出限幅，保护电机和驱动 */
    out = LADRC_Clamp(out, -ladrc->out_max, ladrc->out_max);

    ladrc->u = out;
    return out;
}

/* =====================================================================
 * 针对四轮底盘的 LADRC 调度层
 * ===================================================================== */

LADRC_TypeDef ladrc_motors[4];

/**
  * @brief  一键初始化底层硬件和四个 LADRC 控制器。
  */
void FourWheel_LADRC_Init(void)
{
    int i;

    Motor_Init();

    for (i = 0; i < 4; ++i)
    {
        LADRC_Init(&ladrc_motors[i],
                   LADRC_DEFAULT_WC,
                   LADRC_DEFAULT_WO,
                   LADRC_DEFAULT_B0,
                   LADRC_CONTROL_DT_S,
                   LADRC_DEFAULT_OUT_MAX);
        s_filtered_rpm[i] = 0.0f;
    }
}

void FourWheel_LADRC_ResetAll(void)
{
    int i;

    for (i = 0; i < 4; ++i)
    {
        LADRC_ResetStates(&ladrc_motors[i]);
        s_filtered_rpm[i] = 0.0f;
    }

    Motor_Brake_All();
}

/**
  * @brief  设置四个轮子的目标 RPM。
  */
void FourWheel_Set_Target_RPM(float fl_rpm, float rl_rpm, float fr_rpm, float rr_rpm)
{
    ladrc_motors[MOTOR_FL].v = fl_rpm;
    ladrc_motors[MOTOR_RL].v = rl_rpm;
    ladrc_motors[MOTOR_FR].v = fr_rpm;
    ladrc_motors[MOTOR_RR].v = rr_rpm;
}

/**
  * @brief  四轮 LADRC 大循环，建议固定 10ms 调用一次。
  * @note   调度顺序：
  *         1) 更新编码器测速
  *         2) 一阶低通滤波，减小测速毛刺
  *         3) 每个轮子独立做 LADRC
  *         4) 输出到 H 桥
  */
void FourWheel_LADRC_Control_Loop(void)
{
    int i;

    Motor_Update_RPM(LADRC_CONTROL_DT_S);

    if (g_robot_ctrl.state != SYSTEM_OPERATIONAL)
    {
        FourWheel_LADRC_ResetAll();
        return;
    }

    for (i = 0; i < 4; ++i)
    {
        float raw_rpm = Get_Motor_RPM((Motor_ID_t)i);
        float pwm_out;

        s_filtered_rpm[i] = (1.0f - LADRC_RPM_FILTER_ALPHA) * s_filtered_rpm[i]
                          + LADRC_RPM_FILTER_ALPHA * raw_rpm;

        pwm_out = LADRC_Calc(&ladrc_motors[i], s_filtered_rpm[i]);
        Set_Motor_Output((Motor_ID_t)i, (int16_t)pwm_out);
    }
}

/**
  * @brief  读取指定轮子的“当前目标 RPM”。
  * @note   该接口主要供上层做状态上报与故障诊断使用。
  */
float FourWheel_Get_Target_RPM(Motor_ID_t id)
{
    if ((id < MOTOR_FL) || (id > MOTOR_RR))
    {
        return 0.0f;
    }

    return ladrc_motors[id].v;
}

/**
  * @brief  读取指定轮子的“当前控制输出”。
  * @note   该值尚未经过电机左右镜像翻转，但足够用于判断是否长期顶满。
  */
float FourWheel_Get_Control_Output(Motor_ID_t id)
{
    if ((id < MOTOR_FL) || (id > MOTOR_RR))
    {
        return 0.0f;
    }

    return ladrc_motors[id].u;
}
