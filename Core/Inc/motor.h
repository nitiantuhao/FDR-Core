#ifndef __MOTOR_H
#define __MOTOR_H

#include "main.h"

/* ================== 硬件与物理参数配置 ================== */
// PWM 满载占空比 (对应 CubeMX 中的 ARR = 1049)
#define MAX_PWM_DUTY 1049

// 为安全起见设置占空比上限，防溢出或满载锁死 (约 95%)
#define PWM_LIMIT    1000

// 编码器与减速箱参数 (JGB37-520 11线 90减速比)
#define ENCODER_RESOLUTION  11.0f   // 电机尾部编码器线数
#define REDUCTION_RATIO     90.0f   // 减速箱减速比 (90:1)
// 车轮转一圈的总脉冲数 = 11 * 4(倍频) * 90 = 3960
#define PULSES_PER_REV      (ENCODER_RESOLUTION * 4.0f * REDUCTION_RATIO)

/* ================== 枚举与类型定义 ================== */
// 定义电机 ID (与物理位置一一对应)
typedef enum {
  MOTOR_FL = 0, // 左上 Front-Left  (PWM: TIM2, Encoder: TIM5)
  MOTOR_RL,     // 左下 Rear-Left   (PWM: TIM9, Encoder: TIM3)
  MOTOR_FR,     // 右上 Front-Right (PWM: TIM8, Encoder: TIM4)
  MOTOR_RR      // 右下 Rear-Right  (PWM: TIM8, Encoder: TIM1)
} Motor_ID_t;

/* ================== 函 数 声 明 ================== */
// --- 基础控制层 ---
void Motor_Init(void);
// 设置电机输出 (正数代表期望车轮推动车体【向前】，负数【向后】)
void Set_Motor_Output(Motor_ID_t id, int16_t control_out);
// 紧急制动所有电机
void Motor_Brake_All(void);

// --- 状态观测层 ---
// 读取脉冲并更新转速 (需周期性调用，dt_s 为调用间隔的秒数，如 10ms 则传入 0.01f)
void Motor_Update_RPM(float dt_s);
// 获取计算好的实时转速 (正数代表车轮正使车体【向前】行驶)
float Get_Motor_RPM(Motor_ID_t id);
// 获取自上次调用以来的累加脉冲增量，并自动清零
void Motor_Get_And_Clear_Delta_Ticks(int16_t* d_fl, int16_t* d_rl, int16_t* d_fr, int16_t* d_rr);

#endif /* __MOTOR_H */