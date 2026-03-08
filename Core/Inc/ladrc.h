#ifndef __LADRC_H
#define __LADRC_H

#include <stdint.h>
#include "motor.h"   // 引入底层电机库以使用 Motor_ID_t 宏

/* LADRC 结构体定义 --------------------------------------------------------*/
typedef struct {
  float v;          // 目标速度 (Target)
  float y;          // 实际速度 (Actual)

  // LESO (线性扩展状态观测器) 状态变量
  float z1;         // 系统输出的估计值 (估算的实际速度)
  float z2;         // 总扰动的估计值 (包含所有未知的阻力和模型误差)

  // 控制器核心参数 (仅需调这 3 个)
  float wc;         // 控制器带宽 (Controller Bandwidth)
  float wo;         // 观测器带宽 (Observer Bandwidth)
  float b0;         // 系统的控制增益 (非常关键，决定了 PWM 到速度的转换系数)

  // LESO 内部增益
  float beta1;      // 观测器增益 1
  float beta2;      // 观测器增益 2

  float h;          // 采样周期 (单位：秒，比如 10ms 就是 0.01)
  float out_max;    // 输出限幅 (如 1000)

  float u;          // 最终计算出的控制量 (PWM)
} LADRC_TypeDef;

/* 原有算法函数声明 --------------------------------------------------------*/
void LADRC_Init(LADRC_TypeDef *ladrc, float wc, float wo, float b0, float h, float max);
float LADRC_Calc(LADRC_TypeDef *ladrc, float actual_val);

/* ================= 针对四轮底盘的 LADRC 扩展调度层 ================= */

// 将 4 个控制器暴露出来，方便 main.c 里面调取数据用 VOFA+ 打印波形
extern LADRC_TypeDef ladrc_motors[4]; 

// 1. 初始化：一键初始化底层定时器和4个LADRC控制器
void FourWheel_LADRC_Init(void);

// 2. 设定目标转速：给四个轮子下发指令
void FourWheel_Set_Target_RPM(float fl_rpm, float rl_rpm, float fr_rpm, float rr_rpm);

// 3. 核心闭环运算：必须且只能在 10ms 基础定时器中断里调用
void FourWheel_LADRC_Control_Loop(void);

#endif /* __LADRC_H */