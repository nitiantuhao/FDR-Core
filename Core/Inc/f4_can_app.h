#ifndef __F4_CAN_APP_H
#define __F4_CAN_APP_H

#include "main.h"
#include "can.h"  // CubeMX生成的CAN头文件，提供 hcan1 句柄

// ---------------- 1. CAN ID 定义 (标准11位) ----------------
#define CAN_ID_EMCY_F4          0x010  // 紧急故障 (最高优先级)
#define CAN_ID_HEARTBEAT_H7     0x080  // 上位机心跳
#define CAN_ID_HEARTBEAT_F4     0x081  // 下位机心跳
#define CAN_ID_RXPDO_CTRL       0x100  // 控制下发 (H7 -> F4)
#define CAN_ID_TXPDO1_ODOM      0x200  // 里程计遥测1 (F4 -> H7)
#define CAN_ID_TXPDO2_LADRC     0x201  // LADRC状态2 (F4 -> H7)

// ---------------- 2. 机器人底盘物理参数 ----------------
// 【注意】请根据你实际测量的小车物理尺寸修改以下参数
#define ROBOT_TRACK_WIDTH_M     0.195f  // 左右轮距 23.6 cm (0.236 m)
#define ROBOT_WHEEL_RADIUS_M    0.040f // 车轮半径 (假设65mm轮子，即 0.0325 m)
#define ICR_COEFFICIENT         1.0f    // 瞬时旋转中心漂移系数 (默认1.0，根据场地摩擦力微调)
#define PI_VALUE                3.1415926535f

// ---------------- 3. 状态机枚举 ----------------
typedef enum {
  F4_STATE_FAULT = 0,     // 故障/急停状态
  F4_STATE_OPERATIONAL = 1 // 正常运行状态
} F4_SystemState_t;

// ---------------- 4. 数据解包联合体 (处理浮点数字节序) ----------------
typedef union {
  uint8_t bytes[8];
  struct {
    float target_vx; // 目标线速度 (m/s)
    float target_wz; // 目标角速度 (rad/s)
  } data;
} RxCtrlPayload_t;

// ---------------- 5. 全局变量与函数声明 ----------------
extern volatile F4_SystemState_t f4_fsm_state;
extern RxCtrlPayload_t rx_ctrl_cmd;

// 填平 CubeMX 坑的初始化函数
void F4_CAN_Filter_And_Start(void);

// 核心任务调度函数 (需放在 TIM6 10ms 中断内调用)
void F4_CAN_Task_10ms(void);

#endif /* __F4_CAN_APP_H */