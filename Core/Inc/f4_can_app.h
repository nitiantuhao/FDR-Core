#ifndef __F4_CAN_APP_H
#define __F4_CAN_APP_H

#include <stdint.h>

#include "main.h"
#include "can.h"

/*
 * 这个文件定义“上位机 CAN 协议 + 底盘状态机 + 故障/诊断上报”的公共接口。
 *
 * 设计原则：
 * 1) 10ms TIM6 中断只做硬实时内容：看门狗、速度解算、LADRC、故障判定。
 * 2) CAN 接收中断只解析帧，不做复杂浮点运算。
 * 3) 主循环只做低实时性的周期发送：状态帧、诊断帧、里程计。
 * 4) 运动安全只依赖“新鲜合法的 0x100 控制帧”，0x080 心跳不再允许底盘继续沿用旧速度运动。
 */

/* ================== 系统状态机定义 ================== */
typedef enum
{
    SYSTEM_BOOTING = 0,   /* 上电启动，尚未收到第一帧合法速度命令。 */
    SYSTEM_OPERATIONAL,   /* 正常工作，允许按目标速度运动。 */
    SYSTEM_SAFE_FAULT     /* 安全保护：目标速度强制清零，控制器内部状态清零，并禁止继续驱动。 */
} System_State_t;

/* ================== 健康等级定义 ================== */
typedef enum
{
    SYSTEM_HEALTH_OK = 0,        /* 当前没有活动中的告警/故障。 */
    SYSTEM_HEALTH_WARNING = 1,   /* 有警告，但系统还可继续工作。 */
    SYSTEM_HEALTH_FAULT = 2      /* 有明确故障，通常已经或应当进入保护。 */
} System_Health_t;

/* ================== 诊断位图定义 ================== */
typedef enum
{
    DIAG_COMM_TIMEOUT        = (1UL << 0),  /* 速度控制帧超时。属于故障。 */
    DIAG_CAN_BUS_OFF         = (1UL << 1),  /* CAN Bus-Off。属于故障。 */
    DIAG_CMD_CRC_STORM       = (1UL << 2),  /* 连续 CRC 错误过多。属于故障。 */
    DIAG_CMD_CNT_STORM       = (1UL << 3),  /* 连续 rolling counter 错误过多。属于故障。 */
    DIAG_MOTOR_FL_STALL      = (1UL << 4),  /* 左前轮堵转/失效趋势。属于故障。 */
    DIAG_MOTOR_RL_STALL      = (1UL << 5),  /* 左后轮堵转/失效趋势。属于故障。 */
    DIAG_MOTOR_FR_STALL      = (1UL << 6),  /* 右前轮堵转/失效趋势。属于故障。 */
    DIAG_MOTOR_RR_STALL      = (1UL << 7),  /* 右后轮堵转/失效趋势。属于故障。 */
    DIAG_CONTROL_SATURATION  = (1UL << 8)   /* 控制输出长时间顶满。属于警告。 */
} Chassis_DiagBits_t;

/*
 * 便于快速区分“只是提醒”还是“必须当故障处理”的位掩码。
 * 上位机解析时可以直接用这两个宏做分类。
 */
#define CHASSIS_DIAG_FATAL_MASK   (DIAG_COMM_TIMEOUT | DIAG_CAN_BUS_OFF | DIAG_CMD_CRC_STORM | \
                                   DIAG_CMD_CNT_STORM | DIAG_MOTOR_FL_STALL | DIAG_MOTOR_RL_STALL | \
                                   DIAG_MOTOR_FR_STALL | DIAG_MOTOR_RR_STALL)
#define CHASSIS_DIAG_WARN_MASK    (DIAG_CONTROL_SATURATION)

/* ================== 核心控制结构体 ================== */
typedef struct
{
    float target_vx;         /* 目标线速度，单位 m/s。 */
    float target_wz;         /* 目标角速度，单位 rad/s。 */
    uint8_t ctrl_flags;      /* 上位机附带的控制标志位，当前透传保存。 */
    uint8_t rolling_cnt;     /* 最近一次接受的速度控制帧 rolling counter。 */
    System_State_t state;    /* 当前系统状态机状态。 */
} Robot_Control_t;

/* ================== 上位机可读的状态快照 ================== */
typedef struct
{
    System_State_t state;      /* 当前状态机状态。 */
    System_Health_t health;    /* 当前健康等级。 */
    uint32_t diag_bits;        /* 当前活动中的诊断位图。 */
    uint8_t cmd_age_10ms;      /* 距最近一次合法 0x100 已过去多少个 10ms tick。 */
    uint8_t last_cmd_counter;  /* 最近一次接受的 rolling counter。 */
} Chassis_StatusSnapshot_t;

/*
 * g_robot_ctrl 会在 CAN 接收中断里更新，也会在控制中断里读取，
 * 因此这里声明为 volatile，避免编译器把它优化成寄存器缓存。
 */
extern volatile Robot_Control_t g_robot_ctrl;

/* ================== CAN 协议说明 ==================
 *
 * Rx: 0x080 心跳帧
 *     - 仅用于链路存在性，不参与运动看门狗
 *
 * Rx: 0x100 速度控制帧（8 字节）
 *     Byte0~1 : int16_t vx * 1000, 单位 m/s，小端
 *     Byte2~3 : int16_t wz * 1000, 单位 rad/s，小端
 *     Byte4   : ctrl_flags
 *     Byte5   : 预留
 *     Byte6   : rolling counter
 *     Byte7   : CRC8-SAE J1850(前 7 字节)
 *
 * Tx: 0x181 状态帧（20ms）
 *     Byte0   : system_state
 *     Byte1   : system_health
 *     Byte2~5 : diag_bits, uint32 little-endian
 *     Byte6   : cmd_age_10ms
 *     Byte7   : status rolling counter
 *
 * Tx: 0x182 实际轮速帧（轮询，约 60ms）
 *     Byte0~1 : FL 实际 RPM, int16 little-endian
 *     Byte2~3 : RL 实际 RPM
 *     Byte4~5 : FR 实际 RPM
 *     Byte6~7 : RR 实际 RPM
 *
 * Tx: 0x183 目标轮速帧（轮询，约 60ms）
 *     Byte0~1 : FL 目标 RPM, int16 little-endian
 *     Byte2~3 : RL 目标 RPM
 *     Byte4~5 : FR 目标 RPM
 *     Byte6~7 : RR 目标 RPM
 *
 * Tx: 0x184 通信诊断帧（100ms）
 *     Byte0   : valid_cmd_total 低 8 位
 *     Byte1   : crc_error_total 低 8 位
 *     Byte2   : counter_reject_total 低 8 位
 *     Byte3   : can_tx_drop_total 低 8 位
 *     Byte4   : busoff_total 低 8 位
 *     Byte5   : rx_overrun_total 低 8 位
 *     Byte6   : last accepted rolling counter
 *     Byte7   : [7:4] 连续 counter 错误计数(饱和到15), [3:0] 连续 CRC 错误计数(饱和到15)
 *
 * Tx: 0x200 里程计增量帧（轮询，约 60ms）
 *     Byte0~1 : FL delta ticks, int16 little-endian
 *     Byte2~3 : RL delta ticks
 *     Byte4~5 : FR delta ticks
 *     Byte6~7 : RR delta ticks
 */

/* ================== 对外接口 ================== */

/*
 * 初始化 CAN 应用层：
 * - 配置硬件滤波器，仅接收 0x080 / 0x100
 * - 启动 CAN 外设
 * - 打开 FIFO0 接收中断
 * - 清空通信诊断状态
 */
void CAN_App_Init(void);

/*
 * 10ms 周期调用：
 * - 维护速度控制命令看门狗
 * - 合法控制帧超时则进入 SAFE_FAULT
 */
void CAN_Safety_Watchdog_Tick(void);

/*
 * 10ms 周期调用：
 * - 从最近一次完整一致的目标命令快照进行差速解算
 * - 在 SAFE_FAULT / BOOTING 下强制目标轮速为 0，并禁止继续输出驱动
 */
void Kinematics_Update_LADRC(void);

/*
 * 10ms 周期调用：
 * - 根据目标 RPM、实际 RPM、控制输出做堵转/饱和诊断
 * - 该函数应放在 LADRC 控制环执行之后，以便读取最新反馈
 */
void Chassis_Diagnostics_10ms_Tick(void);

/*
 * 20ms 周期调用：
 * - 发送 0x181 状态帧
 * - 发送 0x182 实际轮速帧
 * - 发送 0x183 目标轮速帧
 * - 发送 0x200 里程计帧
 * - 内部每 5 次额外发送 1 次 0x184 通信诊断帧
 */
void CAN_Send_Telemetry_20ms(void);

/*
 * 获取当前快照，便于未来扩展到 USB/串口调试打印或上位机测试。
 */
void Chassis_Get_StatusSnapshot(Chassis_StatusSnapshot_t *snapshot);

#endif
