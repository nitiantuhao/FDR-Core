#include "f4_can_app.h"
#include <stdio.h>
#include "motor.h"      // 引入电机底层，以获取里程计增量
#include "ladrc.h"      // 引入 LADRC 层，以下发控制目标

// ---------------- 全局变量定义 ----------------
volatile uint32_t can_survival_timer = 0;      // 存活计时器 (单位：10ms)
volatile F4_SystemState_t f4_fsm_state = F4_STATE_FAULT; // 默认启动为故障保护状态
RxCtrlPayload_t rx_ctrl_cmd = {0};             // 存放解析后的控制指令

static uint32_t timer_10ms_tick = 0;           // 10ms 任务节拍器

// ---------------- 1. 补充 CubeMX 没做的 CAN 初始化 ----------------
void F4_CAN_Filter_And_Start(void)
{
    CAN_FilterTypeDef canFilterConfig;

    canFilterConfig.FilterBank = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000;
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(&hcan1, &canFilterConfig);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

// ---------------- 2. CAN 接收中断回调函数 ----------------
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (hcan->Instance == CAN1) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {

            // 【安全机制】：收到 H7 心跳或控制帧，喂狗 [cite: 20]
            if (RxHeader.StdId == CAN_ID_HEARTBEAT_H7 || RxHeader.StdId == CAN_ID_RXPDO_CTRL) {
                can_survival_timer = 0;
                f4_fsm_state = F4_STATE_OPERATIONAL;
            }

            // 【业务逻辑】：解析控制下发 (ID: 0x100) [cite: 14]
            if (RxHeader.StdId == CAN_ID_RXPDO_CTRL && RxHeader.DLC == 8) {
                for(int i = 0; i < 8; i++) {
                    rx_ctrl_cmd.bytes[i] = RxData[i];
                }
            }
        }
    }
}

// ---------------- 3. 内部数据发送函数：增量里程计 ----------------
static void CAN_Send_TxPDO1_Odom_Delta(int16_t d_tick1, int16_t d_tick2, int16_t d_tick3, int16_t d_tick4)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[8];

    TxHeader.StdId = CAN_ID_TXPDO1_ODOM; // 0x200 [cite: 14]
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = (uint8_t)(d_tick1 & 0xFF);
    TxData[1] = (uint8_t)((d_tick1 >> 8) & 0xFF);
    TxData[2] = (uint8_t)(d_tick2 & 0xFF);
    TxData[3] = (uint8_t)((d_tick2 >> 8) & 0xFF);
    TxData[4] = (uint8_t)(d_tick3 & 0xFF);
    TxData[5] = (uint8_t)((d_tick3 >> 8) & 0xFF);
    TxData[6] = (uint8_t)(d_tick4 & 0xFF);
    TxData[7] = (uint8_t)((d_tick4 >> 8) & 0xFF);

    // 【防死锁修复】：检查邮箱是否已满。若总线断开导致发送失败，强行清空邮箱防止程序卡死
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// ---------------- 4. 内部数据发送函数：下位机心跳 ----------------
static void CAN_Send_Heartbeat(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    uint8_t TxData[1];

    TxHeader.StdId = CAN_ID_HEARTBEAT_F4; // 0x081 [cite: 14]
    TxHeader.ExtId = 0;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;

    TxData[0] = (uint8_t)f4_fsm_state;

    // 【防死锁修复】
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    }
    HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
}

// ---------------- 5. 核心调度任务 (每10ms由TIM6中断调用一次) ----------------
void F4_CAN_Task_10ms(void)
{
    timer_10ms_tick++;
    can_survival_timer++;

    // ================= 步骤A：急停安全判定 (150ms) =================
    if (can_survival_timer > 15) { // [cite: 21]
        f4_fsm_state = F4_STATE_FAULT;
        FourWheel_Set_Target_RPM(0.0f, 0.0f, 0.0f, 0.0f);
    }
    // ================= 步骤B：正常运行逆运动学解析 =================
    else {
        // 【防数据撕裂修复】：短暂关闭全局中断，防止读取联合体浮点数时被 CAN 接收中断打断
        __disable_irq();
        float vx = rx_ctrl_cmd.data.target_vx;
        float wz = rx_ctrl_cmd.data.target_wz;
        __enable_irq();

        float v_left = vx - wz * (ICR_COEFFICIENT * ROBOT_TRACK_WIDTH_M / 2.0f);
        float v_right = vx + wz * (ICR_COEFFICIENT * ROBOT_TRACK_WIDTH_M / 2.0f);

        float rpm_left = (v_left / (2.0f * PI_VALUE * ROBOT_WHEEL_RADIUS_M)) * 60.0f;
        float rpm_right = (v_right / (2.0f * PI_VALUE * ROBOT_WHEEL_RADIUS_M)) * 60.0f;

        FourWheel_Set_Target_RPM(rpm_left, rpm_left, rpm_right, rpm_right);
    }

    // ================= 步骤C：CAN 数据定期上报 =================
    if (timer_10ms_tick % 2 == 0) { // 20ms [cite: 15]
        int16_t d_fl, d_rl, d_fr, d_rr;
        Motor_Get_And_Clear_Delta_Ticks(&d_fl, &d_rl, &d_fr, &d_rr);
        CAN_Send_TxPDO1_Odom_Delta(d_fl, d_rl, d_fr, d_rr);
    }

    if (timer_10ms_tick % 5 == 0) { // 50ms [cite: 14]
        CAN_Send_Heartbeat();
    }
}