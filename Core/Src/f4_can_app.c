#include "f4_can_app.h"

#include <stdbool.h>
#include <string.h>

#include "ladrc.h"
#include "motor.h"
volatile uint32_t g_dbg_rx_100_hits = 0U;
volatile uint32_t g_dbg_dlc_reject = 0U;
volatile uint32_t g_dbg_crc_reject = 0U;
volatile uint32_t g_dbg_cnt_reject = 0U;
volatile uint32_t g_dbg_valid_accept = 0U;
/*
 * 下面两个 getter 由 ladrc.c 提供，用于诊断层读取每个轮子的目标值与控制输出。
 * 之所以在这里做前向声明，是为了尽量少改你原工程里现有的 ladrc.h。
 */


/* ================= 机器人底盘物理参数 ================= */
#define WHEEL_RADIUS_M                    0.06f
#define WHEEL_TRACK_M                     0.30f
#define PI_F                              3.14159265358979323846f

/* ================= CAN 协议参数 ================= */
#define CAN_ID_HEARTBEAT                  0x080U
#define CAN_ID_CMD_VEL                    0x100U
#define CAN_ID_STATUS                     0x181U
#define CAN_ID_ACTUAL_RPM                 0x182U
#define CAN_ID_TARGET_RPM                 0x183U
#define CAN_ID_COMM_DIAG                  0x184U
#define CAN_ID_ODOM                       0x200U

#define CONTROL_LOOP_PERIOD_MS            10U
#define CMD_TIMEOUT_MS                    150U
#define MAX_ACCEPTED_COUNTER_GAP          3U
#define TELEMETRY_COMM_DIAG_DIV           5U   /* 20ms * 5 = 100ms */
#define CAN_RX_ISR_MAX_FRAMES_PER_CALL      2U

/* ================= 通信故障判定参数 ================= */
#define CRC_STORM_THRESHOLD               5U
#define COUNTER_STORM_THRESHOLD           5U

/* ================= 执行机构诊断参数 ================= */
#define STALL_TARGET_RPM_MIN              40.0f
#define STALL_FEEDBACK_RPM_MAX            8.0f
#define STALL_OUTPUT_ABS_MIN              850.0f
#define STALL_CONFIRM_TICKS               50U   /* 50 * 10ms = 500ms */

#define SATURATION_TARGET_RPM_MIN         30.0f
#define SATURATION_OUTPUT_ABS_MIN         980.0f
#define SATURATION_CONFIRM_TICKS          20U   /* 20 * 10ms = 200ms */

#define CAN_TX_SHORT_WAIT_MS              2U

volatile Robot_Control_t g_robot_ctrl = {
    .target_vx = 0.0f,
    .target_wz = 0.0f,
    .ctrl_flags = 0U,
    .rolling_cnt = 0U,
    .state = SYSTEM_BOOTING
};

/*
 * 只用“合法且新鲜”的 0x100 控制帧喂运动看门狗。
 * 这样即使上位机还在发心跳，但速度命令停了，底盘也会在超时后安全停车。
 */
static volatile uint32_t s_last_valid_cmd_tick_ms = 0U;
static volatile uint8_t s_cmd_age_10ms = 0U;
static volatile bool s_have_seen_valid_cmd = false;

/* 最近一次接受的 rolling counter，同步状态用于首帧/故障恢复。 */
static volatile uint8_t s_last_rx_counter = 0U;
static volatile bool s_counter_synced = false;

/* 当前活动中的诊断位图。 */
static volatile uint32_t s_diag_bits = 0U;

/* 通信统计计数器：全部为累加值，方便上位机做趋势诊断。 */
static volatile uint32_t s_valid_cmd_total = 0U;
static volatile uint32_t s_crc_error_total = 0U;
static volatile uint32_t s_counter_reject_total = 0U;
static volatile uint32_t s_can_tx_drop_total = 0U;
static volatile uint32_t s_busoff_total = 0U;
static volatile uint32_t s_rx_overrun_total = 0U;

/* 连续错误计数，用于判断“风暴/持续异常”。 */
static volatile uint8_t s_consecutive_crc_errors = 0U;
static volatile uint8_t s_consecutive_counter_errors = 0U;

/* 轮子堵转与控制饱和诊断的确认计数器。 */
static uint16_t s_stall_ticks[4] = {0U, 0U, 0U, 0U};
static uint16_t s_saturation_ticks = 0U;

/* 周期发送辅助计数器。 */
static uint8_t s_status_tx_counter = 0U;
static uint8_t s_comm_diag_divider = 0U;

/* ================= 内部辅助函数声明 ================= */
static inline uint8_t CAN_CRC8_SAE_J1850(const uint8_t *data, uint16_t length);
static int16_t CAN_Read_I16_LE(uint8_t low_byte, uint8_t high_byte);
static void CAN_Write_I16_LE(uint8_t *dst, int16_t value);
static int16_t CAN_SaturateFloatToI16(float value);
static float CAN_AbsFloat(float value);
static void CAN_Config_StdDataFilter(uint8_t filter_bank, uint16_t std_id);
static void CAN_SetDiagBits(uint32_t bits);
static void CAN_ClearDiagBits(uint32_t bits);
static void CAN_AtomicIncU32(volatile uint32_t *value);
static void CAN_AtomicIncU8Saturated(volatile uint8_t *value, uint8_t max_value);
static void CAN_ResetMotionCommand(void);
static void CAN_EnterSafeFault(uint32_t reason_bits, bool desync_counter);
static void CAN_ProcessHeartbeat(const uint8_t *rx_data, uint8_t dlc);
static void CAN_ProcessControlFrame(const uint8_t *rx_data, uint8_t dlc);
static System_Health_t CAN_GetCurrentHealth(uint32_t diag_bits, System_State_t state);
static HAL_StatusTypeDef CAN_TxStdFrame(uint16_t std_id, const uint8_t *payload, uint8_t dlc);
static void CAN_Send_StatusFrame(void);
static void CAN_Send_ActualRpmFrame(void);
static void CAN_Send_TargetRpmFrame(void);
static void CAN_Send_CommDiagFrame(void);
static void CAN_Send_OdomFrame(void);

/* =====================================================================
 * 基础辅助函数
 * ===================================================================== */

/*
 * CRC8-SAE J1850（poly=0x1D, init=0xFF, xorout=0xFF, 非反射）
 *
 * 说明：
 * - 上一版为了“先把功能修对”，用了按位循环的通用实现；
 *   但这会在 CAN Rx ISR 中产生 7*8=56 次 bit 循环，属于不必要的开销。
 * - 这里改成查表版：每字节 1 次查表 + XOR，满足“ISR 内尽量不做重计算”的工程规范。
 * - 表是静态常量，放在 Flash，不占用 RAM。
 */
static const uint8_t s_crc8_j1850_table[256] = {
    0x00U, 0x1DU, 0x3AU, 0x27U, 0x74U, 0x69U, 0x4EU, 0x53U, 0xE8U, 0xF5U, 0xD2U, 0xCFU, 0x9CU, 0x81U, 0xA6U, 0xBBU,
    0xCDU, 0xD0U, 0xF7U, 0xEAU, 0xB9U, 0xA4U, 0x83U, 0x9EU, 0x25U, 0x38U, 0x1FU, 0x02U, 0x51U, 0x4CU, 0x6BU, 0x76U,
    0x87U, 0x9AU, 0xBDU, 0xA0U, 0xF3U, 0xEEU, 0xC9U, 0xD4U, 0x6FU, 0x72U, 0x55U, 0x48U, 0x1BU, 0x06U, 0x21U, 0x3CU,
    0x4AU, 0x57U, 0x70U, 0x6DU, 0x3EU, 0x23U, 0x04U, 0x19U, 0xA2U, 0xBFU, 0x98U, 0x85U, 0xD6U, 0xCBU, 0xECU, 0xF1U,
    0x13U, 0x0EU, 0x29U, 0x34U, 0x67U, 0x7AU, 0x5DU, 0x40U, 0xFBU, 0xE6U, 0xC1U, 0xDCU, 0x8FU, 0x92U, 0xB5U, 0xA8U,
    0xDEU, 0xC3U, 0xE4U, 0xF9U, 0xAAU, 0xB7U, 0x90U, 0x8DU, 0x36U, 0x2BU, 0x0CU, 0x11U, 0x42U, 0x5FU, 0x78U, 0x65U,
    0x94U, 0x89U, 0xAEU, 0xB3U, 0xE0U, 0xFDU, 0xDAU, 0xC7U, 0x7CU, 0x61U, 0x46U, 0x5BU, 0x08U, 0x15U, 0x32U, 0x2FU,
    0x59U, 0x44U, 0x63U, 0x7EU, 0x2DU, 0x30U, 0x17U, 0x0AU, 0xB1U, 0xACU, 0x8BU, 0x96U, 0xC5U, 0xD8U, 0xFFU, 0xE2U,
    0x26U, 0x3BU, 0x1CU, 0x01U, 0x52U, 0x4FU, 0x68U, 0x75U, 0xCEU, 0xD3U, 0xF4U, 0xE9U, 0xBAU, 0xA7U, 0x80U, 0x9DU,
    0xEBU, 0xF6U, 0xD1U, 0xCCU, 0x9FU, 0x82U, 0xA5U, 0xB8U, 0x03U, 0x1EU, 0x39U, 0x24U, 0x77U, 0x6AU, 0x4DU, 0x50U,
    0xA1U, 0xBCU, 0x9BU, 0x86U, 0xD5U, 0xC8U, 0xEFU, 0xF2U, 0x49U, 0x54U, 0x73U, 0x6EU, 0x3DU, 0x20U, 0x07U, 0x1AU,
    0x6CU, 0x71U, 0x56U, 0x4BU, 0x18U, 0x05U, 0x22U, 0x3FU, 0x84U, 0x99U, 0xBEU, 0xA3U, 0xF0U, 0xEDU, 0xCAU, 0xD7U,
    0x35U, 0x28U, 0x0FU, 0x12U, 0x41U, 0x5CU, 0x7BU, 0x66U, 0xDDU, 0xC0U, 0xE7U, 0xFAU, 0xA9U, 0xB4U, 0x93U, 0x8EU,
    0xF8U, 0xE5U, 0xC2U, 0xDFU, 0x8CU, 0x91U, 0xB6U, 0xABU, 0x10U, 0x0DU, 0x2AU, 0x37U, 0x64U, 0x79U, 0x5EU, 0x43U,
    0xB2U, 0xAFU, 0x88U, 0x95U, 0xC6U, 0xDBU, 0xFCU, 0xE1U, 0x5AU, 0x47U, 0x60U, 0x7DU, 0x2EU, 0x33U, 0x14U, 0x09U,
    0x7FU, 0x62U, 0x45U, 0x58U, 0x0BU, 0x16U, 0x31U, 0x2CU, 0x97U, 0x8AU, 0xADU, 0xB0U, 0xE3U, 0xFEU, 0xD9U, 0xC4U,
};

static inline uint8_t CAN_CRC8_SAE_J1850(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0xFFU;

    while (length-- > 0U)
    {
        crc = s_crc8_j1850_table[(uint8_t)(crc ^ *data++)];
    }

    return (uint8_t)(crc ^ 0xFFU);
}

static int16_t CAN_Read_I16_LE(uint8_t low_byte, uint8_t high_byte)
{
    return (int16_t)((uint16_t)low_byte | ((uint16_t)high_byte << 8));
}

static void CAN_Write_I16_LE(uint8_t *dst, int16_t value)
{
    if (dst == NULL)
    {
        return;
    }

    dst[0] = (uint8_t)((uint16_t)value & 0xFFU);
    dst[1] = (uint8_t)(((uint16_t)value >> 8) & 0xFFU);
}

static int16_t CAN_SaturateFloatToI16(float value)
{
    if (value > 32767.0f)
    {
        value = 32767.0f;
    }
    else if (value < -32768.0f)
    {
        value = -32768.0f;
    }

    if (value >= 0.0f)
    {
        value += 0.5f;
    }
    else
    {
        value -= 0.5f;
    }

    return (int16_t)value;
}

static float CAN_AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

static void CAN_Config_StdDataFilter(uint8_t filter_bank, uint16_t std_id)
{
    CAN_FilterTypeDef filter = {0};

    filter.FilterBank = filter_bank;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /*
     * bxCAN 32 位过滤寄存器的标准 ID 对齐方式：StdId 左移 5 位。
     * MaskHigh=0xFFE0 表示 11 位标准 ID 完全匹配。
     * MaskLow =0x0006 表示同时限定 IDE=0（标准帧）和 RTR=0（数据帧）。
     */
    filter.FilterIdHigh = (uint16_t)(std_id << 5);
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = 0xFFE0U;
    filter.FilterMaskIdLow = 0x0006U;

    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }
}

static void CAN_SetDiagBits(uint32_t bits)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_diag_bits |= bits;
    __set_PRIMASK(primask);
}

static void CAN_ClearDiagBits(uint32_t bits)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_diag_bits &= ~bits;
    __set_PRIMASK(primask);
}

static void CAN_AtomicIncU32(volatile uint32_t *value)
{
    uint32_t primask;

    if (value == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    *value = *value + 1U;
    __set_PRIMASK(primask);
}

static void CAN_AtomicIncU8Saturated(volatile uint8_t *value, uint8_t max_value)
{
    uint32_t primask;

    if (value == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (*value < max_value)
    {
        *value = (uint8_t)(*value + 1U);
    }
    __set_PRIMASK(primask);
}

static void CAN_ResetMotionCommand(void)
{
    g_robot_ctrl.target_vx = 0.0f;
    g_robot_ctrl.target_wz = 0.0f;
    g_robot_ctrl.ctrl_flags = 0U;
}

static void CAN_EnterSafeFault(uint32_t reason_bits, bool desync_counter)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    g_robot_ctrl.state = SYSTEM_SAFE_FAULT;
    CAN_ResetMotionCommand();
    s_diag_bits |= reason_bits;

    if (desync_counter)
    {
        s_counter_synced = false;
    }

    __set_PRIMASK(primask);

    FourWheel_LADRC_ResetAll();
}

static System_Health_t CAN_GetCurrentHealth(uint32_t diag_bits, System_State_t state)
{
    if (((diag_bits & CHASSIS_DIAG_FATAL_MASK) != 0U) || (state == SYSTEM_SAFE_FAULT))
    {
        return SYSTEM_HEALTH_FAULT;
    }

    if (((diag_bits & CHASSIS_DIAG_WARN_MASK) != 0U) ||
        (state == SYSTEM_BOOTING) ||
        (s_consecutive_crc_errors != 0U) ||
        (s_consecutive_counter_errors != 0U))
    {
        return SYSTEM_HEALTH_WARNING;
    }

    return SYSTEM_HEALTH_OK;
}

/* =====================================================================
 * 接收处理
 * ===================================================================== */

static void CAN_ProcessHeartbeat(const uint8_t *rx_data, uint8_t dlc)
{
    (void)rx_data;
    (void)dlc;

    /*
     * 心跳只表示“链路上仍然有人在说话”。
     * 它不会刷新运动看门狗，避免上位机停止发速度命令后底盘继续跑旧速度。
     */
}

static void CAN_ProcessControlFrame(const uint8_t *rx_data, uint8_t dlc)
{
    bool accept_frame = false;
    uint8_t rx_cnt;

    if (dlc != 8U)
    {
        g_dbg_dlc_reject++;
        return;
    }

    if (CAN_CRC8_SAE_J1850(rx_data, 7U) != rx_data[7])
    {
        CAN_AtomicIncU32(&s_crc_error_total);
        s_consecutive_counter_errors = 0U;
        CAN_AtomicIncU8Saturated(&s_consecutive_crc_errors, 0xFFU);

        if (s_consecutive_crc_errors >= CRC_STORM_THRESHOLD)
        {
            CAN_EnterSafeFault(DIAG_CMD_CRC_STORM, true);
        }
        g_dbg_crc_reject++;
        return;
    }

    rx_cnt = rx_data[6];

    if ((!s_counter_synced) || (g_robot_ctrl.state != SYSTEM_OPERATIONAL))
    {
        accept_frame = true;
    }
    else
    {
        uint8_t diff = (uint8_t)(rx_cnt - s_last_rx_counter);
        if ((diff >= 1U) && (diff <= MAX_ACCEPTED_COUNTER_GAP))
        {
            accept_frame = true;
        }
    }

    if (!accept_frame)
    {
        g_dbg_cnt_reject++;
        CAN_AtomicIncU32(&s_counter_reject_total);
        s_consecutive_crc_errors = 0U;
        CAN_AtomicIncU8Saturated(&s_consecutive_counter_errors, 0xFFU);

        if (s_consecutive_counter_errors >= COUNTER_STORM_THRESHOLD)
        {
            CAN_EnterSafeFault(DIAG_CMD_CNT_STORM, true);
        }
        return;
    }

    {
        int16_t vx_scaled = CAN_Read_I16_LE(rx_data[0], rx_data[1]);
        int16_t wz_scaled = CAN_Read_I16_LE(rx_data[2], rx_data[3]);
        uint32_t primask = __get_PRIMASK();

        __disable_irq();

        g_robot_ctrl.target_vx = (float)vx_scaled / 1000.0f;
        g_robot_ctrl.target_wz = (float)wz_scaled / 1000.0f;
        g_robot_ctrl.ctrl_flags = rx_data[4];
        g_robot_ctrl.rolling_cnt = rx_cnt;
        g_robot_ctrl.state = SYSTEM_OPERATIONAL;

        s_last_rx_counter = rx_cnt;
        s_counter_synced = true;
        s_last_valid_cmd_tick_ms = HAL_GetTick();
        s_have_seen_valid_cmd = true;
        s_cmd_age_10ms = 0U;

        /* 合法新帧到来后，允许通信相关与执行器相关的自动恢复。 */
        s_consecutive_crc_errors = 0U;
        s_consecutive_counter_errors = 0U;
        s_stall_ticks[MOTOR_FL] = 0U;
        s_stall_ticks[MOTOR_RL] = 0U;
        s_stall_ticks[MOTOR_FR] = 0U;
        s_stall_ticks[MOTOR_RR] = 0U;
        s_saturation_ticks = 0U;

        s_diag_bits &= ~(DIAG_COMM_TIMEOUT |
                         DIAG_CAN_BUS_OFF |
                         DIAG_CMD_CRC_STORM |
                         DIAG_CMD_CNT_STORM |
                         DIAG_MOTOR_FL_STALL |
                         DIAG_MOTOR_RL_STALL |
                         DIAG_MOTOR_FR_STALL |
                         DIAG_MOTOR_RR_STALL |
                         DIAG_CONTROL_SATURATION);

        __set_PRIMASK(primask);
    }

    CAN_AtomicIncU32(&s_valid_cmd_total);
    g_dbg_valid_accept++;
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
}

/* =====================================================================
 * 初始化与中断回调
 * ===================================================================== */

void CAN_App_Init(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();

    g_robot_ctrl.target_vx = 0.0f;
    g_robot_ctrl.target_wz = 0.0f;
    g_robot_ctrl.ctrl_flags = 0U;
    g_robot_ctrl.rolling_cnt = 0U;
    g_robot_ctrl.state = SYSTEM_BOOTING;

    s_last_valid_cmd_tick_ms = 0U;
    s_cmd_age_10ms = 0U;
    s_have_seen_valid_cmd = false;
    s_last_rx_counter = 0U;
    s_counter_synced = false;

    s_diag_bits = 0U;
    s_valid_cmd_total = 0U;
    s_crc_error_total = 0U;
    s_counter_reject_total = 0U;
    s_can_tx_drop_total = 0U;
    s_busoff_total = 0U;
    s_rx_overrun_total = 0U;
    s_consecutive_crc_errors = 0U;
    s_consecutive_counter_errors = 0U;

    memset(s_stall_ticks, 0, sizeof(s_stall_ticks));
    s_saturation_ticks = 0U;
    s_status_tx_counter = 0U;
    s_comm_diag_divider = 0U;

    __set_PRIMASK(primask);

    CAN_Config_StdDataFilter(0U, CAN_ID_HEARTBEAT);
    CAN_Config_StdDataFilter(1U, CAN_ID_CMD_VEL);

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR_WARNING |
                                              CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE |
                                              CAN_IT_ERROR) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t frames_processed = 0U;

    if (hcan != &hcan1)
    {
        return;
    }

    while ((frames_processed < CAN_RX_ISR_MAX_FRAMES_PER_CALL) &&
           (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U))
    {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];

        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        frames_processed++;

        if ((rx_header.IDE != CAN_ID_STD) || (rx_header.RTR != CAN_RTR_DATA))
        {
            continue;
        }

        if (rx_header.StdId == CAN_ID_HEARTBEAT)
        {
            CAN_ProcessHeartbeat(rx_data, rx_header.DLC);
        }
        else if (rx_header.StdId == CAN_ID_CMD_VEL)
        {
            g_dbg_rx_100_hits++;
            CAN_ProcessControlFrame(rx_data, rx_header.DLC);
        }
        else
        {
            /* 理论上硬件滤波器已拦掉其他 ID，这里只是双保险。 */
        }
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error_code;

    if (hcan != &hcan1)
    {
        return;
    }

    error_code = hcan->ErrorCode;

    if ((error_code & HAL_CAN_ERROR_BOF) != 0U)
    {
        CAN_AtomicIncU32(&s_busoff_total);
        CAN_EnterSafeFault(DIAG_CAN_BUS_OFF, true);
    }

#ifdef HAL_CAN_ERROR_FOV0
    if ((error_code & HAL_CAN_ERROR_FOV0) != 0U)
    {
        CAN_AtomicIncU32(&s_rx_overrun_total);
    }
#endif
}

/* =====================================================================
 * 10ms 硬实时任务
 * ===================================================================== */

void CAN_Safety_Watchdog_Tick(void)
{
    uint32_t elapsed_ms = 0U;
    uint32_t age_ticks = 0U;

    if (!s_have_seen_valid_cmd)
    {
        s_cmd_age_10ms = 0U;
        return;
    }

    elapsed_ms = HAL_GetTick() - s_last_valid_cmd_tick_ms;
    age_ticks = elapsed_ms / CONTROL_LOOP_PERIOD_MS;

    if (age_ticks > 0xFFU)
    {
        age_ticks = 0xFFU;
    }

    s_cmd_age_10ms = (uint8_t)age_ticks;

    if ((g_robot_ctrl.state == SYSTEM_OPERATIONAL) && (elapsed_ms >= CMD_TIMEOUT_MS))
    {
        CAN_EnterSafeFault(DIAG_COMM_TIMEOUT, true);
    }
}

void Kinematics_Update_LADRC(void)
{
    float target_vx;
    float target_wz;
    System_State_t state;
    float v_left;
    float v_right;
    float rpm_left;
    float rpm_right;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    target_vx = g_robot_ctrl.target_vx;
    target_wz = g_robot_ctrl.target_wz;
    state = g_robot_ctrl.state;
    __set_PRIMASK(primask);

    if (state != SYSTEM_OPERATIONAL)
    {
        FourWheel_Set_Target_RPM(0.0f, 0.0f, 0.0f, 0.0f);
        return;
    }

    v_left = target_vx - target_wz * (WHEEL_TRACK_M * 0.5f);
    v_right = target_vx + target_wz * (WHEEL_TRACK_M * 0.5f);

    rpm_left = (v_left / (2.0f * PI_F * WHEEL_RADIUS_M)) * 60.0f;
    rpm_right = (v_right / (2.0f * PI_F * WHEEL_RADIUS_M)) * 60.0f;

    FourWheel_Set_Target_RPM(rpm_left, rpm_left, rpm_right, rpm_right);
}

void Chassis_Diagnostics_10ms_Tick(void)
{
    uint32_t stall_bits = 0U;
    System_State_t state = g_robot_ctrl.state;
    uint32_t motor_stall_mask = (DIAG_MOTOR_FL_STALL |
                                 DIAG_MOTOR_RL_STALL |
                                 DIAG_MOTOR_FR_STALL |
                                 DIAG_MOTOR_RR_STALL);

    if (state != SYSTEM_OPERATIONAL)
    {
        s_stall_ticks[MOTOR_FL] = 0U;
        s_stall_ticks[MOTOR_RL] = 0U;
        s_stall_ticks[MOTOR_FR] = 0U;
        s_stall_ticks[MOTOR_RR] = 0U;
        s_saturation_ticks = 0U;
        CAN_ClearDiagBits(DIAG_CONTROL_SATURATION);
        return;
    }

    for (int i = 0; i < 4; ++i)
    {
        float target = CAN_AbsFloat(FourWheel_Get_Target_RPM((Motor_ID_t)i));
        float actual = CAN_AbsFloat(Get_Motor_RPM((Motor_ID_t)i));
        float output = CAN_AbsFloat(FourWheel_Get_Control_Output((Motor_ID_t)i));
        uint32_t bit = 0U;

        switch (i)
        {
            case MOTOR_FL: bit = DIAG_MOTOR_FL_STALL; break;
            case MOTOR_RL: bit = DIAG_MOTOR_RL_STALL; break;
            case MOTOR_FR: bit = DIAG_MOTOR_FR_STALL; break;
            case MOTOR_RR: bit = DIAG_MOTOR_RR_STALL; break;
            default: break;
        }

        if ((target >= STALL_TARGET_RPM_MIN) &&
            (actual <= STALL_FEEDBACK_RPM_MAX) &&
            (output >= STALL_OUTPUT_ABS_MIN))
        {
            if (s_stall_ticks[i] < STALL_CONFIRM_TICKS)
            {
                s_stall_ticks[i]++;
            }
        }
        else
        {
            s_stall_ticks[i] = 0U;
            CAN_ClearDiagBits(bit);
        }

        if (s_stall_ticks[i] >= STALL_CONFIRM_TICKS)
        {
            stall_bits |= bit;
        }
    }

    if (stall_bits != 0U)
    {
        CAN_EnterSafeFault(stall_bits, false);
    }
    else
    {
        CAN_ClearDiagBits(motor_stall_mask);
    }

    {
        bool saturation_found = false;

        for (int i = 0; i < 4; ++i)
        {
            float target = CAN_AbsFloat(FourWheel_Get_Target_RPM((Motor_ID_t)i));
            float output = CAN_AbsFloat(FourWheel_Get_Control_Output((Motor_ID_t)i));

            if ((target >= SATURATION_TARGET_RPM_MIN) && (output >= SATURATION_OUTPUT_ABS_MIN))
            {
                saturation_found = true;
                break;
            }
        }

        if (saturation_found)
        {
            if (s_saturation_ticks < SATURATION_CONFIRM_TICKS)
            {
                s_saturation_ticks++;
            }
        }
        else
        {
            s_saturation_ticks = 0U;
            CAN_ClearDiagBits(DIAG_CONTROL_SATURATION);
        }

        if (s_saturation_ticks >= SATURATION_CONFIRM_TICKS)
        {
            CAN_SetDiagBits(DIAG_CONTROL_SATURATION);
        }
    }
}

/* =====================================================================
 * 周期发送
 * ===================================================================== */

static HAL_StatusTypeDef CAN_TxStdFrame(uint16_t std_id, const uint8_t *payload, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    if ((payload == NULL) || (dlc > 8U))
    {
        CAN_AtomicIncU32(&s_can_tx_drop_total);
        return HAL_ERROR;
    }

    tx_header.StdId = std_id;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    /* Telemetry 帧允许丢：为避免阻塞主循环（引入 20ms 抖动/漂移），这里不做忙等。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        CAN_AtomicIncU32(&s_can_tx_drop_total);
        return HAL_BUSY;
    }

if (HAL_CAN_AddTxMessage(&hcan1, &tx_header, (uint8_t *)payload, &tx_mailbox) != HAL_OK)
    {
        CAN_AtomicIncU32(&s_can_tx_drop_total);
        return HAL_ERROR;
    }

    return HAL_OK;
}

void Chassis_Get_StatusSnapshot(Chassis_StatusSnapshot_t *snapshot)
{
    uint32_t primask;

    if (snapshot == NULL)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();

    snapshot->state = g_robot_ctrl.state;
    snapshot->diag_bits = s_diag_bits;
    snapshot->cmd_age_10ms = s_cmd_age_10ms;
    snapshot->last_cmd_counter = s_last_rx_counter;
    snapshot->health = CAN_GetCurrentHealth(snapshot->diag_bits, snapshot->state);

    __set_PRIMASK(primask);
}

static void CAN_Send_StatusFrame(void)
{
    Chassis_StatusSnapshot_t snapshot;
    uint8_t tx_data[8];

    Chassis_Get_StatusSnapshot(&snapshot);

    s_status_tx_counter++;

    tx_data[0] = (uint8_t)snapshot.state;
    tx_data[1] = (uint8_t)snapshot.health;
    tx_data[2] = (uint8_t)(snapshot.diag_bits & 0xFFU);
    tx_data[3] = (uint8_t)((snapshot.diag_bits >> 8) & 0xFFU);
    tx_data[4] = (uint8_t)((snapshot.diag_bits >> 16) & 0xFFU);
    tx_data[5] = (uint8_t)((snapshot.diag_bits >> 24) & 0xFFU);
    tx_data[6] = snapshot.cmd_age_10ms;
    tx_data[7] = s_status_tx_counter;

    (void)CAN_TxStdFrame(CAN_ID_STATUS, tx_data, 8U);
}

static void CAN_Send_ActualRpmFrame(void)
{
    uint8_t tx_data[8];

    CAN_Write_I16_LE(&tx_data[0], CAN_SaturateFloatToI16(Get_Motor_RPM(MOTOR_FL)));
    CAN_Write_I16_LE(&tx_data[2], CAN_SaturateFloatToI16(Get_Motor_RPM(MOTOR_RL)));
    CAN_Write_I16_LE(&tx_data[4], CAN_SaturateFloatToI16(Get_Motor_RPM(MOTOR_FR)));
    CAN_Write_I16_LE(&tx_data[6], CAN_SaturateFloatToI16(Get_Motor_RPM(MOTOR_RR)));

    (void)CAN_TxStdFrame(CAN_ID_ACTUAL_RPM, tx_data, 8U);
}

static void CAN_Send_TargetRpmFrame(void)
{
    uint8_t tx_data[8];

    CAN_Write_I16_LE(&tx_data[0], CAN_SaturateFloatToI16(FourWheel_Get_Target_RPM(MOTOR_FL)));
    CAN_Write_I16_LE(&tx_data[2], CAN_SaturateFloatToI16(FourWheel_Get_Target_RPM(MOTOR_RL)));
    CAN_Write_I16_LE(&tx_data[4], CAN_SaturateFloatToI16(FourWheel_Get_Target_RPM(MOTOR_FR)));
    CAN_Write_I16_LE(&tx_data[6], CAN_SaturateFloatToI16(FourWheel_Get_Target_RPM(MOTOR_RR)));

    (void)CAN_TxStdFrame(CAN_ID_TARGET_RPM, tx_data, 8U);
}

static void CAN_Send_CommDiagFrame(void)
{
    uint8_t tx_data[8];

    tx_data[0] = (uint8_t)(s_valid_cmd_total & 0xFFU);
    tx_data[1] = (uint8_t)(s_crc_error_total & 0xFFU);
    tx_data[2] = (uint8_t)(s_counter_reject_total & 0xFFU);
    tx_data[3] = (uint8_t)(s_can_tx_drop_total & 0xFFU);
    tx_data[4] = (uint8_t)(s_busoff_total & 0xFFU);
    tx_data[5] = (uint8_t)(s_rx_overrun_total & 0xFFU);
    tx_data[6] = s_last_rx_counter;
    tx_data[7] = (uint8_t)(((s_consecutive_counter_errors > 15U ? 15U : s_consecutive_counter_errors) << 4) |
                           (s_consecutive_crc_errors > 15U ? 15U : s_consecutive_crc_errors));

    (void)CAN_TxStdFrame(CAN_ID_COMM_DIAG, tx_data, 8U);
}

static void CAN_Send_OdomFrame(void)
{
    uint8_t tx_data[8];
    int16_t d_fl;
    int16_t d_rl;
    int16_t d_fr;
    int16_t d_rr;

    Motor_Get_And_Clear_Delta_Ticks(&d_fl, &d_rl, &d_fr, &d_rr);

    CAN_Write_I16_LE(&tx_data[0], d_fl);
    CAN_Write_I16_LE(&tx_data[2], d_rl);
    CAN_Write_I16_LE(&tx_data[4], d_fr);
    CAN_Write_I16_LE(&tx_data[6], d_rr);

    (void)CAN_TxStdFrame(CAN_ID_ODOM, tx_data, 8U);
}

void CAN_Send_Telemetry_20ms(void)
{
    static uint8_t s_telem_slot = 0U;

    /* 状态帧最重要，固定每 20ms 发一次 */
    CAN_Send_StatusFrame();

    /* 第二帧轮换发送，避免同一拍挤爆 3 个 TX mailbox */
    switch (s_telem_slot)
    {
        case 0:
            CAN_Send_ActualRpmFrame();   /* 0x182 */
            break;

        case 1:
            CAN_Send_TargetRpmFrame();   /* 0x183 */
            break;

        case 2:
            CAN_Send_OdomFrame();        /* 0x200 */
            break;

        default:
            s_telem_slot = 0U;
            CAN_Send_ActualRpmFrame();
            break;
    }

    s_telem_slot++;
    if (s_telem_slot >= 3U)
    {
        s_telem_slot = 0U;
    }

    /* 通信诊断帧保持 100ms 一次 */
    s_comm_diag_divider++;
    if (s_comm_diag_divider >= TELEMETRY_COMM_DIAG_DIV)
    {
        s_comm_diag_divider = 0U;
        CAN_Send_CommDiagFrame();        /* 0x184 */
    }
}
