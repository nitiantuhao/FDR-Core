/* USER CODE BEGIN Header */

#include <stdio.h>
#include "ladrc.h"
#include "usbd_cdc_if.h" // USB 虚拟串口发送头文件
#include "f4_can_app.h"
#include "usart.h"
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TELEMETRY_TX_PERIOD_MS    20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern USBD_HandleTypeDef hUsbDeviceFS;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
  uint32_t start_tick;
  uint8_t result;

  (void)file;

  if ((ptr == NULL) || (len <= 0))
  {
    return 0;
  }

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return 0;
  }

  start_tick = HAL_GetTick();
  do
  {
    result = CDC_Transmit_FS((uint8_t *)ptr, (uint16_t)len);
    if (result == USBD_OK)
    {
      return len;
    }
  } while ((result == USBD_BUSY) && ((HAL_GetTick() - start_tick) < 2U));

  return 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_USB_DEVICE_Init();
  MX_TIM9_Init();
  MX_TIM6_Init();
  MX_CAN1_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  FourWheel_LADRC_Init();
  CAN_App_Init();

  /* 启动 10ms 硬实时控制节拍。 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * 20ms 周期发送的“无漂移（driftless）”调度器：
     *
     * 之前版本用 last = now 的写法：
     *   if (now - last >= period) { last = now; ... }
     * 在主循环存在抖动/阻塞时会把“相位”带着一起漂移，表现为 20ms 周期不再对齐。
     *
     * 这里改成 next_deadline += period 的写法，让发送时刻尽量锁在固定相位上；
     * 若主循环被阻塞太久（落后 >= 1 个周期），则做一次“软重同步”，避免一口气补发多次造成总线突发。
     */
    static uint32_t next_telemetry_deadline = 0U;
    uint32_t now = HAL_GetTick();

    if (next_telemetry_deadline == 0U)
    {
      next_telemetry_deadline = now + TELEMETRY_TX_PERIOD_MS;
    }

    if ((int32_t)(now - next_telemetry_deadline) >= 0)
    {
      next_telemetry_deadline += TELEMETRY_TX_PERIOD_MS;
      CAN_Send_Telemetry_20ms();

      /* 如果主循环太慢导致仍然落后（>= 1 周期），重同步，避免连发突刺。 */
      if ((int32_t)(now - next_telemetry_deadline) >= 0)
      {
        next_telemetry_deadline = now + TELEMETRY_TX_PERIOD_MS;
        /* 可选：在这里置一个诊断位，提示“Telemetry Late”。 */
        /* CAN_Report_TelemetryLate(); */
      }
    }

    /*
     * 主循环保持轻量。
     * 如果后续要加 LED、串口 shell、参数配置等慢任务，尽量都放这里。
     */
    /* USER CODE END WHILE */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief  定时器周期中断回调。
  * @note   TIM6 是整个底盘控制的“硬实时心跳”。
  *         固定顺序：
  *         1) 控制命令看门狗
  *         2) 差速逆解算
  *         3) 四轮 LADRC 闭环
  *         4) 基于最新反馈做故障诊断
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    CAN_Safety_Watchdog_Tick();
    Kinematics_Update_LADRC();
    FourWheel_LADRC_Control_Loop();
    Chassis_Diagnostics_10ms_Tick();
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
