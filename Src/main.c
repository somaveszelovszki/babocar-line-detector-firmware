/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "config.h"
#include "linepanel.h"
#include "linepos.h"
#include "linefilter.h"

#include <micro/math/numeric.h>
#include <micro/panel/panelLink.h>
#include <micro/panel/LineDetectPanelData.h>

#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define uart_cmd (&huart1)
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static bool indicatorLedsEnabled = false;
static panelLink_t panelLink;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

static void handleRxData(const lineDetectPanelDataIn_t *rxData) {
    indicatorLedsEnabled = !!(rxData->flags & LINE_DETECT_PANEL_FLAG_INDICATOR_LEDS_ENABLED);
}

static void fillTxData(lineDetectPanelDataOut_t *txData, const trackedLines_t *lines) {

    for (uint8_t i = 0; i < lines->numLines; ++i) {
        txData->values[i] = lines->values[i];
    }

    for (uint8_t i = lines->numLines; i < MAX_NUM_LINES; ++i) {
        trackedLine_t *l = &txData->values[i];
        l->pos_mm = 0;
        l->id = INVALID_LINE_IDX;
    }
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  linePos_t linesData;
  lineFilter_t lineFilter;

  uint8_t measurements[NUM_OPTOS];
  uint8_t leds[NUM_OPTOS / 8] = { 0, 0, 0, 0 };
  uint32_t nextLedToggleTime  = HAL_GetTick();
  lineDetectPanelDataIn_t rxData;
  lineDetectPanelDataOut_t txData;
  bool prevConnected = false;
  trackedLines_t trackedLines;
  trackedLines.numLines = 0;

  linepanel_initialize();
  linepos_initialize(&linesData);
  linefilter_initialize(&lineFilter);

  lineDetectPanelDataIn_t rxDataBuffer;
  lineDetectPanelDataOut_t txDataBuffer;
  panelLink_initialize((panelLink_t*)&panelLink, PanelLinkRole_Slave, uart_cmd,
      &rxDataBuffer, sizeof(lineDetectPanelDataIn_t), LINE_DETECT_PANEL_LINK_IN_TIMEOUT_MS,
      &txDataBuffer, sizeof(lineDetectPanelDataOut_t), LINE_DETECT_PANEL_LINK_OUT_PERIOD_MS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1) {

      panelLink_update((panelLink_t*)&panelLink);
      const bool isConnected = panelLink_isConnected((const panelLink_t*)&panelLink);

      if (panelLink_readAvailable((panelLink_t*)&panelLink, &rxData)) {
          handleRxData(&rxData);
      }

      linepanel_read_optos(measurements);
      linepos_calc(&linesData, measurements);
      linefilter_apply(&lineFilter, &linesData.lines, &trackedLines);

      if (panelLink_shouldSend((panelLink_t*)&panelLink)) {
          fillTxData(&txData, &trackedLines);
          panelLink_send((panelLink_t*)&panelLink, &txData);
      }

      if (isConnected) {
          if (indicatorLedsEnabled) {
              linepos_set_leds(&trackedLines, leds);
              linepanel_write_leds(leds);
          } else if (!prevConnected) { // resets LEDS after blinking
              for (uint8_t i = 0; i < NUM_OPTOS / 8; ++i) {
                  leds[i] = 0x00;
              }
              linepanel_write_leds(leds);
          }
      } else if (HAL_GetTick() >= nextLedToggleTime) {
          nextLedToggleTime += 250;
          for (uint8_t i = 1; i < NUM_OPTOS / 8; ++i) {
              leds[i] = 0x00;
          }
          leds[0] = leds[0] == 0x00 ? 0x80 : 0x00;
          linepanel_write_leds(leds);
      }

      prevConnected = isConnected;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        panelLink_onNewRxData((panelLink_t*)&panelLink);
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
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
