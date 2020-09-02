/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "lattice.h"
#include "eeprom_emulator.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

HRTIM_HandleTypeDef hhrtim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

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
    MX_HRTIM1_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_USART3_UART_Init();

    /* USER CODE BEGIN 2 */
    HAL_Delay(STARTUP_DELAY_IN_MS);

    // Turn on the contactor.
    EepromEmulator_Init();
    HAL_GPIO_WritePin(CONTACTOR_CTRL_GPIO_Port, CONTACTOR_CTRL_Pin, GPIO_PIN_RESET);
    Lattice_Start();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        Lattice_Execute();
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
    RCC_OscInitTypeDef RCC_OscInitStruct ={ 0 };
    RCC_ClkInitTypeDef RCC_ClkInitStruct ={ 0 };
    RCC_PeriphCLKInitTypeDef PeriphClkInit ={ 0 };

    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1 | RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
    PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode ={ 0 };
    ADC_ChannelConfTypeDef sConfig ={ 0 };

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */
    /** Common config
    */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONVHRTIM_TRG1;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the ADC multi-mode
    */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SingleDiff = ADC_DIFFERENTIAL_ENDED;
    sConfig.SamplingTime = ADC_SAMPLETIME_19CYCLES_5;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure Regular Channel
    */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

    /* USER CODE BEGIN HRTIM1_Init 0 */

    /* USER CODE END HRTIM1_Init 0 */

    HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg ={ 0 };
    HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg ={ 0 };
    HRTIM_TimerCfgTypeDef pTimerCfg ={ 0 };
    HRTIM_CompareCfgTypeDef pCompareCfg ={ 0 };
    HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg ={ 0 };
    HRTIM_OutputCfgTypeDef pOutputCfg ={ 0 };

    /* USER CODE BEGIN HRTIM1_Init 1 */

    /* USER CODE END HRTIM1_Init 1 */
    hhrtim1.Instance = HRTIM1;
    hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
    hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
    if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_14) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
    {
        Error_Handler();
    }
    pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_B;
    pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT13_TIMERB_CMP2;
    if (HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_1, &pADCTriggerCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pTimeBaseCfg.Period = 0xFFF7;
    pTimeBaseCfg.RepetitionCounter = 0x00;
    pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL8;
    pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
    pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
    pTimerCfg.DMASrcAddress = 0x0000;
    pTimerCfg.DMADstAddress = 0x0000;
    pTimerCfg.DMASize = 0x1;
    pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
    pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
    pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
    pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
    pTimerCfg.PreloadEnable = HRTIM_PRELOAD_ENABLED;
    pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
    pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
    pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
    pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
    pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
    pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READONLY;
    pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
    pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
    pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
    pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
    pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_ENABLED;
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pTimerCfg.DMASrcAddress = 0x0000;
    pTimerCfg.DMADstAddress = 0x0000;
    pTimerCfg.DMASize = 0x1;
    pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
    pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
    pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_TIMER_A;
    pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_OTHER1_CMP1;
    pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
    if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pCompareCfg.CompareValue = 0x0018;
    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pCompareCfg.CompareValue = 0xFFF7;
    pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
    pCompareCfg.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pCompareCfg.CompareValue = 0x8000;
    pCompareCfg.AutoDelayedTimeout = 0x0000;

    if (HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_DIV1;
    pDeadTimeCfg.RisingValue = 360;
    pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
    pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_READONLY;
    pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_READONLY;
    pDeadTimeCfg.FallingValue = 332;
    pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
    pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_READONLY;
    pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_READONLY;
    if (HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_LOW;
    pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
    pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP2;
    pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
    pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
    pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
    pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
    pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
    {
        Error_Handler();
    }
    pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
    pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
    if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN HRTIM1_Init 2 */

    /* USER CODE END HRTIM1_Init 2 */
    HAL_HRTIM_MspPostInit(&hhrtim1);
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

    /* USER CODE BEGIN USART1_Init 0 */

    /* USER CODE END USART1_Init 0 */

    /* USER CODE BEGIN USART1_Init 1 */

    /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 38400;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART1_Init 2 */

    /* USER CODE END USART1_Init 2 */
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

    /* USER CODE BEGIN USART3_Init 0 */

    /* USER CODE END USART3_Init 0 */

    /* USER CODE BEGIN USART3_Init 1 */

    /* USER CODE END USART3_Init 1 */
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 38400;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART3_Init 2 */

    /* USER CODE END USART3_Init 2 */
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{
    /* DMA controller clock enable */
    __HAL_RCC_DMA1_CLK_ENABLE();

    /* DMA interrupt init */
    /* DMA1_Channel1_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct ={ 0 };

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, RS485_DEnRE_Pin | LED_RED_Pin | LED_GREEN_Pin | LED_BLUE_Pin | MISC0_Pin | MISC1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(CONTACTOR_CTRL_GPIO_Port, CONTACTOR_CTRL_Pin, GPIO_PIN_SET);

    /*Configure GPIO pins : TRIGIN_RISING_Pin */
    GPIO_InitStruct.Pin = TRIGIN_RISING_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : TRIGIN_FALLING_Pin */
    GPIO_InitStruct.Pin = TRIGIN_FALLING_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : RS485_DEnRE_Pin */
    GPIO_InitStruct.Pin = RS485_DEnRE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : LED_RED_Pin LED_GREEN_Pin LED_BLUE_Pin MISC0_Pin
                             MISC1_Pin MISC2_Pin */
    GPIO_InitStruct.Pin = LED_RED_Pin | LED_GREEN_Pin | LED_BLUE_Pin | MISC0_Pin | MISC1_Pin | CONTACTOR_CTRL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* USER CODE BEGIN 4 */
void Lattice_StatusChangeCb(Lattice_Status_t *status)
{
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == TRIGIN_RISING_Pin)
    {
        Lattice_CmdPowerOutput(TRUE);
    }

    if (GPIO_Pin == TRIGIN_FALLING_Pin)
    {
        Lattice_CmdPowerOutput(FALSE);
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
void assert_failed(char *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
       /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
