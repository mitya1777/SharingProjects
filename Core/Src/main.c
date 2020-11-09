#include "main.h"

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
DAC_HandleTypeDef hdac1;
TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);

uint16_t Uset;
uint16_t DMA_buffer[0x03];
uint16_t U1[U_ARRAY_SIZE];
uint16_t U2[U_ARRAY_SIZE];
uint16_t U3[U_ARRAY_SIZE];

uint32_t DMA_bufer_index = 0x00;
volatile uint8_t DMA_bufer_is_updated = 0x00;
uint8_t UART_start_flag = 0x00;
uint8_t transmittion_en = 0x00;
uint16_t tx_cnt = 0x00;
uint16_t u_rx = 0x00;
uint16_t byte1 = 0x00;
uint16_t byte2 = 0x00;

uint8_t button_on = 0x00;
uint8_t new_iteration = 0x00;
uint8_t Rmes = R_MES;
uint8_t Pcnst = P_CONST;

float u1, u2, u3;
float Icur;
float Pcur;

int main(void)
{
HAL_Init();
SystemClock_Config();

MX_GPIO_Init();
MX_DMA_Init();
MX_ADC1_Init();
MX_DAC1_Init();
MX_TIM6_Init();
MX_USART1_UART_Init();

HAL_ADC_Start_DMA(&hadc1, (uint32_t*) DMA_buffer, 0x03);
TIM6 -> CR1 |= TIM_CR1_CEN;

	while (1)
	{
		if (DMA_bufer_is_updated == 0x01)
		{
			/*
			 * 		Current power consumption calculating
			 */
			u1 = 3.3 * ((float) DMA_buffer[0] / 0xFFF);
			u2 = 3.3 * ((float) DMA_buffer[1] / 0xFFF);
			u3 = 3.3 * ((float) DMA_buffer[2] / 0xFFF);

			Icur = (u1 - u2) / Rmes;
			Pcur = Icur * (u2 - u3);

			/*
			 * 		Impact action
			 */
			if (Pcur < Pcnst)
			{
				Uset ++;
			}

			else if (Pcur > Pcnst)
			{
				Uset --;
			}

			/*
			 * 		Store voltage data capturing by DMA in the base array
			 */
			for (uint8_t sw = 0x00; sw < 0x04; sw ++)
			{
				switch (sw)
				{
					case 0:
						U1[DMA_bufer_index] = DMA_buffer[0];
						break;

					case 1:
						U2[DMA_bufer_index] = DMA_buffer[1];
						break;

					case 2:
						U3[DMA_bufer_index] = DMA_buffer[2];
						break;
				}
			}

			DMA_bufer_index ++;

			if ((Uset == 0xFFC) || (DMA_bufer_index == U_ARRAY_SIZE))
			{
				NVIC_DisableIRQ(TIM6_DAC_IRQn);
				NVIC_EnableIRQ(EXTI0_IRQn);
				DMA_bufer_index = 0x00;
				transmittion_en = 0x01;
				GPIOE -> ODR |= GPIO_ODR_OD8;
			}
			DMA_bufer_is_updated = 0x00;
		}

		/*
		 * 		Launch data transmission in a full transistor open case
		 */
		if ((transmittion_en == 0x01) &&
		   ((USART1 -> ISR & USART_ISR_TXE) != 0x00) &&
		    (button_on == 0x01))
		{
			if ((USART1 -> CR1 & USART_CR1_TE) == 0x00)
			{
				USART1 -> CR1 |= USART_CR1_TE;
			}

			byte1 = (U1[u_rx] & 0xFF0) >> 0x04;
			USART1 -> TDR = byte1;

			while ((USART1 -> ISR & USART_ISR_TXE) == 0x00);
			byte2 = (U1[u_rx] & 0x00F) << 0x04;
			USART1 -> TDR = byte2 ;

			u_rx ++;

			if (u_rx == 0xFFF)
			{
				USART1 -> CR1 &= ~USART_CR1_TE;
				u_rx = 0x00;
				transmittion_en ++;
				button_on = 0x00;
			}
		}

		if ((transmittion_en == 0x02) &&
		   ((USART1 -> ISR & USART_ISR_TXE) != 0x00) &&
		    (button_on == 0x01))
		{
			if ((USART1 -> CR1 & USART_CR1_TE) == 0x00)
			{
				USART1 -> CR1 |= USART_CR1_TE;
			}

			byte1 = (U2[u_rx] & 0xFF0) >> 0x04;
			USART1 -> TDR = byte1;

			while ((USART1 -> ISR & USART_ISR_TXE) == 0x00);
			byte2 = (U2[u_rx] & 0x00F) << 0x04;
			USART1 -> TDR = byte2 ;

			u_rx ++;

			if (u_rx == 0xFFF)
			{
				USART1 -> CR1 &= ~USART_CR1_TE;
				u_rx = 0x00;
				transmittion_en ++;
				button_on = 0x00;
			}
		}
		if ((transmittion_en == 0x03) &&
		   ((USART1 -> ISR & USART_ISR_TXE) != 0x00) &&
		    (button_on == 0x01))
		{
			if ((USART1 -> CR1 & USART_CR1_TE) == 0x00)
			{
				USART1 -> CR1 |= USART_CR1_TE;
			}

			byte1 = (U3[u_rx] & 0xFF0) >> 0x04;
			USART1 -> TDR = byte1;

			while ((USART1 -> ISR & USART_ISR_TXE) == 0x00);
			byte2 = (U3[u_rx] & 0x00F) << 0x04;
			USART1 -> TDR = byte2 ;

			u_rx ++;

			if (u_rx == 0xFFF)
			{
				USART1 -> CR1 &= ~USART_CR1_TE;
				u_rx = 0x00;
				button_on = 0x00;
				new_iteration = 0x01;
			}
		}
	}
}



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
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
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
   sConfig.Channel = ADC_CHANNEL_6;
   sConfig.Rank = ADC_REGULAR_RANK_1;
   sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
   sConfig.SingleDiff = ADC_SINGLE_ENDED;
   sConfig.OffsetNumber = ADC_OFFSET_NONE;
   sConfig.Offset = 0;
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
   {
     Error_Handler();
   }
   /** Configure Regular Channel
   */
   sConfig.Channel = ADC_CHANNEL_7;
   sConfig.Rank = ADC_REGULAR_RANK_2;
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
   {
     Error_Handler();
   }
   /** Configure Regular Channel
   */
   sConfig.Channel = ADC_CHANNEL_8;
   sConfig.Rank = ADC_REGULAR_RANK_3;
   if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
   {
     Error_Handler();
   }
   HAL_ADC_Start(&hadc1);
}

static void MX_DAC1_Init(void)
{
  DAC_ChannelConfTypeDef sConfig = {0};

  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT2 config 
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  DAC1 -> CR |= DAC_CR_EN2;
}

static void MX_TIM6_Init(void)
{
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0x10;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0x01;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  TIM6 -> DIER |= TIM_DIER_UIE;
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 19200;
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
}

static void MX_DMA_Init(void) 
{
  __HAL_RCC_DMA1_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PE8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
   HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
}

void Error_Handler(void)
{}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{ }
#endif /* USE_FULL_ASSERT */
