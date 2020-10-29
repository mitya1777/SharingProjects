#include "main.h"
#include "stm32l4xx_it.h"

extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

extern uint16_t U1[0x746];
extern uint16_t U2[0x746];
extern uint16_t U3[0x746];
extern uint8_t flag_UART_start;

uint8_t conversion = 0x00;
uint16_t U_out;
uint16_t count = 0x00;
uint8_t chanel_switch = 0x06;

void NMI_Handler(void)
{
  }

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

void ADC1_2_IRQHandler(void)
{
	if ((ADC1 -> ISR & ADC_ISR_EOC) != 0x00)
	{
		ADC1 -> ISR |= ADC_ISR_EOC;

		switch (chanel_switch)
		{
		case 0x06:
			U1[count] = ADC1 -> DR;
			break;

		case 0x07:
			U2[count] = ADC1 -> DR;
			break;

		case 0x08:
			U3[count] = ADC1 -> DR;
			chanel_switch = 0x05;
			break;
		}

		chanel_switch ++;
	}

	if ((ADC1 -> ISR & ADC_ISR_EOS) != 0x00)
	{
		ADC1 -> ISR |= ADC_ISR_EOS;
		chanel_switch = 0x06;
	}
}

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);
}

void TIM6_DAC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim6);

	DAC1 -> DHR12R2 = U_out;
	ADC1 -> CR |= ADC_CR_ADSTART;

	U_out ++;
	count ++;

	if (count == U_EDGE)
	{
		count = 0x00;
		flag_UART_start = 0x01;
	}

	if (U_out == U_EDGE)
	{
		U_out = 0x00;
	}


	//GPIOE -> ODR ^= GPIO_ODR_OD8;
}
