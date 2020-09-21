#include "main.h"
#include "stm32l4xx_it.h"

extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim6;

uint8_t conversion = 0x00;
uint16_t U1, U2;
uint16_t U_out;



void NMI_Handler(void)
{  }


void HardFault_Handler(void)
{
  while (1)
  {  }
}

void MemManage_Handler(void)
{
  while (1)
  {  }
}


void BusFault_Handler(void)
{
  while (1)
  {  }
}

void UsageFault_Handler(void)
{
  while (1)
  {  }
}

void SVC_Handler(void)
{	}

void DebugMon_Handler(void)
{	}

void PendSV_Handler(void)
{	}

void SysTick_Handler(void)
{
  HAL_IncTick();
}


void ADC1_2_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);

  if ((ADC1 -> ISR & ADC_ISR_EOC) != 0x00)
  {
	  U1 = ADC1 -> DR;
	  conversion ++;

	  if (conversion == 0x01)
	  {
		  U2 = ADC1 -> DR;
		  conversion = 0x00;
	  }
  }
}


void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);
//  HAL_DAC_IRQHandler(&hdac1);

  ADC1 -> CR |= ADC_CR_ADSTART;
  DAC1 -> DHR12R2 = U_out;

  U_out ++;

  if (U_out == U_EDGE)
  {
	  U_out = 0x00;
  }
}

