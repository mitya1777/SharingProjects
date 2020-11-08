#include "main.h"
#include "stm32l4xx_it.h"

extern DMA_HandleTypeDef hdma_adc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

extern uint8_t transmittion_en;
extern volatile uint8_t DMA_bufer_is_updated;
extern uint16_t tx_cnt;
extern uint16_t U1[0xFFF];
extern uint8_t button_on;
extern uint8_t new_iteration;
uint16_t Uset = 0x00;

void NMI_Handler(void)
{}

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
{}

void DebugMon_Handler(void)
{}

void PendSV_Handler(void)
{}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/

void DMA1_Channel1_IRQHandler(void)
{
   HAL_DMA_IRQHandler(&hdma_adc1);
   DMA_bufer_is_updated = 0x01;
 }

void USART1_IRQHandler(void)
{
   if ((USART1 -> ISR & USART_ISR_TC) != 0x00)
   {
	   USART1 -> ICR |= USART_ICR_TCCF;
	   USART1 -> CR1 &= ~USART_CR1_TE;
   }
	HAL_UART_IRQHandler(&huart1);
}

void TIM6_DAC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim6);

  DAC1 -> DHR12R2 = Uset;
  Uset ++;

  if (Uset == 0xFFF)
  {
	  Uset = 0x00;
	  HAL_TIM_Base_Stop_IT(&htim6);
  }
  ADC1 -> CR |= ADC_CR_ADSTART;
}

void EXTI0_IRQHandler(void)
{
   HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
   button_on = 0x01;

   if (new_iteration != 0x00)
   {
	   transmittion_en = 0x00;
	   HAL_TIM_Base_Start_IT(&htim6);
	   new_iteration = 0x00;
   }
}
