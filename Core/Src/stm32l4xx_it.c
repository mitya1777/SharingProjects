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
extern uint16_t Uset;

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
   DMA1 -> IFCR |= DMA_IFCR_CGIF1;
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
  TIM6 -> SR &= ~TIM_SR_UIF;
  DAC1 -> DHR12R2 = Uset;
  //if (DMA_bufer_is_updated == 0x00)
  //{
	  ADC1 -> CR |= ADC_CR_ADSTART;
  //}
}

void EXTI0_IRQHandler(void)
{
   EXTI -> PR1 |= EXTI_PR1_PIF0;
   button_on = 0x01;

   if (new_iteration != 0x00)
   {
	   transmittion_en = 0x00;
	   new_iteration = 0x00;
	   NVIC_EnableIRQ(TIM6_DAC_IRQn);
	   TIM6 -> CR1 |= TIM_CR1_CEN;
   }
}
