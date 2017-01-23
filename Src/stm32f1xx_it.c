#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"
#include "cmsis_os.h"

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim1;
extern void GSM_UART_IRQHandler(void);
void NMI_Handler(void){}
void HardFault_Handler(void){while (1);}
void MemManage_Handler(void) {while (1);}
void BusFault_Handler(void) {while (1);}
void UsageFault_Handler(void) {while (1);}
void DebugMon_Handler(void) {}

void SysTick_Handler(void){
	osSystickHandler();
}

void SPI1_IRQHandler(void){
  HAL_SPI_IRQHandler(&hspi1);
}

void TIM1_UP_IRQHandler(void){
  HAL_TIM_IRQHandler(&htim1);
}

void USART2_IRQHandler(void){

}
