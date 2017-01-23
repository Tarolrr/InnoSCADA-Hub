#include "stm32f1xx_hal.h"
#include "pin_description.h"
extern void Error_Handler(void);

void HAL_MspInit(void)
{
	__HAL_RCC_AFIO_CLK_ENABLE();
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);

	__HAL_AFIO_REMAP_SWJ_NOJTAG();
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi){
	GPIO_InitTypeDef GPIO_InitStruct;
	if(hspi->Instance==SPI1)
	{
		__HAL_RCC_SPI1_CLK_ENABLE();

		/**SPI1 GPIO Configuration
		PB0     ------> SPI1_NSS
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
		GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		/*GPIO_InitStruct.Pin = GPIO_PIN_0;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);/**/


		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);/**/

		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		HAL_NVIC_SetPriority(SPI1_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SPI1_IRQn);
	}
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi){
	if(hspi->Instance==SPI1){
		__HAL_RCC_SPI1_CLK_DISABLE();

		/**SPI1 GPIO Configuration
		PB0     ------> SPI1_NSS
		PA5     ------> SPI1_SCK
		PA6     ------> SPI1_MISO
		PA7     ------> SPI1_MOSI
		*/
	//	HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0);
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
		HAL_NVIC_DisableIRQ(SPI1_IRQn);
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart){
  GPIO_InitTypeDef GPIO_InitStruct;
  if(huart->Instance==USART2){
    __HAL_RCC_USART2_CLK_ENABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(USART2_IRQn, 14, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart){
  if(huart->Instance==USART2){
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  }
}
