/*
 * pin_description.h
 *
 *  Created on: Oct 24, 2016
 *      Author: Tarolrr
 */

#ifndef PIN_DESCRIPTION_H_
#define PIN_DESCRIPTION_H_

#include "stm32f1xx_hal.h"

#define RING_TIMEOUT 100

struct {
	GPIO_TypeDef * const port;
	uint16_t const pin;
	uint32_t lastEvent;			//время последнего изменения состояния. HAL_GetTick
	GPIO_PinState lastState;	//последнее состояние
} typedef PinDescription;

struct {
	GPIO_PinState state;
	uint16_t pinNumber;
} typedef MessagePinStateChange;	//Используется в очереди

static PinDescription SPICSPin = {GPIOA, GPIO_PIN_4, 0, 0};
//static PinDescription SPICSPin = {GPIOB, GPIO_PIN_0, 0, 0};
//static PinDescription LoRaTxRxPin = {GPIOC, GPIO_PIN_13, 0, 0};
//static PinDescription LoRaRxEnPin = {GPIOC, GPIO_PIN_13, 0, 0};
//static PinDescription LoRaTxEnPin = {GPIOA, GPIO_PIN_4, 0, 0};

#define GPIO_PIN_SET(pinDesc) {HAL_GPIO_WritePin((pinDesc)->port,(pinDesc)->pin,GPIO_PIN_SET); (pinDesc)->lastState = GPIO_PIN_SET; (pinDesc)->lastEvent = HAL_GetTick();}
#define GPIO_PIN_RESET(pinDesc) {HAL_GPIO_WritePin((pinDesc)->port,(pinDesc)->pin,GPIO_PIN_RESET); (pinDesc)->lastState = GPIO_PIN_RESET; (pinDesc)->lastEvent = HAL_GetTick();}
#define GPIO_PIN_TOGGLE(pinDesc) {HAL_GPIO_TogglePin((pinDesc)->port,(pinDesc)->pin); (pinDesc)->lastState = 1 - (pinDesc)->lastState; (pinDesc)->lastEvent = HAL_GetTick();}
#define GPIO_PIN_WRITE(pinDesc,state) {HAL_GPIO_WritePin((pinDesc)->port,(pinDesc)->pin,state); (pinDesc)->lastState = state; (pinDesc)->lastEvent = HAL_GetTick();}
#define GPIO_PIN_READ(pinDesc) HAL_GPIO_ReadPin((pinDesc)->port,(pinDesc)->pin)

#endif /* PIN_DESCRIPTION_H_ */
