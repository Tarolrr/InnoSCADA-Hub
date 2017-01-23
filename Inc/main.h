/*
 * main.h
 *
 *  Created on: Oct 15, 2016
 *      Author: Tarolrr
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "pin_description.h"

const uint64_t GUID[] = {
	0x2B2CB0ED7CAA4FED,
	0xBAD921D6CA8A536B
};

//constant for LoRa driver

const uint8_t NetworkID = 0x01;
const uint16_t LoRaAddress = 0x0000;


#endif /* MAIN_H_ */
