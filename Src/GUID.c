/*
 * GUID.c
 *
 *  Created on: Nov 18, 2016
 *      Author: Tarolrr
 */

#include "GUID.h"

bool checkGUID(uint8_t *buf){
	uint8_t idx;
	for(idx = 0; idx < 16; idx++){
		if(buf[idx] != ((uint8_t *)GUID)[idx])
			return false;
	}
	return true;
}
