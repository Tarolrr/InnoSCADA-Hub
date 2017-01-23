/*
 * GUID.h
 *
 *  Created on: Nov 18, 2016
 *      Author: Tarolrr
 */

#ifndef GUID_H_
#define GUID_H_

#include <stdint.h>
#include <stdbool.h>

extern const uint64_t GUID[];

bool checkGUID(uint8_t *buf);

#endif /* GUID_H_ */
