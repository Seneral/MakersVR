/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "stm32.h"
#include "stddef.h"

#define DEBUG
#ifdef DEBUG

#include <string.h>
#include <stdint.h>

#define DEBUG_BUFFER_SIZE 512
extern uint_fast16_t debugPos;
extern uint_fast16_t debugOffset;
extern uint8_t debugBuffer[DEBUG_BUFFER_SIZE];

#define DEBUG_BUF(BUF, SZ) \
debugPos = debugPos + (SZ); \
if (debugPos <= DEBUG_BUFFER_SIZE) \
	memcpy(&debugBuffer[debugPos - (SZ)], BUF, SZ); \
else \
	debugPos = DEBUG_BUFFER_SIZE // Prevent any other debug messages to come through to avoid confusion
#define DEBUG_CHARR(...) \
{ \
	uint8_t arr[] = { __VA_ARGS__ }; \
	DEBUG_BUF(arr, sizeof(arr)); \
}
#define DEBUG_STR(STR) \
{ \
	const uint8_t arr[] = u8 ## STR; \
	DEBUG_BUF(arr, sizeof(arr)-1); \
}
static const uint8_t hex[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };
#define DEBUG_HEX(BUF, SZ) \
{ \
	uint8_t arr[SZ*2]; \
	for (int i = 0; i < SZ; i++) \
	{ \
		arr[2*i+0] = hex[((uint8_t*)(BUF))[i] >> 4]; \
		arr[2*i+1] = hex[((uint8_t*)(BUF))[i] & 0xF]; \
	} \
	DEBUG_BUF(arr, sizeof(arr)); \
}
#else
#define DEBUG_BUF(...) {}
#define DEBUG_CHARR(...) {}
#define DEBUG_STR(...) {}
#endif // DEBUG


typedef struct {
	uint32_t ms;
	uint_fast16_t us;
} TimePoint;
typedef int32_t TimeSpan;

// Global System Timer
extern uint32_t msCounter;

// Simple Time helpers
inline uint_fast16_t getUSCounter()
{
	return TIM2->CNT;
}
inline void SetTimePoint(TimePoint *point)
{
	point->ms = msCounter;
	point->us = getUSCounter();
	if (point->ms != msCounter)
		point->us = getUSCounter();
}
inline TimeSpan GetTimeSpan(TimePoint *pointB, TimePoint *pointA)
{
	return (pointB->ms - pointA->ms) * 1000 + (pointB->us - pointA->us);
}


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
