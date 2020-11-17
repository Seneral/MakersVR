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
#include <stdlib.h>

static const uint8_t hex[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F' };

/* Debug */

#define DEBUG
#ifdef DEBUG

#include <string.h>
#include <stdint.h>

#define DEBUG_BUFFER_SIZE 256
extern volatile uint_fast16_t debugPos;
extern volatile uint_fast16_t debugOffset;
extern uint8_t debugBuffer[DEBUG_BUFFER_SIZE];

#define DEBUG_BUF(BUF, SZ) \
if (debugPos + (SZ) <= DEBUG_BUFFER_SIZE) { \
	memcpy(&debugBuffer[debugPos], BUF, SZ); \
	debugPos += SZ; \
} else debugPos = DEBUG_BUFFER_SIZE // Prevent any other debug messages to come through to avoid confusion
#define DEBUG_CHARR(...) \
{ \
	uint8_t arr[] = { __VA_ARGS__ }; \
	DEBUG_BUF(arr, sizeof(arr)); \
}
#define DEBUG_STR(STR) \
{ \
	const uint8_t arr[] = u8 ## STR; \
	DEBUG_BUF(arr, sizeof(arr)-1); \
} // No -1 is fine, don't worry about it
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
#define DEBUG_HEX(...) {}
#endif // DEBUG


/* us Timing */

typedef struct {
	uint32_t ms;
	uint_fast16_t us;
} TimePoint;
typedef uint64_t TimeSpan; // Good for ~1hr time difference only

// Global System Timer
extern volatile uint32_t msCounter;

// Simple Time helpers
inline __attribute__((always_inline)) uint_fast16_t getUSCounter()
{
	return TIM2->CNT;
}
inline __attribute__((always_inline)) void SetTimePoint(TimePoint *point)
{
	point->ms = msCounter;
	point->us = getUSCounter();
	if (point->ms != msCounter)
		point->us = getUSCounter();
}
inline __attribute__((always_inline)) TimeSpan GetTimeSpanUS(TimePoint *pointB, TimePoint *pointA)
{
	return (uint64_t)abs((int32_t)pointB->ms - (int32_t)pointA->ms) * 1000 + abs((int32_t)pointB->us - (int32_t)pointA->us);
}
inline __attribute__((always_inline)) TimeSpan GetTimeSpanMS(TimePoint *pointB, TimePoint *pointA)
{
	return (uint64_t)abs((int32_t)pointB->ms - (int32_t)pointA->ms);
}
inline __attribute__((always_inline)) void delayUS(uint32_t us)
{
	uint32_t tgtUS = getUSCounter() + us%1000;
	uint32_t tgtMS = msCounter + us/1000 + tgtUS/1000;
	tgtUS = tgtUS % 1000;

	while (msCounter < tgtMS);
	while (getUSCounter() < tgtUS);
}


#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
