/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/***** Defines *****/

//#define DEBUG_TRANSFER_IN
#define MEASURE_RECEIVE_RATE
//#define DEBUG_BLOBS_IN

/***** Includes *****/
#include <stdio.h>
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <ctime>

#define NOMINMAX
#include <windows.h>

#include "comm.hpp"
#include "util.hpp"

CommState commState;

#ifdef MEASURE_RECEIVE_RATE
static std::chrono::time_point<std::chrono::steady_clock> g_lastReceiveTime;
static StatValue receiveRate;
static std::atomic<long long> g_receiveCount = 0;
#endif

inline void printBuffer(uint8_t *buffer, uint8_t size)
{
	printf("0x");
	for (int i = 0; i < size; i++) printf("%02X", buffer[i]);
}

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length);
static void onIsochronousIN(uint8_t *data, int length);
static void onInterruptIN(uint8_t *data, int length);

int main(int argc, char **argv)
{
	printf("MakersVR Console Emulator. Press 'Q' to quit. \n");
	// Discard any keypresses before this point
	GetAsyncKeyState(0x51); // Q
	GetAsyncKeyState(0x43); // C

	// Comm setup
	if (!comm_init(&commState)) return 1;
	std::atexit([]{ comm_exit(&commState); });	
	std::atexit([]{ comm_disconnect(&commState); });
	commState.onControlResponse = onControlResponse;
	commState.onInterruptIN = onInterruptIN;
	commState.onIsochronousIN = onIsochronousIN;

	bool exit = false;
	while (!exit) 
	{
		// Wait for device
		while(!comm_check_device(&commState))
		{
			if (GetAsyncKeyState(0x51) != 0) return 0; // Q
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// Connect to device
		if (!comm_connect(&commState, true)) return 2;

		int frameCount = 0;
		while (commState.usbDeviceActive)
		{ // Comm loop
			if (exit = (GetAsyncKeyState(0x51) != 0)) break; // Q

			if (frameCount % 1 == 0)
			{ // Ask for debug output from device
				comm_submit_control_request(&commState, 0x01, 0, 0);
			}

			/*if (GetAsyncKeyState(0x43) != 0 && !commState.ctrlINPending)
			{ // C -> Request for test data (inverted request)q
				comm_submit_control_request(&commState, 0x00, 0, 0);
				std::this_thread::sleep_for(std::chrono::milliseconds(10));
			}*/

			frameCount++;
			std::this_thread::sleep_for(std::chrono::milliseconds(10));
		}

		// Disconnect on error and on abort
		comm_disconnect(&commState);
	}

	// atexit handles library exit
	return 0;	
}

static void onControlResponse(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length)
{
	if (request == 0x01) 
	{ // Debug Response
		if (length != 0) 
		{
			//printf("DEBUG (%d): ", length);
			//printBuffer(data, length);
			//printf("\n");
			for (int i = 0; i < length; i++) if (data[i] == 0) data[i] = '*';
			data[length] = 0;
			printf("DEBUG (%d): %s \n", length, data);
		}
	}
	else 
	{
		printf("Control Response %d (%d): ", request, length);
		printBuffer(data, length);
		printf("\n");
	}
}

static void onIsochronousIN(uint8_t *data, int length)
{
	
#ifdef MEASURE_RECEIVE_RATE
	g_receiveCount++;
	auto now = std::chrono::high_resolution_clock::now();
	//if (std::chrono::duration_cast<std::chrono::hours>(std::chrono::time_point<std::chrono::high_resolution_clock>::max() - g_lastReceiveTime).count() > 1) 
	if (g_receiveCount > 10)
	{
		UpdateStatValue(&receiveRate, std::chrono::duration_cast<std::chrono::microseconds>(now - g_lastReceiveTime).count() / 1000.0);
		if (g_receiveCount % 10 == 0)
		{
			printf("Receive Rate: %.2fms +-%.2fms - Max %.2fms \n", receiveRate.avg, receiveRate.diff, receiveRate.max);
			receiveRate.max = 0;
		}
	}
	g_lastReceiveTime = now;
#endif

#ifdef DEBUG_TRANSFER_IN
	printf("Isochronous IN (%d): ", length);
	data[length] = 0;
	printf("%.*s", length, (char*)data);
//	printBuffer(data, length);
	printf("\n");
#endif

#ifdef DEBUG_BLOBS_IN
/*	if (length == 3 && data[0] == 'N' && data[1] == 'C' && data[2] == 'D')
	{ // No connected detectors
		printf("Marker Tracker reports no connected Marker Detectors!\n");
	}
	else if (length == 3 && data[0] == 'S' && data[1] == 'T' && data[2] == 'L')
	{ // No connected detectors
		printf("Marker Tracker reports stalled Marker Detectors!\n");
	}
	else*/
	{
		const int headerSize = 8;
		static int blobUpdate = 0;
		blobUpdate++;
		int blobCount = (length-headerSize) / 6;
		if (blobCount > 0 || blobUpdate % 100 == 0)	
		{
			printf("Isochronous IN (%d) %.*s %d Blobs: ", length, headerSize, data, blobCount);
//			printBuffer(data+headerSize, length-headerSize);
			for(int i = 0; i < blobCount; i++)
			{
				int pos = headerSize+i*6;
				uint16_t *blobData = (uint16_t*)&data[pos];
				float posX = (double)blobData[0] / 65536.0, posY = (double)blobData[1] / 65536.0, size = (float)data[pos+4]/2, col = data[pos+5];
				printf("(%.1f, %.1f, %.1f) ", posX * 2048, posY * 2048, size);
			}
			printf("\n");
		}
	}
#endif
}

static void onInterruptIN(uint8_t *data, int length)
{
#ifdef MEASURE_RECEIVE_RATE
	g_receiveCount++;
	auto now = std::chrono::high_resolution_clock::now();
	//if (std::chrono::duration_cast<std::chrono::hours>(std::chrono::time_point<std::chrono::high_resolution_clock>::max() - g_lastReceiveTime).count() > 1) 
	if (g_receiveCount > 10)
	{
		UpdateStatValue(&receiveRate, std::chrono::duration_cast<std::chrono::microseconds>(now - g_lastReceiveTime).count() / 1000.0);
		if (g_receiveCount % 10 == 0)
		{
			printf("Receive Rate: %.2fms (%.2fms +-%.2fms) - Max %.2fms \n", receiveRate.cur, receiveRate.avg, receiveRate.diff, receiveRate.max);
			receiveRate.max = 0;
		}
	}
	g_lastReceiveTime = now;
#endif
	
#ifdef DEBUG_TRANSFER_IN
	printf("Interrupt IN (%d): ", length);
	printBuffer(data, length);
	printf("\n");
#endif

#ifdef DEBUG_BLOBS_IN
	if (length == 3 && data[0] == 'N' && data[1] == 'C' && data[2] == 'D')
	{ // No connected detectors
		printf("Marker Tracker reports no connected Marker Detectors!\n");
	}
	else if (length == 3 && data[0] == 'S' && data[1] == 'T' && data[2] == 'L')
	{ // No connected detectors
		printf("Marker Tracker reports stalled Marker Detectors!\n");
	}
	else
	{
		static int blobUpdate = 0;
		blobUpdate++;
		int blobCount = (length-4) / 6;
		char message[5] = {0};
		memcpy(message, data, 4);
		if (blobCount > 0 || blobUpdate % 10 == 0)	
		{
			printf("Interrupt IN (%d) %s %d Blobs: ", length, message, blobCount);
			printBuffer(data+4, length-4);
			printf("\n");
		}
	}
#endif
}


/*

inline void ErasePacketLine();
inline void StatValues(int channel, float yaw, float pitch, float roll);
inline void StatPacketNoneReceived();
inline void StatPacketDropped();
inline void StatPacketReceived();
inline float BytesToFloat(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3);

// Stats for packet rates

long g_packetsReceived = 0;
long g_packetsDropped = 0;
long g_packetsNone = 0;

std::chrono::time_point<std::chrono::steady_clock> g_lastPacketClock;
double g_lastPacketInterval = 0.0;

double g_avgPacketInterval = 0.0;
double g_minPacketInterval = 1000.0;
double g_maxPacketInterval = 0.0;

double g_highAvgPacketInterval = 0.0;
long g_highAvgPackets = 0;

// Stats for Value Jitter / Accuracy

long g_lastMove = 0, g_numMoves = 0;
double g_startYaw = 0, g_avgYaw = 0, g_diffYaw = 0;
double g_startPitch = 0, g_avgPitch = 0, g_diffPitch = 0;
double g_startRoll = 0, g_avgRoll = 0, g_diffRoll = 0;


inline void ErasePacketLine()
{
	if (g_packetsReceived > 100)
	{
		printf("\0337");
		printf("\n");          
		printf("                                                  \n");
		printf("                                                                                \n");
		printf("                                                                                                              \n");
		printf("\0338");
	}
}

inline void StatValues(int channel, float yaw, float pitch, float roll)
{
	int restDelay = 500;

	if (abs(yaw - g_avgYaw) > 5 || abs(pitch - g_avgPitch) > 5 || abs(roll - g_avgRoll) > 5)
	{
		g_lastMove = g_packetsReceived;
		g_numMoves++;
	}

	if (g_packetsReceived - g_lastMove <= restDelay)
	{
		g_startYaw = g_avgYaw = yaw;
		g_startPitch = g_avgPitch = pitch;
		g_startRoll = g_avgRoll = roll;
	}
	else
	{
		g_avgYaw += (yaw - g_avgYaw) / (g_packetsReceived - g_lastMove - restDelay) * 2;
		g_avgPitch += (pitch - g_avgPitch) / (g_packetsReceived - g_lastMove - restDelay) * 2;
		g_avgRoll += (roll - g_avgRoll) / (g_packetsReceived - g_lastMove - restDelay) * 2;
		g_diffYaw += (abs(yaw - g_avgYaw) - g_diffYaw) / g_packetsReceived;
		g_diffPitch += (abs(pitch - g_avgPitch) - g_diffPitch) / g_packetsReceived;
		g_diffRoll += (abs(roll - g_avgRoll) - g_diffRoll) / g_packetsReceived;
	}

	if (g_packetsReceived > 10)
	{
		ErasePacketLine();
		printf("\0337");
		printf("\n");
		printf("CH %d:  Y: %.2f  P: %.2f  R: %.2f  (%.2fms) \n",
			channel, yaw, pitch, roll, g_lastPacketInterval);
		printf("STATS: %d (%.1fms / %.1fms / %.1fms) - d: %d (%.2f%%) - n: %d (%.2f%%) \n",
			g_packetsReceived, g_minPacketInterval, g_avgPacketInterval, g_maxPacketInterval,
			g_packetsDropped, static_cast<double>(g_packetsDropped) / (g_packetsDropped + g_packetsReceived) * 100.0,
			g_packetsNone, static_cast<double>(g_packetsNone) / (g_packetsReceived) * 100.0);
		printf("VALUES: Y: %.2f +-%.3f |%.3f - P: %.2f +-%.3f |%.3f - R: %.2f +-%.3f |%.3f - HIGH: %d (%.2fms) - Moves: %d \n",
			g_avgYaw, g_diffYaw, g_avgYaw - g_startYaw,
			g_avgPitch, g_diffPitch, g_avgPitch - g_startPitch,
			g_avgRoll, g_diffRoll, g_avgRoll - g_startRoll,
			g_highAvgPackets, g_highAvgPacketInterval, g_numMoves);
		printf("\0338");
	}
}

inline void StatPacketNoneReceived()
{
	g_packetsNone++;
}

inline void StatPacketDropped() 
{
	g_packetsDropped++;
}

inline void StatPacketReceived()
{
	g_lastPacketInterval = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - g_lastPacketClock).count() / 1000.0;
	g_lastPacketClock = std::chrono::high_resolution_clock::now();

	g_packetsReceived++;
	if (g_packetsReceived > 100)
	{
		if (g_lastPacketInterval > 10)
		{
			//ErasePacketLine();
			//printf("%f > %f \n", g_lastPacketInterval, g_avgPacketInterval);
			g_highAvgPackets++;
			g_highAvgPacketInterval += (g_lastPacketInterval - g_highAvgPacketInterval) / g_highAvgPackets;
		}
		else
		{
			g_avgPacketInterval += (g_lastPacketInterval - g_avgPacketInterval) / (g_packetsReceived - g_highAvgPackets);
			g_maxPacketInterval = std::max(g_maxPacketInterval, g_lastPacketInterval);
		}
		g_minPacketInterval = std::min(g_minPacketInterval, g_lastPacketInterval);
	}
	else 
	{
		g_avgPacketInterval = g_lastPacketInterval;
	}
}
 
inline float BytesToFloat(unsigned char b0, unsigned char b1, unsigned char b2, unsigned char b3)
{
	float output;
	unsigned char bytes[] = { b0, b1, b2, b3 };
	memcpy(&output, &bytes, sizeof(output));
	return output;
}*/