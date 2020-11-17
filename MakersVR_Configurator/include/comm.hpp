/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef COMM_H
#define COMM_H

#include <atomic>

// Opaque struct
struct libusb_state;

struct CommState
{
	libusb_state *libusb;
	std::atomic<bool> usbDeviceActive = { false };
	std::atomic<bool> intINPending = { false };
	std::atomic<bool> ctrlINPending = { false };
	std::atomic<bool> ctrlOUT1Pending = { false };
	std::atomic<bool> ctrlOUT2Pending = { false };

	void *userData;
	void (*onControlResponse)(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length, void *userData);
	void (*onIsochronousIN)(uint8_t *data, int length, void *userData);
	void (*onInterruptIN)(uint8_t *data, int length, void *userData);
};

bool comm_init(CommState *state);
bool comm_check_device(CommState *state);
bool comm_connect(CommState *state, bool altSetting);
bool comm_disconnect(CommState *state);
bool comm_startStream(CommState *state);
bool comm_stopStream(CommState *state);
void comm_exit(CommState *state);

bool comm_submit_control_request(CommState *state, uint8_t request, uint16_t value, uint16_t index);
bool comm_submit_control_data(CommState *state, uint8_t request, uint16_t value, uint16_t index, void* data = NULL, uint8_t size = 0);

#endif // COMM_H