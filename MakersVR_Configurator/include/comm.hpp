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
	std::atomic<bool> usbDeviceActive;
	std::atomic<bool> intINPending;
	std::atomic<bool> ctrlINPending;
	std::atomic<bool> ctrlOUTPending;

	void (*onControlResponse)(uint8_t request, uint16_t value, uint16_t index, uint8_t *data, int length);
	void (*onIsochronousIN)(uint8_t *data, int length);
	void (*onInterruptIN)(uint8_t *data, int length);
};

bool comm_init(CommState *state);
bool comm_check_device(CommState *state);
bool comm_connect(CommState *state, bool altSetting);
bool comm_disconnect(CommState *state);
void comm_exit(CommState *state);

bool comm_submit_control_request(CommState *state, uint8_t request, uint16_t value, uint16_t index);
uint8_t* comm_get_control_data(); // Returns the buffer to write control data to
bool comm_submit_control_data(CommState *state, uint8_t request, uint16_t value, uint16_t index);

#endif // COMM_H