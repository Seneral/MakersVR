/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "comm.h"
#include "util.h"

#define DEBUG_DEVICE_DESC
#define MEASURE_STALL_RECOVERY

#pragma warning(disable : 4200)
#include "libusb/libusb.h"

#include <iostream>
#include <thread>

#ifdef MEASURE_STALL_RECOVERY
#include <chrono>
#include <ctime>
#endif

#define ISO_PACKET_SIZE		128
#define NUM_PACKETS			1
#define NUM_TRANSFERS		32


// Descriptor Defines
#define USBD_VID						5824
#define USBD_PID						1503
#define USBD_MANUFACTURER_STRING		"Seneral seneral.dev"
#define USBD_PRODUCT_STRING				"MakersVR Device"
#define USBD_INTERFACE					0

struct libusb_state
{
	libusb_context *context;
	libusb_device_handle *devHandle;
	libusb_transfer *intIN;
	libusb_transfer *isoIN[NUM_TRANSFERS];
	libusb_transfer *controlIN;
	libusb_transfer *controlOUT;
	std::thread *usbEventHandlerThread;
	std::atomic<bool> usbEventHandlerRun;
	std::atomic<bool> usbDeviceConnected;
	std::atomic<bool> usbAltSettingISO;
	std::atomic<bool> isoINSubmittedALL;
	std::atomic<bool> isoINSubmitted[NUM_TRANSFERS];
	std::atomic<libusb_transfer*> isoINStalled;
	std::atomic<libusb_transfer*> isoINBackup;
};

static void usbEventHandler(CommState *state);
static void tryFreeTransfer(libusb_transfer *transfer);

static void onIsochronousIN(libusb_transfer *transfer);
static void onInterruptIN(libusb_transfer *transfer);
static void onControlSent(libusb_transfer *transfer);
static void onControlResponse(libusb_transfer *transfer);

alignas(2) uint8_t intINBuf[64];
alignas(2) uint8_t isoINBuf[NUM_TRANSFERS][ISO_PACKET_SIZE*NUM_PACKETS];
alignas(2) uint8_t ctrlINBuf[64+8];
alignas(2) uint8_t ctrlOUTBuf[64+8];

#ifdef MEASURE_STALL_RECOVERY
std::chrono::time_point<std::chrono::steady_clock> g_lastStallStart;
std::atomic<bool> g_stalling;
static StatValue g_stallRecovery;
std::atomic<int> g_stallTransferCount;
#endif


bool comm_init(CommState *state)
{
	if (state->libusb != NULL) return true;
	memset(state, 0, sizeof(state));
	libusb_context *context;
	if (libusb_init(&context) == 0)
	{ // Successful init
		libusb_set_debug(context, LIBUSB_LOG_LEVEL_NONE);
		state->libusb = (libusb_state*)malloc(sizeof(libusb_state));
		memset(state->libusb, 0, sizeof(state->libusb));
		state->libusb->usbDeviceConnected = false;
		state->libusb->usbEventHandlerRun = false;
		state->libusb->usbAltSettingISO = false;
		state->libusb->context = context;
		return true;
	}
	return false;
}

bool comm_check_device(CommState *state)
{
	static char strBuf[128];
	memset(strBuf, 0, sizeof(strBuf));

	libusb_device **devList;
	libusb_device *device;
	libusb_device_descriptor devDesc;
	libusb_device_handle *devHandle = NULL;
	
	ssize_t devCount = libusb_get_device_list(state->libusb->context, &devList);
	if (devCount < 0) printf("ERROR: Failed to list devices!\n");

	for (int i = 0; i < devCount; i++)
	{
		device = devList[i];
		if (libusb_get_device_descriptor(device, &devDesc) < 0) continue;
		if (devDesc.idVendor == USBD_VID && devDesc.idProduct == USBD_PID)
		{ // Candidate found - check with Manufacturer String match
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			int e = libusb_open(device, &devHandle);
			if (e != 0) {
				printf("ERROR: Failed to open device: %s!\n", libusb_error_name(e));
				continue;
			}
			libusb_get_string_descriptor_ascii(devHandle, devDesc.iManufacturer, (unsigned char*)strBuf, sizeof(strBuf));
			if (strcmp((const char*)strBuf, USBD_MANUFACTURER_STRING) == 0) break; // Found our device
			libusb_close(devHandle);
			devHandle = NULL;
		}
	}
	libusb_free_device_list(devList, 1);
	if (devHandle == NULL) return false;

	/* Found valid device */

#ifdef DEBUG_DEVICE_DESC /* Print device identification */

	// Product strings
	printf("VID: %d; PID: %d; Manufacturer: '%s'; ", devDesc.idVendor, devDesc.idProduct, strBuf); 
	libusb_get_string_descriptor_ascii(devHandle, devDesc.iProduct, (unsigned char*)strBuf, sizeof(strBuf));
	printf("Product: '%s'\n", strBuf);

	// Config and interface overview
	libusb_config_descriptor *configDesc;
	libusb_get_active_config_descriptor(device, &configDesc);
	for (int i = 0; i < configDesc->bNumInterfaces; i++)
	{
		const libusb_interface *interface = &configDesc->interface[i];
		for (int a = 0; a < interface->num_altsetting; a++)
		{
			const libusb_interface_descriptor *interfaceDesc = &interface->altsetting[a];
			printf("Interface %d - %d (C %d, SC %d, P %d) has %d endpoints!\n", interfaceDesc->bInterfaceNumber, interfaceDesc->bAlternateSetting, interfaceDesc->bInterfaceClass, interfaceDesc->bInterfaceSubClass, interfaceDesc->bInterfaceProtocol, interfaceDesc->bNumEndpoints);
			for (int e = 0; e < interfaceDesc->bNumEndpoints; e++)
			{
				const libusb_endpoint_descriptor *epDesc = &interfaceDesc->endpoint[e];
				libusb_transfer_type type = (libusb_transfer_type)(epDesc->bmAttributes & 0b11);
				printf("    %s EP %d has packet size %d and polling interval of %dms!\n",
					type == LIBUSB_TRANSFER_TYPE_ISOCHRONOUS? "ISO" : (type == LIBUSB_TRANSFER_TYPE_INTERRUPT? "INT" : "Other"),
					epDesc->bEndpointAddress, epDesc->wMaxPacketSize, epDesc->bInterval);
			}
		}
	}
	libusb_free_config_descriptor(configDesc);
#endif

	state->libusb->devHandle = devHandle;

	return true;
}

bool comm_connect(CommState *state, bool altSetting)
{
	int code;

	// Claim interface
	if (libusb_kernel_driver_active(state->libusb->devHandle, USBD_INTERFACE) == 1)
		libusb_detach_kernel_driver(state->libusb->devHandle, USBD_INTERFACE);
	if ((code = libusb_claim_interface(state->libusb->devHandle, USBD_INTERFACE)) != 0)
	{
		printf("ERROR: Failed to claim interface: %s\n", libusb_error_name(code));
		return false;
	}

	if (altSetting)
	{
		if ((code = libusb_set_interface_alt_setting(state->libusb->devHandle, USBD_INTERFACE, 1)) != 0) {
			printf("ERROR: Failed to set alternate interface: %s\n", libusb_error_name(code));
			libusb_release_interface(state->libusb->devHandle, USBD_INTERFACE);
			return false;
		}
		state->libusb->usbAltSettingISO = true;
		
		for (int i = 0; i < NUM_TRANSFERS; i++)
		{
			state->libusb->isoIN[i] = libusb_alloc_transfer(NUM_PACKETS);
			libusb_fill_iso_transfer(state->libusb->isoIN[i], state->libusb->devHandle, 2 | LIBUSB_ENDPOINT_IN, isoINBuf[i], sizeof(isoINBuf[i]), NUM_PACKETS, &onIsochronousIN, state, NUM_TRANSFERS*10);
			libusb_set_iso_packet_lengths(state->libusb->isoIN[i], ISO_PACKET_SIZE);
		}
	}
	else
	{
		// Setup interrupt transfers
		state->libusb->intIN = libusb_alloc_transfer(0);
		libusb_fill_interrupt_transfer(state->libusb->intIN, state->libusb->devHandle, 1 | LIBUSB_ENDPOINT_IN, intINBuf, sizeof(intINBuf), &onInterruptIN, state, 1000);
	}

	// Setup generic control request expecting a response with data from device
	libusb_fill_control_setup(ctrlINBuf, LIBUSB_RECIPIENT_INTERFACE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN, 0x00, 0x0000, USBD_INTERFACE, 64);
	state->libusb->controlIN = libusb_alloc_transfer(0);
	libusb_fill_control_transfer(state->libusb->controlIN, state->libusb->devHandle, ctrlINBuf, &onControlResponse, state, 4);
	// Setup generic control request sending data to device
	libusb_fill_control_setup(ctrlOUTBuf, LIBUSB_RECIPIENT_INTERFACE | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT, 0x00, 0x0000, USBD_INTERFACE, 64);
	state->libusb->controlOUT = libusb_alloc_transfer(0);
	libusb_fill_control_transfer(state->libusb->controlOUT, state->libusb->devHandle, ctrlOUTBuf, &onControlSent, state, 4);

	// Start usb thread
	state->libusb->usbEventHandlerRun = true;
	state->libusb->usbEventHandlerThread = new std::thread(usbEventHandler, state);

	// Start submitting transfers
	if (state->libusb->usbAltSettingISO)
	{
		code = libusb_submit_transfer(state->libusb->isoIN[0]);
		printf("Submit Isochronous transfer 1 (%p): %s!\n", state->libusb->isoIN[0], libusb_error_name(code));
		state->libusb->isoINSubmittedALL = false;
		state->libusb->isoINSubmitted[0] = true;
		for (int i = 1; i < NUM_TRANSFERS; i++)
		{
			state->libusb->isoINSubmitted[i] = false;
		}
		state->libusb->isoINStalled = NULL;
	}
	else
	{
		int status = libusb_submit_transfer(state->libusb->intIN);
		printf("Interrupt transfer value: %s!\n", libusb_error_name(code));
		state->intINPending = true;
	}
	state->ctrlINPending = false;
	state->ctrlOUTPending = false;

	state->libusb->usbDeviceConnected = true;
	state->usbDeviceActive = true;
	return true;
}

bool comm_disconnect(CommState *state)
{
	if (state->libusb == NULL) return false;
	state->usbDeviceActive = false;
	if (!state->libusb->usbDeviceConnected) return false;
	printf("Disconnecting device...\n");
	state->libusb->usbDeviceConnected = false;

	// Cancel transfers if ongoing
	tryFreeTransfer(state->libusb->intIN);
	for (int i = 0; i < NUM_TRANSFERS; i++)
	{
		tryFreeTransfer(state->libusb->isoIN[i]);	
	}
	tryFreeTransfer(state->libusb->controlIN);
	tryFreeTransfer(state->libusb->controlOUT);
	
	// Start joining thread
	bool join = state->libusb->usbEventHandlerThread->joinable();
	state->libusb->usbEventHandlerRun = false;

	// Disconnect device
	libusb_release_interface(state->libusb->devHandle, USBD_INTERFACE);
	libusb_close(state->libusb->devHandle); // Crashes program for some reason
	
	if (join)
	{ // Finish joining thread
		state->libusb->usbEventHandlerThread->join();
	}
	delete state->libusb->usbEventHandlerThread;

	// Reset everything but context
	libusb_context *context = state->libusb->context;
	memset(state->libusb, 0, sizeof(libusb_state));
	state->libusb->context = context;
	
	printf("Device is disconnected!\n");
	return true;
}

void comm_exit(CommState *state)
{
	if (state->libusb == NULL) return;
	// Exit
	printf("Exiting...\n");
	libusb_exit(state->libusb->context);
	free(state->libusb);
}

bool comm_submit_control_request(CommState *state, uint8_t request, uint16_t value, uint16_t index)
{
	if (!state->ctrlINPending)
	{
		state->ctrlINPending = true;
		struct libusb_control_setup *setup = (struct libusb_control_setup *)ctrlINBuf;
		setup->bRequest = request;
		setup->wValue = value;
		setup->wIndex = index;
		libusb_submit_transfer(state->libusb->controlIN);
		return true;
	}
	return false;
}

uint8_t* comm_get_control_data()
{
	return ctrlOUTBuf;
}

bool comm_submit_control_data(CommState *state, int request, int value, int index)
{
	if (!state->ctrlOUTPending)
	{
		state->ctrlOUTPending = true;
		struct libusb_control_setup *setup = (struct libusb_control_setup *)ctrlOUTBuf;
		setup->bRequest = request;
		setup->wValue = value;
		setup->wIndex = index;
		libusb_submit_transfer(state->libusb->controlOUT);
		return true;
	}
	return false;
}

static void onControlSent(libusb_transfer *transfer)
{
	((CommState*)transfer->user_data)->ctrlOUTPending = false;
}

static void onControlResponse(libusb_transfer *transfer)
{
	CommState *state = (CommState*)transfer->user_data;
	state->ctrlINPending = false;

	switch (transfer->status)
	{
	case LIBUSB_TRANSFER_COMPLETED: break;
	case LIBUSB_TRANSFER_TIMED_OUT: return;
	case LIBUSB_TRANSFER_ERROR:
		state->usbDeviceActive = false;
		return;
	default:
		printf("Control IN Transfer completed with status code: %s!\n", libusb_error_name(transfer->status));
		return;
	}
	
	struct libusb_control_setup *setup = (struct libusb_control_setup *)ctrlINBuf;
	if (state->onControlResponse != NULL)
		state->onControlResponse(setup->bRequest, setup->wValue, setup->wIndex, &transfer->buffer[8], transfer->actual_length);
}

static void onIsoINSubmission(libusb_transfer *transfer)
{
	CommState *state = (CommState*)transfer->user_data;
	state->libusb->isoINSubmittedALL = true;
	int code;
	for (int i = 0; i < NUM_TRANSFERS; i++)
	{
		if (state->libusb->isoINSubmitted[i] == false)
		{
			code = libusb_submit_transfer(state->libusb->isoIN[i]);
			printf("Submit Isochronous transfer %d (%p): %s!\n", i, state->libusb->isoIN[i], libusb_error_name(code));
			state->libusb->isoINSubmitted[i] = true;
//			return;
		}
	}

	if (state->libusb->isoINBackup == NULL)
		state->libusb->isoINBackup = state->libusb->isoIN[1];
}
static void onIsochronousIN(libusb_transfer *transfer)
{
	CommState *state = (CommState*)transfer->user_data;

/*	if (state->libusb->isoINBackup == transfer)
	{ // Expected, this backup is always ran behind the scenes with ContinueTransfer=False
		if (transfer->status == LIBUSB_TRANSFER_STALL)
			transfer->callback = NULL; // Signal to not resubmit this transfer
	}*/

//	printf("Isochronous IN Transfer (%p) completed with status code: %s!\n", transfer, libusb_error_name(transfer->status));
	
	switch (transfer->status)
	{
	case LIBUSB_TRANSFER_COMPLETED: break;
	case LIBUSB_TRANSFER_STALL:
		// Isochronous stream got interrupted OR just started (only relevant for libusb WinUSB backend)
		// For the case that this is the first transfer and the stream was not started yet:
		// 		The current transfer is isoIN1, and it will get resubmitted by libusb AFTER this callback
		// 		We have the chance to submit all our other transfers right AFTER the isoIN1 is get resubmitted
		// 		By overwriting the transfer callback temporarily
		if (state->libusb->isoINStalled != NULL)
		{ // Already handled stall before, stream should already be on it's way
//			printf("(%p) detected continues stall! Sending signal NULL!\n", transfer);
//			transfer->callback = NULL; // Signal to not resubmit this transfer
//			libusb_submit_transfer(transfer);

			/*state->libusb->isoINSubmittedALL = false;
			for (int i = 0; i < NUM_TRANSFERS; i++)
			{
				if (state->libusb->isoIN[i] == transfer)
				{
					state->libusb->isoINSubmitted[i] = false;
					break;
				}
			}*/
		}
		else
		{ // Have not handled stall before, resubmit this transfer to initiate new stream
			printf("(%p) reported stream interruption!\n", transfer);
	#ifdef MEASURE_STALL_RECOVERY
			g_lastStallStart = std::chrono::high_resolution_clock::now();
			g_stallTransferCount = 0;
	#endif
			state->libusb->isoINStalled = transfer;
			if (!state->libusb->isoINSubmittedALL)
			{ // Some transfers are not (yet) submitted, do so after this one has been resubmitted with ContinueStream=False
				transfer->callback = onIsoINSubmission;
			}
		}
	#ifdef MEASURE_STALL_RECOVERY
		g_stalling = true;
		g_stallTransferCount++;
	#endif
		return;
	case LIBUSB_TRANSFER_TIMED_OUT:
		libusb_submit_transfer(transfer);
		return;
	case LIBUSB_TRANSFER_ERROR:
		state->usbDeviceActive = false;
		return;
	case LIBUSB_TRANSFER_CANCELLED: return;
	default:
		printf("Isochronous IN Transfer completed with status code: %s!\n", libusb_error_name(transfer->status));
		return;
	}
	if (state->libusb->isoINStalled == transfer)
	{ // Successfully unstalled previous stall
		state->libusb->isoINStalled = NULL;
		/*if (!state->libusb->isoINSubmittedALL)
		{ // Some transfers are not (yet) submitted, do so after this one has been resubmitted with ContinueStream=False
			onIsoINSubmission(transfer);
		}*/
	}
#ifdef MEASURE_STALL_RECOVERY
	if (g_stalling)
	{
		auto now = std::chrono::high_resolution_clock::now();
		float recoveryTime = std::chrono::duration_cast<std::chrono::microseconds>(now - g_lastStallStart).count() / 1000.0f;
		UpdateStatValue(&g_stallRecovery, recoveryTime);
		printf("(%p) recovered from stall affecting %d transfers: %.2fms (%.2fms +-%.2fms) - Max %.2fms \n", transfer, g_stallTransferCount.load(), g_stallRecovery.cur, g_stallRecovery.avg, g_stallRecovery.diff, g_stallRecovery.max);
		g_stalling = false;
	}
#endif

	
	for (int i = 0; i < transfer->num_iso_packets; i++) 
	{
		libusb_iso_packet_descriptor *packet = &transfer->iso_packet_desc[i];
//		printf("Isochronous IN Transfer Packet %d completed with status code: %s!\n", i, libusb_error_name(packet->status));
		switch (packet->status)
		{
		case LIBUSB_TRANSFER_COMPLETED: break;
		case LIBUSB_TRANSFER_ERROR: continue;
		default:
			printf("Isochronous IN Transfer Packet %d completed with status code: %s (%u)!\n", i, libusb_error_name(packet->status), packet->status);
			continue;
		}
		if (packet->actual_length != 0)
		{
			uint8_t* buf = (uint8_t*)libusb_get_iso_packet_buffer(transfer, i);
//			printf("Iso IN %d: %.*s \n", i, packet->actual_length, buf);
			if (state->onIsochronousIN != NULL)
				state->onIsochronousIN(buf, packet->actual_length);
		}
	}

	libusb_submit_transfer(transfer);
}
static void onInterruptIN(libusb_transfer *transfer)
{
	CommState *state = (CommState*)transfer->user_data;
	state->intINPending = false;

	switch (transfer->status)
	{
	case LIBUSB_TRANSFER_COMPLETED: break;
	case LIBUSB_TRANSFER_TIMED_OUT:
		state->intINPending = true;
		libusb_submit_transfer(state->libusb->intIN);
		return;
	case LIBUSB_TRANSFER_ERROR:
		state->usbDeviceActive = false;
		return;
	case LIBUSB_TRANSFER_CANCELLED: return;
	default:
		printf("Interrupt IN Transfer completed with status code: %s!\n", libusb_error_name(transfer->status));
		return;
	}
	
	if (state->onInterruptIN != NULL)
		state->onInterruptIN(transfer->buffer, transfer->actual_length);

	state->intINPending = true;
	libusb_submit_transfer(state->libusb->intIN);
}


static void usbEventHandler(CommState *state)
{
	while (state->libusb->usbEventHandlerRun)
	{
		libusb_handle_events(state->libusb->context);		
	}
}

static void tryFreeTransfer(libusb_transfer *transfer)
{
	if (transfer == NULL) return;
	int cnt = 1000;
	if (libusb_cancel_transfer(transfer) == 0)
		while (transfer->status != LIBUSB_TRANSFER_CANCELLED && cnt-- > 0);
	libusb_free_transfer(transfer);
}