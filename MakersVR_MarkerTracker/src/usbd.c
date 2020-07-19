/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "usbd.h"

#define USBD_INTERFACE_NUM 0		// Number of default interface
#define USBD_CONFIGURATION_NUM 1	// Number of default configuration (must be over 0)

#define USBD_EP_CTRL_SIZE 64			// Control Endpoint number 0 (could be set higher)
#define USBD_EP_INT_IN_ADDR (0x80 | 1)	// Endpoint number 1, direction IN (Device to host)
#define USBD_EP_INT_IN_SIZE 64	 		// Packet size of interrupt endpoint IN
#define USBD_EP_ISO_IN_ADDR (0x80 | 2)	// Endpoint number 2, direction IN (Host to device)
#define USBD_EP_ISO_IN_SIZE 128			// Packet size of isochronous endpoint IN

#define WCID_VENDOR_CODE 0x20			// Anything non-zero should work

// State
static usbd_device hUSB;
static usbd_callbacks *impl_callbacks;
static int usbd_alt_interface;
TimePoint lastSOF;
TimePoint lastISO;
bool usbd_TX_ready;
// Used for holding received packets from control endpoint
static uint32_t ctl_buffer[USBD_EP_CTRL_SIZE/4];

/* USB string descriptors */
static const struct usb_string_descriptor str_desc_wcid = USB_ARRAY_DESC('M', 'S', 'F', 'T', '1', '0', '0', WCID_VENDOR_CODE);
static const struct usb_string_descriptor str_desc_lang = USB_ARRAY_DESC(USB_LANGID_ENG_US);
static const struct usb_string_descriptor str_desc_manufacturer = USB_STRING_DESC("Seneral seneral.dev");
static const struct usb_string_descriptor str_desc_product = USB_STRING_DESC("MakersVR Device");
static const struct usb_string_descriptor *const str_desc_table[] = {
	&str_desc_lang,
	&str_desc_manufacturer,
	&str_desc_product,
};

/* USB device descriptor */
static const struct usb_device_descriptor USBD_Device_Desc = {
	.bLength = sizeof(struct usb_device_descriptor),
	.bDescriptorType = USB_DTYPE_DEVICE,
	.bcdUSB = VERSION_BCD(2, 0, 0),
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 5824,
	.idProduct = 1503,
	.bcdDevice = VERSION_BCD(1, 0, 0),
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = INTSERIALNO_DESCRIPTOR,
	.bNumConfigurations = 1,
};

/* Structure of custom device configuration descriptor */
struct usb_config
{
	struct usb_config_descriptor config;
	struct usb_interface_descriptor interface_0_0_int;
		struct usb_endpoint_descriptor ep_int_in;
	struct usb_interface_descriptor interface_0_1_iso;
		struct usb_endpoint_descriptor ep_iso_in;
} __attribute__((packed));

/* USB configuration descriptor */
static const struct usb_config USBD_Config_Desc = {
	.config = {
		.bLength = sizeof(struct usb_config_descriptor),
		.bDescriptorType = USB_DTYPE_CONFIGURATION,
		.wTotalLength = sizeof(struct usb_config),
		.bNumInterfaces = 1,
		.bConfigurationValue = USBD_CONFIGURATION_NUM,
		.iConfiguration = NO_DESCRIPTOR,
		.bmAttributes = USB_CFG_ATTR_RESERVED,
		.bMaxPower = USB_CFG_POWER_MA(100),
	},
	.interface_0_0_int = {
		.bLength = sizeof(struct usb_interface_descriptor),
		.bDescriptorType = USB_DTYPE_INTERFACE,
		.bInterfaceNumber = USBD_INTERFACE_NUM,
		.bAlternateSetting = 0,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_VENDOR,
		.bInterfaceSubClass = USB_SUBCLASS_NONE,
		.bInterfaceProtocol = USB_PROTO_NONE,
		.iInterface = NO_DESCRIPTOR,
	},
	.ep_int_in = {
		.bLength = sizeof(struct usb_endpoint_descriptor),
		.bDescriptorType = USB_DTYPE_ENDPOINT,
		.bEndpointAddress = USBD_EP_INT_IN_ADDR,
		.bmAttributes = USB_EPTYPE_INTERRUPT,
		.wMaxPacketSize = USBD_EP_INT_IN_SIZE,
		.bInterval = 1,
	},
	.interface_0_1_iso = {
		.bLength = sizeof(struct usb_interface_descriptor),
		.bDescriptorType = USB_DTYPE_INTERFACE,
		.bInterfaceNumber = USBD_INTERFACE_NUM,
		.bAlternateSetting = 1,
		.bNumEndpoints = 1,
		.bInterfaceClass = USB_CLASS_VENDOR,
		.bInterfaceSubClass = USB_SUBCLASS_NONE,
		.bInterfaceProtocol = USB_PROTO_NONE,
		.iInterface = NO_DESCRIPTOR,
	},
	.ep_iso_in = {
		.bLength = sizeof(struct usb_endpoint_descriptor),
		.bDescriptorType = USB_DTYPE_ENDPOINT,
		.bEndpointAddress = USBD_EP_ISO_IN_ADDR,
		.bmAttributes = USB_EPTYPE_ISOCHRONUS | USB_EPATTR_NO_SYNC | USB_EPUSAGE_DATA,
		.wMaxPacketSize = USBD_EP_ISO_IN_SIZE,
		.bInterval = 1,
	}
};

/*
	WCID Compat ID descriptor
*/
struct usb_compatid_header
{
	uint32_t dwLength;
	uint16_t bcdVersion;
	uint16_t wIndex;
	uint8_t bCount;
	uint8_t bReserved0[7];
} __attribute__((packed));
struct usb_compatid_section
{
	uint8_t bFirstInterfaceNumber;
	uint8_t bReserved1;
	uint8_t bCompatibleID[8];
	uint8_t bSubCompatibleID[8];
	uint8_t bReserved0[6];
} __attribute__((packed));
struct usb_compatid_descriptor
{
	struct usb_compatid_header header;
	struct usb_compatid_section section0;
} __attribute__((packed));
static const struct usb_compatid_descriptor WCID_CompatID_Desc = {
	.header = {
		.dwLength = sizeof(struct usb_compatid_descriptor),
		.bcdVersion = VERSION_BCD(1, 0, 0),
		.wIndex = 0x0004, // Index of compatID descriptor defined by WCID spec
		.bCount = 0x01,	// number of sections to follow
		.bReserved0 = {0}
	},
	.section0 = {
		.bFirstInterfaceNumber = USBD_INTERFACE_NUM, // Interface (feature group) to load driver for
		.bReserved1 = 0x01,
		.bCompatibleID = "WINUSB", // This tells windows to load the WinUSB drivers for this feature group
		.bSubCompatibleID = {0},
		.bReserved0 = {0}
	}
};

/* USB callbacks */

static void usbd_SOF_CB(usbd_device *usbd, uint8_t event, uint8_t ep)
{
	SetTimePoint(&lastSOF);
}

static void usbd_EP_INT_IN_CB(usbd_device *usbd, uint8_t event, uint8_t ep)
{
//	DEBUG_CHARR('/', 'I', 'N', 'T', '0' + (ep&0xF), 'T');
	usbd_TX_ready = true;
}

static void usbd_EP_ISO_IN_CB(usbd_device *usbd, uint8_t event, uint8_t ep)
{
	// Note time of current iso finish
	SetTimePoint(&lastISO);
	// Clear buffer that was sent so it doesn't get send again
	usbd_ep_write(usbd, USBD_EP_ISO_IN_ADDR, NULL, 0);
	// Allow writing to TX buffer again
	usbd_TX_ready = true;

	// Toggle DTOG to always use the same buffer
//	*reg |= USB_EP_DTOG_TX;

/*	static int index = 0;
	if (index++ % 15 == 0)
	{
		static uint8_t usbSendBuffer[32];
		usbSendBuffer[0] = 'T';
		usbSendBuffer[1] = 'E';
		usbSendBuffer[2] = 'S';
		usbSendBuffer[3] = 'T';
		usbSendBuffer[4] = ':';
		usbSendBuffer[5] = '0' + index/10000;
		usbSendBuffer[6] = '0' + (index%10000)/1000;
		usbSendBuffer[7] = '0' + (index%1000)/100;
		usbSendBuffer[8] = '0' + (index%100)/10;
		usbSendBuffer[9] = '0' + index%10;
		usbSendBuffer[10] = '/';
		usbSendBuffer[11] = 'M';
		usbSendBuffer[12] = 'S';
		usbSendBuffer[13] = ':';
		usbSendBuffer[14] = '0' + msCounter/1000000;
		usbSendBuffer[15] = '0' + (msCounter%1000000)/100000;
		usbSendBuffer[16] = '0' + (msCounter%100000)/10000;
		usbSendBuffer[17] = '0' + (msCounter%10000)/1000;
		usbSendBuffer[18] = '0' + (msCounter%1000)/100;
		usbSendBuffer[19] = '0' + (msCounter%100)/10;
		usbSendBuffer[20] = '0' + msCounter%10;
		uint_fast16_t us = getUS();
		usbSendBuffer[21] = '/';
		usbSendBuffer[22] = 'U';
		usbSendBuffer[23] = 'S';
		usbSendBuffer[24] = ':';
		usbSendBuffer[25] = '0' + (us%1000)/100;
		usbSendBuffer[26] = '0' + (us%100)/10;
		usbSendBuffer[27] = '0' + us%10;
		usbd_ep_write(usbd, USBD_EP_ISO_IN_ADDR, usbSendBuffer, 32);
	}*/

//	DEBUG_CHARR('/', 'I', 'S', 'O', '0' + (ep&0xF), 'T');
}

/* USB class implementation functions */
static usbd_respond class_impl_getdesc(usbd_ctlreq *req, void **address, uint16_t *length)
{
	switch (req->wValue >> 8)
	{
	/* Standard USB descriptors, implementation-specific */
	case USB_DTYPE_DEVICE:
		*address = (void*)&USBD_Device_Desc;
		*length = sizeof(USBD_Device_Desc);
		return usbd_ack;
	case USB_DTYPE_CONFIGURATION:
		*address = (void*)&USBD_Config_Desc;
		*length = sizeof(USBD_Config_Desc);
		return usbd_ack;
	case USB_DTYPE_STRING:
		if ((req->wValue & 0xFF) == 0xEE)
		{ // Windows WCID descriptor, this initiates the WCID process after Windows gets a response
			*address = (void*)&str_desc_wcid;
			*length = str_desc_wcid.bLength;
			return usbd_ack;
		}
		if ((req->wValue & 0xFF) >= sizeof(str_desc_table) / sizeof(struct usb_string_descriptor *))
			return usbd_fail; // No such string descriptor
		*address = (void*)str_desc_table[req->wValue & 0xFF];
		*length = str_desc_table[req->wValue & 0xFF]->bLength;
		return usbd_ack;
	/* Class-specific descriptors are usually targetted at interface (handled in control) */
	default:
		return usbd_fail;
	}
}

static usbd_respond class_impl_setinterface(usbd_device *usbd, uint8_t interface)
{
	usbd_alt_interface = interface;
	if (interface == 0)
	{
		// Deconfigure isochronous
		usbd_ep_deconfig(usbd, USBD_EP_ISO_IN_ADDR);
		usbd_reg_endpoint(usbd, USBD_EP_ISO_IN_ADDR, 0);
		// Configure interrupt
		if (!usbd_ep_config(usbd, USBD_EP_INT_IN_ADDR, USB_EPTYPE_INTERRUPT, USBD_EP_INT_IN_SIZE)) DEBUG_STR("/IntRecfgFailed");
		usbd_reg_endpoint(usbd, USBD_EP_INT_IN_ADDR, usbd_EP_INT_IN_CB);
		return usbd_ack;
	}
	else if (interface == 1)
	{
		// Deconfigure interrupt
		usbd_ep_deconfig(usbd, USBD_EP_INT_IN_ADDR);
		usbd_reg_endpoint(usbd, USBD_EP_INT_IN_ADDR, 0);
		// Configure isochronous
		if (!usbd_ep_config(usbd, USBD_EP_ISO_IN_ADDR, USB_EPTYPE_ISOCHRONUS, USBD_EP_ISO_IN_SIZE)) DEBUG_STR("/IsoRecfgFailed");
		usbd_reg_endpoint(usbd, USBD_EP_ISO_IN_ADDR, usbd_EP_ISO_IN_CB);
		return usbd_ack;
	}
	return usbd_fail;	
}

/*
	Incoming request on control pipeline (endpoint 0 OUT)
	Handle standard class requests and all non-standard requests
*/
static usbd_respond class_impl_control(usbd_device *usbd, usbd_ctlreq *req, usbd_rqc_callback *callback)
{
	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_STANDARD
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_INTERFACE
		&& req->bRequest == USB_STD_GET_INTERFACE
		&& req->wIndex == USBD_INTERFACE_NUM)
	{ // Standard Get Interface Request (for alternative ISO transfer)
		usbd->status.data_ptr = &usbd_alt_interface;
		usbd->status.data_count = 1;
		return usbd_ack;
	}

	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_STANDARD
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_INTERFACE
		&& req->bRequest == USB_STD_SET_INTERFACE
		&& req->wIndex == USBD_INTERFACE_NUM)
	{ // Standard Set Interface Request (to set to alternative ISO transfer)
		if (req->wValue == 0 || req->wValue == 1)
		{
			if (req->wValue != usbd_alt_interface)
				class_impl_setinterface(usbd, req->wValue);
			return usbd_ack;
		}
		return usbd_fail;
	}

	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_VENDOR
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_DEVICE
		&& req->bRequest == WCID_VENDOR_CODE
		&& req->wIndex == 0x0004)
	{ // WCID compatible ID request (part of WinUSB support for windows)
		usbd->status.data_ptr = (void*)&WCID_CompatID_Desc;
		usbd->status.data_count = sizeof(WCID_CompatID_Desc);
		return usbd_ack;
	}

	if ((USB_REQ_TYPE & req->bmRequestType) == USB_REQ_VENDOR
		&& (USB_REQ_RECIPIENT & req->bmRequestType) == USB_REQ_INTERFACE
		&& req->wIndex == USBD_INTERFACE_NUM)
	{ // Vendor-specific requests (for the only interface)
		if (req->bmRequestType & USB_REQ_DEVTOHOST)
			return impl_callbacks->usbd_ctrl_rsp(usbd, req);
		else
			return impl_callbacks->usbd_ctrl_rcv(usbd, req);
	}

	return usbd_fail;
}

static usbd_respond class_impl_setconf(usbd_device *usbd, uint8_t cfg)
{
	switch (cfg)
	{
	case 0: // Deconfigure
		// Deconfigure endpoints
		usbd_ep_deconfig(usbd, USBD_EP_INT_IN_ADDR);
		usbd_ep_deconfig(usbd, USBD_EP_ISO_IN_ADDR);
		// Reset callbacks
		usbd_reg_endpoint(usbd, USBD_EP_INT_IN_ADDR, 0);
		usbd_reg_endpoint(usbd, USBD_EP_ISO_IN_ADDR, 0);
		return usbd_ack;
	case USBD_CONFIGURATION_NUM: // Configuration 1
		// Setup default interrupt endpoints and callback
		usbd_ep_config(usbd, USBD_EP_INT_IN_ADDR, USB_EPTYPE_INTERRUPT, USBD_EP_INT_IN_SIZE);
		usbd_reg_endpoint(usbd, USBD_EP_INT_IN_ADDR, usbd_EP_INT_IN_CB);
		return usbd_ack;
	default:
		return usbd_fail;
	}
}

void usbd_impl_init(usbd_callbacks *callbacks)
{
	impl_callbacks = callbacks;
	// Setup interrupt
	NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);	
	// Init with hardware driver usbd_hw and control endpoint setup
	usbd_init(&hUSB, &usbd_hw, USBD_EP_CTRL_SIZE, ctl_buffer, sizeof(ctl_buffer));
	// Register class implementation functions
	usbd_reg_config(&hUSB, class_impl_setconf);
	usbd_reg_control(&hUSB, class_impl_control);
	usbd_reg_descr(&hUSB, class_impl_getdesc);
	usbd_reg_event(&hUSB, usbd_evt_sof, usbd_SOF_CB);
	usbd_enable(&hUSB, true);
	//usbd_connect(usbd, true); // Useless on BluePill, since it has no pin to control USB pullup it will always autoconnect
	usbd_TX_ready = true;
}

int32_t usbd_impl_send(uint8_t* buffer, uint_fast16_t size)
{
	int32_t b = usbd_ep_write(&hUSB, usbd_alt_interface? USBD_EP_ISO_IN_ADDR : USBD_EP_INT_IN_ADDR, buffer, size);
	usbd_TX_ready = false;
	if (b != size)
	{
		if (b < 0) DEBUG_CHARR('/', 'T', 'X', '0' + usbd_alt_interface, 'E', 'R', 'R')
		else DEBUG_CHARR('/', 'T', 'X', '0' + usbd_alt_interface, 'E', '0' + b/10, '0' + b%10);
	}
	return b;
}

void USB_LP_CAN1_RX0_IRQHandler(void) { usbd_poll(&hUSB); }
