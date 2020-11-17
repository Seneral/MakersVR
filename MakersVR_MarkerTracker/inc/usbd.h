/**
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef __USBD_H
#define __USBD_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>

#include "main.h"

#include "usb_std.h"
#include "usbd_core.h"
#include "usb.h"

typedef struct
{
	usbd_respond (*usbd_ctrl_rsp)(usbd_device *usbd, usbd_ctlreq *req);
	usbd_respond (*usbd_ctrl_rcv)(usbd_device *usbd, usbd_ctlreq *req);
} usbd_callbacks;

struct USBPortState {
	TimePoint lastFrame;
	uint_fast16_t framePos;
	uint_fast16_t bufferPos;
	uint_fast16_t bufferSz;
};

extern int usbd_alt_interface;
extern TimePoint lastSOF;
extern struct USBPortState isoPort;
extern struct USBPortState intPort[1];

void usbd_impl_init(usbd_callbacks impl_callbacks);

int32_t usbd_impl_send(uint8_t *buffer, uint_fast16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_H */
