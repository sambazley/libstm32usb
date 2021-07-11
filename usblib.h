/* Copyright (C) 2021 Sam Bazley
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software. If you use this software
 *    in a product, an acknowledgment in the product documentation would be
 *    appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */

#ifndef USBLIB_H
#define USBLIB_H

#include <stdint.h>

struct usb_setup_packet {
	volatile uint8_t bmRequestType;
	volatile uint8_t bRequest;
	volatile uint16_t wValue;
	volatile uint16_t wIndex;
	volatile uint16_t wLength;
} __attribute__((packed));

struct usb_endpoint {
	const uint16_t rx_size, tx_size;
	const uint16_t type;
	const enum {DIR_BIDIR, DIR_IN, DIR_OUT} dir;
};

struct usb_interface {
	const uint16_t index;
	uint16_t alternate;
	void (*on_ctrl)
		(struct usb_interface *, volatile struct usb_setup_packet *);
};

struct usb_descriptor {
	const uint16_t wValue;
	const uint16_t wIndex;
	const uint8_t *addr;
	const uint8_t length;
};

struct usb_configuration {
	struct usb_endpoint *endpoints;
	struct usb_interface *interfaces;
	struct usb_descriptor *descriptors;

	uint8_t endpoint_count;
	uint8_t interface_count;
	uint8_t descriptor_count;

	void (*on_correct_transfer)(uint8_t ep);
	void (*log_str)(const char *str);
	void (*log_int)(uint32_t n);
};

#define USB_EP(x) (&USB->EP0R + x * 2)

#define PMA_TX 0
#define PMA_RX 1

uint16_t *usb_pma_addr(uint8_t ep, uint8_t isrx);
void usb_pma_memcpy(uint16_t *dest, const uint16_t *src, uint16_t n);

uint8_t usb_get_selected_config();
uint8_t usb_has_received_request();

uint16_t usb_ep_get_rx_count(int ep);

void usb_ep_set_tx_status(uint8_t ep, uint16_t status);
void usb_ep_set_rx_status(uint8_t ep, uint16_t status);

void usb_ack(uint8_t ep);
void usb_init(struct usb_configuration *conf);

#endif /* USBLIB_H */
