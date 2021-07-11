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

#include "usblib.h"
#include <string.h>

#if defined STM32F0
#include <stm32f0xx.h>
#endif

#define FROM_PMA(x) (x + USB_PMAADDR)
#define TO_PMA(x) (x - USB_PMAADDR)

struct buf_desc_entry {
	volatile uint16_t addr;
	volatile uint16_t count;
} __attribute__((packed));

struct buf_desc {
	struct buf_desc_entry tx, rx;
} __attribute__((packed));

#define btable ((volatile struct buf_desc *) USB_PMAADDR)

static struct usb_configuration *conf;

static void log_str(const char *str)
{
	if (conf->log_str) {
		conf->log_str(str);
	}
}

static void log_int(uint32_t n)
{
	if (conf->log_int) {
		conf->log_int(n);
	}
}

uint16_t *usb_pma_addr(uint8_t ep, uint8_t isrx)
{
	uint16_t addr = conf->endpoint_count * sizeof(struct buf_desc);

	for (int i = 0; i < ep; i++) {
		addr += conf->endpoints[i].tx_size;
		addr += conf->endpoints[i].rx_size;
	}

	if (isrx) {
		addr += conf->endpoints[ep].tx_size;
	}

	return (uint16_t *) FROM_PMA(addr);
}

static void usb_set_irq()
{
	USB->ISTR = 0;
	USB->CNTR = USB_CNTR_CTRM | USB_CNTR_WKUPM | USB_CNTR_SUSPM |
		USB_CNTR_RESETM;
}

static void set_ep_rx_count(volatile struct buf_desc_entry *e, int count)
{
	if (count > 62) {
		e->count = 0x8000 | (((count >> 5) - 1) << 10);
	} else {
		e->count = (count >> 1) << 10;
	}
}

uint16_t usb_ep_get_rx_count(int ep)
{
	if (conf->endpoints[ep].type == USB_EP_ISOCHRONOUS &&
			!(*USB_EP(ep) & USB_EP_DTOG_RX)) {
		return btable[ep].tx.count & 0x3ff;
	}

	return btable[ep].rx.count & 0x3ff;
}

void usb_ep_set_tx_status(uint8_t ep, uint16_t status)
{
	*USB_EP(ep) = (*USB_EP(ep) & USB_EPTX_DTOGMASK) ^ status;
}

void usb_ep_set_rx_status(uint8_t ep, uint16_t status)
{
	*USB_EP(ep) = (*USB_EP(ep) & USB_EPRX_DTOGMASK) ^ status;
}

static volatile struct usb_transmit_data {
	uint16_t length, wLength;
	uint16_t sent;
	const uint8_t *data;
} usb_tx_data [8];

static void usb_continue_send_data(int ep)
{
	volatile struct usb_transmit_data *tx = &usb_tx_data[ep];
	uint16_t *dest = usb_pma_addr(ep, PMA_TX);
	uint8_t tx_length = tx->length - tx->sent;

	if (tx->wLength != 0 && tx_length > tx->wLength) {
		tx_length = tx->wLength;
	} else if (tx_length > conf->endpoints[ep].tx_size) {
		tx_length = conf->endpoints[ep].tx_size;
	}

	for (int i = 0; i < tx_length; i += 2) {
		*dest++ = tx->data[i] | ((uint16_t) tx->data[i + 1] << 8);
	}

	tx->data += tx_length;
	tx->sent += tx_length;

	btable[ep].tx.count = tx_length;

	usb_ep_set_tx_status(ep, USB_EP_TX_VALID);
}

static void usb_send_data(uint8_t endpoint,
		const uint8_t *data,
		uint16_t length,
		uint16_t wLength)
{
	usb_tx_data[endpoint].data = data;
	usb_tx_data[endpoint].length = length;
	usb_tx_data[endpoint].wLength = wLength;
	usb_tx_data[endpoint].sent = 0;

	usb_continue_send_data(endpoint);
}

static void find_descriptor(const uint16_t wValue,
		const uint16_t wIndex,
		const uint8_t **addr,
		uint8_t *length)
{
	for (int i = 0; i < conf->descriptor_count; i++) {
		const struct usb_descriptor *entry = conf->descriptors + i;
		if (entry->wValue == wValue && entry->wIndex == wIndex) {
			*addr = entry->addr;
			*length = entry->length;
			return;
		}
	}

	log_str("No descriptor found, wValue: ");
	log_int(wValue);
	log_str(", wIndex: ");
	log_int(wIndex);
	log_str("\n");

	*addr = 0;
	*length = 0;
}

enum {
	REQ_CLEAR_FEATURE = 1,
	REQ_SET_ADDRESS = 5,
	REQ_GET_DESCRIPTOR = 6,
	REQ_SET_CONFIGURATION = 9,
	REQ_SET_INTERFACE = 11,
};

enum {
	RECIP_DEVICE,
	RECIP_INTERFACE,
	RECIP_ENDPOINT,
	RECIP_OTHER
};

static volatile uint8_t usb_selected_config;
static volatile uint8_t usb_received_request;
static volatile int address = 0;

uint8_t usb_get_selected_config()
{
	return usb_selected_config;
}

uint8_t usb_has_received_request()
{
	return usb_received_request;
}

void usb_ack(uint8_t ep)
{
	usb_send_data(ep, 0, 0, 0);
}

static void on_control_out_interface(volatile struct usb_setup_packet *sp)
{
	for (int i = 0; i < conf->interface_count; i++) {
		struct usb_interface *iface = conf->interfaces + i;

		if (sp->bRequest == REQ_SET_INTERFACE) {
			iface->alternate = sp->wValue;
		}

		if ((sp->wIndex & 0xFF) == iface->index) {
			iface->on_ctrl(iface, sp);
			return;
		}
	}

	log_str("Interface ");
	log_int(sp->wIndex);
	log_str(" not found\n");
}

static void on_control_out_device(volatile struct usb_setup_packet *sp)
{
	switch (sp->bRequest) {
		case REQ_GET_DESCRIPTOR:
		{
			const uint8_t *addr;
			uint8_t length;

			find_descriptor(sp->wValue, sp->wIndex, &addr, &length);

			if (!addr) {
				log_str("Failed to find descriptor\n");
				usb_ep_set_tx_status(0, USB_EP_TX_STALL);
				break;
			}

			usb_send_data(0, addr, length, sp->wLength);
			break;
		}
		case REQ_SET_ADDRESS:
			address = sp->wValue & 0x7F;
			usb_ack(0);
			break;
		case REQ_SET_CONFIGURATION:
			usb_selected_config = sp->wValue;
			usb_ack(0);
			break;
		default:
			log_str("== UNHANDLED REQUEST ");
			log_int(sp->bRequest);
			log_str(" ==\n");
			usb_ack(0);
	}
}

static void on_control_out()
{
	if (USB->EP0R & USB_EP_SETUP) {
		volatile struct usb_setup_packet *sp =
			(volatile struct usb_setup_packet *) usb_pma_addr(0, PMA_RX);

		uint8_t recipient = sp->bmRequestType & 0x1f;

		switch (recipient) {
		case RECIP_DEVICE:
			on_control_out_device(sp);
			break;
		case RECIP_INTERFACE:
			on_control_out_interface(sp);
			return;
		default:
			log_str("Recipient ");
			log_int(recipient);
			log_str(" not implemented for control out\n");
			usb_ack(0);
			return;
		}
	}
}

static void on_control_in()
{
	usb_ep_set_rx_status(0, USB_EP_RX_VALID);

	if (address) {
		USB->DADDR = address | USB_DADDR_EF;
		address = 0;
	}
}

static void on_correct_transfer()
{
	uint8_t ep = USB->ISTR & USB_ISTR_EP_ID;

	usb_received_request = 1;

	if (USB->ISTR & USB_ISTR_DIR) {
		if (ep == 0) {
			on_control_out();
		} else {
			conf->on_correct_transfer(0x80 | ep);
		}

		*USB_EP(ep) = *USB_EP(ep) & ~USB_EP_CTR_RX & USB_EPREG_MASK;
	} else {
		uint16_t remaining = usb_tx_data[ep].length - usb_tx_data[ep].sent;

		if (remaining) {
			usb_continue_send_data(ep);
		} else if ((usb_tx_data[ep].length % usb_tx_data[ep].wLength) == 0) {
			usb_ack(ep);
		}

		if (ep == 0) {
			on_control_in();
		} else {
			conf->on_correct_transfer(ep);
		}

		*USB_EP(ep) = *USB_EP(ep) & ~USB_EP_CTR_TX & USB_EPREG_MASK;
	}
}

static void open_endpoints()
{
	for (unsigned int i = 0; i < conf->endpoint_count; i++) {
		volatile uint16_t *epr = USB_EP(i);
		const struct usb_endpoint *ep = &conf->endpoints[i];

		usb_ep_set_rx_status(i, USB_EP_RX_DIS);
		usb_ep_set_tx_status(i, USB_EP_TX_DIS);

		*epr = (*epr & USB_EPREG_MASK) | i;
		*epr = (*epr & USB_EP_T_MASK) | ep->type;

		if (*epr & USB_EP_DTOG_TX) {
			*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_TX;
		}

		if (*epr & USB_EP_DTOG_RX) {
			*epr = (*epr & USB_EPREG_MASK) | USB_EP_DTOG_RX;
		}

		switch (ep->type) {
		case USB_EP_CONTROL:
			set_ep_rx_count(&btable[i].rx, ep->rx_size);
			usb_ep_set_rx_status(i, USB_EP_RX_VALID);
			usb_ep_set_tx_status(i, USB_EP_TX_NAK);
			break;
		case USB_EP_ISOCHRONOUS:
			if (ep->dir == DIR_OUT) {
				set_ep_rx_count(&btable[i].rx, ep->rx_size);
				set_ep_rx_count(&btable[i].tx, ep->tx_size);
				usb_ep_set_rx_status(i, USB_EP_RX_VALID);
			}
			break;
		case USB_EP_INTERRUPT:
		case USB_EP_BULK:
			if (ep->dir == DIR_OUT) {
				set_ep_rx_count(&btable[i].rx, ep->rx_size);
				usb_ep_set_rx_status(i, USB_EP_RX_VALID);
			} else {
				usb_ep_set_tx_status(i,USB_EP_TX_NAK);
			}
			break;
		default:
			log_str("Endpoint type ");
			log_int(ep->type);
			log_str(" not implemented\n");
		}
	}
}

void usb_irq()
{
	while (USB->ISTR & USB_ISTR_CTR) {
		on_correct_transfer();
	}

	if (USB->ISTR & USB_ISTR_WKUP) {
		USB->CNTR &= ~USB_CNTR_FSUSP;
		USB->CNTR &= ~USB_CNTR_LPMODE;
		USB->ISTR &= ~USB_ISTR_WKUP;
	}

	if (USB->ISTR & USB_ISTR_SUSP) {
		USB->CNTR |= USB_CNTR_FSUSP;
		USB->CNTR |= USB_CNTR_LPMODE;
		USB->ISTR &= ~USB_ISTR_SUSP;
	}

	if (USB->ISTR & USB_ISTR_RESET) {
		usb_selected_config = 0;
		USB->DADDR = USB_DADDR_EF;
		open_endpoints();
		USB->ISTR &= ~USB_ISTR_RESET;
	}
}

void usb_init(struct usb_configuration *_conf)
{
	conf = _conf;

	RCC->APB1ENR |= RCC_APB1ENR_CRSEN;
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	CRS->CR |= CRS_CR_AUTOTRIMEN | CRS_CR_CEN;

#if defined STM32F0_PA11_PA12_RMP
	SYSCFG->CFGR1 |= SYSCFG_CFGR1_PA11_PA12_RMP;
#endif

	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USBEN;

	USB->BTABLE = 0;

	USB->BCDR |= USB_BCDR_DPPU;

	for (unsigned int i = 0; i < conf->endpoint_count; i++) {
		uint32_t rx = (uint32_t) usb_pma_addr(i, PMA_RX);
		uint32_t tx = (uint32_t) usb_pma_addr(i, PMA_TX);
		btable[i].rx.addr = TO_PMA(rx);
		btable[i].tx.addr = TO_PMA(tx);
	}

	usb_selected_config = 0;

	NVIC_EnableIRQ(USB_IRQn);
	usb_set_irq();
}
