/*
  FirmwareUpdate.h - Firmware Updater for WiFi101 / WINC1500.
  Copyright (c) 2015 Arduino LLC.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <WiFi101.h>
#include <spi_flash/include/spi_flash.h>

// #include <asf.h>
#include "common/include/nm_common.h"
#include "bus_wrapper/include/nm_bus_wrapper.h"
#include "driver/source/nmbus.h"
#include "driver/include/m2m_wifi.h"
#include "usart_stream.h"
#include <string.h>

#ifndef MIN
#define MIN(x, y) (x > y) ? y : x
#endif

enum cmd_err_code {
	CMD_ERR_NO_ERROR = 0,
	CMD_ERR_INTERNAL_ERROR,
	CMD_ERR_INVALID_FRAME,
};

struct uart_cmd_hdr {
	uint32_t cmd;
	uint32_t addr;
	uint32_t val;
};

static void buffer_to_cmd_hdr(uint8_t *buffer, struct uart_cmd_hdr *cmd_hdr)
{
	union {
		uint32_t i;
		char c[4];
	}
	bint = {0x01020304};

	if (bint.c[0] == 1) {
		/* Big endian. */
		cmd_hdr->cmd = ((uint32)buffer[0] << 24) | ((uint32)buffer[1] << 16) | ((uint32)buffer[2] << 8) | ((uint32)buffer[3] << 0);
		cmd_hdr->addr = ((uint32)buffer[4] << 24) | ((uint32)buffer[5] << 16) | ((uint32)buffer[6] << 8) | ((uint32)buffer[7] << 0);
		cmd_hdr->val = ((uint32)buffer[8] << 24) | ((uint32)buffer[9] << 16) | ((uint32)buffer[10] << 8) | ((uint32)buffer[11] << 0);
	} else {
		/* Little endian. */
		memcpy(cmd_hdr, buffer, sizeof(struct uart_cmd_hdr));
	}
}

static uint8_t checksum_check(uint8_t *buffer, uint8_t size)
{
	uint8 checksum = 0;
	int i;

	for (i = 0; i < size; i++) {
		checksum ^= buffer[i];
	}

	return checksum;
}

static int usart_sync_cmd_handler(uint8_t *buffer, uint8_t size)
{
	usart_stream_move(1);
	usart_stream_write(0x5B);

	return CMD_ERR_NO_ERROR;
}

static int usart_read_reg_with_ret_handler(uint8_t *buffer, uint8_t size)
{
	struct uart_cmd_hdr cmd_hdr;
	uint32_t val;

	if (checksum_check(buffer + 1, sizeof(struct uart_cmd_hdr)) != 0) {
		usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
		usart_stream_write(0x5A);
		return CMD_ERR_INVALID_FRAME;
	}

	buffer_to_cmd_hdr(buffer + 1, &cmd_hdr);

	usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
	usart_stream_write(0xAC);

	/* Translate it to SPI Read register command. */
	val = nm_read_reg(cmd_hdr.addr);
	usart_stream_write((uint8_t)(val >> 24));
	usart_stream_write((uint8_t)(val >> 16));
	usart_stream_write((uint8_t)(val >> 8));
	usart_stream_write((uint8_t)(val >> 0));

	return CMD_ERR_NO_ERROR;
}

static int usart_write_reg_handler(uint8_t *buffer, uint8_t size)
{
	struct uart_cmd_hdr cmd_hdr;

	if (checksum_check(buffer + 1, sizeof(struct uart_cmd_hdr)) != 0) {
		usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
		usart_stream_write(0x5A);
		return CMD_ERR_INVALID_FRAME;
	}

	buffer_to_cmd_hdr(buffer + 1, &cmd_hdr);
	nm_write_reg(cmd_hdr.addr, cmd_hdr.val);

	usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
	usart_stream_write(0xAC);

	return CMD_ERR_NO_ERROR;
}

static int usart_read_block_handler(uint8_t *buffer, uint8_t size)
{
	struct uart_cmd_hdr cmd_hdr;
	uint8_t reg_buffer[USART_BUFFER_MAX];
	uint32_t payload_length;

	if (checksum_check(buffer + 1, sizeof(struct uart_cmd_hdr)) != 0) {
		usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
		usart_stream_write(0x5A);
		return CMD_ERR_INVALID_FRAME;
	}

	buffer_to_cmd_hdr(buffer + 1, &cmd_hdr);
	usart_stream_write(0xAC);

	payload_length = (cmd_hdr.cmd >> 16) & 0xFFFF;

	nm_read_block(cmd_hdr.addr, reg_buffer, payload_length);

	usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
	usart_stream_write_buffer(reg_buffer, payload_length);

	return CMD_ERR_NO_ERROR;
}

static int usart_write_block_handler(uint8_t *buffer, uint8_t size)
{
	struct uart_cmd_hdr cmd_hdr;
	uint16_t payload_length;
	uint32_t read_size;
	uint8 *read_buffer;

	if (checksum_check(buffer + 1, sizeof(struct uart_cmd_hdr)) != 0) {
		usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
		usart_stream_write(0x5A);
		return CMD_ERR_INVALID_FRAME;
	}

	buffer_to_cmd_hdr(buffer + 1, &cmd_hdr);
	payload_length = (cmd_hdr.cmd >> 16) & 0xFFFF;

	usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
	usart_stream_write(0xAC);

	while (1) {
		if (usart_stream_read(&read_buffer, &read_size)) {
			continue;
		}

		if (read_size < payload_length) {
			continue;
		}

		break;
	}

	nm_write_block(cmd_hdr.addr, read_buffer, payload_length);
	usart_stream_move(payload_length);
	usart_stream_write(0xAC);

	return CMD_ERR_NO_ERROR;
}

static int usart_reconfigure_handler(uint8_t *buffer, uint8_t size)
{
	struct uart_cmd_hdr cmd_hdr;

	if (checksum_check(buffer + 1, sizeof(struct uart_cmd_hdr)) != 0) {
		usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
		usart_stream_write(0x5A);
		return CMD_ERR_INVALID_FRAME;
	}

	buffer_to_cmd_hdr(buffer + 1, &cmd_hdr);
	usart_stream_move(sizeof(struct uart_cmd_hdr) + 1);
	usart_stream_write(0xAC);

	configure_usart(cmd_hdr.val);

	return CMD_ERR_NO_ERROR;
}

struct usart_frame {
	uint8_t header[4];
	uint32_t header_size;
	uint32_t min_size;
	int (*handler)(uint8_t *buffer, uint8_t size);
};

struct usart_frame usart_handler[] = {
	{{0x12, 0}, 1, 1, usart_sync_cmd_handler}, /* nm_uart_sync_cmd */
	{{0xa5, 0x00, 0}, 2, sizeof(struct uart_cmd_hdr) + 1, usart_read_reg_with_ret_handler}, /* nm_uart_read_reg_with_ret */
	{{0xa5, 0x01, 0}, 2, sizeof(struct uart_cmd_hdr) + 1, usart_write_reg_handler}, /* nm_uart_write_reg */
	{{0xa5, 0x02, 0}, 2, sizeof(struct uart_cmd_hdr) + 1, usart_read_block_handler}, /* nm_uart_read_block */
	{{0xa5, 0x03, 0}, 2, sizeof(struct uart_cmd_hdr) + 1, usart_write_block_handler}, /* nm_uart_write_block */
	{{0xa5, 0x04, 0}, 2, 2, NULL}, /* Reset */
	{{0xa5, 0x05, 0}, 2, sizeof(struct uart_cmd_hdr) + 1, usart_reconfigure_handler}, /* nm_uart_reconfigure */
	{{0xa5, 0x0A, 0}, 2, 2, NULL}, /* Read SPI read GPIO */
};

#define HANDLER_SIZE sizeof(usart_handler) / sizeof(struct usart_frame)

/**
 * \brief Parse incoming frame and find the handler to process the request.
 */
static void usart_frame_parse(uint8_t *buffer, uint8_t size)
{
	if (size == 0) {
		return;
	}

	for (uint32_t i = 0; i < HANDLER_SIZE; i++) {
		if (size >= usart_handler[i].min_size &&
				!memcmp(usart_handler[i].header, buffer, usart_handler[i].header_size)) {
			if (usart_handler[i].handler) {
				if ((usart_handler[i].handler(buffer, size)) == 0) {
					return;
				} else {
					break;
				}
			}
		}
	}

	if (buffer[0] == 0xFF) {
		usart_stream_reset();
	} else if (buffer[0] != 0x12 && buffer[0] != 0xA5) {
		/* Undefined message, send error. */
		usart_stream_write(0xEA);
		usart_stream_reset();
	}
}

/**
 * \brief Process input UART command and forward to SPI.
 */
static sint8 enter_wifi_firmware_download(void)
{
	sint8 ret;
	uint8_t *usart_data;
	uint32_t usart_size;

	ret = m2m_wifi_download_mode();
	if (ret != M2M_SUCCESS) {
		puts("Failed to put the WiFi Chip in download mode!\n");
		return M2M_ERR_INIT;
	}

	usart_stream_reset();

	/* Process UART input command and forward to SPI. */
	while (1) {
		if (usart_stream_read(&usart_data, &usart_size) != 0) {
			continue;
		}

		usart_frame_parse(usart_data, usart_size);
	}
	return ret;
}

static uint8_t usart_buffer[USART_BUFFER_MAX];
static uint32_t usart_recv_size = 0;

void configure_usart(uint32_t baudrate)
{
	// usart_serial_options_t uart_serial_options = {
	// 	.baudrate = baudrate,
	// 	.paritytype = CONF_UART_PARITY,
	// 	.charlength = US_MR_CHRL_8_BIT,
	// 	.stopbits = 0,
	// };

	// /* Configure the UART console. */
	// usart_serial_init((usart_if)CONF_UART, &uart_serial_options);
    Serial.begin(baudrate);
    Serial.flush();

	usart_stream_reset();
}

void usart_stream_reset(void)
{
// #ifdef SAMG55
// 	usart_reset_rx((Usart *)CONF_UART);
// 	usart_enable_rx((Usart *)CONF_UART);
// #else
// 	uart_reset((Uart *)CONF_UART);
// 	uart_enable((Uart *)CONF_UART);
// #endif
	usart_recv_size = 0;
}

void usart_stream_write(uint8_t data)
{
    Serial.write(data);
// 	usart_serial_putchar((usart_if)CONF_UART, data);
// #ifdef SAMG55
// 	while (!usart_is_tx_empty((Usart *)CONF_UART)) {
// #else
// 	while (!uart_is_tx_buf_empty((Uart *)CONF_UART)) {
// #endif
// 	}
}

void usart_stream_write_buffer(uint8_t *data, uint32_t size)
{
    Serial.write(data, size);

	// for (uint32_t i = 0; i < size; i++) {
	// 	usart_serial_putchar((usart_if)CONF_UART, data[i]);
	// }
}

int usart_stream_read(uint8_t **data, uint32_t *size)
{
// #ifdef SAMG55
// 	uint32_t val;

// 	while (usart_recv_size < sizeof(usart_buffer) && usart_read((Usart *)CONF_UART, &val) == 0) {
// #else
	uint8_t val;
    val = Serial.read();

	while (usart_recv_size < sizeof(usart_buffer) && (val != -1)) {
// #endif
		usart_buffer[usart_recv_size++] = val;
        val = Serial.read();
	}

	*data = usart_buffer;
	*size = usart_recv_size;

	return 0;
}

void usart_stream_move(uint32_t offset)
{
	usart_recv_size -= offset;
	if (usart_recv_size > 0) {
		memmove(usart_buffer, usart_buffer + offset, usart_recv_size);
	} else {
		usart_recv_size = 0;
	}
}


typedef struct __attribute__((__packed__)) {
    uint8_t command;
    uint32_t address;
    uint32_t arg1;
    uint16_t payloadLength;

    // payloadLenght bytes of data follows...
} UartPacket;

static const int MAX_PAYLOAD_SIZE = 1024 * 8;

#define CMD_READ_FLASH 0x01
#define CMD_WRITE_FLASH 0x02
#define CMD_ERASE_FLASH 0x03
#define CMD_MAX_PAYLOAD_SIZE 0x50
#define CMD_HELLO 0x99

void setup() {
    WiFi.setPins(8, 7, 4, 2);

    Serial.begin(115200);

    nm_bsp_init();
    if (m2m_wifi_download_mode() != M2M_SUCCESS) {
        Serial.println(F("Failed to put the WiFi module in download mode"));
        while (true)
            ;
    }
}

void receivePacket(UartPacket *pkt, uint8_t *payload) {
    // Read command
    uint8_t *p = reinterpret_cast<uint8_t *>(pkt);
    uint16_t l = sizeof(UartPacket);
    while (l > 0) {
        int c = Serial.read();
        if (c == -1)
            continue;
        *p++ = c;
        l--;
    }

    // Convert parameters from network byte order to cpu byte order
    pkt->address = fromNetwork32(pkt->address);
    pkt->arg1 = fromNetwork32(pkt->arg1);
    pkt->payloadLength = fromNetwork16(pkt->payloadLength);

    // Read payload
    l = pkt->payloadLength;
    while (l > 0) {
        int c = Serial.read();
        if (c == -1)
            continue;
        *payload++ = c;
        l--;
    }
}

// Allocated statically so the compiler can tell us
// about the amount of used RAM
static UartPacket pkt;
static uint8_t payload[MAX_PAYLOAD_SIZE];

void loop() {
    receivePacket(&pkt, payload);

    if (pkt.command == CMD_HELLO) {
        if (pkt.address == 0x11223344 && pkt.arg1 == 0x55667788)
            Serial.print("v10000");
    }

    if (pkt.command == CMD_MAX_PAYLOAD_SIZE) {
        uint16_t res = toNetwork16(MAX_PAYLOAD_SIZE);
        Serial.write(reinterpret_cast<uint8_t *>(&res), sizeof(res));
    }

    if (pkt.command == CMD_READ_FLASH) {
        uint32_t address = pkt.address;
        uint32_t len = pkt.arg1;
        if (spi_flash_read(payload, address, len) != M2M_SUCCESS) {
            Serial.println("ER");
        } else {
            Serial.write(payload, len);
            Serial.print("OK");
        }
    }

    if (pkt.command == CMD_WRITE_FLASH) {
        uint32_t address = pkt.address;
        uint32_t len = pkt.payloadLength;
        if (spi_flash_write(payload, address, len) != M2M_SUCCESS) {
            Serial.print("ER");
        } else {
            Serial.print("OK");
        }
    }

    if (pkt.command == CMD_ERASE_FLASH) {
        uint32_t address = pkt.address;
        uint32_t len = pkt.arg1;
        if (spi_flash_erase(address, len) != M2M_SUCCESS) {
            Serial.print("ER");
        } else {
            Serial.print("OK");
        }
    }
}
