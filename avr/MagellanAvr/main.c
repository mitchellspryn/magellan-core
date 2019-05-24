/*
 * MagellanAvr.c
 *
 * Created: 5/19/2019 5:02:28 PM
 * Author : Mitchell
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "Common.h"
#include "AdafruitUltimateGps.h"

#define CLIENT_TX_BUF_LEN 1022

typedef struct ClientTxBuf_t
{
	char Buffer[CLIENT_TX_BUF_LEN + 2]; // Leave space for \n
	size_t Index;
} ClientTxBuf_t;

static void init_client_serial_port();
static void send_client_message(const char* message);

static ClientTxBuf_t ClientTxBuf;

int main(void)
{
    init_client_serial_port();
	send_client_message("Starting...\r\n");

	init_gps();
	send_client_message("Initialized GPS.\r\n");

	sei();
	for (;;)
	{
		ClientTxBuf.Index = 0;
		ClientTxBuf.Index += append_gps_reading(ClientTxBuf.Buffer, CLIENT_TX_BUF_LEN - ClientTxBuf.Index);
		
		if (ClientTxBuf.Index > 0)
		{
			ClientTxBuf.Buffer[ClientTxBuf.Index++] = '\n';
			ClientTxBuf.Buffer[ClientTxBuf.Index] = '\0';
			send_client_message(ClientTxBuf.Buffer);
		}
	}
}

void init_client_serial_port()
{
	uint16_t ubrrh;
	uint16_t ubrrl;
	unsigned char use2x;

	get_38400_baud_settings(&ubrrh, &ubrrl, &use2x);

	DDRE |= (1 << 1) | (1 << 0);
	UBRR0H = ubrrh;
	UBRR0L = ubrrl;

	if (use2x)
	{
		UCSR0A |= (1 << U2X0);
	}
	else
	{
		UCSR0A &= ~(1 << U2X0);
	}

	UCSR0B |= (1 << RXEN0) | (1 << TXEN0);
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}

void send_client_message(const char* message)
{
	for (; *message; message++)
	{
		UDR0 = *message;
		while(!(UCSR0A & (1 << UDRE0))) {};
	}
}
