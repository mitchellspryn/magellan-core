/*
 * Common.c
 *
 * Created: 5/20/2019 9:09:14 PM
 *  Author: Mitchell
 */ 

#include "Common.h"

void get_9600_baud_settings(uint16_t *ubrrh, uint16_t *ubrrl, unsigned char* use2x)
{
	// Believe it or not, this is actually the recommended way of setting the UART baud rate.
	// https://www.nongnu.org/avr-libc/user-manual/group__util__setbaud.html
	#undef BAUD
	#define BAUD 9600
	#include <util/setbaud.h>
	
	*ubrrh = UBRRH_VALUE;
	*ubrrl = UBRRL_VALUE;
	
	#if USE_2x
		*use2x = 0x01;
	#else
		*use2x = 0x00;
	#endif
}

void get_38400_baud_settings(uint16_t *ubrrh, uint16_t *ubrrl, unsigned char* use2x)
{
	#undef BAUD
	#define BAUD 38400
	
	#include <util/setbaud.h>
	
	*ubrrh = UBRRH_VALUE;
	*ubrrl = UBRRL_VALUE;
	
	#if USE_2x
		*use2x = 0x01;
	#else
		*use2x = 0x00;
	#endif
}

void get_76800_baud_settings(uint16_t *ubrrh, uint16_t *ubrrl, unsigned char* use2x)
{
	#undef BAUD
	#define BAUD 76800
	
	#include <util/setbaud.h>
	
	*ubrrh = UBRRH_VALUE;
	*ubrrl = UBRRL_VALUE;
	
	#if USE_2x
		*use2x = 0x01;
	#else
		*use2x = 0x00;
	#endif
}