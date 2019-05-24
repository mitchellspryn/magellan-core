/*
 * AdafruitUltimateGps.c
 *
 * Created: 5/20/2019 9:08:46 PM
 *  Author: Mitchell
 */ 

#include "AdafruitUltimateGps.h"

#define GPS_RX_BUF_LEN 127

typedef struct GpsRxBuf_t
{
	char Buffer[GPS_RX_BUF_LEN + 1];
	size_t Index;
	int Ready;
} GpsRxBuf_t;

static volatile GpsRxBuf_t GpsRxBuf;


void init_gps()
{
	// NOTE: This assumes that the GPS has been pre-configured to the following settings:
	//	BAUD: 38400
	//  Only sending GPRMC packets
	//  Fix rate: 5 hz
	//
	// If the GPS has a battery, it will remember these settings between boots.
	// If the battery dies, replace it and send the following commands to the device:
	// 
	// "$PMTK251,38400*27\r\n"                                    -- Set baud rate to 38400
	// "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"    -- Enable only GPRMC output
	// "$PMTK300,200,0,0,0,0*2F\r\n"                              -- Set GPS fix to 5 hz
	// "$PMTK220,200*2C\r\n"                                      -- Set NMEA sentence output to 5 hz
	//
	// PMTK commands here: https://cdn-shop.adafruit.com/datasheets/PMTK_A11.pdf

	// Although the docs say sentences can be output at 10hz, the fix can only be obtained at 5 hz.
	// Sentences can be output at 10hz, which contains additional data
	//		(which is uninteresting for our purposes)
	// Reference this thread: https://forums.adafruit.com/viewtopic.php?f=22&p=303966

	uint16_t ubrrh;
	uint16_t ubrrl;
	unsigned char use2x;
	
	get_38400_baud_settings(&ubrrh, &ubrrl, &use2x);
	UBRR1H = ubrrh;
	UBRR1L = ubrrl;
	if (use2x)
	{
		UCSR1A |= (1 << U2X1);
	}
	else
	{
		UCSR1A &= ~(1 << U2X1);
	}

	DDRD |= (1 << 2);
	UCSR1B |= (1 << RXEN1);
	UCSR1C |= (1 << UCSZ11) | (1 < UCSZ10);

	// Yes, these should be statically initialized to zero by the compiler. 
	// But it doesn't hurt to be sure.
	GpsRxBuf.Ready = 0x00;
	GpsRxBuf.Index = 0;

	UCSR1B |= (1 << RXCIE1);
}

// Format: "GPS:<buf_data>|"
size_t append_gps_reading(char* buffer, size_t remainingBytes)
{
	if (!GpsRxBuf.Ready || remainingBytes <= 6 || GpsRxBuf.Index == 0)
	{
		return 0;
	}

	char* originalBuffer = buffer;

	size_t numBytesToCopy = GpsRxBuf.Index + 6;
	if (remainingBytes < numBytesToCopy)
	{
		numBytesToCopy = remainingBytes;
	}

	memcpy(buffer, "GPS:", 4);
	buffer += 4;
	
	// GpsRxBuf has to be volatile, so memcpy() does not work.
	// It's also not a c string, so strcpy() doesn't work either.
	for (int i = 0; i < numBytesToCopy - 6; i++)
	{
		*buffer++ = GpsRxBuf.Buffer[i];
	}
	
	memcpy(buffer, "|\0", 2);
	buffer++;

	GpsRxBuf.Ready = 0x00;
	GpsRxBuf.Index = 0;

	return buffer - originalBuffer;
}

ISR(USART1_RX_vect)
{
	char c = UDR1;

	if (GpsRxBuf.Index < GPS_RX_BUF_LEN)
	{
		// There is the possibility for a chopped message.
		// However, given the frequency of the GPS updates (5 Hz)
		//    and the speed of mega (16 MHz),
		//    this situation indicates bigger problems to solve.
		if (!GpsRxBuf.Ready && c != '\r' && c != '\n')
		{
			GpsRxBuf.Buffer[GpsRxBuf.Index] = c;
			GpsRxBuf.Index++;
		}	
	}

	if (c == '\n')
	{
		GpsRxBuf.Ready = 0x01;
	}
}