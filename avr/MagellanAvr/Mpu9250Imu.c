/*
 * Mpu9250Imu.c
 *
 * Created: 5/25/2019 4:39:51 PM
 *  Author: Mitchell
 */ 

 #include "Mpu9250Imu.h"

 void init_imu()
 {
	// Configure output for SPI
	DDRB |= (1 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
	PORTB &= ~( (1 << 3 ) | (1 << 2) | (1 << 1) | (1 << 0) );
	
	// SPI config (1 MHz)
	SPCR |= (1 << SPE) | (1 << MSTR) | (1 << SPR0);
	SPCR &= ~(1 << SPIE);
	SPSR &= ~(1 << SPI2X);

	// CS output => L0
	DDRL |= (1 << 0);
	PORTL |= (1 << 0);
 }

 size_t read_and_append_imu_reading(char* buffer, size_t remainingBytes)
 {
	PORTL &= ~(1 << 0);

	// For now, try reading the WHO_AM_I register.
	SPDR = WHO_AM_I_MPU9250 | READ_FLAG;
	while(!(SPSR & (1 << SPIF))) 
	{
	};

	SPDR = 0x00;
	while(!(SPSR & (1 << SPIF)))
	{
	};

	unsigned char whoami = SPDR;

	PORTL |= (1 << 0);

	sprintf(buffer, "IMU: 0x%x|", whoami);

	return 10;
 }