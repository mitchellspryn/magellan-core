/*
 * Mpu9250Imu.c
 *
 * Created: 5/25/2019 4:39:51 PM
 *  Author: Mitchell
 */ 

#include "Mpu9250Imu.h"

static void write_spi_bytes(unsigned char startAddress, unsigned char* data, size_t num_bytes);
static void read_spi_bytes(unsigned char startAddress, unsigned char* data, size_t num_bytes);
static inline void write_spi_byte(unsigned char startAddress, unsigned char data);
static inline unsigned char read_spi_byte(unsigned char startAddress);
static inline unsigned char spi_char_transaction(unsigned char data);

int init_imu()
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

	// Read WHO_AM_I register. Ensure that it is 0x71
	unsigned char whoAmI = read_spi_byte(WHO_AM_I_MPU9250);
	
	// TODO: add proper error handling
	if (whoAmI != 0x71)
	{
		UDR0 = '!';
		while(!(UCSR0A & (1 << UDRE0))) {};
		return 0x00;
	}

	// Disable I2C mode
	// TODO: is this really necessary?
	write_spi_byte(USER_CTRL_MPU9250, 0x30);

	// Enable PLL if ready. Otherwise, use internal 20 MHz oscillator
	write_spi_byte(PWR_MGMT_1_MPU9250, 0x01);
	
	// Enable all sensors
	write_spi_byte(PWR_MGMT_2_MPU9250, 0x00);

	// Set the digital low pass filter for the accelerometer.
	// This code sets the bandwidth to 41 hz, yielding a 6ms delay.
	write_spi_byte(CONFIG_MPU9250, 0x03);

	// Set the sample rate to 200 hz
	write_spi_byte(SAMPLE_RATE_DIVIDER_MPU9250, 0x04);

	// Enable low pass filter on gyros by clearing FChoice_b bits.
	// Also set scale to 250 deg / sec. This will give us the most accuracy.
	char gyro_config = read_spi_byte(GYRO_CONFIG_MPU9250);
	gyro_config &= ~( (1 << 4) | (1 << 3) | (1 << 1) | (1 << 0) );
	write_spi_byte(GYRO_CONFIG_MPU9250, gyro_config);

	// Set the accelerometer to +- 16g mode
	char accel_config = read_spi_byte(ACCEL_CONFIG_1_MPU9250);
	accel_config |= ( (1 << 4) | (1 << 3) );
	write_spi_byte(ACCEL_CONFIG_1_MPU9250, accel_config);

	// Enable the low pass filter for the accelerometer
	// Set the bandwidth to 44.8 Hz. This introduces a delay of 5 ms
	write_spi_byte(ACCEL_CONFIG_2_MPU9250, 0x0B);

	// Check that magnetometer is online.
	write_spi_byte(I2C_MSTR_CTRL_MPU9250, 0x8D);
	write_spi_byte(INT_PIN_CFG_MPU9250, 0x22);
	write_spi_byte(I2C_SLV4_ADDR_MPU9250, ADDRESS_AK8963 | READ_FLAG);
	write_spi_byte(I2C_SLV4_DO_MPU9250, 0x00);
	write_spi_byte(I2C_SLV4_REG_MPU9250, 0x00);
	write_spi_byte(I2C_SLV4_CTRL_MPU9250, (1 << 7) | (1 << 0));

	whoAmI = 0x00;
	do 
	{
		whoAmI = read_spi_byte(I2C_MASTER_STATUS_MPU9250);
	} while (!(whoAmI & (1 << 6)));

	whoAmI = read_spi_byte(I2C_SLV4_DI_MPU9250);

	if (whoAmI != 0x48)
	{
		UDR0 = 'X';
		while(!(UCSR0A & (1 << UDRE0))) {};
		return 0x00;
	}

	// Configure the magnetometer
	write_spi_byte(I2C_SLV4_ADDR_MPU9250, ADDRESS_AK8963);
	write_spi_byte(I2C_SLV4_DO_MPU9250, 0x16);
	write_spi_byte(I2C_SLV4_REG_MPU9250, CTRL1_AK8963);
	write_spi_byte(I2C_SLV4_CTRL_MPU9250, (1 << 7) | (1 << 0));
	whoAmI = 0x00;
	do
	{
		whoAmI = read_spi_byte(I2C_MASTER_STATUS_MPU9250);
	} while (!(whoAmI & (1 << 6)));

	// For now, check that the config went through
	write_spi_byte(I2C_SLV4_ADDR_MPU9250, ADDRESS_AK8963 | READ_FLAG);
	write_spi_byte(I2C_SLV4_DO_MPU9250, 0x00);
	write_spi_byte(I2C_SLV4_REG_MPU9250, CTRL1_AK8963);
	write_spi_byte(I2C_SLV4_CTRL_MPU9250, (1 << 7) | (1 << 0));
	whoAmI = 0x00;
	do
	{
		whoAmI = read_spi_byte(I2C_MASTER_STATUS_MPU9250);
	} while (!(whoAmI & (1 << 6)));

	whoAmI = read_spi_byte(I2C_SLV4_DI_MPU9250);
	if (whoAmI != 0x16)
	{
		UDR0 = 'Y';
		while(!(UCSR0A & (1 << UDRE0))) {};
		UDR0 = 'Y';
		while(!(UCSR0A & (1 << UDRE0))) {};
		UDR0 = whoAmI;
		while(!(UCSR0A & (1 << UDRE0))) {};
		UDR0 = 'Y';
		while(!(UCSR0A & (1 << UDRE0))) {};
		UDR0 = 'Y';
		while(!(UCSR0A & (1 << UDRE0))) {};
		return 0x00;
	}

	// Read WHO_AM_I register. Ensure that it is 0x71
	whoAmI = read_spi_byte(WHO_AM_I_MPU9250);
	
	// TODO: add proper error handling
	if (whoAmI != 0x71)
	{
		UDR0 = '%';
		while(!(UCSR0A & (1 << UDRE0))) {};
		return 0x00;
	}

	return 0x01;
}

size_t read_and_append_imu_reading(char* buffer, size_t remainingBytes)
{
	if (remainingBytes <= MESSAGE_LENGTH)
	{
		return 0;
	}

	unsigned char data[14];

	read_spi_bytes(DATA_START_MPU9250, data, 14);

	int16_t accelX = (data[0] << 8) | data[1];
	int16_t accelY = (data[2] << 8) | data[3];
	int16_t accelZ = (data[4] << 8) | data[5];
	int16_t temp = (data[6] << 8) | data[7];
	int16_t gyroX = (data[8] << 8) | data[9];
	int16_t gyroY = (data[10] << 8) | data[11];
	int16_t gyroZ = (data[12] << 8) | data[13];

	// TODO: This is hacky. 
	// There should be a way to read this into EXT_SENS_DATA via I2C_SLV0 registers
	// However, the data doesn't seem to show up.
	char c;
	for (int i = 0; i < 7; i++)
	{
		write_spi_byte(I2C_SLV4_ADDR_MPU9250, ADDRESS_AK8963 | READ_FLAG);
		write_spi_byte(I2C_SLV4_DO_MPU9250, 0x00);
		write_spi_byte(I2C_SLV4_REG_MPU9250, DATA_START_AK8963 + i);
		write_spi_byte(I2C_SLV4_CTRL_MPU9250, (1 << 7) | (1 << 0));

		do 
		{
			c = read_spi_byte(I2C_MASTER_STATUS_MPU9250);
		} while (!(c & (1 << 6)));

		data[i] = read_spi_byte(I2C_SLV4_DI_MPU9250);
	}

	// Magnetometer is opposite endian as the accelerometer.
	int16_t magX = 0xFFFF;
	int16_t magY = 0xFFFF;
	int16_t magZ = 0xFFFF;
	if (!(data[6] & 0x08))
	{
		magX = (data[1] << 8) | data[0];
		magY = (data[3] << 8) | data[2];
		magZ = (data[5] << 8) | data[4];
	}

	*buffer++ = 'I';
	*buffer++ = 'M';
	*buffer++ = 'U';
	*buffer++ = ':';

	// This doesn't get unrolled automatically by GCC. 
	*buffer++ = (char)(accelX >> 8);
	*buffer++ = (char)(accelX & 0xFF);
	*buffer++ = (char)(accelY >> 8);
	*buffer++ = (char)(accelY & 0xFF);
	*buffer++ = (char)(accelZ >> 8);
	*buffer++ = (char)(accelZ & 0xFF);

	*buffer++ = (char)(temp >> 8);
	*buffer++ = (char)(temp & 0xFF);

	*buffer++ = (char)(gyroX >> 8);
	*buffer++ = (char)(gyroX & 0xFF);
	*buffer++ = (char)(gyroY >> 8);
	*buffer++ = (char)(gyroY & 0xFF);
	*buffer++ = (char)(gyroZ >> 8);
	*buffer++ = (char)(gyroZ & 0xFF);

	// Magnetometer reading is 14 bit
	*buffer++ = (char)(magX >> 8);
	*buffer++ = (char)(magX & 0xFF);
	*buffer++ = (char)(magY >> 8);
	*buffer++ = (char)(magY & 0xFF);
	*buffer++ = (char)(magZ >> 8);
	*buffer++ = (char)(magZ & 0xFF);

	*buffer++ = '|';
	*buffer = 0;

	return MESSAGE_LENGTH;
}

static void write_spi_bytes(unsigned char startAddress, unsigned char* data, size_t num_bytes)
{
	PORTL &= ~(1 << 0);

	spi_char_transaction(startAddress & ~(READ_FLAG));

	for (size_t i = 0; i < num_bytes; i++)
	{
		spi_char_transaction(data[i]);
	}

	PORTL |= (1 << 0);
}

static void read_spi_bytes(unsigned char startAddress, unsigned char* data, size_t num_bytes)
{
	PORTL &= ~(1 << 0);

	spi_char_transaction(startAddress | READ_FLAG);

	for (size_t i = 0; i < num_bytes; i++)
	{
		data[i] = spi_char_transaction(0x00);
	}

	PORTL |= (1 << 0);
}

static inline unsigned char read_spi_byte(unsigned char startAddress)
{
	unsigned char c;
	read_spi_bytes(startAddress, &c, 1);
	return c;
}

static inline void write_spi_byte(unsigned char startAddress, unsigned char data)
{
	write_spi_bytes(startAddress, &data, 1);
}


static inline unsigned char spi_char_transaction(unsigned char data)
{
	SPDR = data;
	while(!(SPSR & (1 << SPIF))) {};
	unsigned char result = SPDR;
	return result;
}

//static float to_g(int16_t raw)
//{
	//float d = raw;
	//d /= 16384.0;
	//return d;
//}