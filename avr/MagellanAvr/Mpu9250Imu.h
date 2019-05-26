/*
 * Mpu9250Imu.h
 *
 * Created: 5/25/2019 4:39:37 PM
 *  Author: Mitchell
 */ 


#ifndef MPU9250IMU_H_
#define MPU9250IMU_H_

#include <avr/io.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

void init_imu();
size_t read_and_append_imu_reading(char* buffer, size_t remainingBytes);

#define READ_FLAG          0x80

#define WHO_AM_I_MPU9250   0x75

#endif /* MPU9250IMU_H_ */