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

#include "Common.h"

int init_imu();
size_t read_and_append_imu_reading(char* buffer, size_t remainingBytes);

#define READ_FLAG          0x80

#define SAMPLE_RATE_DIVIDER_MPU9250 0x19
#define CONFIG_MPU9250              0x1A
#define GYRO_CONFIG_MPU9250         0x1B
#define ACCEL_CONFIG_1_MPU9250      0x1C
#define ACCEL_CONFIG_2_MPU9250      0x1D
#define I2C_MSTR_CTRL_MPU9250       0x24
#define I2C_SLV4_ADDR_MPU9250       0x31
#define I2C_SLV4_REG_MPU9250        0x32
#define I2C_SLV4_DO_MPU9250         0x33
#define I2C_SLV4_CTRL_MPU9250       0x34
#define I2C_SLV4_DI_MPU9250         0x35
#define I2C_MASTER_STATUS_MPU9250   0x36
#define DATA_START_MPU9250          0x3B
#define USER_CTRL_MPU9250           0x6A
#define PWR_MGMT_1_MPU9250          0x6B
#define PWR_MGMT_2_MPU9250          0x6C
#define WHO_AM_I_MPU9250            0x75

//wtf
#define I2C_SLV0_ADDR_MPU9250       0x25
#define I2C_SLV0_REG_MPU9250        0x26
#define I2C_SLV0_DO_MPU9250         0x27
#define I2C_SLV0_CTRL_MPU9250       0x28
#define I2C_SLV0_DI_MPU9250         0x29

#define ADDRESS_AK8963              0x0C

#define WHO_AM_I_AK8963             0x00
#define DATA_START_AK8963           0x03

#define INT_PIN_CFG_MPU9250         55

#endif /* MPU9250IMU_H_ */