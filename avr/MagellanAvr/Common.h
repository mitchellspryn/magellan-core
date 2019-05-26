/*
 * Common.h
 *
 * Created: 5/20/2019 9:09:06 PM
 *  Author: Mitchell
 */ 


#ifndef COMMON_H_
#define COMMON_H_

#include <stdint.h>

#define F_CPU 16000000
#define BAUD_TOL 2

#include <util/delay.h>

void get_9600_baud_settings(uint16_t *ubrrh, uint16_t *ubrrl, unsigned char* use2x);
void get_38400_baud_settings(uint16_t *ubrrh, uint16_t *ubrrl, unsigned char* use2x);
void get_76800_baud_settings(uint16_t *ubrrh, uint16_t *ubrrl, unsigned char* use2x);

void delay_one_second();

#endif /* COMMON_H_ */