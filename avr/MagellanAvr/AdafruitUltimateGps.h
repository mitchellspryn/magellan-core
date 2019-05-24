/*
 * AdafruitUltimateGps.h
 *
 * Created: 5/20/2019 9:08:35 PM
 *  Author: Mitchell
 */ 


#ifndef ADAFRUITULTIMATEGPS_H_
#define ADAFRUITULTIMATEGPS_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "Common.h"

void init_gps();
size_t append_gps_reading(char* buffer, size_t remainingBytes);

#endif /* ADAFRUITULTIMATEGPS_H_ */