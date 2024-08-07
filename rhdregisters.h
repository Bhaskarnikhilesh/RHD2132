/*
 * rhdregisters.h
 *
 *  Created on: Jul 11, 2024
 *      Author: nikhileshbhaskar
 */

#ifndef RHDREGISTERS_H_
#define RHDREGISTERS_H_

#include <stdint.h>
#include "stm32f0xx.h"
#include "drivers.h"

uint16_t convert_command(uint8_t channel, uint8_t h_bit);
uint16_t calibrate_command();
uint16_t clear_command();
uint16_t write_command(uint8_t reg_addr, uint8_t data);
extern uint16_t read_command(uint8_t reg_addr);


#endif /* RHDREGISTERS_H_ */
