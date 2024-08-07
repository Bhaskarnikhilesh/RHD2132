/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#include <stdint.h>
#include "drivers.h"
#include "rhdregisters.h"
#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int main(void)
{
	//uint16_t reading;
	// Initialize GPIO for SPI
		 spi_gpio_init();
		// Configure SPI settings
		spi_configure();
		//error_led_init();
		// Initialize ADXL sensor
		//spi_transmit_receive(read_command(40));
		// Initialize UART for DMA transmission
		uart2_tx_DMA_init();

		//adxl_init();
	    adc_init();
		//RHD_Read();
	   //for(int nice = 0; nice<10; nice++){
	   //spi_transmit_receive(read_command(40));
	  //}
		tim3_20kS_init_interrupt();




		while (1) {
			//spi_transmit_receive(read_command(40));
		    return 0;
		}
}