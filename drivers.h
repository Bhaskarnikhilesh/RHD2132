/*
 * drivers.h
 *
 *  Created on: Jul 18, 2024
 *      Author: nikhileshbhaskar
 */

#ifndef DRIVERS_H_
#define DRIVERS_H_

#include "stm32f0xx.h"
#include <stdint.h>
#include "rhdregisters.h"
#include <stdio.h>

#define GPIOAEN (1U << 17)
#define GPIOBEN (1U << 18)
#define TIM3ENR (1U<<1)
#define CR1EN	(1U<<0)
#define UIE		(1U<<0)
#define UIF		(1U<<0)
#define UART2EN (1U<<17)
#define DMAT	(1U<<7)
#define CR1_TE	(1U<<3)
#define UART2_UE (1U<<0)
#define DMAEN	(1U<<0)
#define DMA_EN	(1U<<0)
#define CTEIF4  (1U<<15)
#define CTCIF4  (1U<<13)
#define CGIF4	(1U<<12)
#define CHTIF4	(1U<<14)
#define MINC	(1U<<7)
#define DIR		(1U<<4)
#define ADC_CHANNEL          (1)
#define ADC_CHANNEL2          (1)
#define H_BIT                (0)
#define MICROSEC_DELAY 200
#define SYSTICK_LOAD_VAL 9600
#define SYSTICK_CTRL_EN (1U<<0)
#define SYSTICK_CLKSRC (1U<<2)
#define SYSTICK_COUNTFLAG (1U<<16)

#define REGISTER_0  (1U<<7) | (1U<<6) | (0U<<5) | (1U<<4) | (1U<<3) | (1U<<2) | (1U<<1) | (0U<<0)
#define REGISTER_1  (0U<<7) | (1U<<6) | (1U<<5) | (0U<<4) | (0U<<3) | (0U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_2  (0U<<7) | (0U<<6) | (1U<<5) | (0U<<4) | (1U<<3) | (0U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_3  (0U<<7) | (0U<<6) | (0U<<5) | (0U<<4) | (0U<<3) | (0U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_4  (0U<<7) | (0U<<6) | (0U<<5) | (1U<<4) | (0U<<3) | (1U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_5  (0U<<7) | (0U<<6) | (0U<<5) | (0U<<4) | (0U<<3) | (0U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_6  (0U<<7) | (0U<<6) | (0U<<5) | (0U<<4) | (0U<<3) | (0U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_7  (0U<<7) | (0U<<6) | (0U<<5) | (0U<<4) | (0U<<3) | (0U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_8  (0U<<7) | (0U<<6) | (0U<<5) | (1U<<4) | (0U<<3) | (1U<<2) | (1U<<1) | (0U<<0)
#define REGISTER_9  (0U<<7) | (0U<<6) | (0U<<5) | (0U<<4) | (0U<<3) | (0U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_10 (0U<<7) | (0U<<6) | (0U<<5) | (1U<<4) | (0U<<3) | (1U<<2) | (1U<<1) | (1U<<0)
#define REGISTER_11 (0U<<7) | (0U<<6) | (0U<<5) | (0U<<4) | (0U<<3) | (0U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_12 (0U<<7) | (0U<<6) | (1U<<5) | (0U<<4) | (1U<<3) | (1U<<2) | (0U<<1) | (0U<<0)
#define REGISTER_13 (0U<<7) | (0U<<6) | (0U<<5) | (0U<<4) | (0U<<3) | (1U<<2) | (1U<<1) | (0U<<0)
#define REGISTER_14 (1U<<7) | (1U<<6) | (1U<<5) | (1U<<4) | (1U<<3) | (1U<<2) | (1U<<1) | (1U<<0)
#define REGISTER_15 (1U<<7) | (1U<<6) | (1U<<5) | (1U<<4) | (1U<<3) | (1U<<2) | (1U<<1) | (1U<<0)
#define REGISTER_16 (1U<<7) | (1U<<6) | (1U<<5) | (1U<<4) | (1U<<3) | (1U<<2) | (1U<<1) | (1U<<0)
#define REGISTER_17 (1U<<7) | (1U<<6) | (1U<<5) | (1U<<4) | (1U<<3) | (1U<<2) | (1U<<1) | (1U<<0)

#define USART_BAUDRATE 115200
#define SYS_CLK_FREQ   48000000
#define BUFFER_SIZE   128

void tim3_20kS_init_interrupt(void);
void spi_gpio_init(void);
void spi_configure(void);
void error_led_init(void);
void error_led_on(void);
void error_led_off(void);
//void spi_transmit(uint16_t *data);
//uint16_t spi1_receive(void);
void spi1_receive(uint16_t *data2, uint16_t size);
//void spi_transmit(uint16_t *data1, uint16_t size);
uint16_t spi_transmit_receive(uint16_t txdata1);
void adxl_write(uint16_t address, uint16_t value);
void adxl_read(uint16_t address, uint16_t *rxdata);
void adxl_init(void);
void cs_enable(void);
void cs_disable(void);
void uart2_tx_DMA_init(void);
void adc_init(void);
void RHD_Read(void);
void dummy_command_list(void);
void SystickDelay_Microsec(uint16_t delay);
uint16_t read_command(uint8_t reg_addr);


#endif /* DRIVERS_H_ */
