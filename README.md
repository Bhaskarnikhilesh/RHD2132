# RHD2132 ADC SPI Interface
Firmware for STM32F030R8 board to capture the RHD2132 converted ADC amplifier channel 1 data and other supporting resources are available for the A HIGH-SPEED MULTI-CHANNEL EMG SIGNAL CAPTURE DEVICE WITH LATER SIGNAL ANALYSIS project
This project demonstrates how to interface with the RHD2000 ADC using SPI communication on an STM32 microcontroller. The code includes configurations for SPI, GPIO, UART with DMA, and timers. It also provides functions to perform ADC calibration and read data from the ADC.

# Hardware Setup

Microcontroller
Microcontroller: STM32F0 series 
RHD2000 ADC
RHD2000 series: RHD2132
Connections
SPI Interface:

SCLK: PA5
MISO: PA6
MOSI: PA7
CS: PA9
UART Interface (for DMA):

TX: PA2
Error LED:

LED: PB13
Power Supply: 3.3V

Ensure that the ADC and microcontroller are properly powered according to their specifications.
Software Setup
Required Libraries
STM32F0xx CMSIS libraries (if using STM32CubeMX, these are typically included)
STM32CubeIDE for development and debugging

# File Descriptions
main.c: Main application file containing initialization and main loop.
drivers.h: Header file defining driver functions and configurations.
rhdregisters.h: Header file with register definitions and command functions for RHD2000 ADC.
spi_transmit_receive(): Function to send and receive data via SPI.
calibrate_command(): Function to generate a calibration command for the ADC.
write_command(): Function to write data to a specific register.
read_command(): Function to read data from a specific register.
SystickDelay_Microsec(): Function to provide microsecond delays using SysTick timer.
Build and Flash
Open STM32CubeIDE and create a new STM32 project or open the existing project containing these files.

Build the Project:

Click on the "Build" button to compile the code.
Flash the Microcontroller:

Connect the STM32 microcontroller to your computer using a debugger/programmer.
Click on the "Debug" button to flash the code onto the microcontroller.
Configuration
SPI Configuration:

SPI settings are configured in spi_configure() function for communication with the ADC.
UART Configuration:

UART settings for DMA transmission are configured in uart2_tx_DMA_init().
Timer Configuration:

Timer3 is configured to generate interrupts at 30 kHz in tim3_30kS_init_interrupt().
ADC Initialization:

ADC configuration and calibration are handled in adc_init() and RHD_Read() functions.
Usage
Initialization:

Ensure all peripherals are properly initialized by calling spi_gpio_init(), spi_configure(), uart2_tx_DMA_init(), and adc_init().
Start Data Acquisition:

The tim3_30kS_init_interrupt() function sets up a periodic interrupt to read ADC data.

Monitor UART output for DMA data transmission.
