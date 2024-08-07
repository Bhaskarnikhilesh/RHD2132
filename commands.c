#include "rhdregisters.h"
#include <math.h>

// Run analog-to-digital conversion on specified channel.
// If hBit is true when DSP offset removal is enabled, then the output of the digital HPF is reset to zero.
// A special case with channel = 63 can be used to cycle through successive amplifier channels,
// so long as at least one defined-channel convert command is called first.
// Once sent, SPI returns (2 commands later) the 16-bit result of this conversion.
// Command: 00_C[5]-C[0]_0000000H for channel C and hBit H
// Result:  A[15]-A[0] for ADC conversion output A
uint16_t convert_command(uint8_t channel, uint8_t h_bit) {
	const uint16_t convert_mask = 0b0000000000000000;
	const uint16_t channel_mask = channel << 8;
	const uint16_t h_bit_mask = (uint16_t) h_bit;
	return convert_mask | channel_mask | h_bit_mask;
}


// Initiate ADC self-calibration routine.
// Self-calibration should be performed after chip power-up and register configuration.
// This takes 9 clock cycles to execute - 9 "dummy" commands should be sent after a calibrate command.
// These dummy commands are not executed (unless another calibration command is sent, which resets the process).
// During the entire 9-command process, the results are all 0s except the for the MSB.
// The MSB will be 0 if 2's complement mode is enabled (see Register 4), otherwise it will be 1.
// Command: 01010101_00000000
// Result:  *0000000_00000000 where * depends on 2's complement mode
uint16_t calibrate_command() {
	return 0b0101010100000000;
}

// Clear ADC calibration.
// Clears the calibration parameters acquired by running the above calibrate command.
// In normal operation, it is not necessary to execute this command.
// Once sent, SPI returns (2 commands later) all 0s except for the MSB.
// The MSB will be 0 if 2's complement mode is enabled (see Register 4), otherwise it will be 1.
// Command: 01101010_00000000
// Result:  *0000000_00000000 where * depends on 2's complement mode
uint16_t clear_command() {
	return 0b0110101000000000;
}

// Write data to register.
// Writes 8 bits of data to specified registers.
// Once sent, SPI returns (2 commands later) 8 MSBs of 1s, and 8 LSBs of the
// echoed data that was written (to verify reception of correct data).
// Any attempt to write to a read-only register (or non-existent register) will produce this same result,
// but data will not be written to that register.
// Command: 10_R[5]-R[0]_D[7]-D[0]
// Result:  11111111_D[7]-D[0]
uint16_t write_command(uint8_t reg_addr, uint8_t data) {
	const uint16_t read_mask = 0b1000000000000000;
	const uint16_t reg_mask = reg_addr << 8;
	return read_mask | reg_mask | (uint16_t) data;
}

// Read contents of register.
// Once sent, SPI returns (2 commands later) 8 MSBs of 0s, and 8 LSBs of the read data.
// Command: 11_R[5]-R[0]_00000000
// Result:  00000000_D[7]-D[0]
uint16_t read_command(uint8_t reg_addr) {
	const uint16_t read_mask = 0b1100000000000000;
	const uint16_t reg_mask = reg_addr << 8;
	return read_mask | reg_mask;
}

