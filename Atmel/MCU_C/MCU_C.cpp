// copyright, license, all that fun stuff
#include <inttypes.h>
#include <math.h>
#include <avr/io.h>
#ifndef F_CPU
#define F_CPU 8000000UL
#endif // F_CPU
// WARNING: If you change F_CPU, you must also change:
// * ADC TIMER PRESCALER
// * TIMER PRESCALER
// * LED CYCLE TICKING (modded_time?)
// * ANYTHING TO DO WITH `dt`
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/twi.h>
#include <avr/eeprom.h>

#include "../lib/SPI-codes.h" 

enum MuxLine {
	MUX_1 = 0,
	MUX_2,
	MUX_3,
	MUX_4,
	MUX_5,
	MUX_6,
	MUX_7,
	MUX_8,
	MUX_NUM
};

// Control variables.
volatile bool isReady = false;

// Data variables.
volatile uint8_t t_light_map_A		= 0x00;	// 8 bits
volatile uint8_t t_light_map_B		= 0x00;	// 8 bits
volatile uint8_t t_light_A[MUX_NUM]	= {0,0,0,0,0,0,0,0};	// 4 bits each
volatile uint8_t t_light_B[MUX_NUM]	= {0,0,0,0,0,0,0,0};	// 4 bits each

// SPI variables.
volatile uint8_t byte_read = SPI_UNOWN;
volatile uint8_t byte_write = SPI_UNOWN;

//// Functions.
//void initialize_io();
//void initialize_adc();
//void initialize_spi();
//void initialize_pcint();

int main()
{
	while (true) {
	}
}
