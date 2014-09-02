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
#include <avr/eeprom.h>

#include "../../lib/SPI-codes.h"

void initialize_io();
void initialize_spi();



int main()
{
	initialize_io();
	initialize_spi();
	
	int RGB_counter = 0;
	int LED_counter = 0;
	int LED_R = 222/4;
	int LED_G = 108/4;
	int LED_B = 173/4;

	// Doesn't work on an ATmega8.
	//// Delay the clock frequency change to ensure re-programmability.
	//// The argument for the delay may be completely off because at this
	//// point in the program the specified `F_CPU` does not match the
	//// actual clock's frequency.
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	//	// Delay!
	//	_delay_ms(100);
	//
	//	//CLKPR = 1 << CLKPCE; // "only updated when [... the other bits are] written to zero"
	//	//CLKPR &= ~(1<<CLKPS0);
	//	//CLKPR &= ~(1<<CLKPS1);
	//	//CLKPR &= ~(1<<CLKPS2);
	//	//CLKPR &= ~(1<<CLKPS3);
	//	//// ^ (0b0000) Corresponds to a division factor of 1.
	//
	//	// Someone else's way of doing it.
	//	CLKPR = 0x80;
	//	CLKPR = 0x00;
	//}

	while (true) {
		++RGB_counter;
		RGB_counter %= 3;
		if (RGB_counter == 0) {
			++LED_counter;
			LED_counter %= 128;
		}

		PORTC |= 0b111<<PC0;
		switch (RGB_counter) {
			case 0 :
				if (LED_counter < LED_R) {
					PORTC &= ~(1<<PC0);
				}
				break;
			case 1 :
				if (LED_counter < LED_G) {
					PORTC &= ~(1<<PC1);
				}
				break;
			case 2 :
				if (LED_counter < LED_B) {
					PORTC &= ~(1<<PC2);
				}
				break;
		}

		_delay_us(10);
	}
}

void initialize_io()
{
	// Set up I/O port directions with the DDRx registers. 1=out, 0=in.
	// These can be changed later in the program (and some sensors need
	// to do this, e.g. ultrasonic sensors).
	DDRB =
		1<<DDB0 | // SPI_SS'_A
		1<<DDB1 | // SPI_SS'_B
		1<<DDB2 | // SPI_SS'_COM
		1<<DDB3 | // SPI_MOSI
		0<<DDB4 | // SPI_MISO
		1<<DDB5 | // SPI_SCLK
		1<<DDB6 | // SPI_SS'_C
		1<<DDB7 ; // SPI_SS'_D
	DDRC =
		1<<DDC0 | // LED_RGB_R
		1<<DDC1 | // LED_RGB_G
		1<<DDC2 | // LED_RGB_B
		1<<DDC3 | // LED_COMM_R
		1<<DDC4 | // LED_COMM_Y
		1<<DDC5 | // LED_COMM_G
		0<<DDC6 ; // RESET - init to input
		// PORTC does NOT have a 7th bit.
	DDRD =
		1<<DDD0 | // NXT_MOSI_F
		1<<DDD1 | // NXT_MOSI_E
		1<<DDD2 | // NXT_MOSI_D
		1<<DDD3 | // NXT_MOSI_C
		1<<DDD4 | // NXT_MOSI_B
		1<<DDD5 | // NXT_MOSI_A
		0<<DDD6 | // NXT_MISO
		0<<DDD7 ; // NXT_SCLK
		
	// Initialize all outputs to 0 (LOW), and enable internal pull-ups for
	// the appropriate inputs. 1=pull-up resistor enabled. See the schematic
	// for more clarification. (Most of) SPI doesn't need pull-up resistors.
	PORTB =
		0<<PB0 |
		0<<PB1 |
		1<<PB2 | // SS' defaults to high
		0<<PB3 | // other SPI lines should default to low
		0<<PB4 |
		0<<PB5 |
		0<<PB6 |
		0<<PB7 ;
	PORTC =
		1<<PC0 |	// this LED sinks current; high=off, low=on
		1<<PC1 |	// this LED sinks current; high=off, low=on
		1<<PC2 |	// this LED sinks current; high=off, low=on
		0<<PC3 |
		0<<PC4 |
		0<<PC5 |
		0<<PC6 ; // TODO: figure out if this is necessary.
		// PORTC does NOT have a 7th bit.
	PORTD =
		0<<PD0 |
		0<<PD1 |
		0<<PD2 |
		0<<PD3 |
		0<<PD4 |
		0<<PD5 |
		0<<PD6 |
		0<<PD7 ;
}

void initialize_spi()
{
	SPCR |= 1<<SPIE; // Enable SPI interrupts
	SPCR |= 0<<DORD; // MSB transmitted first
	SPCR |= 1<<MSTR; // master mode
	SPCR |= 0<<CPOL | 0<<CPHA; // SPI Mode 0; just needs to be consistent
	SPCR |= 1<<SPE; // Enable SPI
	// SPR0, SPR1, and SPI2X all default to 0 (f/4).
}
