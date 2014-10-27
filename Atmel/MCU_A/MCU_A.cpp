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

enum MCU{
	MCU_B = MUX_1,
	MCU_C = MUX_2,
	MCU_NUM
};

void resync_spi(); // NOTE: BLOCKING!

void initialize_io();
void initialize_spi();

void set_SPI_mux(MuxLine line);



int main()
{
	initialize_io();
	initialize_spi();

	const int MAGIC_SPI_TRANSMIT_DELAY = 12; // actually seems to be 10 but let's be safe here
	const int MAGIC_MUX_SWITCH_DELAY = 100; // way to large but let's be generous
	
	int RGB_counter = 0;
	int LED_counter = 0;
	int LED_R = 0x01;
	int LED_G = 0x01;
	int LED_B = 0x01;
	const int CYCLE_SIZE = 16;

	bool isReadySPI = false;
	bool isCommResetting = false;
	bool isCommTransmitting = false;

	// Data variables. (NOTE: Not sure if these should be "volatile".)
	uint8_t t_isPressedDebugA	= 0x00;	// 1 bit (1 = true)
	uint8_t t_isPressedDebugB	= 0x00;	// 1 bit (1 = true)
	uint8_t t_XY_low			= 0x00;	// 8 bits
	uint8_t t_XY_high			= 0x00;	// 7 bits
	uint8_t t_Z_low				= 0x00;	// 8 bits
	uint8_t t_Z_high			= 0x00;	// 1 bit
	uint8_t t_bump_map			= 0x00;	// 8 bits
	uint8_t t_overheat_alert	= 0x00; // 1 bit
	uint8_t t_overheat_map		= 0x00; // 8 bits
	uint8_t t_IR_alert			= 0x00; // 1 bit

	// Doesn't work on an ATmega8.
	//// Delay the clock frequency change to ensure re-programmability.
	//// The argument for the delay may be completely off because at this
	//// point in the program the specified `F_CPU` does not match the
	//// actual clock's frequency.
	//ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	//	// Delay!
		_delay_ms(100);
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

	// Check if other MCUs are ready to go.
	// TODO: Periodically check for connectivity later.
	resync_spi();
	isReadySPI = true;
	
	//sei(); // ready to go.

	while (true) {
		++RGB_counter;
		RGB_counter %= 3;
		if (RGB_counter == 0) {
			++LED_counter;
			LED_counter %= CYCLE_SIZE;
		}

		auto req_data = [](uint8_t SPI_code) -> uint8_t
		{
			_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
			SPDR = SPI_code;
			while ( !(SPSR & (1<<SPIF)) ) { ; } // wait for reception to complete
			uint8_t byte_read = SPDR;
			return byte_read;
		};

		// ask for stuff, update registers
		set_SPI_mux(MUX_1);
		_delay_us(MAGIC_MUX_SWITCH_DELAY);

		PORTC |= 1<<PC5; // G

		_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		SPDR = SPI_REQ_DEBUG_A;
		//_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		while ( !(SPSR & (1<<SPIF)) ) { ; }
		//_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		uint8_t check_byte = SPDR;
		
		_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		SPDR = SPI_REQ_DEBUG_B;
		//_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		while ( !(SPSR & (1<<SPIF)) ) { ; }
		//_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		t_isPressedDebugA = SPDR;
		
		_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		SPDR = SPI_TRANSMIT_OVER;
		//_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		while ( !(SPSR & (1<<SPIF)) ) { ; }
		//_delay_us(MAGIC_SPI_TRANSMIT_DELAY);
		t_isPressedDebugB = SPDR;

		PORTC |= 1<<PC4; // Y

		//uint8_t check_link = req_data(SPI_REQ_DEBUG_A);
		//if (check_link != SPI_ACK_READY) {
			////resync_spi();
		//}
		//t_isPressedDebugA = req_data(SPI_REQ_DEBUG_B);
		//t_isPressedDebugB = req_data(SPI_REQ_Z_LOW);
		//t_Z_low = req_data(SPI_REQ_Z_HIGH);
		//t_Z_high = req_data(SPI_REQ_XY_LOW);
		//t_XY_low = req_data(SPI_REQ_XY_HIGH);
		//t_XY_high = req_data(SPI_TRANSMIT_OVER);
		
		set_SPI_mux(MUX_2);
		_delay_us(MAGIC_MUX_SWITCH_DELAY);

		// NOTE: get rid of this
		_delay_us(200); // pretend we do other stuff here

		if (t_isPressedDebugA == 0x00) {
			LED_G = 0x01;
		} else {
			LED_G = 0x00;
		}
		if (t_isPressedDebugB == 0x00) {
			LED_B = 0x01;
		} else {
			LED_B = 0x00;
		}

		// LED stuffs
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

		if (isReadySPI) {
			PORTC |= 1<<PC3; // always true for now
		} else {
			PORTC &= ~(1<<PC3); // R
		}
		//if (isCommResetting) {
		//	PORTC |= 1<<PC4;
		//} else {
		//	PORTC &= ~(1<<PC4); // Y
		//}
		//if (isCommTransmitting) {
		//	PORTC |= 1<<PC5;
		//} else {
		//	PORTC &= ~(1<<PC5); // G
		//}

		//_delay_us(10); // Only enable this delay if there aren't other delay sources.
	}
}

void resync_spi()
{
	const int delay_const = 50; // usec
	bool isSynced = false;
	int ready_counter = 0;
	
	while (!isSynced) {
		set_SPI_mux(MUX_1);
		_delay_us(delay_const);
		SPDR = SPI_REQ_CONFIRM;
		while ( !(SPSR & (1<<SPIF)) ) { ; }
		uint8_t read_byte = SPDR;
		if (read_byte == SPI_ACK_READY) {
			ready_counter++;
		} else {
			ready_counter = 0;
		}
		if (ready_counter > 25) {
			isSynced = true;
		}
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
		0<<PB2 | // SS' is active low
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
		1<<PC6 ; // TODO: figure out if this is necessary.
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

void set_SPI_mux(MuxLine line)
{
	PORTB &= ~(1 << PB0);
	PORTB &= ~(1 << PB1);
	PORTB &= ~(1 << PB6);
	PORTB &= ~(1 << PB7);
	switch (line) {
		// Hopefully this monstrosity gets optimized away. (TODO: Nope, says a simple size test.)
		case MUX_1 :
			PORTB |= 0<<PB0;
			PORTB |= 0<<PB1;
			PORTB |= 0<<PB6;
			PORTB |= 0<<PB7;
			break;
		case MUX_2 :
			PORTB |= 1<<PB0;
			PORTB |= 0<<PB1;
			PORTB |= 0<<PB6;
			PORTB |= 0<<PB7;
			break;
		case MUX_3 :
			PORTB |= 0<<PB0;
			PORTB |= 1<<PB1;
			PORTB |= 0<<PB6;
			PORTB |= 0<<PB7;
			break;
		case MUX_4 :
			PORTB |= 1<<PB0;
			PORTB |= 1<<PB1;
			PORTB |= 0<<PB6;
			PORTB |= 0<<PB7;
			break;
		case MUX_5 :
			PORTB |= 0<<PB0;
			PORTB |= 0<<PB1;
			PORTB |= 1<<PB6;
			PORTB |= 0<<PB7;
			break;
		case MUX_6 :
			PORTB |= 1<<PB0;
			PORTB |= 0<<PB1;
			PORTB |= 1<<PB6;
			PORTB |= 0<<PB7;
			break;
		case MUX_7 :
			PORTB |= 0<<PB0;
			PORTB |= 1<<PB1;
			PORTB |= 1<<PB6;
			PORTB |= 0<<PB7;
			break;
		case MUX_8 :
			PORTB |= 1<<PB0;
			PORTB |= 1<<PB1;
			PORTB |= 1<<PB6;
			PORTB |= 0<<PB7;
			break;
		default :
			// YOU HAVE MADE A GRAVE ERROR
			break;
	}

	PORTB &= ~(1<<PB2); // `SS always is active low
}
