#include <avr/io.h>
#ifndef F_CPU
#define F_CPU 1000000UL
#endif // F_CPU
#include <util/delay.h>

void initialize_io();

int main()
{
	initialize_io();
	
	while (true) {
		_delay_us(100);
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
		0<<PORTB0 |
		0<<PORTB1 |
		1<<PORTB2 | // SS' defaults to high
		0<<PORTB3 | // other SPI lines should default to low
		0<<PORTB4 |
		0<<PORTB5 |
		0<<PORTB6 |
		0<<PORTB7 ;
	PORTC =
		1<<PORTC0 |	// this LED sinks current; high=off, low=on
		1<<PORTC1 |	// this LED sinks current; high=off, low=on
		1<<PORTC2 |	// this LED sinks current; high=off, low=on
		0<<PORTC3 |
		0<<PORTC4 |
		0<<PORTC5 |
		0<<PORTC6 ; // TODO: figure out if this is necessary.
		// PORTC does NOT have a 7th bit.
	PORTD =
		0<<PORTD0 |
		0<<PORTD1 |
		0<<PORTD2 |
		0<<PORTD3 |
		0<<PORTD4 |
		0<<PORTD5 |
		0<<PORTD6 |
		0<<PORTD7 ;
}
