// copyright, license, all that fun stuff
#include <avr/io.h>
#define F_CPU 1000000UL
#include <util/delay.h>
#include <util/twi.h>

void initialize_io();

int main()
{
	// TODO: Do initialization here.
	initialize_io();
	
    while (true)
    {
		// process gyro
		// read temp
		// read bump
		// read IR <- depending on mode!
		// do LEDs
    }
}

void initialize_io()
{
	// Set up I/O port directions with the DDRx registers. 1=out, 0=in.
	// These can be changed later in the program (and some sensors need
	// to do this, e.g. ultrasonic sensors).
	DDRB =
		1<<DDB0 | // BUMP_SEL_A
		1<<DDB1 | // BUMP_SEL_B
		0<<DDB2 | // SPI_SS'_B
		0<<DDB3 | // SPI_MOSI
		1<<DDB4 | // SPI_MISO
		0<<DDB5 | // SPI_SCLK
		1<<DDB6 | // BUMP_SEL_C
		0<<DDB7 ; // BUMP_COM
	DDRC =
		1<<DDC0 | // LED_SEL_C
		1<<DDC1 | // LED_SEL_B
		1<<DDC2 | // LED_SEL_A
		1<<DDC3 | // LED_COM
		0<<DDC4 | // IMU_SDA - re-init'd upon twi init
		1<<DDC5 | // IMU_SCL
		0<<DDC6 ; // RESET - init to input
		// PORTC does NOT have a 7th bit.
	DDRD =
		0<<DDD0 | // DEBUG_A
		0<<DDD1 | // DEBUG_B
		1<<DDD2 | // IR_SEL_A
		1<<DDD3 | // IR_SEL_B
		1<<DDD4 | // IR_SEL_C
		1<<DDD5 | // TEMP_SEL_A
		1<<DDD6 | // TEMP_SEL_B
		1<<DDD7 ; // TEMP_SEL_C
		
	// Initialize all outputs to 0 (LOW), and enable internal pull-ups for
	// the appropriate inputs. 1=pull-up resistor enabled. See the schematic
	// for more clarification. (Most of) SPI doesn't need pull-up resistors.
	PORTB =
		0<<PORTB0 |
		0<<PORTB1 |
		1<<PORTB2 | // SS' *does* need to be pulled up
		0<<PORTB3 | // the rest does not
		0<<PORTB4 |
		0<<PORTB5 |
		0<<PORTB6 |
		1<<PORTB7 ;
	PORTC =
		0<<PORTC0 |
		0<<PORTC1 |
		0<<PORTC2 |
		0<<PORTC3 |
		0<<PORTC4 | // IMU has strong built-in pull-up
		0<<PORTC5 | // IMU has strong built-in pull-up
		0<<PORTC6 ; // TODO: figure out if this is necessary.
		// PORTC does NOT have a 7th bit.
	PORTD =
		1<<PORTD0 |
		1<<PORTD1 |
		0<<PORTD2 |
		0<<PORTD3 |
		0<<PORTD4 |
		0<<PORTD5 |
		0<<PORTD6 |
		0<<PORTD7 ;
}
