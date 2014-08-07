#include <avr/io.h>
#ifndef F_CPU
#define F_CPU 1000000UL
#endif // F_CPU
#include <util/delay.h>

int main()
{
	DDRB = 0;
	DDRC = 0;
	DDRD = 0;

	DDRC |= 1<<DDC0;
	DDRC |= 1<<DDC1;
	DDRC |= 1<<DDC2;

	PORTB = 0;
	PORTC = 0;
	PORTD = 0;
	
	while(true) {
		_delay_us(100);
	}
}