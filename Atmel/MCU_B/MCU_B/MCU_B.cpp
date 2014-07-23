// copyright, license, all that fun stuff
#include <avr/io.h>
#define F_CPU 1000000UL
// WARNING: If you change F_CPU, you must also change:
// * ADC TIMER PRESCALER
// * TIMER PRESCALER
// * LED CYCLE TICKING (modded_time?)
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/twi.h>

void initialize_io();
void initialize_adc();
void initialize_spi();

void LED_mux_set(int id, bool isOn);

enum LedId {
	LED_1 = 0,
	LED_2,
	LED_3,
	LED_4,
	LED_5,
	LED_6,
	LED_7,
	LED_8,
	LED_NUM
};
enum LedMode // Make *sure* to update `LED_on_states` when this is updated!
{
	LED_OFF = 0,
	LED_STEADY,
	LED_FLASH,
	LED_STROBE,
	LED_BLINK,
	LED_DOUBLE_BLINK,
	LED_TRIPLE_BLINK,
	LED_FLICKER,
	LED_DOUBLE_FLICKER,
	LED_TRIPLE_FLICKER,
	LED_MODE_NUM
};



int main()
{
	const short LED_CYCLE_LENGTH = 12;
	const bool LED_on_states[LED_MODE_NUM][LED_CYCLE_LENGTH] = {
		{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // LED_OFF
		{1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // LED_STEADY
		{1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0}, // LED_FLASH
		{1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0}, // LED_STROBE
		{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}, // LED_BLINK
		{1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0}, // LED_DOUBLE_BLINK
		{1, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0}, // LED_TRIPLE_BLINK
		{0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, // LED_FLICKER
		{0, 0, 1, 0, 0, 1, 1, 1, 1, 1, 1, 1}, // LED_DOUBLE_FLICKER
		{0, 0, 1, 0, 0, 1, 0, 0, 1, 1, 1, 1}  // LED_TRIPLE_FLICKER
	};
	volatile LedMode LED_states[LED_NUM]; // quickly init this with a for loop
	for (int i=0; i<LED_NUM; ++i) {
		LED_states[i] = LED_OFF;
	}
	volatile bool doPollIR = true;

	// Initialize "system"-wide timer (TODO: make this a class...)
	uint64_t SYSTEM_TIME = 0; // in microseconds
	// TCCR1A, TCCR1B, and TCCR1C default to 0
	TCCR1B |= 1<<CS10 | 0<<CS11 | 0<<CS12; // prescaler of 1 (timer clock = system clock)

	// Initialize peripherals (everything other than timers :P )
	initialize_io();
	initialize_adc();

    while (true)
    {
		// Update system timer. TODO: Make this a class.
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			SYSTEM_TIME += TCNT1;
			TCNT1 = 0;
		}

		// process gyro
		// read temp
		// read bump

		// Read from the IR sensors.
		if (doPollIR) {
			;
		}

		// Set LEDs according to their statuses.
		short modded_time = SYSTEM_TIME/100000UL; // each "tick" is 100ms; MAGIC_NUM!
		modded_time %= LED_CYCLE_LENGTH;
		for (int i=0; i<LED_NUM; ++i) {
			bool isOn = LED_on_states[LED_states[i]][modded_time];
			LED_mux_set(i, isOn);
		}
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

void initialize_adc()
{
	// ADCL, ADCH, ADMUX, ADCSRA, and ADCSRB all default to 0.
	ADMUX |= 1<<REFS0 | 0<<REFS1; // set voltage reference: "AVCC with external capacitor at AREF pin"--datasheet
	ADCSRA |= 1<<ADPS0 | 1<<ADPS1 | 0<<ADPS2; // ADC prescaler = 8: if F_CPU is 1MHz, then ADC clock is 125kHz (within 50~200kHz)
	ADCSRA |= 1<<ADEN; // enable ADC
	// each conversion takes 13 ADC clock cycles, except for initial one (takes 25)
}

void LED_mux_set(int id, bool isOn)
{
	PORTC &= ~(0b1111 << PORTC3);
	switch (id) {
		case LED_1 :
			break;
		case LED_2 :
			break;
		case LED_3 :
			break;
		case LED_4 :
			break;
		case LED_5 :
			break;
		case LED_6 :
			break;
		case LED_7 :
			break;
		case LED_8 :
			break;
	}
	if (isOn) {
		PORTC |= 1<<PORTC3;
	}
}
