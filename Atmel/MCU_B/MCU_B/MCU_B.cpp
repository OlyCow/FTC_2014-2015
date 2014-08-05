// copyright, license, all that fun stuff
#include <avr/io.h>
#define F_CPU 1000000UL
// WARNING: If you change F_CPU, you must also change:
// * ADC TIMER PRESCALER
// * TIMER PRESCALER
// * LED CYCLE TICKING (modded_time?)
// * ANYTHING TO DO WITH `dt`
#include <util/delay.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/twi.h>

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

// Volatile variables shared between ISRs and main.
volatile LedMode LED_states[MUX_NUM]; // remember to initialize to `LED_OFF`.
volatile bool doPollIR = true;
volatile bool isPressedDebugA = false;
volatile bool isPressedDebugB = false;
volatile bool isOverheat[MUX_NUM]; // remember to initialize to `false`.
volatile uint8_t IR[MUX_NUM]; // remember to initialize to `0`.
volatile bool isClosed[MUX_NUM]; // remember to initialize to `false`.

void initialize_io();
void initialize_adc();
void initialize_spi();

void setLED(MuxLine line, LedMode mode);

void LED_mux_set(MuxLine line, bool isOn);
void bump_mux_set(MuxLine line);
void temp_mux_set(MuxLine line);
void IR_mux_set(MuxLine line);



int main()
{
	int dt = 0; // microseconds, I believe.
	short modded_time = 0;
	const short LED_CYCLE_LENGTH = 12;
	const short OVERHEAT_THRESHOLD = 200; // TODO: No clue what this number should be... consult datasheets
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
	int current_MUX = MUX_1;
	bool isOn = false; // for LED cycling
	bool isPressedDebugA_prev = false;
	bool isPressedDebugB_prev = false;
	bool isClosed_prev[MUX_NUM];
	int debugA_bounce_timer = 0;
	int debugB_bounce_timer = 0;
	int isClosed_bounce_timer[MUX_NUM];
	const int debounce_delay = 15 *1000; // in microseconds, so the first number is in milliseconds.
	for (int i=0; i<MUX_NUM; ++i) {
		LED_states[i] = LED_OFF;
		isOverheat[i] = false;
		IR[i] = 0;
		isClosed[i] = false;
		isClosed_prev[i] = false;
		isClosed_bounce_timer[i] = 0;
	}

	// Initialize "system"-wide timer (TODO: make this a class...)
	uint64_t SYSTEM_TIME = 0; // in microseconds
	// TCCR1A, TCCR1B, and TCCR1C default to 0
	TCCR1B |= 1<<CS10 | 0<<CS11 | 0<<CS12; // prescaler of 1 (timer clock = system clock)

	// Initialize peripherals (everything other than timers :P )
	initialize_io();
	initialize_adc();
	initialize_spi();

	setLED(MUX_1, LED_BLINK); // All is well. :P

    while (true)
    {
		// Update system timer. TODO: Make this a class.
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			dt = TCNT1;
			SYSTEM_TIME += TCNT1;
			TCNT1 = 0;
		}
		current_MUX++;
		current_MUX %= MUX_NUM;

		// process gyro

		// read temp
		// set temp MUX line
		temp_mux_set(static_cast<MuxLine>(current_MUX));

		// set ADC MUX line (ADC6)
		ADMUX &= ~(0b1111 << MUX0); // clear bits MUX3:0
		//ADMUX |= 0<<MUX0;
		ADMUX |= 1<<MUX1;
		ADMUX |= 1<<MUX2;
		//ADMUX |= 0<<MUX3; // We've set bits MUX3..0 to 0b0110 (ADC6).

		// start conversion
		// TODO: Do we need a delay here?
		ADCSRA |= 1<<ADSC;

		// wait for conversion to finish
		while ((ADCSRA & 1<<ADSC) != 0) {;} // do nothing
		// ADSC is set to `0` when the conversion finishes.

		// update variable(s)
		isOverheat[current_MUX] = ADCH>OVERHEAT_THRESHOLD;

		// read bump
		for (int i=0; i<MUX_NUM; ++i) {
			// Set the appropriate mux line.
			bump_mux_set(static_cast<MuxLine>(i));

			// Read.
			isClosed_prev[i] = isClosed[i];
			if ((PINB & (1<<PINB7)) != 0) { // The line has a pull-up resistor.
				isClosed[i] = false;
			} else {
				isClosed[i] = true;
			} // TODO: Atomic stuff (`isClosed[]` needs to be buffered).

			// Debounce.
			if (isClosed[i] == isClosed_prev[i]) {
				isClosed_bounce_timer[i] = 0;
			} else {
				isClosed_bounce_timer[i] += dt;
				if (isClosed_bounce_timer[i] < debounce_delay) {
					isClosed[i] = isClosed_prev[i];
				} else {
					isClosed_bounce_timer[i] = 0;
				}
				// TODO: ^Might not need to explicitly clear this (will  be cleared by next iteration?).
			}
		}

		// Check the two pushbutton switches (PD0 & PD1). Remember,
		// these both have pull-up resistors.
		isPressedDebugA_prev = isPressedDebugA;
		isPressedDebugB_prev = isPressedDebugB;
		if ((PIND & (1<<PIND0)) != 0) {
			isPressedDebugA = false;
		} else {
			isPressedDebugA = true;
		}
		if ((PIND & (1<<PIND1)) != 0) {
			isPressedDebugB = false;
		} else {
			isPressedDebugB = true;
		}
		// Debounce!
		if (isPressedDebugA == isPressedDebugA_prev) {
			debugA_bounce_timer = 0; // this possibility is more common.
		} else {
			debugA_bounce_timer += dt;
			if (debugA_bounce_timer < debounce_delay) {
				isPressedDebugA = isPressedDebugA_prev;
			} else {
				debugA_bounce_timer = 0;
				// TODO: ^Might not need to explicitly clear this (will  be cleared by next iteration?).
			}
		}
		if (isPressedDebugB == isPressedDebugB_prev) {
			debugB_bounce_timer = 0; // this possibility is more common.
		} else {
			debugB_bounce_timer += dt;
			if (debugB_bounce_timer < debounce_delay) {
				isPressedDebugB = isPressedDebugB_prev;
			} else {
				debugB_bounce_timer = 0;
				// TODO: ^Might not need to explicitly clear this (will be cleared by next iteration?).
			}
		}
		if (isPressedDebugA) {
			setLED(MUX_2, LED_STEADY);
		} else {
			setLED(MUX_2, LED_OFF);
		}
		if (isPressedDebugB) {
			setLED(MUX_3, LED_STEADY);
		} else {
			setLED(MUX_3, LED_OFF);
		}

		// Read from the IR sensors.
		if (doPollIR) {
			// set IR MUX line
			IR_mux_set(static_cast<MuxLine>(current_MUX));

			// Set ADC MUX line (ADC7 for IR).
			ADMUX &= ~(0b1111 << MUX0); // clear bits MUX3:0
			ADMUX |= 1<<MUX0;
			ADMUX |= 1<<MUX1;
			ADMUX |= 1<<MUX2;
			//ADMUX |= 0<<MUX3; // We've set bits MUX3..0 to 0b0111 (ADC7).

			// start conversion
			// TODO: Do we need a delay here?
			ADCSRA |= 1<<ADSC;

			// wait for conversion to finish
			while ((ADCSRA & 1<<ADSC) != 0) {;} // do nothing
			// ADSC is set to `0` when the conversion finishes.

			// update variable(s)
			IR[current_MUX] = ADCH;
		}

		// Set LEDs according to their statuses.
		modded_time = SYSTEM_TIME/100000UL; // each "tick" is 100ms; MAGIC_NUM!
		modded_time %= LED_CYCLE_LENGTH;
		isOn = LED_on_states[LED_states[current_MUX]][modded_time];
		LED_mux_set(static_cast<MuxLine>(current_MUX), isOn);
    }
}

void initialize_io()
{
	// Set up I/O port directions with the DDRx registers. 1=out, 0=in.
	// These can be changed later in the program (and some sensors need
	// to do this, e.g. ultrasonic sensors).
	// ADC6 reads from the temp mux, ADC7 reads from the IR mux.
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
	// Each conversion takes 13 ADC clock cycles, except for the initial one (takes 25).

	// set voltage reference: "AVCC with external capacitor at AREF pin"--datasheet
	ADMUX |= 1<<REFS0 | 0<<REFS1;

	// Left-adjust the ADC result (we don't need 10-bit accuracy).
	ADMUX |= 1<<ADLAR;

	// ADC prescaler = 4: if F_CPU is 1MHz, then ADC clock is 250kHz (above 50~200kHz, but it should be accurate enough)
	ADCSRA |= 0<<ADPS0 | 1<<ADPS1 | 0<<ADPS2;

	// enable ADC
	ADCSRA |= 1<<ADEN;
}

void initialize_spi()
{
	SPCR |= 1<<SPIE; // Enable SPI interrupts
	SPCR |= 0<<DORD; // MSB transmitted first
	SPCR |= 0<<MSTR; // slave mode
	SPCR |= 0<<CPOL | 0<<CPHA; // SPI Mode 0; just needs to be consistent
	SPCR |= 1<<SPE; // Enable SPI
	// SPR0, SPR1, and SPI2X have no effect on slave (only master), and all default to 0.
}

void setLED(MuxLine line, LedMode mode)
{
	LED_states[line] = mode; // Really tempted to cast to and from const here, but whatever.
}

void LED_mux_set(MuxLine line, bool isOn)
{
	PORTC &= ~(1 << PORTC3);
	PORTC &= ~(1 << PORTC2);
	PORTC &= ~(1 << PORTC1);
	PORTC &= ~(1 << PORTC0);
	switch (line) {
		// Hopefully this monstrosity gets optimized away. (TODO: Nope, says a simple size test.)
		case MUX_1 :
			// PORTC |= 0<<PORTC2;
			// PORTC |= 0<<PORTC1;
			// PORTC |= 0<<PORTC0;
			break;
		case MUX_2 :
			PORTC |= 1<<PORTC2;
			// PORTC |= 0<<PORTC1;
			// PORTC |= 0<<PORTC0;
			break;
		case MUX_3 :
			// PORTC |= 0<<PORTC2;
			PORTC |= 1<<PORTC1;
			// PORTC |= 0<<PORTC0;
			break;
		case MUX_4 :
			PORTC |= 1<<PORTC2;
			PORTC |= 1<<PORTC1;
			// PORTC |= 0<<PORTC0;
			break;
		case MUX_5 :
			// PORTC |= 0<<PORTC2;
			// PORTC |= 0<<PORTC1;
			PORTC |= 1<<PORTC0;
			break;
		case MUX_6 :
			PORTC |= 1<<PORTC2;
			// PORTC |= 0<<PORTC1;
			PORTC |= 1<<PORTC0;
			break;
		case MUX_7 :
			// PORTC |= 0<<PORTC2;
			PORTC |= 1<<PORTC1;
			PORTC |= 1<<PORTC0;
			break;
		case MUX_8 :
			PORTC |= 1<<PORTC2;
			PORTC |= 1<<PORTC1;
			PORTC |= 1<<PORTC0;
			break;
		default :
			// YOU HAVE MADE A GRAVE ERROR
			break;
	}
	if (isOn != 0) {
		PORTC |= 1<<PORTC3;
	}
	//_delay_ms(1); // NOTE: Only enable if the loop doesn't have other delay sources (e.g. ADC).
}

void bump_mux_set(MuxLine line)
{
	PORTB &= ~(1 << PORTB0);
	PORTB &= ~(1 << PORTB1);
	PORTB &= ~(1 << PORTB6);
	switch (line) {
		// Hopefully this monstrosity gets optimized away. (TODO: Nope, says a simple size test.)
		case MUX_1 :
			// PORTB |= 0<<PORTB0;
			// PORTB |= 0<<PORTB1;
			// PORTB |= 0<<PORTB6;
			break;
		case MUX_2 :
			PORTB |= 1<<PORTB0;
			// PORTB |= 0<<PORTB1;
			// PORTB |= 0<<PORTB6;
			break;
		case MUX_3 :
			// PORTB |= 0<<PORTB0;
			PORTB |= 1<<PORTB1;
			// PORTB |= 0<<PORTB6;
			break;
		case MUX_4 :
			PORTB |= 1<<PORTB0;
			PORTB |= 1<<PORTB1;
			// PORTB |= 0<<PORTB6;
			break;
		case MUX_5 :
			// PORTB |= 0<<PORTB0;
			// PORTB |= 0<<PORTB1;
			PORTB |= 1<<PORTB6;
			break;
		case MUX_6 :
			PORTB |= 1<<PORTB0;
			// PORTB |= 0<<PORTB1;
			PORTB |= 1<<PORTB6;
			break;
		case MUX_7 :
			// PORTB |= 0<<PORTB0;
			PORTB |= 1<<PORTB1;
			PORTB |= 1<<PORTB6;
			break;
		case MUX_8 :
			PORTB |= 1<<PORTB0;
			PORTB |= 1<<PORTB1;
			PORTB |= 1<<PORTB6;
			break;
		default :
			// YOU HAVE MADE A GRAVE ERROR
			break;
	}
}

void temp_mux_set(MuxLine line)
{
	PORTD &= ~(1 << PORTD5);
	PORTD &= ~(1 << PORTD6);
	PORTD &= ~(1 << PORTD7);
	switch (line) {
		// Hopefully this monstrosity gets optimized away. (TODO: Nope, says a simple size test.)
		case MUX_1 :
			// PORTD |= 0<<PORTD5;
			// PORTD |= 0<<PORTD6;
			// PORTD |= 0<<PORTD7;
			break;
		case MUX_2 :
			PORTD |= 1<<PORTD5;
			// PORTD |= 0<<PORTD6;
			// PORTD |= 0<<PORTD7;
			break;
		case MUX_3 :
			// PORTD |= 0<<PORTD5;
			PORTD |= 1<<PORTD6;
			// PORTD |= 0<<PORTD7;
			break;
		case MUX_4 :
			PORTD |= 1<<PORTD5;
			PORTD |= 1<<PORTD6;
			// PORTD |= 0<<PORTD7;
			break;
		case MUX_5 :
			// PORTD |= 0<<PORTD5;
			// PORTD |= 0<<PORTD6;
			PORTD |= 1<<PORTD7;
			break;
		case MUX_6 :
			PORTD |= 1<<PORTD5;
			// PORTD |= 0<<PORTD6;
			PORTD |= 1<<PORTD7;
			break;
		case MUX_7 :
			// PORTD |= 0<<PORTD5;
			PORTD |= 1<<PORTD6;
			PORTD |= 1<<PORTD7;
			break;
		case MUX_8 :
			PORTD |= 1<<PORTD5;
			PORTD |= 1<<PORTD6;
			PORTD |= 1<<PORTD7;
			break;
		default :
			// YOU HAVE MADE A GRAVE ERROR
			break;
	}
}

void IR_mux_set(MuxLine line)
{
	PORTD &= ~(1 << PORTD2);
	PORTD &= ~(1 << PORTD3);
	PORTD &= ~(1 << PORTD4);
	switch (line) {
		// Hopefully this monstrosity gets optimized away. (TODO: Nope, says a simple size test.)
		case MUX_1 :
			// PORTD |= 0<<PORTD2;
			// PORTD |= 0<<PORTD3;
			// PORTD |= 0<<PORTD4;
			break;
		case MUX_2 :
			PORTD |= 1<<PORTD2;
			// PORTD |= 0<<PORTD3;
			// PORTD |= 0<<PORTD4;
			break;
		case MUX_3 :
			// PORTD |= 0<<PORTD2;
			PORTD |= 1<<PORTD3;
			// PORTD |= 0<<PORTD4;
			break;
		case MUX_4 :
			PORTD |= 1<<PORTD2;
			PORTD |= 1<<PORTD3;
			// PORTD |= 0<<PORTD4;
			break;
		case MUX_5 :
			// PORTD |= 0<<PORTD2;
			// PORTD |= 0<<PORTD3;
			PORTD |= 1<<PORTD4;
			break;
		case MUX_6 :
			PORTD |= 1<<PORTD2;
			// PORTD |= 0<<PORTD3;
			PORTD |= 1<<PORTD4;
			break;
		case MUX_7 :
			// PORTD |= 0<<PORTD2;
			PORTD |= 1<<PORTD3;
			PORTD |= 1<<PORTD4;
			break;
		case MUX_8 :
			PORTD |= 1<<PORTD2;
			PORTD |= 1<<PORTD3;
			PORTD |= 1<<PORTD4;
			break;
		default :
			// YOU HAVE MADE A GRAVE ERROR
			break;
	}
}
