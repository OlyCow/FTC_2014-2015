// copyright, license, all that fun stuff
#include <inttypes.h>
#include <math.h>
#include <avr/io.h>
#ifndef F_CPU
#define F_CPU 1000000UL
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

// SD CARD FATFS TESTING!!!
#include "../lib/FatFs/ff.h"
FATFS FatFs;	// work area for each volume
FIL Fil;		// file object for each open file

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

// Functions.
void initialize_io();
void initialize_adc();
void initialize_spi();
void initialize_pcint();

int main()
{
	//initialize_io();
	//initialize_adc();
	//initialize_spi();
	//initialize_pcint();
	
	uint8_t* eeprom_pointer = reinterpret_cast<uint8_t*>(0x00);
	uint8_t buffer = 0xFF;
	FRESULT result;
	unsigned int bw;
	//while (buffer == 0xFF) {
		//buffer = eeprom_read_byte(eeprom_pointer);
		//eeprom_pointer += 6;
	//}

	_delay_ms(100);
	
	result = f_mount(&FatFs, "", 0);
	switch (result) {
		case FR_OK :
			buffer = 0x91;
			break;
		case FR_INVALID_DRIVE :
			buffer = 0x6F;
			break;
		case FR_DISK_ERR :
			buffer = 0xF5;
			break;
		case FR_NOT_READY :
			buffer = 0x5E;
			break;
		case FR_NO_FILESYSTEM :
			buffer = 0x99;
			break;
		default :
			buffer = 0x85;
			break;
	}
	eeprom_write_byte(eeprom_pointer, buffer);
	eeprom_pointer++;

	result = f_open(&Fil, "hello.txt", FA_WRITE | FA_CREATE_ALWAYS);
	switch (result) {
		case FR_OK :
			buffer = 0xB4;
			break;
		case FR_DISK_ERR :
			buffer = 0x9D;
			break;
		case FR_INT_ERR :
			buffer = 0x38;
			break;
		case FR_NOT_READY :
			buffer = 0x51;
			break;
		case FR_NO_FILE :
			buffer = 0xE9;
			break;
		case FR_NO_PATH :
			buffer = 0xE8;
			break;
		case FR_INVALID_NAME :
			buffer = 0xAA;
			break;
		case FR_DENIED :
			buffer = 0x4A;
			break;
		case FR_EXIST :
			buffer = 0x2F;
			break;
		case FR_INVALID_OBJECT :
			buffer = 0x6E;
			break;
		case FR_WRITE_PROTECTED :
			buffer = 0x40;
			break;
		case FR_INVALID_DRIVE :
			buffer = 0x75;
			break;
		case FR_NOT_ENABLED :
			buffer = 0xC6;
			break;
		case FR_NO_FILESYSTEM :
			buffer = 0x13;
			break;
		case FR_TIMEOUT :
			buffer = 0x11;
			break;
		case FR_LOCKED :
			buffer = 0xC0;
			break;
		case FR_NOT_ENOUGH_CORE :
			buffer = 0xBB;
			break;
		case FR_TOO_MANY_OPEN_FILES :
			buffer = 0x58;
			break;
		default :
			buffer = 0x89;
			break;
	}
	eeprom_write_byte(eeprom_pointer, buffer);
	eeprom_pointer++;

	result = f_write(&Fil, "I AM A WIZARD\r\n", 15, &bw);
	switch (result) {
		case FR_OK :
			buffer = 0x45;
			break;
		case FR_DISK_ERR :
			buffer = 0x69;
			break;
		case FR_INT_ERR :
			buffer = 0x44;
			break;
		case FR_NOT_READY :
			buffer = 0xCB;
			break;
		case FR_INVALID_OBJECT :
			buffer = 0x1A;
			break;
		case FR_TIMEOUT :
			buffer = 0xBD;
			break;
		default :
			buffer = 0x6B;
			break;
	}
	eeprom_write_byte(eeprom_pointer, buffer);
	eeprom_pointer++;

	result = f_close(&Fil);
	switch (result) {
		case FR_OK :
			buffer = 0x2C;
			break;
		case FR_DISK_ERR :
			buffer = 0xA4;
			break;
		case FR_INT_ERR :
			buffer = 0x36;
			break;
		case FR_NOT_READY :
			buffer = 0xA5;
			break;
		case FR_INVALID_OBJECT :
			buffer = 0x9D;
			break;
		case FR_TIMEOUT :
			buffer = 0x5B;
			break;
		default :
			buffer = 0x60;
			break;
	}
	eeprom_write_byte(eeprom_pointer, buffer);
	eeprom_pointer++;

	while (true) {
		_delay_ms(10);
	}
}

void initialize_io()
{
	// Set up I/O port directions with the DDRx registers. 1=out, 0=in.
	// These can be changed later in the program (and some sensors need
	// to do this, e.g. ultrasonic sensors).
	// ADC6 reads from the temp mux, ADC7 reads from the IR mux.
	DDRB =
		1<<DDB0 | // LED_D_R
		1<<DDB1 | // LED_D_G
		0<<DDB2 | // SPI_SS'_B
		0<<DDB3 | // SPI_MOSI
		1<<DDB4 | // SPI_MISO
		0<<DDB5 | // SPI_SCLK
		1<<DDB6 | // LED_D_B
		1<<DDB7 ; // LED_C_B
	DDRC =
		1<<DDC0 | // LIGHT_A_SEL_A
		1<<DDC1 | // LIGHT_A_SEL_B
		1<<DDC2 | // LIGHT_A_SEL_C
		1<<DDC3 | // LIGHT_B_SEL_A
		1<<DDC4 | // LIGHT_B_SEL_B
		1<<DDC5 | // LIGHT_B_SEL_C
		0<<DDC6 ; // RESET - init to input
		// PORTC does NOT have a 7th bit.
	DDRD =
		1<<DDD0 | // LED_A_R
		1<<DDD1 | // LED_A_G
		1<<DDD2 | // LED_A_B
		1<<DDD3 | // LED_B_R
		1<<DDD4 | // LED_B_G
		1<<DDD5 | // LED_B_B
		1<<DDD6 | // LED_C_R
		1<<DDD7 ; // LED_C_B
		
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
		0<<PORTB7 ;
	PORTC =
		0<<PORTC0 |
		0<<PORTC1 |
		0<<PORTC2 |
		0<<PORTC3 |
		0<<PORTC4 |
		0<<PORTC5 |
		1<<PORTC6 ; // TODO: figure out if this is necessary.
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

void initialize_adc()
{
	// ADCL, ADCH, ADMUX, ADCSRA, and ADCSRB all default to 0.
	// Each conversion takes 13 ADC clock cycles, except for the initial one (takes 25).

	// set voltage reference: "AVCC with external capacitor at AREF pin"--datasheet
	ADMUX |= 1<<REFS0 | 0<<REFS1;

	// Left-adjust the ADC result (we don't need 10-bit accuracy).
	ADMUX |= 1<<ADLAR;

	// ADC prescaler = 32: if F_CPU is 8MHz, then ADC clock is 250kHz (above 50~200kHz, but it should be accurate enough)
	ADCSRA |= 1<<ADPS0 | 0<<ADPS1 | 1<<ADPS2;

	// enable ADC
	ADCSRA |= 1<<ADEN;
}


void initialize_spi()
{
	//SPCR |= 1<<SPIE; // Enable SPI interrupts
	SPCR |= 0<<DORD; // MSB transmitted first
	//SPCR |= 0<<MSTR; // slave mode
	SPCR |= 1<<MSTR; // master mode
	SPCR |= 0<<CPOL | 0<<CPHA; // SPI Mode 0; just needs to be consistent across all MCUs
	//SPCR |= 1<<SPE; // Enable SPI
	// SPR0, SPR1, and SPI2X have no effect on slave (only master), and all default to 0.
	
	SPDR = SPI_ACK_READY; // Means we're ready to receive data.
}

void initialize_pcint()
{
	// NOTE: Does not apply to this MCU (it's from MCU B); kept here for reference
	//PCICR |= 1<<PCIE0; // Enable PCINT0 interrupts
	//PCMSK0 |= 1<<PCINT2; // Unmask PCINT2 ('SS / PB2)
	//// PCICR and PCMSK0 both default to 0.
}
