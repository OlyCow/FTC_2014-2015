#ifndef SPI_CODES_H
#define SPI_CODES_H
#include <inttypes.h>

const uint8_t SPI_FILLER		= 0x00;
const uint8_t SPI_ERROR			= 0x01;
const uint8_t SPI_REQ_CONFIRM	= 0x02;
const uint8_t SPI_ACK_READY		= 0x03;
const uint8_t SPI_ACK_DATA_REQ	= 0x04;

// MCU_B control codes.
const uint8_t SPI_RESET_MCU		= 0x10;
const uint8_t SPI_CONFIRM_OPERATION = 0x11;
const uint8_t SPI_CHANGE_LED	= 0x12; // next byte is select + mode
const uint8_t SPI_CLEAR_GYRO	= 0x13;
const uint8_t SPI_SET_IR_ON		= 0x14;
const uint8_t SPI_SET_IR_OFF	= 0x15;

// MCU_B data register access codes.
const uint8_t SPI_REQ_ACK		= 0x50;
const uint8_t SPI_REQ_DEBUG_A	= 0x51;
const uint8_t SPI_REQ_DEBUG_B	= 0x52;
const uint8_t SPI_REQ_XY_LOW	= 0x53;
const uint8_t SPI_REQ_XY_HIGH	= 0x54;
const uint8_t SPI_REQ_Z_LOW		= 0x55;
const uint8_t SPI_REQ_Z_HIGH	= 0x56;
const uint8_t SPI_REQ_BUMP_MAP	= 0x57;
const uint8_t SPI_REQ_OVERHEAT_ALERT = 0x58;
const uint8_t SPI_REQ_OVERHEAT_MAP = 0x59;
// Codes 0x5A through 0x5F are still usable here.
const uint8_t SPI_REQ_IR_ALERT	= 0x60;
const uint8_t SPI_REQ_IR_1		= 0x61;
const uint8_t SPI_REQ_IR_2		= 0x62;
const uint8_t SPI_REQ_IR_3		= 0x63;
const uint8_t SPI_REQ_IR_4		= 0x64;
const uint8_t SPI_REQ_IR_5		= 0x65;
const uint8_t SPI_REQ_IR_6		= 0x66;
const uint8_t SPI_REQ_IR_7		= 0x67;
const uint8_t SPI_REQ_IR_8		= 0x68;

// MCU_C control codes.

// MCU_C data register access codes.
const uint8_t SPI_REQ_LIGHT_MAP_A	= 0x50;
const uint8_t SPI_REQ_LIGHT_A_1		= 0x51;
const uint8_t SPI_REQ_LIGHT_A_2		= 0x52;
const uint8_t SPI_REQ_LIGHT_A_3		= 0x53;
const uint8_t SPI_REQ_LIGHT_A_4		= 0x54;
const uint8_t SPI_REQ_LIGHT_A_5		= 0x55;
const uint8_t SPI_REQ_LIGHT_A_6		= 0x56;
const uint8_t SPI_REQ_LIGHT_A_7		= 0x57;
const uint8_t SPI_REQ_LIGHT_A_8		= 0x58;
// Codes 0x59 through 0x5F are still usable here.
const uint8_t SPI_REQ_LIGHT_MAP_B	= 0x60;
const uint8_t SPI_REQ_LIGHT_B_1		= 0x61;
const uint8_t SPI_REQ_LIGHT_B_2		= 0x62;
const uint8_t SPI_REQ_LIGHT_B_3		= 0x63;
const uint8_t SPI_REQ_LIGHT_B_4		= 0x64;
const uint8_t SPI_REQ_LIGHT_B_5		= 0x65;
const uint8_t SPI_REQ_LIGHT_B_6		= 0x66;
const uint8_t SPI_REQ_LIGHT_B_7		= 0x67;
const uint8_t SPI_REQ_LIGHT_B_8		= 0x68;

#endif // SPI_CODES_H
