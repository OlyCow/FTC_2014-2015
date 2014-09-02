#ifndef SPI_CODES_H
#define SPI_CODES_H
#include <inttypes.h>

const uint8_t SPI_FILLER		= 0x00;

const uint8_t SPI_CLEAR_GYRO	= 0x01;
const uint8_t SPI_SET_IR_ON		= 0x02;
const uint8_t SPI_SET_IR_OFF	= 0x03;
const uint8_t SPI_RESET_MCU		= 0x04;

const uint8_t SPI_ERROR			= 0x10;
const uint8_t SPI_ACK			= 0x11;
const uint8_t SPI_ACK_OVER		= 0x12;
const uint8_t SPI_ACK_DATA_REQ	= 0x13;

const uint8_t SPI_REQ_ACK		= 0x20;
const uint8_t SPI_REQ_DEBUG_A	= 0x21;
const uint8_t SPI_REQ_DEBUG_B	= 0x22;

#endif // SPI_CODES_H
