/*
 * command-parser.h
 *
 *  Created on: Mar 21, 2018
 *      Author: yuri
 */

#ifndef COMMAND_PARSER_H_
#define COMMAND_PARSER_H_

#include "main.h"
#include <stdint.h>

#define RF_PACKET_START_SIGNATURE       0x55
#define RF_PACKET_START_SIGNATURE2      0x5A

#define VALVE_HIGH_VOLTAGE_START_TIME		4000
#define VALVE_HIGH_VOLTAGE_ENABLE_BRIDGE	200

typedef enum {
	CMD_UNDEFINED_COMMAND = -2,
	CMD_CRC_ERROR = -1,
	CMD_OK,
    CMD_GET_VERSION = 'V',
    CMD_VALVE_1_SET_ON = '1',
	CMD_VALVE_1_SET_OFF = '2',
	CMD_VALVE_1_GET_STATE = 's',
	CMD_HV_ENABLE,
	CMD_HV_DISABLE
} CMD_RESULT;

/**
 * RF Packet description
 * 0 - Packet signature 0x55
 * 1 - Packet signature 0xA5
 * 2..3 - Device address
 * 4 - Number of bytes in payload
 * 5 - Message Counter in transmitter
 * 6,7 - Checksum (Include header)- 2 byte
 * 8 .. 31 Payload
 */
typedef struct __attribute__((__packed__)) {
	uint16_t StartSign;     // 2 bytes x55, xA5
	uint16_t DevAddr;       // 2 bytes
	uint8_t  PayloadSize;   // 1 byte, size in bytes
    uint8_t  TrMessageCnt;  // 1 byte, Transmitter message counter
    uint16_t CheckSum;      // 2 bytes, Checksum 16 bit
    uint8_t  Payload[24];   // 24 bytes
} CMD_RF_Request;

int cmd_exec(uint8_t* arCommand);
int cmd_extract(CMD_RF_Request* arRequest, uint8_t* arCmd);
int cmd_parse_request(uint8_t* arRfPacket, CMD_RF_Request** out_ParsedRequest);
int update_response(CMD_RF_Request* pResponseData);

#endif /* COMMAND_PARSER_H_ */
