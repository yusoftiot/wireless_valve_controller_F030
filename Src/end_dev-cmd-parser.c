/*
 * command-parser.c
 *
 *  Created on: Mar 21, 2018
 *      Author: yuri
 */

#include <end_dev-cmd-parser.h>
#include "version.h"
#include <string.h>


int cmd_extract(CMD_RF_Request* arRequest, uint8_t* arCmd) {
	int iReturnCode = 0;
	*arCmd = arRequest->Payload[0];
	return iReturnCode;
}

int cmd_exec(uint8_t* arCommand) {
	int iReturnCOde = CMD_OK;

	switch (arCommand[0]) {

    case CMD_VALVE_1_SET_ON:
		high_voltage_valve(1);
        charge_delay();
		high_voltage_valve(0);
		bridge_lever_A(1);
		delay(VALVE_HIGH_VOLTAGE_ENABLE_BRIDGE);
		bridge_lever_A(0);
		break;

	case CMD_VALVE_1_SET_OFF:
		high_voltage_valve(1);
        charge_delay();
		high_voltage_valve(0);
		bridge_lever_B(1);
		delay(VALVE_HIGH_VOLTAGE_ENABLE_BRIDGE);
		bridge_lever_B(0);
		break;

	case CMD_HV_ENABLE:
		high_voltage_valve(1);
		break;

	case CMD_HV_DISABLE:
		high_voltage_valve(0);
		break;

	default:
		break;

	}


	return iReturnCOde;
}

/**
 *
 * @param arRfPacket
 * @param out_ParsedRequest
 * @return
 */
int cmd_parse_request(uint8_t* arRfPacket, CMD_RF_Request** out_ParsedRequest) {
    int iRetCode = -1;
    int iCnt;
//    int test_val;

    // Find request signature
    for (iCnt = 0; iCnt < 16; iCnt++) {
        if (RF_PACKET_START_SIGNATURE == arRfPacket[iCnt] && RF_PACKET_START_SIGNATURE2 == arRfPacket[iCnt +1]) {
//        	test_val = (int)arRfPacket + iCnt;
        	*out_ParsedRequest = (CMD_RF_Request*)(arRfPacket + iCnt);
        	iRetCode = 0;
            break;
        }
    }

    // TODO Insert Checksum test

    return iRetCode;
}


int update_response(CMD_RF_Request* pResponseData) {
    int iReturnCode = 0;
    uint8_t iCmd = pResponseData->Payload[0];

    pResponseData->DevAddr = get_device_addr();
    pResponseData->PayloadSize = 32;
    pResponseData->TrMessageCnt = 35;
    pResponseData->Payload[0] = ':';
    if ('V' == iCmd) {
        memcpy(pResponseData->Payload +1, VERSION , strlen(VERSION));
    } else {
        pResponseData->Payload[1] = 'O';
        pResponseData->Payload[2] = 'k';
    }

    return iReturnCode;
}
