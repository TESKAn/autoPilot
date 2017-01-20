/*
 * can.h
 *
 *  Created on: 6. jan. 2017
 *      Author: jmoc
 */

#ifndef CAN_H_
#define CAN_H_

// Define macros
// Device IDs
#define CAN_SERVO_FR_ID		10
#define CAN_SERVO_FL_ID		11
#define CAN_SERVO_R_ID		12
#define CAN_SERVO_ALL_ID	13

// Message IDs
#define CAN_SET_SERVO_ENABLE
#define CAN_CLEAR_SERVO_ENABLE
#define CAN_SET_SERVO_ANGLE	1024

typedef struct tagCANSTRUCT
{
	int16_t ui16CANTxMsgBufStore;
	int16_t ui16CANTxMsgBufRead;
	CanTxMsg CANTxMsgBuf[32];

}__attribute__((aligned(4),packed)) CANSTRUCT;

CANSTRUCT CANData;

void InitCANLink();
void ProcessCANMessage(CanRxMsg *msg);
int16_t SendCANMessage(CanTxMsg *msg);

#endif /* CAN_H_ */
