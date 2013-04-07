/*
 * AudioComm.h
 *
 *  Created on: Apr 5, 2013
 *      Author: Jure
 */

#ifndef AUDIOCOMM_H_
#define AUDIOCOMM_H_

// Buffer A and B
extern uint8_t ACBufferA[220];
extern uint8_t ACBufferB[220];
// Buffer for message to be transmitted
extern uint8_t ACMessage[100];
// Index in message buffer
extern uint8_t ACMessageIndex;
// Message length
extern uint8_t ACMessageLength;
// Flags for AC
extern volatile Flag ACflag;
// Variable for running disparity. At start RD is -1
extern volatile int8_t AC_RDisparity;
// Variable to store next transition of bus level
extern uint8_t AC_NextTransition;

// Typedef to define
#define TRANSITION_LOW		0
#define TRANSITION_HIGH		100
// Buffer length
#define AC_BUFFER_LENGTH	220

// AC flag definitions
#define AC_SENDING_MESSAGE		ACflag.bits.BIT0
#define AC_SET_BUFFER_A			ACflag.bits.BIT1
#define AC_SET_BUFFER_B			ACflag.bits.BIT2
#define AC_MESSAGE_IN_BUFFER	ACflag.bits.BIT3
#define AC_SENDING_BUFFER_A		ACflag.bits.BIT4
#define AC_SENDING_BUFFER_B		ACflag.bits.BIT5
#define AC_BUFFER_A_COMMAS		ACflag.bits.BIT6
#define AC_BUFFER_B_COMMAS		ACflag.bits.BIT7

#endif /* AUDIOCOMM_H_ */
