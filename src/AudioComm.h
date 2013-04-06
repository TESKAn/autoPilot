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

#endif /* AUDIOCOMM_H_ */
