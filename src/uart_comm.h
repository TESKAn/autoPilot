/*
 * uart_comm.h
 *
 *  Created on: 27. nov. 2015
 *      Author: Jure
 */

#ifndef UART_COMM_H_
#define UART_COMM_H_

#define DATATYPE_UI8		1
#define DATATYPE_I8			2
#define DATATYPE_UI16		3
#define DATATYPE_I16		4
#define DATATYPE_UI32		5
#define DATATYPE_I32		6
#define DATATYPE_f			7




extern uint8_t UART2_Transferring;

void UART_Init();
void UART_Timeout();
int32_t UART_RcvData(uint8_t data);
int32_t UART_SendBuffer();
int32_t UART_BufCount();
int UART_CopyToTransmitBuf();
int32_t UART_QueueMessagei16(int16_t var, int16_t data);
int32_t UART_QueueMessagei32(int16_t var, int32_t data);
int32_t UART_QueueMessagef(int16_t var, float data);
int32_t UART_QueueMessageui16(int16_t var, uint16_t data);
int32_t UART_QueueMessageui32(int16_t var, uint32_t data);
int32_t UART_QueueMessagei8(int16_t var, int8_t data);
int32_t UART_QueueMessageui8(int16_t var, uint8_t data);

#endif /* UART_COMM_H_ */
