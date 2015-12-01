/*
 * uart_comm.h
 *
 *  Created on: 27. nov. 2015
 *      Author: Jure
 */

#ifndef UART_COMM_H_
#define UART_COMM_H_

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

#endif /* UART_COMM_H_ */
