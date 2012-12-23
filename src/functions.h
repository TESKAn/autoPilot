/*
 * functions.h
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

void openLog(void);
void closeLog(void);
void write_toLog(void);
int storeNumber(uint16_t number, char* buffer, int offset);
int storeNegativeNumber(uint16_t number, char* buffer, int offset);
uint16_t strToNumber(char* file, char* str);
void loadSettings(void);
char charFromNumber(uint8_t number);
uint8_t numberFromChar(char c);
void NVIC_EnableInterrupts(FunctionalState newState);
void extPeripheralInit(void);
void Delaynus(vu32 nus);
void transferDMA_USART2(uint8_t *data, int length);
void transferDMA_USART3(uint8_t *data, int length);

#endif /* FUNCTIONS_H_ */
