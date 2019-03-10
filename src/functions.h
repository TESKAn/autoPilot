/*
 * functions.h
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_


int16_t CheckMainLoopStates();
int16_t CheckRCInputTimeouts();
int16_t SendCommData();
int16_t SendCommData1();
void calibrateI2CSensors(void);
int16_t checkMotorHealth();
void refreshMotorRPM();
void enableMotors(uint8_t ui8Enable);
int16_t storeMotorParams();
void refreshPWMOutputs(void);
uint32_t getSystemTime(void);
float32_t getFTime(void);
void extPeripheralInit(void);
void Delayms(uint32_t ms);
void Delaynus(vu32 nus);
void transferDMA_USART1(uint8_t *data, int length);
void transferDMA_USART2(uint8_t *data, int length);
void transferDMA_USART3(uint8_t *data, int length);
float32_t intToFloat(uint16_t whole, uint16_t frac);
void sendUSBMessage(char* message);
uint16_t Float32ToFloat16(float value);

#endif /* FUNCTIONS_H_ */
