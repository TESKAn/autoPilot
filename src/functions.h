/*
 * functions.h
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

void updateExportVars(void);
uint32_t getSystemTime(void);
float32_t getFTime(void);
ErrorStatus FS_Initialize(void);
void storeAHRSAngles(void);
void openLog(void);
void closeLog(void);
void write_toLog(void);
int storeNumber(uint16_t number, char* buffer, int offset);
int storeNegativeNumber(uint16_t number, char* buffer, int offset);
ErrorStatus int16ToStr(int16_t value, char* text, char* str);
ErrorStatus float32ToStr(float32_t value, char* text, char* str);
ErrorStatus strToFloat32(float32_t* result, char* file, char* str);
uint16_t strTouint16(char* file, char* str);
ErrorStatus loadSingleSetting(char* name, float32_t* storeLocation);
ErrorStatus storeRunningValues(void);
ErrorStatus storeSettings(void);
ErrorStatus loadSettings(void);
char charFromNumber(uint8_t number);
uint8_t numberFromChar(char c);
ErrorStatus mountSDCard(void);
ErrorStatus unmountSDCard(void);
void extPeripheralInit(void);
void Delayms(uint32_t ms);
void Delaynus(vu32 nus);
void transferDMA_USART2(uint8_t *data, int length);
void transferDMA_USART3(uint8_t *data, int length);
float32_t intToFloat(int whole, int frac);
void sendUSBMessage(char* message);

#endif /* FUNCTIONS_H_ */
