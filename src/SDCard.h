/*
 * SDCard.h
 *
 *  Created on: 10. feb. 2019
 *      Author: Jure
 */

#ifndef SDCARD_H_
#define SDCARD_H_

ErrorStatus FS_Initialize(void);
void openLog(void);
void closeLog(void);
void write_toLog(void);
int storeNumber(uint16_t number, char* buffer, int offset);
int storeNegativeNumber(uint16_t number, char* buffer, int offset);
ErrorStatus int16ToStr(int16_t value, char* text, char* str);
ErrorStatus uint32ToStr(uint32_t value, char* text, char* str);
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

#endif /* SDCARD_H_ */
