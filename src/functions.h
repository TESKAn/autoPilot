/*
 * functions.h
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */

#ifndef FUNCTIONS_H_
#define FUNCTIONS_H_

typedef struct
{
     uint8_t* buffer;
     uint8_t* buffer_end;
     uint8_t* data_start;
     uint8_t* data_end;
     volatile int32_t count;
     int32_t size;
 }__attribute__((aligned(4),packed))  RING_BUFFER;

 typedef struct
 {
      uint32_t* buffer;
      uint32_t* buffer_end;
      uint32_t* data_start;
      uint32_t* data_end;
      volatile int32_t count;
      int32_t size;
  }__attribute__((aligned(4),packed))  RING_BUFFER32;

  int16_t CheckRCInputTimeouts();
int16_t SendCommData();
int16_t SendCommData1();
void calibrateI2CSensors(void);
void Refresh485();
int16_t CheckMotor(RS485MOTOR* motor);
int16_t CheckServo(RS485SERVO * servo);
int16_t CheckBatmon(RS485BATMON * batmon);
void refreshPWMOutputs(void);
uint32_t getSystemTime(void);
float32_t getFTime(void);
ErrorStatus FS_Initialize(void);
void storeAHRSAngles(FUSION_CORE *data);
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
void extPeripheralInit(void);
void Delayms(uint32_t ms);
void Delaynus(vu32 nus);
void transferDMA_USART1(uint8_t *data, int length);
void transferDMA_USART2(uint8_t *data, int length);
void transferDMA_USART3(uint8_t *data, int length);
float32_t intToFloat(uint16_t whole, uint16_t frac);
void sendUSBMessage(char* message);

int16_t RB_full(RING_BUFFER* rb);
int16_t RB_Init(RING_BUFFER* rb, uint8_t *buf, int16_t size);
int16_t RB_push(RING_BUFFER* rb, uint8_t data);
uint8_t RB_pop(RING_BUFFER* rb);
int16_t RB_flush(RING_BUFFER* rb);

int16_t RB32_full(RING_BUFFER32* rb);
int16_t RB32_Init(RING_BUFFER32* rb, uint32_t *buf, int16_t size);
int16_t RB32_push(RING_BUFFER32* rb, uint32_t data);
uint32_t RB32_pop(RING_BUFFER32* rb);
int16_t RB32_flush(RING_BUFFER32* rb);

#endif /* FUNCTIONS_H_ */
