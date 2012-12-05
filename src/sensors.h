/*
 * sensors.h
 *
 *  Created on: Oct 31, 2012
 *      Author: Jure
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#define MPU6000_ADDRESS		0xD0
#define HMC5883_ADDRESS		0x3C
#define MPL3115A2_ADDRESS	0xC0
#define DMA_BUF_COUNT		128

void sensorInit();
void copySensorData(void);
ErrorStatus MPU6000_Enable(FunctionalState newState);
ErrorStatus MPU6000_EnableI2CBypass(FunctionalState newState);
ErrorStatus MPU6000_ConfigureI2CMaster(void);
ErrorStatus MPU6000_EnableI2CMaster(FunctionalState newState);
ErrorStatus HMC5883_Enable(FunctionalState newState);
ErrorStatus MPL3115A2_Enable(FunctionalState newState);
ErrorStatus masterSend(uint8_t device, uint8_t *dataBuffer, uint8_t byteCount);
ErrorStatus masterReceive_beginDMA(uint8_t device, uint8_t startReg, uint8_t *dataBuffer, uint8_t byteCount);
ErrorStatus masterReceive(uint8_t device,uint8_t startReg, uint8_t *dataBuffer, uint8_t byteCount);
ErrorStatus masterReceive_HMC5883L(uint8_t device, uint8_t startReg, uint8_t *dataBuffer, uint8_t byteCount);

extern uint16_t I2C2_ProcesState;
extern Flag I2C2_Flags;
extern uint16_t I2C2_StartReg;
extern uint16_t I2C2_ReadData;
extern uint16_t I2C2_WriteData;
extern uint8_t I2C2_DeviceAddress;
extern uint8_t I2C2_DMABufTX[DMA_BUF_COUNT];
extern uint8_t I2C2_DMABufRX[DMA_BUF_COUNT];
extern int I2C2_DMABufTXCount;
extern int I2C2_DMABufRXCount;
extern int I2C2_PollTimer;

#define I2C2_POLLTIME		100

// I2C2 process states
#define I2C2_IDLE			0
#define I2C2_STARTSENT		1
#define I2C2_ADDRESSENT		2
#define I2C2_REGADDRESSENT	3
#define I2C2_RSTARTSENT		4
#define I2C2_RADDRESSENT	5
#define I2C2_BYTE1READ		6
#define I2C2_BYTE2READ		7
#define I2C2_STOPSENT		8
#define I2C2_DATASENT		9


// I2C2 flags
#define I2C2_READING		I2C2_Flags.bits.BIT0

#endif /* SENSORS_H_ */
