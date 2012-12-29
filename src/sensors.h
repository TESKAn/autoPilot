/*
 * sensors.h
 *
 *  Created on: Oct 31, 2012
 *      Author: Jure
 */

#ifndef SENSORS_H_
#define SENSORS_H_

#define MPU6000_ADDRESS			0xD0
#define HMC5883_ADDRESS			0x3C
#define MPL3115A2_ADDRESS		0xC0
#define DMA_BUF_COUNT			128
#define I2C2_DMA_WAITDEINIT		500
#define I2C2_DMA_TIMEOUT_TIME	2000
#define I2C2_ERRORTIMEOUT		2000
#define I2C2_ERROR_RETRIESCOUNT	5		// How many times to retry communication
#define OFFSET_SAMPLE_COUNT		20		// How many samples to take for nulling
#define OFFSET_MAX_VALUE		400		// Maximum combined value of 20 samples
#define OFFSET_MIN_VALUE		-400	// Minimum combined value of 20 samples
#define OFFSET_SAMPLE_MAX_VALUE	200		// Maximum sample value to be valid
#define OFFSET_SAMPLE_MIN_VALUE	-200	// Minimum sample value to be valid

void sensorTimer(void);
void sensorInterruptTimer(void);
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
ErrorStatus I2C_DMACheckForError(DMA_Stream_TypeDef* DMAy_Streamx);
ErrorStatus I2C_CheckForError(I2C_TypeDef* I2Cx);
void I2C2_Configure(FunctionalState NewState);
void I2C2_ResetInterface(void);
void I2C2_DMA_ClearErrors(void);
void nullGyro(void);
void nullAcc(void);

extern volatile uint16_t I2C2_ProcesState;
extern volatile Flag I2C2_Flags;
extern volatile uint16_t I2C2_StartReg;
extern volatile uint16_t I2C2_ReadData;
extern volatile uint16_t I2C2_WriteData;
extern volatile uint8_t I2C2_DeviceAddress;
extern uint8_t I2C2_DMABufTX[DMA_BUF_COUNT];
extern uint8_t I2C2_DMABufRX[DMA_BUF_COUNT];
extern volatile int I2C2_DMABufTXCount;
extern volatile int I2C2_DMABufRXCount;
extern volatile int I2C2_PollTimer;
extern volatile uint16_t sensorTimeCounter;
extern volatile uint16_t sensoruTimeCounter;
// Data taken at time
extern uint32_t sensorAcquisitionTime;

// Offset registers
extern volatile int16_t gyroOffsets[3];
extern uint8_t gyroOffsetSampleCount;

extern volatile int16_t accOffsets[3];
extern uint8_t accOffsetSampleCount;

#define I2C2_POLLTIME		10

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
#define I2C2_ERROR_TIMEOUT	I2C2_Flags.bits.BIT1
#define I2C2_ERROR_PEC		I2C2_Flags.bits.BIT2
#define I2C2_ERROR_OVR		I2C2_Flags.bits.BIT3
#define I2C2_ERROR_AF		I2C2_Flags.bits.BIT4
#define I2C2_ERROR_ARLO		I2C2_Flags.bits.BIT5
#define I2C2_ERROR_BERR		I2C2_Flags.bits.BIT6
#define I2C2_DMA_TX_TXERR	I2C2_Flags.bits.BIT7
#define I2C2_DMA_TX_DMEIF	I2C2_Flags.bits.BIT8
#define I2C2_DMA_TX_FEIF	I2C2_Flags.bits.BIT9
#define I2C2_DMA_RX_TXERR	I2C2_Flags.bits.BIT10
#define I2C2_DMA_RX_DMEIF	I2C2_Flags.bits.BIT11
#define I2C2_DMA_RX_FEIF	I2C2_Flags.bits.BIT12
#define I2C2_TIMEOUT		I2C2_Flags.bits.BIT13
#define I2C2_DMA_TIMEOUT	I2C2_Flags.bits.BIT14

#endif /* SENSORS_H_ */
