/*
 * sensors_spi.h
 *
 *  Created on: 10. apr. 2019
 *      Author: jmoc
 */

#ifndef SENSORS_SPI_H_
#define SENSORS_SPI_H_

#define SPI_DMA_WAITDEINIT		500
#define SPI_DMA_TIMEOUT_TIME	2000
#define SPI_ERRORTIMEOUT		2000
#define SPI_ERROR_RETRIESCOUNT	5		// How many times to retry communication


int16_t Sensor_SPIInit();
int16_t Sensor_SPICommProcess();
int16_t Sensor_setCS(uint8_t device, uint8_t state);
int16_t Sensor_SPIWrite(uint8_t device, uint8_t address, uint8_t data);
int16_t Sensor_SPIRead(uint8_t device, uint8_t address);
int16_t Sensor_SPIReadWord(uint8_t device, uint8_t address);
int16_t Sensor_SPIReadDMA();
int16_t Sensor_SPIInitAG();
int16_t Sensor_SPIInitM();
int16_t Sensor_SPIInitB();

#endif /* SENSORS_SPI_H_ */
