/*
 * macro.h
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#ifndef MACRO_H_
#define MACRO_H_

// Configuration macros
// Define GPIO pin 5 as debug output
#define GPIOE5_IS_DEBUG
// Define send debug messages over USB
#define DEBUG_USB

// File write buffer size
#define FATFS_BUFF_SIZE		256
// After how many writes flush buffer?
#define FATFS_FLUSH_COUNT	2
// Messages that fit in buffer - 19 * 102 bytes
#define SD_BUF_MESSAGE_LIMIT	1938

#define NUM 10

// Define bit macros
#define _BIT0	0x0001
#define _BIT1	0x0002
#define _BIT2	0x0004
#define _BIT3	0x0008
#define _BIT4	0x0010
#define _BIT5	0x0020
#define _BIT6	0x0040
#define _BIT7	0x0080
#define _BIT8	0x0100
#define _BIT9	0x0200
#define _BIT10	0x0400
#define _BIT11	0x0800
#define _BIT12	0x1000
#define _BIT13	0x2000
#define _BIT14	0x4000
#define _BIT15	0x8000

// Flag macros
#define COPYI2C					flag0.bits.BIT0
#define PSBUSY					flag0.bits.BIT1
#define PS_WAITINGDATA			flag0.bits.BIT2
#define I2C2_WAITINGDATA		flag0.bits.BIT3
#define I2C2_INITDONE			flag0.bits.BIT4
#define GPS_SENDING				flag0.bits.BIT5
#define I2C2_REENABLE			flag0.bits.BIT6
#define PWM_PASSTHROUGH			flag0.bits.BIT7
#define EXTSENS_INIT_DONE		flag0.bits.BIT8
#define SYSTEM_INTERRUPTS_ON	flag0.bits.BIT9
#define LED_BLINK				flag0.bits.BIT10
#define ADC_ENABLED				flag0.bits.BIT11
#define USB_REQUEST_DATA_0		flag0.bits.BIT12
#define USB_REQUEST_DATA_1		flag0.bits.BIT13
#define USB_REQUEST_DATA_2		flag0.bits.BIT14
#define SD_WRITE_LOG			flag0.bits.BIT15
#define SD_LOG_ISOPEN			flag0.bits.BIT16
#define EXTSENS_NULLING_GYRO	flag0.bits.BIT17
#define EXTSENS_NULLING_ACC		flag0.bits.BIT18
#define EXTSENS_NULLING_MAG		flag0.bits.BIT19
#define EXTSENS_FS_REPORT		flag0.bits.BIT20
#define SD_INITIALIZED			flag0.bits.BIT21
#define SD_WRITING_BUF1			flag0.bits.BIT22
#define SD_WRITING_BUF2			flag0.bits.BIT23
#define SD_BUF_IN_USE			flag0.bits.BIT24	// 0 - writing buf1, 1 - writing buf2


// DMA macros
//DMA1_Stream0 used by USB
#define DMA_I2C2_RX				DMA1_Stream3
#define DMA_USART3				DMA1_Stream4
#define DMA_DAC1				DMA1_Stream5
#define DMA_USART2				DMA1_Stream6
#define DMA_I2C2_TX				DMA1_Stream7

#define DMA_I2C2_RX_TCIF 		DMA_FLAG_TCIF3
#define DMA_I2C2_RX_HTIF 		DMA_FLAG_HTIF3
#define DMA_I2C2_RX_TEIF 		DMA_FLAG_TEIF3
#define DMA_I2C2_RX_DMEIF 		DMA_FLAG_DMEIF3
#define DMA_I2C2_RX_FEIF 		DMA_FLAG_FEIF3

#define DMA_I2C2_RX_IT_TCIF 	DMA_IT_TCIF7
#define DMA_I2C2_RX_IT_HTIF 	DMA_IT_HTIF7
#define DMA_I2C2_RX_IT_TEIF 	DMA_IT_TEIF7
#define DMA_I2C2_RX_IT_DMEIF 	DMA_IT_DMEIF7
#define DMA_I2C2_RX_IT_FEIF 	DMA_IT_FEIF7

#define DMA_I2C2_TX_TCIF 		DMA_FLAG_TCIF7
#define DMA_I2C2_TX_HTIF 		DMA_FLAG_HTIF7
#define DMA_I2C2_TX_TEIF 		DMA_FLAG_TEIF7
#define DMA_I2C2_TX_DMEIF 		DMA_FLAG_DMEIF7
#define DMA_I2C2_TX_FEIF 		DMA_FLAG_FEIF7

#define DMA_I2C2_TX_IT_TCIF 	DMA_IT_TCIF7
#define DMA_I2C2_TX_IT_HTIF 	DMA_IT_HTIF7
#define DMA_I2C2_TX_IT_TEIF 	DMA_IT_TEIF7
#define DMA_I2C2_TX_IT_DMEIF 	DMA_IT_DMEIF7
#define DMA_I2C2_TX_IT_FEIF 	DMA_IT_FEIF7



// DAC macros
#define DAC_DHR12R1_ADDRESS    0x40007408

// LED macros
#define LED_ERR_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_8, 1)
#define LED_ERR_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_8, 0)

#define LED_OK_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_9, 1)
#define LED_OK_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_9, 0)

#define LED_RUN_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_10, 1)
#define LED_RUN_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_10, 0)
#define LED_RUN_TOGGLE	GPIO_ToggleBits(GPIOD, GPIO_Pin_10)

// SCR1 macros
#define SCR_01				0x0001
#define SCR_02				0x0002
#define SCR_03				0x0004
#define SCR_04				0x0008
#define SCR_05				0x0010
#define SCR_06				0x0020
#define SCR_07				0x0040
#define SCR_08				0x0080
#define SCR_09				0x0100
#define SCR_10				0x0200
#define SCR_11				0x0400
#define SCR_12				0x0800
#define SCR_13				0x1000
#define SCR_14				0x2000
#define SCR_15				0x4000
#define SCR_16				0x8000

// SCR2 macros
#define SCR2_ACCOK			0x0001
#define SCR2_GYROOK			0x0002
#define SCR2_MAGOK			0x0004
#define SCR2_BAROK			0x0008
#define SCR2_POWEROK		0x0010
#define SCR2_GPSOK			0x0020
#define SCR2_LOGOPEN		0x0040

//Timer 1 macros
#define TIM1_PERIOD		32200
#define TIM1_PRESCALER	84
#define TIM1_PULSE		2100

// Timer 2 macros
#define TIM2_PERIOD		32200
#define TIM2_PRESCALER	42
#define TIM2_PULSE		2100

// Timer 3 macros
#define TIM3_PERIOD		32200
#define TIM3_PRESCALER	42
#define TIM3_PULSE		2100

// Timer 4 macros
#define TIM4_PRESCALER	42
#define TIM4_FILTER		1
#define TIM4_PERIOD		0xFFFF

// Timer 8 macros
#define TIM8_PRESCALER	84
#define TIM8_FILTER		1
#define TIM8_PERIOD		0xFFFF

// Timer 14 macros
#define TIM14_PERIOD		1000	//1 ms period = 1000
#define TIM14_PRESCALER		84		//divide ref clock by 84 to get 1 MHz

//USART2 macros
//define data register address - base address + DR offset
#define USART2_DR_ADDRESS	((uint32_t)0x40004404)

// SPI macros
#define SPI_SDCARD_SELECT	GPIO_WriteBit(GPIOB, GPIO_Pin_12, 0)
#define SPI_SDCARD_RELEASE	GPIO_WriteBit(GPIOB, GPIO_Pin_12, 1)

// Debug macros
#define DEBUG_PIN_ON		GPIO_WriteBit(GPIOE, GPIO_Pin_5, 1)
#define DEBUG_PIN_OFF		GPIO_WriteBit(GPIOE, GPIO_Pin_5, 0)
#define DEBUG_PIN_TOGGLE	GPIO_ToggleBits(GPIOE, GPIO_Pin_5)

#endif /* MACRO_H_ */
