/*
 * macro.h
 *
 *  Created on: 26. avg. 2012
 *      Author: Jure
 */

#ifndef MACRO_H_
#define MACRO_H_

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

// DMA macros
#define DMA_USART2		DMA1_Stream6
#define DMA_USART3		DMA1_Stream4
#define DMA_I2C2_TX		DMA1_Stream7
#define DMA_I2C2_RX		DMA1_Stream3

// LED macros
#define LED_ERR_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_8, 1)
#define LED_ERR_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_8, 0)

#define LED_OK_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_9, 1)
#define LED_OK_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_9, 0)

#define LED_RUN_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_10, 1)
#define LED_RUN_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_10, 0)
#define LED_RUN_TOGGLE	GPIO_ToggleBits(GPIOD, GPIO_Pin_10)

// SCR1 macros
#define SCR_GETPSDATA		0x0001
#define SCR_SETPSI0			0x0002
#define SCR_PSRESET			0x0004
#define SCR_INCPWM			0x0008
#define SCR_READI2C2		0x0010
#define SCR_WRITEI2C2		0x0020
#define SCR_TESTI2C2AUTO	0x0040
#define SCR_GETGPSDATA		0x0080
#define SCR_SET_PWM_0		0x0100
#define SCR_SET_PWM_PASSON	0x0200
#define SCR_SET_PWM_PASSOFF	0x0400
#define SCR_STOP_GPS		0x0800
#define SCR_START_AD		0x1000
#define SCR_INIT_SENSORS	0x2000

// SCR2 macros
#define SCR2_ACCOK			0x0001
#define SCR2_GYROOK			0x0002
#define SCR2_MAGOK			0x0004
#define SCR2_BAROK			0x0008
#define SCR2_POWEROK		0x0010
#define SCR2_GPSOK			0x0020

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

#endif /* MACRO_H_ */
