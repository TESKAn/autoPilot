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
//#define GPIOE5_IS_DEBUG

#define USE_FREEMASTER		// Use freemaster instead of own comm


// APStatus flags macros
#define LOG_ISOPEN	APStatus1.bits.BIT0

// File write buffer size
#define FATFS_BUFF_SIZE		256
// After how many writes flush buffer?
#define FATFS_FLUSH_COUNT	2
// Messages that fit in buffer - 19 * 102 bytes
#define SD_BUF_MESSAGE_LIMIT	1938

#define NUM 10

#define byte_swap32(val) asm("rev %[swap], %[swap]" : [swap] "=r" (val) : "0" (val));

#define byte_swap16(val) asm("rev16 %[swap], %[swap]" : [swap] "=r" (val) : "0" (val));

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
#define MPU_COMM_ENABLED		flag0.bits.BIT5
#define I2C2_REENABLE			flag0.bits.BIT6
#define PWM_PASSTHROUGH			flag0.bits.BIT7
#define EXTSENS_INIT_DONE		flag0.bits.BIT8
#define SYSTEM_INTERRUPTS_ON	flag0.bits.BIT9
#define LED_BLINK				flag0.bits.BIT10
#define ADC_ENABLED				flag0.bits.BIT11
#define SENSOR_TEST_STEP		flag0.bits.BIT12
#define SYSTEM_RUNNING			flag0.bits.BIT13
//#define SPI_TEST_RX				flag0.bits.BIT14

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
#define AHRS_FIRSTRUN_PID		flag0.bits.BIT25	// Mark to run once
#define AHRS_FIRSTRUN_MATRIX	flag0.bits.BIT26
#define TIMEOUT_USEINTERRUPT	flag0.bits.BIT27	// Use interrupt to count timeout
#define SPI3_WAITINGDATA		flag0.bits.BIT28
#define SD_SETTINGS_OPEN		flag0.bits.BIT29
#define SD_MOUNTED				flag0.bits.BIT30	// Mark SD card is mounted and ready
#define SENSORS_UPDATING		flag0.bits.BIT31	// Mark sensor data is being updated

// Flag 1 macros
#define CONSTANT_SERIAL_UPDATE	flag1.bits.BIT0		// Mark keep sending data over serial line
#define FPU_EXCEPTION			flag1.bits.BIT1		// Mark FPU exception has occured
#define FPU_EXC_UNDERFLOW		flag1.bits.BIT2
#define FPU_EXC_OVERFLOW		flag1.bits.BIT3
#define FPU_EXC_DIVZERO			flag1.bits.BIT4
#define FPU_EXC_INVALIDOP		flag1.bits.BIT5
#define COMM_SEND_DATA			flag1.bits.BIT6
#define SPI_INC_ADDRESS_READ	flag1.bits.BIT7
#define SPI_INC_ADDRESS_WRITE	flag1.bits.BIT8

// Clear flag1 FPU exceptions
#define CLEAR_FPU_EXCEPTIONS	flag1.flag.flag = flag1.flag.flag & 0xFFFFFFC3

// Read system time counter (32 bit)
#define READ_SYS_TIME			(uint32_t)(TIM2->CNT)

// DMA macros
//DMA1_Stream0 used by USB
#define DMA_I2C2_RX				DMA1_Stream3
#define DMA_USART3				DMA1_Stream4
#define DMA_DAC1				DMA1_Stream5
#define DMA_USART2				DMA1_Stream6
#define DMA_I2C2_TX				DMA1_Stream7

#define DMA_USART1				DMA2_Stream7
#define DMA_USART1_CH			DMA_Channel_4

#define DMA_SPI3_RX_STREAM		DMA1_Stream0
#define DMA_SPI3_RX_CHANNEL		DMA_Channel_0

#define DMA_SPI3_TX_STREAM		DMA1_Stream5
#define DMA_SPI3_TX_CHANNEL		DMA_Channel_0

#define DMA_I2C2_RX_TCIF 		DMA_FLAG_TCIF3
#define DMA_I2C2_RX_HTIF 		DMA_FLAG_HTIF3
#define DMA_I2C2_RX_TEIF 		DMA_FLAG_TEIF3
#define DMA_I2C2_RX_DMEIF 		DMA_FLAG_DMEIF3
#define DMA_I2C2_RX_FEIF 		DMA_FLAG_FEIF3

#define DMA_I2C2_RX_IT_TCIF 	DMA_IT_TCIF3
#define DMA_I2C2_RX_IT_HTIF 	DMA_IT_HTIF3
#define DMA_I2C2_RX_IT_TEIF 	DMA_IT_TEIF3
#define DMA_I2C2_RX_IT_DMEIF 	DMA_IT_DMEIF3
#define DMA_I2C2_RX_IT_FEIF 	DMA_IT_FEIF3

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
#define LED_ERR_ON		GPIO_WriteBit(GPIOE, GPIO_Pin_12, 1)
#define LED_ERR_OFF		GPIO_WriteBit(GPIOE, GPIO_Pin_12, 0)

#define LED_OK_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_11, 1)
#define LED_OK_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_11, 0)
#define LED_OK_TOGGLE	GPIO_ToggleBits(GPIOD, GPIO_Pin_11)

#define LED_RUN_ON		GPIO_WriteBit(GPIOE, GPIO_Pin_10, 1)
#define LED_RUN_OFF		GPIO_WriteBit(GPIOE, GPIO_Pin_10, 0)
#define LED_RUN_TOGGLE	GPIO_ToggleBits(GPIOE, GPIO_Pin_10)



//Timer 1 macros
#define TIM1_PERIOD		19999
#define TIM1_PRESCALER	167
#define TIM1_PULSE		1049

// Timer 2 macros
#define TIM2_PERIOD		0xffffffff	// Max. period for 32 bit timer
#define TIM2_PRESCALER	83

// Timer 3 macros
#define TIM3_PERIOD		0xFFFF
#define TIM3_PRESCALER	83
#define TIM3_PULSE		1049
#define TIM3_FILTER		1

// Timer 4 macros
#define TIM4_PRESCALER	83
#define TIM4_FILTER		1
#define TIM4_PERIOD		0xFFFF

// Timer 8 macros
#define TIM8_PRESCALER	167
#define TIM8_FILTER		1
#define TIM8_PERIOD		0xFFFF

// Timer 9 macros
// Set for 0,1 ms/pulse prescaler
#define TIM9_PRESCALER	16700
#define TIM9_FILTER		1
#define TIM9_PERIOD		0xFFFF

// Timer 14 macros
#define TIM14_PERIOD		999	//1 ms period = 1000
#define TIM14_PRESCALER		83		//divide ref clock by 84 to get 1 MHz

//USART2 macros
//define data register address - base address + DR offset
#define USART2_DR_ADDRESS	((uint32_t)0x40004404)




// SD card macros
// SD power ON
#define SD_POWER_ON			GPIO_WriteBit(GPIOA, GPIO_Pin_8, 0)
// SD power OFF
#define SD_POWER_OFF		GPIO_WriteBit(GPIOA, GPIO_Pin_8, 1)
// Toggle SD power
#define SD_POWER_TOGGLE		GPIO_ToggleBits(GPIOA, GPIO_Pin_8)

// Sensor power ON
#define SENSOR_POWER_ON		GPIO_WriteBit(GPIOC, GPIO_Pin_5, 0)
// Sensor power OFF
#define SENSOR_POWER_OFF	GPIO_WriteBit(GPIOC, GPIO_Pin_5, 1)

// Sensor macros
#define GYRO_DEV_CS			1
#define GYRO_START_REG		0x17
#define GYRO_BYTE_COUNT		10

#define ACC_DEV_CS			2
#define ACC_START_REG		0x27
#define ACC_BYTE_COUNT		8

#define MAG_DEV_CS			3
#define MAG_START_REG		0x67
#define MAG_BYTE_COUNT		8

#define BARO_DEV_CS			4
#define BARO_START_REG		0x65
#define BARO_BYTE_COUNT		10

#define PI_500								1570.7963267948966192313216916398f
#define GIRO_DIV							5898060.0f
#define GYRO_DEFAULT_RATE					0.01525925473799859614856410412915f//0.030517578125f			// 500/32768 -> deg/sec
#define GYRO_DEG_TO_RAD						0.017453292519f			// 1 deg/sec is this in rad/sec
#define GYRO_RAD_TO_DEG						57.295779513082f		// 1 rad/sec is this in deg/sec


//**************************************
//#define SPI_SENSOR_TESTING
//**************************************

// Sensor reg settings
// A_G
#define A_G_INT1_CTRL		0x02		// Gyro data ready on INT1 pin
#define A_G_INT2_CTRL		0x01		// Acc data ready on INT2 pin
#define A_G_CTRL_REG1_G		0x69		// ODR 119 Hz, 500 dps, BW 31 Hz
#define A_G_CTRL_REG2_G		0x00		// INT/OUTput after A/D
#define A_G_CTRL_REG3_G		0x47		// HP filter enable, cutoff 0.05 Hz
#define A_G_CTRL_REG4_G		0x3a		// Enable gyro outputs, latched interrupt
#define A_G_CTRL_REG5_XL	0x38		// Enable acc outputs
#define A_G_CTRL_REG6_XL	0x7b		// Acc ODR 119 Hz, +/- 8G, 50 Hz BW
#define A_G_CTRL_REG7_XL	0x00		// Acc
#define A_G_CTRL_REG8		0x44		// Reg address increment on data access, output not updated until readout, BLE set
#define A_G_CTRL_REG9		0x04		// SPI only
#define A_G_INT_GEN_CFG_G	0x40		// Latch gyro interrupt

// MAG
#define MAG_INT_CFG_M		0x00		// Int disabled on INT_M pin
#define MAG_CTRL_REG_1_M	0x7c		// High performance, 80 Hz output data rate
#define MAG_CTRL_REG_2_M	0x00		// +/- 4 gauss full-scale
#define MAG_CTRL_REG_3_M	0x80		// Disable I2C, no low power, SPI write, continuous conversion
#define MAG_CTRL_REG_4_M	0x0c		// High - performance mode on Z axis
#define MAG_CTRL_REG_5_M	0x00		// Block data update until readout

// BARO
#define BARO_CTRL_REG1		0xcc		// Active mode, interrupt enable, 25 Hz rate, block update until readout, 4 - wire interface
#define BARO_CTRL_REG2		0x08		// I2C interface disabled
#define BARO_CTRL_REG3		0x00		// Default
#define BARO_CTRL_REG4		0x01		// Data ready on interrupt pin

// Sensor SPI ring buffer
#define ACC_GET_DATA		1
#define GYRO_GET_DATA		2
#define MAG_GET_DATA		3
#define BARO_GET_DATA		4
#define SENSOR_WRITE_REG	5
#define SENSOR_READ_REG		6
#define SENSOR_READ_WORD	7
#define SENSOR_READ_T		8

// Sensor CS
// Baro
#define BARO_CS_0			GPIO_WriteBit(GPIOC, GPIO_Pin_4, 0)
#define BARO_CS_1			GPIO_WriteBit(GPIOC, GPIO_Pin_4, 1)

// Mag
#define MAG_CS_0			GPIO_WriteBit(GPIOE, GPIO_Pin_7, 0)
#define MAG_CS_1			GPIO_WriteBit(GPIOE, GPIO_Pin_7, 1)

// Gyro/acc
#define A_G_CS_0			GPIO_WriteBit(GPIOE, GPIO_Pin_8, 0)
#define A_G_CS_1			GPIO_WriteBit(GPIOE, GPIO_Pin_8, 1)

// Enable acc/gyro
#define A_G_ON				GPIO_WriteBit(GPIOC, GPIO_Pin_13, 1)
#define A_G_OFF				GPIO_WriteBit(GPIOC, GPIO_Pin_13, 0)

// USB power ON
#define USB_POWER_ON		GPIO_WriteBit(GPIOA, GPIO_Pin_15, 0)
// USB power OFF
#define USB_POWER_OFF		GPIO_WriteBit(GPIOA, GPIO_Pin_15, 1)




// Signal strength macros
// How much time between pulses to declare value to be DC
#define SIGNALSTRENGTH_MAXTIME	3000	// 3 seconds


// CAN
#define CAN_ID							10

// Define macros
// Device IDs
#define CAN_MOTOR_FR_ID			20
#define CAN_MOTOR_FL_ID			21
#define CAN_MOTOR_RR_ID			22
#define CAN_MOTOR_RL_ID			23

#define CAN_MOTOR_ALL_ID		26

// From main controller
// CAN message IDs
#define CAN_MID_STATUS					341
#define CAN_MID_ORIENTATION				20001
#define CAN_MID_SETRPMLIMIT				20002
#define CAN_MID_ORIENTATION_PID			20009

#define CAN_MID_SETRPM_BASE				20200		// Base ID for set RPM
#define CAN_MID_ENABLE_BASE				20300		// Base ID for ENABLE
#define CAN_MID_SET_REG_BASE			20400		// Set register MID


// CAN message priorities
#define CAN_PRIO_STATUS					0x15
#define CAN_PRIO_ORIENTATION			0x15
#define CAN_PRIO_SETRPMLIMIT			0x15
#define CAN_PRIO_SETRPM					0x14
#define CAN_PRIO_ENABLE					0x14
#define CAN_PRIO_SETRPM_FR				0x01
#define CAN_PRIO_SETRPM_FL				0x01
#define CAN_PRIO_SETRPM_RR				0x01
#define CAN_PRIO_SETRPM_RL				0x01
#define CAN_PRIO_ORIENTATION_PID		0x15
#define CAN_PRIO_RESET_ESC				0x15
#define CAN_PRIO_WRITE_REG				0x15

// From ESC
// CAN message IDs
#define CAN_MID_UIN						20101
#define CAN_MID_RPMINFO					20102
// CAN message priorities
#define CAN_PRIO_UIN					0x15
#define CAN_PRIO_RPMINFO				0x15

// Reg write macros
#define CAN_WRITE_TOFLASH				0
#define CAN_WRITE_REG_RESET				1
#define CAN_WRITE_REG_BATLOW			2
#define CAN_WRITE_REG_MINRPM			3
#define CAN_WRITE_REG_MAXRPM			4


// Mavlink


#endif /* MACRO_H_ */
