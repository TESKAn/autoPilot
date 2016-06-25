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
// Define send debug messages over USB
#define DEBUG_USB

//#define USE_FREEMASTER		// Use freemaster instead of own comm
//#define RS485_DEBUG				// Disable sending commands over RS485
//#define RS485_DISABLE_POLL	// Disable RS485 slave polling

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
#define PWMEN_OUT_ENABLE		flag0.bits.BIT28
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

// Clear flag1 FPU exceptions
#define CLEAR_FPU_EXCEPTIONS	flag1.flag.flag = flag1.flag.flag & 0xFFFFFFC3

// DMA macros
//DMA1_Stream0 used by USB
#define DMA_I2C2_RX				DMA1_Stream3
#define DMA_USART3				DMA1_Stream4
#define DMA_DAC1				DMA1_Stream5
#define DMA_USART2				DMA1_Stream6
#define DMA_I2C2_TX				DMA1_Stream7

#define DMA_USART1				DMA2_Stream7
#define DMA_USART1_CH			DMA_Channel_4

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
#define LED_ERR_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_8, 1)
#define LED_ERR_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_8, 0)

#define LED_OK_ON		GPIO_WriteBit(GPIOD, GPIO_Pin_9, 1)
#define LED_OK_OFF		GPIO_WriteBit(GPIOD, GPIO_Pin_9, 0)
#define LED_OK_TOGGLE	GPIO_ToggleBits(GPIOD, GPIO_Pin_9)

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
#define SCR2_ACCOK					0x0001
#define SCR2_GYROOK					0x0002
#define SCR2_MAGOK					0x0004
#define SCR2_BAROK					0x0008
#define SCR2_POWEROK				0x0010
#define SCR2_GPSOK					0x0020
#define SCR2_LOGOPEN				0x0040


//Timer 1 macros
#define TIM1_PERIOD		19999
#define TIM1_PRESCALER	167
#define TIM1_PULSE		1049

// Timer 2 macros
#define TIM2_PERIOD		2999//19999	// Make PWM faster - 3 ms.
#define TIM2_PRESCALER	83
#define TIM2_PULSE		1049

// Timer 3 macros
#define TIM3_PERIOD		19999
#define TIM3_PRESCALER	83
#define TIM3_PULSE		1049

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

// SPI macros
#define SPI_SDCARD_SELECT	GPIO_WriteBit(GPIOB, GPIO_Pin_12, 0)
#define SPI_SDCARD_RELEASE	GPIO_WriteBit(GPIOB, GPIO_Pin_12, 1)

// Debug macros
/*
#define DEBUG_PIN_ON		GPIO_WriteBit(GPIOE, GPIO_Pin_5, 1)
#define DEBUG_PIN_OFF		GPIO_WriteBit(GPIOE, GPIO_Pin_5, 0)
#define DEBUG_PIN_TOGGLE	GPIO_ToggleBits(GPIOE, GPIO_Pin_5)
*/

// RS485 dir macros
#define RS485_TXEN			GPIO_WriteBit(GPIOA, GPIO_Pin_5, 1)
#define RS485_RXEN			GPIO_WriteBit(GPIOA, GPIO_Pin_5, 0)

// PWM enable macro
#define PWMEN_PIN_TOGGLE	GPIO_ToggleBits(GPIOE, GPIO_Pin_15)

// SD card macros
// SD power ON
#define SD_POWER_ON			GPIO_WriteBit(GPIOA, GPIO_Pin_8, 0)
// SD power OFF
#define SD_POWER_OFF		GPIO_WriteBit(GPIOA, GPIO_Pin_8, 1)
// Toggle SD power
#define SD_POWER_TOGGLE		GPIO_ToggleBits(GPIOA, GPIO_Pin_8)

// Sensor power ON
#define SENSOR_POWER_ON		GPIO_WriteBit(GPIOE, GPIO_Pin_7, 0)
// Sensor power OFF
#define SENSOR_POWER_OFF	GPIO_WriteBit(GPIOE, GPIO_Pin_7, 1)

// Signal strength macros
// How much time between pulses to declare value to be DC
#define SIGNALSTRENGTH_MAXTIME	3000	// 3 seconds



// UART2 comm macros
// Declare variables
// Must be accessible to all files

#define VAR_VERSION						0
#define VAR_BUILD						1
#define VAR_GYRO_X						2
#define VAR_GYRO_Y						3
#define VAR_GYRO_Z						4
#define VAR_ACC_X						5
#define VAR_ACC_Y						6
#define VAR_ACC_Z						7
#define VAR_MAG_X						8
#define VAR_MAG_Y						9
#define VAR_MAG_Z						10
#define VAR_MAIN_LOOP_STATE				11
#define VAR_RS485_SERVO_POSITION		12
#define VAR_SERVOFR						13
#define VAR_SERVOFL						14
#define VAR_SERVOR						15
#define VAR_MOTOR_FR_RPM				16
#define VAR_RREADRS485DATA				17
#define VAR_RS485COMMAND				18
#define VAR_DCM_AX                      19
#define VAR_DCM_AY                      20
#define VAR_DCM_AZ                      21
#define VAR_DCM_BX                      22
#define VAR_DCM_BY                      23
#define VAR_DCM_BZ                      24
#define VAR_DCM_CX                      25
#define VAR_DCM_CY                      26
#define VAR_DCM_CZ                      27
#define VAR_PWMIN_1                     28
#define VAR_PWMIN_2                     29
#define VAR_PWMIN_3                     30
#define VAR_PWMIN_4                     31
#define VAR_PWMIN_5                     32
#define VAR_PWMIN_6                     33
#define VAR_PWMIN_7                     34
#define VAR_PWMIN_8                     35
#define VAR_PWMIN_1_ZERO                36
#define VAR_PWMIN_2_ZERO                37
#define VAR_PWMIN_3_ZERO                38
#define VAR_PWMIN_4_ZERO                39
#define VAR_PWMIN_5_ZERO                40
#define VAR_PWMIN_6_ZERO                41
#define VAR_PWMIN_7_ZERO                42
#define VAR_PWMIN_8_ZERO                43
#define VAR_PWMOUT_1                    44
#define VAR_PWMOUT_2                    45
#define VAR_PWMOUT_3                    46
#define VAR_PWMOUT_4                    47
#define VAR_PWMOUT_5                    48
#define VAR_PWMOUT_6                    49
#define VAR_PWMOUT_7                    50
#define VAR_PWMOUT_8                    51
#define VAR_PWMOUT_9                    52
#define VAR_PWMOUT_10                   53
#define VAR_PWMOUT_11                   54
#define VAR_PWMOUT_12                   55

#define VAR_MOTOR_FR_ARMED              56
#define VAR_MOTOR_FR_USEPWM             57
#define VAR_MOTOR_FR_REVERSE            58
#define VAR_MOTOR_FR_PARK               59
#define VAR_MOTOR_FR_PARKPOS            60
#define VAR_MOTOR_FR_PWMMIN             61
#define VAR_MOTOR_FR_PWMMAX             62
#define VAR_MOTOR_FR_CURRENTPWM         63
#define VAR_MOTOR_FR_SETRPM             64
#define VAR_MOTOR_FR_CURRENTRPM         65
#define VAR_MOTOR_FL_ARMED              66
#define VAR_MOTOR_FL_USEPWM             67
#define VAR_MOTOR_FL_REVERSE            68
#define VAR_MOTOR_FL_PARK               69
#define VAR_MOTOR_FL_PARKPOS            70
#define VAR_MOTOR_FL_PWMMIN             71
#define VAR_MOTOR_FL_PWMMAX             72
#define VAR_MOTOR_FL_CURRENTPWM         73
#define VAR_MOTOR_FL_SETRPM             74
#define VAR_MOTOR_FL_CURRENTRPM         75
#define VAR_MOTOR_R_ARMED               76
#define VAR_MOTOR_R_USEPWM              77
#define VAR_MOTOR_R_REVERSE             78
#define VAR_MOTOR_R_PARK                79
#define VAR_MOTOR_R_PARKPOS             80
#define VAR_MOTOR_R_PWMMIN              81
#define VAR_MOTOR_R_PWMMAX              82
#define VAR_MOTOR_R_CURRENTPWM          83
#define VAR_MOTOR_R_SETRPM              84
#define VAR_MOTOR_R_CURRENTRPM          85

#define VAR_SERVO_FR_TORQ_ON            86
#define VAR_SERVO_FR_GOAL_POS           87
#define VAR_SERVO_FR_POS                88
#define VAR_SERVO_FR_VOLTAGE            89
#define VAR_SERVO_FR_TEMPERATURE        90
#define VAR_SERVO_FL_TORQ_ON            91
#define VAR_SERVO_FL_GOAL_POS           92
#define VAR_SERVO_FL_POS                93
#define VAR_SERVO_FL_VOLTAGE            94
#define VAR_SERVO_FL_TEMPERATURE        95
#define VAR_SERVO_R_TORQ_ON             96
#define VAR_SERVO_R_GOAL_POS            97
#define VAR_SERVO_R_POS                 98
#define VAR_SERVO_R_VOLTAGE             99
#define VAR_SERVO_R_TEMPERATURE         100

#define VAR_UI32FLIGHTSTATEMACHINE      101
#define VAR_UI32FLIGHTINITSTATE         102
#define VAR_UI32TESTVAR                 103

#define VAR_MOTOR_FR_TEMPERATURE        104
#define VAR_MOTOR_FL_TEMPERATURE        105
#define VAR_MOTOR_R_TEMPERATURE         106

#define VAR_RS485_NO_RESPONSE_ID        107

#define VAR_UI32FLIGHTDEINITSTATE       108

#define VAR_UI16REQUESTEDPOSITION_FR    109
#define VAR_UI16REQUESTEDPOSITION_FL    110
#define VAR_UI16REQUESTEDPOSITION_R     111

#define VAR_F32NACELLETILT_FR           112
#define VAR_F32NACELLETILT_FL           113
#define VAR_F32NACELLETILT_R            114

#define VAR_PIDALTITUDE                 115
#define VAR_PIDPITCH                    116
#define VAR_PIDROLL                     117
#define VAR_PIDYAW                      118

#define VAR_ORIENTATIONALTITUDE         119
#define VAR_ORIENTATIONZEROALTITUDE     120
#define VAR_ORIENTATIONREQUIREDALTITUDE 121

#define VAR_PIDALTITUDEKP               122
#define VAR_PIDALTITUDEKI               123
#define VAR_PIDALTITUDEKD               124
#define VAR_PIDROLLKP                   125
#define VAR_PIDROLLKI                   126
#define VAR_PIDROLLKD                   127
#define VAR_PIDPITCHKP                  128
#define VAR_PIDPITCHKI                  129
#define VAR_PIDPITCHKD                  130
#define VAR_PIDYAWKP                    131
#define VAR_PIDYAWKI                    132
#define VAR_PIDYAWKD                    133

#define VAR_RC_AILERON_DIFF             134
#define VAR_RC_ELEVATOR_DIFF            135
#define VAR_RC_THROTTLE_DIFF            136
#define VAR_RC_RUDDER_DIFF              137

#define VAR_YAW_ACT                     138
#define VAR_YAW_REQ                     139
#define VAR_YAW_ERR                     140

#define VAR_MOTOR_FR_DCLINK             141
#define VAR_MOTOR_FL_DCLINK             142
#define VAR_MOTOR_R_DCLINK              143

#define VAR_MOTOR_FR_POSITION           144
#define VAR_MOTOR_FL_POSITION           145
#define VAR_MOTOR_R_POSITION            146
#define VAR_MOTOR_FR_SETPARKPOSITION    147
#define VAR_MOTOR_FL_SETPARKPOSITION    148
#define VAR_MOTOR_R_SETPARKPOSITION     149

#define VAR_THROTTLE_INPUT              150

#define VAR_RC_AILERON_MAX              151
#define VAR_RC_ELEVATOR_MAX             152
#define VAR_RC_THROTTLE_MAX             153
#define VAR_RC_RUDDER_MAX               154
#define VAR_RC_AILERON_MIN              155
#define VAR_RC_ELEVATOR_MIN             156
#define VAR_RC_THROTTLE_MIN             157
#define VAR_RC_RUDDER_MIN               158

#define VAR_ROLL_REQ                    159
#define VAR_PITCH_REQ                   160

#define VAR_PWM_FR                      161
#define VAR_PWM_FL                      162
#define VAR_PWM_R                       163

#define VAR_MOTOR_FR_IIN                164
#define VAR_MOTOR_FL_IIN                165
#define VAR_MOTOR_R_IIN                 166

#define VAR_PIDROLLMAX                  167
#define VAR_PIDPITCHMAX                 168
#define VAR_PIDYAWMAX                   169
#define VAR_PIDROLLMIN                  170
#define VAR_PIDPITCHMIN                 171
#define VAR_PIDYAWMIN                   172

#define VAR_PIDGYROX                    173
#define VAR_PIDGYROY                    174
#define VAR_PIDGYROZ                    175

#define VAR_GYROERRORX                  176
#define VAR_GYROERRORY                  177
#define VAR_GYROERRORZ                  178

#define VAR_PIDROLLIM                   179
#define VAR_PIDPITCHIM                  180
#define VAR_PIDYAWIM                    181
#define VAR_UI32SENSACQELAPSTIME        182

#define VAR_PIDROLLD                    183
#define VAR_PIDPITCHD                   184
#define VAR_PIDYAWD                     185

#define VAR_PIDROLLI                    186
#define VAR_PIDPITCHI                   187
#define VAR_PIDYAWI                     188
#define VAR_PIDROLLP                    189
#define VAR_PIDPITCHP                   190
#define VAR_PIDYAWP                     191

#define VAR_UI32SENSORUPDATEINTERVAL    192

#define VAR_BATMON_CV0                  193
#define VAR_BATMON_CV1                  194
#define VAR_BATMON_CV2                  195
#define VAR_BATMON_CV3                  196
#define VAR_BATMON_CV4                  197
#define VAR_BATMON_CV5                  198
#define VAR_BATMON_CV6                  199
#define VAR_BATMON_CV7                  200
#define VAR_BATMON_CV8                  201
#define VAR_BATMON_CV9                  202
#define VAR_BATMON_CV10                 203
#define VAR_BATMON_CV11                 204
#define VAR_BATMON_STACKVOLTAGE         205
#define VAR_BATMON_OUTPUTVOLTAGE        206
#define VAR_BATMON_OUTPUTCURRENT        207
#define VAR_BATMON_PCBTEMPERATURE       208
#define VAR_BATMON_MAHUSED              209



#endif /* MACRO_H_ */
