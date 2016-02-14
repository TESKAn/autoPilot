/*
 * RS485comm.h
 *
 *  Created on: 10. jun. 2015
 *      Author: Jure
 */

#ifndef RS485COMM_H_
#define RS485COMM_H_

/*
 * RS485comm.h
 *
 *  Created on: May 20, 2015
 *      Author: jmoc
 */
// Typedefs
typedef uint32_t UInt32;
typedef uint16_t UInt16;
typedef uint8_t UInt8;
typedef int16_t Int16;
typedef int8_t Int8;

// This unit ID
#define RS485_ID		0x03

// Master mode macros
// Define number of slave devices
#define RS485_NUMSLAVES		3
// Slave timeout in ms
#define RS485_SLAVE_TIMEOUT		1000

// Command store struct
typedef struct tagRS485COMMAND
{
	union
	{
		UInt32 ui32Packed;
		struct tagVARS
		{
			UInt8 ui8Address;
			UInt8 ui8Command;
			UInt16 ui16Data;
		}VARS;
	};
}__attribute__((aligned(4),packed)) RS485COMMAND;

// Hardware dependent macros
#define RS485_ENABLE_RX						RS485_RXEN
#define RS485_ENABLE_TX						RS485_TXEN
#define RS485_ENABLE_TX_INT					//ioctl(SCI_0, SCI_TX_EMPTY_INT, SCI_ENABLE)
#define RS485_DISABLE_TX_INT				//ioctl(SCI_0, SCI_TX_EMPTY_INT, SCI_DISABLE)
#define RS485_ENABLE_TX_IDLE_INT			//ioctl(SCI_0, SCI_TX_IDLE_INT, SCI_ENABLE)
#define RS485_DISABLE_TX_IDLE_INT			//ioctl(SCI_0, SCI_TX_IDLE_INT, SCI_DISABLE)
#define RS485_WRITE(X)						//ioctl(SCI_0, SCI_WRITE_DATA, X)
#define RS485_READ							//ioctl(SCI_0, SCI_READ_DATA, NULL)
#define RS485_TEST_TX_EMPTY					1//ioctl(SCI_0, SCI_GET_TX_EMPTY, NULL)
#define RS485_TEST_TX_IDLE					1//ioctl(SCI_0, SCI_GET_TX_IDLE, NULL)
#define RS485_TEST_RX_FULL					1//ioctl(SCI_0, SCI_GET_RX_FULL, NULL)

// Define commands that we can queue
#define RS485_POLL_SERVO					1
#define RS485_POLL_MOTOR					2
#define RS485_SERVO_TORQ_ON					3
#define RS485_SERVO_TORQ_OFF				4
#define RS485_SET_SERVO_POSITION			5
#define RS485_SET_MOTOR_SPEED				6
#define RS485_SET_MOTOR_MIN_SPEED			7
#define RS485_SET_MOTOR_MAX_SPEED			8
#define RS485_ENABLE_MOTOR_PWMIN			9
#define RS485_SET_SERVO_ID					10
#define RS485_SET_SERVO_ANGLEMIN_LIM		11
#define RS485_SET_SERVO_ANGLEMAX_LIM		12
#define RS485_SET_SERVO_SPEED				13
#define RS485_SET_MOTOR_POSITION			14
#define RS485_SET_MOTOR_PARK				15
#define RS485_SET_MOTOR_RUN					16
#define RS485_SET_MOTOR_RPM					17
#define RS485_WRITE_MOTOR_ARMED_REG			18
#define RS485_WRITE_MOTOR_PARK_REG			19
#define RS485_WRITE_SERVO_TORQ_ENABLE		20
#define RS485_WRITE_MOTOR_REVERSE_REG		21
#define RS485_WRITE_MOTOR_MEAS_PWMLOW		22
#define RS485_WRITE_MOTOR_MEAS_PWMHIGH		23
#define RS485_WRITE_MOTOR_PWM_ZERO_SPEED	24
#define RS485_WRITE_MOTOR_USE_PWM			25
#define RS485_WRITE_MOTOR_REVERSE_ROTATION	26

// Commands macros
#define RS485_COMMAND_NONE					0x00
#define RS485_COMMAND_PING					0x01
#define RS485_COMMAND_READ					0x02
#define RS485_COMMAND_WRITE					0x03
#define RS485_COMMAND_REG_WRITE				0x04
#define RS485_COMMAND_GO					0x05
#define RS485_COMMAND_RESET_TOFACTORY		0x06
#define RS485_COMMAND_REBOOT				0x08
#define RS485_COMMAND_REQSTATUS				0x55
#define RS485_COMMAND_SYNC_READ				0x82
#define RS485_COMMAND_SYNC_WRITE			0x83
#define RS485_COMMAND_BULK_READ				0x92
#define RS485_COMMAND_BULK_WRITE			0x93

// Master defines
// Master state machine
#define RS485_M_STATE_IDLE					0
#define RS485_M_STATE_POLL					1
#define RS485_M_STATE_WAITING_RESPONSE		2
#define RS485_M_STATE_DELAY					3

// Master poll slaves
#define RS485_POLL_STATE_SERVO_FR			0
#define RS485_POLL_STATE_SERVO_FL			1
#define RS485_POLL_STATE_SERVO_R			2
#define RS485_POLL_STATE_MOTOR_FR			3
#define RS485_POLL_STATE_MOTOR_FL			4
#define RS485_POLL_STATE_MOTOR_R			5

// Transmitter states
#define RS485_M_TX_IDLE				0
#define RS485_M_TX_SENDING			1
#define RS485_M_TX_FINISHED			2

// Receiver states
#define RS485_RX_IDLE					0
#define RS485_RX_WAIT_FOR_SIGNAL		1
#define RS485_RX_WAIT_FOR_ID			2
#define RS485_RX_WAIT_FOR_LENGTH		3
#define RS485_RX_WAIT_FOR_INSTR_ERR		4
#define RS485_RX_WAIT_FOR_PARAMETERS	5
#define RS485_RX_WAIT_FOR_CHECKSUM		6

// Structure that holds all relevant data for servo
typedef struct tagRS485SERVO
{
	UInt8 errStatus;
	UInt8 ui8FreshData;
	UInt8 ui8Empty1;
	UInt16 ui16MoveToPos;
	UInt16 ui16Empty1;
	union
	{
		struct
		{
			UInt8 ui8REGSData[74];				// Main data structure
		};
		struct
		{
			UInt16 ui16ModelNumber;			// 0
			UInt8 ui8FirmwareVersion;		// 2
			UInt8 ui8ID;					// 3
			UInt8 ui8BaudRate;				// 4
			UInt8 ui8ReturnDelayTime;		// 5
			UInt16 CWAngleLimit;			// 6
			UInt16 CCWAngleLimit;			// 8
			UInt8 ui8Empty1;				// 10
			UInt8 ui8InternalTempLimit;		// 11
			UInt8 ui8LowLimitVoltage; 		// 12
			UInt8 ui8HighLimitVoltage;		// 13
			UInt16 ui16MaxTorque;			// 14
			UInt8 ui8StatusReturnLevel;		// 16
			UInt8 ui8AlarmLED;				// 17
			UInt8 ui8AlarmShutdown;			// 18
			UInt8 ui8Empty2;				// 19
			UInt16 ui16MultiTurnOffset;		// 20
			UInt8 ui8ResolutionDivider;		// 22
			UInt8 ui8Empty3;				// 23
			UInt8 ui8TorqueEnabled;			// 24
			UInt8 ui8LEDONOFF;				// 25
			UInt8 ui8DGain;					// 26
			UInt8 ui8IGain;					// 27
			UInt8 ui8PGain;					// 28
			UInt8 ui8Empty4;				// 29
			UInt16 ui16GoalPosition;		// 30
			UInt16 ui16MovingSpeed;			// 32
			UInt16 ui16TorqueLimit;			// 34
			UInt16 ui16PresentPosition;		// 36
			UInt16 ui16PresentSpeed;		// 38
			UInt16 ui16PresentLoad;			// 40
			UInt8 ui8PresentVoltage;		// 42
			UInt8 ui8PresentTemperature;	// 43
			UInt8 ui8Registered;			// 44
			UInt8 ui8Empty5;				// 45
			UInt8 ui8Moving;				// 46
			UInt8 ui8Lock;					// 47
			UInt16 ui16Punch;				// 48
			UInt8 ui8Empty6[23];			// 49 - 72
			UInt8 ui8GoalAcceleration;		// 73
		}REGS;
	};

}__attribute__((aligned(4),packed)) RS485SERVO;

// Structure that holds all relevant data for motor
typedef struct tagRS485MOTOR
{
	UInt8 errStatus;
	UInt8 ui8FreshData;
	union
	{
		UInt8 ui8REGSData[64];				// Main data structure
		struct
		{
			// Some params
			// Errors
			UInt16 ui16Errors;			// 0
			UInt16 ui16ModelNumber;		// 2
			UInt8 ui8FirmwareVersion;	// 4
			UInt8 ui8ID;				// 5
			UInt8 ui8BaudRate;			// 6
			UInt8 ui8Empty;				// 7


			// Motor state - idle, run, error
			UInt16 ui16State;			// 8

			// Status of the motor
			Int16 i16UIn;				// 10
			Int16 i16IIn;				// 12
			Int16 i16PIn;				// 14
			Int16 i16RPM;				// 16

			Int16 i16Empty;
			// Future expansion
			UInt8 uiEmpty1[12];			// 32 bytes total

			// Motor control
			// Arm
			UInt8 ui8Armed;				// 32
			// Park
			UInt8 ui8Park;				// 33
			// Reverse rotation
			UInt8 ui8ReverseRotation;	// 34
			UInt8 uiEmpty2;
			// Park position
			Int16 i16ParkPosition;		// 36

			Int16 i16SetRPM;			// 38
			Int16 i16MaxRPM;			// 40
			Int16 i16MinRPM;			// 42

			// PWM input
			UInt8 ui8MeasurePWMMin;		// 44
			UInt8 ui8MeasurePWMMax;		// 45
			UInt8 ui8UsePWMIN;			// 46
			UInt8 uiEmpty3;				// 47
			Int16 i16PWMMin;			// 48
			Int16 i16PWMMax;			// 50
			Int16 i16ZeroSpeedPWM;		// 52
			Int16 i16CurrentPWM;		// 54	//55 bytes total

			UInt8 uiEmpty4[8];					// 64 bytes total

		}REGS;
	};

}__attribute__((aligned(4),packed)) RS485MOTOR;

typedef struct tagRS485RXDATA
{
	UInt8 ui8RXState;

	UInt8 ui8RXCounter;			// Count bytes for RX

	// Timeouts
	UInt16 ui16RXTimeoutCounter;
	UInt16 ui16RXCommTimeout;

	struct
	{
		UInt8 ui8Parameters[128];
		UInt8 ui8RS485RXIndex;			// Index of next place to write to
		UInt8 ui8ParamByteCount;		// How many bytes are in parameters
		struct
		{
			union
			{
				UInt32 ui32Header;
				UInt8 ui8Bytes[4];
			}HEADER;
			UInt8 ui8ID;
			union
			{
				UInt16 ui16PacketLength;
				UInt8 ui8Bytes[2];
			}LENGTH;
			UInt8 ui8Instruction;

			union
			{
				UInt16 ui16CRC;
				UInt8 ui8Bytes[2];
			}CRCDATA;
		};
	}RXDATA;
}__attribute__((aligned(4),packed)) RS485RXDATA;

typedef struct tagRS485WAITINGUNIT
{
	UInt8 ui8ID;
	UInt8 ui8Instruction;
	UInt8 ui8Empty[2];
	UInt16 ui16RegAddress;
	UInt16 ui16ByteCount;

}__attribute__((aligned(4),packed)) RS485WAITINGUNIT;

// Defs for registers
#define MOTORREG_SETRPM				38
#define MOTORREG_ARMED				32
#define MOTORREG_PARK				33
#define MOTORREG_REVERSE			34
#define MOTORREG_PARKPOSITION		36
#define MOTORREG_MIN_SPEED			42
#define MOTORREG_MAX_SPEED			40
#define MOTORREG_ENABLE_PWMIN		46
#define MOTORREG_MEAS_MIN_PWM		44
#define MOTORREG_MEAS_MAX_PWM		45
#define MOTORREG_ZERO_SPEED_PWM		52
#define MOTORREG_
#define MOTORREG_
#define MOTORREG_
#define MOTORREG_

#define SERVOREG_ENABLE_TORQUE		24
#define SERVOREG_POSITION			30
#define SERVOREG_SPEED				32
#define SERVOREG_PGAIN				28
#define SERVOREG_IGAIN				27
#define SERVOREG_DGAIN				26
#define SERVOREG_MIN_ANGLE_REG		6
#define SERVOREG_MAX_ANGLE_REG		8
#define SERVOREG_
#define SERVOREG_
#define SERVOREG_
#define SERVOREG_
#define SERVOREG_
#define SERVOREG_


// Function declarations
Int16 RS485_Timing();
Int16 RS485_WriteServoPosition(UInt8 ID, UInt16 position);
Int16 RS485_WriteServoTorqueEnable(UInt8 ID, UInt16 enable);
Int16 RS485_WriteMotorEnable(UInt8 ID, UInt16 enable);
Int16 RS485_WriteMotorPark(UInt8 ID, UInt16 enable);
Int16 RS485_WriteMotorSpeed(UInt8 ID, UInt16 speed);
Int16 RS485_WriteMotorMeasPWMMin(UInt8 ID, UInt16 enable);
Int16 RS485_WriteMotorMeasPWMMax(UInt8 ID, UInt16 enable);
Int16 RS485_WriteMotorZeroPWM(UInt8 ID, UInt16 value);
Int16 RS485_WriteMotorUsePWM(UInt8 ID, UInt16 enable);
Int16 RS485_WriteMotorReverseRotation(UInt8 ID, UInt16 enable);
Int16 RS485_MasterInitData(void);
Int16 RS485_QueueCommand(RS485COMMAND cmdToExec);
Int16 RS485_MasterState(int state);
Int16 RS485_MotorTest(UInt8 func);
Int16 RS485_ServoTest(UInt8 servoID);
UInt16 RS485_Write8(UInt8 ID, UInt16 address, UInt8 data);
UInt16 RS485_Write16(UInt8 ID, UInt16 address, UInt16 data);
UInt16 RS485_Writefloat(UInt8 ID, UInt16 address, float data);
UInt16 RS485_Read(UInt8 ID, UInt16 address, UInt16 count);
UInt16 RS485_BufferQueuedCommand(RS485COMMAND command);
Int16 RS485_MasterWriteByte(uint8_t *data, int length);
void RS485_States_Master();
void RS485_ReceiveMessage(UInt8 data);
Int16 RS485_DecodeMessage();
Int16 RS485_SetupServos();
Int16 RS485_SetupMotors();
UInt16 update_crc(UInt16 crc_accum, UInt8 *data_blk_ptr, UInt16 data_blk_size);

// Extern's
extern RS485SERVO RS485Servo_FL;
extern RS485SERVO RS485Servo_FR;
extern RS485SERVO RS485Servo_R;

extern RS485MOTOR RS485Motor_FL;
extern RS485MOTOR RS485Motor_FR;
extern RS485MOTOR RS485Motor_R;


#endif /* RS485COMM_H_ */
