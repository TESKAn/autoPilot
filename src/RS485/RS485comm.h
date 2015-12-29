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
}RS485COMMAND;

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
#define RS485_SET_MOTOR_POSITION			5
#define RS485_SET_MOTOR_SPEED				6
#define RS485_SET_MOTOR_MIN_SPEED			7
#define RS485_SET_MOTOR_MAX_SPEED			8
#define RS485_ENABLE_MOTOR_PWMIN			9
#define RS485_SET_SERVO_ID					10
#define RS485_SET_SERVO_ANGLEMIN_LIM		11
#define RS485_SET_SERVO_ANGLEMAX_LIM		12
#define RS485_SET_SERVO_SPEED				13

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

// Master poll slaves
#define RS485_POLL_STATE_SERVO_FR			0
#define RS485_POLL_STATE_SERVO_FL			1
#define RS485_POLL_STATE_SERVO_R			2
#define RS485_POLL_STATE_MOTOR_FR			3
#define RS485_POLL_STATE_MOTOR_FL			4
#define RS485_POLL_STATE_MOTOR_R			5

// Receiver states
#define RS485_M_IDLE				0
#define RS485_M_WAIT_FOR_SIGNAL		1
#define RS485_M_WAIT_FOR_ID			2
#define RS485_M_WAIT_FOR_LENGTH		3
#define RS485_M_WAIT_FOR_DATA		4
#define RS485_M_WAIT_FOR_CHECKSUM	5

// Transmitter states
#define RS485_M_TX_IDLE				0
#define RS485_M_TX_SENDING			1
#define RS485_M_TX_FINISHED			2

// Structure that holds all relevant data for servo
typedef struct tagRS485SERVO
{
	UInt8 errStatus;
	UInt16 ui16MoveToPos;
	union
	{
		struct
		{
			UInt8 ui8Data[49];				// Main data structure
		};
		struct
		{
			UInt16 ui16ModelNumber;
			UInt8 ui8FirmwareVersion;
			UInt8 ui8ID;
			UInt8 ui8BaudRate;
			UInt8 ui8ReturnDelayTime;
			UInt16 CWAngleLimit;
			UInt16 CCWAngleLimit;
			UInt8 ui8Empty1;
			UInt8 ui8InternalTempLimit;
			UInt8 ui8LowLimitVoltage;
			UInt8 ui8HighLimitVoltage;
			UInt16 ui16MaxTorque;
			UInt8 ui8StatusReturnLevel;
			UInt8 ui8AlarmLED;
			UInt8 ui8AlarmShutdown;
			UInt8 ui8Empty2[5];
			//UInt8 ui8AlarmShutdown;
			UInt8 ui8TorqueEnabled;
			UInt8 ui8LEDONOFF;
			UInt8 ui8CWComplianceMargin;
			UInt8 ui8CCWComplianceMargin;
			UInt8 ui8CWComplianceSlope;
			UInt8 ui8CCWComplianceSlope;
			UInt16 ui16GoalPosition;
			UInt16 ui16MovingSpeed;
			UInt16 ui16TorqueLimit;
			UInt16 ui16PresentPosition;
			UInt16 ui16PresentSpeed;
			UInt16 ui16PresentLoad;
			UInt8 ui8PresentVoltage;
			UInt8 ui8PresentTemperature;
			UInt8 ui8Registered;
			UInt8 ui8Empty3;
			UInt8 ui8Moving;
			UInt8 ui8Lock;
			UInt16 ui16Punch;
		}REGS;
	};

}RS485SERVO;

// Structure that holds all relevant data for motor
typedef struct tagRS485MOTOR
{
	UInt8 errStatus;
	union
	{
		struct
		{
			UInt8 ui8Data[39];				// Main data structure
		};
		struct
		{
			// Some params
			UInt16 ui16ModelNumber;
			UInt8 ui8FirmwareVersion;
			UInt8 ui8ID;
			UInt8 ui8BaudRate;
			// Motor control
			// Motor state - idle, run, error
			UInt16 ui16State;
			// State transition command
			UInt16 ui16Command;
			// Motor min/max RPM
			// 9
			float32_t f32MinRPM;
			// 13
			float32_t f32MaxRPM;
			// Enable PWMs
			// 17
			UInt8 ui8PWMEnabled;
			// Errors
			UInt16 ui16Errors;
			// Status of the motor
			float32_t f32UIn;
			float32_t f32IIn;
			float32_t f32PIn;
			float32_t f32RPM;
			float32_t f32SetRPM;


		}REGS;
	};

}RS485MOTOR;

// Function declarations
UInt16 RS485_MasterInitData(void);
UInt16 RS485_MasterState(int state);
UInt16 RS485_ServoTest(UInt8 servoID);
UInt16 RS485_ServoWrite8(UInt8 servoID, UInt8 address, UInt8 data);
UInt16 RS485_ServoWrite16(UInt8 servoID, UInt8 address, UInt16 data);
UInt16 RS485_ServoTorqueON(UInt8 servoID);
UInt16 RS485_ServoTorqueOFF(UInt8 servoID);
UInt16 RS485_ServoReadAll(UInt8 servoID);
UInt16 RS485_ServoSetPosition(UInt8 servoID, UInt16 servoPosition);
UInt16 RS485_ServoSetSpeed(UInt8 servoID, UInt16 servoSpeed);
UInt16 RS485_ServoSetCompliance(UInt8 servoID, UInt8 CWMargin, UInt8 CCWMargin, UInt8 CWSlope, UInt8 CCWSlope);
UInt16 RS485_MotorReadAll(UInt8 motorID);
UInt16 RS485_MotorSetMinSpeed(UInt8 motorID, float32_t minSpeed);
UInt16 RS485_MotorSetMaxSpeed(UInt8 motorID, float32_t minSpeed);
UInt16 RS485_MotorEnablePWMs(UInt8 motorID, UInt16 enable);
UInt16 RS485_BufferQueuedCommand(RS485COMMAND command);
UInt16 RS485_MasterWriteByte(uint8_t *data, int length);
UInt16 RS485_States_Master();
UInt16 RS485_ReceiveMessage(UInt8 data);
UInt16 RS485_DecodeMessage();
Int16 RS485_SetupServos();
Int16 RS485_SetupMotors();



#endif /* RS485COMM_H_ */
