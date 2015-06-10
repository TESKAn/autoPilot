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

// Function declarations

UInt16 RS485_MasterInitData(void);
UInt16 RS485_MasterWriteByte(void);
UInt16 RS485_States_Master(void);
UInt16 RS485_MasterecodeMessage(UInt8 data);

UInt16 RS485_initData(void);
UInt16 RS485_writeByte(void);
UInt16 RS485_States_slave(UInt8 data);
UInt16 RS485_decodeMessage(void);

// Hardware dependent macros
#define RS485_ENABLE_RX						//ioctl(GPIO_C, GPIO_CLEAR_PIN, BIT_3)
#define RS485_ENABLE_TX						//ioctl(GPIO_C, GPIO_SET_PIN, BIT_3)
#define RS485_ENABLE_TX_INT					//ioctl(SCI_0, SCI_TX_EMPTY_INT, SCI_ENABLE)
#define RS485_DISABLE_TX_INT				//ioctl(SCI_0, SCI_TX_EMPTY_INT, SCI_DISABLE)
#define RS485_ENABLE_TX_IDLE_INT			//ioctl(SCI_0, SCI_TX_IDLE_INT, SCI_ENABLE)
#define RS485_DISABLE_TX_IDLE_INT			//ioctl(SCI_0, SCI_TX_IDLE_INT, SCI_DISABLE)
#define RS485_WRITE(X)						//ioctl(SCI_0, SCI_WRITE_DATA, X)
#define RS485_READ							//ioctl(SCI_0, SCI_READ_DATA, NULL)
#define RS485_TEST_TX_EMPTY					1//ioctl(SCI_0, SCI_GET_TX_EMPTY, NULL)
#define RS485_TEST_TX_IDLE					1//ioctl(SCI_0, SCI_GET_TX_IDLE, NULL)
#define RS485_TEST_RX_FULL					1//ioctl(SCI_0, SCI_GET_RX_FULL, NULL)

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
#define RS485_M_STATE_WRITE					1
#define RS485_M_STATE_TORQUE_ON				2
#define RS485_M_STATE_TORQUE_OFF			3
#define RS485_M_STATE_READ_ALL				4
#define RS485_M_STATE_SET_POS				5
#define RS485_M_STATE_SET_SPEED				6
#define RS485_M_STATE_SET_COMPLIANCE		7
#define RS485_M_STATE_REQUEST				8
//#define RS485_M_STATE_
//#define RS485_M_STATE_

// Receiver states
#define RS485_M_IDLE				0
#define RS485_M_WAIT_FOR_SIGNAL		1
#define RS485_M_WAIT_FOR_ID			2
#define RS485_M_WAIT_FOR_LENGTH		3
#define RS485_M_WAIT_FOR_INSTR_ERR	4
#define RS485_M_WAIT_FOR_PARAMETERS	5
#define RS485_M_WAIT_FOR_CHECKSUM	6

// Transmitter states
#define RS485_M_TX_IDLE				0
#define RS485_M_TX_SENDING			1
#define RS485_M_TX_FINISHED			2

// Slave defines
// Receiver states
#define RS485_IDLE					0
#define RS485_WAIT_FOR_SIGNAL		2
#define RS485_WAIT_FOR_ID			4
#define RS485_WAIT_FOR_LENGTH		8
#define RS485_WAIT_FOR_INSTR_ERR	16
#define RS485_WAIT_FOR_PARAMETERS	32
#define RS485_WAIT_FOR_CHECKSUM		64

// Transmitter states
#define RS485_TX_IDLE				0
#define RS485_TX_SENDING			1
#define RS485_TX_FINISHED			2

// Instructions
#define RS485_INSTR_PING			1
#define RS485_INSTR_READ_DATA		2
#define RS485_INSTR_WRITE_DATA		3
#define RS485_INSTR_REG WRITE		4
#define RS485_INSTR_ACTION			5
#define RS485_INSTR_RESET			6
#define RS485_INSTR_SYNC_WRITE		7

// Structure that holds all relevant data
typedef struct tagRS485SERVO
{
	union
	{
		struct
		{
			UInt8 ui8Data[49];				// Main data structure
			UInt8 ui8SendRcvBuffer[56];		// Buffer for transmitting/receiving data
			UInt8 ui8RcvBufferIndex;		// Index in buffer for receiving/transmitting
			UInt8 ui8BytesToSend;			// How many bytes to transmitt (including signal, ID, checksum)
			UInt8 ui8RcvState;				// Receiver state
			UInt8 ui8TxState;				// Transmitter state
			UInt8 ui8DataLength;			// How many parameter bytes to receive/read
			UInt8 ui8InstrErr;				// Instruction/error code of current message
			UInt8 ui8RWAddress;				// Address to start read/write
			UInt8 ui8BytesToRW;				// Bytes to read/write
			UInt8 ui8ParamsReceived;
			UInt8 ui8Empty[1];
			UInt8 ui8Checksum;				// Checksum of received data
			/*
			union
			{
				UInt8 ui8Bytes[2];
				UInt16 ui16Word;
			}RcvdData;*/
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
			UInt8 ui8Empty2[5];
			UInt8 ui8AlarmShutdown;
			UInt8 ui8TorqueEnabled;
			UInt8 ui8LED;
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
			UInt8 ui8SendRcvBuffer_[56];
			UInt8 ui8RcvBufferIndex_;
			UInt8 ui8BytesToSend_;
			UInt8 ui8RcvState_;
			UInt8 ui8TxState_;
			UInt8 ui8DataLength_;
			UInt8 ui8InstrErr_;
			UInt8 ui8RWAddress_;
			UInt8 ui8BytesToRW_;
			UInt8 ui8ParamsReceived_;
			UInt8 ui8Empty_[1];
			UInt8 ui8Checksum_;
			//UInt8 ui8RcvdData[2];

		}REGS;
	};

}RS485SERVO;


typedef struct tagRS485SERVOSLAVE
{
	// Reading from address
	UInt8 ui8ReadStartAddress;
	// Status
	UInt8 ui8Status;
	// Is data good?
	UInt8 ui8DataGood;
	// Checksum
	UInt8 ui8Checksum;
	union
	{
		UInt8 ui8Data[49];				// Main data structure
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
			UInt8 ui8Empty2[5];
			UInt8 ui8AlarmShutdown;
			UInt8 ui8TorqueEnabled;
			UInt8 ui8LED;
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
			UInt8 ui8SendRcvBuffer_[56];
		}REGS;
	};

}RS485SERVOSLAVE;



typedef struct tagRS485SERVOMASTER
{
	// Structures that hold data for all servos
	RS485SERVOSLAVE RS485Slaves[RS485_NUMSLAVES];
	// Pointer to slave that we are receiving transmission for
	RS485SERVOSLAVE* RS485CurrentSlave;
	UInt8 ui8SendRcvBuffer[56];		// Buffer for transmitting/receiving data
	UInt8 ui8RcvBufferIndex;		// Index in buffer for receiving/transmitting
	UInt8 ui8BytesToSend;			// How many bytes to transmitt (including signal, ID, checksum)
	UInt8 ui8RcvState;				// Receiver state
	UInt8 ui8TxState;				// Transmitter state
	UInt8 ui8MasterState;			// Master state
	UInt8 ui8MasterRequest;			// Request to send
	UInt8 ui8ReqSlaveAddress;		// Slave to query
	UInt8 ui8DataLength;			// How many parameter bytes to receive/read
	UInt8 ui8InstrErr;				// Instruction/error code of current message
	UInt8 ui8RWAddress;				// Address to start read/write
	UInt8 ui8BytesToRW;				// Bytes to read/write
	UInt8 ui8ParamsReceived;
	UInt8 ui8Checksum;				// Checksum of received data
	UInt16 ui16SlaveTimeout;		// Holds timeout value for slave comm. When 0, timeout
	union
	{
		UInt8 ui8Bytes[2];
		UInt16 ui16Word;
	}RcvdData;

}RS485SERVOMASTER;



#endif /* RS485COMM_H_ */
