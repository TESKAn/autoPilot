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


// Structure that holds all relevant data
typedef struct tagRS485SERVO
{
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


#endif /* RS485COMM_H_ */
