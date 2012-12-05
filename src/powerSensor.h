/*
 * powerSensor.h
 *
 *  Created on: Oct 15, 2012
 *      Author: Jure
 */

#ifndef POWERSENSOR_H_
#define POWERSENSOR_H_

// Data typedef
typedef union
{
	struct	//bytes
	{
		unsigned char  IDFCODE:8;
		unsigned char  PSCRC:8;
		unsigned char  CurrentLow:8;
		unsigned char  CurrentHi:8;
		unsigned char  mAhLow:8;
		unsigned char  mAhHi:8;
		unsigned char  VoltageLow:8;
		unsigned char  VoltageHi:8;
		unsigned char  T1Low:8;
		unsigned char  T1Hi:8;
		unsigned char  T2Low:8;
		unsigned char  T2Hi:8;
		unsigned char  T3Low:8;
		unsigned char  T3Hi:8;

	}bytes;
	struct
	{
		unsigned char  IDFCODE:8;
		unsigned char  PSCRC:8;
		unsigned int  Current:16;
		unsigned int  mAh:16;
		unsigned int  Voltage:16;
		unsigned int  T_1:16;
		unsigned int  T_2:16;
		unsigned int  T_3:16;
	}data;
}PS_DataStruct;

// CRC table
extern const int crc8_Table[256];

// Data
extern uint8_t PS_CRC;
extern uint8_t PS_ReceiveStateVar;
extern int PS_TIMER;
extern PS_DataStruct PS_DATA;
extern int PS_PollTimer;

void PS_Timer(void);
void PSRequestData(void);
void PSSetI0(void);
void PSReset(void);
void processPSData(void);
void PowerSensorCommProcess(uint8_t data);
void PS_CalculateCRC(uint8_t data);


#define PS_ENABLETIMEOUT
#define PS_3CHAR				50
#define PS_1CHAR				10
#define PS_TIMEOUT				1000
#define PS_POLLTIME				500
#define PS_MAXTIME				1100

// State macros
#define PS_IDLE					0
#define PS_GETCURRENTHI			1
#define PS_GETCURRENTLOW		2
#define PS_GETMAHHI				3
#define PS_GETMAHLOW			4
#define PS_GETVOLTAGEHI			5
#define PS_GETVOLTAGELOW		6
#define PS_GETTEMP1HI			7
#define PS_GETTEMP1LOW			8
#define PS_GETTEMP2HI			9
#define PS_GETTEMP2LOW			10
#define PS_GETTEMP3HI			11
#define PS_GETTEMP3LOW			12
#define PS_GETCRC				13
#define PS_DATAREADY			14

// Macros
#define PS_READALL	0x07

#define PS_SENSORID	0x50

#define PS_HASDATA	PS_ReceiveStateVar == PS_DATAREADY
#define PS_SETIDLE	PS_ReceiveStateVar = PS_IDLE


#endif /* POWERSENSOR_H_ */
