/*
 * gps.h
 *
 *  Created on: Oct 21, 2012
 *      Author: Jure
 */

#ifndef GPS_H_
#define GPS_H_

#define GPS_MAXMESSAGELENGTH	128

#define GPS_RECEIVE


#define USE_TIMER

extern uint8_t GPS_DataBuffer[64];
extern uint16_t GPS_RECDATA[32];
extern uint8_t GPS_Digits[16];
extern uint8_t GPS_ProcesState;
extern uint8_t GPS_NextData;
extern uint8_t GPS_Checksum;
extern uint8_t GPS_Checksum_Save;
extern uint8_t GPS_ReceivedChecksum[2];
extern uint8_t GPS_ChecksumCounter;
extern uint16_t GPS_DataTemp;
extern uint8_t GPS_DataTemp_Count;
extern uint8_t GPS_Digits_Count;
// Flag variable
extern Flag GPSFlag;
#define GPSFLAG_DATAVALID		GPSFlag.bits.BIT0
#define GPSFLAG_DOPARSE			GPSFlag.bits.BIT1
#define GPSFLAG_HASLOCK			GPSFlag.bits.BIT2
#define GPSFLAG_GGA				GPSFlag.bits.BIT3
#define GPSFLAG_RMC				GPSFlag.bits.BIT4
#define GPSFLAG_CR_RECEIVED		GPSFlag.bits.BIT5
#define GPSFLAG_CHECKSUM_RESET	GPSFlag.bits.BIT6

void GPSStopOutput(void);
void GPSSetDataOutput(void);
uint8_t GPS_GetChar(uint8_t data, FlagStatus part);
uint8_t GPS_CalculateChecksum(uint8_t *data, int startByte, int dataCount);
uint8_t GPS_CharToByte(uint8_t data);
void GPS_ReceiveProcess(uint8_t data);

// GPS state variables
// Waiting to get start of message, '$'
#define GPS_WAITINGSTART		0
#define GPS_RECEIVE_NUMBER		1	// Waiting for number
#define GPS_RECEIVE_NUMFRAC		2	// Waiting for number fraction
#define GPS_WAITFOREND			3	// Wait for '*'

#define GPS_NEXTDATA_TIME		0
#define GPS_NEXTDATA_DATE		1
#define GPS_NEXTDATA_LATITUDE	2
#define GPS_NEXTDATA_LONGITUDE	3
#define GPS_NEXTDATA_NS			4
#define GPS_NEXTDATA_EW			5
#define GPS_NEXTDATA_FIXVALID	6
#define GPS_NEXTDATA_NUMSAT		7
#define GPS_NEXTDATA_HDOP		8
#define GPS_NEXTDATA_ALTITUDE	9
#define GPS_NEXTDATA_M1			10
#define GPS_NEXTDATA_GG			11
#define GPS_NEXTDATA_M2			12
#define GPS_NEXTDATA_STATUS		13
#define GPS_NEXTDATA_SPEED		14
#define GPS_NEXTDATA_HEADING	15
#define GPS_NEXTDATA_MAGVAR		16
#define GPS_NEXTDATA_MAGVAR_EW	17
#define GPS_NEXTDATA_MODE		18
#define GPS_NEXTDATA_DATACODE	19
#define GPS_NEXTDATA_CHECKSUM	20
#define GPS_NEXTDATA_VOID		21

// GPS
#define GPS_LATITUDE_T			GPS_RECDATA[0]
#define GPS_LATITUDE_FRAC_T		GPS_RECDATA[1]
#define GPS_LONGITUDE_T			GPS_RECDATA[2]
#define GPS_LONGITUDE_FRAC_T	GPS_RECDATA[3]
#define GPS_ALTITUDE_T			GPS_RECDATA[4]
#define GPS_ALTITUDE_FRAC_T		GPS_RECDATA[5]
#define GPS_SPEED_T				GPS_RECDATA[6]
#define GPS_SPEED_FRAC_T		GPS_RECDATA[7]
#define GPS_TRACKANGLE_T		GPS_RECDATA[8]
#define GPS_TRACKANGLE_FRAC_T	GPS_RECDATA[9]
#define GPS_SATSTATUS_T			GPS_RECDATA[10]
#define GPS_HOURS_T				GPS_RECDATA[11]
#define GPS_MINUTES_T			GPS_RECDATA[12]
#define GPS_SECONDS_T			GPS_RECDATA[13]
#define GPS_MILISECONDS_T		GPS_RECDATA[14]
#define GPS_DAY_T				GPS_RECDATA[15]
#define GPS_MONTH_T				GPS_RECDATA[16]
#define GPS_YEAR_T				GPS_RECDATA[17]
#define GPS_NS_EW_T				GPS_RECDATA[18]
#define GPS_VALID_T				GPS_RECDATA[19]	// bits0-4 = fix valid bit15=GPS_VALID bit14=CHECKSUM_OK
#define GPS_HDOP_T				GPS_RECDATA[20]
#define GPS_HDOP_FRAC_T			GPS_RECDATA[21]
#define GPS_GG_T				GPS_RECDATA[22]
#define GPS_GG_FRAC_T			GPS_RECDATA[23]

#endif /* GPS_H_ */
