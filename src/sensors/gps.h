/*
 * gps.h
 *
 *  Created on: Aug 25, 2013
 *      Author: Jure
 */


#ifndef GPS_H_1
#define GPS_H_1

// Flag register typedef
typedef union
{
	struct
	{
		volatile uint32_t flag;
	}flag;

	 struct
	 {
		volatile uint8_t BIT0:1;
		volatile uint8_t BIT1:1;
		volatile uint8_t BIT2:1;
		volatile uint8_t BIT3:1;
		volatile uint8_t BIT4:1;
		volatile uint8_t BIT5:1;
		volatile uint8_t BIT6:1;
		volatile uint8_t BIT7:1;
		volatile uint8_t BIT8:1;
		volatile uint8_t BIT9:1;
		volatile uint8_t BIT10:1;
		volatile uint8_t BIT11:1;
		volatile uint8_t BIT12:1;
		volatile uint8_t BIT13:1;
		volatile uint8_t BIT14:1;
		volatile uint8_t BIT15:1;
		volatile uint8_t BIT16:1;
		volatile uint8_t BIT17:1;
		volatile uint8_t BIT18:1;
		volatile uint8_t BIT19:1;
		volatile uint8_t BIT20:1;
		volatile uint8_t BIT21:1;
		volatile uint8_t BIT22:1;
		volatile uint8_t BIT23:1;
		volatile uint8_t BIT24:1;
		volatile uint8_t BIT25:1;
		volatile uint8_t BIT26:1;
		volatile uint8_t BIT27:1;
		volatile uint8_t BIT28:1;
		volatile uint8_t BIT29:1;
		volatile uint8_t BIT30:1;
		volatile uint8_t BIT31:1;
		volatile uint8_t BIT32:1;

	 }bits;
}FlagGPS;


#define GPS_MAXMESSAGELENGTH	128

#define GPS_RECEIVE

// How long to wait for GPS send end. Each tick is 0.1 ms
#define GPS_IMEOUT		100000

#define USE_TIMER

extern uint8_t GPS_DataBuffer[64];
extern volatile uint16_t GPS_RECDATA[32];
extern volatile uint8_t GPS_Digits[16];
extern volatile uint8_t GPS_ProcesState;
extern volatile uint8_t GPS_NextData;
extern volatile uint8_t GPS_Checksum;
extern volatile uint8_t GPS_Checksum_Save;
extern volatile uint8_t GPS_ReceivedChecksum[2];
extern volatile uint8_t GPS_ChecksumCounter;
extern volatile uint16_t GPS_DataTemp;
extern volatile uint8_t GPS_DataTemp_Count;
extern volatile uint8_t GPS_Digits_Count;
// Flag variable

extern volatile FlagGPS GPSFlag;
#define GPSFLAG_DATAVALID		GPSFlag.bits.BIT0
#define GPSFLAG_DOPARSE			GPSFlag.bits.BIT1
#define GPSFLAG_HASLOCK			GPSFlag.bits.BIT2
#define GPSFLAG_GGA				GPSFlag.bits.BIT3
#define GPSFLAG_RMC				GPSFlag.bits.BIT4
#define GPSFLAG_CR_RECEIVED		GPSFlag.bits.BIT5
#define GPSFLAG_CHECKSUM_RESET	GPSFlag.bits.BIT6
#define GPS_SENDING				GPSFlag.bits.BIT7



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
#define GPS_MAGVAR_T			GPS_RECDATA[24]
#define GPS_MAGVAR_FRAC_T		GPS_RECDATA[25]
#define GPS_MAGWAR_EW_T			GPS_RECDATA[26]


void GPSTransferDMA(uint8_t *data, int length);
void GPS_Sending(int state);
float32_t GPSIntToFloat(uint16_t whole, uint16_t frac);
ErrorStatus gps_initData(GPSData *data, uint32_t time);
void GPSStopOutput(void);
void GPSSetDataOutput(uint32_t time);
uint8_t GPS_GetChar(uint8_t data, FlagStatus part);
uint8_t GPS_CalculateChecksum(uint8_t *data, int startByte, int dataCount);
uint8_t GPS_CharToByte(uint8_t data);
void GPS_ReceiveProcess(uint8_t data, uint32_t time);



#endif /* GPS_H_ */
