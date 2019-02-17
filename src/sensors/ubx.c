/*
 * ubx.c
 *
 *  Created on: 11. jul. 2016
 *      Author: jmoc
 */


#include "stm32f4xx.h"
#include "arm_math.h"
#include "math/myMath_typedefs.h"
#include "math/myMath_vec3.h"

#include <inttypes.h>
#include "sensor_typedefs.h"
#include "ubx.h"


// ubx protocol parser state machine
#define	UBXSTATE_IDLE	0
#define	UBXSTATE_SYNC1	1
#define	UBXSTATE_SYNC2	2
#define	UBXSTATE_CLASS	3
#define	UBXSTATE_LEN1	4
#define	UBXSTATE_LEN2	5
#define UBXSTATE_DATA	6
#define	UBXSTATE_CKA	7
#define	UBXSTATE_CKB	8

// ublox protocoll identifier
#define	UBX_CLASS_NAV	0x01

#define	UBX_ID_POSLLH	0x02
#define UBX_ID_SOL		0x06
#define	UBX_ID_VELNED	0x12

#define	UBX_SYNC1_CHAR	0xB5
#define	UBX_SYNC2_CHAR	0x62



GPS_INFO_t      GPSInfo;

volatile uint16_t GPSTimeout = 0;
volatile uint8_t ubxstate = UBXSTATE_IDLE;

void ubx_initData()
{
	GPSInfo.PAcc = 0;
	GPSInfo.VAcc = 0;
	GPSInfo.altitude = 0;
	GPSInfo.flags = 0;
	GPSInfo.latitude = 0;
	GPSInfo.longitude = 0;
	GPSInfo.satfix = 0;
	GPSInfo.satnum = 0;
	GPSInfo.status = 0;
	GPSInfo.veleast = 0;
	GPSInfo.velground = 0;
	GPSInfo.velnorth = 0;
	GPSInfo.veltop = 0;
	GPSInfo.ui32NumMessages = 0;

	fusionData._ubxVelNED.CAcc = 0;
	fusionData._ubxVelNED.GSpeed = 0;
	fusionData._ubxVelNED.Heading = 0;
	fusionData._ubxVelNED.ITOW = 0;
	fusionData._ubxVelNED.SAcc = 0;
	fusionData._ubxVelNED.Speed = 0;
	fusionData._ubxVelNED.Status = 0;
	fusionData._ubxVelNED.VEL_D = 0;
	fusionData._ubxVelNED.VEL_E = 0;
	fusionData._ubxVelNED.VEL_N = 0;

	fusionData._ubxPOSLLH.HEIGHT = 0;
	fusionData._ubxPOSLLH.HMSL = 0;
	fusionData._ubxPOSLLH.Hacc = 0;
	fusionData._ubxPOSLLH.ITOW = 0;
	fusionData._ubxPOSLLH.LAT = 0;
	fusionData._ubxPOSLLH.LON = 0;
	fusionData._ubxPOSLLH.Status = 0;
	fusionData._ubxPOSLLH.Vacc = 0;

	fusionData._ubxSol.ECEFVX = 0;
	fusionData._ubxSol.ECEFVY = 0;
	fusionData._ubxSol.ECEFVZ = 0;
	fusionData._ubxSol.ECEF_X = 0;
	fusionData._ubxSol.ECEF_Y = 0;
	fusionData._ubxSol.ECEF_Z = 0;
	fusionData._ubxSol.Flags = 0;
	fusionData._ubxSol.Frac = 0;
	fusionData._ubxSol.GPSfix = 0;
	fusionData._ubxSol.ITOW = 0;
	fusionData._ubxSol.PAcc = 0;
	fusionData._ubxSol.PDOP = 0;
	fusionData._ubxSol.SAcc = 0;
	fusionData._ubxSol.Status = 0;
	fusionData._ubxSol.numSV = 0;
	fusionData._ubxSol.res1 = 0;
	fusionData._ubxSol.res2 = 0;
	fusionData._ubxSol.week = 0;



}

void UpdateGPSInfo (void)
{

	if ((fusionData._ubxSol.Status == NEWDATA) && (fusionData._ubxPOSLLH.Status == NEWDATA) && (fusionData._ubxVelNED.Status == NEWDATA))
	{
		//RED_FLASH;
		if(GPSInfo.status != NEWDATA)
		{
			GPSInfo.status = INVALID;
			// NAV SOL
			GPSInfo.flags = fusionData._ubxSol.Flags;
			GPSInfo.satfix = fusionData._ubxSol.GPSfix;
			GPSInfo.satnum = fusionData._ubxSol.numSV;
			GPSInfo.PAcc = fusionData._ubxSol.PAcc;
			GPSInfo.VAcc = fusionData._ubxSol.SAcc;
			// NAV POSLLH
			GPSInfo.longitude = fusionData._ubxPOSLLH.LON;
			GPSInfo.latitude = fusionData._ubxPOSLLH.LAT;
			GPSInfo.altitude = fusionData._ubxPOSLLH.HEIGHT;

			GPSInfo.veleast = fusionData._ubxVelNED.VEL_E;
			GPSInfo.velnorth = fusionData._ubxVelNED.VEL_N;
			GPSInfo.veltop = -fusionData._ubxVelNED.VEL_D;
			GPSInfo.velground = fusionData._ubxVelNED.GSpeed;

			GPSInfo.status = NEWDATA;

		}
		// set state to collect new data
		fusionData._ubxSol.Status = PROCESSED;			// never update old data
		fusionData._ubxPOSLLH.Status = PROCESSED;		// never update old data
		fusionData._ubxVelNED.Status = PROCESSED;		// never update old data

		GPSInfo.ui32NumMessages++;
	}
}

void ubx_timeout()
{
	if(0 < GPSTimeout)
	{
		GPSTimeout--;
		if(0 == GPSTimeout)
		{
			ubxstate = UBXSTATE_IDLE;
		}
	}
}

// this function should be called within the UART RX ISR
void ubx_parser(uint8_t c)
{

	static uint8_t cka, ckb;
	static uint16_t msglen;
	static int8_t *ubxP, *ubxEp, *ubxSp; // pointers to data currently transfered

	switch(ubxstate)
	{
		case UBXSTATE_IDLE: // check 1st sync byte
			if (c == UBX_SYNC1_CHAR) ubxstate = UBXSTATE_SYNC1;
			else ubxstate = UBXSTATE_IDLE; // out of synchronization
			break;

		case UBXSTATE_SYNC1: // check 2nd sync byte
			if (c == UBX_SYNC2_CHAR) ubxstate = UBXSTATE_SYNC2;
			else ubxstate = UBXSTATE_IDLE; // out of synchronization
			break;

		case UBXSTATE_SYNC2: // check msg class to be NAV
			if (c == UBX_CLASS_NAV) ubxstate = UBXSTATE_CLASS;
			else ubxstate = UBXSTATE_IDLE; // unsupported message class
			break;

		case UBXSTATE_CLASS: // check message identifier
			switch(c)
			{
				case UBX_ID_POSLLH: // geodetic position
					ubxP =  (int8_t *)&fusionData._ubxPOSLLH; // data start pointer
					ubxEp = (int8_t *)(&fusionData._ubxPOSLLH + 1); // data end pointer
					ubxSp = (int8_t *)&fusionData._ubxPOSLLH.Status; // status pointer
					break;

				case UBX_ID_SOL: // navigation solution
					ubxP =  (int8_t *)&fusionData._ubxSol; // data start pointer
					ubxEp = (int8_t *)(&fusionData._ubxSol + 1); // data end pointer
					ubxSp = (int8_t *)&fusionData._ubxSol.Status; // status pointer
					break;

				case UBX_ID_VELNED: // velocity vector in tangent plane
					ubxP =  (int8_t *)&fusionData._ubxVelNED; // data start pointer
					ubxEp = (int8_t *)(&fusionData._ubxVelNED + 1); // data end pointer
					ubxSp = (int8_t *)&fusionData._ubxVelNED.Status; // status pointer
					break;

				default:			// unsupported identifier
					ubxstate = UBXSTATE_IDLE;
					break;
			}
			if (ubxstate != UBXSTATE_IDLE)
			{
				ubxstate = UBXSTATE_LEN1;
				cka = UBX_CLASS_NAV + c;
				ckb = UBX_CLASS_NAV + cka;
			}
			break;

		case UBXSTATE_LEN1: // 1st message length byte
			msglen = c;
			cka += c;
			ckb += cka;
			ubxstate = UBXSTATE_LEN2;
			break;

		case UBXSTATE_LEN2: // 2nd message length byte
			msglen += ((uint16_t)c)<<8;
			cka += c;
			ckb += cka;
			// if the old data are not processed so far then break parsing now
			// to avoid writing new data in ISR during reading by another function
			if ( *ubxSp == NEWDATA )
			{
				UpdateGPSInfo(); //update GPS info respectively
				ubxstate = UBXSTATE_IDLE;
			}
			else // data invalid or allready processd
			{
				*ubxSp = INVALID;
				ubxstate = UBXSTATE_DATA;
			}
			break;

		case UBXSTATE_DATA:
			if (ubxP < ubxEp) *ubxP++ = c; // copy curent data byte if any space is left
			cka += c;
			ckb += cka;
			if (--msglen == 0) 	ubxstate = UBXSTATE_CKA; // switch to next state if all data was read
			break;

		case UBXSTATE_CKA:
			if (c == cka) ubxstate = UBXSTATE_CKB;
			else
			{
				*ubxSp = INVALID;
				ubxstate = UBXSTATE_IDLE;
			}
			break;

		case UBXSTATE_CKB:
			if (c == ckb)
			{
				*ubxSp = NEWDATA; // new data are valid
				UpdateGPSInfo(); //update GPS info respectively
				GPSTimeout = 2000;
			}
			else
			{	// if checksum not fit then set data invalid
				*ubxSp = INVALID;
			}
			ubxstate = UBXSTATE_IDLE; // ready to parse new data
			break;

		default: // unknown ubx state
			ubxstate = UBXSTATE_IDLE;
			break;
	}

}



