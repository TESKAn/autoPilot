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



UBX_SOL		*UbxSol;//	  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, INVALID};
UBX_POSLLH    *UbxPosLlh;// = {0,0,0,0,0,0,0, INVALID};
UBX_VELNED    *UbxVelNed;// = {0,0,0,0,0,0,0,0,0, INVALID};
GPS_INFO_t      GPSInfo   = {0,0,0,0,0,0,0,0,0,0, INVALID};

volatile uint8_t GPSTimeout = 0;

void ubx_initData()
{
	UbxSol = fusionData._ubxSol;
	UbxPosLlh = fusionData._ubxPOSLLH;
	UbxVelNed = fusionData._ubxVelNED;
}

void UpdateGPSInfo (void)
{

	if ((UbxSol->Status == NEWDATA) && (UbxPosLlh->Status == NEWDATA) && (UbxVelNed->Status == NEWDATA))
	{
		//RED_FLASH;
		if(GPSInfo.status != NEWDATA)
		{
			GPSInfo.status = INVALID;
			// NAV SOL
			GPSInfo.flags = UbxSol->Flags;
			GPSInfo.satfix = UbxSol->GPSfix;
			GPSInfo.satnum = UbxSol->numSV;
			GPSInfo.PAcc = UbxSol->PAcc;
			GPSInfo.VAcc = UbxSol->SAcc;
			// NAV POSLLH
			GPSInfo.longitude = UbxPosLlh->LON;
			GPSInfo.latitude = UbxPosLlh->LAT;
			GPSInfo.altitude = UbxPosLlh->HEIGHT;

			GPSInfo.veleast = UbxVelNed->VEL_E;
			GPSInfo.velnorth = UbxVelNed->VEL_N;
			GPSInfo.veltop = -UbxVelNed->VEL_D;
			GPSInfo.velground = UbxVelNed->GSpeed;

			GPSInfo.status = NEWDATA;

		}
		// set state to collect new data
		UbxSol->Status = PROCESSED;			// never update old data
		UbxPosLlh->Status = PROCESSED;		// never update old data
		UbxVelNed->Status = PROCESSED;		// never update old data
	}


}


// this function should be called within the UART RX ISR
void ubx_parser(uint8_t c)
{
	static uint8_t ubxstate = UBXSTATE_IDLE;
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
					ubxP =  (int8_t *)&UbxPosLlh; // data start pointer
					ubxEp = (int8_t *)(&UbxPosLlh + 1); // data end pointer
					ubxSp = (int8_t *)&UbxPosLlh->Status; // status pointer
					break;

				case UBX_ID_SOL: // navigation solution
					ubxP =  (int8_t *)&UbxSol; // data start pointer
					ubxEp = (int8_t *)(&UbxSol + 1); // data end pointer
					ubxSp = (int8_t *)&UbxSol->Status; // status pointer
					break;

				case UBX_ID_VELNED: // velocity vector in tangent plane
					ubxP =  (int8_t *)&UbxVelNed; // data start pointer
					ubxEp = (int8_t *)(&UbxVelNed + 1); // data end pointer
					ubxSp = (int8_t *)&UbxVelNed->Status; // status pointer
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
				GPSTimeout = 255;
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



