/* Martin Thomas 4/2009 */

#include "integer.h"
#include "fattime.h"
#include "allinclude.h"
//#include "rtc.h"

DWORD get_fattime (void)
{
	DWORD res = 0;

	//RTC_t rtc;

	//rtc_gettime( &rtc );
	if((GPS_VALID & _BIT15) != 0)
	{
	res =  (((DWORD)GPS_YEAR - 1980) << 25)
			| ((DWORD)GPS_MONTH << 21)
			| ((DWORD)GPS_DAY << 16)
			| (WORD)(GPS_HOURS << 11)
			| (WORD)(GPS_MINUTES << 5)
			| (WORD)(GPS_SECONDS >> 1);
		return res;
	}
	else return 0x20c96ca0;	// return 18.12.2013 13:37:00
}

