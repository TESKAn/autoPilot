/*
 * modbus.h
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */

#ifndef MODBUS_H_
#define MODBUS_H_

//function definitions
void MODBUS_Timer(void);
void MODBUS_ExecuteFunction();
void MODBUS_ProcessData(unsigned int data);
void MODBUS_Init(void);
void crcOnByte(unsigned int ucMsgByte);
void crcOnMessage(unsigned int uiDataLen);

// Define MODBUS ID
#define MB_SLAVEID						0x33

// Define total registers
#define MB_TOTALREGISTERS				90

// Define number of total bytes to be used for MODBUS receive/send
#define MB_TOTALBYTES					200//18 + (MB_TOTALREGISTERS * 2) + 2	// Add two to be certain

// MODBUS functionality include
#define MB_USETIMEOUTS



// Define registers
// System/control
#define SCR1				MODBUSReg[0]
#define SCR2				MODBUSReg[1]
// GPS
#define GPS_LATITUDE		MODBUSReg[2]
#define GPS_LATITUDE_FRAC	MODBUSReg[3]
#define GPS_LONGITUDE		MODBUSReg[4]
#define GPS_LONGITUDE_FRAC	MODBUSReg[5]
#define GPS_ALTITUDE		MODBUSReg[6]
#define GPS_ALTITUDE_FRAC	MODBUSReg[7]
#define GPS_SPEED			MODBUSReg[8]
#define GPS_SPEED_FRAC		MODBUSReg[9]
#define GPS_TRACKANGLE		MODBUSReg[10]
#define GPS_TRACKANGLE_FRAC	MODBUSReg[11]
#define GPS_SATSTATUS		MODBUSReg[12]
#define GPS_HOURS			MODBUSReg[13]
#define GPS_MINUTES			MODBUSReg[14]
#define GPS_SECONDS			MODBUSReg[15]
#define GPS_MILISECONDS		MODBUSReg[16]
#define GPS_DAY				MODBUSReg[17]
#define GPS_MONTH			MODBUSReg[18]
#define GPS_YEAR			MODBUSReg[19]
#define GPS_NS_EW			MODBUSReg[20]	// NS, EW, MAGWAR_E/W
#define GPS_VALID			MODBUSReg[21]	// bits0-4 = fix valid bit15=GPS_VALID bit14=CHECKSUM_OK
#define GPS_HDOP			MODBUSReg[22]
#define GPS_HDOP_FRAC		MODBUSReg[23]
#define GPS_GG				MODBUSReg[24]
#define GPS_GG_FRAC			MODBUSReg[25]

// Power analyzer
#define CURRENT				MODBUSReg[26]
#define MAH					MODBUSReg[27]
#define VOLTAGE				MODBUSReg[28]
#define T1					MODBUSReg[29]
#define T2					MODBUSReg[30]
#define T3					MODBUSReg[31]
// I2C
#define I2C2_STARTREG		MODBUSReg[32]
#define I2C2_WRITEDATA		MODBUSReg[33]
#define I2C2_READDATA		MODBUSReg[34]
// Accelerometer
#define ACC_X				MODBUSReg[35]
#define ACC_Y				MODBUSReg[36]
#define ACC_Z				MODBUSReg[37]
// Magnetometer
#define MAG_X				MODBUSReg[38]
#define MAG_Y				MODBUSReg[39]
#define MAG_Z				MODBUSReg[40]
// Gyrometer
#define GYRO_X				MODBUSReg[41]
#define GYRO_Y				MODBUSReg[42]
#define GYRO_Z				MODBUSReg[43]
// Barometer
#define BARO				MODBUSReg[44]
// PWM IN
#define PWMIN_1				MODBUSReg[45]
#define PWMIN_2				MODBUSReg[46]
#define PWMIN_3				MODBUSReg[47]
#define PWMIN_4				MODBUSReg[48]
#define PWMIN_5				MODBUSReg[49]
#define PWMIN_6				MODBUSReg[50]
#define PWMIN_7				MODBUSReg[51]
#define PWMIN_8				MODBUSReg[52]
// Signal status IN
#define SIG_IN				MODBUSReg[53]
// PWM OUT
#define PWMOUT_1			MODBUSReg[54]
#define PWMOUT_2			MODBUSReg[55]
#define PWMOUT_3			MODBUSReg[56]
#define PWMOUT_4			MODBUSReg[57]
#define PWMOUT_5			MODBUSReg[58]
#define PWMOUT_6			MODBUSReg[59]
#define PWMOUT_7			MODBUSReg[60]
#define PWMOUT_8			MODBUSReg[61]
#define PWMOUT_9			MODBUSReg[62]
#define PWMOUT_10			MODBUSReg[63]
#define PWMOUT_11			MODBUSReg[64]
#define PWMOUT_12			MODBUSReg[65]
// Analog input
#define AIN0				MODBUSReg[66]
#define AIN1				MODBUSReg[67]
#define AIN2				MODBUSReg[68]
#define AIN3				MODBUSReg[69]
// SD result
#define SD_RESULT			MODBUSReg[70]
// direction angles
#define AHRS_PITCH			MODBUSReg[71]
#define AHRS_ROLL			MODBUSReg[72]
#define AHRS_YAW			MODBUSReg[73]
// baro altitude - fraction
#define BARO_FRAC			MODBUSReg[74]
// external graphs
#define GRAPH_1				MODBUSReg[75]
#define GRAPH_2				MODBUSReg[76]
#define GRAPH_3				MODBUSReg[77]
#define GRAPH_4				MODBUSReg[78]
#define GRAPH_5				MODBUSReg[79]
#define GRAPH_6				MODBUSReg[80]

// Magnetometer heading in deg/10
#define MAG_HEADING			MODBUSReg[81]

// Signal strength indicator
#define RSSI_STRENGTH		MODBUSReg[82]

// Typedefs
// MODBUS data structure
typedef union
{
	struct	//MB_TOTALBYTES bytes
	{
		uint8_t ucdata[MB_TOTALBYTES];
	}base;

	 struct	//MB_TOTALBYTES bytes
	 {
		 char cSlaveID:8;
		 char cFunctionCode:8;
		 unsigned int uiReadStartingAddress:16;
		 unsigned int uiQuantToRead:16;
		 unsigned int uiWriteStartingAddress:16;
		 unsigned int uiQuantToWrite:16;
		 unsigned char ucWriteByteCount:8;
	 	 unsigned char ucDataReady:8;
	 	 unsigned int uiDataIndex:16;
	 	 unsigned int uiDataCount:16;
	 	 unsigned int uiCRC:16;
	 	uint16_t uiData[(MB_TOTALBYTES - 18)/2];
	 }data;

	 struct	//MB_TOTALBYTES bytes
	 {
	 	 char cSlaveID:8;
		 char cFunctionCode:8;
		 unsigned char uiReadStartingAddressLo:8;
		 unsigned char uiReadStartingAddressHi:8;
		 unsigned char uiQuantToReadLo:8;
		 unsigned char uiQuantToReadHi:8;
		 unsigned char uiWriteStartingAddressLo:8;
		 unsigned char uiWriteStartingAddressHi:8;
		 unsigned char uiQuantToWriteLo:8;
		 unsigned char uiQuantToWriteHi:8;
		 unsigned char ucWriteByteCount:8;
	 	 unsigned char ucDataReady:8;
	 	 unsigned int uiDataIndex:16;
	 	 unsigned int uiDataCount:16;
	 	 unsigned char uiCRCLo:8;
	 	 unsigned char uiCRCHi:8;
	 	uint8_t cdata[MB_TOTALBYTES - 18];
	 }bytes;

} Typedef_mbd;

//MODBUS variables
extern volatile int MODBUS_ReceiveState;
extern volatile unsigned char ucMBCRCLOW;
extern volatile unsigned char ucMBCRCHI;
// Var holds data
extern Typedef_mbd MODBUSData;
// MODBUS temp reg variable
extern volatile uint16_t MODBUSTempReg;

extern int volatile MODBUS_Timeout;
// MODBUS registers
extern uint16_t MODBUSReg[MB_TOTALREGISTERS];


// Define MODBUS timeouts
// 50 msec between messages
#define MB_3CHAR						50
// 10 msec between characters
#define MB_1CHAR						10
// Timeout is 1 sec delay
#define MB_TIMEOUT						1000
// Maximum measured time
#define MB_MAXTIME						1100

//MB functions macros
#define MBRWMULTIPLEREGISTERS			23
#define MBWRITESINGLEREGISTER			6
#define MBREADHOLDINGREGISTERS			3
#define MBWRITEMULTIPLEREGISTERS		16
#define MBEXCEPTION						0x80
#define MBILLEGALFCODE					0x01
#define MBILLEGALDATAADDRESS			0x02

// MODBUS states definition
#define MB_IDLE							0
#define MB_RECEIVINGFCODE				1
#define MB_RECEIVINGRSTARTADDRESS_HI	2
#define MB_RECEIVINGRSTARTADDRESS_LO	3
#define MB_RECEIVINGQREAD_HI			4
#define MB_RECEIVINGQREAD_LO			5
#define MB_RECEIVINGWRITESTART_HI		6
#define MB_RECEIVINGWRITESTART_LO		7
#define MB_RECEIVINGQWRITE_HI			8
#define MB_RECEIVINGQWRITE_LO			9
#define MB_RECEIVINGBYTECOUNT			10
#define MB_RECEIVINGDATA_HI				11
#define MB_RECEIVINGDATA_LO				12
#define MB_RECEIVINGCRC_HI				13
#define MB_RECEIVINGCRC_LO				14
#define MB_DATAREADY					15

// MODBUS check macros
#define MB_HASDATA						MODBUS_ReceiveState == MB_DATAREADY
#define MB_SETTOIDLE					MODBUS_ReceiveState = MB_IDLE

#endif /* MODBUS_H_ */
