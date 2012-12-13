/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/



#include "allinclude.h"

/* Private macro */
/* Private variables */
/* Private function prototypes */
/* Private functions */

/*
 * ===========================================================================
 *
 *  Abstract: main program
 *
 * ===========================================================================
 */

// Using Git for versioning


int main(void)
{
	// Startup delay - 0,5 sec
	Delaynus(500000);
	//configure hardware
	System_Config();
	// Set LED OK = 1
	LED_OK_ON;
	// Set default PWM out values
	PWMOUT_1 = TIM1_PULSE;
	PWMOUT_2 = TIM1_PULSE;
	PWMOUT_3 = TIM1_PULSE;
	PWMOUT_4 = TIM1_PULSE;
	PWMOUT_5 = TIM1_PULSE;
	PWMOUT_6 = TIM1_PULSE;
	PWMOUT_7 = TIM1_PULSE;
	PWMOUT_8 = TIM1_PULSE;
	PWMOUT_9 = TIM1_PULSE;
	PWMOUT_10 = TIM1_PULSE;
	PWMOUT_11 = TIM1_PULSE;
	PWMOUT_12 = TIM1_PULSE;

    while (1)
    {
        //Delaynus(1000000 / 2000);    /* A short delay */
        //Delaynus(1000000 / 2000);
        // Check MODBUS for messages
        if(MB_HASDATA)
        {
        	// Execute process data function
        	MODBUS_ExecuteFunction();
        	// Send data
        	// Enable DMA transfer
        	transferDMA_USART2(MODBUSData.bytes.cdata, MODBUSData.bytes.uiDataCount);
        	// Set MODBUS to IDLE
        	MB_SETTOIDLE;
        }

        //check SCR
        if(SCR1 & SCR_GETPSDATA)
        {
        	PSRequestData();
        	SCR1 = SCR1 & ~SCR_GETPSDATA;
        }
        if(SCR1 & SCR_SETPSI0)
        {
        	PSSetI0();
        	SCR1 = SCR1 & ~SCR_SETPSI0;
        }
        if(SCR1 & SCR_PSRESET)
        {
        	PSReset();
           	SCR1 = SCR1 & ~SCR_PSRESET;
        }
        if(SCR1 & SCR_GETGPSDATA)
        {
        	GPSSetDataOutput();
        	SCR1 = SCR1 & ~SCR_GETGPSDATA;
        }
        if(SCR1 & SCR_STOP_GPS)
        {
        	GPSStopOutput();
        	SCR1 = SCR1 & ~SCR_STOP_GPS;
        }
        if(SCR1 & SCR_INCPWM)
        {
        	TIM_SetCompare4(TIM1, PWMOUT_1);
        	SCR1 = SCR1 & ~SCR_INCPWM;
        }
        if(SCR1 & SCR_READI2C2)
        {
        	// Begin read of 21 registers from MPU6000
        	masterReceive_beginDMA(MPU6000_ADDRESS, 59, I2C2_DMABufRX, 22);
        	SCR1 = SCR1 & ~SCR_READI2C2;
        }
        if(SCR1 & SCR_WRITEI2C2)
        {

        	SCR1 = SCR1 & ~SCR_WRITEI2C2;
        }
        if(SCR1 & SCR_TESTI2C2AUTO)
        {
        	// Enable MPU
        	MPU6000_Enable(ENABLE);
        	// Enable I2C bypass to write to HMC5883
        	MPU6000_EnableI2CBypass(ENABLE);
        	// Configure HMC5883
        	HMC5883_Enable(ENABLE);
        	// Configure MPL3115A2
        	MPL3115A2_Enable(ENABLE);
        	// Test read HMC
        	//masterReceive_HMC5883L(HMC5883_ADDRESS, 3, I2C2_DMABufRX, 6);
        	// Test read MPL
        	//masterReceive(MPL3115A2_ADDRESS, 0, I2C2_DMABufRX, 45);
        	// Test read MPU
        	//masterReceive(MPU6000_ADDRESS, 59, I2C2_DMABufRX, 21);
        	// Disable I2C bypass
        	MPU6000_EnableI2CBypass(DISABLE);
        	// Configure MPU I2C master mode
        	MPU6000_ConfigureI2CMaster();
        	// Enable MPU I2C master
        	MPU6000_EnableI2CMaster(ENABLE);

        	I2C2_INITDONE = 1;

        	SCR1 = SCR1 & ~SCR_TESTI2C2AUTO;
        }
        if(SCR1 & SCR_SET_PWM_0)
        {
        	PWMOUT_1 = 2100;
       		TIM_SetCompare4(TIM1, PWMOUT_1);
        	SCR1 = SCR1 & ~SCR_SET_PWM_0;
        }
        if(SCR1 & SCR_SET_PWM_PASSON)
        {
        	PWM_PASSTHROUGH = 1;
        	SCR1 = SCR1 & ~SCR_SET_PWM_PASSON;
        }
        if(SCR1 & SCR_SET_PWM_PASSOFF)
        {
        	PWM_PASSTHROUGH = 0;
        	SCR1 = SCR1 & ~SCR_SET_PWM_PASSOFF;
        }
        if(SCR1 & SCR_START_AD)
        {
        	ADC_ENABLED = ~ADC_ENABLED;
        	SCR1 = SCR1 & ~SCR_START_AD;
        }
        if(SCR1 & SCR_INIT_SENSORS)
        {
        	extPeripheralInit();
        	SCR1 = SCR1 & ~SCR_INIT_SENSORS;
        }
        if(SCR1 & SCR_DEC_DAC_FREQ)
        {
        	DAC1_TIM6reloadValue += 0xF;
        	if(DAC1_TIM6reloadValue > 0x11F)
        	{
        		DAC1_TIM6reloadValue = 0xFF;
        	}
        	TIM_SetAutoreload(TIM6, DAC1_TIM6reloadValue);
        	SCR1 = SCR1 & ~SCR_DEC_DAC_FREQ;
        }



        // Check PS for messages
        if(PS_HASDATA)
        {
        	// Execute process data function
        	processPSData();
        	// Set PS process to IDLE
        	PS_SETIDLE;
        	// Mark PS not busy
        	PSBUSY = 0;
        	// Mark sensor OK
        	SCR2 = SCR2 | SCR2_POWEROK;
        }
        // Check I2C data
        if(COPYI2C)
        {
        	copySensorData();
        	// Mark sensors OK
        	SCR2 = SCR2 | SCR2_ACCOK;
        	SCR2 = SCR2 | SCR2_GYROOK;
        	SCR2 = SCR2 | SCR2_MAGOK;
        	SCR2 = SCR2 | SCR2_BAROK;
        }
    }
    /* Infinite loop */
    while (1)
    {
    }
}

