/*
 * functions.c
 *
 *  Created on: Oct 7, 2012
 *      Author: Jure
 */
#include "allinclude.h"


void NVIC_EnableInterrupts(FunctionalState newState)
{
	//interrupt controller
	NVIC_InitTypeDef NVCInitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

	//init ADC interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = ADC_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART1 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART1_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART2 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART2_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init USART3 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = USART3_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream3 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream4 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init DMA1 stream6 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM4 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM4_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM8 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM8_CC_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init TIM14 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = TIM8_TRG_COM_TIM14_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	//init I2C2 interrupt
	//set IRQ channel
	NVCInitStructure.NVIC_IRQChannel = I2C2_EV_IRQn;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	//set priority 0 - 15
	NVCInitStructure.NVIC_IRQChannelSubPriority = 15;
	//enable IRQ channel
	NVCInitStructure.NVIC_IRQChannelCmd = newState;
	NVIC_Init(&NVCInitStructure);

	if(newState)
	{
		SYSTEM_INTERRUPTS_ON = 1;
	}
	else
	{
		SYSTEM_INTERRUPTS_ON = 0;
	}
}

void extPeripheralInit(void)
{
	// Wait some time
	Delaynus(50000);
	// Set LED OK = 1
	LED_OK_OFF;
	// Short delay
	Delaynus(50000);
	// Set LED OK = 0
	LED_OK_ON;
	// Do a long delay, ca. 1 sec
	Delaynus(1000000);
	// set PS busy
	PSBUSY = 1;
	// Set I0 current for sensor
	PSSetI0();
	LED_OK_OFF;
	// Short delay
	Delaynus(50000);
	// Set LED OK = 0
	LED_OK_ON;
	// Reset PS
	PSReset();
	LED_OK_OFF;
	// Long delay
	Delaynus(2000000);
	// Set LED OK = 0
	LED_OK_ON;
	// Reset PS busy
	PSBUSY = 0;
	// Configure GPS
	GPSSetDataOutput();
	LED_OK_OFF;
	// Short delay
	Delaynus(50000);
	// Set LED OK = 0
	LED_OK_ON;
	// Disable interrupts before configuring I2C
	NVIC_EnableInterrupts(DISABLE);
	// Configure I2C sensors
	sensorInit();
	// Mark sensors initiated
	EXTSENS_INIT_DONE = 1;
	// Reenable interrupts
	NVIC_EnableInterrupts(ENABLE);
}

/*
 * Function Name  : Delaynus
 * Description    : Inserts a delay time abort nus.
 * Input          :nus
 * Output         : None
 * Return         : None
 */
void Delaynus(vu32 nus)
{
    u8 nCount;

    while (nus--)
    {
        for (nCount = 6; nCount != 0; nCount--);
    }
}

void transferDMA_USART2(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART2 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART2);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_4;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;//    USART2_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART2, &DMAInitStructure);

	//Enable DMA1 stream 0 - USART2 TX
	DMA_Cmd(DMA_USART2, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
}

void transferDMA_USART3(uint8_t *data, int length)
{
	DMA_InitTypeDef DMAInitStructure;
	// Configure USART3 DMA
	//deinit DMA channel
	DMA_DeInit(DMA_USART3);
	//set init structure
	//channel to use
	DMAInitStructure.DMA_Channel = DMA_Channel_7;
	//peripheral data address
	DMAInitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;//    USART3_DR_ADDRESS;
	// DMA buffer address
	DMAInitStructure.DMA_Memory0BaseAddr = (uint32_t)data;
	DMAInitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMAInitStructure.DMA_BufferSize = length;
	DMAInitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMAInitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMAInitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMAInitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMAInitStructure.DMA_Mode = DMA_Mode_Normal;
	DMAInitStructure.DMA_Priority = DMA_Priority_Low;
	DMAInitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMAInitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMAInitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMAInitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	//configure peripheral
	DMA_Init(DMA_USART3, &DMAInitStructure);

	//Enable DMA1 stream 4 - USART3 TX
	DMA_Cmd(DMA_USART3, ENABLE);
	//configure to use DMA
	USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
	// Configure end of transfer interrupt
	DMA_ITConfig(DMA_USART3, DMA_IT_TC, ENABLE);
}
