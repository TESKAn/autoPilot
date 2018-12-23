/**
*****************************************************************************
**
**  File        : stm32f4xx_it.c
**
**  Abstract    : Main Interrupt Service Routines.
**                This file provides template for all exceptions handler and
**                peripherals interrupt service routine.
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed �as is,� without any warranty
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "allinclude.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif


/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
	LED_ERR_ON;
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	LED_ERR_ON;
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	LED_ERR_ON;
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
	LED_ERR_ON;
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
	LED_ERR_ON;
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
	LED_ERR_ON;
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
	LED_ERR_ON;
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
	LED_ERR_ON;
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles ADC event interrupt request.
  * @param  None
  * @retval None
  * @services ADC
  */
__attribute__ ((interrupt ("IRQ")))
void ADC_IRQHandler(void)
{
	ADC_ISR_Handler();
}

/**
  * @brief  This function handles I2C 2 event interrupt request.
  * @param  None
  * @retval None
  * @services I2C 2
  */
void I2C2_EV_IRQHandler(void)
{
	// Clear all events
	I2C_ClearFlag(I2C2, I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
}

/**
  * @brief  This function handles I2C 2 error interrupt request.
  * @param  None
  * @retval None
  * @services I2C 2 error
  */
void I2C2_ER_IRQHandler(void)
{
	I2C_ClearFlag(I2C2, I2C_FLAG_TIMEOUT | I2C_FLAG_PECERR | I2C_FLAG_OVR | I2C_FLAG_AF | I2C_FLAG_ARLO | I2C_FLAG_BERR);
}

/**
  * @brief  This function handles Timer 4 event interrupt request.
  * @param  None
  * @retval None
  * @services TIM4
  */
void TIM4_IRQHandler(void)
{
	TIM4_ISR_Handler();
}

/**
  * @brief  This function handles Timer 8 event interrupt request.
  * @param  None
  * @retval None
  * @services TIM8
  */
void TIM8_CC_IRQHandler(void)
{
	TIM8_CC_ISR_Handler();
}

/**
  * @brief  This function handles Timer 14 event interrupt request.
  * @param  None
  * @retval None
  * @services TIM14
  */
void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	TIM8_TRG_COM_TIM14_ISR_Handler();
}

/**
  * @brief  This function handles Timer 9 event interrupt request.
  * @param  None
  * @retval None
  * @services TIM9
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
	TIM1_BRK_TIM9_ISR_Handler();
}

/**
  * @brief  This function handles DMA1 stream6 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA1 stream 6
  */
void DMA1_Stream6_IRQHandler(void)
{
	//DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TC);
	DMA1_Stream6_ISR_Handler();
}

/**
  * @brief  This function handles DMA1 stream4 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA1 stream 4
  */
void DMA1_Stream4_IRQHandler(void)
{
	DMA1_Stream4_ISR_Handler();
}

/**
  * @brief  This function handles DMA1 stream3 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA1 stream 3
  */
void DMA1_Stream3_IRQHandler(void)
{
	DMA1_Stream3_ISR_Handler();
}

/**
  * @brief  This function handles DMA2 stream7 event interrupt request.
  * @param  None
  * @retval None
  * @services DMA7 stream 7
  */
void DMA2_Stream7_IRQHandler(void)
{
	DMA2_Stream7_ISR_Handler();
}

/**
  * @brief  This function handles TIM1 capture compare interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_CC_IRQHandler(void)
{
	TIM1_CC_ISR_Handler();
}

/**
  * @brief  This function handles CAN1 RX interrupt request.
  * @param  None
  * @retval None
  */
void CAN1_RX0_IRQHandler(void)
{
	CAN1_RX0_ISR_Handler();
}

void CAN1_RX1_IRQHandler(void)
{
	CAN1_RX1_ISR_Handler();
}

/**
  * @brief  This function handles CAN1 RX interrupt request.
  * @param  None
  * @retval None
  */
void CAN1_TX0_IRQHandler(void)
{
	CAN1_TX0_ISR_Handler();
}

/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
	USART1_ISR_Handler();
}

/**
  * @brief  This function handles USART2 interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
	USART2_ISR_Handler();
}

/**
  * @brief  This function handles USART3 interrupt request.
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void)
{
	USART3_ISR_Handler();
}

/**
  * @brief  This function handles EXTI0 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI0_IRQHandler(void)
{
	EXTI0_ISR_Handler();
}



/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/



/*******************************************************************************
* Function Name  : DMA1_Channel1_IRQHandler
* Description    : This function handles DMA1 Channel 1 interrupt request.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/
//void DMA1_Channel1_IRQHandler(void)
void DMA1_Stream0_IRQHandler(void)
{
  //Buffer[0] = 0x06;


    /* Write the descriptor through the endpoint
    //USB_SIL_Write(EP1_IN, (uint8_t*) Send_Buffer, 2);
    USBD_HID_SendReport (&USB_OTG_dev, Buffer, 2);
*/

  DMA_ClearITPendingBit(DMA1_Stream0, DMA_IT_TCIF0);

}

/**
  * @brief  This function handles FPU interrupt request.
  * @param  None
  * @retval None
  */
void FPU_IRQHandler(void)
{
	FPU_ISR_Handler();
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

