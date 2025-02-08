/**
  ******************************************************************************
  * @file    Templates/Src/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"


/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/
/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}


/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//  // HAL_IncTick();
//}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/


/**
  * This function handles EXTI line 13 interrupt request.
  */


extern xSemaphoreHandle xSem_UART_TC;

void USART2_IRQHandler(){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	// Test for TC pending interrupt
	if ( USART2->ISR & USART_ISR_TC ) {
		// Clear pending bit by writing a '1'
		// USART2->CR1 = USART_ISR_TCIE;
		USART2->ICR |= USART_ICR_TCCF;
		// Release the semaphore
		xSemaphoreGiveFromISR(xSem_UART_TC, &xHigherPriorityTaskWoken);
		// Perform a context switch to the waiting task
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}
extern xSemaphoreHandle	xSem_DMA_TC;
void DMA1_Channel4_5_6_7_IRQHandler()
{
	/*
	// Test for Channel 5 Half Transfer
	if ((DMA1->ISR & DMA_ISR_HTIF5) == DMA_ISR_HTIF5)
	{
		// Clear the interrupt pending bit
		DMA1->IFCR |= DMA_IFCR_CHTIF5;
		// Set global variable
		xSem_DMA_TC = 1;
	}
	// Test for Channel 5 Transfer Complete
	if ((DMA1->ISR & DMA_ISR_TCIF5) == DMA_ISR_TCIF5)
	{
		// Clear the interrupt pending bit
		DMA1->IFCR |= DMA_IFCR_CTCIF5;
		// Set global variable
		xSem_DMA_TC = 2;
	}
	*/
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	// Test for TC pending interrupt
	if ( DMA1->ISR & DMA_ISR_TCIF4 ) {
		// Clear pending bit by writing a '1'
		// USART2->CR1 = USART_ISR_TCIE;
		DMA1->IFCR |= DMA_IFCR_CTCIF4;
		// Release the semaphore
		xSemaphoreGiveFromISR(xSem_DMA_TC, &xHigherPriorityTaskWoken);
		// Perform a context switch to the waiting task
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
