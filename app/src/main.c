/*
 * main.c
 *
 *  Created on: 16 ao�t 2018
 *      Author: Laurent
 */

#include "stm32f0xx.h"
#include "main.h"
#include "bsp.h"
#include "factory_io.h"
#include "delay.h"


/*
 * Local Static Functions
 */

static uint8_t 	SystemClock_Config	(void);


/*
 * Project Entry Point
 */

int main(void)
{
	// Configure System Clock for 48MHz from 8MHz HSE
	SystemClock_Config();

	// Initialize LED and USER Button
	BSP_LED_Init();
	BSP_PB_Init();

	// Initialize Debug Console
	BSP_Console_Init();
	my_printf("\r\nConsole Ready!\r\n");
	my_printf("SYSCLK = %d Hz\r\n", SystemCoreClock);

	// Read all states from the scene
	FACTORY_IO_update();

	// Wait here for user button
	while(BSP_PB_GetState() == 0);

	// Start conveyor A[0] = 1
	my_printf("Starting Conveyor\r\n");
	FACTORY_IO_Actuators_Set(0b0000000000000000000000000001);

	// Wait for sensor S[1] = 0 (optical barrier)
	my_printf("Waiting for sensor...\r\n");
	while (FACTORY_IO_Sensors_Get(0b0000000000000000000000000010) == 1);

	// Stop conveyor A[0] = 0
	my_printf("Stop!\r\n");
	FACTORY_IO_Actuators_Set(0x00000000);

	// Loop forever
	while(1)
	{
		// LED blinking
		BSP_LED_Toggle();
		delay_ms(100);
	}
}



/*
 * 	Clock configuration for the Nucleo STM32F072RB board
 * 	HSE input Bypass Mode 			-> 8MHz
 * 	SYSCLK, AHB, APB1 				-> 48MHz
 *  PA8 as MCO with /16 prescaler 	-> 3MHz
 */

static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout;

	// Start HSE in Bypass Mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1);	// HSE error


	// Select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);

	// Set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;

	// Set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2);	// PLL error


	// Set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	//Set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

	// Enable FLASH Prefetch Buffer and set Flash Latency (required for high speed)
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL becomes main switch input
	timeout = 1000;

	do
	{
		status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (3);	// SW error


	// Set MCO source as SYSCLK (48MHz)
	RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOSEL_SYSCLK;

	// Set MCO prescaler to /16 -> 3MHz
	RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOPRE_DIV16;

	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA8 as Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER8_Pos);

	// Set to AF0 (MCO output)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000000);

	// Update SystemCoreClock global variable
	SystemCoreClockUpdate();
	return (0);
}

/*
 * Assertion Handler
 */
void vAssertCalled( char *file, int line )
{
	taskDISABLE_INTERRUPTS();

	my_printf("Assertion Failed\r\n");
	my_printf("File %s\r\n", file);
	my_printf("Line %d\r\n", line);

	while(1);
}


/*
 * Malloc failed Basic Hook
 */
void vApplicationMallocFailedHook()
{
	my_printf("Malloc Failed\r\n");

	while(1);
}
