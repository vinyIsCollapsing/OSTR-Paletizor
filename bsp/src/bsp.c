/*
 * bsp.c
 *
 *  Created on: 5 ao�t 2017
 *      Author: Laurent
 */

#include "bsp.h"
// #include "factory_io.h"

uint8_t tx_dma_buffer[FRAME_LENGTH];

/*
 * BSP_LED_Init()
 * Initialize LED pin (PA5) as  High-Speed Push-Pull Output
 * Set LED initial state to OFF
 */

void BSP_LED_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA5 as output
	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER5_Pos);

	// Configure PA5 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;

	// Configure PA5 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR5_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEEDR5_Pos);

	// Disable PA5 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5_Msk;

	// Set Initial State OFF
	GPIOA->BSRR = GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_On()
 * Turn ON LED on PA5
 */

void BSP_LED_On()
{
	GPIOA->BSRR = GPIO_BSRR_BS_5;
}

/*
 * BSP_LED_Off()
 * Turn OFF LED on PA5
 */

void BSP_LED_Off()
{
	GPIOA->BSRR = GPIO_BSRR_BR_5;
}

/*
 * BSP_LED_Toggle()
 * Toggle LED on PA5
 */

void BSP_LED_Toggle()
{
	GPIOA->ODR ^= GPIO_ODR_5;
}

/*
 * BSP_PB_Init()
 * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
 */

void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODER13_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13_Msk;
}

/*
 * BSP_PB_GetState()
 * Returns the state of the button (0=released, 1=pressed)
 */

uint8_t	BSP_PB_GetState()
{
	uint8_t state;

	if ((GPIOC->IDR & GPIO_IDR_13) == GPIO_IDR_13)
	{
		state = 0;
	}
	else
	{
		state = 1;
	}

	return state;
}

/*
 * BSP_Console_Init()
 * USART2 @ 115200 Full Duplex
 * 1 start - 8-bit - 1 stop
 * TX -> PA2 (AF1)
 * RX -> PA3 (AF1)
 */

void BSP_Console_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
	GPIOA->MODER |=  (0x02 <<GPIO_MODER_MODER2_Pos) | (0x02 <<GPIO_MODER_MODER3_Pos);

	// Set PA2 and PA3 to AF1 (USART2)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00001100);

	// Enable USART2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART2->CR1 = 0x00000000;
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CFGR3 &= ~RCC_CFGR3_USART2SW_Msk;

	// Baud Rate = 115200
	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
	// BRR = 417 -> Actual BaudRate = 115107.9137 -> 0.08% error
	//
	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
	// BRR = 833 -> Actual BaudRate = 115246.0984 -> 0.04% error (better choice)

	USART2->CR1 |= USART_CR1_OVER8;
	USART2->BRR = 833;

	// Enable both Transmitter and Receiver
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;


	// Setur TX on DMA Channel 4
	// Enable TX (TC) interrupt
	USART2->CR1 |= USART_CR1_TCIE;

	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;

	// Setup TX on DMA Channel 4
	// Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;
	// Reset DMA1 Channel 4 configuration
	DMA1_Channel4->CCR = 0x00000000;
	// Set direction Memory -> Peripheral
	DMA1_Channel4->CCR |= DMA_CCR_DIR;
	// Peripheral is USART2 TDR
	DMA1_Channel4->CPAR = (uint32_t) &USART2->TDR;
	// Peripheral data size is 8-bit (byte)
	DMA1_Channel4->CCR |= (0x00 <<DMA_CCR_PSIZE_Pos);
	// Disable auto-increment Peripheral address
	DMA1_Channel4->CCR &= ~DMA_CCR_PINC;
	// Memory is tx_dma_buffer
	DMA1_Channel4->CMAR = (uint32_t) tx_dma_buffer;
	// Memory data size is 8-bit (byte)
	DMA1_Channel4->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);
	// Enable auto-increment Memory address
	DMA1_Channel4->CCR |= DMA_CCR_MINC;
	// Set Memory Buffer size
	DMA1_Channel4->CNDTR = 0;
	// DMA mode is not circular
	DMA1_Channel4->CCR &= ~DMA_CCR_CIRC;
	// Enable transfer complete interrupt
	DMA1_Channel4->CCR |= DMA_CCR_TCIE;
	// Enable DMA1 Channel 4
	//DMA1_Channel4->CCR |= DMA_CCR_EN;

	// Setup RX on DMA Channel 5

	// Start DMA clock
	RCC->AHBENR |= RCC_AHBENR_DMA1EN;

	// Reset DMA1 Channel 5 configuration
	DMA1_Channel5->CCR = 0x00000000;

	// Set direction Peripheral -> Memory
	DMA1_Channel5->CCR &= ~DMA_CCR_DIR;

	// Peripheral is USART2 RDR
	DMA1_Channel5->CPAR = (uint32_t)&USART2->RDR;

	// Peripheral data size is 8-bit (byte)
	DMA1_Channel5->CCR |= (0x00 <<DMA_CCR_PSIZE_Pos);

	// Disable auto-increment Peripheral address
	DMA1_Channel5->CCR &= ~DMA_CCR_PINC;

	// Memory is rx_dma_buffer
	DMA1_Channel5->CMAR = (uint32_t)rx_dma_buffer;

	// Memory data size is 8-bit (byte)
	DMA1_Channel5->CCR |= (0x00 <<DMA_CCR_MSIZE_Pos);

	// Enable auto-increment Memory address
	DMA1_Channel5->CCR |= DMA_CCR_MINC;

	// Set Memory Buffer size
	DMA1_Channel5->CNDTR = FRAME_LENGTH;

	// DMA mode is circular
	DMA1_Channel5->CCR |= DMA_CCR_CIRC;

	// Enable DMA1 Channel 5
	DMA1_Channel5->CCR |= DMA_CCR_EN;

	// Enable USART2 DMA Request on RX
	USART2->CR3 |= USART_CR3_DMAR;

	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;
}


