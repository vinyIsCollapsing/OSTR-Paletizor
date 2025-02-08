/*
 * main.c
 *
 *  Created on: 16 août 2018
 *      Author: Laurent
 */
#include "stm32f0xx.h"
#include "main.h"
#include "bsp.h"
#include "factory_io.h"
#include "delay.h"
#include "readTask.h"

/*
 * Local Static Functions
 */
static uint8_t SystemClock_Config(void);
void ActivateActuator(uint8_t bit);
void DeactivateActuator(uint8_t bit);
uint8_t IsSensorActive(uint8_t bit);

// Global variable to store the actuator states
static uint32_t actuator_state = 0;

// FreeRTOS tasks
void vTaskUp(void *pvParameters);

// FreeRTOS task handles
xTaskHandle vTaskUp_handle;

// Kernel objects
xSemaphoreHandle S_Up;  // Semaphore to control vTaskUp

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

    // Start Trace Recording
    xTraceEnable(TRC_START);

    // Create Semaphore
    S_Up = xSemaphoreCreateBinary();
    vTraceSetSemaphoreName(S_Up, "S_Up");

    // Initialize readTask and writeTask
    vTaskPubInit();  // readTask initialization
    // Create the vTaskUp task
    xTaskCreate(vTaskUp, "vTaskUp", 128, NULL, 1, &vTaskUp_handle);

    vTaskStartScheduler();

    // Loop forever
    while(1)
    {
        // The program should never be here...
    }
}

/*
 * Task_Up
 */
void vTaskUp(void *pvParameters)
{
	// Read all states from the scene
	FACTORY_IO_update();

	// Wait here for user button
	while(BSP_PB_GetState() == 0);

	//ActivateActuator(TAPISCARTONPALET);

	ActivateActuator(CARTON);
	ActivateActuator(BLOCAGE);
	while(1)
    {
		// while(FACTORY_IO_Sensors_Get(1 << CARTONDISTRIBUE) == 1);
		// Espera pelo evento no sensor CARTONDISTRIBUE (saída de estado 1)
		my_printf("Esperando o CARTON DISTRIBUE\r\n");
		subscribe(CARTONDISTRIBUE, CARTONDISTRIBUE, 1);  // Inscreve para ser notificado quando o sensor sair do estado 1
		xSemaphoreTake(sems[CARTONDISTRIBUE], portMAX_DELAY);
		ActivateActuator(TAPISCARTON);
		my_printf("CARTON DISTRIBUE liberado\r\n");

		// Espera pelo sensor CARTONENVOYE sair do estado 1 (fica 1 e deve mudar para 0)
		my_printf("Esperando mudança de estado do CARTONENVOYE para 0\r\n");
		subscribe(CARTONENVOYE, CARTONENVOYE, 1);  // Aguarda a transição de 1 para 0
		xSemaphoreTake(sems[CARTONENVOYE], portMAX_DELAY);
		ActivateActuator(TAPISCARTONPALET);
		my_printf("Primeira caixa\r\n");

		// Espera pelo sensor CARTONENVOYE entrar no estado 1 (ou seja, de 0 para 1)
		my_printf("Esperando mudança de estado do CARTONENVOYE para 1\r\n");
		subscribe(CARTONENVOYE, CARTONENVOYE, 0);  // Aguarda a transição de 0 para 1
		xSemaphoreTake(sems[CARTONENVOYE], portMAX_DELAY);
		my_printf("Primeira caixa livre\r\n");

		// Espera pelo sensor CARTONENVOYE sair do estado 1 para o caso da segunda caixa
		my_printf("Esperando mudança de estado do CARTONENVOYE para 0 (segunda caixa)\r\n");
		subscribe(CARTONENVOYE, CARTONENVOYE, 1);  // Aguarda a transição de 1 para 0
		xSemaphoreTake(sems[CARTONENVOYE], portMAX_DELAY);
		DeactivateActuator(CARTON);
		my_printf("Segunda caixa\r\n");

		// Espera pelo sensor CARTONENVOYE entrar no estado 1 (segunda caixa livre)
		my_printf("Esperando mudança de estado do CARTONENVOYE para 1 (segunda caixa livre)\r\n");
		subscribe(CARTONENVOYE, CARTONENVOYE, 0);  // Aguarda a transição de 0 para 1
		xSemaphoreTake(sems[CARTONENVOYE], portMAX_DELAY);
		my_printf("Segunda caixa livre\r\n");


		// Espera pelo sensor ENTREEPALETIZOR sair do estado 1 (primeira caixa no bloqueado)
		my_printf("Esperando mudança de estado do ENTREEPALETIZOR para 0 (primeira caixa no bloqueado)\r\n");
		subscribe(ENTREEPALETIZOR, ENTREEPALETIZOR, 1);
		xSemaphoreTake(sems[ENTREEPALETIZOR], portMAX_DELAY);
		my_printf("Primeira caixa no bloqueado\r\n");

		// Espera pelo sensor ENTREEPALETIZOR entrar no estado 1 (primeira caixa no bloqueado esperando segunda caixa)
		my_printf("Esperando mudança de estado do ENTREEPALETIZOR para 1 (primeira caixa no bloqueado esperando segunda caixa)\r\n");
		subscribe(ENTREEPALETIZOR, ENTREEPALETIZOR, 0);
		xSemaphoreTake(sems[ENTREEPALETIZOR], portMAX_DELAY);
		my_printf("Primeira caixa no bloqueado esperando segunda caixa\r\n");

		// Espera pelo sensor ENTREEPALETIZOR sair do estado 1 (segunda caixa no bloqueado)
		my_printf("Esperando mudança de estado do ENTREEPALETIZOR para 0 (segunda caixa no bloqueado)\r\n");
		subscribe(ENTREEPALETIZOR, ENTREEPALETIZOR, 1);
		xSemaphoreTake(sems[ENTREEPALETIZOR], portMAX_DELAY);
		my_printf("Segunda caixa no bloqueado\r\n");

		// Após a notificação, procede com as ações correspondentes
		DeactivateActuator(BLOCAGE);
		my_printf("Barreira abaixada\r\n");
		ActivateActuator(CHARGERPALET);

		// Espera pelo sensor ENTREEPALETIZOR entrar no estado 1 (segunda caixa fora bloqueado)
		my_printf("Esperando mudança de estado do ENTREEPALETIZOR para 1 (segunda caixa fora bloqueado)\r\n");
		subscribe(ENTREEPALETIZOR, ENTREEPALETIZOR, 0);
		xSemaphoreTake(sems[ENTREEPALETIZOR], portMAX_DELAY);
		my_printf("Segunda caixa fora bloqueado\r\n");

		vTaskDelay(750);
		ActivateActuator(BLOCAGE);
		my_printf("Barreira levantada\r\n");

		my_printf("Duas caixas no pistao\r\n");
		vTaskDelay(600);
		ActivateActuator(POUSSOIR);
		my_printf("Pistao empurra\r\n");

		// Espera pelo sensor LIMITEPOUSSOIR sair do estado 1 (pistão atinge o final)
		my_printf("Esperando mudança de estado do LIMITEPOUSSOIR para 0 (pistao no final)\r\n");
		subscribe(LIMITEPOUSSOIR, LIMITEPOUSSOIR, 1);
		xSemaphoreTake(sems[LIMITEPOUSSOIR], portMAX_DELAY);
		my_printf("Pistao no final\r\n");

		// Espera pelo sensor LIMITEPOUSSOIR entrar no estado 1 (pistão volta ao início)
		my_printf("Esperando mudança de estado do LIMITEPOUSSOIR para 1 (pistao no inicio)\r\n");
		subscribe(LIMITEPOUSSOIR, LIMITEPOUSSOIR, 0);
		xSemaphoreTake(sems[LIMITEPOUSSOIR], portMAX_DELAY);
		my_printf("Pistao no inicio\r\n");

		DeactivateActuator(POUSSOIR);
		ActivateActuator(CARTON);

    }
}

// Function to activate a specific actuator without modifying others
void ActivateActuator(uint8_t bit)
{
    actuator_state |= (1 << bit); // Set the specific bit to 1
    FACTORY_IO_Actuators_Set(actuator_state);
}

// Function to deactivate a specific actuator without modifying others
void DeactivateActuator(uint8_t bit)
{
    actuator_state &= ~(1 << bit); // Set the specific bit to 0
    FACTORY_IO_Actuators_Set(actuator_state);
}

// Function to check if a specific sensor is active (returns 1 if active, 0 if inactive)
uint8_t IsSensorActive(uint8_t bit)
{
    return FACTORY_IO_Sensors_Get(1 << bit);
}

/*
 * Clock configuration for the Nucleo STM32F072RB board
 */
static uint8_t SystemClock_Config()
{
    uint32_t status;
    uint32_t timeout;

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

    if (timeout == 0) return (1); // HSE error

    // Select HSE as PLL input source
    RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
    RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);

    // Set PLL PREDIV to /1
    RCC->CFGR2 = 0x00000000;

    // Set PLL MUL to x6
    RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
    RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);

    // Enable the main PLL
    RCC->CR |= RCC_CR_PLLON;

    // Wait until PLL is ready
    timeout = 1000;

    do
    {
        status = RCC->CR & RCC_CR_PLLRDY_Msk;
        timeout--;
    } while ((status == 0) && (timeout > 0));

    if (timeout == 0) return (2); // PLL error

    // Set AHB prescaler to /1
    RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

    // Set APB1 prescaler to /1
    RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
    RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

    // Enable FLASH Prefetch Buffer and set Flash Latency
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

    if (timeout == 0) return (3); // SW error

    // Update SystemCoreClock global variable
    SystemCoreClockUpdate();
    return (0);
}

/*
 * Assertion Handler
 */
void vAssertCalled(char *file, int line)
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
