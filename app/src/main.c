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
BaseType_t writing(uint8_t on_off, uint8_t actuator);

// Global variable to store the actuator states
static uint32_t actuator_state = 0;
typedef struct
{
	uint8_t on_off; 		// Turn On or Off
	uint8_t actuator;		// The desired actuator
} commande_message_t;

// FreeRTOS tasks
void vTaskUp(void *pvParameters);
void vTaskUpDos(void *pvParameters);
// void vTaskMiddle (void *pvParameters);
void vTask_Palette(void *pvParameters);
void vTask_Write(void *pvParameters);

// FreeRTOS task handles
// xTaskHandle vTaskUp_handle;
// xTaskHandle vTaskUpDos_handle;
// xTaskHandle vTaskMiddle_handle;

xQueueHandle	xCommandeQueue;
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
    xCommandeQueue = xQueueCreate(MAX_SUBSCRIBERS, sizeof(commande_message_t));

    // Initialize readTask and writeTask
    vTaskPubInit();  // readTask initialization
    xTaskCreate(vTask_Write,   "Task_Write",   128, NULL, 1, NULL);
    // Create the vTaskUp task
    xTaskCreate(vTaskUp, "vTaskUp", 128, NULL, 1, NULL);
    xTaskCreate(vTaskUpDos, "vTaskUpDos", 128, NULL, 1, NULL);
    // xTaskCreate(vTaskMiddle, "vTaskMiddle", 128, NULL, 1, &vTaskMiddle_handle);
    xTaskCreate(vTask_Palette, "vTask_Palette", 128, NULL, 1, NULL);

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
	// Wait here for user button
	// while(BSP_PB_GetState() == 0);
	// Read all states from the scene
	FACTORY_IO_update();
	// ActivateActuator(BLOCAGE);
	writing(1, CARTON);
	writing(1, BLOCAGE);
	//ActivateActuator(CARTON);
	while(1)
    {

		// while(FACTORY_IO_Sensors_Get(1 << CARTONDISTRIBUE) == 1);
		// Espera pelo evento no sensor CARTONDISTRIBUE (saída de estado 1)
		my_printf("Esperando o CARTON DISTRIBUE\r\n");
		subscribe(CARTONDISTRIBUE, CARTONDISTRIBUE, 1);  // Inscreve para ser notificado quando o sensor sair do estado 1
		xSemaphoreTake(sems[CARTONDISTRIBUE], portMAX_DELAY);
		writing(1, TAPISCARTON);
		my_printf("CARTON DISTRIBUE liberado\r\n");

		// Espera pelo sensor CARTONENVOYE sair do estado 1 (fica 1 e deve mudar para 0)
		my_printf("Esperando mudança de estado do CARTONENVOYE para 0\r\n");
		subscribe(CARTONENVOYE, CARTONENVOYE, 1);  // Aguarda a transição de 1 para 0
		xSemaphoreTake(sems[CARTONENVOYE], portMAX_DELAY);
		writing(1, TAPISCARTONPALET);
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
		writing(0, CARTON);
		my_printf("Segunda caixa\r\n");

		// Espera pelo sensor CARTONENVOYE entrar no estado 1 (segunda caixa livre)
		my_printf("Esperando mudança de estado do CARTONENVOYE para 1 (segunda caixa livre)\r\n");
		subscribe(CARTONENVOYE, CARTONENVOYE, 0);  // Aguarda a transição de 0 para 1
		xSemaphoreTake(sems[CARTONENVOYE], portMAX_DELAY);
		my_printf("Segunda caixa livre\r\n");

    }
}


void vTaskUpDos(void *pvParameters){

	while(1){
		// Espera pelo sensor ENTREEPALETIZOR sair do estado 1 (primeira caixa no bloqueado)
		my_printf("Esperando mudanca de estado do ENTREEPALETIZOR para 0 (primeira caixa no bloqueado)\r\n");
		subscribe(ENTREEPALETIZOR, ENTREEPALETIZOR, 1);
		xSemaphoreTake(sems[ENTREEPALETIZOR], portMAX_DELAY);
		my_printf("Primeira caixa no bloqueado\r\n");

		// Espera pelo sensor ENTREEPALETIZOR entrar no estado 1 (primeira caixa no bloqueado esperando segunda caixa)
		my_printf("Esperando mudanca de estado do ENTREEPALETIZOR para 1 (primeira caixa no bloqueado esperando segunda caixa)\r\n");
		subscribe(ENTREEPALETIZOR, ENTREEPALETIZOR, 0);
		xSemaphoreTake(sems[ENTREEPALETIZOR], portMAX_DELAY);
		my_printf("Primeira caixa no bloqueado esperando segunda caixa\r\n");

		// Espera pelo sensor ENTREEPALETIZOR sair do estado 1 (segunda caixa no bloqueado)
		my_printf("Esperando mudanca de estado do ENTREEPALETIZOR para 0 (segunda caixa no bloqueado)\r\n");
		subscribe(ENTREEPALETIZOR, ENTREEPALETIZOR, 1);
		xSemaphoreTake(sems[ENTREEPALETIZOR], portMAX_DELAY);
		my_printf("Segunda caixa no bloqueado\r\n");

		// Após a notificação, procede com as ações correspondentes
		my_printf("Desativando a barreira\r\n");
		writing(0, BLOCAGE);
		my_printf("Barreira abaixada\r\n");
		ActivateActuator(CHARGERPALET);

		// Espera pelo sensor ENTREEPALETIZOR entrar no estado 1 (segunda caixa fora bloqueado)
		my_printf("Esperando mudança de estado do ENTREEPALETIZOR para 1 (segunda caixa fora bloqueado)\r\n");
		subscribe(ENTREEPALETIZOR, ENTREEPALETIZOR, 0);
		xSemaphoreTake(sems[ENTREEPALETIZOR], portMAX_DELAY);
		my_printf("Segunda caixa fora bloqueado\r\n");

		vTaskDelay(750);
		writing(1, BLOCAGE);
		my_printf("Barreira levantada\r\n");

		my_printf("Duas caixas no pistao\r\n");
		vTaskDelay(600);
		writing(1, POUSSOIR);

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

		writing(0, POUSSOIR);
		writing(1, CARTON);
	}
}

void vTask_Palette (void *pvParameters)
{
	// Read all states from the scene
	FACTORY_IO_update();

	while(1)
	{
		writing(1, distribuition_palette);
		writing(1, tapis_distribuition_palette);
		writing(1, tapis_palette_vers_ascenseur);

		vTaskDelay(300);

		writing(0, distribuition_palette);

		subscribe(1, entree_palette, 0);
		xSemaphoreTake(sems[entree_palette], portMAX_DELAY);

		writing(0, tapis_distribuition_palette);
		writing(0, tapis_palette_vers_ascenseur);
		writing(1, charger_palette);

		subscribe(1, sortie_palette, 0);
		xSemaphoreTake(sems[sortie_palette], portMAX_DELAY);


		writing(0, charger_palette);
		writing(1, monter_ascenseur);
		writing(1, ascenseur_to_limit);

		subscribe(1, ascenseur_etage_1, 0);
		xSemaphoreTake(sems[ascenseur_etage_1], portMAX_DELAY);

		writing(0, monter_ascenseur);
		writing(0, ascenseur_to_limit);

		//while(IsSensorActive(porte_ouverte)==1);
		while(BSP_PB_GetState() == 0);

		writing(1, descendre_ascenseur);

		//while(IsSensorActive(porte_ouverte)==1);
		while(BSP_PB_GetState() == 1);

		writing(1, ascenseur_to_limit);

		subscribe(1, ascenseur_etage_rdc, 0);
		xSemaphoreTake(sems[ascenseur_etage_rdc], portMAX_DELAY);

		writing(0, descendre_ascenseur);

		writing(0, ascenseur_to_limit);
		writing(1, charger_palette);
		writing(1, tapis_fin);
	}
}


/*
void vTaskMiddle (void *pvParameters)
{
	// Read all states from the scene
	FACTORY_IO_update();
	while(1)
	{
		subscribe(LIMITEPOUSSOIR, LIMITEPOUSSOIR, 0);
		xSemaphoreTake(sems[LIMITEPOUSSOIR], portMAX_DELAY);
		// while(IsSensorActive(limite_poussoir)==0);
		DeactivateActuator(poussoir);
		ActivateActuator(clamp);
		while(IsSensorActive(clamped)==0);
		DeactivateActuator(clamp);

		ActivateActuator(porte);
		while(IsSensorActive(ascenseur_en_mouvement)==0);
		DeactivateActuator(porte);

		while(IsSensorActive(limite_porte)==1);
	}
}
*/

void vTask_Write (void *pvParameters)
{
	commande_message_t actuator;

	while(1)
	{
		// Wait for something in the message Queue
		xQueueReceive(xCommandeQueue, &actuator, portMAX_DELAY);

		if(actuator.on_off == 1) {
			// Activate the actuator
			ActivateActuator(actuator.actuator);
		}

		if(actuator.on_off == 0) {
			// Deactivate the actuator
			DeactivateActuator(actuator.actuator);
		}
	}
}

BaseType_t writing(uint8_t on_off, uint8_t actuator)
{
	commande_message_t data = {
		.on_off = on_off,
		.actuator = actuator
	};

	return xQueueSend(xCommandeQueue, &data, 0);
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
