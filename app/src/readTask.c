/*
 * readTask.c
 *
 *  Created on: Jan 13, 2025
 *      Author: vinic
 */
#include "main.h"
#include "readTask.h"

static xTaskHandle vTaskPub_handle;
static xQueueHandle xSubscribeQueue;

xSemaphoreHandle sems[MAX_SEMAPHORE];

uint8_t sensor_states[SENSOR_TABLE_SIZE] = {0};
extern uint8_t	rx_dma_buffer[FRAME_LENGTH];

static void updateSubs(subscribe_message_t *subs, subscribe_message_t *new_sub);
static void print_subscription_table(subscribe_message_t *subs);
static void publish(subscribe_message_t *subs);



BaseType_t vTaskPubInit(){
	size_t i;

    // Create the subscription queue
    xSubscribeQueue = xQueueCreate(QUEUE_LENGTH, sizeof(subscribe_message_t));

	for(i = 0; i < MAX_SEMAPHORE; i++) {
		sems[i] = xSemaphoreCreateBinary();
	}

    xTaskCreate(vTask_Pub, "vTask_Pub", 128, NULL, 1, &vTaskPub_handle);

    my_printf("READ TASK DEFINED\r\n");


    return pdPASS;
}

void vTask_Pub(void *pvParameters){
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 200 / portTICK_PERIOD_MS;

	xLastWakeTime = xTaskGetTickCount();

	subscribe_message_t subscription_table[MAX_SUBSCRIBERS] = {0};
	subscribe_message_t msg;
	size_t i;
	// char rx_byte;

	// Reseting the message
	for(i = 0; i < MAX_SUBSCRIBERS; i++) {
	 	subscription_table[i].sem_id = SUBSCRIPTION_EMPTY;
	   	subscription_table[i].sensor_id = SUBSCRIPTION_EMPTY;
	   	subscription_table[i].sensor_state = SUBSCRIPTION_EMPTY;
	}

	for(i = 0; i < SENSOR_TABLE_SIZE; i++) {
		sensor_states[i] = 0;
	}
	// BSP_LED_Toggle();

	while (1) {
	  	if(xQueueReceive(xSubscribeQueue, &msg, 0)){
	  		updateSubs(subscription_table, &msg);
	   		print_subscription_table(subscription_table);
	   	}

		publish(subscription_table);

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
}

BaseType_t subscribe(uint8_t sem_id, uint8_t sensor_id, uint8_t sensor_state)
{
	subscribe_message_t data = {
		.sem_id = sem_id,
		.sensor_id = sensor_id,
		.sensor_state = sensor_state
	};

	return xQueueSend(xSubscribeQueue, &data, 0);
}


static void updateSubs(subscribe_message_t *subs, subscribe_message_t *new_sub) {
    size_t i;

    my_printf("Subscribing...");

    // Check for duplicates
    /*
    for (i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (subs[i].sem_id == new_sub->sem_id &&
            subs[i].sensor_id == new_sub->sensor_id &&
            subs[i].sensor_state == new_sub->sensor_state) {
            my_printf("Subscription already exists\r\n");
            return;
        }
    }
    */

    // Add the new subscription to the first available slot
    for (i = 0; i < MAX_SUBSCRIBERS; i++) {
        if (subs[i].sensor_id == SUBSCRIPTION_EMPTY) {
            subs[i] = *new_sub;
            my_printf("Adding subscription in slot [%d]\r\n", i);
            return;
        }
    }

    // If the table is full
    my_printf("No available slots for new subscription\r\n");
}

static void print_subscription_table(subscribe_message_t *subs) {
    for (int i = 0; i < MAX_SUBSCRIBERS; i++) {
        my_printf("[%d] %d %d %d\r\n", i, subs[i].sem_id, subs[i].sensor_id, subs[i].sensor_state);
    }
}


static void publish(subscribe_message_t *subs)
{
    size_t i;

    // Percorre todos os slots da tabela de inscrições
    for (i = 0; i < MAX_SUBSCRIBERS; i++) {
        // Se o slot estiver vazio, pula para o próximo
        if (subs[i].sem_id == SUBSCRIPTION_EMPTY)
            continue;

        uint8_t sensor_id = subs[i].sensor_id;
        // Obtém o estado atual do sensor usando a máscara (1 << sensor_id)
        uint8_t current_state = FACTORY_IO_Sensors_Get(1 << sensor_id);

        // Se o sensor ainda estiver no estado esperado, passa para o próximo slot
        if (current_state == subs[i].sensor_state)
            continue;

        // Se houve mudança, notifica o assinante liberando o semáforo correspondente
        xSemaphoreGive(sems[subs[i].sem_id]);
        my_printf("Published to subscription in slot [%d]\r\n", i);

        // Remove a inscrição, liberando o slot para uma nova assinatura
        subs[i].sem_id = SUBSCRIPTION_EMPTY;
        subs[i].sensor_id = SUBSCRIPTION_EMPTY;
        subs[i].sensor_state = SUBSCRIPTION_EMPTY;
    }
}
