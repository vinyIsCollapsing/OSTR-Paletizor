/*
 * readTask.h
 *
 *  Created on: Jan 13, 2025
 *      Author: vinic
 */

#ifndef INC_READTASK_H_
#define INC_READTASK_H_

#define MAX_SUBSCRIBERS 12
#define QUEUE_LENGTH 10
#define SENSOR_TABLE_SIZE 12
#define MAX_SEMAPHORE 12

#define SUBSCRIPTION_EMPTY 0xFF		// Correcao para o caso do sensor 0

#include <stddef.h>
#include <stdint.h>
// Device header
#include "stm32f0xx.h"
// BSP functions
#include "bsp.h"
// FreeRTOS headers
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "factory_io.h"

#include "printf-stdarg.h"

void vTask1_Pub();
void vTask2_Pub();
void vTask_Pub(void *pvParameters);

typedef struct {
    uint8_t sem_id;        // Semaphore ID to use for publication
    uint8_t sensor_id;     // Awaited sensor ID
    uint8_t sensor_state;  // Awaited sensor state
} subscribe_message_t;

BaseType_t subscribe(uint8_t sem_id, uint8_t sensor_id, uint8_t sensor_state);
BaseType_t vTaskPubInit();

extern xSemaphoreHandle sems[MAX_SEMAPHORE];

#endif
