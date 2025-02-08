/*
 * main.h
 *
 *  Created on: Dec 24, 2023
 *      Author: Laurent
 */

#ifndef INC_MAIN_H_
#define INC_MAIN_H_

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
#include "stream_buffer.h"

// Actuators
#define CARTON              0
#define TAPISCARTON         1
#define BLOCAGE             2
#define POUSSOIR			4
#define TAPISCARTONPALET    12
#define CHARGERPALET		16

// Sensors
#define CARTONDISTRIBUE     0
#define CARTONENVOYE        1
#define ENTREEPALETIZOR     2
#define LIMITEPOUSSOIR		4

// Global functions
int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);

#endif /* INC_MAIN_H_ */
