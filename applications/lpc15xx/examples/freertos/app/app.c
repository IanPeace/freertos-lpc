/*
 * @brief Blinky example using timers and sysTick
 *
 * @note
 * Copyright(C) NXP Semiconductors, 2013
 * All rights reserved.
 *
 * @par
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * LPC products.  This software is supplied "AS IS" without any warranties of
 * any kind, and NXP Semiconductors and its licensor disclaim any and
 * all warranties, express or implied, including all implied warranties of
 * merchantability, fitness for a particular purpose and non-infringement of
 * intellectual property rights.  NXP Semiconductors assumes no responsibility
 * or liability for the use of the software, conveys no license or rights under any
 * patent, copyright, mask work right, or any other intellectual property rights in
 * or to any products. NXP Semiconductors reserves the right to make changes
 * in the software without notification. NXP Semiconductors also makes no
 * representation or warranty that such application will be suitable for the
 * specified use without further testing or modification.
 *
 * @par
 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors' and its
 * licensor's relevant copyrights in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 */

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"
#include <bsp.h>
#include <app_arduino.h>

extern void vUARTCommandConsoleStart( uint16_t usStackSize, UBaseType_t uxPriority );
extern void vRegisterSampleCLICommands( void );

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/

/*****************************************************************************
 * Private functions
 ****************************************************************************/

/* Sets up system hardware */
static void prvSetupHardware(void)
{
	SystemCoreClockUpdate();
	Board_Init();

	/* Initial LED0 state is off */
	Board_LED_Set(LED_RED, false);
	Board_LED_Set(LED_GREEN, false);
	Board_LED_Set(LED_BLUE, false);

    /* Initial Serial console */
    BSP_Ser_Init(115200);
}

/* LED1 toggle thread */
static void vLEDTask(void *pvParameters) {
	bool LedState = false;

	while (1) {
		Board_LED_Set(LED_BLUE, LedState);
		LedState = (bool) !LedState;

		/* About a 1Hz on/off toggle rate */
		vTaskDelay(configTICK_RATE_HZ / 2);
	}
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

/**
 * @brief	main routine for FreeRTOS blinky example
 * @return	Nothing, function should not exit
 */
int main(void)
{
	prvSetupHardware();

	/* LED1 toggle thread */
	xTaskCreate(vLEDTask, "vTaskLed",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 3UL),
				(TaskHandle_t *) NULL);

	/* Start the tasks that implements the command console on the UART, as
	described above. */
	vUARTCommandConsoleStart(configMINIMAL_STACK_SIZE * 2, (tskIDLE_PRIORITY + 2UL));

	/* Register the standard CLI commands. */
	vRegisterSampleCLICommands();

	/* arduino thread */
	xTaskCreate(vArduinoTask, "vTaskArduino",
				configMINIMAL_STACK_SIZE << 1, NULL, (tskIDLE_PRIORITY + 1UL),
				(TaskHandle_t *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Should never arrive here */
	return 1;
}
