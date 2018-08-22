/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                              uC/DHCP-c
*                                           APPLICATION CODE
*
* Filename      : app_sail.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               ENABLE
*********************************************************************************************************
*/


#include <stdio.h>
#include <string.h>

#include <math.h>

#include "board.h"
#include "FreeRTOS.h"
#include "task.h"

#include <bsp.h>
#include <drv_lm75b.h>
#include <drv_ssd1306.h>
#include <app_arduino.h>

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

#if 1

/*
*********************************************************************************************************
*                                          FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             vArduinoTask()
*
* Description : This task monitors the state of the sail
*
* Argument(s) : p_arg   is the argument passed to 'AppKeyTask()' by 'AppTaskCreate()'.
*
* Return(s)  : none.
*
* Caller(s)  : This is a task.
*
* Note(s)    : none.
*********************************************************************************************************
*/

void  vArduinoTask (void *p_arg)
{
	int16_t temp;

    BSP_STLM75_Init();

    ssd1306_init();

	while (1) {
		BSP_STLM75_TempGet(BSP_STLM75_TEMP_UNIT_CELSIUS, &temp);
        //BSP_Ser_Printf("Temperature: %d\r\n", temp);

        ssd1306_clear();
        BSP_SSD1306_Printf("Temperature: %d", temp);

		/* Read the temperature 1Hz rate */
		vTaskDelay(configTICK_RATE_HZ << 1);
	}
}

#endif  /* (APP_CFG_SHELL_EN == DEF_ENABLED) */

