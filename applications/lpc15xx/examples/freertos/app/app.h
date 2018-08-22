/*
*********************************************************************************************************
*                                              uC/OS-III
*                                        The Real-Time Kernel
*
*                          (c) Copyright 2004-2010; Micrium, Inc.; Weston, FL               
*
*               All rights reserved.  Protected by international copyright laws.
*********************************************************************************************************
*/
#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT
/*
*********************************************************************************************************
*                                            INCLUDES
*********************************************************************************************************
*/
#include <stdbool.h>
#include <stddef.h>

#include <bsp.h>

/*
*********************************************************************************************************
*                                       ADDITIONAL uC/MODULE ENABLES
*********************************************************************************************************
*/

#define  APP_CFG_PRINTF_EN                          DEF_DISABLED
#define  APP_CFG_SHELL_EN                           DEF_ENABLED
#define  APP_CFG_GUI_EN                             DEF_DISABLED

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_TASK_SHELL_PRIO                        3u

/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*********************************************************************************************************
*/

#define  APP_TASK_SHELL_STK_SIZE                    128u

/*
*********************************************************************************************************
*                                          SERIAL CONFIGURATION
*********************************************************************************************************
*/

#define  APP_CFG_SERIAL_EN                          DEF_ENABLED
#define  BSP_CFG_SER_COMM_SEL                       BSP_UART1

/*
*********************************************************************************************************
*                                       TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/
#ifndef  TRACE_LEVEL_OFF
#define  TRACE_LEVEL_OFF                               0
#endif

#ifndef  TRACE_LEVEL_INFO
#define  TRACE_LEVEL_INFO                              1
#endif

#ifndef  TRACE_LEVEL_DBG
#define  TRACE_LEVEL_DBG                               2
#endif

#if (APP_CFG_PRINTF_EN == DEF_ENABLED)
#define  APP_CFG_TRACE                              printf
#define  BSP_CFG_TRACE                              printf
#else
#define  APP_CFG_TRACE                              BSP_Ser_Printf
#define  BSP_CFG_TRACE                              BSP_Ser_Printf
#endif

#define  APP_CFG_TRACE_LEVEL                        TRACE_LEVEL_INFO
#define  BSP_CFG_TRACE_LEVEL                        TRACE_LEVEL_INFO

#define  APP_TRACE_INFO(x)               ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_CFG_TRACE x) : (void)0)
#define  APP_TRACE_DBG(x)                ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_DBG)   ? (void)(APP_CFG_TRACE x) : (void)0)

#define  BSP_TRACE_INFO(x)               ((BSP_CFG_TRACE_LEVEL  >= TRACE_LEVEL_INFO) ? (void)(BSP_CFG_TRACE x) : (void)0)
#define  BSP_TRACE_DBG(x)                ((BSP_CFG_TRACE_LEVEL  >= TRACE_LEVEL_DBG)  ? (void)(BSP_CFG_TRACE x) : (void)0)
#endif
