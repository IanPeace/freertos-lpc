/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*
*                          (c) Copyright 2003-2010; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               This BSP is provided in source form to registered licensees ONLY.  It is
*               illegal to distribute this source code to any third party unless you receive
*               written permission by an authorized Micrium representative.  Knowledge of
*               the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*                                       SERIAL (UART) INTERFACE
*
* Filename      : bsp_ser.c
* Version       : V1.00
* Programmer(s) : EHS
*                 SR
*                 AA
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <stdarg.h>
#include <FreeRTOS.h>
#include <semphr.h>
#include <bsp.h>
#include <board.h>

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/

#define  ASCII_CHAR_LINE_FEED                           0x0A    /* '\n'                                                 */
#define  ASCII_CHAR_CARRIAGE_RETURN                     0x0D    /* '\r'                                                 */

/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

#if 1

static  SemaphoreHandle_t      BSP_SerTxWait;
static  SemaphoreHandle_t      BSP_SerRxWait;
static  SemaphoreHandle_t      BSP_SerLock;
static  uint8_t  BSP_SerRxData;
static  uint8_t  BSP_SerTxData;
static  void (*BSP_SerTxCallBack)(LPC_USART_T *pUART);
static  void (*BSP_SerRxCallBack)(LPC_USART_T *pUART);

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void    BSP_Ser_WrByteUnlocked  (uint8_t  c);
static  uint8_t BSP_Ser_RdByteUnlocked  (void);
void  BSP_Ser_Tx_Callback (LPC_USART_T *pUART);
void  BSP_Ser_Rx_Callback (LPC_USART_T *pUART);

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*********************************************************************************************************
**                                         GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                          BSP_Ser_Init()
*
* Description : Initialize a serial port for communication.
*
* Argument(s) : baud_rate           The desire RS232 baud rate.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_Init (int32_t  baud_rate)
{
    BSP_SerTxWait = xSemaphoreCreateBinary();
    BSP_SerRxWait = xSemaphoreCreateBinary();
    BSP_SerLock = xSemaphoreCreateMutex();
    BSP_SerTxCallBack = BSP_Ser_Tx_Callback;
    BSP_SerRxCallBack = BSP_Ser_Rx_Callback;

    Chip_UART_SetBaud(DEBUG_UART, baud_rate);
    NVIC_EnableIRQ(UART0_IRQn);
}

/**
 * @brief	UART interrupt handler using ring buffers
 * @return	Nothing
 */
void UART0_IRQHandler(void)
{
	/* Handle transmit interrupt if enabled */
	if (((Chip_UART_GetIntsEnabled(DEBUG_UART) & UART_INTEN_TXRDY) != 0) &&
        ((Chip_UART_GetStatus(DEBUG_UART) & UART_STAT_TXRDY) != 0)) {
		BSP_SerTxCallBack(DEBUG_UART);
	}

	/* Handle receive interrupt */
	if (((Chip_UART_GetIntsEnabled(DEBUG_UART) & UART_INTEN_RXRDY) != 0) &&
        ((Chip_UART_GetStatus(DEBUG_UART) & UART_STAT_RXRDY) != 0)) {
    	BSP_SerRxCallBack(DEBUG_UART);
    }
}

/*
*********************************************************************************************************
*                                         BSP_Ser_ISR_Handler()
*
* Description : Serial ISR.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_Tx_Callback (LPC_USART_T *pUART)
{
    BaseType_t reschedule;

    Chip_UART_SendByte(pUART, BSP_SerTxData);
	Chip_UART_IntDisable(pUART, UART_INTEN_TXRDY);

    /* Post to the semaphore */
    xSemaphoreGiveFromISR(BSP_SerTxWait, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}

void  BSP_Ser_Rx_Callback (LPC_USART_T *pUART)
{
    BaseType_t reschedule;

    /* Read one byte from the receive data register */
    BSP_SerRxData = Chip_UART_ReadByte(pUART);     
    /* Post to the semaphore */
    xSemaphoreGiveFromISR(BSP_SerRxWait, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}


/*
*********************************************************************************************************
*                                           BSP_Ser_Printf()
*
* Description : Print formatted data to the output serial port.
*
* Argument(s) : format      String that contains the text to be written.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This function output a maximum of BSP_SER_PRINTF_STR_BUF_SIZE number of bytes to the
*                   serial port.  The calling function hence has to make sure the formatted string will
*                   be able fit into this string buffer or hence the output string will be truncated.
*********************************************************************************************************
*/

void  BSP_Ser_Printf (int8_t  *format, ...)
{
    int8_t  buf_str[BSP_SER_PRINTF_STR_BUF_SIZE + 1u];
    va_list v_args;

    va_start(v_args, format);
    (void)vsnprintf((char       *)&buf_str[0],
                    (size_t      )sizeof(buf_str),
                    (char const *)format,
                    v_args);
    va_end(v_args);

    vSerialPutString(buf_str);
}


/*
*********************************************************************************************************
*                                                BSP_Ser_RdByte()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte.
*
* Caller(s)   : Application.
*
* Note(s)     : (1) This functions blocks until a data is received.
*
*               (2) It can not be called from an ISR.
*********************************************************************************************************
*/

uint8_t  BSP_Ser_RdByte (void)
{
    uint8_t  rx_byte;

    /* Lock resource mutex */
    xSemaphoreTake(BSP_SerLock, portMAX_DELAY);
    rx_byte = BSP_Ser_RdByteUnlocked();
    /* Unlock resource mutex */
    xSemaphoreGive(BSP_SerLock);

    return (rx_byte);
}


/*
*********************************************************************************************************
*                                       BSP_Ser_RdByteUnlocked()
*
* Description : Receive a single byte.
*
* Argument(s) : none.
*
* Return(s)   : The received byte.
*
* Caller(s)   : BSP_Ser_RdByte()
*               BSP_Ser_RdStr()
*
* Note(s)     : none.
*********************************************************************************************************
*/

uint8_t  BSP_Ser_RdByteUnlocked (void)
{
    uint8_t   rx_byte;

    /* Enable the Receive not empty interrupt */
	Chip_UART_IntEnable(DEBUG_UART, UART_INTEN_RXRDY);

    /* Wait until data is received */
    xSemaphoreTake(BSP_SerRxWait, portMAX_DELAY);

    /* Disable the Receive not empty interrupt */
	Chip_UART_IntDisable(DEBUG_UART, UART_INTEN_RXRDY);

    rx_byte = BSP_SerRxData;                                    /* Read the data from the temporary register          */

    return (rx_byte);
}


/*
*********************************************************************************************************
*                                          BSP_Ser_WrByteUnlocked()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : c           The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : BSP_Ser_WrByte()
*               BSP_Ser_WrByteUnlocked()
*
* Note(s)     : (1) This function blocks until room is available in the UART for the byte to be sent.
*********************************************************************************************************
*/

void  BSP_Ser_WrByteUnlocked (uint8_t c)
{
    BSP_SerTxData = c;
    /* Enable the Transmit empty interrupt */
	Chip_UART_IntEnable(DEBUG_UART, UART_INTEN_TXRDY);
    /* Wait for transfer to finish */
    xSemaphoreTake(BSP_SerTxWait, portMAX_DELAY);
}


/*
*********************************************************************************************************
*                                                BSP_Ser_WrByte()
*
* Description : Writes a single byte to a serial port.
*
* Argument(s) : tx_byte     The character to output.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_WrByte(uint8_t  c)
{
    /* Lock resource mutex */
    xSemaphoreTake(BSP_SerLock, portMAX_DELAY);
    BSP_Ser_WrByteUnlocked(c);
    /* Unlock resource mutex */
    xSemaphoreGive(BSP_SerLock);
}


/*
*********************************************************************************************************
*                                                vSerialPutString()
*
* Description : Transmits a string.
*
* Argument(s) : p_str       Pointer to the string that will be transmitted.
*
* Caller(s)   : Application.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  vSerialPutString (int8_t  *p_str)
{
    if (p_str == (int8_t *)0) {
        return;
    }

    /* Lock resource mutex */
    xSemaphoreTake(BSP_SerLock, portMAX_DELAY);

    while ((*p_str) != (int8_t )0) {
        if (*p_str == ASCII_CHAR_LINE_FEED) {
            BSP_Ser_WrByteUnlocked(ASCII_CHAR_CARRIAGE_RETURN);
            BSP_Ser_WrByteUnlocked(ASCII_CHAR_LINE_FEED);
            p_str++;
        } else {
            BSP_Ser_WrByteUnlocked(*p_str++);
        }
    }

    /* Unlock resource mutex */
    xSemaphoreGive(BSP_SerLock);
}


#if 1

/* For Shell usage */

#define CONSOLE_BUF_SIZE                        128             /* Must be 2^x  */

static char CBuf[CONSOLE_BUF_SIZE];
static uint8_t CBuf_Top = 0;
static uint8_t CBuf_Cnt = 0;

static void  BSP_Console_Tx_Callback (LPC_USART_T *pUART)
{
    BaseType_t reschedule;

    Chip_UART_SendByte(pUART, BSP_SerTxData);
	Chip_UART_IntDisable(pUART, UART_INTEN_TXRDY);
    /* Post to the semaphore */
    xSemaphoreGiveFromISR(BSP_SerTxWait, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}

static void  BSP_Console_Rx_Callback (LPC_USART_T *pUART)
{
    BaseType_t reschedule;

    CBuf[(CBuf_Top - CBuf_Cnt + CONSOLE_BUF_SIZE) & (CONSOLE_BUF_SIZE - 1)] =
        Chip_UART_ReadByte(pUART);                          /* Read one byte from the receive data register.      */

    if (CBuf_Cnt != CONSOLE_BUF_SIZE) {
       CBuf_Cnt++;
    } else {
       CBuf_Top = (CBuf_Top + (CONSOLE_BUF_SIZE - 1)) & (CONSOLE_BUF_SIZE - 1);
    }

    /* Post to the semaphore */
    xSemaphoreGiveFromISR(BSP_SerRxWait, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}


void  vSerialEnable (void)
{
    BSP_SerTxCallBack = BSP_Console_Tx_Callback;
    BSP_SerRxCallBack = BSP_Console_Rx_Callback;
    /* Enable the Receive not empty interrupt */
	Chip_UART_IntEnable(DEBUG_UART, UART_INTEN_RXRDY);
    return;
}

int8_t  xSerialGetChar (void)
{
    int8_t      rx_byte;

    /* Wait until data is received */
    xSemaphoreTake(BSP_SerRxWait, portMAX_DELAY);

    /* Disable the Receive not empty interrupt */
	Chip_UART_IntDisable(DEBUG_UART, UART_INTEN_RXRDY);

    rx_byte = CBuf[CBuf_Top];
    CBuf_Top = (CBuf_Top - 1 + CONSOLE_BUF_SIZE) & (CONSOLE_BUF_SIZE - 1);
    CBuf_Cnt--;

    /* Enable the Receive not empty interrupt */
	Chip_UART_IntEnable(DEBUG_UART, UART_INTEN_RXRDY);

    return (rx_byte);
}

void  xSerialPutChar (char c)
{
    /* Lock resource mutex */
    xSemaphoreTake(BSP_SerLock, portMAX_DELAY);
    BSP_Ser_WrByteUnlocked((uint8_t)c);
    /* Unlock resource mutex */
    xSemaphoreGive(BSP_SerLock);
}
#endif  /* (APP_CFG_SHELL_EN == DEF_ENABLED) */


#if 0

/* For printf usage */

#pragma import(__use_no_semihosting)

struct __FILE
{
    int handle;
};

FILE __stdout;

int _sys_exit (int x)
{
    x = x;
    return 0;
}

int fputc (int ch, FILE *f)
{
    /* Lock resource mutex */
    xSemaphoreTake(BSP_SerLock, portMAX_DELAY);
    BSP_Ser_WrByteUnlocked((uint8_t)ch);
    /* Unlock resource mutex */
    xSemaphoreGive(BSP_SerLock);

    return ch;
}
#endif  /* (APP_CFG_PRINTF_EN == DEF_ENABLED) */

#endif  /* (APP_CFG_SERIAL_EN == DEF_ENABLED) */

