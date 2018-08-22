/*
*********************************************************************************************************
*                                     MICRIUM BOARD SUPPORT SUPPORT
*
*                          (c) Copyright 2003-2009; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                     MICIUM BOARD SUPPORT PACKAGE
*                                        I2C DRIVER (MASTER ONLY)
*                                                                         
*
* Filename      : bsp_i2c.c
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
* Note(s)       :
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include <string.h>

#include <FreeRTOS.h>
#include <semphr.h>
#include <bsp.h>
#include <board.h>

/*
*********************************************************************************************************
*                                              LOCAL DEFINES
*********************************************************************************************************
*/

#define BSP_I2C_MODULE
#define BSP_I2C_BUF_SIZE                        0x10
#define BSP_I2C_HDL_MEM_SIZE                    0x20

/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/



/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*
* Note(s) :  The 'BSP_I2C_DEV' structure defines the status of the current transfer
*
*********************************************************************************************************
*/

typedef  struct bsp_i2c_dev {
    I2C_HANDLE_T *I2CHandle;
    SemaphoreHandle_t   SemLock;                                       /* I2C Exclusive access sempahore                       */
    SemaphoreHandle_t   SemWait;                                       /* Transfer Complete signal                             */
    uint8_t     TxBuf[BSP_I2C_BUF_SIZE];                       /* The transfer data area                               */
    uint8_t     RxBuf[BSP_I2C_BUF_SIZE];                       /* The receive data area                                */
    uint32_t    I2CHandleMem[BSP_I2C_HDL_MEM_SIZE];
} BSP_I2C_DEV;

/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/

static  BSP_I2C_DEV     BSP_I2C_DevTbl[BSP_I2C_NBR_MAX];

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static void BSP_I2C0_Callback (uint32_t err_code, uint32_t n);

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

/* Initializes pin muxing for I2C interface - note that SystemInit() may
   already setup your pin muxing at system startup */
static void BSP_I2C_PinMux(void)
{
#if defined(BOARD_NXP_LPCXPRESSO_1549)
	Chip_SYSCTL_PeriphReset(RESET_I2C0);
	Chip_SWM_FixedPinEnable(SWM_FIXED_I2C0_SDA, 1);
	Chip_SWM_FixedPinEnable(SWM_FIXED_I2C0_SCL, 1);

	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 22, IOCON_STDI2C_EN);
	Chip_IOCON_PinMuxSet(LPC_IOCON, 0, 23, IOCON_STDI2C_EN);
	/* Enable the OLCD */
	Chip_IOCON_PinMuxSet(LPC_IOCON, 1, 2, IOCON_MODE_INACT);
	Chip_GPIO_SetPinDIROutput(LPC_GPIO, 1, 2);
	Chip_GPIO_SetPinOutLow(LPC_GPIO, 1, 2);
#else
	#warning "No I2C Pin Muxing defined for board"
#endif
}

/*
*********************************************************************************************************
*                                        BSP_I2C_Init()
*
* Description : Initialize the I2C.
*
* Argument(s) : i2c_id     I2C peripheral ID
*                              BSP_I2C_ID_I2C0
*
*               bit_rate   I2C clock speed. It must be set to a value lower than 100 kHz (Standard Mode) or
*                          400 Khz (Fast mode)
*
* Return(s)   : true     If the I2C peripheral was initialized
*               false   If the I2C peripheral could not be initialized.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/


bool  BSP_I2C_Init (uint8_t     i2c_id,
                    uint8_t     i2c_mode,
                    uint32_t    bit_rate)
                    
{
    uint32_t    i2c_base;
    BSP_I2C_DEV *p_i2c_dev;

    switch (i2c_id) {
        case BSP_I2C_ID_I2C0:
            /* Setup I2C pin muxing */
            BSP_I2C_PinMux();
            /* Enable I2C clock and reset I2C peripheral */
            Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_I2C0);
            i2c_base = LPC_I2C_BASE;
            p_i2c_dev = (BSP_I2C_DEV *)&BSP_I2C_DevTbl[0];

            break;

        default:
            return (false);
    }

    BSP_Ser_Printf("i2c ram size: 0x%x\n\r", LPC_I2CD_API->i2c_get_mem_size());
    /* Perform a sanity check on the storage allocation */
    if (LPC_I2CD_API->i2c_get_mem_size() > sizeof(p_i2c_dev->I2CHandleMem)) {
        /* Example only: this should never happen and probably isn't needed for
           most I2C code. */
        return (false);
    }

    /* Setup the I2C handle */
    p_i2c_dev->I2CHandle = LPC_I2CD_API->i2c_setup(i2c_base, p_i2c_dev->I2CHandleMem);
    if (p_i2c_dev->I2CHandle == NULL) {
        return (false);
    }

    /* Set I2C bitrate */
    if (LPC_I2CD_API->i2c_set_bitrate(p_i2c_dev->I2CHandle,
                                      Chip_Clock_GetSystemClockRate(),
                                      bit_rate) != LPC_OK) {
        return (false);
    }

    /* Create OS Semaphore */
    p_i2c_dev->SemWait = xSemaphoreCreateBinary();
    p_i2c_dev->SemLock = xSemaphoreCreateMutex();

    switch (i2c_id) {
        case BSP_I2C_ID_I2C0:
            /* Enable the interrupt for the I2C */
            NVIC_EnableIRQ(I2C0_IRQn);
            break;

        default:
             break;
    }

    return (true);
    
}


/*
*********************************************************************************************************
*                                        BSP_I2C_Rd()
*
* Description : Read 'n' bytes from the I2C bus.
*
* Argument(s) : i2c_id       I2C peripheral number
*                                BSP_I2C_ID_I2C0
*
*               i2c_addr     The I2C device address
*
*               p_buf        Pointer to the buffer into which the bytes will be stored.
*
*               nbr_bytes    Number of bytes to be read.
*
* Return(s)   : true       If all bytes were read.
*               false     If all bytes could not be read.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

bool  BSP_I2C_Rd (uint8_t   i2c_id,
                  uint8_t   i2c_addr,
                  uint8_t   *p_buf,
                  uint16_t  nbr_bytes)
{
    bool            err;
	I2C_PARAM_T     param;
	I2C_RESULT_T    result;
	ErrorCode_t     error_code;
    BSP_I2C_DEV     *p_i2c_dev;

    if (p_buf == (uint8_t *)0) {
        return (false);
    }
    if ((nbr_bytes < 1) || ((nbr_bytes + 1) > BSP_I2C_BUF_SIZE)) {
        return (false);
    }
    switch (i2c_id) {
        case BSP_I2C_ID_I2C0:
            p_i2c_dev = (BSP_I2C_DEV *)&BSP_I2C_DevTbl[0];
            break;

        default:
            return (false);
    }

    /* Lock the I2C peripheral */
    xSemaphoreTake(p_i2c_dev->SemLock, portMAX_DELAY);

	p_i2c_dev->RxBuf[0] = ((i2c_addr << 1) | DEF_BIT_00);

	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 - ack
	   value 2 - ack - stop */
	param.num_bytes_send    = 0;
	param.num_bytes_rec     = nbr_bytes + 1;
	param.buffer_ptr_rec    = p_i2c_dev->RxBuf;
	param.stop_flag         = 1;
	param.func_pt           = BSP_I2C0_Callback;

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(p_i2c_dev->I2CHandle, 100000);

	/* Do master write transfer */
	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_receive_intr(p_i2c_dev->I2CHandle, &param, &result);

    /* Wait until the transfer completes */
    xSemaphoreTake(p_i2c_dev->SemWait, 500);

    /* Release the I2C Peripheral */
    xSemaphoreGive(p_i2c_dev->SemLock);

    if (error_code != 0) {                                      /* If the transfer is incomplete ...                    */
        err  = false;                                        /* ... return an errror                                 */
    } else {
        memcpy(p_buf, &p_i2c_dev->RxBuf[1], nbr_bytes);
    }

    return (err);

}


/*
*********************************************************************************************************
*                                        BSP_I2C_Wr()
*
* Description : Write 'n' bytes tothe I2C bus.
*
* Argument(s) : i2c_id      I2C peripheral number
*                                BSP_I2C_ID_I2C0
*
*               i2c_addr     The I2C device address
*
*               p_buf        Pointer to the buffer where the bytes will be transfered.
*
*               nbr_bytes    Number of bytes to be read.
*
* Return(s)   : true       If all bytes were written
*               false     If all bytes could not be written.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/


bool  BSP_I2C_Wr (uint8_t   i2c_id,
                  uint8_t   i2c_addr,
                  uint8_t   *offset_buf,
                  uint8_t   offset_len,
                  uint8_t   *p_buf,
                  uint16_t  nbr_bytes)
{
    bool            err;
	I2C_PARAM_T     param;
	I2C_RESULT_T    result;
	ErrorCode_t     error_code;
    BSP_I2C_DEV     *p_i2c_dev;

    if (((offset_buf == (uint8_t *)0) && (offset_len != 0)) ||
        ((p_buf == (uint8_t *)0) && (nbr_bytes != 0))) {
        return (false);
    }
    if (((offset_len + nbr_bytes) < 1) || ((offset_len + nbr_bytes + 1) > BSP_I2C_BUF_SIZE)) {
        return (false);
    }
    switch (i2c_id) {
        case BSP_I2C_ID_I2C0:
            p_i2c_dev = (BSP_I2C_DEV *)&BSP_I2C_DevTbl[0];
            break;

        default:
            return (false);
    }

    /* Lock the I2C peripheral */
    xSemaphoreTake(p_i2c_dev->SemLock, portMAX_DELAY);

	p_i2c_dev->TxBuf[0] = ((i2c_addr << 1) & DEF_BIT_FIELD(7, 1));
    if (offset_len != 0) {
        memcpy(&p_i2c_dev->TxBuf[1], offset_buf, offset_len);
    }
    if (nbr_bytes != 0) {
        memcpy(&p_i2c_dev->TxBuf[offset_len + 1], p_buf, nbr_bytes);
    }

	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 - ack
	   value 2 - ack - stop */
	param.num_bytes_send    = offset_len + nbr_bytes + 1;
	param.buffer_ptr_send   = p_i2c_dev->TxBuf;
	param.num_bytes_rec     = 0;
	param.stop_flag         = 1;
	param.func_pt           = BSP_I2C0_Callback;

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(p_i2c_dev->I2CHandle, 100000);

	/* Do master write transfer */
	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_transmit_intr(p_i2c_dev->I2CHandle, &param, &result);

    /* Wait until the transfer completes */
    xSemaphoreTake(p_i2c_dev->SemWait, 500);

    /* Release the I2C Peripheral */
    xSemaphoreGive(p_i2c_dev->SemLock);

    if (error_code != 0) {                                      /* If the transfer is incomplete ...                    */
        BSP_Ser_Printf("i2c err: 0x%x\r\n", error_code);
        err  = false;                                        /* ... return an errror                                 */
    }

    return (err);
}


/*
*********************************************************************************************************
*                                        BSP_I2C_WrRd()
*
* Description : Perform a write followed by multiples/single read(s)
*
* Argument(s) : i2c_id      I2C peripheral number
*                                BSP_I2C_ID_I2C0
*
*               i2c_addr     The I2C device address
*
*               p_buf        Pointer to the buffer where the bytes will be transfered/received.
*
*               nbr_bytes    Number of bytes to be read.
*
* Return(s)   : true       If all bytes were read
*               false     If all bytes could not be read.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

bool  BSP_I2C_WrRd (uint8_t     i2c_id,
                    uint8_t     i2c_addr,
                    uint8_t     *offset_buf,
                    uint8_t     offset_len,
                    uint8_t     *p_buf,
                    uint16_t    nbr_bytes)
{
    bool            err;
	I2C_PARAM_T     param;
	I2C_RESULT_T    result;
	ErrorCode_t     error_code;
    BSP_I2C_DEV     *p_i2c_dev;

    if ((offset_buf == (uint8_t *)0) || (p_buf == (uint8_t *)0)) {
        return (false);
    }
    if ((nbr_bytes < 1) ||
        ((offset_len + 1) > BSP_I2C_BUF_SIZE) ||
        ((nbr_bytes + 1) > BSP_I2C_BUF_SIZE)) {
        return (false);
    }
    switch (i2c_id) {
        case BSP_I2C_ID_I2C0:
            p_i2c_dev = (BSP_I2C_DEV *)&BSP_I2C_DevTbl[0];
            break;

        default:
            return (false);
    }

    /* Lock the I2C peripheral */
    xSemaphoreTake(p_i2c_dev->SemLock, portMAX_DELAY);

	p_i2c_dev->TxBuf[0] = ((i2c_addr << 1) & DEF_BIT_FIELD(7, 1));
	p_i2c_dev->RxBuf[0] = ((i2c_addr << 1) | DEF_BIT_00);
    memcpy(&p_i2c_dev->TxBuf[1], offset_buf, offset_len);

	/* Setup I2C parameters for number of bytes with stop - appears as follows on bus:
	   Start - address7 or address10upper - ack
	   (10 bits addressing only) address10lower - ack
	   value 1 - ack
	   value 2 - ack - stop */
	param.num_bytes_send    = offset_len + 1;
	param.buffer_ptr_send   = p_i2c_dev->TxBuf;
	param.num_bytes_rec     = nbr_bytes + 1;
	param.buffer_ptr_rec    = p_i2c_dev->RxBuf;
	param.stop_flag         = 1;
	param.func_pt           = BSP_I2C0_Callback;

	/* Set timeout (much) greater than the transfer length */
	LPC_I2CD_API->i2c_set_timeout(p_i2c_dev->I2CHandle, 100000);

	/* Do master write transfer */
	/* Function is non-blocking, returned error should be LPC_OK, but isn't checked here */
	error_code = LPC_I2CD_API->i2c_master_tx_rx_intr(p_i2c_dev->I2CHandle, &param, &result);

    /* Wait until the transfer completes */
    xSemaphoreTake(p_i2c_dev->SemWait, 500);

    /* Release the I2C Peripheral */
    xSemaphoreGive(p_i2c_dev->SemLock);

    if (error_code != 0) {                                      /* If the transfer is incomplete ...                    */
        err  = false;                                        /* ... return an errror                                 */
    } else {
        memcpy(p_buf, &p_i2c_dev->RxBuf[1], nbr_bytes);
    }

    return (err);             
}


/*
*********************************************************************************************************
*                                        BSP_I2C0_ISR_Handler()
*
* Description : I2C0 ISR handlers
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


void  I2C0_IRQHandler (void)
{
	/* Call I2C ISR function in ROM with the I2C handle */
	LPC_I2CD_API->i2c_isr_handler(BSP_I2C_DevTbl[0].I2CHandle);
}

/*
*********************************************************************************************************
*                                        BSP_I2C0_Callback()
*
* Description : I2C0 callback function
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  void  BSP_I2C0_Callback (uint32_t err_code, uint32_t n) 
{
    BaseType_t reschedule;

    /* Post to the semaphore */
    xSemaphoreGiveFromISR(BSP_I2C_DevTbl[0].SemWait, &reschedule);
    portYIELD_FROM_ISR(reschedule);
}

