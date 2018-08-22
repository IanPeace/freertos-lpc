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
*                                 STLM75 CMOS TEMPERATURE SENSOR DRIVER
*                                                                         
*
* Filename      : drv_lm75b.c
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

#define  DRV_STLM75_MODULE
#include <bsp.h>
#include <drv_lm75b.h>


/*
*********************************************************************************************************
*                                              LOCAL DEFINES
*********************************************************************************************************
*/

#define BSP_STLM75_REG_TEMP                      DEF_BIT_NONE
#define BSP_STLM75_REG_CONF                      DEF_BIT_00
#define BSP_STLM75_REG_T_HYST                    DEF_BIT_01
#define BSP_STLM75_REG_T_OS                     (DEF_BIT_01 | DEF_BIT_00)


#define BSP_STLM75_REG_CONF_SD                   DEF_BIT_00
#define BSP_STLM75_REG_CONF_THERMOSTAT_MODE      DEF_BIT_01
#define BSP_STLM75_REG_CONF_POL                  DEF_BIT_02
#define BSP_STLM75_REG_CONF_FT0                  DEF_BIT_03
#define BSP_STLM75_REG_CONF_FT1                  DEF_BIT_04


#define BSP_STLM75_I2C_ADDR                     ((uint8_t)0x48)


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                           LOCAL MACRO'S
*********************************************************************************************************
*/

#define  BSP_STLM75_TEMP_TO_REG(temp)         ((uint16_t)((((uint16_t)(temp) << 8) & DEF_BIT_FIELD(9, 7))))
#define  BSP_STLM75_REG_TO_TEMP(reg)          ((int8_t)((((uint16_t)(reg)  >> 8) & DEF_BIT_FIELD(9, 0))))


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


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


static  bool  BSP_STLM75_WrReg_08  (uint8_t reg, uint8_t reg_val);

static  bool  BSP_STLM75_WrReg_16  (uint8_t reg, uint16_t reg_val);

static  bool  BSP_STLM75_RdReg_16  (uint8_t reg, uint16_t *p_reg_val);


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
*                                        BSP_STLM75_Init ()
*
* Description : Initialize the the STLM75 Tempeture sensor.
*
* Argument(s) : none.
*
* Return(s)   : true     If the STLM75 Tempeture sensor was initialized
*               false   If the STLM75 Tempeture sensor could not be initialized.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

bool  BSP_STLM75_Init (void)
{
    bool  err;
    
    
    err = BSP_I2C_Init(BSP_I2C_ID_I2C0, 
                       BSP_I2C_MODE_STANDARD, 
                       BSP_I2C_MODE_STANDARD_MAX_FREQ_HZ);
    
    return (err);
    
}


/*
*
*********************************************************************************************************
*                                        BSP_STLM75_CfgSet()
*
* Description : Configures the STLM75 Tempeture sensor
*
* Argument(s) : p_stlm75_cfg    Pointer to the STLM75 configuration.
*
* Return(s)   : true     If the STLM75 Tempeture Sensor configuration could be returned
*               false   If the STLM75 Tempeture sensor configuration could not be returned
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

bool  BSP_STLM75_CfgSet (BSP_STLM75_CFG  *p_stlm75_cfg)
{
    uint16_t   reg_val;
    bool  err;
    
    
    if (p_stlm75_cfg == (BSP_STLM75_CFG *)0) {
        return (false);
    }
                                                                /* --------- SET THE CONFIGURATION REGISTER -------- */
    reg_val = DEF_BIT_NONE;
    
    if (p_stlm75_cfg->Mode == BSP_STLM75_MODE_INTERRUPT) {
        reg_val = BSP_STLM75_REG_CONF_THERMOSTAT_MODE;
    }

    if (p_stlm75_cfg->IntPol == BSP_STLM75_INT_POL_HIGH) {
        DEF_BIT_SET(reg_val, BSP_STLM75_REG_CONF_POL);
    }
    
    switch (p_stlm75_cfg->FaultLevel) {
        case BSP_STLM75_FAULT_LEVEL_2:
             DEF_BIT_SET(reg_val, BSP_STLM75_REG_CONF_FT0);
             break;
             
        case BSP_STLM75_FAULT_LEVEL_4:
             DEF_BIT_SET(reg_val, BSP_STLM75_REG_CONF_FT1);
             break;

        case BSP_STLM75_FAULT_LEVEL_6:
             DEF_BIT_SET(reg_val, BSP_STLM75_REG_CONF_FT1 | BSP_STLM75_REG_CONF_FT0);
             break;

        case BSP_STLM75_FAULT_LEVEL_1:
             break;
             
        default:
             return (false);
    }
    
    err = BSP_STLM75_WrReg_08((uint8_t)BSP_STLM75_REG_CONF, 
                              (uint8_t)reg_val);
    
    if (err == false) {
        return (false);
    }
    
    reg_val = BSP_STLM75_TEMP_TO_REG(p_stlm75_cfg->HystTemp);
    
    err = BSP_STLM75_WrReg_16((uint8_t)BSP_STLM75_REG_T_HYST, 
                              (uint16_t)reg_val);
    
    if (err == false) {
        return (false);
    }

    
    reg_val = BSP_STLM75_TEMP_TO_REG(p_stlm75_cfg->OverLimitTemp);
    
    err = BSP_STLM75_WrReg_16((uint8_t)BSP_STLM75_REG_T_OS, 
                              (uint16_t)reg_val);
    
    return (err);

}


/*
*********************************************************************************************************
*                                        BSP_STLM75_WrReg_08()
*
* Description : Write 8-bit value to STLM75 register.
*
* Argument(s) : reg        STLM75's register
*                              BSP_STLM75_REG_TEMP
*                              BSP_STLM75_REG_CONF
*                              BSP_STLM75_REG_T_HYST
*                              BSP_STLM75_REG_T_OS
*
* Return(s)   : true     If the STLM75's register could be written.
*               false   If the STLM75's register could not be written.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  bool  BSP_STLM75_WrReg_08  (uint8_t  reg,
                                           uint8_t  reg_val)
{
    uint8_t   i2c_buf[2];
    bool  err;
    
    
    i2c_buf[0] = reg;
    i2c_buf[1] = reg_val;
    
    
    err = BSP_I2C_Wr( BSP_I2C_ID_I2C0, 
                      BSP_STLM75_I2C_ADDR, 
                     &i2c_buf[0], 
                      1,
                     &i2c_buf[1], 
                      1);
    
    return (err);
}


/*
*********************************************************************************************************
*                                        BSP_STLM75_WrReg_16()
*
* Description : Write 16-bit value to STLM75 register.
*
* Argument(s) : reg        STLM75's register
*                              BSP_STLM75_REG_TEMP
*                              BSP_STLM75_REG_CONF
*                              BSP_STLM75_REG_T_HYST
*                              BSP_STLM75_REG_T_OS
*
* Return(s)   : true     If the STLM75's register could be written.
*               false   If the STLM75's register could not be written.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  bool  BSP_STLM75_WrReg_16  (uint8_t  reg,
                                           uint16_t  reg_val)
{
    uint8_t   i2c_buf[3];
    bool  err;
    
    
    i2c_buf[0] = reg;
    i2c_buf[1] = (reg_val >> 8) & DEF_BIT_FIELD(8, 0);
    i2c_buf[2] = (reg_val >> 0) & DEF_BIT_FIELD(8, 0);;
    
    
    err = BSP_I2C_Wr( BSP_I2C_ID_I2C0, 
                      BSP_STLM75_I2C_ADDR, 
                     &i2c_buf[0], 
                      1,
                     &i2c_buf[1], 
                      2);
    
    return (err);
}


/*
*********************************************************************************************************
*                                        BSP_STLM75_RdReg_16()
*
* Description : Read 16-bit register from STLM75 device
*
* Argument(s) : reg        STLM75's register
*                              BSP_STLM75_REG_TEMP
*                              BSP_STLM75_REG_CONF
*                              BSP_STLM75_REG_T_HYST
*                              BSP_STLM75_REG_T_OS
*
* Return(s)   : true     If the STLM75's register could be read.
*               false   If the STLM75's register could not be read.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

static  bool  BSP_STLM75_RdReg_16  (uint8_t   reg,
                                           uint16_t  *p_reg_val)
{
    uint8_t   i2c_buf[3];
    bool  err;
    
    
    i2c_buf[0] = reg;
    
    
    err = BSP_I2C_WrRd( BSP_I2C_ID_I2C0, 
                        BSP_STLM75_I2C_ADDR, 
                       &i2c_buf[0],
                        1,
                       &i2c_buf[1],
                        2);

    if (err == false) {
       return (false);
    }

    *p_reg_val   = (i2c_buf[2] << 0)
                 | (i2c_buf[1] << 8);
    
    return (true);
}


/*
*********************************************************************************************************
*                                        BSP_STLM75_TempGet()
*
* Description : Read the current temperature from the STLM75
*
* Argument(s) : temp_unit       Temperature unit:
*                                   BSP_STLM75_TEMP_UNIT_CELSIUS
*                                   BSP_STLM75_TEMP_UNIT_FAHRENHEIT
*                                   BSP_STLM75_TEMP_UNIT_KELVIN
*
*               p_temp_val      Pointer to the variable that will store the temperature.

*
* Return(s)   : true     If the temperature could be read.
*               false   If the temperature could not be read.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

bool  BSP_STLM75_TempGet  (uint8_t   temp_unit,
                                  int16_t  *p_temp_val)

{
    int16_t   temp_c;
    uint16_t   reg_val;
    bool  err;
    
    
    err = BSP_STLM75_RdReg_16(BSP_STLM75_REG_TEMP,
                             &reg_val);
        
    if (err == false) {
        return (false);
    }
    

    temp_c  = BSP_STLM75_REG_TO_TEMP(reg_val);

    switch (temp_unit) {
        case BSP_STLM75_TEMP_UNIT_CELSIUS:
            *p_temp_val = temp_c;
             break;
             
        case BSP_STLM75_TEMP_UNIT_FAHRENHEIT:
             *p_temp_val = ((temp_c * 9) / 5) + 32;
             break;
        
        
        case BSP_STLM75_TEMP_UNIT_KELVIN:
             *p_temp_val = ((temp_c + 273));
             break;
             
        
        default:
             return (false);
             
    }
       
   return (true);
}
