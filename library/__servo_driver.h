/*
    __servo_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __servo_driver.h
@brief    Servo Driver
@mainpage Servo Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   SERVO
@brief      Servo Click Driver
@{

| Global Library Prefix | **SERVO** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **Feb 2018.**      |
| Developer             | **MikroE Team**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _SERVO_H_
#define _SERVO_H_

/** 
 * @macro T_SERVO_P
 * @brief Driver Abstract type 
 */
#define T_SERVO_P    const uint8_t*

/** @defgroup SERVO_COMPILE Compilation Config */              /** @{ */

//  #define   __SERVO_DRV_SPI__                            /**<     @macro __SERVO_DRV_SPI__  @brief SPI driver selector */
   #define   __SERVO_DRV_I2C__                            /**<     @macro __SERVO_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __SERVO_DRV_UART__                           /**<     @macro __SERVO_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup SERVO_VAR Variables */                           /** @{ */

// SERVO registers
extern const uint8_t _SERVO_REG_MODE_1       ;
extern const uint8_t _SERVO_REG_MODE_2       ;
extern const uint8_t _SERVO_REG_SUB_ADDRESS_1;
extern const uint8_t _SERVO_REG_SUB_ADDRESS_2;
extern const uint8_t _SERVO_REG_SUB_ADDRESS_3;
extern const uint8_t _SERVO_REG_ALL_CALL_ADR ;

extern const uint8_t _SERVO_REG_MOTOR_1_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_1_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_1_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_1_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_2_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_2_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_2_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_2_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_3_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_3_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_3_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_3_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_4_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_4_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_4_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_4_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_5_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_5_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_5_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_5_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_6_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_6_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_6_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_6_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_7_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_7_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_7_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_7_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_8_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_8_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_8_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_8_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_9_ON_L    ;
extern const uint8_t _SERVO_REG_MOTOR_9_ON_H    ;
extern const uint8_t _SERVO_REG_MOTOR_9_OFF_L   ;
extern const uint8_t _SERVO_REG_MOTOR_9_OFF_H   ;
extern const uint8_t _SERVO_REG_MOTOR_10_ON_L   ;
extern const uint8_t _SERVO_REG_MOTOR_10_ON_H   ;
extern const uint8_t _SERVO_REG_MOTOR_10_OFF_L  ;
extern const uint8_t _SERVO_REG_MOTOR_10_OFF_H  ;
extern const uint8_t _SERVO_REG_MOTOR_11_ON_L   ;
extern const uint8_t _SERVO_REG_MOTOR_11_ON_H   ;
extern const uint8_t _SERVO_REG_MOTOR_11_OFF_L  ;
extern const uint8_t _SERVO_REG_MOTOR_11_OFF_H  ;
extern const uint8_t _SERVO_REG_MOTOR_12_ON_L   ;
extern const uint8_t _SERVO_REG_MOTOR_12_ON_H   ;
extern const uint8_t _SERVO_REG_MOTOR_12_OFF_L  ;
extern const uint8_t _SERVO_REG_MOTOR_12_OFF_H  ;
extern const uint8_t _SERVO_REG_MOTOR_13_ON_L   ;
extern const uint8_t _SERVO_REG_MOTOR_13_ON_H   ;
extern const uint8_t _SERVO_REG_MOTOR_13_OFF_L  ;
extern const uint8_t _SERVO_REG_MOTOR_13_OFF_H  ;
extern const uint8_t _SERVO_REG_MOTOR_14_ON_L   ;
extern const uint8_t _SERVO_REG_MOTOR_14_ON_H   ;
extern const uint8_t _SERVO_REG_MOTOR_14_OFF_L  ;
extern const uint8_t _SERVO_REG_MOTOR_14_OFF_H  ;
extern const uint8_t _SERVO_REG_MOTOR_15_ON_L   ;
extern const uint8_t _SERVO_REG_MOTOR_15_ON_H   ;
extern const uint8_t _SERVO_REG_MOTOR_15_OFF_L  ;
extern const uint8_t _SERVO_REG_MOTOR_15_OFF_H  ;
extern const uint8_t _SERVO_REG_MOTOR_16_ON_L   ;
extern const uint8_t _SERVO_REG_MOTOR_16_ON_H   ;
extern const uint8_t _SERVO_REG_MOTOR_16_OFF_L  ;
extern const uint8_t _SERVO_REG_MOTOR_16_OFF_H  ;

extern const uint8_t _SERVO_REG_ALL_MOTOR_ON_L  ;
extern const uint8_t _SERVO_REG_ALL_MOTOR_ON_H  ;
extern const uint8_t _SERVO_REG_ALL_MOTOR_OFF_L ;
extern const uint8_t _SERVO_REG_ALL_MOTOR_OFF_H ;
extern const uint8_t _SERVO_REG_PRE_SCALE     ;
extern const uint8_t _SERVO_REG_TEST_MODE     ;

// MODE 1 register
extern const uint8_t _SERVO_MODE1_RESTART_ENABLE         ;
extern const uint8_t _SERVO_MODE1_RESTART_DISABLE        ;
extern const uint8_t _SERVO_MODE1_INTERNAL_CLOCK         ;
extern const uint8_t _SERVO_MODE1_EXTCLK_PIN_CLOCK       ;
extern const uint8_t _SERVO_MODE1_AUTO_INCREMENT_ENABLE  ;
extern const uint8_t _SERVO_MODE1_AUTO_INCREMENT_DISABLE ;
extern const uint8_t _SERVO_MODE1_NORMAL_MODE            ;
extern const uint8_t _SERVO_MODE1_LOW_POWER_MODE         ;
extern const uint8_t _SERVO_MODE1_USE_SUBADR_1           ;
extern const uint8_t _SERVO_MODE1_NO_USE_SUBADR_1        ;
extern const uint8_t _SERVO_MODE1_USE_SUBADR_2           ;
extern const uint8_t _SERVO_MODE1_NO_USE_SUBADR_2        ;
extern const uint8_t _SERVO_MODE1_USE_SUBADR_3           ;
extern const uint8_t _SERVO_MODE1_NO_USE_SUBADR_3        ;
extern const uint8_t _SERVO_MODE1_USE_ALL_CALL_ADR      ;
extern const uint8_t _SERVO_MODE1_NO_USE_ALL_CALL_ADR   ;

// MODE 2 register
extern const uint8_t _SERVO_MODE2_OUT_LOGIC_NOT_INVERTED;
extern const uint8_t _SERVO_MODE2_OUT_LOGIC_INVERTED    ;
extern const uint8_t _SERVO_MODE2_OUT_CHANGE_ON_STOP_CMD;
extern const uint8_t _SERVO_MODE2_OUT_CHANGE_ON_ACK_CMD ;
extern const uint8_t _SERVO_MODE2_OPEN_DRAIN_STRUCTURE  ;
extern const uint8_t _SERVO_MODE2_TOTEM_POLE_STRUCTURE  ;

// SERVO MIN/MAX
extern const uint16_t _SERVO_DEFAULT_LOW_RESOLUTION;
extern const uint16_t _SERVO_DEFAULT_HIGH_RESOLUTION;

extern const uint8_t _SERVO_GENERAL_CALL_ADR;
extern const uint8_t _SERVO_SOFT_RESET      ;

extern const uint16_t _SERVO_VREF_3300;
extern const uint16_t _SERVO_VREF_5000;

// SERVO MOTOR
extern const uint8_t _SERVO_MOTOR_1;
extern const uint8_t _SERVO_MOTOR_2;
extern const uint8_t _SERVO_MOTOR_3;
extern const uint8_t _SERVO_MOTOR_4;
extern const uint8_t _SERVO_MOTOR_5;
extern const uint8_t _SERVO_MOTOR_6;
extern const uint8_t _SERVO_MOTOR_7;
extern const uint8_t _SERVO_MOTOR_8;
extern const uint8_t _SERVO_MOTOR_9;
extern const uint8_t _SERVO_MOTOR_10;
extern const uint8_t _SERVO_MOTOR_11;
extern const uint8_t _SERVO_MOTOR_12;
extern const uint8_t _SERVO_MOTOR_13;
extern const uint8_t _SERVO_MOTOR_14;
extern const uint8_t _SERVO_MOTOR_15;
extern const uint8_t _SERVO_MOTOR_16;

extern const uint8_t  _SERVO_POSITIVE_CH0_NEGATIVE_CH1;
extern const uint8_t  _SERVO_POSITIVE_CH2_NEGATIVE_CH3;
extern const uint8_t  _SERVO_POSITIVE_CH4_NEGATIVE_CH5;
extern const uint8_t  _SERVO_POSITIVE_CH6_NEGATIVE_CH7;
extern const uint8_t  _SERVO_POSITIVE_CH8_NEGATIVE_CH9;
extern const uint8_t  _SERVO_POSITIVE_CH10_NEGATIVE_CH11;
extern const uint8_t  _SERVO_POSITIVE_CH12_NEGATIVE_CH13;
extern const uint8_t  _SERVO_POSITIVE_CH14_NEGATIVE_CH15;
extern const uint8_t  _SERVO_POSITIVE_CH1_NEGATIVE_CH0;
extern const uint8_t  _SERVO_POSITIVE_CH3_NEGATIVE_CH2;
extern const uint8_t  _SERVO_POSITIVE_CH5_NEGATIVE_CH4;
extern const uint8_t  _SERVO_POSITIVE_CH7_NEGATIVE_CH6;
extern const uint8_t  _SERVO_POSITIVE_CH9_NEGATIVE_CH8;
extern const uint8_t  _SERVO_POSITIVE_CH11_NEGATIVE_CH10;
extern const uint8_t  _SERVO_POSITIVE_CH13_NEGATIVE_CH12;
extern const uint8_t  _SERVO_POSITIVE_CH15_NEGATIVE_CH14;

extern const uint8_t  _SERVO_POSITIVE_CH0;
extern const uint8_t  _SERVO_POSITIVE_CH2;
extern const uint8_t  _SERVO_POSITIVE_CH4;
extern const uint8_t  _SERVO_POSITIVE_CH6;
extern const uint8_t  _SERVO_POSITIVE_CH8;
extern const uint8_t  _SERVO_POSITIVE_CH10;
extern const uint8_t  _SERVO_POSITIVE_CH12;
extern const uint8_t  _SERVO_POSITIVE_CH14;
extern const uint8_t  _SERVO_POSITIVE_CH1;
extern const uint8_t  _SERVO_POSITIVE_CH3;
extern const uint8_t  _SERVO_POSITIVE_CH5;
extern const uint8_t  _SERVO_POSITIVE_CH7;
extern const uint8_t  _SERVO_POSITIVE_CH9;
extern const uint8_t  _SERVO_POSITIVE_CH11;
extern const uint8_t  _SERVO_POSITIVE_CH13;
extern const uint8_t  _SERVO_POSITIVE_CH15;


                                                                       /** @} */
/** @defgroup SERVO_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup SERVO_INIT Driver Initialization */              /** @{ */

#ifdef   __SERVO_DRV_SPI__
void servo_spiDriverInit(T_SERVO_P gpioObj, T_SERVO_P spiObj);
#endif
#ifdef   __SERVO_DRV_I2C__
void servo_i2cDriverInit(T_SERVO_P gpioObj, T_SERVO_P i2cObj, uint8_t slave_of_PCA9685, uint8_t slave_of_LTC2497);
#endif
#ifdef   __SERVO_DRV_UART__
void servo_uartDriverInit(T_SERVO_P gpioObj, T_SERVO_P uartObj);
#endif

// GPIO Only Drivers - remove in other cases
void servo_gpioDriverInit(T_SERVO_P gpioObj);
                                                                       /** @} */
/** @defgroup SERVO_FUNC Driver Functions */                   /** @{ */

/**
 * @brief Initialization
 *
 * @param[in] minPosition - Minimum position in which the servo motor can be positioned
 * @param[in] maxPosition - Maximum position in which the servo motor can be positioned
 * @param[in] lowResolution - lower limit of resolution
 * @param[in] highResolution - upper limit of resolution
 *
 */
void servo_init(uint8_t minPosition, uint8_t maxPosition, uint16_t lowResolution, uint16_t highResolution);

/**
 * @brief Functions for soft reset chip
 */
void servo_softReset();

/**
 * @brief Functions for sleep mode chip
 */
void servo_sleep();

/**
 * @brief Functions for stop work servo motor
 *
 * When the servo_stop() function is started, all controls and communication with the servo motor are blocked.
   To restore communication, you need to call the servo_start() function.
   Use this function when you need to shut down the all motors quickly.
 */
void servo_stop();

/**
 * @brief Functions for start work servo motor
 */
void servo_start();

/**
 * @brief Functions for settings Vref of  Servo Clicks
 */
void servo_setVref(uint16_t Vref);

/**
 * @brief Functions for set mode
 *
 * @param[in] mode - one of the two modes to be set
 * @param[in] _data - data that will be written in the register
 *
 * Options for mode 1 that can be set:
        Restart ( enable or disable )
        Clock ( internal clock or EXTCLK pin clock )
        Auto-Increment ( enable or disable )
        Mode ( normal mode or low power mod )
        SubAddress 1 ( uses or does not use )
        SubAddress 2 ( uses or does not use )
        SubAddress 3 ( uses or does not use )
        Motor all call address ( uses or does not use )
        
 * Options for mode 2 that can be set:
        Output logic state ( not inverted or inverted )
        Outputs change ( Outputs change on STOP or ACK command )
        Outputs configured ( open-drain structure or totem pole structure)
        
 */
void servo_setMode(uint8_t mode,uint8_t _data);

/**
 * @brief Functions for start work servo motor
 *
 * @param[in] motor - motor to be set
 * @param[in] position - position on which the motor will be set
 *
 */
void servo_setPosition(uint8_t motor, uint8_t position);

/**
 * @brief Functions for start work servo motor
 *
 * @param[in] freq - the frequency to be set
 *
 * Default frequency in 200Hz, the position of the motor depends on the set frequency
   The maximum PWM frequency is 1526 Hz and minimum PWM frequency is 24 Hz
 */
void servo_setFREQ(uint16_t freq);

/**
 * @brief Functions for reading adc value of current
 *
 * @param[in] channel - the channel from which it is read
 *
 * @return adc value of current
 *
 * The function reads the current value of Servo Click witch motor spends.
 */
uint32_t servo_getChannel(uint8_t channel);

/**
 * @brief Functions for reading current in mA
 *
 * @param[in] channel - the channel from which it is read
 *
 * @return current value witch motor spends in the moment when it runs to new position.
 */
uint16_t setvo_getCurrent(uint8_t channel);








                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Servo_STM.c
    @example Click_Servo_TIVA.c
    @example Click_Servo_CEC.c
    @example Click_Servo_KINETIS.c
    @example Click_Servo_MSP.c
    @example Click_Servo_PIC.c
    @example Click_Servo_PIC32.c
    @example Click_Servo_DSPIC.c
    @example Click_Servo_AVR.c
    @example Click_Servo_FT90x.c
    @example Click_Servo_STM.mbas
    @example Click_Servo_TIVA.mbas
    @example Click_Servo_CEC.mbas
    @example Click_Servo_KINETIS.mbas
    @example Click_Servo_MSP.mbas
    @example Click_Servo_PIC.mbas
    @example Click_Servo_PIC32.mbas
    @example Click_Servo_DSPIC.mbas
    @example Click_Servo_AVR.mbas
    @example Click_Servo_FT90x.mbas
    @example Click_Servo_STM.mpas
    @example Click_Servo_TIVA.mpas
    @example Click_Servo_CEC.mpas
    @example Click_Servo_KINETIS.mpas
    @example Click_Servo_MSP.mpas
    @example Click_Servo_PIC.mpas
    @example Click_Servo_PIC32.mpas
    @example Click_Servo_DSPIC.mpas
    @example Click_Servo_AVR.mpas
    @example Click_Servo_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __servo_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */