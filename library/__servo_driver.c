/*
    __servo_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__servo_driver.h"
#include "__servo_hal.c"

/* ------------------------------------------------------------------- MACROS */

// SERVO registers
const uint8_t _SERVO_REG_MODE_1         = 0x00;
const uint8_t _SERVO_REG_MODE_2         = 0x01;
const uint8_t _SERVO_REG_SUB_ADDRESS_1  = 0x02;
const uint8_t _SERVO_REG_SUB_ADDRESS_2  = 0x03;
const uint8_t _SERVO_REG_SUB_ADDRESS_3  = 0x04;
const uint8_t _SERVO_REG_ALL_CALL_ADR   = 0x05;

const uint8_t _SERVO_REG_MOTOR_1_ON_L     = 0x06;
const uint8_t _SERVO_REG_MOTOR_1_ON_H     = 0x07;
const uint8_t _SERVO_REG_MOTOR_1_OFF_L    = 0x08;
const uint8_t _SERVO_REG_MOTOR_1_OFF_H    = 0x09;
const uint8_t _SERVO_REG_MOTOR_2_ON_L     = 0x0A;
const uint8_t _SERVO_REG_MOTOR_2_ON_H     = 0x0B;
const uint8_t _SERVO_REG_MOTOR_2_OFF_L    = 0x0C;
const uint8_t _SERVO_REG_MOTOR_2_OFF_H    = 0x0D;
const uint8_t _SERVO_REG_MOTOR_3_ON_L     = 0x0E;
const uint8_t _SERVO_REG_MOTOR_3_ON_H     = 0x0F;
const uint8_t _SERVO_REG_MOTOR_3_OFF_L    = 0x10;
const uint8_t _SERVO_REG_MOTOR_3_OFF_H    = 0x11;
const uint8_t _SERVO_REG_MOTOR_4_ON_L     = 0x12;
const uint8_t _SERVO_REG_MOTOR_4_ON_H     = 0x13;
const uint8_t _SERVO_REG_MOTOR_4_OFF_L    = 0x14;
const uint8_t _SERVO_REG_MOTOR_4_OFF_H    = 0x15;
const uint8_t _SERVO_REG_MOTOR_5_ON_L     = 0x16;
const uint8_t _SERVO_REG_MOTOR_5_ON_H     = 0x17;
const uint8_t _SERVO_REG_MOTOR_5_OFF_L    = 0x18;
const uint8_t _SERVO_REG_MOTOR_5_OFF_H    = 0x19;
const uint8_t _SERVO_REG_MOTOR_6_ON_L     = 0x1A;
const uint8_t _SERVO_REG_MOTOR_6_ON_H     = 0x1B;
const uint8_t _SERVO_REG_MOTOR_6_OFF_L    = 0x1C;
const uint8_t _SERVO_REG_MOTOR_6_OFF_H    = 0x1D;
const uint8_t _SERVO_REG_MOTOR_7_ON_L     = 0x1E;
const uint8_t _SERVO_REG_MOTOR_7_ON_H     = 0x1F;
const uint8_t _SERVO_REG_MOTOR_7_OFF_L    = 0x20;
const uint8_t _SERVO_REG_MOTOR_7_OFF_H    = 0x21;
const uint8_t _SERVO_REG_MOTOR_8_ON_L     = 0x22;
const uint8_t _SERVO_REG_MOTOR_8_ON_H     = 0x23;
const uint8_t _SERVO_REG_MOTOR_8_OFF_L    = 0x24;
const uint8_t _SERVO_REG_MOTOR_8_OFF_H    = 0x25;
const uint8_t _SERVO_REG_MOTOR_9_ON_L     = 0x26;
const uint8_t _SERVO_REG_MOTOR_9_ON_H     = 0x27;
const uint8_t _SERVO_REG_MOTOR_9_OFF_L    = 0x28;
const uint8_t _SERVO_REG_MOTOR_9_OFF_H    = 0x29;
const uint8_t _SERVO_REG_MOTOR_10_ON_L    = 0x2A;
const uint8_t _SERVO_REG_MOTOR_10_ON_H    = 0x2B;
const uint8_t _SERVO_REG_MOTOR_10_OFF_L   = 0x2C;
const uint8_t _SERVO_REG_MOTOR_10_OFF_H   = 0x2D;
const uint8_t _SERVO_REG_MOTOR_11_ON_L    = 0x2E;
const uint8_t _SERVO_REG_MOTOR_11_ON_H    = 0x2F;
const uint8_t _SERVO_REG_MOTOR_11_OFF_L   = 0x30;
const uint8_t _SERVO_REG_MOTOR_11_OFF_H   = 0x31;
const uint8_t _SERVO_REG_MOTOR_12_ON_L    = 0x32;
const uint8_t _SERVO_REG_MOTOR_12_ON_H    = 0x33;
const uint8_t _SERVO_REG_MOTOR_12_OFF_L   = 0x34;
const uint8_t _SERVO_REG_MOTOR_12_OFF_H   = 0x35;
const uint8_t _SERVO_REG_MOTOR_13_ON_L    = 0x36;
const uint8_t _SERVO_REG_MOTOR_13_ON_H    = 0x37;
const uint8_t _SERVO_REG_MOTOR_13_OFF_L   = 0x38;
const uint8_t _SERVO_REG_MOTOR_13_OFF_H   = 0x39;
const uint8_t _SERVO_REG_MOTOR_14_ON_L    = 0x3A;
const uint8_t _SERVO_REG_MOTOR_14_ON_H    = 0x3B;
const uint8_t _SERVO_REG_MOTOR_14_OFF_L   = 0x3C;
const uint8_t _SERVO_REG_MOTOR_14_OFF_H   = 0x3D;
const uint8_t _SERVO_REG_MOTOR_15_ON_L    = 0x3E;
const uint8_t _SERVO_REG_MOTOR_15_ON_H    = 0x3F;
const uint8_t _SERVO_REG_MOTOR_15_OFF_L   = 0x40;
const uint8_t _SERVO_REG_MOTOR_15_OFF_H   = 0x41;
const uint8_t _SERVO_REG_MOTOR_16_ON_L    = 0x42;
const uint8_t _SERVO_REG_MOTOR_16_ON_H    = 0x43;
const uint8_t _SERVO_REG_MOTOR_16_OFF_L   = 0x44;
const uint8_t _SERVO_REG_MOTOR_16_OFF_H   = 0x45;

const uint8_t _SERVO_REG_ALL_MOTOR_ON_L   = 0xFA;
const uint8_t _SERVO_REG_ALL_MOTOR_ON_H   = 0xFB;
const uint8_t _SERVO_REG_ALL_MOTOR_OFF_L  = 0xFC;
const uint8_t _SERVO_REG_ALL_MOTOR_OFF_H  = 0xFD;
const uint8_t _SERVO_REG_PRE_SCALE      = 0xFE;
const uint8_t _SERVO_REG_TEST_MODE      = 0xFF;

// MODE 1 register
const uint8_t _SERVO_MODE1_RESTART_ENABLE          = 0x01 << 7;
const uint8_t _SERVO_MODE1_RESTART_DISABLE         = 0x00 << 7;
const uint8_t _SERVO_MODE1_INTERNAL_CLOCK          = 0x00 << 6;
const uint8_t _SERVO_MODE1_EXTCLK_PIN_CLOCK        = 0x01 << 6;
const uint8_t _SERVO_MODE1_AUTO_INCREMENT_ENABLE   = 0x01 << 5;
const uint8_t _SERVO_MODE1_AUTO_INCREMENT_DISABLE  = 0x00 << 5;
const uint8_t _SERVO_MODE1_NORMAL_MODE             = 0x00 << 4;
const uint8_t _SERVO_MODE1_LOW_POWER_MODE          = 0x01 << 4;
const uint8_t _SERVO_MODE1_USE_SUBADR_1            = 0x01 << 3;
const uint8_t _SERVO_MODE1_NO_USE_SUBADR_1         = 0x00 << 3;
const uint8_t _SERVO_MODE1_USE_SUBADR_2            = 0x01 << 2;
const uint8_t _SERVO_MODE1_NO_USE_SUBADR_2         = 0x00 << 2;
const uint8_t _SERVO_MODE1_USE_SUBADR_3            = 0x01 << 1;
const uint8_t _SERVO_MODE1_NO_USE_SUBADR_3         = 0x00 << 1;
const uint8_t _SERVO_MODE1_USE_ALL_CALL_ADR        = 0x01;
const uint8_t _SERVO_MODE1_NO_USE_ALL_CALL_ADR     = 0x00;

// MODE 2 register
const uint8_t _SERVO_MODE2_OUT_LOGIC_NOT_INVERTED   = 0x00 << 4;
const uint8_t _SERVO_MODE2_OUT_LOGIC_INVERTED       = 0x01 << 4;
const uint8_t _SERVO_MODE2_OUT_CHANGE_ON_STOP_CMD   = 0x00 << 3;
const uint8_t _SERVO_MODE2_OUT_CHANGE_ON_ACK_CMD    = 0x01 << 3;
const uint8_t _SERVO_MODE2_OPEN_DRAIN_STRUCTURE     = 0x00 << 2;
const uint8_t _SERVO_MODE2_TOTEM_POLE_STRUCTURE     = 0x01 << 2;

// SERVO MIN/MAX
const uint16_t _SERVO_DEFAULT_LOW_RESOLUTION = 0;
const uint16_t _SERVO_DEFAULT_HIGH_RESOLUTION = 330;

const uint8_t _SERVO_GENERAL_CALL_ADR = 0x00;
const uint8_t _SERVO_SOFT_RESET       = 0x06;

const uint16_t _SERVO_VREF_3300 = 3300;
const uint16_t _SERVO_VREF_5000 = 5000;

// SERVO MOTOR
const uint8_t _SERVO_MOTOR_1 = 0x06;
const uint8_t _SERVO_MOTOR_2 = 0x0A;
const uint8_t _SERVO_MOTOR_3 = 0x0E;
const uint8_t _SERVO_MOTOR_4 = 0x12;
const uint8_t _SERVO_MOTOR_5 = 0x16;
const uint8_t _SERVO_MOTOR_6 = 0x1A;
const uint8_t _SERVO_MOTOR_7 = 0x1E;
const uint8_t _SERVO_MOTOR_8 = 0x22;
const uint8_t _SERVO_MOTOR_9 = 0x26;
const uint8_t _SERVO_MOTOR_10 = 0x2A;
const uint8_t _SERVO_MOTOR_11 = 0x2E;
const uint8_t _SERVO_MOTOR_12 = 0x32;
const uint8_t _SERVO_MOTOR_13 = 0x36;
const uint8_t _SERVO_MOTOR_14 = 0x3A;
const uint8_t _SERVO_MOTOR_15 = 0x3E;
const uint8_t _SERVO_MOTOR_16 = 0x42;


const uint8_t  _SERVO_POSITIVE_CH0_NEGATIVE_CH1   = 0xA0;
const uint8_t  _SERVO_POSITIVE_CH2_NEGATIVE_CH3   = 0xA1;
const uint8_t  _SERVO_POSITIVE_CH4_NEGATIVE_CH5   = 0xA2;
const uint8_t  _SERVO_POSITIVE_CH6_NEGATIVE_CH7   = 0xA3;
const uint8_t  _SERVO_POSITIVE_CH8_NEGATIVE_CH9   = 0xA4;
const uint8_t  _SERVO_POSITIVE_CH10_NEGATIVE_CH11 = 0xA5;
const uint8_t  _SERVO_POSITIVE_CH12_NEGATIVE_CH13 = 0xA6;
const uint8_t  _SERVO_POSITIVE_CH14_NEGATIVE_CH15 = 0xA7;
const uint8_t  _SERVO_POSITIVE_CH1_NEGATIVE_CH0   = 0xA8;
const uint8_t  _SERVO_POSITIVE_CH3_NEGATIVE_CH2   = 0xA9;
const uint8_t  _SERVO_POSITIVE_CH5_NEGATIVE_CH4   = 0xAA;
const uint8_t  _SERVO_POSITIVE_CH7_NEGATIVE_CH6   = 0xAB;
const uint8_t  _SERVO_POSITIVE_CH9_NEGATIVE_CH8   = 0xAC;
const uint8_t  _SERVO_POSITIVE_CH11_NEGATIVE_CH10 = 0xAD;
const uint8_t  _SERVO_POSITIVE_CH13_NEGATIVE_CH12 = 0xAE;
const uint8_t  _SERVO_POSITIVE_CH15_NEGATIVE_CH14 = 0xAF;

const uint8_t  _SERVO_POSITIVE_CH0    = 0xB0;
const uint8_t  _SERVO_POSITIVE_CH2    = 0xB1;
const uint8_t  _SERVO_POSITIVE_CH4    = 0xB2;
const uint8_t  _SERVO_POSITIVE_CH6    = 0xB3;
const uint8_t  _SERVO_POSITIVE_CH8    = 0xB4;
const uint8_t  _SERVO_POSITIVE_CH10   = 0xB5;
const uint8_t  _SERVO_POSITIVE_CH12   = 0xB6;
const uint8_t  _SERVO_POSITIVE_CH14   = 0xB7;
const uint8_t  _SERVO_POSITIVE_CH1    = 0xB8;
const uint8_t  _SERVO_POSITIVE_CH3    = 0xB9;
const uint8_t  _SERVO_POSITIVE_CH5    = 0xBA;
const uint8_t  _SERVO_POSITIVE_CH7    = 0xBB;
const uint8_t  _SERVO_POSITIVE_CH9    = 0xBC;
const uint8_t  _SERVO_POSITIVE_CH11   = 0xBD;
const uint8_t  _SERVO_POSITIVE_CH13   = 0xBE;
const uint8_t  _SERVO_POSITIVE_CH15   = 0xBF;














/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __SERVO_DRV_I2C__
static uint8_t _slaveAddress_of_PCA9685;
static uint8_t _slaveAddress_of_LTC2497;
#endif

static uint8_t _minPos = 0;
static uint8_t _maxPos = 180;
static uint16_t _vref = _SERVO_VREF_3300;
static uint16_t _lowRes = _SERVO_DEFAULT_LOW_RESOLUTION;
static uint16_t _highRes = _SERVO_DEFAULT_HIGH_RESOLUTION;


/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */

uint16_t _map(uint8_t x, uint8_t in_min, uint8_t in_max, uint16_t out_min, uint16_t out_max);


/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */

uint16_t _map(uint8_t x, uint8_t in_min, uint8_t in_max, uint16_t out_min, uint16_t out_max)
{
     return (x - in_min)*(out_max - out_min)/(in_max - in_min) + out_min + 10;
}


/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __SERVO_DRV_SPI__

void servo_spiDriverInit(T_SERVO_P gpioObj, T_SERVO_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __SERVO_DRV_I2C__

void servo_i2cDriverInit(T_SERVO_P gpioObj, T_SERVO_P i2cObj, uint8_t slave_of_PCA9685, uint8_t slave_of_LTC2497)
{
    _slaveAddress_of_PCA9685 = slave_of_PCA9685;
    _slaveAddress_of_LTC2497 = slave_of_LTC2497;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    hal_gpio_csSet(0);
}

#endif
#ifdef   __SERVO_DRV_UART__

void servo_uartDriverInit(T_SERVO_P gpioObj, T_SERVO_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

}

#endif

/* ----------------------------------------------------------- IMPLEMENTATION */

void servo_init(uint8_t minPosition, uint8_t maxPosition, uint16_t lowResolution, uint16_t highResolution)
{
    _minPos = minPosition;
    _maxPos = maxPosition;
    _lowRes  = lowResolution & 0x0FFF;
    _highRes = highResolution & 0x0FFF;
}

void servo_setVref(uint16_t Vref)
{
    _vref = Vref;
}

void servo_stop()
{
    hal_gpio_csSet(1);
}

void servo_start()
{
    hal_gpio_csSet(0);
}

void servo_softReset()
{
    uint8_t writeReg[1];
    writeReg[0] = _SERVO_SOFT_RESET;

    hal_i2cStart();
    hal_i2cWrite(_SERVO_GENERAL_CALL_ADR, writeReg, 1, END_MODE_STOP);
    Delay_100ms();
}

void servo_sleep()
{
    servo_setMode( _SERVO_REG_MODE_1,
                   _SERVO_MODE1_RESTART_ENABLE |
                   _SERVO_MODE1_EXTCLK_PIN_CLOCK |
                   _SERVO_MODE1_LOW_POWER_MODE |
                   _SERVO_MODE1_AUTO_INCREMENT_ENABLE |
                   _SERVO_MODE1_USE_ALL_CALL_ADR);
}


void servo_setMode(uint8_t mode,uint8_t _data)
{
    uint8_t writeReg[2];
    writeReg[0] = mode;
    writeReg[1] = _data;
     
    servo_start();
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress_of_PCA9685, writeReg, 2, END_MODE_STOP);
    Delay_100ms();
}

void servo_setPosition(uint8_t motor, uint8_t position)
{
    uint8_t writeReg[5];
    uint16_t setMap;
    uint16_t on = 0x0000;
     
    setMap = _map(position,_minPos,_maxPos,_lowRes,_highRes) ;
    if (setMap < 70 )
        setMap = 70;
     
    writeReg[0] = motor;
    writeReg[1] = on;
    writeReg[2] = on >> 8;
    writeReg[3] = setMap;
    writeReg[4] = setMap >> 8;

    servo_start();
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress_of_PCA9685, writeReg, 5, END_MODE_STOP);
}



void servo_setFREQ(uint16_t freq)
{
    uint8_t writeReg[2];
    uint32_t prescaleval;
    writeReg[0] = _SERVO_REG_PRE_SCALE;
    
    prescaleval = 25000000;
    prescaleval /= 4096;
    prescaleval /= freq;
    prescaleval -= 1;
    writeReg[1] = prescaleval;
    
    servo_start();
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress_of_PCA9685, writeReg, 2, END_MODE_STOP);
    Delay_100ms();
}

uint32_t servo_getChannel(uint8_t channel)
{
    uint8_t writeReg[1];
    uint8_t readReg[3];
    uint32_t ADC_Value;
    writeReg[0] = channel;

    servo_start();
    hal_i2cStart();
    hal_i2cWrite(_slaveAddress_of_LTC2497, writeReg, 1, END_MODE_RESTART);
    hal_i2cRead(_slaveAddress_of_LTC2497, readReg, 3, END_MODE_STOP );
    
    ADC_Value = readReg[0];
    ADC_Value = ADC_Value << 8;
    ADC_Value = ADC_Value | readReg[1];
    ADC_Value = ADC_Value << 8;
    ADC_Value = ADC_Value | readReg[2];
    
    ADC_Value = ADC_Value & 0x00FFFFC0;
    ADC_Value = ADC_Value >> 5;

    return ADC_Value;
}

uint16_t setvo_getCurrent(uint8_t channel)
{
    uint32_t ADC_Value;
    uint16_t current;
    ADC_Value = servo_getChannel(channel);
    ADC_Value = ADC_Value & 0x00003FFF;
    current = ( ADC_Value * _vref ) / 65535;
    return  current;
}





/* -------------------------------------------------------------------------- */
/*
  __servo_driver.c

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