/*
Example for Servo Click

    Date          : Feb 2018.
    Author        : MikroE Team

Test configuration STM32 :
    
    MCU              : STM32F107VCT6
    Dev. Board       : EasyMx PRO v7 for STM32
    ARM Compiler ver : v6.0.0.0

---

Description :

The application is composed of three sections :

- System Initialization - Initializes I2C module and CS pin as output
- Application Initialization - Initializes driver init and servo init (setting the minimum and maximum servo motors position and resolutions).
                               Default resolutions is 1ms. Then sets the chip to sleep mode to set the frequency, after setting the frequency, 
                               the working mode of the servo motor is set.
- Application Task - (code snippet) - The servo motor is set at three different positions 0, 90, 180 - every two second,
                                      and reads current value witch motor spends in the moment when it runs to new position.

*/

#include "Click_Servo_types.h"
#include "Click_Servo_config.h"


uint16_t Current;
char text[256];

void systemInit()
{
    mikrobus_gpioInit( _MIKROBUS1, _MIKROBUS_CS_PIN, _GPIO_OUTPUT );
    mikrobus_i2cInit( _MIKROBUS1, &_SERVO_I2C_CFG[0] );
    mikrobus_logInit(_LOG_USBUART_A, 9600 );
    
    mikrobus_logWrite( " --- system init ---", _LOG_LINE );
    Delay_ms( 100 );
}

void applicationInit()
{
    servo_i2cDriverInit( (T_SERVO_P)&_MIKROBUS1_GPIO, (T_SERVO_P)&_MIKROBUS1_I2C, 0x40, 0x14 );
    servo_init( 0, 180, _SERVO_DEFAULT_LOW_RESOLUTION, _SERVO_DEFAULT_HIGH_RESOLUTION );
	servo_setVref( _SERVO_VREF_3300 );
    servo_setMode( _SERVO_REG_MODE_1, _SERVO_MODE1_RESTART_ENABLE | _SERVO_MODE1_USE_ALL_CALL_ADR );
    servo_sleep();
    servo_setFREQ( 30 ); // 30 Hz
    servo_setMode( _SERVO_REG_MODE_1,_SERVO_MODE1_RESTART_ENABLE| _SERVO_MODE1_AUTO_INCREMENT_ENABLE | _SERVO_MODE1_USE_ALL_CALL_ADR );
}

void applicationTask()
{
     servo_setPosition(_SERVO_MOTOR_1, 0);
     Delay_ms( 2000 );
     servo_setPosition(_SERVO_MOTOR_1, 90);
     Delay_ms( 1000 );
     servo_setPosition(_SERVO_MOTOR_1, 180);
     Delay_ms( 2000 );
     servo_setPosition(_SERVO_MOTOR_1, 90);

     Current = setvo_getCurrent(_SERVO_POSITIVE_CH0);
     IntToStr(Current , text);
     mikrobus_logWrite( "Current - ", _LOG_TEXT );
     mikrobus_logWrite( text, _LOG_TEXT );
     mikrobus_logWrite( " mA", _LOG_LINE );
     
     Delay_ms( 1000 );
}

void main()
{
    systemInit();
    applicationInit();

    while (1)
    {
            applicationTask();
    }
}