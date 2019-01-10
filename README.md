![MikroE](http://www.mikroe.com/img/designs/beta/logo_small.png)

---

# Servo Click

- **CIC Prefix**  : SERVO
- **Author**      : Katarina Perendic
- **Verison**     : 1.0.0
- **Date**        : Feb 2018.

---

### Software Support

We provide a library for the Servo Click on our [LibStock](https://libstock.mikroe.com/projects/view/2352/servo-click) 
page, as well as a demo application (example), developed using MikroElektronika 
[compilers](http://shop.mikroe.com/compilers). The demo can run on all the main 
MikroElektronika [development boards](http://shop.mikroe.com/development-boards).

**Library Description**

This library will allow you to control multiple servo motors at once.

Key functions :

- ```void servo_init(uint8_t minPosition, uint8_t maxPosition, uint16_t lowResolution, uint16_t highResolution);``` - Main click board initialization routine.
- ```void servo_setMode(uint8_t mode,uint8_t _data);``` - Set's the operation mode of the click board.
- ```servo_sleep();``` - Function needed to be set before seting the frequency.
- ```void servo_setFREQ(uint16_t freq);``` - Used for setting the frequency.
- ```void servo_setPosition(uint8_t motor, uint8_t position);``` - Set the position of the selected servo motor.

**Examples Description**

The application is composed of three sections :

- System Initialization - Initializes I2C module and CS pin as output
- Application Initialization - Initializes driver init and servo init (setting the minimum and maximum servo motors position and resolutions).
                               Default resolutions is 1ms. Then sets the chip to sleep mode to set the frequency, after setting the frequency, 
                               the working mode of the servo motor is set.
- Application Task - (code snippet) - The servo motor is set at three different positions 0, 90, 180 - every two second,
                                      and reads current value witch motor spends in the moment when it runs to new position.

```
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
```

The full application code, and ready to use projects can be found on our 
[LibStock](https://libstock.mikroe.com/projects/view/2352/servo-click) page.

Other mikroE Libraries used in the example:

- I2C Library
- UART Library
- Conversions Library
- C_String Library

**Additional notes and informations**

Depending on the development board you are using, you may need 
[USB UART click](http://shop.mikroe.com/usb-uart-click), 
[USB UART 2 Click](http://shop.mikroe.com/usb-uart-2-click) or 
[RS232 Click](http://shop.mikroe.com/rs232-click) to connect to your PC, for 
development systems with no UART to USB interface available on the board. The 
terminal available in all Mikroelektronika 
[compilers](http://shop.mikroe.com/compilers), or any other terminal application 
of your choice, can be used to read the message.

---
---
