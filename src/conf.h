#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//Comment or delete to disable I2C display support
// Board	SDA	SCL
// Arduino Uno	A4	A5
// Arduino Nano	A4	A5
// Arduino Micro	2	3
// Arduino Mega 2560	20	21
// Arduino Leonardo	2	3
// Arduino Due	20	21
#define DISPLAY_I2C true

#ifdef DISPLAY_I2C
    #include <LiquidCrystal_I2C.h>
#endif