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

//if defined, cooler and heater are separate.
//Together could be fan and heater, so you could run just one of them or on heater pin is just heater.
//#define COOLER_SEPARE 1
// Data wire is plugged into port 2 on the Arduino
#define HEATER_PIN 2 //D2
#define FAN_PIN 3 //D3
//Pin for oneWire.
#define ONE_WIRE_BUS 4 //D4
//Update interval
#define SLEEP_INTERVAL 1000

#define LED_PIN 13 //Currently builtin LED
//Watchdog number of tries, after that signal malfunction
#define WATCHDOG_TRIES 60 //180 * sleep interval seconds


#define ERROR_FLASH_DELAY 300
#define HEATER_FAIL_FLASHES 2
#define COOLER_FAIL_FLASHES 3
//LED flashes 10 times, when reading of the temperature failed. 
#define TEMP_READ_FAIL_FLASHES 10

#define TEMP_DALLAS
//#define TEMP_NTC

#include "base.h"
#include "ntc.h"
#include "ds.h"

//Target temperature in degrees Celsius
float TARGET_TEMP = 26.0f;
//Temperature deviation in degrees Celsius, when fan or heater are not adjusting internal temperature
float TEMP_DEV = 2.0f;