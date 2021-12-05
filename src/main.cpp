#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/**
 * Adjust temperature of 3D printer in the box, using fan and heater.
 * Fan and heater are enabled by relay.
 * 
 * @brief 
 * Inspired by sketch https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/Single/Single.ino.
 * 
 * It would be better to use relays with optocoupler.
 * Single Dallas sensor is currently used for this.
 * TODO: Use ULN2003 for relay coils.
 *       One srd-05vdc-sl-c seems to be good enough for Arduino power supply, but it's not a good idea to power too much relays out of that.
 * 
 * TODO: Serial communication with ESP8266/ESP32 as daughterboard for ESPHOME - Homeassistant integration.
 */

// Data wire is plugged into port 2 on the Arduino
#define HEATER_PIN 2 //D2
#define FAN_PIN 3 //D3
//Pin for oneWire.
#define ONE_WIRE_BUS 4 //D4
//Update interval
#define SLEEP_INTERVAL 1000

//Target temperature in degrees Celsius
float TARGET_TEMP = 26.0f;
//Temperature deviation in degrees Celsius, when fan or heater are not adjusting internal temperature
float TEMP_DEV = 2.0f;
//Current temperature
float current_temp = 0.0f;
float prev_temp = 0.0f;
bool temp_read_failed = false;
bool heater_on = false;
bool fan_on = false;


// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress insideThermometer;

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


/*
 * Setup function. Here we do the basics
 */
void setup(void)
{
  // start serial port
  Serial.begin(9600);

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 

  // show the addresses we found on the bus
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();

  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);
}

uint8_t readTemp(DeviceAddress deviceAddress){
  current_temp = sensors.getTempC(deviceAddress);
  if(current_temp == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    temp_read_failed = true;
    return 1;
  }
  temp_read_failed = false;
  return 0;
}

void stall_temp(){
  if(!fan_on && !heater_on){
    Serial.println("STALE_TEMP: State unchanged");
    return;
  }
  digitalWrite(HEATER_PIN, false);
  digitalWrite(FAN_PIN, false);

  heater_on = false;
  fan_on = false;
  Serial.println("STALE_TEMP: fan=off, heater=off");
}

void turn_heater_on(){
  if(!heater_on){
    digitalWrite(HEATER_PIN, true);
    digitalWrite(FAN_PIN, false);

    heater_on = true;
    fan_on = false;
    Serial.println("Turning heater=on, fan=off");
    return;
  }
  Serial.println("HEATER_ON: State unchanged");
}
/*
 * Turn fan on
 */
void turn_fan_on(){
  if(!fan_on){
    digitalWrite(HEATER_PIN, false);
    digitalWrite(FAN_PIN, true);

    heater_on = false;
    fan_on = true;
    Serial.println("Turning fan=on, heater=off");
    return;
  }
  Serial.println("FAN_ON: State unchanged");
}

/*
Flashes error 
*/
void flash_error(uint8_t pin, uint8_t times, uint32_t _delay=100){
  for(int i=0; i < times; i++){
    digitalWrite(pin, true);
    delay(_delay);
    digitalWrite(pin, false);
    delay(_delay);
  }
}


/*
 * 
 */
void loop(void)
{ 
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  //reading DS temp temperature
  if(readTemp(insideThermometer) == 0){
    Serial.print("Temp : ");
    Serial.print(current_temp);
    Serial.println("C");
    //continue adjusting if needed
    float diff = current_temp - TARGET_TEMP;
    if(diff < 0.0f){
      if((-diff) > TEMP_DEV){
        turn_heater_on();
      }
      else{
        stall_temp();
      }
    }
    else if(diff > 0.0f){
      if(diff > TEMP_DEV){
        turn_fan_on();
      }
      else{
        stall_temp();
      }
    }
    else{
      stall_temp();
    }
  }
  else{
    Serial.println("Temp Failed!!");
    //LED flashes 10 times, when reading of the temperature failed. 
    flash_error(LED_BUILTIN, 10);
  }
  delay(SLEEP_INTERVAL);
}

