#include "conf.h"

/**
 * Adjust temperature of 3D printer in the box, using fan and heater.
 * Fan and heater are enabled by relay.
 * 
 * @brief 
 * Inspired by sketch https://github.com/milesburton/Arduino-Temperature-Control-Library/blob/master/examples/Single/Single.ino.
 *
 *  Led flashes:
 * - LED flashes 10 times, when reading of the temperature failed
 * - 2 flashes - too cold for a long time - heater is not working propperly
 * - 3 flashes - too hot for a long time - cooling is not working propperly
 * 
 * 433mHz proposal
 * Prefered:
 * https://github.com/PaulStoffregen/RadioHead.git
 * Alternatives:
 * https://github.com/pouriap/TinyRF.git
 * 
 * It would be better to use relays with optocoupler.
 * Single Dallas sensor is currently used for this.
 * TODO: Use ULN2003 for relay coils.
 *       One srd-05vdc-sl-c seems to be good enough for Arduino power supply, but it's not a good idea to power too much relays out of that.
 * 
 * TODO: Serial communication with ESP8266/ESP32 as daughterboard for ESPHOME - Homeassistant integration.
 */

//Current temperature
float current_temp = 0.0f;

//Watchdog
float prev_temp = 0.0f;
int prev_counts = WATCHDOG_TRIES;

bool temp_read_failed = false;
bool heater_on = false;
bool fan_on = false;

#ifdef DISPLAY_I2C
LiquidCrystal_I2C lcd(0x27, 20, 4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
#endif

const char * TEMP_ERR_STR = "TEMP_ERROR: Failed!!";

//Dallas
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
//DeviceAddress insideThermometer;


/*
Flashes error 
*/
void flash_error(uint8_t pin, uint8_t times, uint32_t _delay=ERROR_FLASH_DELAY){
  for(int i=0; i < times; i++){
    digitalWrite(pin, true);
    delay(_delay);
    digitalWrite(pin, false);
    delay(_delay);
  }
}

void sleep_time(long delay_ms){
  if(delay_ms > 0){
    Serial.print("SLEEPING: ");
    Serial.print(delay_ms);
    Serial.println("ms");
    delay(delay_ms);
  }
}


uint8_t readTempAddr(DeviceAddress deviceAddress){
  float tmp = current_temp;
  current_temp = sensors.getTempC(deviceAddress);
  if(current_temp == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("TEMP_ERROR: Could not read temperature data");
    temp_read_failed = true;
    return 1;
  }
  prev_temp = tmp;
  temp_read_failed = false;
  return 0;
}

uint8_t readTempIdx(uint8_t deviceIndex){
  float tmp = current_temp;
  current_temp = sensors.getTempCByIndex(deviceIndex);
  if(current_temp == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("TEMP_ERROR: Could not read temperature data");
    temp_read_failed = true;
    return 1;
  }
  prev_temp = tmp;
  temp_read_failed = false;
  return 0;
}

void stall_temp(){
  if(!fan_on && !heater_on){
    Serial.println("STALE_TEMP: STATE_UNCHANGED");
    return;
  }
  digitalWrite(HEATER_PIN, false);
  digitalWrite(FAN_PIN, false);

  heater_on = false;
  fan_on = false;
  Serial.println("STALE_TEMP: FAN=OFF, HEATER=OFF");
  #ifdef DISPLAY_I2C
    lcd.setCursor(0,1);
    lcd.print("STALE_TEMP");
  #endif
}

void turn_heater_on(){
  if(!heater_on){
    #ifdef COOLER_SEPARE
      digitalWrite(FAN_PIN, false);
      fan_on = false;
      Serial.println("HEATER_TURN_ON: HEATER=ON, FAN=OFF");
    #else
      digitalWrite(FAN_PIN, true);
      fan_on = true;
      Serial.println("HEATER_TURN_ON: HEATER=ON, FAN=ON");
    #endif
    digitalWrite(HEATER_PIN, true);
    heater_on = true;
    #ifdef DISPLAY_I2C
      lcd.setCursor(0,1);
      lcd.print("HEATING!!");
    #endif
    return;
  }
  Serial.println("HEATER_ON: STATE_UNCHANGED");
}
/*
 * Turn fan on
 */
void turn_cooler_on(){
  if(!fan_on){
    digitalWrite(HEATER_PIN, false);
    digitalWrite(FAN_PIN, true);

    heater_on = false;
    fan_on = true;
    Serial.println("FAN_TURN_ON: FAN=ON, HEATER=OFF");
    #ifdef DISPLAY_I2C
      lcd.setCursor(0,1);
      lcd.print("COOLING!!");
    #endif
    return;
  }
  Serial.println("FAN_ON: STATE_UNCHANGED");
}


/**
 * @brief Health watchdog check if LED is present
 * 2 flashes too cold
 * 3 flashes over heat
 */
void temp_watchdog(){
  float tmp_diff = current_temp - TARGET_TEMP;
  if(abs(tmp_diff) > TEMP_DEV){
    //temp is not ok
    if(prev_counts <= 0){
      if(tmp_diff < 0){
        //Heater 
        Serial.println("TEMP_WATCHDOG: HEATER_FAIL");
        #ifdef DISPLAY_I2C
          lcd.setCursor(0,1);
          lcd.print("HEATER_FAIL");
        #endif
        flash_error(LED_PIN, HEATER_FAIL_FLASHES, ERROR_FLASH_DELAY);
        sleep_time(SLEEP_INTERVAL - (HEATER_FAIL_FLASHES * ERROR_FLASH_DELAY * 2));
        return;
      }
      else{
        //Cooler 
        Serial.println("TEMP_WATCHDOG: COOLER_FAIL");
        #ifdef DISPLAY_I2C
          lcd.setCursor(0,1);
          lcd.print("COOLER_FAIL");
        #endif
        flash_error(LED_PIN, COOLER_FAIL_FLASHES, ERROR_FLASH_DELAY);
        sleep_time(SLEEP_INTERVAL - (COOLER_FAIL_FLASHES * ERROR_FLASH_DELAY * 2));
        return;
      }
    }
    else{
      Serial.println("TEMP_WATCHDOG: WAITING_FOR_CHANGE");
      prev_counts--;
      sleep_time(SLEEP_INTERVAL);
      return;
    }
  }
  Serial.println("TEMP_WATCHDOG: OK");
  prev_counts = WATCHDOG_TRIES;
  sleep_time(SLEEP_INTERVAL);
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
  
  //if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 

  // show the addresses we found on the bus
  //Serial.print("Device 0 Address: ");
  //printAddress(insideThermometer);
  // Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  //sensors.setResolution(insideThermometer, 9);
 
  // Serial.print("Device 0 Resolution: ");
  // Serial.print(sensors.getResolution(insideThermometer), DEC); 
  // Serial.println();

  pinMode(FAN_PIN, OUTPUT);
  pinMode(HEATER_PIN, OUTPUT);

  #ifdef DISPLAY_I2C
  lcd.init();                      // initialize the lcd 
  // Print a message to the LCD.
  lcd.backlight();
  #endif

}


/*
 * 
 */
void loop(void)
{ 
  // call sensors.requestTemperatures() to issue a global temperature 
  // request to all devices on the bus
  Serial.print("TEMP_I: Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  //reading DS temp temperature
  if(readTempIdx(0) == 0){
    Serial.print("TEMP: ");
    Serial.print(current_temp);
    Serial.println("C");
    
    #ifdef DISPLAY_I2C
      lcd.setCursor(0,0);
      lcd.print("TEMP ");
      char outstr[4];
      dtostrf(current_temp, 4, 1, outstr);
      lcd.print(outstr);
      lcd.print("C");
    #endif

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
        turn_cooler_on();
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
    Serial.println(TEMP_ERR_STR);
    #ifdef DISPLAY_I2C
      lcd.setCursor(0,1);
      lcd.print(TEMP_ERR_STR);
    #endif
    flash_error(LED_PIN, TEMP_READ_FAIL_FLASHES);
  }
  
  temp_watchdog();
  #ifdef DISPLAY_I2C
    //lcd.clear();
  #endif
}

