#ifdef TEMP_DALLAS
    //Dallas
    // Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
    OneWire oneWire(ONE_WIRE_BUS);

    // Pass our oneWire reference to Dallas Temperature. 
    DallasTemperature sensors(&oneWire);

    // arrays to hold device address
    //DeviceAddress insideThermometer;

    float current_temp = 0;
    float prev_temp = 0;
    bool temp_read_failed = false;
    uint8_t devicesCount = 0;
    float * temps;



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

    // function to print a device address
    void printAddress(DeviceAddress deviceAddress)
    {
    for (uint8_t i = 0; i < 8; i++)
    {
        if (deviceAddress[i] < 16) Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
    }
    }



    void ds_setup(){
    Serial.print("Found ");
    devicesCount = sensors.getDeviceCount();
    Serial.print(devicesCount, DEC);
    Serial.println(" devices.");

    // report parasite power requirements
    Serial.print("Parasite power is: "); 
    if (sensors.isParasitePowerMode()) Serial.println("ON");
    else Serial.println("OFF");
    
    temps = new float[devicesCount];
    //if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 

    // show the addresses we found on the bus
    //   Serial.print("Device 0 Address: ");
    //   printAddress(insideThermometer);
    //   Serial.println();

    // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
    //   sensors.setResolution(insideThermometer, 9);
    
    //   Serial.print("Device 0 Resolution: ");
    //   Serial.print(sensors.getResolution(insideThermometer), DEC); 
    //   Serial.println();
    }



    void ds_loop(){
    Serial.print("TEMP_I: Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    Serial.println("DONE");
    //reading DS temp temperature
    for(uint8_t i=0 ; i < devicesCount; i++){
        if(readTempIdx(i) == 0){
            Serial.print("TEMP: ");
            Serial.print(current_temp);
            temps[i] = current_temp;
            Serial.println("C");
        }
    }
    }
#endif