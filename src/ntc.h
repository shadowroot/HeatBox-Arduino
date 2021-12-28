#ifdef TEMP_NTC
    //NTC
    int Vo;
    float R1 = 100000;
    float logR2, R2, T;
    float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

    void setup(uint8_t thermistorPin){
        pinMode(thermistorPin, INPUT);
    }

    float getNTCTemp(uint8_t thermistorPin){
    Vo = analogRead(thermistorPin);
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
    T = T - 273.15;
    return T;
    }
#endif