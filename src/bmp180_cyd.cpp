#include <bmp180_cyd.h>

BMP180I2C bmp180(BMP_I2C_ADDRESS);

static bool has_bmp180 = false;

void setup_bmp180() {
    if (!bmp180.begin()) {
        has_bmp180 = false;
        Serial.println("Failed to init the BMP180 pressure sensor");
        return;
    }
    
    has_bmp180 = true;
    Serial.printf("Found BMP180 at 0x%X\n", BMP_I2C_ADDRESS);
    
    // reset 
    bmp180.resetToDefaults();

    // Ultar high resolution mode
    bmp180.setSamplingMode(BMP180I2C::MODE_UHR);
}

float get_temperature() {
    if(!bmp180.measureTemperature()) {
        return NAN;
    }
    do {
        delay(100);
    } while(!bmp180.hasValue());

    return bmp180.getTemperature();
}

float get_pressure() {
   if(!bmp180.measurePressure()) {
        return NAN;
    }
    do {
        delay(100);
    } while(!bmp180.hasValue());

    return bmp180.getPressure();
}