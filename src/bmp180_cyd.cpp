
#include <Arduino.h>
#include <bmp180_cyd.h>
#include <map>

BMP180I2C bmp180(BMP_I2C_ADDRESS);

std::map<String, float> Sensors;

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
    if (!bmp180.measureTemperature()) {
        return NAN;
    }
    do {
        delay(100);
    } while (!bmp180.hasValue());

    return bmp180.getTemperature();
}

float get_pressure() {
    if (!bmp180.measurePressure()) {
        return NAN;
    }
    do {
        delay(100);
    } while (!bmp180.hasValue());

    return bmp180.getPressure();
}

// Read the sensors, update the sensor map
// Only do this at a low data rate so as not to delay everyting else
void handleSensors() {
    static time_t last = 0;
    static const int period = 10;   // seconds between samples
    time_t now = time(NULL);

    if (now > last + period) {
        float temperature;
        float pressure;

        temperature = get_temperature();
        pressure = get_pressure();

        Sensors["Temp"] = temperature;
        Sensors["Press"] = pressure / 100;
        last = now;
    }
}