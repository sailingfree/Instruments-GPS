/*
  configure the library and U-Blox for serial port use as well as
  switching the module from the default 9600 baud to 38400.

  sets the navigation type to sea and sets the lock speed

  This uses the sparkfun arduino library local copy.

  I don't use this library for the nav info though as I dont get reliable results.

  After the setup function has been called the calling program can restart the
  software serial object without write (TX pin).
*/



#include <Arduino.h>
#include <SparkFun_Ublox_Arduino_Library_Series_6_7.h>
#include <cyd_pins.h>
#include <ublox_6m_config.h>

static const int RXPin = SERIAL_RX,
                TXPin = CYD_SCL_PIN;   // Shared with the i2c so only use one at a time

void config_ublox(uint16_t requestedGPSBaud)
{
  // The sparkfun ublox object
  SFE_UBLOX_GPS myGPS;

  // The serial connection to the GPS device
  //SoftwareSerial Serial2(RXPin, TXPin);


  //Assume that the U-Blox GNSS is running at 9600 baud (the default)
  //Loop until we're in sync and then ensure it's at the requested speed baud.
  do {
    Serial.printf("GNSS: trying %d baud\n", requestedGPSBaud);
    Serial2.begin(requestedGPSBaud, SERIAL_8N1, RXPin, TXPin);
    if (myGPS.begin(Serial2) == true) break;

    delay(100);
    Serial.printf("GNSS: trying %d baud\n", FACTORY_GPSBAUD);
    Serial2.begin(FACTORY_GPSBAUD);
    if (myGPS.begin(Serial2) == true) {
        Serial.printf("GNSS: connected at %d baud", FACTORY_GPSBAUD);
        myGPS.setSerialRate(requestedGPSBaud);
        delay(100);
    } else {
        //myGPS.factoryReset();
        delay(2000); //Wait a bit before trying again to limit the Serial output
    }
  } while(1);
  Serial.printf("GNSS serial connected at %d baud\n", requestedGPSBaud);
 
  //Set the UART port to output NMEA only
  myGPS.setUART1Output(COM_TYPE_NMEA); 

  // set the dynamic mode to SEA and also sets the static lock threshold in cm/s
  // 1 knot is 51.4 cm/s
  myGPS.setDynamicModel(DYN_MODEL_SEA, 1100U, 15);
  myGPS.saveConfiguration();
  Serial2.end();
}