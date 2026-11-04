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
#include <SoftwareSerial.h>
#include <ublox_6m_config.h>

static int rates[] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 0};

void config_ublox(int GPSBaud, int rxpin, int txpin)
{
  // The aprkfun ublox object
  SFE_UBLOX_GPS myGPS;

  // The serial connection to the GPS device
  SoftwareSerial mySerial(rxpin, txpin);

  int rateIdx= 0;
  int baud;

  // Assume that the U-Blox GNSS is running at 9600 baud (the default)
  // Loop until we're in sync and then ensure it's at the requested speed baud.
  do {
    baud = rates[rateIdx];

    Serial.printf("GNSS: trying %d baud\n", baud);
    mySerial.begin(baud);
    if (myGPS.begin(mySerial) == true) break;

    // Not connected try next
    rateIdx++;
    if(rates[rateIdx] == 0) {
      rateIdx = 0;
    }
    delay(100);
  } while(1);

  // Connected now set the desired rate
  Serial.printf("GNSS: connected at %d baud\n", baud);

  // Connect at requested rate
  myGPS.setSerialRate(GPSBaud);

  Serial.printf("GNSS serial connected at %d baud\n", GPSBaud);
 
  myGPS.setUART1Output(COM_TYPE_NMEA); //Set the UART port to output NMEA only

  // set the dynamic mode to SEA and also sets the static lock threshold in cm/s
  // 1 knot is 51.4 cm/s
  myGPS.setDynamicModel(DYN_MODEL_SEA, 1100U, 15);
  myGPS.saveConfiguration();
  mySerial.end();
}