// send data to a yacht data YD destination

#include <Arduino.h>
#include <N2kMsg.h>
#include <Seasmart.h>
#include <WiFi.h>
//#include <YDtoN2kUDP.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Handlers.h>
#include <BoatData.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <TinyGPSPlus.h>
#include <ublox_6m_config.h>
#include <cyd_pins.h>
#include <SoftwareSerial.h>

//#include <GwDefs.h>

// Some ublox UBX code from here https://forum.arduino.cc/t/ubx-protocol-help-configuring-a-neo-6m-arduino-gps-module-fletcher-checksum/226600/10
/*
   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device connected to the GPIOS defined below.
*/
static const int RXPin = SERIAL_RX,
                TXPin = CYD_SCL_PIN;   // Shared with the i2c so only use one at a time

static const uint32_t GPSBaud = 9600;

// The NMEA0183 object
tNMEA0183 NMEA0183_3;
tBoatData BoatData;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500

extern Stream* Console;

// UPD broadcast for Navionics, OpenCPN, etc.
// We listenon this port
static const int YDudpPort = 4445;  // Non standard for local devices only

// Create UDP instance for sending YD messages
WiFiUDP     YDSendUDP;

// The buffer to construct YD messages
#define Max_YD_Message_Size 500
static char YD_msg[Max_YD_Message_Size] = "";

void gpsInit() {
    
// Leave this at 9600 for reliability. The software serial dropped data    
//    config_ublox(GPSBaud);
    
    ss.begin(GPSBaud);

    // Setup NMEA0183 ports and handlers
    InitNMEA0183Handlers(&BoatData);
    NMEA0183_3.SetMsgHandler(HandleNMEA0183Msg);

    NMEA0183_3.SetMessageStream(&ss);
    NMEA0183_3.Open();

}

void handleNMEA0183() {
    NMEA0183_3.ParseMessages();
}


/**
 * @name: N2kToYD_Can
 */
void N2kToYD_Can(const tN2kMsg& msg, char* MsgBuf) {
    time_t  DaysSince1970 = 0;
    time_t SecondsSinceMidnight = 0;

    int i, len;
    uint32_t canId = 0;
    char time_str[20];
    char Byte[5];
    unsigned int PF;
    time_t rawtime;
    struct tm ts;
    len = msg.DataLen;
    if (len > 134) {
        len = 134;
        Console->printf("Truncated from %d to 134\n", len);
    }

    // Set CanID

    canId = msg.Source & 0xff;
    PF = (msg.PGN >> 8) & 0xff;

    if (PF < 240) {
        canId = (canId | ((msg.Destination & 0xff) << 8));
        canId = (canId | (msg.PGN << 8));
    }
    else {
        canId = (canId | (msg.PGN << 8));
    }

    canId = (canId | (msg.Priority << 26));

    rawtime = (DaysSince1970 * 3600 * 24) + SecondsSinceMidnight;  // Create time from GNSS time;
    ts = *localtime(&rawtime);
    strftime(time_str, sizeof(time_str), "%T.000", &ts);  // Create time string

    snprintf(MsgBuf, 25, "%s R %08x", time_str, canId);  // Set time and canID

    for (i = 0; i < len; i++) {
        snprintf(Byte, 4, " %02x", msg.Data[i]);  // Add data fields
        strcat(MsgBuf, Byte);
    }
}

// Send to Yacht device clients over udp using the cast address
void GwSendYD(const tN2kMsg& N2kMsg) {
    IPAddress udpAddress = WiFi.broadcastIP();
    N2kToYD_Can(N2kMsg, YD_msg);             // Create YD message from PGN
    YDSendUDP.beginPacket(udpAddress, YDudpPort);  // Send to UDP
    YDSendUDP.printf("%s\r\n", YD_msg);
    YDSendUDP.endPacket();

    char buf[MAX_NMEA2000_MESSAGE_SEASMART_SIZE];
    if (N2kToSeasmart(N2kMsg, millis(), buf, MAX_NMEA2000_MESSAGE_SEASMART_SIZE) == 0) return;
}

