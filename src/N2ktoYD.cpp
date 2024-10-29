// send data to a yacht data YD destination

#include <Arduino.h>
#include <N2kMsg.h>
#include <Seasmart.h>
#include <WiFi.h>
//#include <YDtoN2kUDP.h>
#include <NMEA2000.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <TinyGPSPlus.h>

//#include <GwDefs.h>

#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500

extern Stream* Console;

// UPD broadcast for Navionics, OpenCPN, etc.
// We listenon this port
static const int YDudpPort = 4444;  // port 4444 is for the Yacht devices interface

// Create UDP instance for sending YD messages
WiFiUDP     YDSendUDP;

// The buffer to construct YD messages
#define Max_YD_Message_Size 500
static char YD_msg[Max_YD_Message_Size] = "";

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
    udpAddress.fromString("192.168.1.255");
    N2kToYD_Can(N2kMsg, YD_msg);             // Create YD message from PGN
    YDSendUDP.beginPacket(udpAddress, YDudpPort);  // Send to UDP
    YDSendUDP.printf("%s\r\n", YD_msg);
    YDSendUDP.endPacket();

    char buf[MAX_NMEA2000_MESSAGE_SEASMART_SIZE];
    if (N2kToSeasmart(N2kMsg, millis(), buf, MAX_NMEA2000_MESSAGE_SEASMART_SIZE) == 0) return;
}

// Given a tinygps object construct an n2k gps messages and send it to the YD target
void sendYD(TinyGPSPlus& gps) {
    tN2kMsg N2kMsg;

    TinyGPSTime t = gps.time;
    uint16_t daysSince1970 = 1;
    double secondsSinceMidnight = t.second() + (t.minute() * 60) + (t.hour() * 60 * 60);
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    double altitude = gps.altitude.meters();
    unsigned char nSatellites = gps.satellites.value();
    double hdop = gps.hdop.hdop();
    double pdop = 0.0;

    SetN2kGNSS(N2kMsg,          //Reference to a N2kMsg Object
        1,                      // SID
        daysSince1970,          // UTC date in resolution of 1 day
        secondsSinceMidnight,   // UTC time in seconds
        latitude,               // Latitude in degrees
        longitude,              // Longitude in degrees
        altitude,               // Altitude in metres
        N2kGNSSt_GPS,           // GNSS type
        N2kGNSSm_GNSSfix,       // GNSS method
        nSatellites,            // number of satellites
        hdop,                   // Horizontal Dilution Of Precision in meters
        pdop,                   // Probable dilution of precision in meters.
        0,                      // Geoidal separation in meters
        0,                      // Number of Reference Stations
        N2kGNSSt_GPS,           // Reference station type
        0,                     // Reference Station ID 
        0);                     // Age of DGNSS Corrections

    GwSendYD(N2kMsg);


    // And the course and speed from the GPS
    double courseOverGround = gps.course.deg() * DEG_TO_RAD;     // Course Over Ground in radians
    double speedOverground = gps.speed.mps();                   // Speed Over Ground in m/s
    
    SetN2kCOGSOGRapid(N2kMsg,
        1,          // SID
        N2khr_true,         ///< heading true (eg. GNSS) direction 
        courseOverGround,
        speedOverground);

    GwSendYD(N2kMsg);
}
