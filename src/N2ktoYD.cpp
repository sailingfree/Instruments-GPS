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

static u_char SID = 0xff;             // Identify this source 

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


// Convert the date to days since 1970
uint32_t dateToDaysSince1970(prog_uint32_t year, uint32_t month, uint32_t day) {
    struct tm t;
    time_t t_of_day;

    t.tm_year = year;
    t.tm_mon = month;           // Month, where 0 = jan
    t.tm_mday = day;          // Day of the month
    t.tm_hour = 0;
    t.tm_min = 0;
    t.tm_sec = 0;
    t.tm_isdst = 0;        // Is DST on? 1 = yes, 0 = no, -1 = unknown
    t_of_day = mktime(&t);

    Serial.printf("seconds since the Epoch: %ld Y %d M %d D %d\n", (long) t_of_day, year, month, day);
    Serial.printf("Days since 1970 %d\n", t_of_day / (60 * 60 * 24));
    return t_of_day;
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

// Given a tinygps object construct an n2k gps messages and send it to the YD target
void sendYD(TinyGPSPlus& gps) {
    tN2kMsg N2kMsg;

    TinyGPSTime t = gps.time;
    TinyGPSDate gpsDate = gps.date;

    Serial.printf("Date value %d\n", gpsDate.value());
    uint16_t daysSince1970 = 1;
    double secondsSinceMidnight = t.second() + (t.minute() * 60) + (t.hour() * 60 * 60);
    double latitude = gps.location.lat();
    double longitude = gps.location.lng();
    double altitude = gps.altitude.meters();
    unsigned char nSatellites = gps.satellites.value();
    double hdop = gps.hdop.hdop();
    double pdop = 0.0;

    uint16_t d = dateToDaysSince1970(gps.date.year(), gps.date.month(), gps.date.day());

    SetN2kGNSS(N2kMsg,          //Reference to a N2kMsg Object
        SID,                    // SID
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
        SID,          // SID
        N2khr_true,         ///< heading true (eg. GNSS) direction 
        courseOverGround,
        speedOverground);

    GwSendYD(N2kMsg);

    // Send our product information
    SetN2kPGN126996(N2kMsg,
        1,
        16,
        "Naiad YD GPS",
        "1.0",
        "1.0.0",
        "1.0.0",
        1,
        1);
    GwSendYD(N2kMsg);
}
