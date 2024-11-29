// send data to a yacht data YD destination

#include <Arduino.h>
#include <N2kMsg.h>
#include <Seasmart.h>
#include <WiFi.h>
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
#include <GwPrefs.h>
#include <main.h>

// Define the pis used for the software serial device on the cheap yellow display
static const int RXPin = SERIAL_RX,
TXPin = CYD_SCL_PIN;   // Shared with the i2c so only use one at a time

// Leave this at 9600 for reliability. 
// The software serial dropped data at higher rates
static const uint32_t GPSBaud = 9600;

// The NMEA0183 object
tNMEA0183 NMEA0183_3;

// Where we save the incoming data from the GPS receiver
tBoatData BoatData;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

#define MAX_NMEA2000_MESSAGE_SEASMART_SIZE 500

extern Stream* Console;

// UPD broadcast for Navionics, OpenCPN, etc.
// We send on this port
static int YDudpPort1 = 4445;  // Non standard for local devices only

static int YDudpPort2 = 0;      // If set then send to this as well.

// Create UDP instance for sending YD messages
WiFiUDP     YDSendUDP;

// The buffer to construct YD messages
#define Max_YD_Message_Size 500
static char YD_msg[Max_YD_Message_Size] = "";

// The main nmea0138 freertos task
// Initialises the gps and then readns and processes the sentences.
void handleNMEA0183(void* parameter) {

    // Config the ublox 
    config_ublox(GPSBaud);

    // see if there is an alternate port set
    String ydvalstr1 = GwGetVal(GWYDPORT1, "4445");
    String ydvalstr2 = GwGetVal(GWYDPORT2, "0");
    int ydval1 = ydvalstr1.toInt();
    int ydval2 = ydvalstr2.toInt();
    Serial.printf("YD Port 1 %d Port 2 %d\n", ydval1, ydval2);
    if (ydval1 > 1000 && ydval1 < 65535) {
        YDudpPort1 = ydval1;
    }
    if (ydval2 > 1000 && ydval2 < 65535) {
        YDudpPort2 = ydval2;
    }

    // Setup NMEA0183 ports and handlers
    InitNMEA0183Handlers(&BoatData);
    NMEA0183_3.SetMsgHandler(HandleNMEA0183Msg);

    NMEA0183_3.SetMessageStream(&ss);
    NMEA0183_3.Open();
    ss.begin(GPSBaud);

    while (1) {
        // Read and parse any GPS messages converting them to n2k messages
        NMEA0183_3.ParseMessages();

        // Make sure the n2k messages get sent as YD messages at regular intervals.
        processYD();

        // Allow other threads to run
        vTaskDelay(10 / portTICK_PERIOD_MS);  
    }
}

// Initilaise the nmea thread
void gpsInit() {
    Serial.printf("Going to start task\n");
    xTaskCreate(handleNMEA0183,
        "handleNMEA0183",
        8000,
        NULL,
        PRIO_NMEA_TASK,
        NULL);
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
    YDSendUDP.beginPacket(udpAddress, YDudpPort1);  // Send to UDP
    YDSendUDP.printf("%s\r\n", YD_msg);
    YDSendUDP.endPacket();

    if (YDudpPort2) {
        YDSendUDP.beginPacket(udpAddress, YDudpPort2);  // Send to UDP
        YDSendUDP.printf("%s\r\n", YD_msg);
        YDSendUDP.endPacket();
    }
}

