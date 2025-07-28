// send data to a yacht data YD destination

#include <Arduino.h>
#include <BoatData.h>
#include <GwPrefs.h>
#include <N2kMessages.h>
#include <N2kMsg.h>
#include <NMEA0183.h>
#include <NMEA0183Handlers.h>
#include <NMEA0183Msg.h>
#include <NMEA2000.h>
#include <TinyGPSPlus.h>
#include <WiFi.h>
#include <cyd_pins.h>
#include <main.h>
#include <ublox_6m_config.h>
#include <nmeaqueue.h>

// Define the pins used for the serial device on the cheap yellow
// display
static const int
    RXPin = SERIAL_RX,
    TXPin = CYD_SCL_PIN; // Shared with the i2c so only use one at a time

// The rate we set the GPS to for receiving the NMEA sentences
static const uint32_t GPSBaud = 38400;

// The NMEA0183 object
tNMEA0183 NMEA0183_3;

// Where we save the incoming data from the GPS receiver
tBoatData BoatData;

extern Stream *Console;

// UPD broadcast for Navionics, OpenCPN, etc.
// We send on this port
static int YDudpPort1 = 4445; // Non standard for local devices only

static int YDudpPort2 = 0; // If set then send to this as well.

// Create UDP instance for sending YD messages
WiFiUDP YDSendUDP;

// The buffer to construct YD messages
#define Max_YD_Message_Size 500
static char YD_msg[Max_YD_Message_Size] = "";

// Task handles
TaskHandle_t nmea0183Handle = NULL;
TaskHandle_t ydHandle = NULL;

// The main nmea0138 freertos task
// Initialises the gps and then readns and processes the sentences.
void handleNMEA0183(void *parameter) {

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

    // Use the Serial2 device and specify the config and pins
    Serial2.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    // Attach the serial device to the NMEA library
    NMEA0183_3.SetMessageStream(&Serial2);
    NMEA0183_3.Open();
 
    while (1) {
        // Read and parse any GPS messages converting them to n2k messages
        NMEA0183_3.ParseMessages();

        // Allow other threads to run
        vTaskDelay(500);
    }
}

// The YD sender task.
// Reads from the queue and sends the YD UDP packets
void handleSendYD(void * parm) {
    while(1) {
        tN2kMsg msg;

        // Read from the queue. waits until message queued
        dequeueMsg(msg);
        GwSendYD(msg);

        // Make sure the n2k messages get sent as YD messages at regular
        // intervals.
        processYD();

        // Allow other threads to run
        vTaskDelay(500);
    }
}

// Initilaise the nmea thread, queue and N2k sender
void gpsInit() {
    Serial.printf("Going to start task\n");

    setupMsgQueue();

    xTaskCreate(handleNMEA0183, "handleNMEA0183", 8000, NULL, PRIO_NMEA_TASK,
                & nmea0183Handle);

    xTaskCreate(handleSendYD, "YDSender", 8000, NULL, PRIO_YD_TASK, & ydHandle);
}

void stopTasks() {
    vTaskSuspendAll();
    if(nmea0183Handle) {
        Serial.printf("Deleting nmea task\n");
        vTaskSuspend(nmea0183Handle);
    }
    if(ydHandle) {
        Serial.printf("Deleting YD task\n");
        vTaskSuspend(ydHandle);
    }
}

/**
 * @name: N2kToYD_Can
 */
void N2kToYD_Can(const tN2kMsg &msg, char *MsgBuf) {
    time_t DaysSince1970 = 0;
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
    } else {
        canId = (canId | (msg.PGN << 8));
    }

    canId = (canId | (msg.Priority << 26));

    rawtime = (DaysSince1970 * 3600 * 24) +
              SecondsSinceMidnight; // Create time from GNSS time;
    ts = *localtime(&rawtime);
    strftime(time_str, sizeof(time_str), "%T.000", &ts); // Create time string

// Ignore truncation warning as snprintf limis the number of bytes written
#pragma GCC diagnostic ignored "-Wformat-truncation"

    snprintf(MsgBuf, 25, "%s R %08x", time_str, canId); // Set time and canID

    for (i = 0; i < len; i++) {
        snprintf(Byte, 4, " %02x", msg.Data[i]); // Add data fields
        strcat(MsgBuf, Byte);
    }
}

// Send a N2K message to Yacht device clients over udp using the broadcast address
void GwSendYD(const tN2kMsg &N2kMsg) {
    IPAddress udpAddress = WiFi.broadcastIP();
    N2kToYD_Can(N2kMsg, YD_msg);                   // Create YD message from PGN
    YDSendUDP.beginPacket(udpAddress, YDudpPort1); // Send to UDP
    YDSendUDP.printf("%s\r\n", YD_msg);
    YDSendUDP.endPacket();

    if (YDudpPort2) {
        YDSendUDP.beginPacket(udpAddress, YDudpPort2); // Send to UDP
        YDSendUDP.printf("%s\r\n", YD_msg);
        YDSendUDP.endPacket();
    }
}

// send the product information periodically
void handlesendN2K() {
    time_t now = millis();
    static time_t last = 0;
    return;

    if (now - last > 1000) {
        Serial.printf("sending product info\n");
        last = now;
        tN2kMsg N2kMsg;
        unsigned int N2kVersion = 1;
        unsigned int ProductCode = 2044;
        const char *ModelID = "Naiad GPS";
        const char *SwCode = "1.0";
        const char *ModelVersion = "1.0";
        const char *ModelSerialCode = "1";
        unsigned char CertificationLevel = 1;
        unsigned char LoadEquivalency = 1;

        SetN2kProductInformation(N2kMsg, N2kVersion, ProductCode, ModelID,
                                 SwCode, ModelVersion, ModelSerialCode,
                                 CertificationLevel, LoadEquivalency);

        GwSendYD(N2kMsg);

        // Heartbeat
        uint32_t timeInterval_ms = 1000;
        static uint32_t sequenceCounter = 1;
        SetHeartbeat(N2kMsg, timeInterval_ms, sequenceCounter++);
        GwSendYD(N2kMsg);

        // PGN list - transmit
        // Set the information for other bus devices, which messages we support
        static const unsigned long TransmitMessages[] PROGMEM = {
            127489L, // Engine dynamic
            127488L, // Engine fast dynamic
            127250,  // Vessel heading
            130577,  // Direction data
            130310,  // Outside environmental
            0};
        uint8_t Destination = 0;
        SetN2kPGNTransmitList(N2kMsg, Destination, TransmitMessages);
        GwSendYD(N2kMsg);

        // Configuration information
        const char *ManufacturerInformation = "Naiad";
        const char *InstallationDescription1 = "";
        const char *InstallationDescription2 = "";
        bool UsePgm = false;
        SetN2kConfigurationInformation(N2kMsg, ManufacturerInformation,
                                       InstallationDescription1 = 0,
                                       InstallationDescription2 = 0, UsePgm);
        GwSendYD(N2kMsg);
    }
}