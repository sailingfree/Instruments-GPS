//#include <TinyGPSPlus.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Handlers.h>
#include <BoatData.h>
#include <SoftwareSerial.h>
#include <TFT_eSPI.h>
#include <StringStream.h>
#include <display.h>
#include <cyd_pins.h>
#include <cyd_i2c.h>
#include <bmp180_cyd.h>
#include <DNSServer.h>
#include <WiFi.h>
#include "ESPmDNS.h"
#include <list>
#include <map>
#include <GwPrefs.h>
#include <GwShell.h>
#include <ublox_6m_config.h>
#include <defines.h>
#include <N2ktoYD.h>

extern tBoatData BoatData;

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// Global objects and variables
String hostName;
String Model = "Naiad N2K GPS ";

// Map for the wifi access points
typedef struct {
    String ssid;
    String pass;
} WiFiCreds;

static const uint16_t MaxAP = 2;
WiFiCreds wifiCreds[MaxAP];

// Wifi cofiguration Client and Access Point
String AP_password;  // AP password  read from preferences
String AP_ssid;      // SSID for the AP constructed from the hostname

// Put IP address details here
const IPAddress AP_local_ip(192, 168, 15, 1);  // Static address for AP
const IPAddress AP_gateway(192, 168, 15, 1);
const IPAddress AP_subnet(255, 255, 255, 0);

int wifiType = 0;  // 0= Client 1= AP
const size_t MaxClients = 10;

// Define the console to output to serial at startup.
// this can get changed later, eg in the gwshell.
Stream* Console = &Serial;

// Define the network servers

// The telnet server for the shell.
WiFiServer telnetServer(23);

// A list of connected TCP clients for the TCP NMEA0183 port
template <class T>
using LinkedList = std::list<T>;
using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clients;

IPAddress UnitIP;  // The address of this device.
String SSID = "Unknown";        // The SSID of the AP connected to or ourself if an AP

String WifiMode = "Unknown";
String WifiSSID = "Unknown";
String WifiIP = "Unknown";


#define WLAN_CLIENT 1  // Set to 1 to enable client network. 0 to act as AP only
#define USE_MDNS true

// The string stream object for building text
StringStream output;

static void printFloat(float val, int len, int prec, StringStream& target) {
    if (val == NMEA0183DoubleNA) {
        target.print("---");
    }
    else {
        target.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3
            : vi >= 10 ? 2
            : 1;
        for (int i = flen; i < len; ++i)
            target.print(' ');
    }
}

static void printInt(unsigned long val, int len, StringStream& target) {
    StringStream local;

    target.printf("%d", val);
}

// Connect to a wifi AP
// Try all the configured APs
bool connectWifi() {
    int wifi_retry = 0;

    Serial.printf("There are %d APs to try\n", MaxAP);

    for (int i = 0; i < MaxAP; i++) {
        Serial.printf("\nTrying %s\n", wifiCreds[i].ssid.c_str());
        WiFi.disconnect();
        WiFi.mode(WIFI_OFF);
        WiFi.mode(WIFI_STA);
        display_write(DISPWifi, String("Trying SSID ") + wifiCreds[i].ssid);
        WiFi.begin(wifiCreds[i].ssid.c_str(), wifiCreds[i].pass.c_str());
        wifi_retry = 0;

        while (WiFi.status() != WL_CONNECTED && wifi_retry < 20) {  // Check connection, try 5 seconds
            wifi_retry++;
            delay(500);
            Console->print(".");
        }
        Console->println("");
        if (WiFi.status() == WL_CONNECTED) {
            WifiMode = "Client";
            WifiSSID = wifiCreds[i].ssid;
            WifiIP = WiFi.localIP().toString();
            SSID = wifiCreds[i].ssid;
            Console->printf("Connected to %s\n", wifiCreds[i].ssid.c_str());
            display_write(DISPWifi, String("Connect to ") + wifiCreds[i].ssid);
            return true;
        }
        else {
            Console->printf("Can't connect to %s\n", wifiCreds[i].ssid.c_str());
        }
    }
    return false;
}

void disconnectWifi() {
    Console = &Serial;
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    WiFi.mode(WIFI_STA);
    WifiMode = "Not connected";
}

void setup() {
    Serial.begin(115200);

    GwPrefsInit();

    Wire.setPins(CYD_SDA_PIN, CYD_SCL_PIN);
    Wire.setClock(100000);
    Wire.begin();

    // scan the bus
    scan_i2c_bus();

    // Init the display
    setup_display();

    display_write(DISPWifi, String("Initialising WiFi"));

    // setup the WiFI map from the preferences
    wifiCreds[0].ssid = GwGetVal(SSID1);
    wifiCreds[0].pass = GwGetVal(SSPW1);
    wifiCreds[1].ssid = GwGetVal(SSID2);
    wifiCreds[1].pass = GwGetVal(SSPW2);

    // Setup params if we are to be an AP
    AP_password = GwGetVal(GWPASS);

    if (WLAN_CLIENT == 1) {
        Console->println("Start WLAN Client");  // WiFi Mode Client
        delay(100);
        WiFi.setHostname(hostName.c_str());
        connectWifi();
    }

    if (WiFi.status() != WL_CONNECTED) {  // No client connection start AP
        // Init wifi connection
        Console->println("Start WLAN AP");  // WiFi Mode AP
        WiFi.mode(WIFI_AP);
        AP_ssid = hostName;
        WiFi.softAP(AP_ssid.c_str(), AP_password.c_str());
        delay(100);
        WiFi.softAPConfig(AP_local_ip, AP_gateway, AP_subnet);
        UnitIP = WiFi.softAPIP();
        Console->println("");
        Console->print("AP IP address: ");
        Console->println(UnitIP);
        wifiType = 1;
        WifiMode = "AP";
        WifiIP = UnitIP.toString();
        WifiSSID = AP_ssid;

    }
    else {  // Wifi Client connection was sucessfull

        Console->println("");
        Console->println("WiFi client connected");
        Console->println("IP client address: ");
        Console->println(WiFi.localIP());
        UnitIP = WiFi.localIP();
    }

    // Register host name in mDNS

    if (MDNS.begin(hostName.c_str())) {
        Console->print("* MDNS responder started. Hostname -> ");
        Console->println(hostName);
    }

    // Register the services
    // Start the telnet server
    telnetServer.begin();

    // Init the shell
    initGwShell();
    setShellSource(&Serial);


    // The bmp180 pressure sensor
    setup_bmp180();

    gpsInit();
}


void loop() {
    StringStream Lat, Long, Time, Speed, Course, Dist, MaxSp, AvgSp, Hdop, Sats;

    // read any NMEA0183 messages, decode them and update the BoatData object
    handleNMEA0183();

    if (BoatData.changed) {
        printFloat(BoatData.Latitude, 12, 6, Lat);
        printFloat(BoatData.Longitude, 12, 6, Long);
        printFloat(BoatData.SOG, 6, 2, Speed);
        printFloat(BoatData.COG, 6, 1, Course);
        printFloat(BoatData.HDOP, 6, 2, Hdop);
        printInt(BoatData.SatelliteCount, 6, Sats);

        time_t gpstime = BoatData.GPSTime + (BoatData.DaysSince1970 * 24 * 60 * 60);

        struct tm* tm;
        tm = gmtime(&gpstime);
        Time.printf("%02d:%02d:%02d %d-%d-%d\n", tm->tm_hour, tm->tm_min, tm->tm_sec, tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday);


        String space(" ");
        display_write(DISPWifi, SSID + space + UnitIP.toString());
        display_write(DISPDateTime, Time.data);
        display_write(DISPPosition, Lat.data + space + Long.data);
        display_write(DISPHDOP, String("HDOP ") + Hdop.data);
        display_write(DISPSats, String("Satellites ") + Sats.data);
        display_write(DISPSpeed, String("Speed ") + Speed.data + String(" Kts"));
        display_write(DISPCourse, String("Course ") + Course.data + String(" "));
        BoatData.changed = false;
    }


    handleShell();
}
