#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <TFT_eSPI.h>
#include <StringStream.h>
#include <display.h>
#include <cyd_pins.h>
#include <cyd_i2c.h>
#include <bmp180_cyd.h>
#include <10dof.h>
#include <ArduinoJson.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>
#include "ESPmDNS.h"
#include <list>
#include <map>
#include <GwPrefs.h>
#include <GwShell.h>
#include <html_footer.h>
#include <html_header.h>
#include <imu.h>
#include <ekfNavINS.h>
#include <ublox_6m_config.h>
#include <defines.h>

//#include <uNavINS.h>

SET_LOOP_TASK_STACK_SIZE(16 * 1024);

// Global objects and variables
String host_name;
String Model = "Naiad N2K WiFi ";

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
const uint16_t ServerPort = 2222;  // Define the TCP port.
// This is where server sends NMea0183 data.
// Define the console to output to serial at startup.
// this can get changed later, eg in the gwshell.
Stream* Console = &Serial;

// Define the network servers
// The web server on port 80
WebServer webserver(80);

// TCP server for serving up NMEA0183
WiFiServer server(ServerPort, MaxClients);

// A JSON server to provide JSON formatted output
WiFiServer json(90);

// The telnet server for the shell.
WiFiServer telnet(23);

// A list of connected TCP clients for the TCP NMEA0183 port
template <class T>
using LinkedList = std::list<T>;
using tWiFiClientPtr = std::shared_ptr<WiFiClient>;
LinkedList<tWiFiClientPtr> clients;

IPAddress UnitIP;  // The address of this device. Could be client or AP

String WifiMode = "Unknown";
String WifiSSID = "Unknown";
String WifiIP = "Unknown";

ekfNavINS ekf;

//uNavINS Filter;

// Some ublox UBX code from here https://forum.arduino.cc/t/ubx-protocol-help-configuring-a-neo-6m-arduino-gps-module-fletcher-checksum/226600/10
/*
   This sample code demonstrates the normal use of a TinyGPSPlus (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   9600-baud serial GPS device connected to the GPIOS defined below.
*/
static const int RXPin = SERIAL_RX,
                TXPin = CYD_SCL_PIN;   // Shared with the i2c so only use one at a time

static const uint32_t GPSBaud = 38400;

#define WLAN_CLIENT 0  // Set to 1 to enable client network. 0 to act as AP only
#define USE_ARDUINO_OTA true
#define USE_MDNS true

// The TinyGPSPlus object used for local GNSS data
TinyGPSPlus gps;

// Custom messages PUBX
TinyGPSCustom pubxTime(gps, "PUBX", 2);

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

// The string stream object for building text
StringStream output;

bool docalibrate = false;

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms) {
    unsigned long start = millis();
    do {
        while (ss.available())
            gps.encode(ss.read());
    } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec, bool addToOutput = false, bool doprint = true) {
    StringStream local;

    if (!valid) {
        while (len-- > 1)
            local.print('*');
        local.print(' ');
    }
    else {
        local.print(val, prec);
        int vi = abs((int)val);
        int flen = prec + (val < 0.0 ? 2 : 1);  // . and -
        flen += vi >= 1000 ? 4 : vi >= 100 ? 3
            : vi >= 10 ? 2
            : 1;
        for (int i = flen; i < len; ++i)
            local.print(' ');
    }
    if (doprint) {
        Serial.print(local.data);
    }
    if (addToOutput) {
        output.data += local.data;
    }

    smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len, bool addToOutput = false, bool doprint = true) {
    StringStream local;

    char sz[32] = "*****************";
    if (valid)
        sprintf(sz, "%ld", val);
    sz[len] = 0;
    for (int i = strlen(sz); i < len; ++i)
        sz[i] = ' ';
    if (len > 0)
        sz[len - 1] = ' ';
    local.print(sz);
    if (doprint) {
        Serial.print(local.data);
    }
    if (addToOutput) {
        output.data += local.data;
    }
    smartDelay(0);
}

static void printDateTime(TinyGPSDate& d, TinyGPSTime& t, bool addToOutput = false, bool doprint = true) {
    StringStream local;

    if (!d.isValid()) {
        local.print(F("********** "));
    }
    else {
        char sz[32];
        sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
        local.print(sz);
    }

    if (!t.isValid()) {
        local.print(F("******** "));
    }
    else {
        char sz[32];
        sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
        local.print(sz);
    }

    if (doprint) {
        Serial.print(local.data);
    }
    if (addToOutput) {
        output.data += local.data;
    }
    printInt(d.age(), d.isValid(), 5, false, doprint);
    smartDelay(0);
}

static void printStr(const char* str, int len, bool addToOutput = false, bool doprint = true) {
    StringStream local;

    int slen = strlen(str);
    for (int i = 0; i < len; ++i)
        local.print(i < slen ? str[i] : ' ');

    if (doprint) {
        Serial.print(local.data);
    }
    if (addToOutput) {
        output.data += local.data;
    }
    smartDelay(0);
}

// Initialize the Arduino OTA
void initializeOTA() {
    // TODO: option to authentication (password)
    Console->println("OTA Started");

    // ArduinoOTA
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else  // U_SPIFFS
            type = "filesystem";
        Console->println("Start updating " + type);
        })
        .onEnd([]() {
        Console->println("\nEnd");
            })
        .onProgress([](unsigned int progress, unsigned int total) {
        Console->printf("Progress: %u%%\r", (progress / (total / 100)));
            })
        .onError([](ota_error_t error) {
        Console->printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR)
            Console->println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR)
            Console->println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR)
            Console->println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR)
            Console->println("Receive Failed");
        else if (error == OTA_END_ERROR)
            Console->println("End Failed");
            });

    // Begin
    ArduinoOTA.begin();
}
// HTML handlers
String html_start = HTML_start;  // Read HTML contents
String html_end = HTML_end;
void handleRoot() {
    webserver.send(200, "text/html", html_start + html_end);  // Send web page
}

void handleData() {
    String adcValue(random(100));
    webserver.send(200, "application/json", adcValue);  // Send ADC value only to client ajax request
}

void handleBoat() {
    StringStream boatData;
    boatData.printf("<pre>");
    boatData.printf("<h1>Boat Data</h1>");
    boatData.printf("<div class='info'>");
    // displayBoat(boatData);
    boatData.printf("</div>");

    boatData.printf("<h1>NMEA2000 Devices</h1>");
    boatData.printf("<div class='info'>");
    // ListDevices(boatData, true);
    boatData.printf("</div>");

    boatData.printf("<h1>Network</h1>");
    boatData.printf("<div class='info'>");
    // getNetInfo(boatData);
    boatData.printf("</div>");

    boatData.printf("<h1>System</h1>");
    boatData.printf("<div class='info'>");
    // getSysInfo(boatData);
    boatData.printf("</div>");

    boatData.printf("<h1>GPS</h1>");
    boatData.printf("<div class='info'>");
    // getGps(boatData);
    // getSatellites(boatData);

    boatData.printf("</div>");

    boatData.printf("<h1>Sensors</h1>");
    boatData.printf("<div class='info'>");
    // getSensors(boatData);
    boatData.printf("</div>");

    boatData.printf("</pre>");
    webserver.send(200, "text/html", html_start + boatData.data.c_str() + html_end);  // Send web page
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
            Console->printf("Connected to %s\n", wifiCreds[i].ssid.c_str());
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
        WiFi.setHostname(host_name.c_str());
        connectWifi();
    }

    if (WiFi.status() != WL_CONNECTED) {  // No client connection start AP
        // Init wifi connection
        Console->println("Start WLAN AP");  // WiFi Mode AP
        WiFi.mode(WIFI_AP);
        AP_ssid = host_name;
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

#ifdef USE_ARDUINO_OTA
    // Update over air (OTA)
    initializeOTA();
#endif

    // Register host name in mDNS
#if defined USE_MDNS

    if (MDNS.begin(host_name.c_str())) {
        Console->print("* MDNS responder started. Hostname -> ");
        Console->println(host_name);
    }

    // Register the services

#ifdef WEB_SERVER_ENABLED
    MDNS.addService("http", "tcp", 80);  // Web server
#endif

#ifndef DEBUG_DISABLED
    Console->println("Adding telnet");
    MDNS.addService("telnet", "tcp", 23);  // Telnet server of RemoteDebug, register as telnet
#endif

#endif  // MDNS

    // Start TCP server
    server.begin();

    // Start JSON server
    json.begin();

    // Start the telnet server
    telnet.begin();

    // Start Web Server
    webserver.on("/", handleRoot);
    webserver.on("/data", handleData);
    webserver.on("/boat", handleBoat);

    // webserver.onNotFound(handleNotFound);

    webserver.begin();
    Console->println("HTTP server started");

    // Init the shell
    initGwShell();

    Wire.setPins(CYD_SDA_PIN, CYD_SCL_PIN);
    Wire.setClock(100000);
    Wire.begin();

    // scan the bus
    scan_i2c_bus();

    // Init the display
    setup_display();

    // The bmp180 pressure sensor
    setup_bmp180();

    // The l3gd20 gyro
    setup_10dof();

    // Determin gyro bias
    setGyroBias();

    // IUM fusion
    imu_setup();

    Serial.println(F("FullExample.ino"));
    Serial.println(F("An extensive example of many interesting TinyGPSPlus features"));
    Serial.print(F("Testing TinyGPSPlus library v. "));
    Serial.println(TinyGPSPlus::libraryVersion());
    Serial.println(F("by Mikal Hart"));
    Serial.println();
    Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
    Serial.println(F("           (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
    Serial.println(F("----------------------------------------------------------------------------------------------------------------------------------------"));

    config_ublox(GPSBaud);
    
    ss.begin(GPSBaud);

    docalibrate = false;
    if (docalibrate) {
        calibrate2();
        while (1) {
            Serial.print(".");
            delay(1000);
        }
    }


}

static int looper = 0;

void loop() {
    //lsm303_calibrate_m();
    //return;
    //   lsm303_calibrate_a();
      // delay(100);
     //  return;
    bool print_gps = false;

    static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

    static double   lastLat = 0.0, lastLong = 0.0;
    static uint32_t lastTime = 0;
    static long     distance = 0;

    String Lat, Long, Time, Speed, Course, Dist, MaxSp, AvgSp, Hdop, Sats;
    double gpsLng;
    double gpsLat;

 


 

    // Serial.println(F("Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
    gpsLng = gps.location.lng();
    gpsLat = gps.location.lat();

    if(lastLat == 0.0) {
        lastLat = gpsLat;
    }
    if(lastLong == 0.0) {
        lastLong = gpsLng;
    }


    // Calculate distance travelled since last position change in metres
    if ((lastLat != gpsLat || lastLong != gpsLng)) {
        // position has changed

        unsigned long distanceSincelast =
            (unsigned long)TinyGPSPlus::distanceBetween(
                gpsLat,
                gpsLng,
                lastLat,
                lastLong);

        // Accumulate
        distance += distanceSincelast;

 

 //       Serial.printf("$$$$$ lat %g long %g lastlat %g lastlong %g dist %ld\n",
 //           gpsLat, gpsLng, lastLat, lastLong, distanceSincelast);

        lastLat = gpsLat;
        lastLong = gpsLng;
        lastTime = gps.time.value();
    }


    printInt(gps.satellites.value(), gps.satellites.isValid(), 5, true, print_gps);
    Sats = output.data;
    output.clear();

    printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1, true, print_gps);
    Hdop = output.data;
    output.clear();

    printFloat(gpsLat, gps.location.isValid(), 11, 6, true, print_gps);
    Lat = output.data;
    output.clear();

    printFloat(gpsLng, gps.location.isValid(), 12, 6, true, print_gps);
    Long = output.data;
    output.clear();

    printInt(gps.location.age(), gps.location.isValid(), 5, false, print_gps);
    printDateTime(gps.date, gps.time, true, print_gps);
    Time = output.data;
    output.clear();

    printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2, false, print_gps);
    printFloat(gps.course.deg(), gps.course.isValid(), 7, 2, true, print_gps);
    Course = output.data;
    output.clear();

    printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2, true, print_gps);
    Speed = output.data;
    output.clear();

    // Convert to nautical miles and to string
    Dist = String((float)MtoNM(distance));

    printInt(gps.charsProcessed(), true, 6, false, print_gps);
    printInt(gps.sentencesWithFix(), true, 10, false, print_gps);
    printInt(gps.failedChecksum(), true, 9, false, print_gps);
    if (print_gps) {
        Serial.print("\r");
    }

    smartDelay(1000);

    String space(" ");
    display_write(DISPDateTime, Time);
    display_write(DISPPosition, Lat + space + Long);
    display_write(DISPHDOP, String("HDOP ") + Hdop);
    display_write(DISPSats, String("Satellites ") + Sats);
    display_write(DISPSpeed, String("Speed ") + Speed + String(" Kts"));
    display_write(DISPCourse, String("Course ") + Course + String(" "));
    display_write(DISPDistance, String("Distance ") + Dist + String(" NM"));

 

    float temperature;
    temperature = get_temperature();
    float pressure;
    pressure = get_pressure();

    Serial.printf("temp : %f pressure %f\n", temperature, pressure);

    output.clear();
    String gyro, compass, accel;
    //  printGyrodata(output);
    gyro = output.data;
    output.clear();

 //   printAccel(output);
    accel = output.data;
    //   Serial.println(output.data);
    output.clear();

 //   printCompass(output);
    compass = output.data;
    output.clear();

    // printCompass3();

 //   printCompass2(output);
    compass = output.data;
    output.clear();
    // Serial.println(output.data);

    display_write(DISPCompass, String("Compass ") + compass);

#if 1
    LSM303::vector<float> g = gyroNorm();
    LSM303::vector<float> m = compassNorm();
    LSM303::vector<float> a = accelMss();
    gpsLng = gps.location.lng();
    gpsLat = gps.location.lat();
    int32_t gpsCourse = gps.course.value();  // Course in degress clockwise from north

    double vn = gps.speed.mps() * cos(DEG_TO_RAD * gpsCourse);  // GPS North velocity m/s
    double ve = gps.speed.mps() * sin(DEG_TO_RAD * gpsCourse);  // GPS East velocity m,/s
    double vd = 0.0;                                            // Down velocity - assume none on a boat!
    double lat = DEG_TO_RAD * gpsLat;  // GPS lat radians
    double lon = DEG_TO_RAD * gpsLng;  // GPS lon radians
    double alt = gps.altitude.meters();
    float gyrox = g.x * -1;
    float gyroy = g.y * -1;
    float gyroz = g.z * -1;
    float ax = a.x * -1;//a.x;
    float ay = a.y * -1;//a.y;
    float az = a.z * -1;//a.z;
    float hx = m.x;
    float hy = m.y;
    float hz = m.z;
    float roll, pitch, yaw = 0.0;


    std::tie(pitch, roll, yaw) = ekf.getPitchRollYaw(ax, ay, az, hx, hy, hz);
    uint64_t  now = millis();
    //now = time(NULL);
    //now = micros();
    //now = 0;   // Turn GPS off
       
    //    Filter.update(time(NULL), vn, ve, vd, lat, lon, alt, gyrox, gyroy, gyroz, ax, ay,az, hx, hy, hz);
    ekf.ekf_update(now, vn, ve, vd,
        lat, lon, alt,
        gyrox, gyroy, gyroz,
        ax, ay, az, hx, hy, hz);

    static time_t last = 0;
    if(millis() > last + 1000) {
        last = millis();
        smartDelay(0);  // Read the GPS
    printf("------------------------- %ld -------------------------- \n", time(NULL));
    Serial.printf("Latitude  : %2.7f %2.7f\n", lat, ekf.getLatitude_rad() * RAD_TO_DEG);
    Serial.printf("Longitute : %2.7f %2.7f\n", lon, ekf.getLongitude_rad() * RAD_TO_DEG);
    Serial.printf("Altitude  : %2.3f %2.3f\n", alt, ekf.getAltitude_m());
    Serial.printf("Speed (N) : %2.3f %2.3f\n", vn, ekf.getVelNorth_ms());
    Serial.printf("Speed (E) : %2.3f %2.3f\n", ve, ekf.getVelEast_ms());
    Serial.printf("Speed (D) : %2.3f %2.3f\n", vd, ekf.getVelDown_ms());
    Serial.printf("Roll 	  : %2.3f %2.3f\n", roll, ekf.getRoll_rad() * RAD_TO_DEG);
    Serial.printf("Pitch     : %2.3f %2.3f\n", pitch, ekf.getPitch_rad() * RAD_TO_DEG);
    Serial.printf("Yaw       : %2.3f %2.3f\n", (yaw * RAD_TO_DEG) + 180, (ekf.getHeadingConstrainAngle180_rad() * RAD_TO_DEG) + 180);
    Serial.printf("Mag X     : %f  %f\n", hx, hx);
    Serial.printf("Mag Y     : %f  %f\n", hy, hy);
    Serial.printf("Mag Z     : %f  %f\n", hz, hz);
    Serial.printf("Gyro X    : %f  %f\n", gyrox * DEG_TO_RAD, ekf.getGyroBiasX_rads());
    Serial.printf("Gyro Y    : %f  %f\n", gyroy * DEG_TO_RAD, ekf.getGyroBiasY_rads());
    Serial.printf("Gyro Z    : %f  %f\n", gyroz * DEG_TO_RAD, ekf.getGyroBiasZ_rads());
    Serial.printf("Accel X   : %f  %f\n", ax, ekf.getAccelBiasX_mss());
    Serial.printf("Accel Y   : %f  %f\n", ay, ekf.getAccelBiasY_mss());
    Serial.printf("Accel Z   : %f  %f\n", az, ekf.getAccelBiasZ_mss());
    Serial.printf("-----------------------------------------------------------------\n");
    }
#endif
    Serial.printf("Distance %d\n", distance);
}
