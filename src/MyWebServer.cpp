#include <Arduino.h>

#include <WiFi.h>

#include <WebServer.h>

#include <StringStream.h>
#include <SysInfo.h>
#include <taskstats.h>

// HTML strings
#include <html/index.html>

// Json library
#include <ArduinoJson.h>

#include <SPIFFS.h>


void displayBoat(Stream & stream);

extern Stream * Console;

// The web server
WebServer server(80);

void listSPIFFSFiles(Stream & s) {
    s.printf("SPIFFS Directory\n");
    File root = SPIFFS.open("/");
    File file = root.openNextFile();
    while (file) {
        s.printf("%s\t%d bytes\n", file.name(), file.size());
        file = root.openNextFile(); 
    }
}

// Web server
void webServerSetup(void) {
    SPIFFS.begin(true);

    listSPIFFSFiles(Serial);

    server.begin(80);
    if (WiFi.status() == WL_CONNECTED) {
        Console->println("Web server started");

        server.serveStatic("/favicon.ico", SPIFFS, "/favicon.png");
        server.serveStatic("/style.css", SPIFFS, "/style.css");

        server.on("/", HTTP_GET, []() {
            server.send(200, "text/html",
                head_html +
                "<h2>GPS Instrument</h2>" +
                nav +
                footer_html);
            });

        server.on("/system", HTTP_GET, []() {
            StringStream net, sys, msgs,psinfo;
            getNetInfo(net);
            getSysInfo(sys);
            getTaskStats(psinfo);
            server.sendHeader("Connection", "close");
            server.send(200, "text/html",
                head_html +
                "<h2>GPS Instrument</h2>" +
                nav +
                "<h1>System</h1>" +
                "<pre>" +
                net.data +
                sys.data +
                msgs.data +
                psinfo.data +
                "</pre>" +
                nav +
                footer_html);
            });

            server.on("/boat", HTTP_GET, []() {
            StringStream boat;
            displayBoat(boat);
            server.sendHeader("Connection", "close");
            server.send(200, "text/html",
                head_html +
                "<h2>GPS Instrument</h2>" +
                nav +
                "<h1>Boat</h1>" +
                "<pre>" +
                boat.data +
                "</pre>" +
                nav +
                footer_html);
            });

            server.on("/GPS", HTTP_GET, []() {
            StringStream gps;   
            StringStream chart;            
            getSatellites(gps);
            getSatelliteChart(chart);

            server.sendHeader("Connection", "close");
            server.send(200, "text/html",
                head_html +
                "<h2>GPS Instrument</h2>" +
                nav +
                "<h1>GPS</h1>" +
                "<pre>" +
                gps.data +
                "</pre>" +
                chart.data +
                nav +
                footer_html);
            });
        delay(10);
        server.begin();
    }
}

void webServerWork() {
    if (WiFi.status() == WL_CONNECTED) {
        server.handleClient();
    }
}