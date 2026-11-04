#include <Arduino.h>

#include <WiFi.h>

#include <WebServer.h>

#include <StringStream.h>
#include <SysInfo.h>
#include <taskstats.h>

// HTML strings
#include <html/style.html>  // Must come before the content files
#include <html/index.html>
#include <html/login.html>

// Json library
#include <ArduinoJson.h>

void displayBoat(Stream & stream);

extern Stream * Console;

// The web server
WebServer server(80);

// Web server
void webServerSetup(void) {
    server.begin(80);
    if (WiFi.status() == WL_CONNECTED) {
        Console->println("Web server started");

        server.on("/", HTTP_GET, []() {
            server.send(200, "text/html", style +
                head_html +
                "<h2>GPS Instrument</h2>" +
                index_html +
                nav +
                footer_html);
            server.sendHeader("Connection", "close");
            });

        server.on("/system", HTTP_GET, []() {
            StringStream net, sys, msgs,psinfo;
            getNetInfo(net);
            getSysInfo(sys);
            getTaskStats(psinfo);
            server.sendHeader("Connection", "close");
            server.send(200, "text/html", style +
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
            server.send(200, "text/html", style +
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
            getSatellites(gps);
            server.sendHeader("Connection", "close");
            server.send(200, "text/html", style +
                head_html +
                "<h2>GPS Instrument</h2>" +
                nav +
                "<h1>GPS</h1>" +
                "<pre>" +
                gps.data +
                "</pre>" +
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