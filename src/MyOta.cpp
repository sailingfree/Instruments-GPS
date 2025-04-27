#include <Arduino.h>
#include <ArduinoOTA.h>
#include <MyOta.h>

// Initialize the Arduino OTA
void initOTA() {
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

void handleOta() {
        ArduinoOTA.handle();
}
