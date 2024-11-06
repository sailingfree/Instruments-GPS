//*****************************************************************************

#include <Arduino.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <GwShell.h>

extern WiFiServer telnetServer;

static WiFiClient telnetClient;

void disconnect() {
    telnetClient.stop();
}

void handleTelnet() {
    if (telnetClient && telnetClient.connected()) {
        // Got a connected client so use it
    } else {
        // See if there is a new connection and assign the new client
        telnetClient = telnetServer.available();
        if (telnetClient) {
            // Set up the client
            // telnetClient.setNoDelay(true); // More faster
            telnetClient.flush();  // clear input buffer, else you get strange characters
            setShellSource(&telnetClient);
        }
    }

    if (!telnetClient) {
        setShellSource(&Serial);
        return;
    }
}
