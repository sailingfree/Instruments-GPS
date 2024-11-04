#pragma once

#include <Arduino.h>
#include <N2kMsg.h>
#include <TinyGPSPlus.h>

// Init the NMEA0183 
void gpsInit();

// NMEA0183 handler
void handleNMEA0183();

// Send to Yacht device clients over udp using the cast address
void GwSendYD(const tN2kMsg &N2kMsg);

// Handle any YD messages received 
// Read the YD data, decode the N2K messages
void handleIncomingYD(void);

void sendYD(TinyGPSPlus & gps);