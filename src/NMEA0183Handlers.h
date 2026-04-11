/* 
NMEA0183Handlers.h

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/
 
#pragma once

#include <Arduino.h>
#include <time.h>
#include <NMEA0183.h>
#include <NMEA0183Msg.h>
#include <NMEA0183Messages.h>
#include <NMEA2000.h>
#include "BoatData.h"
#include <N2ktoYD.h>
#include <map>

// Struct that describes the messages we are interested in, when they have been 
// received and their status
// This is used to make sure N2k messages are sent regularly
struct N2kMessages {
    tN2kMsg msg;
    time_t  lastseen;
    bool valid;
};

typedef enum {
    M_RMC,
    M_GGA,
    M_VTG,
    M_GLL,
    M_GSA,
    M_GSV,
    M_MAX
} MSGTypes;

// If we havn't seen the GPS messages in this time 
// mark them as invalid and don't send
// this is milliseconds
#define VALID_GPS_PERIOD  15000

// period at which to send GPS related YD messages in milliseconds
#define SEND_YD_PERIOD    500

// Map for the satellite informations
extern std::map<int, tGSV> Satellites;

// Valid satellite map information
extern bool validGSV;

void InitNMEA0183Handlers(tBoatData *_BoatData);
void DebugNMEA0183Handlers(Stream* _stream);
void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg);
void processYD();
