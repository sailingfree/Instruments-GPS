/*
NMEA0183Handlers.cpp

2015 Copyright (c) Kave Oy, www.kave.fi  All right reserved.

Author: Timo Lappalainen

  This library is free software; you can redistribute it and/or
  modify it as you like.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include <N2kMsg.h>
#include <NMEA2000.h>
#include <N2kMessages.h>
#include <NMEA0183Messages.h>
#include "NMEA0183Handlers.h"
#include <N2ktoYD.h>
#include <map>

// Map for the satellite informations
std::map<int, tGSV> Satellites;
bool validGSV = false;  // true indicates we have had all emssages for a set


// define the message structs.
N2kMessages n2kMessages[M_MAX];

struct tNMEA0183Handler {
    const char* Code;
    void (*Handler)(const tNMEA0183Msg& NMEA0183Msg);
};

// Predefinition for functions to make it possible for constant definition for NMEA0183Handlers
void HandleRMC(const tNMEA0183Msg& NMEA0183Msg);
void HandleGGA(const tNMEA0183Msg& NMEA0183Msg);
void HandleVTG(const tNMEA0183Msg& NMEA0183Msg);
void HandleGLL(const tNMEA0183Msg& NMEA0183Msg);
void HandleGSA(const tNMEA0183Msg& NMEA0183Msg);
void HandleGSV(const tNMEA0183Msg& NMEA0183Msg);

// Internal variables
tBoatData* pBD = 0;
Stream* NMEA0183HandlersDebugStream = NULL;

tNMEA0183Handler NMEA0183Handlers[] = {
    {"GGA", &HandleGGA},
    {"VTG", &HandleVTG},
    {"RMC", &HandleRMC},
    {"GSA", &HandleGSA},
    {"GSV", &HandleGSV},
    {"GLL", &HandleGLL},
    {0, 0} };


// set a n2k message in the message record and update its status
void setn2kMessage(MSGTypes type, tN2kMsg & msg) {
    time_t now = millis();

    if(type < 0 || type > M_MAX) {
        return;
    }

    if(!n2kMessages[type].valid) {
        Serial.printf("Acquired PGN %ld at %ld\n", msg.PGN, now);
    }

    n2kMessages[type].msg = msg;
    n2kMessages[type].valid = true;
    n2kMessages[type].lastseen = now;
}


void InitNMEA0183Handlers(tBoatData* _BoatData) {
    pBD = _BoatData;
}

void DebugNMEA0183Handlers(Stream* _stream) {
    NMEA0183HandlersDebugStream = _stream;
}

tN2kGNSSmethod GNSMethofNMEA0183ToN2k(int Method) {
    switch (Method) {
    case 0:
        return N2kGNSSm_noGNSS;
    case 1:
        return N2kGNSSm_GNSSfix;
    case 2:
        return N2kGNSSm_DGNSS;
    default:
        return N2kGNSSm_noGNSS;
    }
}

void HandleNMEA0183Msg(const tNMEA0183Msg& NMEA0183Msg) {
    int iHandler;
    // Find handler
    for (iHandler = 0; NMEA0183Handlers[iHandler].Code != 0 && !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code); iHandler++);
    if (NMEA0183Handlers[iHandler].Code != 0) {
        NMEA0183Handlers[iHandler].Handler(NMEA0183Msg);
    }
    else {
        Serial.printf("No handler for message %s\n", NMEA0183Msg.MessageCode());
    }
}

// Return true if the lat and long are reasonable
// Checks for zeroand then compares lat and lon to last good
// and makes sure they are close.
// Close is as follows:
// Assume we travel at a maximum speed of 25kts (yes yes I know)
// and we get a GPS message every second
// we will have travelled about 12 metres
// From the Garmin site we can see the following:
// decimal place degrees  Distance
// 0             1.0      111km
// 1             0.1      11.1km
// 2             0.01     1.11km
// 3             0.001    111m
// 4             0.0001   11m
//
// So if the current and last differ by more than 0.0001 then the reading is suspect.
//
bool validLatLong(double lat, double lon) {
    bool latResult = false;
    bool lonResult = false;
    static double lastLat = 0.0;
    static double lastLon = 0.0;
    const double delta = 0.0001;

    if(lat != 0.0) {
        if(lastLat == 0.0) {
            // set first seen non zero
            lastLat = lat;
            latResult = true;
        } else {
            // see if reasonable
            if(fabs(lat - lastLat) <= delta) {
                latResult = true;
                lastLat = lat;
            } else {
                Serial.printf("Got suspect lat %f, last %f\n", lat, lastLat);
            }
        }
    } else {
        Serial.printf("Got zero lat\n");
    }
    if(lon != 0.0) {
        if(lastLon == 0.0) {
            // set first seen non zero
            lastLon = lat;
            lonResult = true;
        } else {
            // see if reasonable
            if(fabs(lon - lastLon) <= delta) {
                lonResult = true;
                lastLon = lat;
            } else {
                Serial.printf("Got suspect lon %f last %f\n", lon, lastLon);
            }
        }
    } else{
        Serial.printf("Got zero lon\n");
    }

    return latResult & lonResult;
}


// NMEA0183 message Handler functions

// Position, velocity, and time
void HandleRMC(const tNMEA0183Msg& NMEA0183Msg) {
    if (pBD == 0) return;

    if (NMEA0183ParseRMC_nc(NMEA0183Msg, pBD->GPSTime, pBD->Latitude, pBD->Longitude, pBD->COG, pBD->SOG, pBD->DaysSince1970, pBD->Variation)) {
        pBD->changed = true;
        pBD->countRMC++;

        // check we have snsible values
        if (!N2kIsNA(pBD->GPSTime) && !N2kIsNA(pBD->Latitude) && !N2kIsNA(pBD->Longitude) && !N2kIsNA(pBD->COG) && !N2kIsNA(pBD->SOG) && validLatLong(pBD->Latitude, pBD->Longitude)) {
            tN2kMsg msg;

            SetN2kCOGSOGRapid(msg, 1, N2khr_true, pBD->COG, pBD->SOG);
            setn2kMessage(M_RMC, msg);
        }
        else {
            n2kMessages[M_RMC].valid = false;
        }
    }
    else if (NMEA0183HandlersDebugStream != 0) {
        pBD->countFail++;
        NMEA0183HandlersDebugStream->println("Failed to parse RMC");
    }
}

// Time, position, and fix related data
void HandleGGA(const tNMEA0183Msg& NMEA0183Msg) {
    if (pBD == 0) return;

    if (NMEA0183ParseGGA_nc(NMEA0183Msg, pBD->GPSTime, pBD->Latitude, pBD->Longitude,
        pBD->GPSQualityIndicator, pBD->SatelliteCount, pBD->HDOP, pBD->Altitude, pBD->GeoidalSeparation,
        pBD->DGPSAge, pBD->DGPSReferenceStationID)) {

        pBD->changed = true;
        pBD->countGGA++;

       // Check we have sensible values
        if (!N2kIsNA(pBD->Latitude) && !N2kIsNA(pBD->Longitude)) {        
            tN2kMsg msg;
            SetN2kGNSS(msg, 1, pBD->DaysSince1970, pBD->GPSTime, pBD->Latitude, pBD->Longitude, pBD->Altitude,
            N2kGNSSt_GPS, GNSMethofNMEA0183ToN2k(pBD->GPSQualityIndicator), pBD->SatelliteCount, pBD->HDOP, 0,
            pBD->GeoidalSeparation, 1, N2kGNSSt_GPS, pBD->DGPSReferenceStationID, pBD->DGPSAge);

 
            setn2kMessage(M_GGA, msg);
        }
        else {
            n2kMessages[M_GGA].valid = false;
        }

    }
    else {
        pBD->countFail++;
        if (NMEA0183HandlersDebugStream != 0) {
            NMEA0183HandlersDebugStream->println("Failed to parse GGA");
        }
    }
}

#define PI_2 6.283185307179586476925286766559

// Track made good and speed over ground
void HandleVTG(const tNMEA0183Msg& NMEA0183Msg) {
    double MagneticCOG;

    if (pBD == 0) return;
    pBD->countVTG++;
    return;  // Disabled for now as I'm not sure this gives useful results at least in the lab when stationary

    if (NMEA0183ParseVTG_nc(NMEA0183Msg, pBD->COG, MagneticCOG, pBD->SOG)) {
        pBD->Variation = pBD->COG - MagneticCOG;  // Save variation for Magnetic heading

        pBD->changed = true;

        // Check for sensible values
        if(!N2kIsNA(pBD->COG) && !N2kIsNA(pBD->SOG)) {
            tN2kMsg msg;

            SetN2kCOGSOGRapid(msg, 1, N2khr_true, pBD->COG, pBD->SOG);
            setn2kMessage(M_VTG, msg);
        } else {
            n2kMessages[M_VTG].valid = false;
        }
    } else {
        pBD->countFail++;
        if (NMEA0183HandlersDebugStream != 0) {
            NMEA0183HandlersDebugStream->println("Failed to parse VTG");
        }
    }
}

// GPS DOP and active satellites
void HandleGSA(const tNMEA0183Msg& NMEA0183Msg) {
    if (pBD == 0) return;
    pBD->countGSA++;
}

// Helper
bool isGSVValid(struct tGSV& msg) {
    return msg.SVID > 0 && msg.Elevation > 0.0 && msg.Azimuth > 0.0 && msg.SNR > 0.0;
}

// Satellite information in view
void HandleGSV(const tNMEA0183Msg& NMEA0183Msg) {
    if (pBD == 0) return;
    int totalMsg;
    int thisMsg;
    int satCount;
    struct tGSV msg1, msg2, msg3, msg4;

    if (NMEA0183ParseGSV(NMEA0183Msg, totalMsg, thisMsg, satCount,
        msg1, msg2, msg3, msg4)) {
        if (thisMsg == 1) {
            Satellites.clear();  // Start from scratch in case they have changed since last cycle
            validGSV = false;
        }

        pBD->countGSV++;
        if (isGSVValid(msg1)) {
            Satellites[msg1.SVID] = msg1;
        }
        if (isGSVValid(msg2)) {
            Satellites[msg2.SVID] = msg2;
        }
        if (isGSVValid(msg3)) {
            Satellites[msg3.SVID] = msg3;
        }
        if (isGSVValid(msg4)) {
            Satellites[msg4.SVID] = msg4;
        }

        //      Serial.printf("GSV total %d this %d sats %d 1.SNR %f 2.SNR %f 3. SNR %f 4.SNR %f\n",
        //        totalMsg, thisMsg, satCount, msg1.SNR, msg2.SNR, msg3.SNR, msg4.SNR);
        if(totalMsg == thisMsg) {
            // Indicate a full set and can be used
            validGSV = true;
        }
    }
    else {
        pBD->countFail++;
    }
}

// Position data: position fix, time of position fix, and status
void HandleGLL(const tNMEA0183Msg& NMEA0183Msg) {
    if (pBD == 0) return;
    char buf[200];
    pBD->countGLL++;
    NMEA0183Msg.GetMessage(buf, 199);
    if (NMEA0183HandlersDebugStream != 0) {
        Serial.printf("%s\n", buf);
    }
}

// Process the saved YD messages at regular intervals.
// Make them invalid if we havn't seen them for 
// VALID_GPS_PERIOD seconds
void processYD() {
    static time_t last = 0;

    time_t nowMills = millis();

    if (nowMills > last + SEND_YD_PERIOD) {
        // check all the messages
        last = nowMills;

        for (int i = 0; i < M_MAX; i++) {
            if (n2kMessages[i].valid) {
                if (nowMills > n2kMessages[i].lastseen + VALID_GPS_PERIOD) {
                    n2kMessages[i].valid = false;
                    Serial.printf("Lost PGN %ld at %ld\n", n2kMessages[i].msg.PGN, nowMills);
                }
                else {
                    GwSendYD(n2kMessages[i].msg);
                }
            }
        }
    }
}

