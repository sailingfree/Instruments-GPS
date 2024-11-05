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

struct tNMEA0183Handler {
  const char *Code;
  void (*Handler)(const tNMEA0183Msg &NMEA0183Msg); 
};

// Predefinition for functions to make it possible for constant definition for NMEA0183Handlers
void HandleRMC(const tNMEA0183Msg &NMEA0183Msg);
void HandleGGA(const tNMEA0183Msg &NMEA0183Msg);
void HandleHDT(const tNMEA0183Msg &NMEA0183Msg);
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg);
void HandleGLL(const tNMEA0183Msg &NMEA0183Msg);
void HandleGSA(const tNMEA0183Msg &NMEA0183Msg);
void HandleGSV(const tNMEA0183Msg &NMEA0183Msg);

// Internal variables
//tNMEA2000 *pNMEA2000=0;
tBoatData *pBD=0;
Stream* NMEA0183HandlersDebugStream=&Serial;

tNMEA0183Handler NMEA0183Handlers[]={
  {"GGA",&HandleGGA},
  {"HDT",&HandleHDT},
  {"VTG",&HandleVTG},
  {"RMC",&HandleRMC},
  {"GSA",&HandleGSA},
  {"GSV",&HandleGSV},
  {"GLL",&HandleGLL},
  {0,0}
};

void InitNMEA0183Handlers(tBoatData *_BoatData) {
  pBD=_BoatData;
}

void DebugNMEA0183Handlers(Stream* _stream) {
  NMEA0183HandlersDebugStream=_stream;
}

tN2kGNSSmethod GNSMethofNMEA0183ToN2k(int Method) {
  switch (Method) {
    case 0: return N2kGNSSm_noGNSS;
    case 1: return N2kGNSSm_GNSSfix;
    case 2: return N2kGNSSm_DGNSS;
    default: return N2kGNSSm_noGNSS;  
  }
}

void HandleNMEA0183Msg(const tNMEA0183Msg &NMEA0183Msg) {
  int iHandler;
  // Find handler
  for (iHandler=0; NMEA0183Handlers[iHandler].Code!=0 && !NMEA0183Msg.IsMessageCode(NMEA0183Handlers[iHandler].Code); iHandler++);
  if (NMEA0183Handlers[iHandler].Code!=0) {
    NMEA0183Handlers[iHandler].Handler(NMEA0183Msg); 
  } else {
    Serial.printf("No handler for message %s\n", NMEA0183Msg.MessageCode());
  }
}

// NMEA0183 message Handler functions

// Position, velocity, and time
void HandleRMC(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseRMC_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->COG,pBD->SOG,pBD->DaysSince1970,pBD->Variation)) {
    pBD->changed = true;
    pBD->countRMC++;
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse RMC"); }
}

// Time, position, and fix related data
void HandleGGA(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
        char buf[200];
      pBD->countGLL++;
      NMEA0183Msg.GetMessage(buf, 199);
      Serial.printf("%s\n", buf);
  int fc = NMEA0183Msg.FieldCount();

  if (NMEA0183ParseGGA_nc(NMEA0183Msg,pBD->GPSTime,pBD->Latitude,pBD->Longitude,
                   pBD->GPSQualityIndicator,pBD->SatelliteCount,pBD->HDOP,pBD->Altitude,pBD->GeoidalSeparation,
                   pBD->DGPSAge,pBD->DGPSReferenceStationID)) {

      tN2kMsg N2kMsg;
      pBD->changed = true;
      pBD->countGGA++;
      SetN2kGNSS(N2kMsg,1,pBD->DaysSince1970,pBD->GPSTime,pBD->Latitude,pBD->Longitude,pBD->Altitude,
                N2kGNSSt_GPS,GNSMethofNMEA0183ToN2k(pBD->GPSQualityIndicator),pBD->SatelliteCount,pBD->HDOP,0,
                pBD->GeoidalSeparation,1,N2kGNSSt_GPS,pBD->DGPSReferenceStationID,pBD->DGPSAge
                );
      GwSendYD(N2kMsg); 


    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("Time="); NMEA0183HandlersDebugStream->println(pBD->GPSTime);
      NMEA0183HandlersDebugStream->print("Latitude="); NMEA0183HandlersDebugStream->println(pBD->Latitude,5);
      NMEA0183HandlersDebugStream->print("Longitude="); NMEA0183HandlersDebugStream->println(pBD->Longitude,5);
      NMEA0183HandlersDebugStream->print("Altitude="); NMEA0183HandlersDebugStream->println(pBD->Altitude,1);
      NMEA0183HandlersDebugStream->print("GPSQualityIndicator="); NMEA0183HandlersDebugStream->println(pBD->GPSQualityIndicator);
      NMEA0183HandlersDebugStream->print("SatelliteCount="); NMEA0183HandlersDebugStream->println(pBD->SatelliteCount);
      NMEA0183HandlersDebugStream->print("HDOP="); NMEA0183HandlersDebugStream->println(pBD->HDOP);
//      NMEA0183HandlersDebugStream->print("GeoidalSeparation="); NMEA0183HandlersDebugStream->println(pBD->GeoidalSeparation);
//      NMEA0183HandlersDebugStream->print("DGPSAge="); NMEA0183HandlersDebugStream->println(pBD->DGPSAge);
//      NMEA0183HandlersDebugStream->print("DGPSReferenceStationID="); NMEA0183HandlersDebugStream->println(pBD->DGPSReferenceStationID);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse GGA"); }
}

#define PI_2 6.283185307179586476925286766559

// Heading from True North
void HandleHDT(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  
  if (NMEA0183ParseHDT_nc(NMEA0183Msg,pBD->TrueHeading)) {

      tN2kMsg N2kMsg;
      pBD->changed = true;
      pBD->countHDT++;
      double MHeading=pBD->TrueHeading-pBD->Variation;
      while (MHeading<0) MHeading+=PI_2;
      while (MHeading>=PI_2) MHeading-=PI_2;
      // Stupid Raymarine can not use true heading
      SetN2kMagneticHeading(N2kMsg,1,MHeading,0,pBD->Variation);
//      SetN2kPGNTrueHeading(N2kMsg,1,pBD->TrueHeading);
      GwSendYD(N2kMsg);

    if (NMEA0183HandlersDebugStream!=0) {
      NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse HDT"); }
}

// Track made good and speed over ground
void HandleVTG(const tNMEA0183Msg &NMEA0183Msg) {
     return;  // Disabled for now as I'm not sure this gives useful results at least in the lab when stationary
 double MagneticCOG;

  if (pBD==0) return;
  
  if (NMEA0183ParseVTG_nc(NMEA0183Msg,pBD->COG,MagneticCOG,pBD->SOG)) {
      pBD->Variation=pBD->COG-MagneticCOG; // Save variation for Magnetic heading

      tN2kMsg N2kMsg;
      pBD->changed = true;
      pBD->countVTG++;
      SetN2kCOGSOGRapid(N2kMsg,1,N2khr_true,pBD->COG,pBD->SOG);
      GwSendYD(N2kMsg);

//    if (NMEA0183HandlersDebugStream!=0) {
//      NMEA0183HandlersDebugStream->print("True heading="); NMEA0183HandlersDebugStream->println(pBD->TrueHeading);
//    }
  } else if (NMEA0183HandlersDebugStream!=0) { NMEA0183HandlersDebugStream->println("Failed to parse VTG"); }
}

// GPS DOP and active satellites
void HandleGSA(const tNMEA0183Msg &NMEA0183Msg) {
  if (pBD==0) return;
  pBD->countGSA++;
}

// Satellite information in view
void HandleGSV(const tNMEA0183Msg &NMEA0183Msg) {
      if (pBD==0) return;
      int totalMsg;
      int thisMsg;
      int satCount;
      struct tGSV msg1, msg2, msg3, msg4;

      if(NMEA0183ParseGSV(NMEA0183Msg, totalMsg, thisMsg, satCount,
        msg1, msg2, msg3, msg4)) {

        pBD->countGSV++;
//        Serial.printf("GSV total %d this %d sats %d 1.SNR %f 2.SNR %f 3. SNR %f 4.SNR %f\n",
//          totalMsg, thisMsg, satCount, msg1.SNR, msg2.SNR, msg3.SNR, msg4.SNR);
        }
}

// Position data: position fix, time of position fix, and status
void HandleGLL(const tNMEA0183Msg &NMEA0183Msg) {
      if (pBD==0) return;
      char buf[200];
      pBD->countGLL++;
      NMEA0183Msg.GetMessage(buf, 199);
//      Serial.printf("%s\n", buf);
}