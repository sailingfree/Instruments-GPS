#ifndef _BoatData_H_
#define _BoatData_H_

struct tBoatData {
    bool changed;                   // Changed since last time we read the data
    unsigned long DaysSince1970;    // Days since 1970-01-01

    double TrueHeading,
        SOG,
        COG,
        Variation,
        GPSTime,// Secs since midnight,
        Latitude,
        Longitude,
        Altitude,
        HDOP,
        GeoidalSeparation,
        DGPSAge,
        pressure,
        temperature;
    int GPSQualityIndicator,
        SatelliteCount,
        DGPSReferenceStationID;
    uint32_t countRMC,
        countGGA,
        countVTG,
        countGLL,
        countGSA,
        countGSV;

    bool MOBActivated;

public:
    tBoatData() {
        changed = false;
        TrueHeading = 0;
        SOG = 0;
        COG = 0;
        Variation = 7.0;
        GPSTime = 0;
        Altitude = 0;
        HDOP = 100000;
        DGPSAge = 100000;
        DaysSince1970 = 0;
        MOBActivated = false;
        SatelliteCount = 0;
        DGPSReferenceStationID = 0;
        countRMC =
            countGGA =
            countVTG =
            countGLL =
            countGSA =
            countGSV = 0;
    };
};

#endif // _BoatData_H_