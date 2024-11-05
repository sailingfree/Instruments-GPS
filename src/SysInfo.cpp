// System and net info
/*
Copyright (c) 2022 Peter Martin www.naiadhome.com

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to use,
copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <Arduino.h>
#include <GwPrefs.h>
#include <NMEA0183Messages.h>
#include <SysInfo.h>
#include <esp_wifi.h>
#include <BoatData.h>

extern tBoatData BoatData;

#include "uptime_formatter.h"

void getNetInfo(Stream& s) {
    wifi_sta_list_t wifi_sta_list;
    tcpip_adapter_sta_list_t adapter_sta_list;

    s.println("=========== NETWORK ==========");
    s.printf("HOST NAME: %s\n", hostName.c_str());
    s.printf("WifiMode %s\n", WifiMode.c_str());
    s.printf("WifiIP %s\n", WifiIP.c_str());
    s.printf("WifiSSID %s\n", WifiSSID.c_str());

    // Get the wifi station list and list the connected device details
    memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
    memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);

    if (adapter_sta_list.num) {
        s.println("----- DHCP clients ---------");
        for (int i = 0; i < adapter_sta_list.num; i++) {
            tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];

            s.printf("Station number %d \n", i);
            s.printf("MAC: ");
            for (int j = 0; j < 6; j++) {
                s.printf("%02X", station.mac[j]);
                if (i < 5) s.print(":");
            }
            s.printf("\nIP: ");
            s.println(ip4addr_ntoa((const ip4_addr_t*)&(station.ip)));
            s.println("");
        }
    }

    s.println("=========== END ==========");
}

void getSysInfo(Stream& s) {
    EspClass esp;

    uint32_t heap = esp.getHeapSize();      // total heap size
    uint32_t freeheap = esp.getFreeHeap();  // available heap
    uint32_t heapUsedPc = (heap - freeheap) * 100 / heap;

    uint8_t chiprev = esp.getChipRevision();
    const char* model = esp.getChipModel();
    uint32_t sketchSize = esp.getSketchSize();
    uint32_t freeSketch = esp.getFreeSketchSpace();
    uint32_t flashsize = esp.getFlashChipSize();
    uint32_t flashUsedPc = (flashsize - freeSketch) * 100 / flashsize;
    uint64_t efuse = esp.getEfuseMac();
    String uptime = uptime_formatter::getUptime();

    s.println("=========== SYSTEM ==========");
    s.printf("Model %s\n", Model.c_str());
    s.printf("Uptime: %s", uptime.c_str());
    s.printf("Heap \t%d\n", heap);
    s.printf("Heap Free\t%d\n", freeheap);
    s.printf("Heap used %d%%\n", heapUsedPc);
    s.printf("ChipRev \t%d\n", chiprev);
    s.printf("Model \t%s\n", model);
    s.printf("Sketch \t%d\n", sketchSize);
    s.printf("Sketch Free \t%d\n", freeSketch);
    s.printf("Flash used %d%%\n", flashUsedPc);
    s.printf("Efuse \t0x%llx\n", efuse);

    s.println("=========== SETTINGS ==========");
    GwPrint(s);
    s.println("=========== END ==========");
}

void getGps(Stream& s) {
    /*
    std::map<String, String>::iterator it = Gps.begin();
    s.println("=========== GPS ==========");

    while (it != Gps.end()) {
        s.printf("%s %s\n", it->first.c_str(), it->second.c_str());
        it++;
    }
    */
    s.println("=========== END ==========");
}

void getSatellites(Stream& s) {
    time_t now = time(NULL);
    /*
        std::map<int, tGSV>::iterator it = Satellites.begin();
        // the map may have changed so go through it again
        it = Satellites.begin();
        s.println("=========== GPS Satellites==========");
        s.printf("Satellites %s\n", Gps["GSV sats"].c_str());
        s.printf("SVID\tAZ\tELEV\tSNR\n");
        while (it != Satellites.end()) {
            tGSV sat = it->second;
            if (sat.Azimuth != NMEA0183DoubleNA && sat.Elevation != NMEA0183DoubleNA && sat.SNR != NMEA0183DoubleNA) {
                s.printf("%d\t%g\t%g\t%g\n", sat.SVID, sat.Azimuth, sat.Elevation, sat.SNR);
            }
            it++;
        }
    */
    s.println("================ END ===============");
}

void getSensors(Stream& s) {
    std::map<String, String>::iterator it = Sensors.begin();

    s.println("=========== SENSORS ==========");

//    while (it != Sensors.end()) {
//        s.printf("%s %s\n", it->first.c_str(), it->second.c_str());
//        it++;
//    }
    s.println("=========== END ==========");
}

//extern std::map<int, int> N2kMsgMap;
void getN2kMsgs(Stream& s) {
    //   std::map<int, int>::iterator it = N2kMsgMap.begin();

    s.println("======== N2K Messages ====");

    //    while (it != N2kMsgMap.end()) {
    //        s.printf("%d %d\n", it->first, it->second);
     //       it++;
     //   }
    s.println("=========== END ==========");
}

void displayBoat(Stream& s) {
    s.printf("============ BOAT ===========\n");
    s.printf("TrueHeading %f\n", BoatData.TrueHeading);
    s.printf("SOG %f\n", BoatData.SOG);
    s.printf("COG %f\n", BoatData.COG);
    s.printf("GPSTime %f\n", BoatData.GPSTime);
    s.printf("HDOP %f\n", BoatData.HDOP);
    s.printf("DaysSince1970 %lu\n", BoatData.DaysSince1970);
    s.printf("Sats %d\n", BoatData.SatelliteCount);
    s.printf("LAT %f\n", BoatData.Latitude);
    s.printf("LON %f\n", BoatData.Longitude);
    s.printf("RMC %d\n", BoatData.countRMC);
    s.printf("GGA %d\n", BoatData.countGGA);
    s.printf("VTG %d\n", BoatData.countVTG);
    s.printf("GLL %d\n", BoatData.countGLL);
    s.printf("GSA %d\n", BoatData.countGSA);
    s.printf("GSV %d\n", BoatData.countGSV);

}