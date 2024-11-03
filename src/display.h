// Display handler
#pragma once 

#include <Arduino.h>

// define the objects that can be displayed
// These are used as keys
typedef enum {
    DISPWifi,
    DISPDateTime,
    DISPPosition,
    DISPHDOP,
    DISPSats,
    DISPSpeed,
    DISPCourse,
    DISPDistance,
    DISPCompass,
    DISPEnd
} Obj;

class Txt {
    public:
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
};

void setup_display();
void display_write(Obj obj, String str);