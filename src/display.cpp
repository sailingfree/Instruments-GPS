// Display information on the screen

#include <TFT_eSPI.h>
#include <display.h>
#include <cyd_pins.h>


// The TFT display
TFT_eSPI tft = TFT_eSPI();

Txt text[DISPEnd];

void setup_display() {
// Start the tft display and set it to black
  tft.init();
  tft.setRotation(1); //This is the display in landscape
  
  // Clear the screen before writing to it
  tft.fillScreen(TFT_BLACK);
  
}


void display_write(Obj obj, String str) {

    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    uint16_t fontno = 4;
    uint16_t wn = tft.textWidth(str, fontno);  // New width of string to display
    uint16_t font_h = tft.fontHeight(fontno);
    uint16_t x = 5;                             // Left margin
    uint16_t y = obj * font_h;

    // If the width of the new string is less than the previous
    // clear to the end of the old    
    if(wn < text[obj].w) {
      tft.fillRect(x, y, text[obj].w, font_h, TFT_BLACK);
    }

    tft.drawString(str, x, y, fontno);

    // Save the coords and width for the next update
    text[obj].x = x;
    text[obj].y = y;
    text[obj].w = wn;
    text[obj].h = font_h;

}