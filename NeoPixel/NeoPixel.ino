// NeoPixel test sketch
// UBC: the UBC Submarine Design Team

#include <Adafruit_NeoPixel.h>

#define JEWEL_PIN 7
#define NEOPIXEL_COUNT 7

Adafruit_NeoPixel jewel(NEOPIXEL_COUNT, JEWEL_PIN, NEO_GRB + NEO_KHZ800);
int i=0,j=0;
uint32_t settings[3]={jewel.Color(153,51,0),jewel.Color(0,0,102),jewel.Color(128,0,128)};
uint32_t red;

void setup() {
  jewel.begin();
  //jewel.fill(jewel.Color(255, 0, 255));
  jewel.setPixelColor(0,jewel.Color(0,154,50));
  jewel.setBrightness(20);
  jewel.show();
  red=jewel.Color(255,0,0);
  
}

void loop() {
  for (j=0;j<3;j++){
    for (i=1;i<NEOPIXEL_COUNT;i++){
      jewel.setPixelColor(i, red);
      jewel.show();
      delay(500);
    }
    for (i=1;i<NEOPIXEL_COUNT;i++){
      jewel.setPixelColor(i, 0);
      jewel.show();
      delay(500);
    }
    jewel.setPixelColor(0,settings[j]);
  }
  
}
