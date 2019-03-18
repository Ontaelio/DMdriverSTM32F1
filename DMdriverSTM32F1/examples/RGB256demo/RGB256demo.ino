/*
 * A simple DMdriver library demo for a setup with RGB leds.
 * It assumes you have the DM256PWM option in the config file ON (uncommented).
 * This demo produces an animated rainbow on all present RGB LEDs.
 * Compatible with any DM63x LED drivers, both 12- and 16-bit.
 * Dmitry Reznikov - ontaelio<at>gmail.com - May 2016
 */

#include <DMdriverSTM32F1.h>

#define SPEED 3 // the less the faster
#define DMNUMBER 1 // number of the LED drivers

// set up the object. Change the values according to your setup
DMdriver Test (DM634, DMNUMBER, 9);

int curpos;
int numLeds = (16*DMNUMBER)/3;
byte red, green, blue;

void setup() {
Test.init();
Test.setGlobalBrightness(100);
}

void loop() {
int count = 0;
Test.clearAll();
do
{
  for (int k = 0; k<numLeds; k++)
  {
    int tcount = (count + k * (765/numLeds))%765;
    if (tcount < 256) {red = 255 - tcount; green = tcount; blue = 0;}
    else if (tcount < 512) {red = 0; green = 511 - tcount; blue = tcount-256;}
    else {red = tcount - 512; green = 0; blue = 765 - tcount;}
    Test.setRGBpoint(k, red, green, blue);
  }
Test.sendAll();
count++;
delay(SPEED);
}
while (count<765);
}



