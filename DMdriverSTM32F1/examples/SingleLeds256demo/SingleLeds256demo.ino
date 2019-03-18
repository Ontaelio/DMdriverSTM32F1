/*
 * A simple DMdriver library demo for a setup with single color leds.
 * It assumes you have the DM256PWM option in the config file ON (uncommented).
 * This demo produces a snake with a fading tail across all present LEDs.
 * Compatible with any DM63x LED drivers, both 12- and 16-bit.
 * Dmitry Reznikov - ontaelio<at>gmail.com - May 2016
 */

#include <DMdriverSTM32F1.h>

#define TAIL 12 // the length of the 'LED tail'
#define STEP 2// speed of animation; more = faster
#define DMNUMBER 1 // number of DM LED drivers in your chain

// set up the object. Change the values according to your setup
DMdriver Test (DM631, DMNUMBER, 7);

int curpos;
int numLeds = 16*DMNUMBER;
byte stepLeds = 255/TAIL;

void setup() {
Test.init();
//Test.setGlobalBrightness(100); // needed for the DM634 chips as they tend to
                                 // change their brightness on startup
}

void loop() {
int count = 0;

Test.clearAll();
do
{
  Test.setPoint((curpos+1)%numLeds, count);
  Test.setPoint(curpos, 255);
  for (int k = 1; k<TAIL; k++)
  {
    Test.setPoint((curpos+numLeds-k)%numLeds, stepLeds*(TAIL-k)+(255-count)/TAIL);
  }
Test.sendAll();
count+=STEP;
}
while (count<256);
curpos++;
if (curpos>(DMNUMBER*16-1)) curpos = 0;
}
