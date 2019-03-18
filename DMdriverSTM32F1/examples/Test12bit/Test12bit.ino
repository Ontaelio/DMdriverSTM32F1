/*
 * A simple DMdriver library demo for a setup with any leds.
 * This demo is for 12bit LED drivers DM631 and DM633.
 * It assumes you have the DM256PWM option in the DMconfig.h file OFF (commented).
 * This demo just turns on individual leds with the delay defined as SPEED.
 * Useful for cheching your setup and connections.
 * Dmitry Reznikov - ontaelio<at>gmail.com - May 2016
 */

#include <DMdriverSTM32F1.h>

#define DMNUMBER 1 // number of DM LED drivers in your chain
#define SPEED 300 // the less the faster

// set up the object. Change the values according to your setup
DMdriver Test (DM631, DMNUMBER, 7);

void setup() {
Test.init();
//Test.setGlobalBrightness(100); // needed for the DM634 chips as they tend to
                                 // change their brightness on startup
}

void loop() {
for (int k = 0; k<DMNUMBER*16; k++)
{
  Test.clearAll();
  Test.setPoint(k, 4095);
  Test.sendAll();
  delay(SPEED);
}

 
}
