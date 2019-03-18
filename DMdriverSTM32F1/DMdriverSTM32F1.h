/*
 * This is the header file of the Arduino library for DM63x LED driver chips
 * (c) 2016 Dmitry Reznkov, dmitry@ultiblink.com
 * The library is free software under GNU General Public License.
 * You can use it any way you wish, but NO WARRANTY is provided.
 *
 * Special thanks: Alex Leone and his excellent Arduino TLC5940 library
 * that served as an inspiration for this one.
*/
#ifndef DM_library
#define DM_library

#include "Arduino.h"
#include "DMconfig.h"


#define DM631 24
#define DM632 32
#define DM633 24
#define DM634 32

#define __PORTA 0x40010800
#define __PORTB 0x40010C00
#define __PORTC 0x40011000
#define __PORTD 0x40011400
#define __PORTE 0x40011800
#define __PORTF 0x40011C00
#define __PORTG 0x40012000

#define __SPI1 0x40013000
#define __SPI2 0x40003800
#define __SPI3 0x40003C00

class DMdriver
{
 public:
   DMdriver(uint8_t Driver, uint8_t Number, uint32_t LatchPort = __PORTB, uint8_t LatchPin = 0, uint32_t SPI_addr = __SPI1);
   DMLEDTABLE* ledTable; // led lookup table, type defined by DMLEDTABLE
   void init(DMLEDTABLE *table = NULL); // initialise the object and SPI interface  
   void setPoint(uint16_t pixNum, uint16_t pixVal); // sets single LED
   uint16_t getPoint(uint16_t pixNum); // returns the value of a single LED
   void setRGBpoint(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue); // sets an RGB LED on an RGB-only board
   void setRGBmax(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue, uint16_t max = 0); //sets and RGB LED correcting the overall 
																								//brightness by max value
   void setRGBled(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue); //sets an RGB LED using the pin number of R as an address
   void sendAll(); // send data to the chip
   void clearAll(); // clear all pixels
   void setGlobalBrightness(uint8_t bri); // set overall brightness, DM633 and DM634 only
   void setGBCbyDriver(uint8_t *bri); // set individual Global Brightness Correction for each DM chip
									  // *bri points to an array of bytes with GB info for each DM chip
   void turnOff(); // destructor; frees the dynamic memory; use init to start again
   void deallocLedTable(); // free the lookup table memory if not needed
   volatile uint8_t* pixel;  //actual bytes sent to the driver
   
 private: 
   void dm_shift(uint8_t value);
   uint16_t getChainValue (uint16_t pixNum);
   //uint8_t LATbit, LATport;
   uint16_t pinNum, byteNum;
   //volatile uint8_t *LATreg, *LATout;
   uint8_t DMtype,  DMnum,  LATpin, SCKpin;
   uint32_t SPIreg, LATport, LATmode0, LATmode1, LATcnf0, SCKport, SCKmode, SCK_BSRR, SCK_BRR;
};

#endif