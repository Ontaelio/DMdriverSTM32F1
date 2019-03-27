/*
 * This is the main file of the STM32F103x library for DM63x LED driver chips
 * (c) 2016 Dmitry Reznkov, dmitry@ultiblink.com
 * The library is free software under GNU General Public License.
 * You can use it any way you wish, but NO WARRANTY is provided.
 *
 * Special thanks: Alex Leone and his excellent Arduino TLC5940 library
 * that served as an inspiration for this one.
*/
//#include "Arduino.h"
#include "stdint.h"
//#include "string.h"
#include "DMdriverSTM32F1.h"

// A bunch of addresses to avoid using HALs and stuff
#define _RCC_(mem_offset) (*(volatile uint32_t *)(0x40021000 + (mem_offset)))

#define _CR 0x00
#define _CFGR 0x04
#define _CIR 0x08
#define _APB2RSTR 0x0C
#define _APB1RSTR 0x10
#define _AHBENR 0x14
#define _APB2ENR 0x18
#define _APB1ENR 0x1C
#define _BDCR 0x20
#define _CSR 0x24

#define IOPAEN 0x0004
#define IOPBEN 0x0008
#define IOPCEN 0x0010
#define IOPDEN 0x0020
#define IOPEEN 0x0040
#define IOPFEN 0x0080
#define IOPGEN 0x0100
#define AFIOEN 0x0001
#define SPI1EN 0x1000
#define SPI2EN 0x4000
#define SPI3EN 0x8000

#define _PORTA_(mem_offset) (*(volatile uint32_t *)(0x40010800 + (mem_offset)))
#define _PORTB_(mem_offset) (*(volatile uint32_t *)(0x40010C00 + (mem_offset)))
#define _PORTC_(mem_offset) (*(volatile uint32_t *)(0x40011000 + (mem_offset)))

#define _LATPORT_(mem_offset) (*(volatile uint32_t *)(LATport + (mem_offset)))


#define _BRR  0x14
#define _BSRR 0x10
#define _CRL  0x00
#define _CRH  0x04
#define _IDR  0x08
#define _ODR  0x0C

#define _SPI1_(mem_offset) (*(volatile uint32_t *)(0x40013000 + (mem_offset)))

#define _SPIREG_(mem_offset) (*(volatile uint32_t *)(SPIreg + (mem_offset)))

#define _SCKPORT_(mem_offset) (*(volatile uint32_t *)(mem_offset))


#define _SPI_CR1 0x00
#define _SPI_CR2 0x04
#define _SPI_SR  0x08
#define _SPI_DR  0x0C

#define CPHA        0x0001
#define CPOL        0x0002
#define MSTR        0x0004
#define BR_0        0x0008
#define BR_1        0x0010
#define BR_2        0x0020
#define SPE         0x0040
#define LSB_FIRST   0x0080
#define SSI         0x0100
#define SSM         0x0200
#define RX_ONLY     0x0400
#define DFF         0x0800
#define CRC_NEXT    0x1000
#define CRC_EN      0x2000
#define BIDIOE      0x4000
#define BIDIMODE    0x8000

#define BSY         0x0080

#define TXE         0x0002

#define CNF0_0 0x00000004
#define CNF0_1 0x00000008
#define CNF1_0 0x00000040
#define CNF1_1 0x00000080
#define CNF2_0 0x00000400
#define CNF2_1 0x00000800
#define CNF3_0 0x00004000
#define CNF3_1 0x00008000
#define CNF4_0 0x00040000
#define CNF4_1 0x00080000
#define CNF5_0 0x00400000
#define CNF5_1 0x00800000
#define CNF6_0 0x04000000
#define CNF6_1 0x08000000
#define CNF7_0 0x40000000
#define CNF7_1 0x80000000
#define CNF8_0 0x00000004
#define CNF8_1 0x00000008
#define CNF9_0 0x00000040
#define CNF9_1 0x00000080
#define CNF10_0 0x00000400
#define CNF10_1 0x00000800
#define CNF11_0 0x00004000
#define CNF11_1 0x00008000
#define CNF12_0 0x00040000
#define CNF12_1 0x00080000
#define CNF13_0 0x00400000
#define CNF13_1 0x00800000
#define CNF14_0 0x04000000
#define CNF14_1 0x08000000
#define CNF15_0 0x40000000
#define CNF15_1 0x80000000

#define MODE0_0 0x00000001
#define MODE0_1 0x00000002
#define MODE1_0 0x00000010
#define MODE1_1 0x00000020
#define MODE2_0 0x00000100
#define MODE2_1 0x00000200
#define MODE3_0 0x00001000
#define MODE3_1 0x00002000
#define MODE4_0 0x00010000
#define MODE4_1 0x00020000
#define MODE5_0 0x00100000
#define MODE5_1 0x00200000
#define MODE6_0 0x01000000
#define MODE6_1 0x02000000
#define MODE7_0 0x10000000
#define MODE7_1 0x20000000
#define MODE8_0 0x00000001
#define MODE8_1 0x00000002
#define MODE9_0 0x00000010
#define MODE9_1 0x00000020
#define MODE10_0 0x00000100
#define MODE10_1 0x00000200
#define MODE11_0 0x00001000
#define MODE11_1 0x00002000
#define MODE12_0 0x00010000
#define MODE12_1 0x00020000
#define MODE13_0 0x00100000
#define MODE13_1 0x00200000
#define MODE14_0 0x01000000
#define MODE14_1 0x02000000
#define MODE15_0 0x10000000
#define MODE15_1 0x20000000

/*** Get LAT low (just in case) */
#define LAT_low() _LATPORT_(_BRR) = (1<<LATpin) 

/*** LAT pulse - high, then low */
#define LAT_pulse() _LATPORT_(_BSRR) = (1<<LATpin); _LATPORT_(_BRR) = (1<<LATpin)

/*** Get LAT high */
#define LAT_high() _LATPORT_(_BSRR) = (1<<LATpin);

DMdriver::DMdriver(uint8_t Driver, uint8_t Number, uint32_t LatchPort, uint8_t LatchPin, uint32_t SPI_addr)
{
	DMnum = Number; // number of DM chips in chain
	DMtype = Driver; // 16 or 12 bits (32 or 24 bytes per chip)
	LATpin = LatchPin; // latch pin
	LATport = LatchPort; // latch port
	SPIreg = SPI_addr; // SPI selection, SPI register address
	LATmode0 = 1<<((LATpin%8)*4); 
	LATmode1 = 1<<((LATpin%8)*4 + 1);
	LATcnf0 = 1<<((LATpin%8)*4 + 2);
	
	if (SPIreg == __SPI1){
		SCKpin = 5;
		SCKport = __PORTA + _CRL;
		SCKmode = CNF5_1;  
		SCK_BRR = __PORTA + _BRR;
		SCK_BSRR = __PORTA + _BSRR;
	}
	
	if (SPIreg == __SPI2){
		SCKpin = 13;
		SCKport = __PORTB + _CRH;
		SCKmode = CNF13_1;  
		SCK_BRR = __PORTB + _BRR;
		SCK_BSRR = __PORTB + _BSRR;
	}
	
	if (SPIreg == __SPI3){
		SCKpin = 3;
		SCKport = __PORTB + _CRL;
		SCKmode = CNF3_1;  
		SCK_BRR = __PORTB + _BRR;
		SCK_BSRR = __PORTB + _BSRR;
	}
	
	
	pinNum = DMnum * 16; // number of outputs (single color LEDs)
#ifdef DM256PWM
	byteNum = pinNum; // if 8bit resolution was defined, 16 bytes per chip used
#else
	byteNum = DMnum * DMtype;
#endif
	
}

void DMdriver::dm_shift(uint8_t value)
{
    _SPIREG_(_SPI_DR) = value; //send a byte
    while (!(_SPIREG_(_SPI_SR) & TXE)); //wait until it's sent
}

void DMdriver::init(DMLEDTABLE *table)
{
	pixel = new uint8_t[byteNum];  //init the table of actual pixels
	ledTable = table;
	
	// turn on clocks on ports A and B, as well as alternate functions
	_RCC_(_APB2ENR) |= IOPBEN | IOPAEN | AFIOEN;
	
	// in case LAT pin is on other ports
	if (LATport == __PORTC) _RCC_(_APB2ENR) |= IOPCEN;
	if (LATport == __PORTD) _RCC_(_APB2ENR) |= IOPDEN;
	if (LATport == __PORTE) _RCC_(_APB2ENR) |= IOPEEN;
	if (LATport == __PORTF) _RCC_(_APB2ENR) |= IOPFEN;
	if (LATport == __PORTG) _RCC_(_APB2ENR) |= IOPGEN;
    
	// SPI setup
	if (SPIreg == __SPI1){
	// Default SPI1: A7(MOSI) and A5(SCK) alt push pull; A4(CS) push pull output
	_PORTA_ (_CRL) &= ~(CNF7_0 | CNF5_0 | CNF4_0); //clear the bit set by default
	_PORTA_ (_CRL) |= CNF7_1 | CNF5_1; //alt functions
	_PORTA_ (_CRL) |= MODE7_1 | MODE7_0 | MODE5_1 | MODE5_0 | MODE4_1 | MODE4_0; //output 50 MHz
	
	//turn on SPI1 clock
	_RCC_(_APB2ENR) |= SPI1EN;
	}
	
	if (SPIreg == __SPI2){
	// SPI2: B15(MOSI) and B13(SCK) alt push pull; B12(CS) push pull output
	_PORTB_ (_CRH) &= ~(CNF15_0 | CNF13_0 | CNF12_0); //clear the bit set by default
	_PORTB_ (_CRH) |= CNF15_1 | CNF13_1; //alt functions
	_PORTB_ (_CRH) |= MODE15_1 | MODE15_0 | MODE13_1 | MODE13_0 | MODE12_1 | MODE12_0; //output 50 MHz
	
	//turn on SPI2 clock
	_RCC_(_APB1ENR) |= SPI2EN;
	}
	
	if (SPIreg == __SPI3){
	// SPI3: B5(MOSI) and B3(SCK) alt push pull; A15(CS) push pull output
	_PORTB_ (_CRL) &= ~(CNF5_0 | CNF3_0); //clear the bit set by default
	_PORTB_ (_CRL) |= CNF5_1 | CNF3_1; //alt functions
	_PORTB_ (_CRL) |= MODE5_1 | MODE3_0 | MODE3_1 | MODE3_0; //output 50 MHz
	_PORTA_ (_CRH) &= ~(CNF15_0);
	_PORTA_ (_CRH) |= MODE15_0 | MODE15_1;
	
	//turn on SPI1 clock
	_RCC_(_APB1ENR) |= SPI3EN;
	}
	

	// LAT pin: normal push-pull output
	if (LATpin < 8){
	_LATPORT_ (_CRL) &= ~LATcnf0; //clear default bit
	_LATPORT_ (_CRL) |= LATmode0 | LATmode1; //output 50MHz
    } else {
	_LATPORT_ (_CRH) &= ~LATcnf0; //clear default bit
	_LATPORT_ (_CRH) |= LATmode0 | LATmode1; //output 50MHz
    }	
	
	//setup SPI
	_SPIREG_ (_SPI_CR1) |= BR_0;// pclk/4 works ok. 3 bits (BR_1 and BR_2)
	_SPIREG_ (_SPI_CR1) &= ~CPOL; //low SCK when idle if 0
	_SPIREG_ (_SPI_CR1) &= ~CPHA; //rising edge if 0
	_SPIREG_ (_SPI_CR1) &= ~DFF; // 8-bit mode
	_SPIREG_ (_SPI_CR1) &= ~LSB_FIRST; //MSB first
	_SPIREG_ (_SPI_CR1) |= SSM | SSI; //enable software control of SS, SS high
	_SPIREG_ (_SPI_CR1) |= MSTR; //SPI master
	_SPIREG_ (_SPI_CR1) |= SPE; //enable SPI
	
	clearAll();
	sendAll(); // found that this is useful to have a clear start
	
}

#ifdef DM256PWM
void DMdriver::setPoint(uint16_t pixNum, uint16_t pixVal)
{
	#ifdef DMCHAIN
	pixNum = getChainValue (pixNum);
	#endif
	
	if (ledTable) pixNum = ledTable[pixNum];

	pixel[pixNum] = pixVal;
}

uint16_t DMdriver::getPoint(uint16_t pixNum)
{
	#ifdef DMCHAIN
	pixNum = getChainValue (pixNum);
	#endif
	
	if (ledTable) pixNum = ledTable[pixNum];

	return pixel[pixNum];
}

void DMdriver::sendAll()
{
	LAT_low();
	//cli();
	uint16_t k = byteNum;
	if (DMtype==32) do
	{   k--;
		uint16_t pixVal = pixel[k] DM256PWM; // either square or <<8
		dm_shift(pixVal>>8);
		dm_shift(pixVal & 0xFF);
	} while (k);
	
	if (DMtype==24) do
	{   k--;
		uint16_t pixVal1 = (pixel[k] DM256PWM)>>4; 
		k--;
		uint16_t pixVal2 = (pixel[k] DM256PWM)>>4; 
		// shift MSB 8 bits, first value
		dm_shift(pixVal1>>4);
		// make second byte from LSB 4 bits of the first and MSB 4 bits of the second
		pixVal1 = (uint8_t)(pixVal1<<4) | (pixVal2 >> 8);
		dm_shift(pixVal1 & 0xFF);
		// shift LSB 8 bits of the second value
		dm_shift(pixVal2 & 0xFF);
	} while (k); 
	
	while (_SPIREG_(_SPI_SR) & BSY); // finish transmission
	
	LAT_pulse();
}

#else  // end of DM256 stuff

void DMdriver::setPoint(uint16_t pixNum, uint16_t pixVal)
{
   #ifdef DMCHAIN
	pixNum = getChainValue (pixNum);
   #endif
   
   if (ledTable) pixNum = ledTable[pixNum];

  if (DMtype==24)  // 12-bit data shift
{
	
	uint16_t place = ((pixNum * 3) >> 1);
	if (pixNum & 1) 
	{ // starts clean
                      // 8 upper bits 
        pixel[place+1] = pixVal >> 4;
                               // 4 lower bits  | keep last 4 bits intact
        pixel[place] = ((uint8_t)(pixVal << 4)) | (pixel[place] & 0x0F);
   }
   else
   { // starts in the middle
                     // keep first 4 bits intact | 4 top bits 
        pixel[place+1] = (pixel[place+1] & 0xF0) | (pixVal >> 8);
		             // 8 lower bits of value
        pixel[place] = pixVal & 0xFF;
    } 
}
   else             // 16-bit data shift
{
	//uint16_t index = (pinNum-1) - pixNum;
	uint16_t place = (pixNum << 1);
	pixel[place+1] = pixVal >> 8;  // 8 top bits
	pixel[place] = pixVal & 0xFF; // 8 lower bits
}
}

uint16_t DMdriver::getPoint(uint16_t pixNum)
{
   #ifdef DMCHAIN
	pixNum = getChainValue (pixNum);
   #endif
   
   if (ledTable) pixNum = ledTable[pixNum];
	
  if (DMtype==24)  // 12-bit data 
{ 
	uint16_t place = ((pixNum * 3) >> 1);
	if (pixNum & 1) 
    { // starts clean
                 // 8 upper bits  | 4 lower bits
        return ((pixel[place+1]<<4) |  ((pixel[place] & 0xF0)>>4));
   
    }
    else
	{ // starts in the middle
                      // top 4 bits intact | lower 8 bits 
        return (((pixel[place+1] & 0xF)<<8)  | (pixel[place]));
        
    }
}
   else             // 16-bit data 
{
	uint16_t place = (pixNum << 1);
	return (pixel[place+1]<<8 | pixel[place]);
}
}

void DMdriver::sendAll()
{
	LAT_low();
	
	uint16_t k = byteNum;
	do
	{   k--;
	    dm_shift(pixel[k]);
	} while (k);
	
	while (_SPI1_(_SPI_SR) & BSY); // finish transmission
	
	LAT_pulse();
}
#endif

void DMdriver::setRGBpoint(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue)
{
	LED *= 3;
		
	setPoint(LED, red);
	setPoint(LED+1, green);
	setPoint(LED+2, blue);
	
}

void DMdriver::setRGBmax(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue, uint16_t max)
{
	if (max)
	{
		uint32_t valsum = (red + green + blue);
		if (valsum > max)
			{
				red = ((uint32_t)red * max) / valsum;
				green = ((uint32_t)green * max) / valsum;
				blue = ((uint32_t)blue * max) / valsum;
			}
	}
	
	LED *= 3;
	
	setPoint(LED, red);
	setPoint(LED+1, green);
	setPoint(LED+2, blue);
	
}

void DMdriver::setRGBled(uint16_t LED, uint16_t red, uint16_t green, uint16_t blue)
{
	setPoint(LED, red);
	setPoint(LED+1, green);
	setPoint(LED+2, blue);
}


void DMdriver::clearAll()
{
	for (uint16_t k=0; k<byteNum; k++)
	{pixel[k]=0;}
}

void DMdriver::setGlobalBrightness(uint8_t bri)
{

//clear GS data to avoid flickering
for (uint8_t k=0; k<pinNum*2; k++) dm_shift(0);

_SCKPORT_ (SCKport) &= ~SCKmode; //switch SCK to normal outout
_SCKPORT_ (SCK_BSRR) = (1<<SCKpin); //SCK high

/*** 4 LAT pulses to turn on GBC input mode */
LAT_pulse(); LAT_pulse(); LAT_pulse(); LAT_pulse();

_SCKPORT_ (SCK_BRR) = (1<<SCKpin);  //SCK low
_SCKPORT_ (SCKport) |= SCKmode; // SCK back to SPI mode

/*** shift GBC data to the drivers */
/*** (each DM gets one byte; 7 upper bits control GB data, LSB should be 0 (o/s flag) */
for (uint8_t count=0; count<DMnum; count++) {dm_shift(bri<<1);}

while (_SPIREG_(_SPI_SR) & BSY); //finish the transmission
LAT_pulse();

sendAll(); //restore GS data

}

void DMdriver::setGBCbyDriver(uint8_t *bri)
{

//clear GS data to avoid flickering
for (uint8_t k=0; k<pinNum*2; k++) dm_shift(0);

_SCKPORT_ (SCKport) &= ~SCKmode; //switch SCK to normal outout
_SCKPORT_ (SCK_BSRR) = (1<<SCKpin); //SCK high

/*** 4 LAT pulses to turn on GBC input mode */
LAT_pulse(); LAT_pulse(); LAT_pulse(); LAT_pulse();

_SCKPORT_ (SCK_BRR) = (1<<SCKpin);  //SCK low
_SCKPORT_ (SCKport) |= SCKmode; // SCK back to SPI mode

for (uint8_t count=0; count<DMnum; count++)
{dm_shift((bri[DMnum-count-1]<<1));}

while (_SPIREG_(_SPI_SR) & BSY); //finish the transmission
LAT_pulse();

sendAll(); //restore GS data

for (uint8_t count=0; count<DMnum; count++)
{dm_shift((bri[DMnum-count-1]<<1));}

}

uint16_t DMdriver::getChainValue (uint16_t pixNum)
{
	uint8_t numSeg = DMnum / 3;
	uint8_t allStops = numSeg*2 - 1;
	uint16_t DMstep = 8 * 3;
	uint16_t curSeg = pixNum / DMstep;
	
	if (pixNum < (pinNum>>1)) pixNum = pixNum + DMstep * curSeg;
	else pixNum = (allStops - curSeg) * (DMstep<<1) + DMstep + pixNum%DMstep;
	return pixNum;
}

void DMdriver::turnOff()
{
	delete [] pixel; // free the allocated array memory
}

void DMdriver::deallocLedTable()
{
	delete [] ledTable; // free the ledtable memory
}

