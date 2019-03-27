DM631, DM632, DM633, DM634 library for STM32F103

This library was tested with the Arduino IDE (STM32duino) and Atollic TrueStudio for STM32. It should work with other GCC-based IDEs, frameworks, toolchains, whatever, as it doesn't use any STM32duino/Maple specific port addressing; nor does it use the STM-approved approach. 

Usage: same as Atmega328 version (details [here](https://github.com/Ontaelio/DMdriver)), except the DMdriver object declaration:

**DMdriver** ObjName (uint8_t **Driver**, uint8_t **Number** [, uint32_t **LatchPort**, uint8_t **LatchPin** [, uint32_t **SPI_addr**]])

**Driver**: LED driver chip used. Possible values: `DM631`, `DM632`, `DM633`, `DM634`.

**Number**: the number of DM63x chips in chain.

**LatchPort**: the IO port of the pin LAT is connected to. Possible values: `__PORTA`, `__PORTB` .. `__PORTG`. Only __PORTA and __PORTB tested. *Default __PORTB*.

**LatchPin**: the number of the pin LAT is connected to, **0-15**. *Default 0*.

**SPI_addr**: the SPI used as DAI and DCK lines for the DM chip. Possible values: `__SPI1`, `__SPI2`, `__SPI3`; SPI3 untested, you'll have to change the default functions of the pins it uses yourself to make it work. Alternate pin mappings not supported in the library. *Default __SPI1*.

Default connections are:

LAT - B0

DAI - A7

DCK - A5

If you change the LAT pin, you'll need to provide it's new pin address. The address consists of the IO port name and pin number. If you're dealing with a breakout board (Discovery, Blue Pill, etc), pins are named with the letter followed by a number. The letter is the port, the number is the pin. Thus, A10 translates into __PORTA, 10; B0 into __PORTB, 0; etc.

SPI interface is used for communication. You'll need to connect DAI and DCK lines to MOSI and SCK lines of the selected SPI. The pins are:

###### SPI1
MOSI - A7

SCK - A5

###### SPI2
MOSI - B15

SCK - B13

###### SPI3*
MOSI - B5

SCK - B3

(*) SPI3 pins (if SPI3 is present) are used by other communication protocols by default. You'll need to turn these protocols off by yourself. I do not recommend using SPI3, if you really really need it you presumably know what you're doing and will be able to configure it accordingly.

If you need to change something inside the library, check the #defines at the start of the DMdriverSTM32F1.cpp file and the comments in the init() function.


