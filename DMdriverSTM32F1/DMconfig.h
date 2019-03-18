/************************************************************************
/*  The DMdriver Config file
/*  Comment/uncomment and change defines below if needed
/*  (c) 2016 Dmitry Reznikov - dmitry@ultiblink.com 
/***********************************************************************/

//#pragma GCC optimize ("-O2") // O2 optimization seems best for this library
							 // when used together with DM256PWM

#define DM256PWM *pixel[k] // 8-bit resolution mode
                           // saves SRAM at the cost of a little speed
                           // possible values:
						   // *pixel[k] // square the value, slower but good with fade-outs
						   // <<8 // shift 8 bits (multiply by 256) - a bit faster but not as fancy
						   
#define DMCHAIN 3 // remap the LED driver outputs to be consistent with their physical locations
                    // when used in chain. Namely, if you put three DM chips on a breadboard, you'll want
                    // their outputs to be numbered 0-23 on the one side and 24-47 on the other.
					// DMCHAIN allows to do this without filling SRAM with look-up tables
					// (at the cost of speed and some program memory).
					// The value is the number of DM chips per single unit (1 in the aforementioned
					// case; 3 if used with my UltiBlink board).


// DO NOT change the following					
#define DMLEDTABLE uint8_t // defines the type of data in the lookup table if used