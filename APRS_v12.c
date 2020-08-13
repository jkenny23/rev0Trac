/*¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯¯*\
|    rev0Trac APRS Tracker     |
|                              |
|      by: Justin Kenny        |
|          19.1.2010           |
|            v1.2              |
\*____________________________*/

#define F_CPU 9830400 //Default Clock Speed 1MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "init.c"

#define sbi(a, b) ((a) |= 1 << (b))     //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))  //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))     //toggles bit B in variable A

/* User Defines/Constants */
#define NUMBYTES 77  //Total number of bytes in packet
#define NUMFLAGS 3   //Number of flags at beginning and end of packet
#define NUMPREAMB 12 //Number of preamble bytes
#define NUMADDR 21   //Number of address bytes
#define DELAYTIME 30 //Time between transmissions, in seconds * 2 (120 = every minute)
#define PTT 2        //PTT output pin on microcontroller -- PB2
#define D_KEY 500    //Key on delay
#define D_UNKEY 500  //Unkey delay

#define MAXCOMMAS 14 //Max commas to store positions of (14 for GGA sentence)
#define MAXGPS 82    //Max NMEA string size
#define MAXLAT 9     //Max latitude size in bytes
#define MAXLONG 10   //Max longitude size in bytes
#define MAXTIME 10   //Max time size in bytes
#define MAXALTI 8    //Max altitude size in bytes
#define LATSTART (NUMPREAMB + NUMADDR + 4)   //Latitude start index
#define NSSTART (LATSTART + MAXLAT - 1)      //North/South start index
#define LONGSTART (NSSTART + 2)              //Longitude start index
#define EWSTART (LONGSTART + MAXLONG - 1)    //East/West start index
#define CMTSTART (EWSTART + 2)               //Comment start index
#define TIMEEND (CMTSTART + 6 - 1)           //Timestamp end index
#define CMTEND (TIMEEND + 0)                 //Comment end index

/* Function Declarations */
char data_bit(char byte, char bit);
void fcs_update(void);
void start(void);
void end(void);
void phaseupdate(void);
void tonehigh(unsigned int decisec);
void tonelow(unsigned int decisec);

/* Volatile Variable Declarations */
volatile unsigned char counter = 0; //Sample counter
volatile signed   char n = 0;       //Sample index
volatile unsigned char mark = 0;    //  0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  24  25  26  27  28  29  30  31  32
volatile unsigned char coslow[33]  = { 63, 62, 60, 57, 54, 49, 44, 38, 32, 27, 21, 15, 10,  6,  3,  1,  0,  0,  1,  3,  6, 10, 15, 21, 27, 32, 38, 44, 49, 54, 57, 60, 62}; //LUT for the 1200Hz tone
volatile unsigned char coshigh[33] = { 63, 61, 55, 47, 36, 26, 15,  7,  1,  0,  1,  7, 15, 26, 36, 47, 55, 61, 63, 61, 55, 47, 36, 26, 15,  7,  1,  0,  1,  7, 15, 26, 36}; //LUT for the 2200Hz tone
volatile signed   char phshl[33]   = {  0,  1,  2,  2,  3,  4,  5,  6,  7,  7,  8,  9, 10, 11, 12, 13, 13, 14, 15, 16,-16,-16,-15,-14,-13,-12,-11,-10,-10, -9, -8, -7, -6}; //LUT for 2200Hz -> 1200Hz phase diff.
volatile signed   char phslh[33]   = {  0, -1, -1, -1, -2, -2, -2, -3, -3, -4, -5, -5, -5, -6, -6, -7, -7, -8, -8, -9, -9,-10,-10,-10,-11,-11,-12,-12,-13,-13,-14,-14,-14}; //LUT for 1200Hz -> 2200Hz phase diff.
volatile char data[NUMBYTES] = 
{0x00,   //Zeros for synchronization
0x00,
0x00,
0x00,
0x00,
0x00,
0x00,
0x00,
0x00,
0x00,

0x7E,    //Flags
0x7E,
0x7E,    //(NUMPREAMB)

0x82,    //A
0xA0,    //P
0xA4,    //R
0xA6,    //S
0x40,    //
0x40,    //
0xE0,    //0 -- Follows SSID byte format of CRRSSID0, where C is the command/response bit (1 in this case), RR is 11, SSID is the SSID number, and a 0 is appended to the end.

0x96,    //K (NUMPREAMB+8)
0x94,    //J
0x6C,    //6
0x96,    //K
0xA6,    //S
0xA8,    //T
0xE2,    //1 -- See above about SSID byte.

0xAE,    //W (NUMPREAMB+15)
0x92,    //I
0x88,    //D
0x8A,    //E
0x62,    //1
0x40,    //  -- Space.
0x63,    //1 -- End of address fields, LSB = 1.

0x03,    //Control -- 0x03 for UI (Unnumbered Information) frame, the default for APRS.
0xF0,    //PID -- 0xF0 for no Layer 3 protocol implementation, the default for APRS.

0x21,    //!
0x30,    //0 -- Lat start @37 (LATSTART)
0x30,    //0
0x30,    //0
0x30,    //0
0x2E,    //.
0x30,    //0
0x30,    //0
0x4E,    //N -- N/S start @44 (NSSTART)
0x2F,    /// -- Forward slash.
0x30,    //0 -- Long start @46 (LONGSTART)
0x30,    //0
0x30,    //0
0x30,    //0
0x30,    //0
0x2E,    //.
0x30,    //0
0x30,    //0
0x57,    //W -- E/W start @54 (EWSTART)
0x3E,    //>
0x30,    //0 -- Comment start @56 (CMTSTART)
0x30,    //0
0x30,    //0
0x30,    //0
0x30,    //0
0x30,    //0 -- Time end @61 (TIMEEND)
0x49,    //I -- Valid/Invalid
0x20,    //  -- Space
0x72,    //r
0x65,    //e
0x76,    //v
0x30,    //0
0x54,    //T
0x72,    //r
0x61,    //a
0x63,    //c

0xFF,    //FCS High Byte -- NUMBYTES - NUMFLAGS - 2
0xFF,    //FCS Low Byte

0x7E,    //Flag -- NUMBYTES - NUMFLAGS
0x7E,
0x7E};   // -- NUMBYTES - 1
volatile short unsigned int fcs = 0xFFFF;
volatile unsigned char byteindex = 0;
volatile signed char bitindex = 0;
volatile unsigned char ended = 0;
volatile unsigned char onecount = 0;
volatile unsigned char stuff = 3;

volatile unsigned char byteRX = 0;
volatile unsigned char done = 1;
volatile unsigned char gpsgood = 0;
volatile unsigned char delay = 1;

volatile unsigned char gpsdata[MAXGPS];
volatile unsigned char gpsindex = 0;
volatile unsigned char commas[MAXCOMMAS];
volatile unsigned char lat[MAXLAT] = "0000.0000";
volatile unsigned char lng[MAXLONG] = "00000.0000";
volatile unsigned char time[MAXTIME] = "123456.789";
volatile unsigned char alti[MAXALTI] = "000000.0";
volatile unsigned char ns = 'S';
volatile unsigned char ew = 'E';
volatile unsigned char valid = 0;

volatile unsigned char rx_end = 1;
volatile unsigned char waitcnt = 0;

#include "gps.c"

int main(void)
{
   initialize();

   /* Program Begin */
   sei(); //Global Interrupt Enable

   OCR0A = 30; //Set timer0 for APRS; 33*1200Hz interrupt frequency

   USART_Init(); //Start USART to begin reading GPS data

   _delay_ms(2000);
   if(jumper() == 0)
      start(); //Send a packet to check if the device is working
   ac_on();

   while(1)
   {
      //All operations are intiated by interrupts, nothing is needed here
      if(tx_enable)
      {
         sbi(PORTB,4);
         while(tx_enable);
      }
      cbi(PORTB,4);
   }

   return 0;
}

char data_bit(char byte, char bit) //Returns a bit (0x01 or 0x00) in a byte of the main data array
{
   return ((data[(int)byte]>>bit) & 0x01);
}

void fcs_update(void) //Updates the FCS checksum bit-by-bit
{
   unsigned char shiftbit = 0x0001 & fcs; //Store bit rotated off in variable shiftbit
   fcs = fcs >> 1; //Shift fcs right by 1
   if(shiftbit != data_bit(byteindex,bitindex)) //If shiftbit doesn't match the data being sent, xor with 0x8408
   {
      fcs ^= 0x8408;
   }
}

void start(void)
{
   UCSR0B = 0x00; //Disable USART Receiver, disable receive interrupt
   sbi(PORTB,PTT); //Key the radio, delay
   _delay_ms(D_KEY);
   fcs = 0xFFFF; //Reset frame check sequence
   mark = data_bit(0,0); //Set first bit of audio output
   bitindex = 1; //Increment bit index to reflect this
   ac_off(); //Disable AC interrupt while transmitting
   TCCR0B = 0b00000010; //Timer on
}

void end(void)
{
   TCCR0B = 0b00000000; //Timer off
   PORTC = 32; //Output low
   byteindex = 0; //Reset all flags/counters
   ended = 0;
   n = 0;
   counter = 0;
   _delay_ms(D_UNKEY); //Delay, unkey radio
   cbi(PORTB,PTT);
   UCSR0B = (1<<RXEN0)|(1<<RXCIE0); //Enable USART Receiver, enable receive interrupt
   ac_on(); //Re-enable AC interrupt

   _delay_ms(500);
   tonehigh(1);
   _delay_ms(50);
   if(valid)
   {
      tonehigh(1);
      _delay_ms(50);
      tonehigh(1);
   }
   else
   {
      tonelow(2);
   }
}

ISR(USART_RX_vect)
{
   ac_off();
	byteRX = USART_Receive();
	
	if(byteRX == '$') //$ = Start of NMEA Sentence
	{
		gpsindex = 0;
		done = 0;
      gpsgood = 1;
	}
	else if(byteRX == 0x0D) //<CR> = End of Transmission
	{
		done = 1;

		if(gpsdata[4] == 'G') //Make sure this is a GGA sentence
		{         
			parse_nmea();
		}
      else if(gpsdata[4] == '0') //PRG0X sentence
      {
         parse_prog();
      }
	}
	if(done != 1)
	{
		gpsdata[gpsindex] = byteRX;
		gpsindex++;
	}
   ac_on();
}

void tonehigh(unsigned int decisec)
{
   unsigned int i;

   for(i=0;i<(44*decisec+1);i++)
   {
      for(n=0;n<33;n++)
      {
         PORTC = coslow[n];
         _delay_us(68);
      }
   }
}

void tonelow(unsigned int decisec)
{
   unsigned int i;

   for(i=0;i<(22*decisec+1);i++)
   {
      for(n=0;n<33;n++)
      {
         PORTC = coslow[n];
         _delay_us(137);
      }
   }
}

/* This block checks for traffic before sending a packet.
   The first interrupt service routine is run every time 
   the comparator is triggered (when the squelch opens or
   closes on the radio). If there has been no activity on
   the comparator input for 3 seconds, then it is ok to 
   send a packet. The WDT ISR just counts up until 3 
   seconds. */
ISR(ANALOG_COMP_vect) //10 = falling edge, 11 = rising edge
{
   if(ac_init) //This if statment prevents turn-on transient from disabling transmit
   {
      waitcnt = 0;
      tx_enable = 0;
   }

   if(ACSR & 0x03) //If AC set to detect rising edge
   {
      rx_end = 0; //Rising edge occurred
      ACSR = 0b00001010; //Set interrupt trigger to falling edge
   }
   else if(ACSR & 0x02) //If AC set to detect falling edge
   {
      rx_end = 1; //Falling edge occurred
      ACSR = 0b00001011; //Set interrupt trigger to rising edge
   }

   ac_init = 1; //Allow AC to disable transmit
}

/* This is the heart of the program; the APRS audio
   generator. It is run 33*1200 times per second,
   for each of the 33 samples of either a 1200Hz or 
   2200Hz sine wave, at 1200 baud. This service
   routine handles data formatting such as the FCS,
   bit stuffing, and NRZI. */
ISR(TIMER0_COMPA_vect)
{
   if(mark == 1)
   {
      PORTC = coslow[n];   //Output next sample of a 1200Hz cosine
   }
   else
   {
      PORTC = coshigh[n];  //Output next sample of a 2200Hz cosine
   }

   counter++; //Increment sample counter and sample index
   n++;

   if(mark == 0)
   {
      if(n>32)
         n = 15; //If sample index overflows, restart at start point for 2200Hz cosine
   }
   else
   {
      if(n>32)
         n = 0; //If sample index overflows, restart at start point for 1200Hz cosine
   }

   if((ended == 1) && (counter > 32))
   {
      end();
   }

   if(counter > 32)
   {
      counter = 0;
      //Change next sent tone according to NRZI (non-return to zero inverted)
      if((data_bit(byteindex,bitindex) == 0))
      {
         onecount = 1;
         mark ^= 0x01; //If the next bit is zero, toggle the output, else no change

         phaseupdate(); //Update the sample index to ensure correct phase
      }
      else if((data_bit(byteindex,bitindex) == 1))
      {
         onecount++;

         if(data[(int)byteindex] == 0x7E)
         {
            onecount = 1;
         }
         else if(onecount > 5) //If there are 5 or more ones in a row, insert a zero
         {
            stuff = 2;
            onecount = 1;
         }
      }

      if(stuff > 0 && stuff != 3)
      stuff--;
   
      if(stuff > 0) //Update FCS, increment bit index
      {
         if(byteindex < (NUMBYTES - NUMFLAGS - 2) && byteindex > NUMPREAMB)
            fcs_update();
         bitindex++;
      }
      else //Insert zero, update phase, do not update FCS, do not increment bit index
      {
         stuff = 3;

         if((data_bit(byteindex,bitindex) == 1))
         {
            mark ^= 0x01;

            phaseupdate();
         }
      }

      if(bitindex>7) //Done transmitting a byte
      {
         bitindex = 0;
         byteindex++;
         if(byteindex == (NUMBYTES - NUMFLAGS - 2)) //Transmit FCS bytes
         {
            fcs ^= 0xFFFF;
            data[(NUMBYTES - NUMFLAGS - 1)] = (char)((fcs >> 8) & 0x00FF);
            data[(NUMBYTES - NUMFLAGS - 2)] = (char)(fcs & 0x00FF);
         }
         else if(byteindex == NUMBYTES) //Done sending packet, end after last bit sent
         {
            ended = 1;
         }
      }
   }
}

void phaseupdate(void)
{
   n--;
   if(mark == 0)
   {
      n += phslh[n]; //Last value was 1, add phase shift for 1200Hz to 2200Hz
      if(n>32)
         n = 14;  //If 2200Hz index overflows, start at position 15
   }
   else
   {
      n += phshl[n]; //Last value was 0, add phase shift for 2200Hz to 1200Hz
      if(n>32)
         n = -1;   //If 1200Hz index overflows, start at position 0
   }
   n++;
}
