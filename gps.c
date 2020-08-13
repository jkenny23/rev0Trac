#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "util.c"

#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A

/* User Defines/Constants */
//Set Baud Rate to 4800, calculated by: Fosc/(16*Baud) - 1
#define CBAUD 127
#define TFCWAIT 3 //Delay, in seconds after traffic is heard, until transmitting

/* Function Declarations */
void USART_Init(void);
unsigned char USART_Receive(void);
void parse_nmea(void);
void parse_prog(void);
void update_packet(void);

void USART_Init(void)
{
   cli(); //Disable interrupts
   UBRR0 = CBAUD; //Set Baud Rate
   UCSR0B = (1<<RXEN0)|(1<<RXCIE0); //Enable USART Receiver, enable receive interrupt
   UCSR0C = (3<<UCSZ00); //Set up for 8-bit data frames, defaults to no parity, 1 stop bit, asynchronous mode
   sei(); //Enable interrupts
}

unsigned char USART_Receive(void)
{
   while(!(UCSR0A & (1<<RXC0))); //Wait until data received
   return UDR0; //Return received byte
}

void parse_nmea(void)
{
   int i = 0, n;

	for(n=0;n<MAXCOMMAS;n++) //Find the positions of all commas in the NMEA sentence, put positions in commas[]
   {
      for(;gpsdata[i]!=0x2C;i++); //Find next comma; continue stepping through the array until we find 0x2C (,)
      commas[n] = i; //Store the index in commas[] array
      i++;
   }
	
	if(gpsdata[commas[5]+1] != 0x30) //Make sure we have GPS fix; 0 = invalid
   {    
      for(i=commas[1]+1;i<commas[2];i++)
      {
         lat[i-(commas[1]+1)] = gpsdata[i]; //Load latitude into lat[] array from stored NMEA string
      }
      ns = gpsdata[commas[2]+1];
      
      for(i=commas[3]+1;i<commas[4];i++)
      {
         lng[i-(commas[3]+1)] = gpsdata[i]; //Load longitude into lng[] array from stored NMEA string
      }
      ew = gpsdata[commas[4]+1];
		
		for(i=commas[0]+1;i<commas[1];i++)
      {
         time[i-(commas[0]+1)] = gpsdata[i]; //Load time into time[] array from stored NMEA string
      }
		valid = 1;

		update_packet(); //Update the packet with new good GPS data
   }
   else //Else update the timestamp, but retain old GPS data
   {
   	for(i=commas[0]+1;i<commas[1];i++)
   	{
   		time[i-(commas[0]+1)] = gpsdata[i];
   	}
      valid = 0;

   	update_packet();
   }

   waitcnt++; //Increment wait counter
   if(waitcnt>TFCWAIT) //If wait counter reaches predefined value, re-enable transmit
   {
      tx_enable = 1;
      waitcnt = (TFCWAIT+1);
   }
}

//Programming sentence parser, updates APRS packet with information in the following format:
//$PRG01,APRS  0,KJ6KST1,WIDE2 2,!0000.00N/00000.00W>,T,V,rev0Trac,,,,,,,*7D
//0     6       14      22      30                   51  55                73
//       Destin.,Source ,Path   ,Data format ,Timestamp,Valid flag,Comment,<Extra>,CRC
void parse_prog(void)
{
   int i = 0, n;

	for(n=0;n<MAXCOMMAS;n++) //Find the positions of all commas in the PRG sentence, put positions in commas[]
   {
      for(;gpsdata[i]!=0x2C;i++); //Find next comma; continue stepping through the array until we find 0x2C (,)
      commas[n] = i; //Store the index in commas[] array
      i++;
   }

   for(i=commas[0]+1,n=0;i<commas[1];i++,n++)
   {
      data[(NUMPREAMB+1+n)] = gpsdata[i]; //Copy destination address into packet
   }
   for(i=commas[1]+1,n=0;i<commas[2];i++,n++)
   {
      data[(NUMPREAMB+8+n)] = gpsdata[i]; //Copy source address into packet
   }
   for(i=commas[2]+1,n=0;i<commas[3];i++,n++)
   {
      data[(NUMPREAMB+15+n)] = gpsdata[i]; //Copy path into packet
   }
}

void update_packet(void) //This function updates the APRS packet with GPS information
{
	int i;
	
	for(i=LATSTART;i<NSSTART;i++)
	{
		data[i] = lat[i-LATSTART]; //Copy latitude into packet at position LATSTART
	}
	data[i] = ns;
	for(i=LONGSTART;i<EWSTART;i++)
	{
		data[i] = lng[i-LONGSTART]; //Copy longitude into packet at position LONGSTART
	}
	data[i] = ew;
	for(i=CMTSTART;i<(TIMEEND+1);i++)
	{
		data[i] = time[i-CMTSTART]; //Copy time into packet at position CMTSTART
	}
   if(valid) //If GPS coordinates are valid, put a V after the timestamp
   {
      data[(TIMEEND+1)] = 'V';
   }
   else //Else put an I after the timestamp
   {
      data[(TIMEEND+1)] = 'I';
   }

   if(delay == DELAYTIME) //If DELAYTIME seconds have passed, begin packet transmission
   {
      if(jumper()) //If jumper is set (lower 2 pins bridged), must have GPS lock, no traffic to tx
      {
         if(tx_enable && valid)
         {
            start();
            delay = 1;
         }
         else
         {
            delay = DELAYTIME;
         }
      }
      else
      {
         start();
         delay = 1;
      }
   }
   else //Else continue counting up
   {
      delay++;
   }
}
