#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A

void wdt_disable(void);
void wdt_enable(void);
void ac_on(void);
void ac_off(void);
unsigned char jumper(void);

volatile unsigned char tx_enable = 1;
volatile unsigned char ac_init = 0;

unsigned char jumper(void)
{
   return ((PINB & 0x08) >> 3);
}

void wdt_disable(void)
{
   WDTCSR = 0b00011000;
   WDTCSR = 0b00000010; //Disable WDT Interrupt
}

void wdt_enable(void)
{
   WDTCSR = 0b00011000;
   WDTCSR = 0b01000010; //Enable WDT Interrupt, Change prescaler to 64ms
}

void ac_on(void)
{
   DDRC = 0x1F;
   PORTC = 17;
   ACSR = 0b00001011; //Analog Comparator enabled, rising edge interrupt
   ac_init = 0; //Reset ac_init to prevent turn on of AC from disabling transmit
}

void ac_off(void)
{
   ACSR = 0b00000011; //Analog Comparator disabled
   DDRC = 0x3F;
   PORTC = 32;
}
