#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define sbi(a, b) ((a) |= 1 << (b))       //sets bit B in variable A
#define cbi(a, b) ((a) &= ~(1 << (b)))    //clears bit B in variable A
#define tbi(a, b) ((a) ^= 1 << (b))       //toggles bit B in variable A

void initialize(void);
void init_clk(void);
void init_ports(void);
void init_tim0(void);
void init_wdt(void);
void init_ac(void);
void eeprom_write(char address, char data);
char eeprom_read(char address);

void initialize(void)
{
   init_clk();
   init_ports();
   init_tim0();
   init_ac();
}

void init_clk(void)
{
   /* CLKPR - Clock Prescaler Register
      7: Clk PC Enable
      6:4: -
      3:0: Prescaler select bits;
         0000  1
         0001  2
         0010  4
         0011  8
         0100  16
         0101  32
         0110  64
         0111  128
         1000  256
         1001+ - */
   CLKPR = 0b10000000;
   CLKPR = 0b00000000; //Set microcontroller clock to full speed, 9.8304MHz
}

void init_ports(void)
{
   /* Port as output: (DDRAn = 1)
      PORTA = 0b00000000; //1 for output high

      Port as input: (DDRAn = 0)
      temp = PINA; //PINA = 0b00000000
                   //1 is input high */

   /* DDRC - Data Direction Register C
      0b7654 3210 */
   /* Port C Alternate Functions:
      7: ADC7/OC0B            /PCINT7
      6: ADC6/OC1A/SDA     /DI/PCINT6
      5: ADC5/OC1B/DO         /PCINT5/(DAC5)
      4: ADC4     /SCL/USCK/T1/PCINT4/(DAC4)
      3: ADC3              /T0/PCINT3/(DAC3)
      2: ADC2/AIN1            /PCINT2/(DAC2)
      1: ADC1/AIN0            /PCINT1/(DAC1)
      0: ADC0/AREF            /PCINT0/(DAC0) */
   DDRC = 0b00111111;

   /* DDRB - Data Direction Register B
      0bXXXX 3210 */
   /* Port B Alternate Functions:
      7:
      6:
      5:
      4: MISO           /PCINT4
      3:                /PCINT3
      2: INT0/OC0A/CKOUT/PCINT2
      1: XTAL2          /PCINT1
      0: XTAL1    /CLKI /PCINT8 */
   DDRB = 0b00110100;

   /* DDRD - Data Direction Register D
      0b7654 3210 */
   /* PDX (PinN): Function
      7 (11):      AIN1  /PCINT23/(AIN-)
      6 (10): OC0A/AIN0  /PCINT22/(AIN+)
      5 ( 9): OC0B/T1    /PCINT21
      4 ( 2): XCK /T0    /PCINT20
      3 ( 1): OC2B/INT1  /PCINT19
      2 (32):      INT0  /PCINT18
      1 (31): TXD        /PCINT17/(GPSRX)
      0 (30): RXD        /PCINT16/(GPSTX) */
   DDRD = 0b00000010;
   sbi(PORTD,1);
}

void init_tim0(void)
{
   /* TCCR0A - Timer/Counter 0 Control Register A
      7:6: Compare Match Output A Mode (OC0A Output pin, PB2)
         00 Normal Port op. OC0A Disconnected
         01 Toggle OC0A on Compare Match
         10 Clear OC0A on Compare Match (normal)
         11 Set OC0A on Compare Match (inverted)
      5:4: COM2B 1:0 (OC0B Output pin, PA7)
      3:2: -
      1:0: WGM0 1:0 Waveform Generation Mode:
         000 Normal  0xFF  Immediate
         001 PWM PC  0xFF  Top
         010 CTC     OCRA  Immediate
         011 FastPWM 0xFF  Bottom
         100 -
         101 PWM PC  OCRA  Top
         110 -
         111 FastPWM OCRA  Bottom */
   TCCR0A = 0b00000010;

   /* TCCR0B - Timer/Counter 0 Control Register B
      7:6: Force Output Compare A:B
      5:4: -
      3: WGM0 2 Waveform Generation Mode
      2:0: Clock Select:
         000 No Clock Source (Timer/Counter stopped)
         001 ClkIO/1
         010 ClkIO/8
         011 ClkIO/64
         100 ClkIO/256
         101 ClkIO/1024
         110 Ext. Clk on T0, FET
         111 Ext. Clk on T0, RET */
   TCCR0B = 0b00000000;

   /* TIMSK0 - Timer/Counter Interrupt Mask Register
      7:3: -
      2: TCNT0 Output Compare Match B Interrupt Enable
      1: TCNT0 Output Compare Match A Interrupt Enable
      0: TCNT0 Overflow Interrupt Enable */
   TIMSK0 = 0b00000010;
}

void init_wdt(void)
{
   /* WDTCSR - Watchdog Timer Control and Status Register
      7: WDT Interrupt Flag
      6: Watchdog Timeout Interrupt Enable
      5: WDT Prescaler 3
      4: Watchdog Change Enable
      3: Watchdog Enable
      2:0: WDT Prescaler 2:0:
         0000  16ms
         0001  32ms
         0010  64ms
         0011  0.125s
         0100  0.25s
         0101  0.5s
         0110  1s
         0111  2s
         1000  4s
         1001  8s
         1010+ - */
   WDTCSR = 0b00011000;
   WDTCSR = 0b01000010; //Change prescaler to 64ms
}

void init_ac(void)
{
   /* ADCSRB - ADC Status and Control Register B
      7: -
      6: Analog Comparator Multiplexer Enable
      5:3: -
      2:0: ADC Auto Trigger Source:
         000 Free Running Mode
         001 Analog Comparator
         010 INT0
         011 TCNT0 Compare Match A
         100 TCNT0 OVF
         101 TCNT1 Compare Match B
         110 TCNT1 OVF
         111 TCNT1 Capture Event */
   ADCSRB = 0b01000000; //Analog Comparator Multiplexer Enabled

   /* ADMUX - ADC Multiplexer Selection Register
      7:6: REFS1:0:
         00 AREF, internal Vref turned off
         01 AVCC
         10 Reserved
         11 Internal 1.1v voltage reference
      5: ADC Left Adjust Result:
         0: ADCH = 0b------98
            ADCL = 0b76543210 (10-bit, append ADCH to ADCL)
         1: ADCH = 0b98765432
            ADCL = 0b10------ (8-bit, read ADCH only)
      4: -
      3:0: MUX3:0: Analog Channel Selection Bits:
         0000 ADC0 (PC0)
         0001 ADC1 (PC1)
         0010 ADC2 (PC2)
         0011 ADC3 (PC3)
         0100 ADC4 (PC4)
         0101 ADC5 (PC5)
         0110 ADC6
         0111 ADC7
         1000-1101 -
         1110 1.1V (Internal Vref)
         1111 0V (AGND) */
   ADMUX = 0b01000101; //Use ADC5 as reference for analog comparator (synthesize 0-5V with R-2R, input = ADC5)

   /* ACSR - Analog Comparator Status and Control Register 
      7: AC Disable
      6: AC Bandgap Select
      5: AC Output
      4: AC Interrupt Flag
      3: AC Interrupt Enable
      2: AC Input Capture Enable
      1:0: AC Interrupt Mode Select
         00 Interrupt on Output Toggle
         01 -
         10 Interrupt on Falling Output Edge
         11 Interrupt on Rising Output Edge */
   ACSR = 0b00001011;
}

void eeprom_write(char address, char data)
{
   while(0b00000010 & EECR); //Wait for EEPE to clear (EEPROM not busy)

   EECR = 0b00000000;

   EEARL = address;
   EEDR = data;

   sbi(EECR, 2);
   sbi(EECR, 1);
}

char eeprom_read(char address)
{
   while(0b00000010 & EECR); //Wait for EEPE to clear (EEPROM not busy)

   EECR = 0b00000000;

   EEARL = address;

   sbi(EECR, 0);

   return EEDR;
}
