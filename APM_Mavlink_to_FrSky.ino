/*
   @author    Nils Hцgberg
   @contact    nils.hogberg@gmail.com
    @coauthor(s):
     Victor Brutskiy, 4refr0nt@gmail.com, er9x adaptation

	Further adaption for ersky9x/er9x by Mike Blandford

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Version 1.1.117 03.10.2014

#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static prog_char __c[] PROGMEM = (s); &__c[0];})) 

//#include <SoftwareSerial.h>
//#include <FlexiTimer2.h>
//#include <FastSerial.h>
//#include "SimpleTelemetry.h"
#include "Mavlink.h"
#include "FrSky.h"
//#include "SimpleFIFO.h"
#include "GCS_MAVLink.h"
#include "Aserial.h"

#define NOINLINE __attribute__ ((noinline))

// temporary
extern void initHubUart( ) ;
extern void initSportUart( uint8_t mode ) ;
extern void startSportTransmit() ;

uint32_t micros( void ) ;
uint32_t millis( void ) ;

volatile byte ActionFrskySend = 0 ;
volatile bool sportAvailable = 0 ;
uint8_t SendVfas ;
uint8_t SendCell ;
uint8_t SportInTx ;

uint16_t MillisPrecount ;
uint16_t lastTimerValue ;
uint32_t TotalMicros ;
uint32_t TotalMillis ;
uint8_t Correction ;


// Do not enable both at the same time
//#define DEBUG
//#define DEBUGFRSKY

// Lighting
// Output pins   ( Note: 5,6,11,12 already used by other needs )
#define FRONT      7
#define REAR       8
#define LEFT       9
#define RIGHT     10
#define WHITE      2
#define RED        3
#define BLUE       4
#define GREEN     13

//static int  pattern_index = 0;
//static int  last_mode = 0;

//char LEFT_STAB[]   PROGMEM = "1111111110";   // pattern for LEFT  light, mode - STAB
//char RIGHT_STAB[]  PROGMEM = "1111111110";   // pattern for RIGHT light, mode - STAB
//char FRONT_STAB[]  PROGMEM = "1111111110";   // pattern for FRONT light, mode - STAB
//char REAR_STAB[]   PROGMEM = "1111111110";   // pattern for REAR  light, mode - STAB

//char LEFT_AHOLD[]  PROGMEM = "111000";  // medium blink
//char RIGHT_AHOLD[] PROGMEM = "111000"; 
//char FRONT_AHOLD[] PROGMEM = "111000"; 
//char REAR_AHOLD[]  PROGMEM = "111000"; 

//char LEFT_RTL[]    PROGMEM = "10";  // fast blink
//char RIGHT_RTL[]   PROGMEM = "10"; 
//char FRONT_RTL[]   PROGMEM = "10"; 
//char REAR_RTL[]    PROGMEM = "10"; 

//char LEFT_OTHER[]  PROGMEM = "1";  // always ON
//char RIGHT_OTHER[] PROGMEM = "1"; 
//char FRONT_OTHER[] PROGMEM = "1"; 
//char REAR_OTHER[]  PROGMEM = "1"; 

//char *left[]  PROGMEM = {LEFT_STAB,  LEFT_AHOLD,  LEFT_RTL,  LEFT_OTHER };
//char *right[] PROGMEM = {RIGHT_STAB, RIGHT_AHOLD, RIGHT_RTL, RIGHT_OTHER};
//char *front[] PROGMEM = {FRONT_STAB, FRONT_AHOLD, FRONT_RTL, FRONT_OTHER};
//char *rear[]  PROGMEM = {REAR_STAB,  REAR_AHOLD,  REAR_RTL,  REAR_OTHER};

// Comment this to run simple telemetry protocol
#define MAVLINKTELEMETRY

//#ifdef MAVLINKTELEMETRY
Mavlink ctelemetry ;
//#else
//SimpleTelemetry ctelemetry ;
//#endif 

//#ifdef MAVLINKTELEMETRY
//Mavlink *dataProvider;
//#else
//SimpleTelemetry *dataProvider;
//#endif

//FastSerialPort0(Serial);
//FrSky *frSky;
FrSky frSky ;

#ifdef DEBUG
SoftwareSerial *debugSerial;
#elif defined DEBUGFRSKY
SoftwareSerial *frskyDebugSerial;
#endif

uint8_t Fcounter = 0;
unsigned long   rateRequestTimer = 0;
unsigned long LastMillis ;

struct t_fifo128
{
	uint8_t fifo[128] ;
	uint8_t in ;
	uint8_t out ;
} RxFifo ;

//void put_fifo128( struct t_fifo128 *pfifo, uint8_t byte )
//{
//  uint8_t next = (pfifo->in + 1) & 0x7f;
//	if ( next != pfifo->out )
//	{
//		pfifo->fifo[pfifo->in] = byte ;
//		pfifo->in = next ;
//	}
//}

int16_t get_fifo128( struct t_fifo128 *pfifo )
{
	uint8_t rxbyte ;
	if ( pfifo->in != pfifo->out )				// Look for char available
	{
		rxbyte = pfifo->fifo[pfifo->out] ;
		pfifo->out = ( pfifo->out + 1 ) & 0x7F ;
		return rxbyte ;
	}
	return -1 ;
}

static void initUart()
{
	UBRR0H = 0 ;
	UBRR0L = 16 ;		// For 57600 baud
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01 ) ;
	UCSR0A = 0 ;
	DDRD |= 0x02 ;	// Enable output pin
	PORTD |= 1 ;		// Enable pullup on input pin
	UCSR0B = (1<<RXEN0) | ( 1<<TXEN0) | ( 1<<RXCIE0) ;	// Enable receiver and tx
	UDR0 = 0x55 ;
}

//uint8_t FOerrors ;
//uint8_t FOproblem ;
//uint8_t FIerrors ;


ISR(USART_RX_vect)
{
  uint8_t stat ;
  uint8_t data ;
  
	UCSR0B &= ~(1 << RXCIE0); // disable Interrupt
	sei() ;

	stat = UCSR0A; // USART control and Status Register 0 A
  
	if (stat & ((1 << FE0) | (1 << DOR0) | (1 << UPE0)))
  { // discard buffer and start fresh on any comms error
 		data = UDR0 ; // USART data register 0
//		FOerrors += 1 ;
//		FOproblem = stat ;
	}
 	else
	{
  	uint8_t next = (RxFifo.in + 1) & 0x7f;
		if ( next != RxFifo.out )
		{
			RxFifo.fifo[RxFifo.in] = UDR0 ;
			RxFifo.in = next ;
		}
		else
		{
//			FIerrors += 1 ;
 			data = UDR0 ; // USART data register 0
		}
	}
	data = UCSR0B | (1 << RXCIE0) ; // enable Interrupt
	cli() ;
  UCSR0B = data ; // enable Interrupt
}

/*
ISR(USART0_UDRE_vect)
{
}

*/

void setup()
{

//// Debug serial port pin 11 rx, 12 tx
//#ifdef DEBUG
//   debugSerial = new SoftwareSerial(11, 12);
//   debugSerial->begin(38400);
//   debugSerial->flush();
//#elif defined DEBUGFRSKY
//   frskyDebugSerial = new SoftwareSerial(11, 12);
//   frskyDebugSerial->begin(38400);
//#endif
   
   // FrSky data port pin 6 rx, 5 tx
//   frSkySerial = new SoftwareSerial(6, 5, true);
//   frSkySerial->begin(9600);

// Port D bits:
// Bit 2 open hub, closed SPort
// Bit 3 open = Send Vfas value
// Bit 4 closed = send pseudo cells
// Bit 5 is used for the hub/SPort serial data output
// Bit 6 closed = SPort used in Tx (send 7E 98 before data)


	DDRB |= 0x38 ;

	if ( ( PIND & 0x40 ) == 0 )
	{
		SportInTx	= 1 ;
	}
	
	if ( PIND & 0x04 )
	{
		initHubUart( ) ;
	}
	else
	{
		sportAvailable = 1 ;
		initSportUart( SportInTx ) ;
	}
	
	if ( PIND & 0x08 )
	{
		SendVfas = 1 ;
	}
	
	if ( ( PIND & 0x10 ) == 0 )
	{
		SendCell = 1 ;
	}

   // Incoming data from APM
	initUart() ;
//   Serial.begin(57600);
//   Serial.flush();
	 sei() ;
   
#ifdef DEBUG
   debugSerial->println("");
   debugSerial->println("Mavlink to FrSky converter start");
   debugSerial->println("Initializing...");
   debugSerial->print("Free ram: ");
   debugSerial->print(freeRam());
   debugSerial->println(" bytes");
#endif
//#ifdef MAVLINKTELEMETRY
////   dataProvider = new Mavlink(&Serial);
//   dataProvider = &ctelemetry ;
//#else
//   dataProvider = &ctelemetry ;
//#endif 
//   frSky = new FrSky();
#ifdef DEBUG
   debugSerial->println("Waiting for APM to boot...");
#endif
  // Initializing output pins
//  pinMode(LEFT, OUTPUT);
//  pinMode(RIGHT,OUTPUT);
//  pinMode(FRONT,OUTPUT);
//  pinMode(REAR, OUTPUT);
//  pinMode(RED,  OUTPUT);
//  pinMode(GREEN,OUTPUT);
//  pinMode(BLUE, OUTPUT);
//  pinMode(WHITE,OUTPUT);

   // Blink fast a couple of times to wait for the APM to boot
   for (int i = 0; i < 200; i++)
   {
//      if (i % 2)
//      {
//         AllOn();
//      }
//      else
//      {
//         AllOff();
//      }
      delay(50);
   }
#ifdef DEBUG
   debugSerial->println("");
   debugSerial->println("Starting Timer...");
#endif
//		initTimer2() ;
//   FlexiTimer2::set( 100, 1.0/1000, sendFrSkyData ); // call every 100 1ms "ticks"
//   FlexiTimer2::start();
#ifdef DEBUG
   debugSerial->println("Initialization done.");
   debugSerial->print("Free ram after setup: ");
   debugSerial->print(freeRam());
   debugSerial->println(" bytes");
#endif
	LastMillis = millis() ;
}

//uint8_t DebugOutputOn = 0 ;

void loop()
{

//    if(millis() - p_preMillis > LOOPTIME) {
      // save the last time you blinked the LED 
//      p_preMillis = millis();   
      // Update base lights
//    }


//	if ( DebugOutputOn == 0 )
//	{
//		if ( millis() > 20000 )
//		{
//			DebugOutputOn = 1 ;
//		}
//	}
	
	uint8_t mavOnline ;
	mavOnline = ctelemetry.heartCounter > 2 ;
		 
#ifdef MAVLINKTELEMETRY
	if ( mavOnline )
	{
   if( ctelemetry.enable_mav_request || ((millis() - ctelemetry.lastMAVBeat) > 5000) )
   {
     if(millis() - rateRequestTimer > 2000)
		 {
//         Serial.flush();
         ctelemetry.reset();
         for(int n = 0; n < 5; n++)
         {
#ifdef DEBUG
            debugSerial->print("Making rate request. ");
            debugSerial->println(millis() - ctelemetry.lastMAVBeat);
#endif
            ctelemetry.makeRateRequest();
            delay(50);

         }
         ctelemetry.enable_mav_request = 0;
         rateRequestTimer = millis();
     }
   }
	}
#endif

  processData();

	uint8_t delayTime = 125 ;
	if ( SportInTx )
	{
		delayTime = 24 ;
	}
	unsigned long lm = LastMillis + delayTime ;
  if (millis() >= lm )
	{
		LastMillis = lm ;
		if ( sportAvailable == 0 )
		{
			sendFrSkyData() ;
		}
		if ( SportInTx )
		{
			if ( sendStatus == LOADED )
			{
				startSportTransmit() ;				
			}
		}
	}	
		
	if ( sportAvailable )
	{
    frSky.queueFrSkySport() ;
	}


}

#define FRSKY_PENDING_5HZ		0x01
#define FRSKY_PENDING_1HZ		0x02
#define FRSKY_PENDING_05HZ	0x04

void sendFrSkyData()
{
	static uint8_t pending = 0 ;
//   updateLights(); // lights every 100ms

   if ((Fcounter % 4) == 0)			 	    // 500mS
	 {
			if ( ctelemetry.msgCounter )
			{
   			if ((Fcounter % 8) != 0)
				{
					PORTB ^= 0x20 ;	// debug
				}
				ctelemetry.msgCounter = 0 ;
			}
	 }

   Fcounter++;
   if ((Fcounter % 40) == 0)          // Send 5000 ms frame
   {
      Fcounter = 0;
			pending |= FRSKY_PENDING_05HZ ;
   } 
   if ((Fcounter % 8) == 0)			 	    // Send 1000 ms frame
	 {
			PORTB ^= 0x20 ;	// debug
			pending |= FRSKY_PENDING_1HZ ;
      if ( ctelemetry.msg_timer > 0 )
			{
         ctelemetry.msg_timer -= 1 ;  // counter -1 sec
      }
   }
	 else if ( (Fcounter % 2) == 0 )			  // Send 250 ms frame if not 1000 mS frame
	 {
			pending |= FRSKY_PENDING_5HZ ;
   }

	 if ( pending & FRSKY_PENDING_5HZ )
	 {
			pending &= ~FRSKY_PENDING_5HZ ;
      frSky.sendFrSky5Hz();
	 }
	 else if ( pending & FRSKY_PENDING_1HZ )
	 {
			pending &= ~FRSKY_PENDING_1HZ ;
      frSky.sendFrSky1Hz( );
	 }
	 else if ( pending & FRSKY_PENDING_05HZ )
	 {
			pending &= ~FRSKY_PENDING_05HZ ;
      frSky.sendFrSky05Hz( );
	 }
}

void processData()
{
	int16_t byte ;
	while ( ( byte = get_fifo128(&RxFifo) ) != -1 )
	{
		ctelemetry.parseMessage(byte) ;
	}

//	while (Serial.available() > 0)
//	{
//		char c = Serial.read() ;
   
//		int msg = ctelemetry.parseMessage(c);
//#ifdef DEBUG
//     if (msg >= 0)
//		 {
//	     ctelemetry.printMessage(debugSerial, dataProvider, msg);
//		 }
//#endif
//	}
}

#ifdef DEBUG
int freeRam () {
   extern int __heap_start, *__brkval; 
   int v; 
   return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}
#endif

// Switch all outputs ON
//void AllOn() {
//#ifdef DEBUG
//   debugSerial->print(".");
//#endif
////    digitalWrite(LEFT,  1);
////    digitalWrite(RIGHT, 1);
////    digitalWrite(FRONT, 1);
////    digitalWrite(REAR,  1);

////    digitalWrite(RED,   1);
////    digitalWrite(WHITE, 1);
////    digitalWrite(BLUE,  1);
////    digitalWrite(GREEN, 1);
//}

//// Switch all outputs OFF
//void AllOff() {
//#ifdef DEBUG
//   debugSerial->print(".");
//#endif
////    digitalWrite(LEFT,  0);
////    digitalWrite(RIGHT, 0);
////    digitalWrite(FRONT, 0);
////    digitalWrite(REAR,  0);

////    digitalWrite(RED,   0);
////    digitalWrite(WHITE, 0);
////    digitalWrite(BLUE,  0);
////    digitalWrite(GREEN, 0);
//}
// Updating base leds state
//void updateLights()
//{
//   int pin, index;
////   if ( dataProvider->motor_armed ) {
////      digitalWrite(WHITE, 1);
////   } else {
////      digitalWrite(WHITE, 0);
////   }
////   if ( (dataProvider->gpsStatus >= 3) && ( dataProvider->gpsHdop <= 200 ) ) {
////      digitalWrite(BLUE, 1);
////   } else {
////      digitalWrite(BLUE, 0);
////   }
////   if ( dataProvider->status_msg > 1) {
////      digitalWrite(RED, 1);
////   } else {
////      digitalWrite(RED, 0);
////   }
//  if (last_mode != dataProvider->apmMode)
//	{
//    pattern_index = 0;
//		last_mode = dataProvider->apmMode;
//	}
//   switch(last_mode) {
//   case 0: // STAB
//       index = 0;
//       break;
//   case 2: // AltHold
//       index = 1;
//       break;
//   case 6: // RTL
//       index = 2;
//       break;
//   default:
//       index = 3;
//       break;
//   }
////   pin = check_pattern((char*)pgm_read_word(&(left[index])));
////   digitalWrite(LEFT, pin);
//#ifdef DEBUG
//   debugSerial->print("LEFT LIGHT ");
//   debugSerial->print(pin);
//   debugSerial->print(" ");
//   debugSerial->println(pattern_index);
//#endif
////   pin = check_pattern((char*)pgm_read_word(&(right[index])));
////   digitalWrite(RIGHT, pin);
////   pin = check_pattern((char*)pgm_read_word(&(front[index])));
////   digitalWrite(FRONT, pin);
////   digitalWrite(GREEN, pin); // green led some as front light
////   pin = check_pattern((char*)pgm_read_word(&(rear[index])));
////   digitalWrite(REAR, pin);
////   pattern_index ++;
////   if (eol((char*)pgm_read_word(&(left[index])))) pattern_index = 0;
//}

//int check_pattern(char *string)
//{
//    int value = 0;
//    if ( pgm_read_byte( string + pattern_index ) == '1' )	{
//	   value = 1;
//	}
//	return value;
//}

//bool eol(char *string)
//{
//    bool value = false;
//    if ( pgm_read_byte( string + pattern_index ) == '\0' )	{
//	   value = true;
//	}
//	return value;
//}

// Code for timer2 for frsky hub timing
// interrupt every 4mS

//#define COUNT4MS		249

//volatile static byte T2count = 0 ;

//void initTimer2()
//{
//	TCCR2B = 0 ;	// off
//	OCR2A = COUNT4MS ;
//	ASSR &= ~0x20 ;
//	TCCR2A = 0x02 ;	// CTC mode
//	TIFR2 = 7 ;			// Clear all flags
//	TIMSK2 = 1 << OCIE2A ;
//	TCCR2B = 0x06 ;	// pre-scale 256
//	OCR2A = COUNT4MS ;
//	DDRB |= 0x30 ;
//}

//ISR(TIMER2_COMPA_vect)
//{
////	OCR2A = COUNT4MS ;
////	TCCR2B = 0x06 ;	// pre-scale 256
//	if ( ++T2count > 24 )
//	{
//		T2count = 0 ;
//		ActionFrskySend = 1 ;
//	}
//}

uint32_t micros()
{
	uint16_t elapsed ;
	uint8_t millisToAdd ;
	uint8_t oldSREG = SREG ;
	cli() ;
	uint16_t time = TCNT1 ;	// Read timer 1
	SREG = oldSREG ;

	elapsed = time - lastTimerValue ;
	elapsed += Correction ;


 #if F_CPU == 20000000L   // 20MHz clock 
   #error Unsupported clock speed
  #elif F_CPU == 16000000L  // 16MHz clock                                                  
        Correction = elapsed & 0x0F ;
        elapsed >>= 4 ;
  #elif F_CPU == 8000000L   // 8MHz clock
        Correction = elapsed & 0x07 ;
        elapsed >>= 3 ;
    #else
    #error Unsupported clock speed
  #endif

        //elapsed >>= 4 ;
	
	uint32_t ltime = TotalMicros ;
	ltime += elapsed ;
	cli() ;
	TotalMicros = ltime ;	// Done this way for RPM to work correctly
	lastTimerValue = time ;
	SREG = oldSREG ;	// Still valid from above
	
	elapsed += MillisPrecount;
	millisToAdd = 0 ;
	if ( elapsed  > 3999 )
	{
		millisToAdd = 4 ;
		elapsed -= 4000 ;
	}
	else if ( elapsed  > 2999 )
	{
		millisToAdd = 3 ;		
		elapsed -= 3000 ;
	}
	else if ( elapsed  > 1999 )
	{
		millisToAdd = 2 ;
		elapsed -= 2000 ;
	}
	else if ( elapsed  > 999 )
	{
		millisToAdd = 1 ;
		elapsed -= 1000 ;
	}
	TotalMillis += millisToAdd ;
	MillisPrecount = elapsed ;
	return TotalMicros ;
}

// return the number of milli second
NOINLINE uint32_t millis()
{
	micros() ;
	return TotalMillis ;
}

void delay(unsigned long ms)
{
	uint16_t start = (uint16_t)micros();
	uint16_t lms = ms ;

	while (lms > 0) {
		if (((uint16_t)micros() - start) >= 1000) {
			lms--;
			start += 1000;
		}
	}
}
 
// Delay for the given number of microseconds.  Assumes a 8 or 16 MHz clock. 
void delayMicroseconds(unsigned int us)
{
//	 calling avrlib's delay_us() function with low values (e.g. 1 or
//	 2 microseconds) gives delays longer than desired.
//	delay_us(us);
#if F_CPU >= 20000000L
//	 for the 20 MHz clock on rare Arduino boards

//	 for a one-microsecond delay, simply wait 2 cycle and return. The overhead
//	 of the function call yields a delay of exactly a one microsecond.
	__asm__ __volatile__ (
		"nop" "\n\t"
		"nop"); //just waiting 2 cycle
	if (--us == 0)
		return;

//	 the following loop takes a 1/5 of a microsecond (4 cycles)
//	 per iteration, so execute it five times for each microsecond of
//	 delay requested.
	us = (us<<2) + us; // x5 us

//	 account for the time taken in the preceeding commands.
	us -= 2;

#elif F_CPU >= 16000000L
//	 for the 16 MHz clock on most Arduino boards

//	 for a one-microsecond delay, simply return.  the overhead
//	 of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

//	 the following loop takes a quarter of a microsecond (4 cycles)
//	 per iteration, so execute it four times for each microsecond of
//	 delay requested.
	us <<= 2;

//	 account for the time taken in the preceeding commands.
	us -= 2;
#else
//	 for the 8 MHz internal clock on the ATmega168

//	 for a one- or two-microsecond delay, simply return.  the overhead of
//	 the function calls takes more than two microseconds.  can't just
//	 subtract two, since us is unsigned; we'd overflow.
	if (--us == 0)
		return;
	if (--us == 0)
		return;

//	 the following loop takes half of a microsecond (4 cycles)
//	 per iteration, so execute it twice for each microsecond of
//	 delay requested.
	us <<= 1;
    
//	 partially compensate for the time taken by the preceeding commands.
//	 we can't subtract any more than this or we'd overflow w/ small delays.
	us--;
#endif

//	 busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}


void init()
{
  // Timer1, set for millis
  TIMSK1 &= ~( 1<< OCIE1A ) ; // Disable interupt on timer 1 for compA
  TCCR1A = 0x00 ;    //Init.
  TCCR1B = 0xC1 ;    // I/p noise cancel, rising edge, Clock/1

	DDRD &= ~0x1C ; // Inputs
	PORTD |= 0x1C ; // With pullup

	DDRB |= 0x20 ;

}

