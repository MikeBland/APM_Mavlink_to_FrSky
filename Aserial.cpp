/* ============================================================
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * ============================================================*/

// Based loosely on:
// Atmel AVR304: Half Duplex Interrupt Driven Software UART

// Author: Mike Blandford

#include "Aserial.h"

#define FORCE_INDIRECT(ptr) __asm__ __volatile__ ("" : "=e" (ptr) : "0" (ptr))


static volatile uint8_t state ;                  //!< Holds the state of the UART.
static volatile unsigned char SwUartTXData ;     //!< Data to be transmitted.
static volatile unsigned char SwUartTXBitCount ; //!< TX bit counter.
static volatile uint8_t SwUartRXData ;           //!< Storage for received bits.
static volatile uint8_t SwUartRXBitCount ;       //!< RX bit counter.
static volatile uint8_t TxCount ;


//For Frsky only
uint8_t ByteStuffByte = 0 ;

// initially only for Hub
//volatile uint8_t TxHubData[maxSizeBuffer] ;
volatile uint8_t *PtrTxHubData ;

volatile uint8_t TxMax ;
//struct t_hubData * volatile ThisHubData = 0 ;
uint8_t volatile HubData[maxSizeBuffer] ; 
//uint8_t volatile hubCurrentData ; //index of current data
uint8_t volatile HubMaxData ;   // max number of data prepared to be send

// initially only for Sport
uint8_t LastRx ;
uint8_t TxSportData[7] ;
uint16_t Crc ;

uint8_t volatile  sportData[7] ;
uint8_t volatile sportDataLock ;
uint8_t volatile sendStatus ;

///*! \brief  Timer0 interrupt service routine.
// *
// *  Timer0 will ensure that bits are written and
// *  read at the correct instants in time.
// *  The state variable will ensure context
// *  switching between transmit and recieve.
// *  If state should be something else, the
// *  variable is set to IDLE. IDLE is regarded
// *  as a safe state/mode.
// *
// */

ISR(TIMER1_COMPA_vect)
{
  if ( sportAvailable )
	{    // ++++++++ here only for SPORT protocol ++++++++++++++++++++++++++++++++++
	  switch (state)
		{
		// Transmit Byte.
		  case TRANSMIT :		// Output the TX buffer.
//#if DEBUG
//	PORTC |= 1 ;
//#endif
    		if( SwUartTXBitCount < 8 )
				{
    		  if( SwUartTXData & 0x01 )
					{           // If the LSB of the TX buffer is 1:
    		    CLEAR_TX_PIN() ;                    // Send a logic 1 on the TX_PIN.
    		  }
    		  else
					{                                // Otherwise:
    		    SET_TX_PIN() ;                      // Send a logic 0 on the TX_PIN.
    		  }
    	  	SwUartTXData = SwUartTXData >> 1 ;    // Bitshift the TX buffer and
    	  	SwUartTXBitCount += 1 ;               // increment TX bit counter.
    		}
    		else		//Send stop bit.
				{
    	  	CLEAR_TX_PIN();                         // Output a logic 1.
    	  	state = TRANSMIT_STOP_BIT;
    		}
		  	OCR1A += TICKS2WAITONESPORT ;  // Count one period into the future.
//#if DEBUG
//	PORTC &= ~1 ;
//#endif
	  	break ;

	  // Go to idle after stop bit was sent.
		  case TRANSMIT_STOP_BIT:
     		if ( ByteStuffByte || (++TxCount < 8 ) )		// Have we sent 8 bytes?
     		{
     			if ( ByteStuffByte )
          {
            SwUartTXData = ByteStuffByte ;
            ByteStuffByte = 0 ;
          }
          else
					{
						if ( TxCount < 7 )		// Data (or crc)?
						{
							SwUartTXData = TxSportData[TxCount] ;
						  Crc += SwUartTXData ; //0-1FF
						  Crc += Crc >> 8 ; //0-100
						  Crc &= 0x00ff ;
						}
						else
						{
							SwUartTXData = 0xFF-Crc ;
						}
            if ( ( SwUartTXData == 0x7E ) || ( SwUartTXData == 0x7D ) )
            {
            	ByteStuffByte = SwUartTXData ^ 0x20 ;
            	SwUartTXData = 0x7D ;					
            }
					}
		      SET_TX_PIN() ;                // Send a logic 0 on the TX_PIN.
			  	OCR1A += TICKS2WAITONESPORT ;      // Count one period into the future.
				  SwUartTXBitCount = 0 ;
				  state = TRANSMIT ;
				}
				else  // 8 bytes have been send
				{
					state = WAITING ;
          sendStatus = SEND ;
					OCR1A += ((uint16_t)3500*16) ;		// 3.5mS gap before listening
					TRXDDR &= ~( 1 << RX_PIN );   // PIN is input, tri-stated.
				  TRXPORT &= ~( 1 << RX_PIN );  // PIN is input, tri-stated.
//			PCIFR = (1<<PCIF2) ;					// clear pending interrupt
//			PCICR |= (1<<PCIE2) ;					// pin change interrupt enabled

//			struct t_sportData *pdata = ThisData ;
//			FORCE_INDIRECT( pdata ) ;

//			pdata->serialSent = 1 ;
//			DataSent = 1 ;
//			pdata = pdata->next ;
//			if ( pdata == 0 )		// Wrap at end
//			{
//				pdata = FirstData ;
//			}
//			ThisData = pdata ;
				}

		  break ;

  //Receive Byte.
			case RECEIVE :  // Start bit has been received and we will read bits of data      
		    OCR1A += TICKS2WAITONESPORT ;                // Count one period after the falling edge is trigged.
		    //Receiving, LSB first.
				{
					uint8_t data ;				// Use a temporary local storage
				 	data = SwUartRXBitCount ;
    			if( data < 8 )
					{
    			  SwUartRXBitCount = data + 1 ;
						data = SwUartRXData ;
						data >>= 1 ;		         // Shift due to receiving LSB first.
//#if DEBUG
//	PORTC &= ~1 ;
//#endif
    	  		if( GET_RX_PIN( ) == 0 )
						{
    	  		  data |= 0x80 ;          // If a logical 1 is read, let the data mirror this.
    	  		}
//#if DEBUG
//	PORTC |= 1 ;
//#endif
						SwUartRXData = data ;
		    	}
		    	else	//Done receiving
					{
//#if DEBUG
//	PORTC &= ~1 ;
//#endif
						if ( LastRx == 0x7E )
						{
PORTB ^= 0x10 ;	// debug
							if ( SwUartRXData == SENSOR_ID )
							{
//						// This is us
//						struct t_sportData *pdata = ThisData ;
//						FORCE_INDIRECT( pdata ) ;
//						if ( pdata )	// We have something to send
         			  if ( sendStatus == LOADED )
								{
         			    if ( sportDataLock == 0 )
									{
                    TxSportData[0] = sportData[0] ;
                    TxSportData[1] = sportData[1] ;
                    TxSportData[2] = sportData[2] ;
                    TxSportData[3] = sportData[3] ;
                    TxSportData[4] = sportData[4] ;
                    TxSportData[5] = sportData[5] ;
                    TxSportData[6] = sportData[6] ;
			            }
			            else
			            {	// Discard frame to be sent if data is locked
                    TxSportData[0] = 0 ;
                    TxSportData[1] = 0 ;
                    TxSportData[2] = 0 ;
                    TxSportData[3] = 0 ;
                    TxSportData[4] = 0 ;
                    TxSportData[5] = 0 ;
                    TxSportData[6] = 0 ;
                  }
									state = TxPENDING ;
                  sendStatus = SENDING ;
									OCR1A += (400*16-TICKS2WAITONESPORT) ;		// 400 uS gap before sending
								}
								else
								{
									// Wait for idle time
//									LastRx = 0 ;
									state = WAITING ;
									OCR1A += ((uint16_t)3500*16) ;		// 3.5mS gap before listening
								}
							}
              else // it is not the expected device ID
              {
                	state = WAITING ;
									OCR1A += ((uint16_t)3500*16) ;		// 3.5mS gap before listening
//                	OCR1A += DELAY_3500 ;		// 3.5mS gap before listening
//                			    SportSync = 1 ;
              }
						}
						else
						{
		    			DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	  		  		state = IDLE ;                // Go back to idle.
							PCIFR = (1<<PCIF2) ;					// clear pending interrupt
							PCICR |= (1<<PCIE2) ;					// pin change interrupt enabled
						}
						LastRx = SwUartRXData ;
          } // End receiving  1 bit or 1 byte (8 bits)
				}
		  break ;
  
			case TxPENDING :
//#if DEBUG
//	PORTC |= 1 ;
//#endif
				TRXDDR |= ( 1 << RX_PIN ) ;       // PIN is output
	      SET_TX_PIN() ;                    // Send a logic 0 on the TX_PIN.
		  	OCR1A = TCNT1 + TICKS2WAITONESPORT ;   // Count one period into the future.
			  SwUartTXBitCount = 0 ;
				Crc = SwUartTXData = TxSportData[0] ;
				TxCount = 0 ;
			  state = TRANSMIT ;
//#if DEBUG
//	PORTC &= ~1 ;
//#endif
	  	break ;

			case WAITING :
		   	DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
		    state = IDLE ;                // Go back to idle.
				PCIFR = (1<<PCIF2) ;					// clear pending interrupt
				PCICR |= (1<<PCIE2) ;					// pin change interrupt enabled
			break ;

  // Unknown state.
		  default:        
	  	  state = IDLE;                           // Error, should not occur. Going to a safe state.
	  } // End CASE
 	} // end sportAvailable == true
	else
	{ //  ++++++++ here only for Hub protocol ++++++++++++++++++++++++++++++++++
	  switch (state)
		{
		  // Transmit Byte.
		  case TRANSMIT :		// Output the TX buffer.************ on envoie des bits de data
#if DEBUGASERIAL
	        PORTC |= 1 ;
#endif
				if( SwUartTXBitCount < 8 )
    		{
      	  if( SwUartTXData & 0x01 )
    			{           // If the LSB of the TX buffer is 1:
      	  	CLEAR_TX_PIN() ;                    // Send a logic 1 on the TX_PIN.
      	  }
      	  else
    			{                                // Otherwise:
      	  	SET_TX_PIN() ;                      // Send a logic 0 on the TX_PIN.
      	  }
      	  SwUartTXData = SwUartTXData >> 1 ;    // Bitshift the TX buffer and
      	  SwUartTXBitCount += 1 ;               // increment TX bit counter.
      	}
      	else		//Send stop bit.
    		{
      	  CLEAR_TX_PIN();                         // Output a logic 1.
      	  state = TRANSMIT_STOP_BIT;
      	      //ENABLE_TIMER0_INT() ;	                  // Allow this in now.
      	}
    		OCR1A += TICKS2WAITONEHUB ;  // Count one period into the future.
  
#if DEBUGASERIAL
	      PORTC &= ~1 ;
#endif
      break ;

  // Go to idle after stop bit was sent.
      case TRANSMIT_STOP_BIT: //************************************* We send a stop bit
				if ( ++TxCount < TxMax) 		// Have we sent all bytes?
				{
    		  SwUartTXData = PtrTxHubData[TxCount] ;			        
    		  SET_TX_PIN() ;                        // Send a logic 0 on the TX_PIN.
	  		  OCR1A = TCNT1 + TICKS2WAITONEHUB ;       // Count one period into the future.
				  SwUartTXBitCount = 0 ;
				  state = TRANSMIT ;
				  //DISABLE_TIMER0_INT() ;		// For the byte duration
				}
        else  // all bytes have been send
				{
				  TxCount = 0 ;
    		  TxMax = 0 ;
    		  state = WAITING ;
    		  //sendStatus = SEND ;
    		  OCR1A += DELAY_100 ;	// 100uS gap
				}

      break ;
               
			case WAITING :
	      DISABLE_TIMER_INTERRUPT() ;		// Stop the timer interrupts.
	      state = IDLE ;                           // Go back to idle.
	    break ;

  // Unknown state.
		  default:        
	      state = IDLE;                               // Error, should not occur. Going to a safe state.
	  } // End CASE

 	} // end "else sport" = end Hub
} // End of ISR


//____________________Here the code for SPORT interface only ++++++++++++++++++++++++++++++++++++++++++

//brief  Function to initialize the UART for Sport protocol
//  This function will set up pins to transmit and receive on. Control of Timer0 and External interrupt 0.
void initSportUart(  )           //*************** initialise UART pour SPORT
{
//    FirstData = ThisSportData = pdata ;
    
    //PORT
  TRXDDR &= ~( 1 << PIN_SERIALTX ) ;       // PIN is input.
  TRXPORT &= ~( 1 << PIN_SERIALTX ) ;      // PIN is tri-stated.

  // External interrupt
  
#if PIN_SERIALTX == 5
  PCMSK2 |= 0x20 ;			// IO5 (PD5) on Arduini mini
#elif PIN_SERIALTX == 2
  PCMSK2 |= 0x04 ;                    // IO2 (PD2) on Arduini mini
#else
  #error "This PIN is not supported"
#endif

  PCIFR = (1<<PCIF2) ;	// clear pending interrupt
  PCICR |= (1<<PCIE2) ;	// pin change interrupt enabled

    // Internal State Variable
  state = IDLE ;

#if DEBUGASERIAL
  DDRC = 0x03 ;		// PC0,1 as o/p debug
  PORTC = 0 ;
#endif

}


void setSportNewData( uint16_t id, uint32_t value )
{
  sportDataLock = 1 ;
  sportData[0] = 0x10 ;
  sportData[1] = id ; // low byte
  sportData[2] = id >> 8 ; // hight byte
  sportData[3] = value ;
  sportData[4] = value >> 8 ;
  sportData[5] = value >> 16 ;
  sportData[6] = value >> 24 ;
  sportDataLock = 0 ;
}


ISR(PCINT2_vect)
{
	if ( TRXPIN & ( 1 << PIN_SERIALTX ) )			// Pin is high = start bit (inverted)
	{
		DISABLE_PIN_CHANGE_INTERRUPT()  ;			// pin change interrupt disabled
		state = RECEIVE ;                 // Change state
    DISABLE_TIMER_INTERRUPT() ;       // Disable timer to change its registers.
   	OCR1A = TCNT1 + TICKS2WAITONE_HALFSPORT - INTERRUPT_EXEC_CYCL - INTERRUPT_EARLY_BIAS ; // Count one and a half period into the future.
#if DEBUGASERIAL
	        PORTC |= 1 ;
#endif
    SwUartRXBitCount = 0 ;            // Clear received bit counter.
    CLEAR_TIMER_INTERRUPT() ;         // Clear interrupt bits
    ENABLE_TIMER_INTERRUPT() ;        // Enable timer1 interrupt on again
	}
//#ifdef PPM_INTERRUPT
//  if ( EIFR & PPM_INT_BIT)  ppmInterrupted = 1 ;
//#endif

}  // end ISR pin change


//____________________Here the code for HUB interface only ++++++++++++++++++++++++++++++++++++++++++
//brief  Function to initialize the UART for Sport protocol
//  This function will set up pins to transmit and receive on. Control of Timer0 and External interrupt 0.
void initHubUart( )
{
//  ThisHubData = pdata ;
  TRXPORT &= ~( 1 << PIN_SERIALTX ) ;      // PIN is low
  TRXDDR |= ( 1 << PIN_SERIALTX ) ;        // PIN is output.
	
  //Internal State Variable
  state = IDLE ;
  TxMax = 0 ;
  TxCount = 0 ;

#if DEBUGASERIAL
  DDRC = 0x03 ;		// PC0,1 as o/p debug
  PORTC = 0 ;
#endif
}


void sendHubData( uint8_t *buffer, uint8_t length )
{
  if ( (TxCount == 0) && (length > 0) )
	{
		TxMax = length  ;
		PtrTxHubData = buffer ;
    startHubTransmit() ;
	}
	
}

//void setHubNewData(  )
//{
//  if ( (TxCount == 0) && (HubMaxData > 0) )
//	{
//    for (uint8_t cntNewData = 0 ; cntNewData < HubMaxData ; cntNewData++)
//		{
//      TxHubData[cntNewData] = HubData[cntNewData] ;
//    }
//    TxMax = HubMaxData  ;
//		PtrTxHubData = TxHubData ;
//    startHubTransmit() ;
//  }    
//}

void startHubTransmit()
{
	if ( state != IDLE )
	{
		return ;
	}
	cli() ;
  SET_TX_PIN() ;                    // Send a logic 0 on the TX_PIN.
	OCR1A = TCNT1 + TICKS2WAITONEHUB ;   // Count one period into the future.
 	CLEAR_TIMER_INTERRUPT() ;         // Clear interrupt bits
	sei() ;
  SwUartTXBitCount = 0 ;
	SwUartTXData = PtrTxHubData[0] ;
	//TxNotEmpty = 0 ;
  state = TRANSMIT ;
 	ENABLE_TIMER_INTERRUPT() ;        // Enable timer1 interrupt on again
#if DEBUGASERIAL
	PORTC &= ~1 ;
#endif

}
// end of function that are hub specific





