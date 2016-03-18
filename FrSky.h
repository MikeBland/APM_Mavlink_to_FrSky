/*
	@author 	Nils Hцgberg
	@contact 	nils.hogberg@gmail.com

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

#ifndef frsky_h
#define frsky_h

//#define lowByte(w) ((uint8_t) ((w) & 0xff))
//#define highByte(w) ((uint8_t) ((w) >> 8))

//#include "SimpleTelemetry.h"
//#include <SoftwareSerial.h>
#include <Arduino.h>
#include "defines.h"

struct t_sportQueueData
{
	uint16_t id ;
	uint32_t value ;
	uint8_t valid ;
} ;

class FrSky
{
	
public:
		FrSky();
		~FrSky(void);
		void sendFrSky5Hz();
		void sendFrSky1Hz();
		void sendFrSky05Hz();
		void queueFrSkySport();
//		void printValues(SoftwareSerial* serialPort, IFrSkyDataProvider* dataProvider);
private:
		void frskyPushByteValue(uint8_t value) ;
		void frskyPushWordValue( uint16_t value ) ;
		unsigned char		FrskyBuffer[120] ;
		uint8_t					bufferLength;
		struct t_sportQueueData sportNext ;
		uint8_t sportIndex ;
		uint8_t sportSubIndex ;
		uint8_t sportSubIndexB ;
		uint8_t sportCellIndex ;
		void		addBufferData(const char id);
		void setBufferData(const char id, uint16_t value) ;
//		unsigned char		writeBuffer(const int length, SoftwareSerial* frSkySerial);
		byte				lsByte(uint16_t value);
		byte				msByte(uint16_t value);
};
int freeRam ();
#endif

