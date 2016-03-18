/*
	@author 	Nils Hцgberg
	@contact 	nils.hogberg@gmail.com
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

// Sport values:
// TEMP1 = APM mode
// ALTITUDE
// GPS COURSE
// VFAS = main battery voltage
// *BASEMODE
// TEMP2 == # satellites
// CURRENT
// RPM = engine speed
// *VCC
// *CPULOAD
// *HOME_DIR
// *WP_BEARING
// GPS LATITUDE
// GPS LONGITUDE
// FUEL
// GPS_ALTITUDE

#include "FrSky.h"
#include "Aserial.h"
#include "Mavlink.h"

extern Mavlink ctelemetry ;

extern uint8_t volatile sendStatus ;
extern uint8_t SendVfas ;
extern uint8_t SendCell ;

FrSky::FrSky()
{
	bufferLength = 0;
	sportIndex = 0 ;
	sportSubIndex = 0 ;
	sportSubIndexB = 0 ;
	sportCellIndex = 0 ;
	sportNext.valid = 0 ;
	sendStatus = SEND ;
}
FrSky::~FrSky(void)
{
}

extern void sendHubData( uint8_t *buffer, uint8_t length ) ;

static uint8_t SpTimer = 0 ;

void FrSky::queueFrSkySport()
{
  uint32_t value ;
	uint16_t id ;

	if ( sendStatus == SEND )
	{
		if ( sportNext.valid )
		{
  		setSportNewData( sportNext.id,  sportNext.value ) ; 
      sendStatus = LOADED ; 
			sportNext.valid = 0 ;
			if ( ++SpTimer > 82 )
			{
				SpTimer = 0 ;
				PORTB ^= 0x20 ;	// debug
			}
		}
	}

	if ( sportNext.valid == 0 )
	{
		switch ( sportIndex )
		{
			default :
			case 0 :
				value = ctelemetry.getTemp1() ;
				id = TEMP1 ;
			break ;
			case 1 :
				value = ctelemetry.getAltitude() * 100.0f ;
				id = ALT_FIRST_ID ;
			break ;
			case 2 :
				value = ctelemetry.getCourse() * 100.0f ;
				id = GPS_COURS_FIRST_ID ;
			break ;
			case 3 :
				if ( SendVfas )
				{
					value = ctelemetry.getMainBatteryVoltage() * 100.0f ;
					id = VFAS_FIRST_ID ;
				}
				else
				{
					id = 0xFFFF ;
				}
			break ;
			case 4 :
				value = ctelemetry.getBaseMode();
				id = BASEMODE ;
			break ;
			case 5 :
				value = ctelemetry.getTemp2() ;
				id = TEMP2 ;
			break ;
			case 6 :
				value = ctelemetry.getBatteryCurrent() ;
				id = CURR_FIRST_ID ;
			break ;
			case 7 :
				value = ctelemetry.getEngineSpeed() ;
				id = RPM_FIRST_ID ;
			break ;
			case 8 :
				value = ctelemetry.getVcc() ;
				id = VCC ;
			break ;
			case 9 :
				value = ctelemetry.getCpu_load();
				id = CPU_LOAD ;
			break ;
			case 10 :
				value = ctelemetry.getHome_dir() ;
				id = HOME_DIR ;
			break ;
			case 11 :
				value = ctelemetry.getWP_bearing() ;
				id = WP_BEARING ;
			break ;
			case 12 :
			{	
				long gpsLatitude = ctelemetry.getLatitude() ;
				value = 0 ;
				if (gpsLatitude < 0)
				{
					value |= 0x40000000 ;
					gpsLatitude = - gpsLatitude ;
				}
				uint32_t temp = gpsLatitude  * 6 ;
				temp /= 100 ;
				temp &= 0x3FFFFFFF ;
				value |= temp ;
				id = GPS_LONG_LATI_FIRST_ID ;
			}
			break ;
			case 13 :
			{	
				long gpsLongitude = ctelemetry.getLongitud() ;
				value = 0x80000000 ;
				if (gpsLongitude < 0)
				{
					value |= 0x40000000 ;
					gpsLongitude = - gpsLongitude ;
				}
				uint32_t temp = gpsLongitude * 6 ;
				temp /= 100 ;
				temp &= 0x3FFFFFFF ;
				value |= temp ;
				id = GPS_LONG_LATI_FIRST_ID ;
			}
			break ;
			case 14 :
				value = ctelemetry.getFuelLevel() ;
				id = FUEL ;
			break ;
			case 15 :
				value = ctelemetry.getGpsAltitude() ;
				id = GPS_ALT_FIRST_ID ;
			break ;
			case 16 :
				value = ctelemetry.getGpsHdop() ;
				id = GPS_HDOP ;
			break ;
			case 17 :
			{
				switch ( sportSubIndex )
				{
					case 0 :
						value = ctelemetry.getWP_dist() ;
						id = WP_DIST ;
					break ;
					case 1 :
						value = ctelemetry.getWP_num() ;
						id = WP_NUM ;
					break ;
					case 2 :
						value = ctelemetry.getWP_bearing() ;
						id = WP_BEARING ;
					break ;
					case 3 :
						value = ctelemetry.getHome_dist() ;
						id = HOME_DIST ;
					break ;
					case 4 :
						value = ctelemetry.getStatus_msg() ;
						id = STATUS_MSG ;
					break ;
					case 5 :
						value = ctelemetry.getHealth() ;
						id = HEALTH ;
					break ;
				}
				if ( ++sportSubIndex > 5 )
				{
					sportSubIndex = 0 ;
				}
			}			 
			break ;
			case 18 :
				switch ( sportSubIndexB )
				{
					case 0 :
						value = ctelemetry.getGpsGroundSpeed(); // / 1.84f;
						id = GPS_SPEED_FIRST_ID ;
					break ;
					case 1 :
						value = ctelemetry.getAirspeed();
						id = AIR_SPEED_FIRST_ID ;
					break ;
				}
				if ( ++sportSubIndexB > 1 )
				{
					sportSubIndexB = 0 ;
				}
			break ;

			case 19 :
			// Bits 0-3 = 0/2/4
			// Bits 4-7 = # cells
			// Bits 8-19 = Cell1/3/5
			// Bits 20-31 = Cell2/4/6
				if ( SendCell )
				{
					uint8_t nCells = ctelemetry.getNCell() ;
					uint32_t cval = ctelemetry.getCell() ;
					value = ( cval << 20 ) | ( cval << 8 ) | ( nCells << 4 ) ;
					id = CELLS_FIRST_ID ;
					value |= sportCellIndex ;
					if ( sportCellIndex > nCells )
					{
						id = 0xFFFF ;
					}
					sportCellIndex += 2 ;
					if ( sportCellIndex > 4 )
					{
						sportCellIndex = 0 ;
					}
				}
				else
				{
					id = 0xFFFF ;
				}
			break ;
		}
		if ( id != 0xFFFF )
		{
			sportNext.value = value ;
			sportNext.id = id ;
			sportNext.valid = 1 ;
		}
		
		if ( ++sportIndex > 19 )
		{
			sportIndex = 0 ;
		}
	}
}

// Send 5s frame
void FrSky::sendFrSky05Hz()
{
	bufferLength = 0 ;
	addBufferData(STATUS_MSG);
	addBufferData(HEALTH);
	FrskyBuffer[bufferLength++] = tail_value;
	sendHubData( FrskyBuffer, bufferLength ) ;
//	bufferLength = writeBuffer(bufferLength, serialPort);
}

// Send 1s frame
void FrSky::sendFrSky1Hz()
{
	bufferLength = 0 ;
	addBufferData(FUEL);
	addBufferData(LATITUDE);
	addBufferData(LONGITUDE);
	addBufferData(WP_DIST);
	addBufferData(WP_NUM);
	addBufferData(GPS_HDOP);
	addBufferData(GPSALT);
	addBufferData(GPSSPEED);
	addBufferData(HOME_DIST);
	addBufferData(AIRSPEED);
	FrskyBuffer[bufferLength++] = tail_value;
	sendHubData( FrskyBuffer, bufferLength ) ;
//	bufferLength = writeBuffer(bufferLength, serialPort);
}


// Send 200 ms frame
void FrSky::sendFrSky5Hz()
{
	bufferLength = 0 ;
	
	addBufferData(BASEMODE);
	addBufferData(TEMP1);
	addBufferData(TEMP2);
	addBufferData(ALTITUDE);
	addBufferData(COURSE);
	addBufferData(HOME_DIR);
	addBufferData(CURRENT);
	addBufferData(WP_BEARING);
	if ( SendVfas )
	{
		addBufferData(VOLTAGE);
	}
	addBufferData(CPU_LOAD);
	addBufferData(RPM);
	addBufferData(VCC);
	if ( SendCell )
	{
		addBufferData(INDVOLT);
	}
//	bufferLength += addBufferData(ACCX, dataProvider);
//	bufferLength += addBufferData(ACCY, dataProvider);
//	bufferLength += addBufferData(ACCZ, dataProvider);
	FrskyBuffer[bufferLength++] = tail_value;
	sendHubData( FrskyBuffer, bufferLength ) ;
//	bufferLength = writeBuffer(bufferLength, serialPort);
}

byte FrSky::lsByte(uint16_t value)
{
  return ((byte) ((value) & 0xff));
}

byte FrSky::msByte(uint16_t value)
{
  return ((byte) ((value) >> 8));
}

void FrSky::frskyPushByteValue(uint8_t value)
{
	uint8_t j ;
	j = 0 ;
  // byte stuff the only byte than might need it
  if (value == header_value)
	{
    j = 1 ;
    value = 0x3e;
  }
  else if (value == escape_value)
	{
    j = 1 ;
    value = 0x3d;
  }
	if ( j )
	{
		FrskyBuffer[bufferLength++] = escape_value;
	}
  FrskyBuffer[bufferLength++] = value;
}

void FrSky::frskyPushWordValue( uint16_t value )
{
	frskyPushByteValue(lsByte(value)) ;
	frskyPushByteValue(msByte(value)) ;
}

void FrSky::setBufferData(const char id, uint16_t value)
{
	FrskyBuffer[bufferLength++] = header_value ;
	FrskyBuffer[bufferLength++] = id ;
	frskyPushWordValue( value ) ;
}

void FrSky::addBufferData(const char id)
{

	switch(id)
	{
		case GPSALT :
		{
			//float gpsAltitude = par->termToDecimal(6) * 100.0f + 100.0f; // GPS Altitude i m, offset by 100
			float gpsAltitude = ctelemetry.getGpsAltitude();
			setBufferData( GPSALT, gpsAltitude ) ;
			unsigned int temp = (unsigned int)((gpsAltitude - (int)gpsAltitude) * 10000.0f);
			setBufferData( GPSALT + decimal, temp ) ;
			return ;
		}
		break;
		case TEMP1 :
		{
			// APM mode
			int temp1 = ctelemetry.getTemp1();
			setBufferData( TEMP1, temp1 ) ;
			return ;
		}
		break;
		case RPM :
		{
			// Throttle out
			int engineSpeed = ctelemetry.getEngineSpeed();
			setBufferData( RPM, engineSpeed ) ;
			return ;
		}
		break;
		case FUEL :
		{
			// Battery remaining in %
			int fuelLevel = ctelemetry.getFuelLevel();
			setBufferData( FUEL, fuelLevel ) ;
			return ;
		}
		break;
		case BASEMODE :
		{
			// APM base mode bitfield
			int base_mode = ctelemetry.getBaseMode();
			setBufferData( BASEMODE, base_mode ) ;
			return ;
		}
		break;
		case TEMP2 :
		{
			// GPS status, number of satelites in view
			int value = ctelemetry.getTemp2();
			setBufferData( TEMP2, value ) ;
			return ;
		}
		break;
		case INDVOLT : // Voltage, first 4 bits are cell number, rest 12 are voltage in 1/500v steps, scale 0-4.2v
		{
      static unsigned char curr_cell;
			if (curr_cell >= ctelemetry.getNCell() || curr_cell > 5) {
			    curr_cell = 0;
			}
			int value = ctelemetry.getCell();
			FrskyBuffer[bufferLength++] = header_value;
      FrskyBuffer[bufferLength++] = INDVOLT;
			frskyPushByteValue( (msByte(value) & 0x000F) + ( curr_cell * 16 ) ) ;
			frskyPushByteValue( lsByte(value) ) ;
			curr_cell ++;
      return ;
		}
		break;
		case ALTITUDE :
		{
			// Altitude in cm minus Home altitude in cm
			// Altitude in Taranis is offset by -10 m
			float altitude = ctelemetry.getAltitude();
			setBufferData( ALTITUDE, altitude ) ;
			int temp ;
			
//			if ( altitude >= 0 )
//			{
//		    temp = (unsigned int)((altitude - (unsigned int)altitude) * 100.0f);
//			}
//			else if ( (altitude <= 0) && (altitude > -1) )
//			{
//		    temp = 0; // FrSky bug for values from 0.0 to -0.99
//			}
//			else
//			{
		    temp = (unsigned int)((altitude - (int)altitude) * 100.0f);
//			}
			setBufferData( ALTIDEC, temp ) ;
			return ;
		}
		break;
		case GPSSPEED :
		{
			// GPS Ground speed in knots
			// Seems like there is an offset of 1.84 for some reason
			int gpsSpeed  = ctelemetry.getGpsGroundSpeed(); // / 1.84f;
			setBufferData( GPSSPEED, gpsSpeed ) ;
			unsigned int temp = (unsigned int)((gpsSpeed - (int)gpsSpeed) * 1000.0f);
			setBufferData( GPSSPEED + decimal, temp ) ;
			return ;
		}
		break;
		case LATITUDE :
		{
			//float gpsLatitude = gpsDdToDmsFormat(termToDecimal(4) / 10000000.0f);
			long latitude = ctelemetry.getLatitude() ;
			char northSouth = 'N' ;
			if ( latitude < 0 )
			{
				latitude = -latitude ;
				northSouth = 'S' ;
			}
			uint16_t degrees ;
			uint16_t minutes ;
			degrees = latitude / 10000000L ;
			latitude -= (uint32_t) degrees * 10000000L ;
			degrees *= 100 ;
			latitude *= 60 ;	// To minutes
			minutes = latitude / 10000000L ;
			latitude -= (uint32_t) minutes * 10000000L ;
			setBufferData( LATITUDE, degrees + minutes ) ;
			minutes = latitude / 1000L ;
			setBufferData( LATITUDE + decimal, minutes ) ;
			setBufferData( NORTHSOUTH, northSouth ) ;
			
//			float gpsLatitude = ctelemetry.getLatitude();


//			setBufferData( LATITUDE, gpsLatitude ) ;
      
//			unsigned int temp = (unsigned int)((gpsLatitude - (int)gpsLatitude) * 10000.0f);
    
//			setBufferData( LATITUDE + decimal, temp ) ;
      
//			char northSouth = (gpsLatitude < 0) ? 'S' : 'N';
//			setBufferData( NORTHSOUTH, northSouth ) ;
			return ;
		}
		break;
		case LONGITUDE :
		{
			//float gpsLongitude = gpsDdToDmsFormat(termToDecimal(5) / 10000000.0f);
			long longitude = ctelemetry.getLongitud() ;
			char eastWest = 'E' ;
			if ( longitude < 0 )
			{
				longitude = -longitude ;
				eastWest = 'W' ;
			}
			uint16_t degrees ;
			uint16_t minutes ;
			degrees = longitude / 10000000L ;
			longitude -= (uint32_t) degrees * 10000000L ;
			degrees *= 100 ;
			longitude *= 60 ;	// To minutes
			minutes = longitude / 10000000L ;
			longitude -= (uint32_t) minutes * 10000000L ;
			setBufferData( LONGITUDE, degrees + minutes ) ;
			minutes = longitude / 1000L ;
			setBufferData( LONGITUDE + decimal, minutes ) ;
			setBufferData( EASTWEST, eastWest ) ;
//			float gpsLongitude = ctelemetry.getLongitud();
//			setBufferData( LONGITUDE, gpsLongitude ) ;
      
//			unsigned int temp = (unsigned int)((gpsLongitude - (int)gpsLongitude) * 10000.0f);
    
//			setBufferData( LONGITUDE + decimal, temp ) ;
			
//			char eastWest = (gpsLongitude < 0 ) ? 'W' : 'E';
//			setBufferData( EASTWEST, eastWest ) ;
			return ;
		}
		break;
		case COURSE :
		{
			//float course = (par->termToDecimal(14) / 100.0f); // Course in 1/100 degree
			float course = ctelemetry.getCourse();
			setBufferData( COURSE, course ) ;
			unsigned int temp = (unsigned int)((course - (int)course) * 1000.0f);
			setBufferData( COURSE + decimal, temp ) ;
			return ;
		}
		break;
/*
		case DATE :
		{
			FrskyBuffer[bufferLength] = header_value;
			FrskyBuffer[bufferLength + 1] = DATE;
			FrskyBuffer[bufferLength + 2] = lsByte(ctelemetry.getDate());
			FrskyBuffer[bufferLength + 3] = msByte(ctelemetry.getDate());
			return 4;
			break;
		}
		case YEAR :
		{
			FrskyBuffer[bufferLength] = header_value;
			FrskyBuffer[bufferLength + 1] = DATE;
			FrskyBuffer[bufferLength + 2] = lsByte(ctelemetry.getYear());
			FrskyBuffer[bufferLength + 3] = msByte(ctelemetry.getYear());
			return 4;
			break;
		}
		case TIME :
		{
			FrskyBuffer[bufferLength] = header_value;
			FrskyBuffer[bufferLength + 1] = TIME;
			FrskyBuffer[bufferLength + 2] = lsByte(ctelemetry.getTime());
			FrskyBuffer[bufferLength + 3] = msByte(ctelemetry.getTime());
			return 4;
			break;
		}
		case SECOND :
		{
			return 0;
			break;
		}
		case ACCX :
		{
			//float accX = par->termToDecimal(17) / 100.0f;
			float accX = ctelemetry.getAccX();
			FrskyBuffer[bufferLength] = header_value;
			FrskyBuffer[bufferLength + 1] = ACCX;
			FrskyBuffer[bufferLength + 2] = lsByte((int)(accX*1000.0f));
			FrskyBuffer[bufferLength + 3] = msByte((int)(accX*1000.0f));
			return 4;
			break;
		}
		case ACCY :
		{
			//float accY = par->termToDecimal(18) / 100.0f;
			float accY =  ctelemetry.getAccY();
			FrskyBuffer[bufferLength] = header_value;
			FrskyBuffer[bufferLength + 1] = ACCY;
			FrskyBuffer[bufferLength + 2] = lsByte((int)(accY*1000.0f));
			FrskyBuffer[bufferLength + 3] = msByte((int)(accY*1000.0f));
			return 4;
			break;
		}
		case ACCZ :
		{
			//float accZ = par->termToDecimal(19) / 100.0f;
			float accZ = ctelemetry.getAccZ();
			FrskyBuffer[bufferLength] = header_value;
			FrskyBuffer[bufferLength + 1] = ACCZ;
			FrskyBuffer[bufferLength + 2] = lsByte((int)(accZ*1000.0f));
			FrskyBuffer[bufferLength + 3] = msByte((int)(accZ*1000.0f));
			return 4;
			break;
		}
*/
		case CURRENT :
		{
			//float current = par->termToDecimal(1) / 1000.0f; // 10.0f -> 1A
			int current = ctelemetry.getBatteryCurrent(); //  12 - 1.2A;
			setBufferData( CURRENT, current ) ;
			return ;
		}
		break;
		case VOLTAGE :
		{
			//float batteryVoltage = par->termToDecimal(0) * 0.5238f;
			//float batteryVoltage = 100.0f * 0.5238;
			float batteryVoltage = ctelemetry.getMainBatteryVoltage() * 0.5238f;
			setBufferData( VOLTAGE, batteryVoltage ) ;
      
			unsigned int temp = (unsigned int)((batteryVoltage - (int)batteryVoltage) * 10.0f);
    
			setBufferData( VOLTAGEDEC, temp ) ;

			return ;
		}
		break;
		case VCC :
		{
			unsigned int vcc = ctelemetry.getVcc();
			setBufferData( VCC, vcc ) ;
			return ;
		}
		break;
		case WP_DIST :
		{
			unsigned int wp_dist = ctelemetry.getWP_dist();
			setBufferData( WP_DIST, wp_dist ) ;
			return ;
		}
		break;
		case WP_NUM :
		{
			unsigned int wp_num = ctelemetry.getWP_num();
			setBufferData( WP_NUM, wp_num ) ;
			return ;
		}
		break;
		case WP_BEARING :
		{
			int wp_bearing = ctelemetry.getWP_bearing();
			setBufferData( WP_BEARING, wp_bearing ) ;
			return ;
		}
		break;
		case HEALTH :
		{
			unsigned int sensors_health = ctelemetry.getHealth();
			setBufferData( HEALTH, sensors_health ) ;
			return ;
		}
		break;
		case STATUS_MSG :
		{
			unsigned int msg_stat = ctelemetry.getStatus_msg();
			setBufferData( STATUS_MSG, msg_stat ) ;
			return ;
		}
		break;
		case HOME_DIR :
		{
			unsigned int home_direction = ctelemetry.getHome_dir();
			setBufferData( HOME_DIR, home_direction ) ;
			return ;
		}
		break;
		case HOME_DIST :
		{
			unsigned int home_distance = ctelemetry.getHome_dist();
			setBufferData( HOME_DIST, home_distance ) ;
			return ;
		}
		break;
		case AIRSPEED :
		{
			unsigned int airspeed = ctelemetry.getAirspeed() * 10.0f ;
			setBufferData( AIRSPEED, airspeed ) ;
			return ;
		}
		break;
		case CPU_LOAD :
		{
			unsigned int cpu_load = ctelemetry.getCpu_load();
			setBufferData( CPU_LOAD, cpu_load ) ;
			return ;
		}
		break;
		case GPS_HDOP :
		{
			unsigned int gpsHdop = ctelemetry.getGpsHdop();
			setBufferData( GPS_HDOP, gpsHdop ) ;
			return ;
		}
		break;
		default :
			return ;
  }
  return ;
}

//unsigned char FrSky::writeBuffer(const int length, SoftwareSerial* frSkySerial)
//{

//  int i = 0;
//  while(i < length)
//  {
//    // If a data value is equal to header (0x5E), tail (0x5E) or escape (0x5D) value exchange it.
//    // There is always a id and two bytes between header and tail therefor every 4th byte is a header and should not be checked
//    if ((i % 4))
//    {
//      switch (FrskyBuffer[i])
//      {
//        case header_value :
//        {
//          frSkySerial->write((byte)0x5D);
//          frSkySerial->write((byte)0x3E);
//          break;
//        }
//        case escape_value :
//        {
//		  frSkySerial->write((byte)0x5D);
//          frSkySerial->write((byte)0x3D);
//          break;
//        }
//        default :
//        {
//          frSkySerial->write((byte)FrskyBuffer[i]);
//        }
//      }
//    }
//    else
//    {
//      frSkySerial->write((byte)FrskyBuffer[i]);
//    }
//    micros() ;	// Keep this ticking
//    i++;
//  }
  
//  return 0;
//}

//void FrSky::printValues(SoftwareSerial* serialPort, IFrSkyDataProvider* dataProvider)
//{
//	serialPort->print("RAM: ");
//	serialPort->print(freeRam());
//	serialPort->print(" MSG: ");
//	serialPort->print(dataProvider->getStatus_msg());
//	serialPort->print(" WP bear: ");
//	serialPort->print(dataProvider->getWP_bearing());
////	serialPort->print(" Health: ");
////	serialPort->print(dataProvider->getHealth(), 10);
//	serialPort->print(" WP_num: ");
//	serialPort->print(dataProvider->getWP_num());
//	serialPort->print(" WP_dist: ");
//	serialPort->print(dataProvider->getWP_dist());
//	serialPort->print(" vcc: ");
//	serialPort->print(dataProvider->getVcc());
//	serialPort->print(" Voltage: ");
//	serialPort->print(dataProvider->getMainBatteryVoltage(), 2);
//	serialPort->print(" Current: ");
//	serialPort->print(dataProvider->getBatteryCurrent());
//	serialPort->print(" CPU: ");
//	serialPort->print(dataProvider->getCpu_load());
//	serialPort->print(" home dir: ");
//	serialPort->print(dataProvider->getHome_dir());
//	serialPort->print(" dist: ");
//	serialPort->print(dataProvider->getHome_dist());
////	serialPort->print(" Fuel: ");
////	serialPort->print(dataProvider->getFuelLevel());
////	serialPort->print(" Latitude: ");
////	serialPort->print(dataProvider->getLatitude(), 6);
////	serialPort->print(" Longitude: ");
////	serialPort->print(dataProvider->getLongitud(), 6);
////  serialPort->print(" GPS hdop: ");
////	serialPort->print(dataProvider->getGpsHdop());
////	serialPort->print(" GPS: ");
////	serialPort->print(dataProvider->getTemp2());
////	serialPort->print(" GPS speed: ");
////	serialPort->print(dataProvider->getGpsGroundSpeed(), 2);
//	serialPort->print(" Alt: ");
//	serialPort->print(dataProvider->getAltitude(), 1);
//	serialPort->print(" GPS alt: ");
//	serialPort->print(dataProvider->getGpsAltitude(), 2);
//	serialPort->print(" Mode: ");
//	serialPort->print(dataProvider->getTemp1());
//	serialPort->print(" Course: ");
//	serialPort->print(dataProvider->getCourse());
////	serialPort->print(" RPM: ");
////	serialPort->print(dataProvider->getEngineSpeed());
////	serialPort->print(" AccX: ");
////	serialPort->print(dataProvider->getAccX(), 2);
////	serialPort->print(" AccY: ");
////	serialPort->print(dataProvider->getAccY(), 2);
////	serialPort->print(" AccZ: ");
////	serialPort->print(dataProvider->getAccZ(), 2);
//	serialPort->println("");
//}
