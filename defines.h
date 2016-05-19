/*
	@author 	Nils Hцgberg
	@contact 	nils.hogberg@gmail.com
 	@coauthor(s):
	  Victor Brutskiy, 4refr0nt@gmail.com, er9x adaptation

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

#ifndef defines_h
#define defines_h

#ifndef byte
typedef unsigned char byte;
#endif

/* Mavlink data
   Changed
0x02 temp1      getTemp1      apmMode   (Flight mode)
0x04 FUEL       getFuelLevel  batteryRemaining
0x05 temp2      getTemp2      gpsStatus * 10 + numberOfSatelites
   New
0xA  HOME_DIR   getHome_dir   home_direction   
0xB  HOME_DIST  getHome_dist  home_distance    
0xC  CPU_LOAD   getCpu_load   cpu_load    
0xD  GPS_HDOP   getGpsHdop    gpsHdop    
0xE  WP_NUM         
0xF  WP_BEARING     
0x1D BASEMODE   getBaseMode   apmBaseMode
0x1E WP_DIST    getWP_dist    wp_dist
0x1F HEALTH     getHealth     sensors_health
0x20 STATUS_MSG getStatus_msg status_msg

enum gcs_severity {
    SEVERITY_LOW=1,
    SEVERITY_MEDIUM,
    SEVERITY_HIGH,
    SEVERITY_CRITICAL,
    SEVERITY_USER_RESPONSE
};

// https://github.com/diydrones/ardupilot/search?q=%23define+THISFIRMWARE
// Actual for #define THISFIRMWARE "ArduCopter V3.3-dev"
//
// https://github.com/diydrones/ardupilot/blob/master/ArduCopter/defines.h
//
*/

#define header_value   0x5e
#define tail_value     0x5e
#define escape_value   0x5d
#define decimal        0x8
      
#define GPSALT         0x1
#define TEMP1          0x2
#define RPM            0x3
#define FUEL           0x4
#define TEMP2          0x5
#define INDVOLT        0x6
#define VCC            0x7
//#define unused       0x8
#define GPSALTd        0x9
/* Mavlink data start */
#define HOME_DIR       0xA
#define HOME_DIST      0xB
#define CPU_LOAD       0xC
#define GPS_HDOP       0xD
#define WP_NUM         0xE
#define WP_BEARING     0xF
/* Mavlink data end */
#define ALTITUDE       0x10
#define GPSSPEED       0x11
#define LONGITUDE      0x12
#define LATITUDE       0x13
#define COURSE         0x14
#define DATE           0x15
#define YEAR           0x16
#define TIME           0x17
#define SECOND         0x18
//#define unused       0x19
//#define unused       0x1A
//#define unused       0x1B
//#define unused       0x1C
/* Mavlink data start */
#define BASEMODE       0x1D
#define WP_DIST        0x1E
#define HEALTH         0x1F
#define STATUS_MSG     0x20
/* Mavlink data end */
#define ALTIDEC        0x21
#define EASTWEST       0x22
#define NORTHSOUTH     0x23
#define ACCX           0x24
#define ACCY           0x25
#define ACCZ           0x26
#define VerticalSpeed  0x27
#define CURRENT        0x28 
//#define unused       0x29
//#define unused       0x2A
#define VSPD        0x30 // 0x30 -> 0x27

#define AIRSPEED	     0x38

#define VOLTAGE        0x3A 
#define VOLTAGEDEC     0x3B 

// MAVLink HeartBeat bits
#define MOTORS_ARMED 7  // 128
#define MSG_TIMER 30    // 30 sec

/*
0x01  GPS Altitude          M     int16
0x02  Temperature1          C     int16
0x03  Engine speed          RPM   uint16
0x04  Fuel Level            %     uint16
0x05  Temperature2          C     int16
0x06  Cell voltage          V     ?
0x07  
0x08
0x09  GPS Altitude decimal  .M    int16
0x0A
0x0B
0x0C
0x0D
0x0E
0x0F
0x10   Altitude		    M     int16
0x11   GPS Speed            Knots uint16
0x12   GPS Longitude        ddmm
0x13   GPS Latitude         ddmm
0x14   Course               Degree uint16
0x15   Date/Month
0x16   Year
0x17   Hour/Minute
0x18   Second
0x19   GPS Speed decimal    .knots
0x1A   GPS Longitude dec    .mmmm
0x1B   GPS Latitude  dec    .mmmm
0x1C   Course decimal       .degree
0x1D
0x1E
0x1F
0x20
0x21   Altitude decimal
0x22   E/W
0x23   N/S            
0x24   Acc-X          G   int16
0x25   Acc-Y          G   int16
0x26   Acc-Z          G   int16
0x27
0x28   Current        A   uint16
.
0x3A   Battery voltage    uint16
.
0x3B   Battery voltage decimal
*/


// SPort IDs
#define ALT_FIRST_ID            0x0100
#define ALT_LAST_ID             0x010f
#define VARIO_FIRST_ID          0x0110
#define VARIO_LAST_ID           0x011f
#define CURR_FIRST_ID           0x0200
#define CURR_LAST_ID            0x020f
#define VFAS_FIRST_ID           0x0210
#define VFAS_LAST_ID            0x021f
#define CELLS_FIRST_ID          0x0300
#define CELLS_SECOND_ID         0x0301
#define CELLS_THIRD_ID          0x0302
#define CELLS_LAST_ID           0x030f
#define T1_FIRST_ID             0x0400
#define T1_LAST_ID              0x040f
#define T2_FIRST_ID             0x0410
#define T2_LAST_ID              0x041f
#define RPM_FIRST_ID            0x0500
#define RPM_LAST_ID             0x050f
#define FUEL_FIRST_ID           0x0600
#define FUEL_LAST_ID            0x060f
#define ACCX_FIRST_ID           0x0700
#define ACCX_LAST_ID            0x070f
#define ACCY_FIRST_ID           0x0710
#define ACCY_LAST_ID            0x071f
#define ACCZ_FIRST_ID           0x0720
#define ACCZ_LAST_ID            0x072f
#define GPS_LONG_LATI_FIRST_ID  0x0800
#define GPS_LONG_LATI_LAST_ID   0x080f
#define GPS_ALT_FIRST_ID        0x0820
#define GPS_ALT_LAST_ID         0x082f
#define GPS_SPEED_FIRST_ID      0x0830
#define GPS_SPEED_LAST_ID       0x083f
#define GPS_COURS_FIRST_ID      0x0840
#define GPS_COURS_LAST_ID       0x084f
#define GPS_TIME_DATE_FIRST_ID  0x0850
#define GPS_TIME_DATE_LAST_ID   0x085f
#define A3_FIRST_ID             0x0900
#define A3_LAST_ID              0x090f
#define A4_FIRST_ID             0x0910
#define A4_LAST_ID              0x091f
#define AIR_SPEED_FIRST_ID      0x0a00
#define AIR_SPEED_LAST_ID       0x0a0f
#define RSSI_ID                 0xf101  // please do not use this code because it is already used by the receiver
#define ADC1_ID                 0xf102  // please do not use this code because it is already used by the receiver
#define ADC2_ID                 0xf103
#define BATT_ID                 0xf104
#define SWR_ID                  0xf105  // please do not use this code because it is already used by the receiver




#endif


