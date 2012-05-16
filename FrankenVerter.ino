/*
	FRANKENVERTER 1.0														10/1/2011
	
		An ARINC 429 to NMEA 0183 and SL30 (Garmin Nav Radio) converter.  The Frankenverter 
		is built as an Arduino Mega Sheild and takes ARINC 429 line level inputs and transcodes
		the GPS specific input into NMEA 0183 (for enroute and terminal) and SL30 format
		sentences (for approach/LPV).
		
		Also, input from Dynon EFIS telemetry is converted to Shadin FADC (air-data) format
		for use by Garmin 4/500 series Navigation GPS units.
		
		The PCB is available at: http://batchpcb.com/index.php/Products/75449
		This Arduino shield is compatible with Arduino Mega 2560 (R2) and probably 1280 as well

	TO DO:
		- Need to publish a BOM for the PCB!  For now:
			DEI1016B ARINC line-driver chip
			77956 CONN,DSUB,.318"RT,15P-F (jameco.com)
			71618 SOCKET,PLCC,44 PIN,SOLDERTAIL (jameco.com)
			1 MHZ 7mmx5mm SMD 5V Crystal Oscillator (http://www.ebay.com/itm/10-PCS-1MHz-5-7-SMT-1-MHz-1-000MHz-Crystal-Oscillator-/120874623633)
			MAX233ACWP RS232 Line Driver (digikey.com)
			(sorry about the smd parts…)
			
		- Major cleanup of vars and defines
		- Better comments for key code features:  explain why the insane Serial output settings...
		- Factor code to reduce footprint
		- Re-establish some sanity of the Serial telemetry output controlled by DEBUG_SERIAL
		
	Written by Neil Cormia
		Based on work of Mark Ewert (http://www.instructables.com/id/Interfacing-Electronic-Circuits-to-Arduinos/)
  
*/
 
#include <avr/io.h>
#include "arduino_io.h"    // these macros map arduino pins
#include "a429.h"    
#include "a429_device.h"    
#include "delay.h" 
#include "avrio.h"         // these macros do direct port io   

//////////////////////////////////////////////////////////////////////////////////////////////////
// Now define several utility functions for accurate AVR timing delay
#define DelayNanoseconds(__SN) _delay_cycles( (double)(F_CPU)*((double)__SN)/1.0e9 + 0.5 ) // Hans Heinrichs delay cycle routine
#define fastWrite(pin, pinval) avrio_WritePin(pin, pinval)

#define NMEA_DELAY_MS	100		// 10 updates per second

int inByte = 0;         		// incoming Serial byte
int Test_Output = 0;
int	Test_Rate = 100;
int Test_VNAV = 0;

byte controlword1;
byte controlword2;

long r1_activity_off_time = 0;
long r2_activity_off_time = 0;

boolean r1_activity = false;
boolean r2_activity = false;

/*************************************************************************************************
  busoutput
  This module sets the bus pins to outputs
**************************************************************************************************/
void busoutput()
{
  int i;
  //for (i=22;i<=37;i++)
  //  pinMode(i,OUTPUT);
  DDRA = 0xFF;
  DDRC = 0xFF;
}

/*************************************************************************************************
  busHiZ
  This module sets the bus pins to inputs.
**************************************************************************************************/
void busHiZ()
{
  int i;
  PORTA = 0x00;
  PORTC = 0x00;
  DDRA = 0x00;
  DDRC = 0x00;
//  for (i=22;i<=37;i++)
//    pinMode(i,INPUT);
}

/*************************************************************************************************
DEBUG to control Serial Output
**************************************************************************************************/
#ifdef DEBUG_SERIAL
	#define SerialPrint(A)		Serial.print A
	#define SerialPrintln(A)	Serial.println A
#else
	#define SerialPrint(A)		(void)0
	#define SerialPrintln(A)	(void)0
#endif

/*************************************************************************************************
  setup
  Standard setup module for arduinos
**************************************************************************************************/
void setup()
{
  // start Serial port:
  Serial.begin(115200);		// DEBUG output and control input (USB)
  
  Serial1.begin(9600);		// Input from GPSMAP (optional) | Shadin FADC Output to GPS 400W
  Serial2.begin(115200);	// Logging Input from Dynon EFIS
  Serial3.begin(9600);		// NMEA Output to Dynon 100/180
  
  pinMode(a429DR1_pin, INPUT);
  pinMode(a429DR2_pin, INPUT);
  pinMode(a429TXR_pin, INPUT);
  pinMode(a429SEL_pin, OUTPUT);
  pinMode(a429OE1_pin, OUTPUT);
  pinMode(a429OE2_pin, OUTPUT);
  pinMode(a429LD1_pin, OUTPUT);
  pinMode(a429LD2_pin, OUTPUT);
  pinMode(a429ENTX_pin, OUTPUT);
  pinMode(a429LDCW_pin, OUTPUT);
  pinMode(a429DBCEN_pin, OUTPUT); 
  pinMode(a429MR_pin, OUTPUT);

  // Init Annunciators
  #define Annunciate_TRM	20
  #define Annunciate_APR	21
  pinMode(Annunciate_TRM, OUTPUT);
  pinMode(Annunciate_APR, OUTPUT);

    
  /* initialize the ARINC uart */
  digitalWrite(a429MR_pin,HIGH);
  digitalWrite(a429SEL_pin,HIGH);
  digitalWrite(a429OE1_pin,HIGH);
  digitalWrite(a429OE2_pin,HIGH);
  digitalWrite(a429LD1_pin,HIGH);
  digitalWrite(a429LD2_pin,HIGH);
  digitalWrite(a429ENTX_pin,HIGH);
  digitalWrite(a429LDCW_pin,HIGH);
  digitalWrite(a429DBCEN_pin,HIGH);

  controlword1  = NOTSLFTST;
  controlword1 |= PAREN;  // enable normal operations and parity generator
  controlword2  = TXLO;
  controlword2 |= RXLO;        // high speed  TX, low speed RX
  SerialPrint(("setup: "));
  SerialPrintln((controlword2, HEX));
  

  busoutput();
}

/*************************************************************************************************
  resetARINC
  This module resets the ARINC 429 transceiver and set it up per controlword1 and 2 settings.
**************************************************************************************************/
void resetARINC()
{
  byte temp;
  // lower then raise MR
  delayMicroseconds(100);  
  fastWrite(a429MR_pin,LOW);
  delayMicroseconds(10);  
  fastWrite(a429MR_pin,HIGH);
  
  // Now load control word
  fastWrite(a429LDCW_pin,LOW);
  PORTA = controlword1;
  PORTC = controlword2;
  delayMicroseconds(1);  // let them settle a bit
  fastWrite(a429LDCW_pin,HIGH);
  delayMicroseconds(100); 
  
  SerialPrint(("resetARINC: "));
  SerialPrintln((controlword2, HEX));

}

/*************************************************************************************************
  txARINC
  This module transmits a test ARINC word for testing
**************************************************************************************************/
void txARINC()
{
  fastWrite(a429ENTX_pin,LOW);
  delayMicroseconds(1);  
  fastWrite(a429LD1_pin,LOW);
  PORTA = 0x12;
  PORTC = 0x34;
  delayMicroseconds(1);  
  fastWrite(a429LD1_pin,HIGH);
  delayMicroseconds(1);  // let them settle a bit
  fastWrite(a429LD2_pin,LOW);
  PORTA = 0x56;
  PORTC = 0x78;
  delayMicroseconds(1);  
  fastWrite(a429LD2_pin,HIGH);
  delayMicroseconds(1);  
  fastWrite(a429ENTX_pin,HIGH);
}



/*************************************************************************************************
  rxARINC
  This module receives the ARINC words from either reciever which has data waiting.
**************************************************************************************************/
void rxARINC()
{
  int rx1, rx2;
  unsigned short int b1,b2,b3,b4;
  
  
  // Read from the ARINC 1 input [DSUB-15 connector: pin 11 (ARINC-A) and pin 4 (ARINC-B)]
  //
  rx1 = digitalRead(a429DR1_pin);
  if (rx1 ==0) // rx1 has data
  {
    r1_activity_off_time = millis() + 50;  // add 50 millis
    r1_activity = true;
    //fastWrite(rx1activity,LOW); 
    busHiZ();
    fastWrite(a429SEL_pin,LOW);
    DelayNanoseconds(Tssel);  
    fastWrite(a429OE1_pin,LOW);
    DelayNanoseconds(Tpwoe);  
    b1 = PINA;
    b2 = PINC;
    fastWrite(a429OE1_pin,HIGH);
    DelayNanoseconds(Thsel);  
    fastWrite(a429SEL_pin,HIGH);
    DelayNanoseconds(Toeoe);  
    fastWrite(a429OE1_pin,LOW);
    DelayNanoseconds(Tdoedr);  
    b3 = PINA;
    b4 = PINC;
    fastWrite(a429OE1_pin,HIGH);
    busoutput();
    
    parse_ARINC(b1,b2,b3,b4);
  }

  /*
  // Read from the ARINC 2 input [DSUB-15 connector: pin 10 (ARINC-A) and pin 3 (ARINC-B)]
  //
  rx2 = digitalRead(a429DR2_pin);
  if (rx2==0) // rx2 has data
  {
    r2_activity_off_time = millis() + 50;  // add 50 millis
    r2_activity = true;
    fastWrite(rx2activity,LOW); 
    busHiZ();
    fastWrite(a429SEL_pin,LOW);
    DelayNanoseconds(Tssel);  
    fastWrite(a429OE2_pin,LOW);
    DelayNanoseconds(Tpwoe);  
    b1 = PINA;
    b2 = PINC;
    fastWrite(a429OE2_pin,HIGH);
    DelayNanoseconds(Thsel);  
    fastWrite(a429SEL_pin,HIGH);
    DelayNanoseconds(Toeoe);  
    fastWrite(a429OE2_pin,LOW);
    DelayNanoseconds(Tdoedr);  
    b3 = PINA;
    b4 = PINC;
    fastWrite(a429OE2_pin,HIGH);
    busoutput();
    SerialPrint(("r");
    SerialPrint((b4, HEX);
    SerialPrint((b3, HEX);
    SerialPrint((b2, HEX);
    SerialPrintln((b1, HEX);
  }
  */

}
 


// -------------------------------------------------------------------------------------------
// RECORD THE ARINC LABELS ARRIVING TO ELIMINATE DUPLICATE REPORTING
word  label_table[300];
byte  b2_table[300];

// Flight Plan Info Struct
struct FPStruct {
	int		stationtype;
	float	wptlatitude;
	int		wptlatSN;
	float	wptlongitude;
	int		wptlongWE;
	char	msgbuf[8];
} FlitePlan[21];

// Store persistant data values received in the ARINC stream
int		MagVar_WE;
float	MagVar = 0;
char	MsgBuf[16];
char	GMTBuf[8];
char	DateBuf[8];
int		StationType = 0;
int		OnRoute = 0;
int		WaypointNumber = 0;
int		NumWaypoints = 0;
int		FromWPT = 0;
int		ToWPT = 0;

int		GS_Timeout = 0;
float	VerticalDeviation = 0;
int		VerticalDev_UD;
float	CrossTrackDist;
int		CrossTrack_RL;

float	SelectedCourse;
float	ETDest;
float	DistanceToDest;
float	DistanceToGo;
float	TrackAngle;
float	GroundSpeed;
float	WPT_Bearing;
float	DesiredTrack;

float	WPT_Latitude;
int		WPT_Lat_SN;
float	WPT_Longitude;
int		WPT_Long_WE;
float	PP_Latitude;
int		PP_Lat_SN;
float	PP_Longitude;
int		PP_Long_WE;

long NMEA_delay = 0;
int	 SL30_Count = 0;


// -------------	GlideSlope and Localizer flags and values	-------------
// These are timeout values that are set to (???) MSec when valid and count down to zero if
// no updates occur or if an invalid data flag is sensed. NOT USED!!!!
#define	GS_VALID_TIME	1000
#define	LOC_VALID_TIME	1000

#define BC_Enable	0x01
#define LOC_Detect	0x02
#define FROM_Flag	0x04
#define TO_Flag		0x08
#define GSI_Super	0x10
#define GSI_Valid	0x20
#define NAV_Super	0x40
#define NAV_Valid	0x80

#define GPS_ENROUTE		 0
#define GPS_TERMINAL	20
#define GPS_APPROACH	21

int		GPS_Range = GPS_ENROUTE;


int   	GS_Valid 	= 0;
int   	LOC_Valid	= 0;

float	VertScaleFactor = 0;
float	LatScaleFactor = 0;

int		VerticalDeflection, HorizontalDeflection;

// -------------------------------------------------------------------------------------------
// NMEA OUTPUT FUNCTIONS
//
//	- - - - - - - - - - - - - - - - - - - - - - - - -
//	PMMRRV25 - Label for Decoded Station ID (Apollo SL30 IM p71)
//
void outputStation(void)
{
char buf[64];
byte i,chksum, flags = 0;
int crs;

	buf[0] = 0;
	
	 	// NMEA Label
		strcpy(buf, "$PMRRV25V");
		
		strcat(buf, FlitePlan[ToWPT].msgbuf);
		
		// Append spaces to pad ID to 5 characters
		for (i=0; i<(5-strlen(FlitePlan[ToWPT].msgbuf)); i++)
			strcat(buf," ");
	
	    // Calculate one byte checksum
		appendSL30checksum(buf);
	
		// Send to RS-232 Out #2 on FrankenVerter
	   	Serial3.write((byte *)buf, strlen(buf));
		//SerialPrint((buf));
}

//	PMMRRV22 - Label for LPV/LNAV+V OBS (Apollo SL30 IM p71)
//
void outputOBS(void)
{
char buf[64];
byte i,chksum, flags = 0;
int crs;

	buf[0] = 0;
	
	 	// NMEA Label
		strcpy(buf, "$PMRRV22V"); // 8 Bytes
		
		// Waypoint to Bearing
		crs = int(round(SelectedCourse));
		pad_int_to_string_noLR(&(buf[strlen(buf)]), crs, 3);
	
	    // Calculate one byte checksum
		appendSL30checksum(buf);
	
		// Send to RS-232 Out #2 on FrankenVerter
	   	Serial3.write((byte *)buf, strlen(buf));
		//SerialPrint((buf));
}

//	Output all the other SL30 sentences in case they're required by the Dynon D180
//
void outputActiveVOR(void)
{
char buf[64];
	
 	// Radial from Active VOR
	strcpy(buf, "$PMRRV2300");
	appendSL30checksum(buf);
   	Serial3.write((byte *)buf, strlen(buf));
	//SerialPrint((buf));
}

//	Output all the other SL30 sentences in case they're required by the Dynon D180
//
void outputSL30Misc(void)
{
char buf[64];
	
 	// Radial from Standby VOR
	strcpy(buf, "$PMRRV2400");
	appendSL30checksum(buf);
   	Serial3.write((byte *)buf, strlen(buf));
	//Serial.print(buf);

 	// NAV Receiver Status
	strcpy(buf, "$PMRRV28E4?PN");
	appendSL30checksum(buf);
   	Serial3.write((byte *)buf, strlen(buf));
	//Serial.print(buf);

 	// COMM Tranceiver Status
	strcpy(buf, "$PMRRV35P4IPR0");
	appendSL30checksum(buf);
   	Serial3.write((byte *)buf, strlen(buf));
	//Serial.print(buf);

}

//	PMMRRV21 - Label for CDI/GSI (Apollo SL30 IM p71)
//
void outputHSI(void)
{
char buf[64];
byte i,chksum, flags = 0;
float dist;

	buf[0] = 0;
	
// NMEA Label for CDI/GSI (Apollo SL30 IM p71)
	strcpy(buf, "$PMRRV21"); // 8 Bytes
	
// Convert lateral and vertical deflection into encoded hex
	if (LatScaleFactor > 0) {
		
		// Calculate how far off center (VSF is full deflection distance)
		dist = CrossTrackDist / LatScaleFactor * 100;
		if (dist > 100) dist = 100;
		if (CrossTrack_RL == 0) dist = dist * -1;
		
		// Encode LOC deflection into (HEX) as two byte ascii chars
		HorizontalDeflection = dist;

		buf[8] = ((HorizontalDeflection & 0xF0) >> 4) + 0x30;
		buf[9] = (HorizontalDeflection & 0x0F) + 0x30;
		flags |= NAV_Valid + NAV_Super;
	}
	else {
		buf[8] = buf[9] = '0';
	}
	
	if (VertScaleFactor > 0) {
		
		// Calculate how far off center (VSF is full deflection distance)
		dist = VerticalDeviation / VertScaleFactor * 100;
		
		// Peg at +/- 100
		if (dist > 100) dist = 100;
		
		// Encode up.down into numbers
		if (VerticalDev_UD) {
			dist *= -1;
		}
		
		// Encode GS deflection into (HEX) as two byte ascii chars
		VerticalDeflection = dist;

		buf[10] = ((VerticalDeflection & 0xF0) >> 4) + 0x30;
		buf[11] = (VerticalDeflection & 0x0F) + 0x30;
		flags  |= GSI_Valid + GSI_Super;
	} 
	else {
		buf[10] = buf[11] = '0';
	}
	
	// Encode the flags
	buf[12] = ((flags & 0xF0) >> 4) + 0x30;
	buf[13] = (flags & 0x0F) + 0x30;
	buf[14] = 0;

    // Calculate one byte checksum
	appendSL30checksum(buf);
  	
	// Send to RS-232 Out #2 on FrankenVerter
   	Serial3.write((byte *)buf, strlen(buf));
	//Serial.print(buf);
}

/*
['$GPRMB', 'A', '2.14', 'L', '', 'PEBGE', '3809.0221', 'N', '12226.4855', 'W', '3.147', '23.0', '107.7', 'V', 'D*25\r\n']

          RMB          Recommended minimum navigation information
           A            Data status A = OK, V = Void (warning)
           0.66,L       Cross-track error (nautical miles, 9.99 max),
                                steer Left to correct (or R = right)
           003          Origin waypoint ID
           004          Destination waypoint ID
           4917.24,N    Destination waypoint latitude 49 deg. 17.24 min. N
           12309.57,W   Destination waypoint longitude 123 deg. 09.57 min. W
           001.3        Range to destination, nautical miles (999.9 max)
           052.5        True bearing to destination
           000.5        Velocity towards destination, knots
           V            Arrival alarm  A = arrived, V = not arrived
           *20          checksum
*/
//	- - - - - - - - - - - - - - - - - - - - - - - - -
//	GPRMB - Recommended Minimum Sentence B
//
void outputRMB(void)
{
char buf[128], chkbuf[8];
byte i,chksum, flags = 0;
struct FPStruct* fp;
float dist;

	buf[0] = 0;
	
	// NMEA Label for RMB
	strcpy(buf, "$GPRMB,");
		
	// Valid flag
	strcat(buf, "A,");
		
	// XTE - Calculate how far off center (LSF is full deflection distance)
	// normalize to no more than full deflection. Dynon max scale is 5 NM.
	//
	dist = CrossTrackDist / LatScaleFactor * 5;
	if (dist > 5) dist = 5;
	
	appendFloat(buf, dist, 2);
	strcat(buf, CrossTrack_RL ? ",R,":",L,");
	
	// Origin and Dest Waypoint IDs
	fp = &(FlitePlan[ToWPT]);
	strcat(buf, FlitePlan[FromWPT].msgbuf);
	strcat(buf, ",");
	strcat(buf, fp->msgbuf);
	strcat(buf, ",");
	
	// Dest Lat/Long
	appendFloatDegrees(buf, fp->wptlatitude);
	strcat(buf, fp->wptlatSN ? ",S,":",N,");
	appendFloatDegrees(buf, fp->wptlongitude);
	strcat(buf, fp->wptlongWE ? ",W,":",E,");
	

	appendFloat(buf, DistanceToGo, 1);
	strcat(buf, ",");
	
	appendFloat(buf, WPT_Bearing, 1);
	strcat(buf, ",");
	
	appendFloat(buf, GroundSpeed, 1);
	strcat(buf, ",");
	
	// If more than 300 ft away we haven't arrived
	if (DistanceToDest < 0.05)
		strcat(buf, "A,");
	else
		strcat(buf, "V,");

	// D for differential, A for autonomous
	strcat(buf, "D");

    // Calculate and attach a HEX checksum and line termination
	appendNMEAchecksum(buf);

	// Send to RS-232 Out #2 on FrankenVerter
    Serial3.write((byte *)buf, strlen(buf));
	//Serial.println(buf);
}



/*
['$GPRMC', '194223', 'A', '3806.1250', 'N', '12228.0474', 'W', '107.6', '19.5', '290112', '14.2', 'E', 'D*07\r\n']

     RMC          Recommended Minimum sentence C
     123519       Fix taken at 12:35:19 UTC
     A            Status A=active or V=Void.
     4807.038,N   Latitude 48 deg 07.038' N
     01131.000,E  Longitude 11 deg 31.000' E
     022.4        Speed over the ground in knots
     084.4        Track angle in degrees True
     230394       Date - 23rd of March 1994
     003.1,W      Magnetic Variation
     *6A          The checksum data, always begins with *
*/
//	- - - - - - - - - - - - - - - - - - - - - - - - -
//	GPRMC - Recommended Minimum Sentence C
//
void outputRMC(void)
{
char buf[128];
byte flags = 0;

	buf[0] = 0;
	
	// NMEA Label for RMC
	strcpy(buf, "$GPRMC,");
		
	// GMT Time + Valid flag
	strcat(buf, GMTBuf);
	strcat(buf, ",A,");
		
	// Lat/Long
	appendFloatDegrees(buf, PP_Latitude);
	strcat(buf, PP_Lat_SN ? ",S,":",N,");
	appendFloatDegrees(buf, PP_Longitude);
	strcat(buf, PP_Long_WE ? ",W,":",E,");
	
	appendFloat(buf, GroundSpeed, 1);
	strcat(buf, ",");
	appendFloat(buf, TrackAngle, 1);
	strcat(buf, ",");
	strcat(buf, DateBuf);
	strcat(buf, ",");
	
	appendFloat(buf, MagVar, 1);
	strcat(buf, MagVar_WE ? ",W,":",E,");
	
	// D for differential, A for autonomous
	strcat(buf, "D");

    // Calculate and attach a HEX checksum and line termination
	appendNMEAchecksum(buf);
	
	// Send to RS-232 Out #2 on FrankenVerter
    Serial3.write((byte *)buf, strlen(buf));
	//Serial.println(buf);
}

/*
  $GPBOD,045.,T,023.,M,DEST,START*01

where:
        BOD          Bearing - origin to destination waypoint
        045.,T       bearing 045 True from "START" to "DEST"
        023.,M       bearing 023 Magnetic from "START" to "DEST"
        DEST         destination waypoint ID
        START        origin waypoint ID
        *01          checksum

$GPGGA,000059,3116.6441,N,09319.0049,W,1,11,0.8,4074.0,M,000.0,M,,*5E
$GPBOD,289.2,T,286.2,M,K25TE,HOME*1A

*/
//	- - - - - - - - - - - - - - - - - - - - - - - - -
//	GPBOD - Bearing Origin to Destination
//
void outputBOD(void)
{
char buf[128];
byte flags = 0;

	buf[0] = 0;
	
	// NMEA Label for RMC
	strcpy(buf, "$GPBOD,");
		
	// DTK (true)
	appendFloat(buf, DesiredTrack, 1);
	strcat(buf, ",T,");
		
	// DTK (mag)
	appendFloat(buf, SelectedCourse, 1);
	strcat(buf, ",M,");	// blank for now
		
	// Origin and Dest Waypoint IDs
	strcat(buf, FlitePlan[ToWPT].msgbuf);
	strcat(buf, ",");
	strcat(buf, FlitePlan[FromWPT].msgbuf);
		
    // Calculate and attach a HEX checksum and line termination
	appendNMEAchecksum(buf);
	
    Serial3.write((byte *)buf, strlen(buf));
	//Serial.println(buf);
}


/*
['$PGRMH',	'A',	'185',	'-19',					'-477', 		'', 		'2400', '65.9', '309.9*3E\r\n']

			VALID	VSI	 	VNAV Err(19 ft below)	FPM to Target	FPM to WPT	HAT		DTK		CRS of next leg
*/
//	- - - - - - - - - - - - - - - - - - - - - - - - -
//	PGRMH - Garmin Proprietary VNAV Sentence
//
void outputPGRMH(int valid)
{
char buf[128];
byte flags = 0;
float dist;

	buf[0] = 0;
	
	// NMEA Label for RMC
	strcpy(buf, "$PGRMH,");
	
	// Valid Sentence
	if (valid) {
		// A for valid, VSI Missing - Don't have it in ARINC spec
		strcat(buf, "A,,");
	
		// Calculate how far off center (VSF is full deflection distance)
		dist = VerticalDeviation / VertScaleFactor * 999;
		
		// Peg at +/- 100
		if (dist > 999) dist = 999;
		
		// Encode up.down into numbers
		if (VerticalDev_UD) {
			dist *= -1;
		}
		
		// Deviation above(+) or below (-) G/S
		appendFloat(buf, dist, 1);
		strcat(buf, ",");
			
		// FPM to Target and Waypoint Missing!!! No HAT either.  Don't have it in ARINC???
		strcat(buf, ",,2000,");
	
		// DTK (true)
		appendFloat(buf, DesiredTrack, 1);
		strcat(buf, ",");
	}
	else {
		strcat(buf, "V,,,,,,,");
	}
		
    // Calculate and attach a HEX checksum and line termination
	appendNMEAchecksum(buf);
	
    Serial3.write((byte *)buf, strlen(buf));
	//Serial.println(buf);
}


//	- - - - - - - - - - - - - - - - - - - - - - - - -
void appendNMEAchecksum(char *buf)
{
char chkbuf[8];
int i, total;
unsigned int chksum;
	
    // Calculate one byte checksum
    total = strlen(buf);
    
    for (i=1,chksum=0; i<total; i++) {
      chksum ^= byte(buf[i]);
    }

    // Pad checksum string and add \r\n
	strcat(buf, "*");

	// Write string for hex checksum
	chksum = chksum & 0xFF;
	itoa(chksum, chkbuf, 16);
	strupr(chkbuf);
	
	if (chksum < 16)
		strcat(buf, "0");
	
	strcat(buf, chkbuf);

	// Add the line termination
   	strcat(buf, "\r\n"); 
}

//	- - - - - - - - - - - - - - - - - - - - - - - - -
void appendSL30checksum(char *buf)
{
char chkbuf[8];
int i, total;
byte chksum;
	
	//Serial.print("SL30 checksum: ");
    // Calculate one byte checksum
    total = strlen(buf);
    
    for (i=6,chksum=0; i<total; i++) {
      chksum += buf[i];
    }

	// Write string for ascii encoded hex checksum
	chksum = chksum & 0xFF;
	
	//Serial.println(chksum,HEX);

	chkbuf[0] = ((chksum & 0xF0) >> 4) + 0x30;
	chkbuf[1] = (chksum & 0x0F) + 0x30;
	chkbuf[2] = 0;
	
	
	strcat(buf, chkbuf);

	// Add the line termination
   	strcat(buf, "\r\n"); 
}

//	- - - - - - - - - - - - - - - - - - - - - - - - -
void appendFloatDegrees(char *buf, float fvar)
{
	int	i,idec, ivar = int(fvar);
	float dec;
	
	//String-ify the Int portion
	itoa(ivar, &(buf[strlen(buf)]),10);
	
	// Pull the minutes into an integer and string-ify
	dec = abs(fvar - ivar);
	idec = dec = dec * 60;
	if (idec < 10) strcat(buf, "0");		
	itoa(idec, &(buf[strlen(buf)]),10);
	strcat(buf, ".");
	
	// Isolate the remaining decimal info, pull digits into the int side, and string-ify
	idec = int(dec*60);
	itoa(idec, &(buf[strlen(buf)]),10);
}

//	- - - - - - - - - - - - - - - - - - - - - - - - -
void appendFloat(char *buf, float fvar, int precision)
{
	int	i, idec, ivar = int(fvar);
	float pwrten, dec;
	
	if (precision == 0) {
		ivar = int(round(fvar));
		itoa(ivar, &(buf[strlen(buf)]),10);
	}
	else {
		//String-ify the Int portion
		itoa(ivar, &(buf[strlen(buf)]),10);
		strcat(buf, ".");
		
		// Pull the decimal portion into an integer and string-ify
		dec = abs(fvar - ivar);
		for (i=0; i<precision; i++) {
			dec *= 10;
			itoa(int(dec), &(buf[strlen(buf)]),10);
			dec -= int(dec);
		}
	}
}

/*************************************************************************************************/
int availableMemory() 
{
 int size = 8192;
 byte *buf;
 while ((buf = (byte *) malloc(--size)) == NULL);
 free(buf);
 return size;
}

/*************************************************************************************************
  parse_ARINC
  This module receives the ARINC words from either reciever which has data waiting.
  
  This parsing routine ASSUMES YOU ARE USING A DEI-1016(B) ARINC transceiver which
  manipulates the incoming ARINC words to make parsing easier. The LABEL bits are reversed so they
  can be directly read by the parsing routine, and the data payload is shifted up to reduce additional
  processing for calculations.  Bit 29 of the original ARINC sentence becomes the MSB of b4 (and word2).
  
  	b1		contains the LABEL which is read as an octal value
  	b4..b2	contain the remainder of the ARINC word 
  	word2	a local var which contains the assembled data payload
  	
**************************************************************************************************/
void parse_ARINC(unsigned short int b1,unsigned short int b2,unsigned short int b3,unsigned short int b4)
{
  word word2;
  unsigned long	word3;
  int twos, i;
  float angle, dist, decim;
  int tho,hun,ten,one,tenth;


    // Assemble a (mostly) complete ARINC word out of the byte data to simplify parsing
    word2 = ((b4 << 8) + b3);
    word3 = word2 + b2<<16;
    
    // Don't re-parse the same ARINC sentence if it has not changed - just exit
   //if ((label_table[b1] == word2) && (b2_table[b1] == b2) && (b1 != 0305) && (b1 != 0117))
   //   return;
   // else {
        label_table[b1] = word2;
        b2_table[b1]	= b2;
	//}

    switch (b1) {
   
    case 01:  
    	break;  
      SerialPrint(("Distance "));

      tho = (word2 & 0xE000)>>13;
      hun = (word2 & 0x1E00)>>9;
      ten = (word2 & 0x01E0)>>5;
      one = (word2 & 0x001E)>>1;
      tenth = ((word2 &0x1) << 3) + ((b2 & 0xE0) >> 5);
      decim = tenth;
      dist = tho*1000+hun*100+ten*10+one+(decim/10);
     
      SerialPrintln((dist));
    break;  
    
    case 02:  
     break;  
     SerialPrint(("Time To Go "));

      hun = (word2 & 0xE000)>>13;
      ten = (word2 & 0x1E00)>>9;
      one = (word2 & 0x01E0)>>5;
      tenth = (word2 & 0x001E)>>1;
      decim = tenth;
      dist = hun*100+ten*10+one+(decim/10);
     
      SerialPrintln((dist));
    break;  
    
    case 012:
      SerialPrint(("Ground Speed (kts) "));

      tho = (word2 & 0xE000)>>13;
      hun = (word2 & 0x1E00)>>9;
      ten = (word2 & 0x01E0)>>5;
      one = (word2 & 0x001E)>>1;
      tenth = ((word2 &0x1) << 3) + ((b2 & 0xE0) >> 5);
      decim = tenth;
      GroundSpeed = tho*1000+hun*100+ten*10+one+(decim/10);
     
      SerialPrintln((GroundSpeed));
    break;  
      
    
    case 074:
		SerialPrint(("Data Record Header (# Recs) "));
		
		for (i=0; i<= 20; i++) 
			FlitePlan[i].stationtype = 0;
		
		NumWaypoints = ((word2 &0x3) << 6) + ((b2 & 0xFC) >> 2);
		SerialPrint(("Num Waypoints: "));
		SerialPrintln((NumWaypoints));
	break;  
      
    
   case 075:
      SerialPrintln(("Active Waypoint From/To "));

      FromWPT	= (word2 & 0x7800)>>11 + ((word2 & 0x0078)>3 * 10);
      ToWPT		= (word2 & 0x0780)>>7 + ((((word2 &0x7) << 1) + ((b2 & 0x80) >> 7)) * 10);
    break;  
      
    
    case 0100:
      SerialPrint(("Selected Course "));  
      SelectedCourse = angle_bnr_calc(b4, b3, b2, 3, 90); 
      SerialPrintln((SelectedCourse));
    break;
    
    case 0113: /* MSG CHECKSUM */ 
    	//if (NumWaypoints == WaypointNumber)
	  	
    	if (OnRoute) {
	  		// Save off the Waypoint info for Flight Plan data ONLY (onRoute)
	  		if (WaypointNumber <= 20) {
		  		FlitePlan[WaypointNumber].stationtype	= StationType;
		  		FlitePlan[WaypointNumber].wptlatitude	= WPT_Latitude;
		  		FlitePlan[WaypointNumber].wptlatSN		= WPT_Lat_SN;
		  		FlitePlan[WaypointNumber].wptlongitude	= WPT_Longitude;
		  		FlitePlan[WaypointNumber].wptlongWE		= WPT_Long_WE;
		  		strcpy(FlitePlan[WaypointNumber].msgbuf, MsgBuf);
		  	}
      
     
			SerialPrint(("From WPT: "));	SerialPrint((FromWPT));
			SerialPrint((" To WPT: "));	SerialPrintln((ToWPT));
    		for (i=1; i<= 20; i++) 
    			if (FlitePlan[i].stationtype > 0) {
		  		SerialPrint((i));
				SerialPrint((", "));
				SerialPrint((FlitePlan[i].stationtype));
				SerialPrint((": "));
				SerialPrint((FlitePlan[i].msgbuf));
				SerialPrint((", "));
		  		SerialPrint((FlitePlan[i].wptlatitude, 6));
		  		SerialPrint((FlitePlan[i].wptlatSN ? 'S':'N'));
				SerialPrint((", "));
		  		SerialPrint((FlitePlan[i].wptlongitude, 6));
		  		SerialPrintln((FlitePlan[i].wptlongWE ? 'W':'E'));
	  		}
		}
	break;	

    case 0114:
      SerialPrint(("Desired Track "));     
   
      DesiredTrack = angle_bnr_calc(b4, b3, b2, 3, 90); 
      SerialPrintln((DesiredTrack));
    break;
    
    case 0115:
      SerialPrint(("Waypoint Bearing "));     
         
      WPT_Bearing = angle_bnr_calc(b4, b3, b2, 3, 90); 
      SerialPrintln((WPT_Bearing));
    break;
    
    case 0116:
      SerialPrint(("Cross Track Distance (NM) "));  
      
	  CrossTrackDist = distance_bnr_calc(b4, b3, b2, 0, 64);
	  
	  // True = Fly Right
	  CrossTrack_RL  = b4 & 0x80;
      
      SerialPrintln((CrossTrackDist));
    break;
    
    case 0117:
      Serial.print(("Vertical Deviation (ft) "));  

	  // True = Fly Up
	  VerticalDev_UD  = b4 & 0x80;
      VerticalDeviation = distance_bnr_calc(b4, b3, b2, 0, 8192);
      
      // Restart the GS timeout.  If this expires it clears VerticalDeviation and the GS display.
      GS_Timeout = 20;
            
      Serial.println((VerticalDeviation));
    break;
    
    case 0121:
      break;	// Disable - it spits out way too much data...
      SerialPrint(("Horizontal Command (deg) "));  
      if (b4 & 0x80)
        SerialPrint(("Fly Left "));  
      else
        SerialPrint(("Fly Right "));  

      angle = distance_bnr_calc(b4, b3, b2, 1, 90); 
      SerialPrintln((angle));
    break;

    case 0122:
    break;  
      SerialPrint(("Vertical Command (deg) "));  
      if (b4 & 0x80)
        SerialPrint(("Fly Down "));  
      else
        SerialPrint(("Fly Up "));  

	// TODO:  Need to parse out the flag info and stop using b2
      angle = distance_bnr_calc(b4, b3, b2, 3, 90); 
      SerialPrintln((angle));
    break;

    case 0125: /* GREENWICH MEAN TIME */ 
    
    	// Parse out the time and make it ASCII HH:MM:SS
    	GMTBuf[0] = ((b4 & 0xE0) >> 5) + 0x30;
    	GMTBuf[1] = ((b4 & 0x1E) >> 1) + 0x30;
    	GMTBuf[2] = ((word2 & 0x01E0) >> 5) + 0x30;
    	GMTBuf[3] = ((word2 & 0x001E) >> 1) + 0x30;

		// Seconds are in 10ths of a minute
    	one = (((b3 & 0x01) << 3) + ((b2 & 0xE0) >> 5)) * 6;
     	GMTBuf[4] = one/10 + 0x30;
  		GMTBuf[5] = one%10 + 0x30;
    	GMTBuf[6] = 0;
 		
      	//SerialPrint("availableMemory: ");  
 		//SerialPrintln(availableMemory());
 		
    break;
    
    case 0147: /* MAGNETIC VARIATION */
      SerialPrint(("Magnetic Variation "));        
        
      MagVar = angle_bnr_calc(b4, b3, b2, 3, 90); 
      MagVar_WE = b4 & 0x80;
      
      SerialPrintln((MagVar));
     break;     
     
    case 0251:
      SerialPrint(("Distance to go (nm) "));  
      DistanceToGo = distance_bnr_calc(b4, b3, b2, 0, 2048); 
      SerialPrintln((angle));
    break;
    
    case 0252:
	    break;  
      SerialPrint(("Time to go (minutes)"));  
      angle = distance_bnr_calc(b4, b3, b2, 6, 256); 
      SerialPrintln((angle));
    break;
    
    case 0260:
      SerialPrint(("Date: "));

      DateBuf[0] = ((b4 & 0xC0)>>6) + 0x30;
      DateBuf[1] = ((b4 & 0x3C)>>2) + 0x30;

      DateBuf[2] = ((b4 & 0x02)>>1) + 0x30;
      DateBuf[3] = ((word2 & 0x01E0)>>5) + 0x30;

      DateBuf[4] = ((b3 & 0x1E)>>1) + 0x30;
      DateBuf[5] = ((b3 & 0x01) + ((b2 & 0xE0) >> 5)) + 0x30;
      DateBuf[6] = 0;
	break;

	case 0261:
      SerialPrint(("GPS Discrete Word 1 "));

      tho = (word2 & 0x0038)>>3;      
     
      switch(tho) {
      	case 0:	SerialPrintln(("No Approach Type Selected ")); break;
      	case 1:	SerialPrintln(("LNAV ")); break;
      	case 2:	SerialPrintln(("LNAV/VNAV ")); break;
      	case 4:	SerialPrintln(("LP ")); break;
      	case 5:	SerialPrintln(("LPV ")); break;
      }
    break;  

    case 0275: /* LRN STATUS WORD */ break;
      
    case 0300: /* DECLINATION???? TODO */ break;     
    
    case 0303: /* MESSAGE */ 
      //SerialPrint(("Station Type: ");

      StationType	 = ((word2 & 0x3) << 1) + ((b2 & 0x80) >> 7);
      WaypointNumber = (word2 & 0x03F8) >> 3;
	  OnRoute		 = (b3 & 0x4) ? 0 : 1; 
	  
    break;
    
    case 0301: /* MESSAGE CHARS*/
    case 0302:
		SerialPrintln(("Msg 301 or 302"));
    break;   
    
    case 0304:
    	MsgBuf[2] = ((b4 & 0xFE) >> 1);
    	MsgBuf[1] = ((word2 &0x01FC) >> 2);
    	MsgBuf[0] = ((word2 &0x0003) << 5) + ((b2 & 0xF8) >> 3);

		SerialPrint(("Msg 304: "));
		SerialPrintln((MsgBuf));
    break;
    
    case 0305:
    	MsgBuf[5] = ((b4 & 0xFE) >> 1);
    	MsgBuf[4] = ((word2 &0x01FC) >> 2);
    	MsgBuf[3] = ((word2 &0x0003) << 5) + ((b2 & 0xF8) >> 3);
    	
    	// Terminate string and trim trailing spaces
    	MsgBuf[6] = 0;
    	for (i=3;i<6;i++)
    		if (MsgBuf[i] == ' ')
    			MsgBuf[i] = 0;
    		
		SerialPrint(("Msg 305: "));
		SerialPrintln((MsgBuf));
    break;   
    
    case 0306:
    case 0307:
    case 0310:
    case 0311:
      angle = distance_bnr_calc(b4, b3, b2, -5, 90); 
      
      if (b1 == 0306) {
      	WPT_Latitude = angle;
      	WPT_Lat_SN = b4 & 0x80;
      } else if (b1 == 0310) {
       	PP_Latitude = angle;
      	PP_Lat_SN = b4 & 0x80;
      } else if (b1 == 0307) {
      	WPT_Longitude = angle;
      	WPT_Long_WE = b4 & 0x80;
      } else if (b1 == 0311) {
       	PP_Longitude = angle;
      	PP_Long_WE = b4 & 0x80;
      }

      //if (b4 & 0x80)
      //  SerialPrint(("West ");  
      //else
      //  SerialPrint(("East ");  
    break;

    case 0312:
	    break;  
      SerialPrint(("Ground Speed "));  
      angle = distance_bnr_calc(b4, b3, b2, 0, 2048); 
      SerialPrintln((angle));
    break;
    
    case 0313:
      SerialPrint(("Track Angle "));  
      TrackAngle = angle_bnr_calc(b4, b3, b2, 3, 90); 
      SerialPrintln((TrackAngle));
    break;
    
    case 0314:
	    break;  
      SerialPrint(("True Heading "));  
      angle = angle_bnr_calc(b4, b3, b2, 0, 90); 
      SerialPrintln((angle));
    break;

    case 0315: 
	    break;  
      SerialPrint(("Wind Speed "));  
      angle = distance_bnr_calc(b4, b3, b2, 7, 128); 
      SerialPrintln((angle));
    break;
    
    case 0316:
 	    break;  
     SerialPrint(("Wind Angle "));  
      angle = angle_bnr_calc(b4, b3, b2, 3, 90); 
      SerialPrintln((angle));
    break;

    case 0320:
	    break;  
      SerialPrint(("Mag Heading "));  
      angle = angle_bnr_calc(b4, b3, b2, 0, 90); 
      SerialPrintln((angle));
    break;

    case 0321:
	    break;  
      SerialPrint(("Drift Angle "));  
      angle = angle_bnr_calc(b4, b3, b2, 3, 90); 
      SerialPrintln((angle));
    break;
    
    case 0326:
      SerialPrint(("Lateral Scale Factor (NM)"));  
      angle = angle_bnr_calc(b4, b3, b2, 0, 64);
      LatScaleFactor = angle;
      
		// Set GPS Mode Range
		if (LatScaleFactor <= 0.3) {
			GPS_Range = GPS_APPROACH;
			digitalWrite(GPS_APPROACH,HIGH);
			digitalWrite(GPS_TERMINAL,LOW);
		}
		else if (LatScaleFactor <= 1.0) {
			GPS_Range = GPS_TERMINAL;
			digitalWrite(GPS_APPROACH,LOW);
			digitalWrite(GPS_TERMINAL,HIGH);
		}
		else {
			GPS_Range = GPS_ENROUTE;
			digitalWrite(GPS_APPROACH,LOW);
			digitalWrite(GPS_TERMINAL,LOW);
		}
      
      SerialPrintln((angle));
    break;
    
    case 0327:
      Serial.print(("Vertical Scale Factor (feet)"));  
      VertScaleFactor = angle_bnr_calc(b4, b3, b2, 0, 1024); 
      
      Serial.println((VertScaleFactor));
    break;
    
    case 0351:
      SerialPrint(("Dist to Dest(nm) "));  
      SerialPrint((b2,HEX));  
      SerialPrint((" "));  
      DistanceToDest = distance_bnr_calc(b4, b3, b2, -3, 16384); 
      SerialPrintln((DistanceToDest));
    break;
    
    case 0352:
      SerialPrint(("ETD (min) "));  
      ETDest = distance_bnr_calc(b4, b3, b2, 3, 2048); 
      SerialPrintln((ETDest));
    break;
    
    case 0371:
		break;
      SerialPrint(("EQUIPMENT IDENT CODE "));

      tho = (word2 & 0x07E0)>>5;      
     
      switch (tho) {
      	case 3:		SerialPrintln(("BENDIX ")); break;
      	case 13:	SerialPrintln(("KING ")); break;
      	case 24:	SerialPrintln(("GARMIN ")); break;
      	default:	SerialPrintln((tho)); break;
      }
    break;  
    
    case 0377:
    default:
		break;
	/*
      if (b4 < 16)
        SerialPrint(("0"));
      SerialPrint((b4, HEX));
      
      SerialPrint((" "));
      if (b3 < 16)
        SerialPrint(("0"));
      SerialPrint((b3, HEX));
      
      SerialPrint((" "));
      if (b2 < 16)
        SerialPrint(("0"));
      SerialPrint((b2, HEX));
      
      SerialPrint((" "));
      SerialPrintln((b1, OCT));
    */
    
    }
    
    
    // When ANY ARINC data comes in do a time-check to pace the NMEA output
	if (NMEA_delay < millis())
	{
		// Reset the delay to 10 hz
		NMEA_delay = millis() + NMEA_DELAY_MS;
			
		// Send the Vertical Nav only if valid
		if (GS_Timeout-- > 0) {
			
			// Send the NMEA VNAV message
			outputPGRMH(1);
		} else {
		
			// Reset GS vars
			VerticalDeviation = 0;
			GS_Timeout = 0;
			
			// Don't display stale GS data
			// outputPGRMH(0);
			
		}
		
		// Send the minimum required NMEA every time
		outputRMC();
		outputRMB();
		outputBOD();
	}


}


/*************************************************************************************************
**************************************************************************************************/
float angle_bnr_calc( word b4, word b3, word b2, int lastbit, float bnrstart)
{
  int word2;
  int twos, i;
  float angle, bnr;

/*
    if (b4 < 16) SerialPrint(("0");
    SerialPrint((b4, HEX);
    if (b3 < 16) SerialPrint(("0");
    SerialPrint((b3, HEX);
    if (b2 < 16) SerialPrint(("0");
    SerialPrint((b2, HEX);
    SerialPrint((" ");
  */   
    // save off the two's complement flag and then consolidate and shift the value so we can use it
    twos  = b4 & 0x80;
    word2 = (((b4&0x7F) << 8) + b3);
     
    // It's either a 2's complement of (360 - Track Angle) else its the positive angle 0 to < 180 degrees
   // if (twos) {
    //  word2 = word2 ^ 0xFFFFFFFF;
   // }
      
    // calculate the angle based on BNR math
    angle = 0; bnr = bnrstart;
    
    // Now put initial binary value into a float and iterate from hi-bit to lo-bit, 
    // dividing the float/2 each time and accumulating the result
    
    for (i=14; i>=lastbit; i--) {
      if (bitRead(word2, i))
        angle += bnr;
        
      bnr /= 2.0;
    }
  
    if (twos)
      angle = 180 + angle;
        
    return angle;
}  



/*************************************************************************************************
**************************************************************************************************/
float distance_bnr_calc( word b4, word b3, word b2, int lastbit, float bnrstart)
{
  int word2, xbit=0;
  int sense, i;
  float dist, bnr;
/*
    if (b4 < 16) SerialPrint(("0"));
    SerialPrint((b4, HEX));
    if (b3 < 16) SerialPrint(("0"));
    SerialPrint((b3, HEX));
    if (b2 < 16) SerialPrint(("0"));
    SerialPrint((b2, HEX));
    SerialPrint((" "));
*/   
    // save off the sense flag and then mask it off
    sense  = b4 & 0x80;
    word2 = (((b4&0x7F) << 8) + b3);

    if (sense) { 
      word2 = word2 ^ 0xFFFF;
    }
    
    // calculate the angle based on BNR math
    dist = 0; bnr = bnrstart;
    
    // Do we need the b2 bits?
    if (lastbit < 0) {
      xbit = abs(lastbit);
      lastbit = 0;
    }
      
    // Now put initial binary value into a float and iterate from hi-bit to lo-bit, 
    // dividing the float/2 each time and accumulating the result
    for (i=14; i>=lastbit; i--) {
      if (bitRead(word2, i))
        dist += bnr;
        
      bnr /= 2.0;
    }
  
    // If we are using extended resolution continue into the b2 word
    if (xbit > 0) {
      for (i=7; i>(7-xbit); i--) {
        if (bitRead(b2, i))
          dist += bnr;
          
        bnr /= 2.0;
      }
    } 
  
    return dist;
}  


//////////////////////////////////////////////////////////////////////////////////////////////////
static int stream_pos = 0;
static  char outstr[128], instrm[128];

//////////////////////////////////////////////////////////////////////////////////////////////////
void pad_int_to_string(char *out, int val, int width)
{
	int i, vwidth;

	if (val)
		vwidth = log10(val) + 1;
	else
		vwidth = 1;
		

	// Pad total width with zeros
	for (i=0; i<width; i++)
		out[i] = '0';
		
	// Write string for integer val(ue)
	itoa(val, &(out[width-vwidth]), 10);

	// Add the line termination
   	strcpy(&(out[width]), "\r\n"); 
}


void pad_int_to_string_noLR(char *out, int val, int width)
{
int	i, vwidth;
	
	if (val)
		vwidth = log10(val) + 1;
	else
		vwidth = 1;
		
	// Pad total width with zeros
	for (i=0; i<width; i++)
		out[i] = '0';
	
	// Write string for integer val(ue)
	itoa(val, &(out[width-vwidth]), 10);
}


//////////////////////////////////////////////////////////////////////////////////////////////////
void convert_efis_to_fadc()
{
	int  spos, i, chksum, thou,hund,tens,ones,tenth;
	char numbuf[8];

		// out-stream position index to start
	spos = 0;
 
  
// <STX> start-transmit character
    outstr[spos++] = char(2);

	
		// Save altitude sign and zero it to terminate the IAS string properly for itoa()
	char altsign = instrm[24];
	instrm[24] = 0;
	
// IAS - Dynon sends M/S - convert to Knots
    strcpy(&(outstr[spos]), "ZA"); 
    spos+= 2;

	float ias = atoi(&(instrm[20])) * 0.19438445;
	pad_int_to_string(&(outstr[spos]), ias, 3);
    spos+= 5;
    
	
		// Save the VSI sign for later and to zero the end of the altitude string for itoa()
	altsign = instrm[29];
	instrm[29] = 0;

	// low bit of pos[47] determines which info is in EFIS data
    if (instrm[46] != '0') {
// PALT  - Dynon sends Meters - convert to 10's of feet
	    strcpy(&(outstr[spos]), "ZD"); 
	    spos+= 2;
		outstr[spos++] = altsign;
	
		float alt = atoi(&(instrm[25])) * .32808399;
		pad_int_to_string(&(outstr[spos]), alt, 4);
	    spos+= 6;
	    } else {
	    }
    
	// low bit of pos[47] determines which info is in EFIS data
    if (instrm[46] == '0') {
// Rate of Turn +/- (deg / sec)
	    strcpy(&(outstr[spos]), "ZJ"); 
		outstr[spos+2] = altsign;	
	    strncpy(&(outstr[spos+3]), &(instrm[30]), 2); 
	    strcpy(&(outstr[spos+5]), "\r\n"); 
	    spos+= 7;
	} else {
// Vertical SPD
	    strcpy(&(outstr[spos]), "ZK"); 
	    
	    // write the sign and zero the end of the number for atoi()
		outstr[spos+2] = altsign;	
		instrm[33] = 0;
		int vsi = atoi(&(instrm[30])) * 6;
		
		pad_int_to_string(&(outstr[spos+3]), vsi, 3);
	    spos+= 8;
	}
	
	// Heading
    strcpy(&(outstr[spos]), "ZL"); 
    strncpy(&(outstr[spos+2]), &(instrm[17]), 3); 
    strcpy(&(outstr[spos+5]), "\r\n"); 
    spos+= 7;
		
	// ERR
    strcpy(&(outstr[spos]), "ZQ000\r\n"); 
    spos+= 7;
		
	// Indicated Altitude (Baro corrected)
    //strcpy(&(outstr[spos]), "Ga+0100\r\n"); 
    //spos+= 9;

    // Calculate one byte checksum
    for (i=chksum=0; i<spos; i++) {
      chksum += byte(outstr[i]);
    }
    chksum = chksum%256;

	//Checksum padded to 3 digits (0..255)
    strcpy(&(outstr[spos]), "ZR");
    spos+= 2;
    
    // Pad checksum string (TODO:  find a more efficient way to do this!!!!)
	pad_int_to_string(&(outstr[spos]), chksum, 3);
    spos+= 5;
	
	// <ETX> end-transmit character
    outstr[spos++] = char(3);
    //strcpy(&outstr[spos++], "0x03", 1); 

    //Serial.write((byte *)outstr, spos);
    
    // Send out to GPS via Tx1
    Serial1.write((byte *)outstr, spos);

}
//////////////////////////////////////////////////////////////////////////////////////////////////
int  EFISCount = 0;

void process_efis_data()
{
  byte inbyte;
  int  num;
  
  // limit read to whats there or what our buffer can handle.  No blocking please!
  instrm[stream_pos++] = inbyte = Serial2.read();
  
  //SerialPrint((inbyte);
  if (inbyte == 0x0A) {

	// Look for the correct line length in lieu of a CRC - seems to weed out the cruft
    if (stream_pos == 53) {
      
      // DEBUG: Serial.write((byte *)instrm, stream_pos);
      
        convert_efis_to_fadc();
     
    }

	// If we found a LF reset the buffer to capture the next one.
    stream_pos = 0;
  }  

  //if (stream_pos > 127) {
  //  SerialPrint(("in stream too long - no 0x0A detected"));
  //  stream_pos = 0;
  //}
  
  
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////

void process_user_command()
{
	byte command = ' ';
	command = Serial.read();

  
  	if (command >= '0' && command <= '9')
  	{
	   	Test_Rate = (command - 0x2F)*50;
  	}
  	
	switch (command)
	{
	case 'S':
	{
	   	Test_Output ^= 1;
	   	Test_Rate = 1000;
		break;
	}
	case 'l':
	{
		controlword1 = controlword1 | NOTSLFTST;
		resetARINC();
		break;
	}
	case 'L':
	{
		controlword1 = controlword1 & ~NOTSLFTST; 
		resetARINC();
		break;
	}
	case 'P':
	{
		controlword1 = controlword1 | PAREN;
		resetARINC();
		break;
	}
	case 'p':
	{
		controlword1 = controlword1 & ~PAREN; 
		resetARINC();
		break;
	}
	case 'r':
	{
		controlword2 = controlword2 | RXLO;
		resetARINC();
		break;
	}
	case 'R':
	{
		controlword2 = controlword2 & ~RXLO; 
		resetARINC();
		break;
	}
	case 't':
	{
		controlword2 = controlword2 | TXLO;
		resetARINC();
		break;
	}
	case 'T':
	{
		controlword2 = controlword2 & ~TXLO; 
		resetARINC();
		break;
	}
	default:
		break;
	}
}
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
  int txReady;
 
  SerialPrintln(("Send L/l for Loopback/normal, R/r for receiver speed HIGH/low, P/p for Parity ENABLE/disable"));

  resetARINC();  
  while (true)
  {
    if (Serial.available()) {
      process_user_command();
    }
    
    if (Serial2.available()) {
      process_efis_data();
    }
    

  // TEST Output
  
    
	if (Test_Output && (NMEA_delay < millis()))
	{
			// Reset the delay to 10 hz
			NMEA_delay = millis() + Test_Rate;

			SelectedCourse	  = Test_VNAV + 100;
		   	VerticalDeviation = Test_VNAV;
		   	CrossTrackDist    = abs(Test_VNAV);
		   	CrossTrack_RL	  = (Test_VNAV < 0) ? 1 : 0;
		   	Test_VNAV++;
		   	
		   	if (Test_VNAV > 100) Test_VNAV = -100;
		   	
		   	VertScaleFactor   = 100;
		   	LatScaleFactor	  = 100;

			outputPGRMH(CrossTrack_RL);
			outputRMC();
			outputRMB();
			outputBOD();

			if (CrossTrack_RL) {
				digitalWrite(Annunciate_TRM,HIGH);
				digitalWrite(Annunciate_APR,LOW);
			} else {
				digitalWrite(Annunciate_TRM,LOW);
				digitalWrite(Annunciate_APR,HIGH);
			}
			
			//outputHSI();
			//outputOBS();
			//outputActiveVOR();
			//outputSL30Misc();
    }

	// Someday we should really play with ARINC 429 output as well.
	//
    txReady = digitalRead(a429TXR_pin);
    if (txReady==HIGH)         // the transmitter is not busy now so we can send
    {
        // txARINC();  // a canned test word
    }
    rxARINC();
  }
}


