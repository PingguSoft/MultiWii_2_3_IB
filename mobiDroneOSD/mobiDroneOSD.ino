/*===============================================================================================
 mobiDrone OSD v 2.3 for MultiWii by Michal Maslik (MichalM_sk) mobicek@gmail.com
 only for atmega 328 1kB ram is too low in atmega 168   
 Compatible with Arduino 1.0

/// for MULTIWII 2.1 and LATER (PLUG AND PLAY) ///////////////

/- mobiDrone OSD v 2.3 PRE Release 7.8 FINAL REPACKED
 + fixed some bugs
 + added hatuul BigNumber Font
 + added support for 50A BiDirectional ACS756 current sensor

/- mobiDrone OSD v 2.3 PRE Release 7.8 FINAL
 + better font for big numbers
 + latitude, longitude is implemented into the new layout 

/- mobiDrone OSD v 2.3 PRE Release 7.8 (Beta2)     
 + pre final version of new layout structure (lat, lon is missing at the moment)

/- mobiDrone OSD v 2.3 PRE Release 7.8 (Beta) 
 + angle algorithm improved    
 + layout modified (new charset)
 
/- mobiDrone OSD v 2.3 PRE Release 7.7  
 + added RSSI number value    
 + added function to shutout osd text by screen switcher (selected AUX switch) 
 + added support for 200A BiDirectional ACS758 current sensor 
 + added support for 25A flytron ultralight current sensor

/- mobiDrone OSD v 2.3 PRE Release 7.6  
 + Main A.horizont was modified (better resolution for low angles)   
 
/- mobiDrone OSD v 2.3 PRE Release 7.5  
 + New result screen with fly time, high altitude, high speed, high current, high distance   
 + Added fly time to main screen
 + Adjustable max radar distance
 + Fixed menu noises in some case
 + GPS speed units changed to km/h (tested by Neo360) 

/- mobiDrone OSD v 2.3 PRE Release 7 (FINAL)  
 + Fixed HOME arrow bug in OFF of latitude and longitude display.   
 + Better optimization of radar screen
 + Some layout changes

/- mobiDrone OSD v 2.3 PRE Release 7 (Beta)
 + Fixed bug in NTSC video  
 + Fixed problem with compass data processing

/- mobiDrone OSD v 2.3 PRE Release 7 (Alpha 3)  
 + Fixed bug in LAT & LON representation
 + Fixed problem with Flytron current sensor

/- mobiDrone OSD v 2.3 PRE Release 7 (Alpha 2)
 + Fixed bug in GPS data processing (LATITUDE, LONGITUDE reading problem)
 + Fixed problem with detection of camera resolution when the camera is turned on later than osd
 + Fixed bug in A.Horizont sensitivity (Contributed by Y.Mita)

/- mobiDrone OSD v 2.3 PRE Release 7 (Alpha)
 + new menu attributes (bat1,bat2 voltage correction, roll, pitch senitivity, current sensor sellection, screen switch sellection)
 + better code optimisation
 + new serial protocol implemented
 + new Pid & Mode configuration
 + automatic detection of camera resolution
 + check the consistency of EEPROM data
 + OSD configuration moved to a new file (config.h) 
 + GPS speed bug removed , speed now in units - m/s (tested by ankimo)
 + remove bug in compass (opposite direction of rotation) (contributed by ymita) 
 + remove bug in voltage comparator (is no longer necessary adjust bat2 voltage - bat2_correction parameter)
 + you can enable/disable to view latitude & longitude 
 + added support for new current sensor - Flytron Ultralight 50A Current Sensor (tested by nhadrian)
 + added support to adjust artifical horizont sensitivity (contributed by nhadrian) 
 
 
/// for MULTIWII 2.0 //////////////////////////////////////////////////////////////////////

 /- mobiDrone OSD v 2.3 PRE Release 6
 + added support for gps speed (right value in A. Horizont; units dm/s => 0.1m/s)
 + added Radar Screen (switching between screens can be configured to AUX1 - AUX3 switch) in config part
 + added support for new current sensor - ACS758 50A UniDirectional (contributed by djrm)  
 + new option(CAMERA_V_RESOLUTION) in config part for adjusting non-standard camera vertical resolution 
 + vertical size of main layout is automatically transformed according to the camera vertical resolution
 + too many optimization changes to free up space :) 
 
 /- mobiDrone OSD v 2.3 PRE Release 5 FIXED
 Bug - NTSC is also shown in PAL mode - FIXED 
 Bug - Altitude is always 0 - FIXED  
 + A. horizont logic is possible to change by #define (ROLL_TILT_REVERSED,PITCH_TILT_REVERSED)
 + You can enable/disable vertical lines in A.horizont by #define VERTICAL_LINES  
 
 /- mobiDrone OSD v 2.3 PRE Release 5
 + new redesigned layout (new icons, block structure,customisable)
 + better RAM management
 + PITCH tilt reversed to normal
   
 /- mobiDrone OSD v 2.3 PRE Release 4 (not Final!)
 + Current sensor ACS758 100A BiDirectional support, indicator for consumed capacity & actual current
 + NTSC support
 + Layout cleared - you can disable not usable values
 + Baro shows relative value now (on the ground 0 meters)
 + Some bugs repaired  
 
 /- mobiDrone OSD v 2.3 PRE Release 3 (not Final!)
 + serial reading/processing routine optimized for better performance
 + better RAM management 
 + bug in compass fixed
 + new layout design + throttle level removed, heading angle visualisaton added, last line relocated   
 
 /- mobiDrone OSD v 2.3 PRE Release 1 (not Final!)
 + added config menu pages (volt,rssi,pid,modes,info,calibrate...)
 + new angle visualisation
 + different frame size for standard serial frame and configure serial frame 
 
 /- mobiDrone OSD v 2.2
 + serial data reading redesigned
 + add gps distance to home pos
 + add gps direction to home pos 
 + add aux3, aux4 visualisation
 + redesigned compass data
 + visualisation of RTH function  
 
 /- mobiDrone OSD v 2.1
 
 + add support for MultiWii 2.0 firmware
 + add GPS PID item in config menu 
 
 Copyright (C) 2012, Michal Maslik, mobicek@gmail.com
 
 This program is free software: you can redistribute it and/or modify it under the terms of the 
 Creative Commons NonCommercial-ShareAlike 3.0 Unported (CC BY-NC-SA 3.0)
 http://creativecommons.org/licenses/by-nc-sa/3.0/
 
 ONLY FOR NON-COMMERCIAL USAGE WITH ORIGINAL MOBIDRONE OSD HARDWARE.
 
 THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE COMMONS PUBLIC LICENSE 
 ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY COPYRIGHT AND/OR OTHER APPLICABLE LAW. 
 ANY USE OF THE WORK OTHER THAN AS AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 
 BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO BE BOUND BY THE 
 TERMS OF THIS LICENSE. TO THE EXTENT THIS LICENSE MAY BE CONSIDERED TO BE A CONTRACT, 
 THE LICENSOR GRANTS YOU THE RIGHTS CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF 
 SUCH TERMS AND CONDITIONS.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 ================================================================================================*/

#include "config.h"
#include <avr/sleep.h>
#include <avr/pgmspace.h>                                
#include <avr/delay.h>
#include <EEPROM.h>

#define VERSION 23

#define PAL 0
#define NTSC 1

#define NUMBER_OF_CURRENT_SENSORS 6

//Hardware definitions//////
#define STANDARD_RC     // 
#define MOBIDRONEOSD_V2 //
////////////////////////////


//Default values - if eeprom is empty these data will be loaded
uint8_t minRSSI = 0;                                                           
uint16_t maxRSSI = 280;

#define NULL_CORRECTION 125

#define DEFAULT_BAT1_CRITICAL_VOLTAGE 100//10.0V
#define DEFAULT_BAT2_CRITICAL_VOLTAGE 65 //6.5V
#define DEFAULT_RC_SENS               1
#define DEFAULT_AUX_SWITCH            3
#define DEFAULT_BAT1_CORRECTION       NULL_CORRECTION //125 for 0.0V  
#define DEFAULT_BAT2_CORRECTION       NULL_CORRECTION //125 for 0.0V  
#define DEFAULT_CURRENT_TYPE          1 
#define DEFAULT_ROLL_SENSITIVITY      10//10 - no correction
#define DEFAULT_PITCH_SENSITIVITY     10 

#define clearVideoOut SPDR = 0b00000000;                
#define dimOn  DDRB |= 0b00000010;                  
#define dimOff DDRB &= 0b11111101;                  

//delays
#define delay1Pixel __asm__("nop\n\t""nop\n\t");   //~ 120nS
#define delay15 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS
#define delay13 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t"); 
#define delay10 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay9 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay8 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS
#define delay7 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS
#define delay6 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay5 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay4 __asm__("nop\n\t""nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay3 __asm__("nop\n\t""nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay2 __asm__("nop\n\t""nop\n\t");   // nop - 62,5nS 
#define delay1 __asm__("nop\n\t");   // nop - 62,5nS 

//Screen & Horizont panel settings
#define FLIGHT_PANEL_START_LINE 94

#if (VIDEO_TYPE == NTSC)
  #define TOP_MARGIN 120
#else
  #define TOP_MARGIN 140  
#endif

#define MAIN_PANEL_CENTER_POINT 6
#define MAIN_PANEL_RADIUS       1
///////////////////////////////////////////////////

static uint8_t videoResDetected = 0;
static uint16_t CAMERA_V_RESOLUTION12 = 0;
static uint16_t CAMERA_V_RESOLUTION12_OLD;
volatile uint16_t FLIGHT_PANEL_END_LINE;
volatile uint8_t  MAIN_PANEL_CENTER_LINE;
volatile uint8_t  CLMR;
volatile uint8_t  CLPR;
volatile uint8_t  CLMR2;

//analog read values
#ifdef MOBIDRONEOSD_V2
 static float compCorrection = 92.16;
#endif

static int16_t bat1_voltage = 0;
static float sensVcc=0 ;                              
static float sensRSSI=0;                              
static float currentAverage = 0.0;
static float consumedmAh = 0.0;

static uint8_t currIndex = 0;

static uint32_t calcTimes=0;                 
static uint32_t calcFlyTimes=0;
static uint32_t mSeconds=0;
static uint16_t seconds=0;
static uint8_t MinTime_dump = 0;
static uint8_t SecTime_dump = 0;

volatile uint32_t bat1Timer=0;
volatile uint32_t bat2Timer=0;
volatile uint32_t gpsFixTimer=0;
volatile uint32_t autoModeTimer = 0;
volatile uint32_t a20mSecTimer = 0;

uint16_t viewTime = 0;
/////
volatile uint8_t setDim = 0;
volatile uint8_t curveBuffer[14];

volatile uint16_t temp1 = 0;
volatile uint8_t temp2 = 0;
int8_t pointYpos = 0;
uint8_t pointXpos = 0;
uint8_t angleType_count = 0;
volatile uint8_t math1 = 0;
volatile uint8_t math2 = 0;
uint8_t startPos_temp = 0;  //startPos of angle Meter
uint8_t endPos_temp = 12;    //endPos of angle Meter
//angles
int16_t readedRollAngle = 0;
int16_t readedRollAngle_radar;
int16_t readedRollAngle_temp = 0;
int16_t readedPitchAngle = 0;
int16_t readedPitchAngle_radar;
int8_t pitchAngleRelative = 0;
int8_t pitchAngleRelative_temp = MAIN_PANEL_CENTER_LINE;
//compass
int16_t readedHeading = 0;
//gps
int16_t homeDirection = 0;
int16_t relativeDirection = 0; 
uint16_t homeDistance = 0;  
//speed
volatile uint8_t speed_pointer = 0;
//altitude
volatile uint8_t cmAlt_pointer = 0;
int16_t storedAlt = 0;
uint8_t firstAlt = 0;
//Statistics
int16_t highAltitude = 0;
uint16_t highSpeed = 0;
uint16_t highDistance = 0; 
uint16_t highAmperage = 0;

//Motor status - armed, disarmed
uint8_t armed = 0;
uint8_t armed_old = 0;

volatile uint8_t cryticalPart = 0;
volatile uint8_t buffered = 0;   //data are writed to video buffer
volatile uint8_t layoutWrited = 0;

////
uint8_t ftoken=0;
uint8_t stoken=0;
volatile uint16_t Hsync = 0;
volatile uint8_t  lineId = 0;
volatile uint16_t varOSD = 0;
volatile uint8_t letterCounter = 0;
uint8_t letterCounter1 = 0;
uint8_t lineCounter = 0;
volatile uint8_t dataCount = 0;
uint8_t segmentCounter = 0;
uint8_t mainPanelLineGenerated = 0;

volatile uint8_t bat1Blink = 0;
volatile uint8_t bat2Blink = 0;
volatile uint8_t gpsFixBlink = 0;
volatile uint8_t autoModeBlink = 0;

//screen identifier
volatile uint8_t screenNumber = 0;
volatile uint8_t screen = 0;

uint8_t screenSwitchEnable = 1;

#define NUM_OF_AUX_SCREEN 3
uint8_t screenMap[] = {1,2,5};
uint8_t screenPointer = 0;

////////////////////////Internal settings
//EEPROM data pointer
#define BAT1_LEVEL 1          //1 byte    calibrate bat1 alarm
#define BAT2_LEVEL 2          //1 byte    calibrate bat2 alarm
#define RSSI_CALIB 3          //2 bytes   calibrate max rssi value
#define AUX_SW 5              //1 byte    calibration temperature sensor (in future)
#define RC_SENS    6          //1 byte    rc sensitivity 1 - 5
#define BAT1_CORRECTION 7     //1 byte    bat1 voltage correction
#define BAT2_CORRECTION 8     //1 byte    bat2 voltage correction
#define CURR_SENS_TYPE  9     //1 byre    current sensor sellection 0 - no sensor
#define ROLL_SENSITIVITY   10 //1 byre    adjusting roll sensitivity of artifical horizont
#define PITCH_SENSITIVITY  11 //1 byre    adjusting pitch sensitivity of artifical horizont 

#define EEPROM_DATA_LENGTH 12  

//readed eeprom data
uint8_t eepromBuffer[EEPROM_DATA_LENGTH];
////////////////////////////Config screen

//Aliases
//RC channels
#define ROLL       0
#define PITCH      1
#define YAW        2
#define THROTTLE   3
#define AUX1       4
#define AUX2       5
#define AUX3       6
#define AUX4       7

//PID data
#define ROLL       0
#define PITCH      1
#define YAW        2
#define ALT        3
#define POS        4
#define POSR       5
#define NAVR       6
#define LEVEL      7
#define MAG        8
#define VEL        9
#define RCRATE     10
#define RCEXPO     11
#define RPRATE     12
#define YAWRATE    13
#define DYNTHRPID  14
#define THRMID     15
#define THREXPO    16

#define PIDITEMS   10
#define PID_DATA_BYTES 3*PIDITEMS

#define RCTUNING_ITEMS 7
#define RCTUNING_ITEMS_BYTES RCTUNING_ITEMS*3

//readed PID data from Multiwii
static uint8_t pidBuffer[PID_DATA_BYTES + RCTUNING_ITEMS_BYTES];
static uint8_t pidData_format[] = {3,3,3,3,2,  //PID1 screen
                                   3,3,3,1,0,  //PID2 screen
                                   1,1,1,1,1}; //PID3 screen

//Mode config 
#define BOXACC       0
#define BOXBARO      1
#define BOXMAG       2
#define BOXCAMSTAB   3
#define BOXCAMTRIG   4
#define BOXARM       5
#define BOXGPSHOME   6
#define BOXGPSHOLD   7
#define BOXPASSTHRU  8
#define BOXHEADFREE  9
#define BOXBEEPERON  10
#define BOXLEDMAX    11 // we want maximum illumination
#define BOXLLIGHTS   12 // enable landing lights at any altitude
#define BOXHEADADJ   13 // acquire heading for HEADFREE mode

#define BOXITEMS 14

static uint16_t activate[BOXITEMS];

static uint16_t aux_map [] = {0x0007, 0x0038, 0x01C0, 0x0E00};
static uint8_t aux_shift[] = {0,3,6,9};
                      
static uint8_t  sensPresent = 0;
static uint16_t sensActivated = 0;
static uint16_t i2c_errors = 0;

uint8_t up = 0;
uint8_t down = 0;
uint8_t left = 0;
uint8_t right = 0;
uint8_t yleft = 0;
uint8_t yright = 0;

uint8_t x_pos = 0;
uint8_t y_pos = 1;
uint8_t y_pos_old = 0;
uint8_t configDataReaded = 0;

//////////////////////////// Serial settings
#define CLOCK 16000000UL
#define BAUD 115200
#define BAUD_SETTINGS ((CLOCK / 4 / BAUD - 1) / 2)
////////////////////////////////////////////

#define CONTENT_END    30*7  //30 chars 7 bytes       

#define VIDEO_BUFFER_SIZE 1200  //video buffer bytes
#define EXTENDED_BUFFER 56      //for radar screen

volatile uint8_t VideoBuffer[VIDEO_BUFFER_SIZE + EXTENDED_BUFFER];

#define HEADER 24
#define HEADER_SIZE HEADER*7

#define LAYOUT 105  //140

//layout
#define ONE_LINE_SIZE 30 * 7         // standard 30 chars  8x8 per line


//radar screen

volatile int8_t posArray[7];
volatile uint16_t g_temp,g_temp1;

#define GRAPHIC_SIZE 96
#define GRAPHIC_SIZE_12 GRAPHIC_SIZE/2
#define GRAPHIC_SIZE_12M1 GRAPHIC_SIZE_12 - 1
#define GRAPHIC_SIZE_18 GRAPHIC_SIZE/8
#define GRAPHIC_POS_START 45

#define RADAR_RADIUS  44
#define RADAR_RADIUS_12 RADAR_RADIUS/2
#define RADAR_RADIUSM2 RADAR_RADIUS - 2
#define RADAR_RADIUSM10 RADAR_RADIUS - 10
#define RADAR_DIAMETER RADAR_RADIUS*2 

#define GRAPHIC_POS_END GRAPHIC_POS_START + GRAPHIC_SIZE

#define RADAR_CENTER_X GRAPHIC_SIZE/2
#define RADAR_CENTER_XP4 RADAR_CENTER_X + 4
#define RADAR_CENTER_XP8 RADAR_CENTER_X + 8
#define RADAR_CENTER_XP12 RADAR_CENTER_X + 12
#define RADAR_CENTER_XP16 RADAR_CENTER_X + 16
#define RADAR_CENTER_XM3 RADAR_CENTER_X - 3

#define RADAR_CENTER_Y GRAPHIC_SIZE/2
#define RADAR_CENTER_YM3 RADAR_CENTER_Y - 3
#define RCYPRRM4 RADAR_CENTER_Y + (RADAR_RADIUS - 4)
#define RCYPRR12 RADAR_CENTER_Y + RADAR_RADIUS_12

#define MAX_RADAR_DISTANCE_HIGH MAX_RADAR_DISTANCE/100
#define MAX_RADAR_DISTANCE_MID  ((MAX_RADAR_DISTANCE/10) - ((MAX_RADAR_DISTANCE/100) * 10))
#define MAX_RADAR_DISTANCE_LOW  (MAX_RADAR_DISTANCE - ((MAX_RADAR_DISTANCE/10) * 10))

#define MAX_RADAR_DISTANCE_HALF_SIZE MAX_RADAR_DISTANCE/2

#define MAX_RADAR_DISTANCE_HALF_SIZE_HIGH MAX_RADAR_DISTANCE_HALF_SIZE/100
#define MAX_RADAR_DISTANCE_HALF_SIZE_MID  ((MAX_RADAR_DISTANCE_HALF_SIZE/10) - ((MAX_RADAR_DISTANCE_HALF_SIZE/100) * 10))
#define MAX_RADAR_DISTANCE_HALF_SIZE_LOW  (MAX_RADAR_DISTANCE_HALF_SIZE - ((MAX_RADAR_DISTANCE_HALF_SIZE/10) * 10))

#define RADAR_DISTANCE_FACTOR RADAR_RADIUS/MAX_RADAR_DISTANCE


//menu//////////////////////////////////////////////////////////////
#define MENU_START_LINE 80 //line of start printing menu items
#define ONE_LINE_IN_MENU 20          // 16 chars per line
#define ONE_LINE_IN_MENU_SIZE ONE_LINE_IN_MENU * 7   // n-chars size 7x8         
#define MAX_MENU_ITEMS 7        //lines              
#define MAX_MENU_PAGES 10       //pages
#define LEFT_MARGIN_IN_MENU 7
#define LAST_CONFIG_PAGE 8

//page index
#define INTERNAL_SETTINGS1 1
#define INTERNAL_SETTINGS2 2
#define PID1     3     
#define PID2     4     
#define PID3     5
#define MODE1    6
#define MODE2    7
#define SENSOR2  8
#define SENSOR1  9
#define INFO     10

uint8_t pageNumber = 1;      //id of menu page
uint8_t pageNumber_old = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Serial routine ////////////////////////////////////////////////////////////////////////////////////////////////////////
#define MSP_IDENT                100   //out message         multitype + multiwii version + protocol version + capability variable
#define MSP_STATUS               101   //out message         cycletime & errors_count & sensor present & box activation
#define MSP_RAW_IMU              102   //out message         9 DOF
#define MSP_SERVO                103   //out message         8 servos
#define MSP_MOTOR                104   //out message         8 motors
#define MSP_RC                   105   //out message         8 rc chan
#define MSP_RAW_GPS              106   //out message         fix, numsat, lat, lon, alt, speed
#define MSP_COMP_GPS             107   //out message         distance home, direction home
#define MSP_ATTITUDE             108   //out message         2 angles 1 heading
#define MSP_ALTITUDE             109   //out message         1 altitude
#define MSP_BAT                  110   //out message         vbat, powermetersum
#define MSP_RC_TUNING            111   //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_PID                  112   //out message         up to 16 P I D (8 are used)
#define MSP_BOX                  113   //out message         up to 16 checkbox (11 are used)
#define MSP_MISC                 114   //out message         powermeter trig + 8 free for future use
#define MSP_MOTOR_PINS           115   //out message         which pins are in use for motors & servos, for GUI 
#define MSP_BOXNAMES             116   //out message         the aux switch names
#define MSP_PIDNAMES             117   //out message         the PID names
#define MSP_WP                   118   //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold

#define MSP_SET_RAW_RC           200   //in message          8 rc chan
#define MSP_SET_RAW_GPS          201   //in message          fix, numsat, lat, lon, alt, speed
#define MSP_SET_PID              202   //in message          up to 16 P I D (8 are used)
#define MSP_SET_BOX              203   //in message          up to 16 checkbox (11 are used)
#define MSP_SET_RC_TUNING        204   //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
#define MSP_ACC_CALIBRATION      205   //in message          no param
#define MSP_MAG_CALIBRATION      206   //in message          no param
#define MSP_SET_MISC             207   //in message          powermeter trig + 8 free for future use
#define MSP_RESET_CONF           208   //in message          no param
#define MSP_WP_SET               209   //in message          sets a given WP (WP#,lat, lon, alt, flags)

#define MSP_EEPROM_WRITE         250   //in message          no param
#define MSP_DEBUG                254   //out message         debug1,debug2,debug3,debug4

static uint8_t mwcRequestHeader[] = {'$','M','<'};
#define HAF_RESPONSE_SIZE 6       //header + footer response size

static uint8_t requestId = 0;
static uint8_t dataRequest[] = {MSP_ATTITUDE, MSP_RAW_GPS, MSP_ATTITUDE, MSP_RC , MSP_ATTITUDE, MSP_COMP_GPS, MSP_ATTITUDE, MSP_RC, MSP_ATTITUDE, 
                                MSP_ALTITUDE, MSP_ATTITUDE, MSP_RC, MSP_ATTITUDE, MSP_BAT, MSP_ATTITUDE, MSP_RC, MSP_ATTITUDE, MSP_STATUS, MSP_ATTITUDE, MSP_RC,  //standard data    
                                MSP_IDENT, MSP_RC_TUNING, MSP_RC, MSP_STATUS, MSP_PID, MSP_RC, MSP_BOX, MSP_STATUS, MSP_BAT};                              //config data

#define DATA_REQUEST_SIZE           29

#define STANDARD_DATA_REQUEST_SIZE  20
#define CONFIG_DATA_REQUEST_START   19
#define CONFIG_DATA_MODULO          10

#define RS232_TX_BUFFER_SIZE 38 
volatile uint8_t RS232_TX_Buffer[RS232_TX_BUFFER_SIZE];        

volatile uint8_t synchronization = 0;
volatile uint8_t readEnable;

//multiwii data variables
static int16_t rcData[8];
static int16_t throttle_old;
static uint8_t gpsFix = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PROGMEM prog_uchar header[] =                     
{ 
  #if (VIDEO_TYPE == NTSC)
    10,'M'-54,'O'-54,'B'-54,'I'-54,'D'-54,'R'-54,'O'-54,'N'-54,'E'-54,10,'O'-54,'S'-54,'D'-54,10,'V'-54,2,38,3,10,'N'-54,'T'-54,'S'-54,'C'-54,
  #else
    10,'M'-54,'O'-54,'B'-54,'I'-54,'D'-54,'R'-54,'O'-54,'N'-54,'E'-54,10,'O'-54,'S'-54,'D'-54,10,'V'-54,2,38,3,10,'P'-54,'A'-54,'L'-54,10,
  #endif
};

//Layout components
#define LEFT_MARGIN_LAYOUT 6

#define RSSIBAR_PADDING 17
#define GPS_NUMSAT_BAR_MARGIN 15

#define RSSIBAR_Y_POS  40
#define TIMEBATBAR_Y_POS  RSSIBAR_Y_POS + 9      //49
#define CURBAR_Y_POS  TIMEBATBAR_Y_POS + 18      //66

#define TOPBAR_CENTER_PADDING 4
#define CENTER_PADDING 6
#define BAT2_PADDING 19

#define COMPASSBAR_Y_POS  83
#define POINT_MARGIN 23
#define COMPASS_PADDING 1
#define COMPASSBAR_MARGIN 16
#define HEADINGBAR_MARGIN 20
#define ARROW_MARGIN  20

//start time title
#define TIMEBAR_TITLE_SIZE 3
#define TIMEBAR_TITLE_START 0  
#define TIMEBAR_TITLE_END TIMEBAR_TITLE_START + (TIMEBAR_TITLE_SIZE * 7)

//start time value
#define TIMEBAR_SIZE 3
#define TIMEBAR_START TIMEBAR_TITLE_END  
#define TIMEBAR_END TIMEBAR_START + (TIMEBAR_SIZE * 7)

//fly time title
#define FLYTIMEBAR_TITLE_SIZE 3
#define FLYTIMEBAR_TITLE_START TIMEBAR_END  
#define FLYTIMEBAR_TITLE_END FLYTIMEBAR_TITLE_START + (FLYTIMEBAR_TITLE_SIZE * 7)

//fly time value
#define FLYTIMEBAR_SIZE 3
#define FLYTIMEBAR_START FLYTIMEBAR_TITLE_END  
#define FLYTIMEBAR_END FLYTIMEBAR_START + (FLYTIMEBAR_SIZE * 7)

//functions bar
#define FUNCTIONSBAR_SIZE 9
#define FUNCTIONSBAR_START  FLYTIMEBAR_END
#define FUNCTIONSBAR_END  FUNCTIONSBAR_START + (FUNCTIONSBAR_SIZE * 7)

//rssi bar
#define RSSIBAR_SIZE 8
#define RSSIBAR_START  FUNCTIONSBAR_END
#define RSSIBAR_END  RSSIBAR_START + (RSSIBAR_SIZE * 7)

//current icon
#define CURRENTBAR_SIZE 3
#define CURRENTBAR_START  RSSIBAR_END
#define CURRENTBAR_END  CURRENTBAR_START + (CURRENTBAR_SIZE * 7)

//capacity (mAh)
#define CAPACITYBAR_SIZE 3
#define CAPACITYBAR_START  CURRENTBAR_END
#define CAPACITYBAR_END  CAPACITYBAR_START + (CAPACITYBAR_SIZE * 7)

//bat2 image
#define BAT2BAR_SIZE 3
#define BAT2BAR_START  CAPACITYBAR_END
#define BAT2BAR_END  BAT2BAR_START + (BAT2BAR_SIZE * 7)

//latitude bar
#define LATBAR_SIZE 9
#define LATBAR_START  BAT2BAR_END
#define LATBAR_END  LATBAR_START + (LATBAR_SIZE * 7)

//longitude bar
#define LONBAR_SIZE 9
#define LONBAR_START  LATBAR_END
#define LONBAR_END  LONBAR_START + (LONBAR_SIZE * 7)

//gps satelite bar
#define NUMSATBAR_SIZE 3
#define NUMSATBAR_START  LONBAR_END
#define NUMSATBAR_END  NUMSATBAR_START + (NUMSATBAR_SIZE * 7)

//gps distance bar
#define GPSDISTANCEBAR_SIZE 6
#define GPSDISTANCEBAR_START  NUMSATBAR_END
#define GPSDISTANCEBAR_END  GPSDISTANCEBAR_START + (GPSDISTANCEBAR_SIZE * 7)

//heading bar
#define HEADINGBAR_SIZE 4
#define HEADINGBAR_START  GPSDISTANCEBAR_END
#define HEADINGBAR_END  HEADINGBAR_START + (HEADINGBAR_SIZE * 7)

//compass bar
#define COMPASSBAR_SIZE 11
#define COMPASSBAR_START  HEADINGBAR_END
#define COMPASSBAR_END  COMPASSBAR_START + (COMPASSBAR_SIZE * 7)

//bat2 voltage value
#define BAT2VOLTAGEBAR_SIZE 3
#define BAT2VOLTAGEBAR_START  COMPASSBAR_END
#define BAT2VOLTAGEBAR_END  BAT2VOLTAGEBAR_START + (BAT2VOLTAGEBAR_SIZE * 7)

//current value
#define CURRENTVALUEBAR_SIZE 5
#define CURRENTVALUEBAR_START  BAT2VOLTAGEBAR_END
#define CURRENTVALUEBAR_END  CURRENTVALUEBAR_START + (CURRENTVALUEBAR_SIZE * 7)

//capacity value
#define CAPACITYVALUEBAR_SIZE 5
#define CAPACITYVALUEBAR_START  CURRENTVALUEBAR_END
#define CAPACITYVALUEBAR_END  CAPACITYVALUEBAR_START + (CAPACITYVALUEBAR_SIZE * 7)

//altitude title bar
#define ALTITUDE_TITLE_SIZE 3
#define ALTITUDE_TITLE_START  CAPACITYVALUEBAR_END
#define ALTITUDE_TITLE_END  ALTITUDE_TITLE_START + (ALTITUDE_TITLE_SIZE * 7)

//altitude units bar
#define ALTITUDE_UNITS_SIZE 3
#define ALTITUDE_UNITS_START  ALTITUDE_TITLE_END
#define ALTITUDE_UNITS_END  ALTITUDE_UNITS_START + (ALTITUDE_UNITS_SIZE * 7)

//altitude title bar
#define SPEED_TITLE_SIZE 3
#define SPEED_TITLE_START  ALTITUDE_UNITS_END
#define SPEED_TITLE_END  SPEED_TITLE_START + (SPEED_TITLE_SIZE * 7)

//altitude units bar
#define SPEED_UNITS_SIZE 3
#define SPEED_UNITS_START  SPEED_TITLE_END
#define SPEED_UNITS_END  SPEED_UNITS_START + (SPEED_UNITS_SIZE * 7)

//gps home direction 
#define ARROWBAR_SIZE 1
#define ARROWBAR_START  SPEED_UNITS_END
#define ARROWBAR_END ARROWBAR_START + ARROWBAR_SIZE*32

//bat1 big numbers bar
#define BAT1_BIGNUMBAR_SIZE 2
#define BAT1_BIGNUMBAR_START  ARROWBAR_END
#define BAT1_BIGNUMBAR_END  BAT1_BIGNUMBAR_START + (BAT1_BIGNUMBAR_SIZE * 32)

//bat2 big numbers bar
#define BAT2_BIGNUMBAR_SIZE 2
#define BAT2_BIGNUMBAR_START  BAT1_BIGNUMBAR_END
#define BAT2_BIGNUMBAR_END  BAT2_BIGNUMBAR_START + (BAT2_BIGNUMBAR_SIZE * 32)

//flytime big numbers bar
#define FLYTIME_BIGNUMBAR_SIZE 2
#define FLYTIME_BIGNUMBAR_START  BAT2_BIGNUMBAR_END
#define FLYTIME_BIGNUMBAR_END  FLYTIME_BIGNUMBAR_START + (FLYTIME_BIGNUMBAR_SIZE * 32)

//starttime big numbers bar
#define STARTTIME_BIGNUMBAR_SIZE 2
#define STARTTIME_BIGNUMBAR_START  FLYTIME_BIGNUMBAR_END
#define STARTTIME_BIGNUMBAR_END  STARTTIME_BIGNUMBAR_START + (STARTTIME_BIGNUMBAR_SIZE * 32)

//altitude big numbers bar
#define ALTITUDE_BIGNUMBAR_SIZE 3
#define ALTITUDE_BIGNUMBAR_START  STARTTIME_BIGNUMBAR_END
#define ALTITUDE_BIGNUMBAR_END  ALTITUDE_BIGNUMBAR_START + (ALTITUDE_BIGNUMBAR_SIZE * 32)

//speed big numbers bar
#define SPEED_BIGNUMBAR_SIZE 2
#define SPEED_BIGNUMBAR_START  ALTITUDE_BIGNUMBAR_END
#define SPEED_BIGNUMBAR_END  SPEED_BIGNUMBAR_START + (SPEED_BIGNUMBAR_SIZE * 32)

//bat1 value in extended buffer (variable shared with compass screen)
#define BAT1BAR_SIZE 3
#define BAT1BAR_START  VIDEO_BUFFER_SIZE
#define BAT1BAR_END  BAT1BAR_START + (BAT1BAR_SIZE * 7)

//bat1 voltage value
#define BAT1VOLTAGEBAR_SIZE 5
#define BAT1VOLTAGEBAR_START  BAT1BAR_END
#define BAT1VOLTAGEBAR_DECIMAL_START  BAT1VOLTAGEBAR_START + 2*7
#define BAT1VOLTAGEBAR_END  BAT1VOLTAGEBAR_START + (BAT1VOLTAGEBAR_SIZE * 7)

PROGMEM prog_uchar layout[] =                     
{ 
  //start time title
  'O'-54,'N'-54,10,
  
  //start time value
  36,0,0,
  
  //fly time title
  'F'-54,'L'-54,'Y'-54,
  
  //fly time value
  36,0,0,
  
  //functions bar
  10,10,10,10,10,10,10,10,10,
  
  //rssi bar
  47,52,53,54,55,56,0,0,
  
  //current bar
  65,10,10,
  
  //capacity bar
  51,'A'-54,62,
  
  //bat2 bar
  57,57,59,
  
  //latitude bar
  0,0,0,38,0,0,0,0,0,
  
  //longitude bar
  0,0,0,38,0,0,0,0,0,
  
  //numsat
  48,10,10,
  
  //gps distance 
  10,10,10,10,0,51,
  
  //heading
  0,0,0,66,

  //compass bar
  63,63,63,63,63,63,63,63,63,63,63,

  //bat2voltage
  38,0,'V'-54,
 
  //current value
  10,0,38,0,'A'-54,
  
  //capacity value
  0,10,10,10,10,
 
  //altitude title 
  'A'-54,'L'-54,'T'-54, 
  
  //altitude units 
  'M'-54,10,10, 
  
  //speed title 
  'S'-54,'P'-54,'D'-54, 
  
  //speed units 
  'K'-54,'M'-54,'H'-54, 
    
  //arrow bar
  10,10,10,10,10,
};

PROGMEM prog_uchar menuPages[] =                     
{ 
  /////////////////////////////INTERNAL SETTINGS 1////////////////////////////////////////
  'I'-54,'N'-54,'P'-54,'U'-54,'T'-54,1,39,2,10,10,10,10,10,10,10,INTERNAL_SETTINGS1/10,INTERNAL_SETTINGS1%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  
  #ifdef BAT1
    10,'B'-54,'A'-54,'T'-54,1,10,'C'-54,'R'-54,'T'-54,36,10,10,10,10,10,10,10,38,10,'V'-54,
  #else
    10,'B'-54,'A'-54,'T'-54,1,10,'C'-54,'R'-54,'T'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,
  #endif
  
  #ifdef BAT2
    10,'B'-54,'A'-54,'T'-54,2,10,'C'-54,'R'-54,'T'-54,36,10,10,10,10,10,10,10,38,10,'V'-54,
  #else
    10,'B'-54,'A'-54,'T'-54,2,10,'C'-54,'R'-54,'T'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,
  #endif
  
  10,'R'-54,'S'-54,'S'-54,'I'-54,36,10,10,10,10,10,10,10,10,10,'C'-54,'A'-54,'L'-54,10,10,
  10,'S'-54,'C'-54,'R'-54,38,'S'-54,'W'-54,36,10,10,10,10,10,10,10,'A'-54,'U'-54,'X'-54,10,10,  //pos 18
  10,'R'-54,'C'-54,'S'-54,'E'-54,'N'-54,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'C'-54,'O'-54,'N'-54,'F'-54,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////INTERNAL SETTINGS 2////////////////////////////////////////
  'I'-54,'N'-54,'P'-54,'U'-54,'T'-54,2,39,2,10,10,10,10,10,10,10,INTERNAL_SETTINGS2/10,INTERNAL_SETTINGS2%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  
  #ifdef BAT1
    10,'B'-54,'A'-54,'T'-54,1,10,'A'-54,'D'-54,'J'-54,36,10,10,10,10,10,10,10,38,10,'V'-54,       //pos 15,16,18
  #else
    10,'B'-54,'A'-54,'T'-54,1,10,'A'-54,'D'-54,'J'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,
  #endif
  
  #ifdef BAT2
    10,'B'-54,'A'-54,'T'-54,2,10,'A'-54,'D'-54,'J'-54,36,10,10,10,10,10,10,10,38,10,'V'-54,
  #else
    10,'B'-54,'A'-54,'T'-54,2,10,'A'-54,'D'-54,'J'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,
  #endif
  
  #ifdef CURRENT_SENSOR
    10,'C'-54,'U'-54,'R'-54,38,'T'-54,'Y'-54,'P'-54,'E'-54,36,10,10,10,10,10,10,10,10,10,10,      //pos 15
  #else
    10,'C'-54,'U'-54,'R'-54,38,'T'-54,'Y'-54,'P'-54,'E'-54,36,10,10,10,10,10,'N'-54,'O'-54,'N'-54,'E'-54,10,      
  #endif
  
  10,'R'-54,'O'-54,'L'-54,10,'S'-54,'E'-54,'N'-54,36,10,10,10,10,10,10,10,10,10,10,10,          //pos 15,16
  10,'P'-54,'I'-54,'T'-54,10,'S'-54,'E'-54,'N'-54,36,10,10,10,10,10,10,10,10,10,10,10,          //pos 15,16
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'C'-54,'O'-54,'N'-54,'F'-54,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  
  
  /////////////////////////////////////////////////////////////////////
  'P'-54,'I'-54,'D'-54,1,39,3,10,10,10,10,10,10,10,10,10,PID1/10,PID1%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'R'-54,'O'-54,'L'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'P'-54,'I'-54,'T'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'Y'-54,'A'-54,'W'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'A'-54,'L'-54,'T'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'P'-54,'O'-54,'S'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,'N'-54,'C'-54,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'P'-54,'I'-54,'D'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'P'-54,'I'-54,'D'-54,2,39,3,10,10,10,10,10,10,10,10,10,PID2/10,PID2%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'P'-54,'S'-54,'R'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'N'-54,'V'-54,'R'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'L'-54,'V'-54,'L'-54,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'M'-54,'A'-54,'G'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'V'-54,'E'-54,'L'-54,10,10,'N'-54,'C'-54,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'P'-54,'I'-54,'D'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'P'-54,'I'-54,'D'-54,3,39,3,10,10,10,10,10,10,10,10,10,PID3/10,PID3%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'R'-54,'T'-54,'E'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'E'-54,'X'-54,'P'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'R'-54,'P'-54,'R'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,         //roll pitch rate
  10,'Y'-54,'R'-54,'T'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,         //yaw rate
  10,'D'-54,'T'-54,'R'-54,10,10,10,10,10,10,10,'N'-54,'C'-54,10,10,10,10,'N'-54,'C'-54,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'P'-54,'I'-54,'D'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'M'-54,'O'-54,'D'-54,'E'-54,1,39,2,10,'A'-54,1,2,3,4,10,10,MODE1/10,MODE1%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'L'-54,'V'-54,'L'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'B'-54,'A'-54,'R'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'M'-54,'A'-54,'G'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'C'-54,'M'-54,'S'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'C'-54,'M'-54,'T'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'C'-54,'H'-54,'A'-54,'N'-54,'G'-54,'E'-54,10,'M'-54,'O'-54,'D'-54,'E'-54,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'M'-54,'O'-54,'D'-54,'E'-54,2,39,2,10,'A'-54,1,2,3,4,10,10,MODE2/10,MODE2%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'A'-54,'R'-54,'M'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'G'-54,'H'-54,'E'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'G'-54,'H'-54,'D'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'P'-54,'A'-54,'S'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'H'-54,'D'-54,'E'-54,10,68,10,69,10,68,10,69,10,68,10,69,10,68,10,69,
  10,'C'-54,'H'-54,'A'-54,'N'-54,'G'-54,'E'-54,10,'M'-54,'O'-54,'D'-54,'E'-54,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,  

  /////////////////////////////////////////////////////////////////////
  'S'-54,'E'-54,'N'-54,'S'-54,'O'-54,'R'-54,'S'-54,1,39,2,10,10,10,10,10,SENSOR2/10,SENSOR2%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  10,'A'-54,'C'-54,'C'-54,10,10,10,10,10,10,10,10,10,10,10,'C'-54,'A'-54,'L'-54,10,10,
  10,'M'-54,'A'-54,'G'-54,10,10,10,10,10,10,10,10,10,10,10,'C'-54,'A'-54,'L'-54,10,10,
  10,'G'-54,'P'-54,'S'-54,10,'H'-54,'O'-54,'M'-54,'E'-54,10,10,10,10,10,10,'S'-54,'E'-54,'T'-54,10,10,
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'A'-54,'L'-54,'L'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,

  /////////////////////////////////////////////////////////////////////
  'S'-54,'E'-54,'N'-54,'S'-54,'O'-54,'R'-54,'S'-54,2,39,2,10,10,10,10,10,SENSOR1/10,SENSOR1%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  'A'-54,'C'-54,'C'-54,39,'N'-54,'K'-54,36,10,68,10,10,10,69,10,10,68,10,10,10,69,
  'B'-54,'A'-54,'R'-54,'O'-54,10,10,36,10,68,10,10,10,69,10,10,68,10,10,10,69,
  'M'-54,'A'-54,'G'-54,10,10,10,36,10,68,10,10,10,69,10,10,68,10,10,10,69,
  'G'-54,'P'-54,'S'-54,10,10,10,36,10,68,10,10,10,69,10,10,68,10,10,10,69,
  'I'-54,2,'C'-54,10,'E'-54,'R'-54,'R'-54,36,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'A'-54,'L'-54,'L'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,

  /////////////////////////////////////////////////////////////////////
  'I'-54,'N'-54,'F'-54,'O'-54,10,10,10,10,10,10,10,10,10,10,10,INFO/10,INFO%10,39,MAX_MENU_PAGES/10,MAX_MENU_PAGES%10,
  'V'-54,'E'-54,'R'-54,'S'-54,10,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  'R'-54,'O'-54,'L'-54,'L'-54,10,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  'P'-54,'I'-54,'T'-54,'C'-54,'H'-54,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  'Y'-54,'A'-54,'W'-54,10,10,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  'T'-54,'H'-54,'R'-54,'O'-54,'T'-54,10,36,10,10,10,10,10,10,10,10,10,10,10,10,10,
  10,'S'-54,'A'-54,'V'-54,'E'-54,10,'A'-54,'L'-54,'L'-54,10,10,10,10,10,10,'E'-54,'X'-54,'I'-54,'T'-54,10,

  /////////////////////////////////////////////////////////////////////
  ///Result page
  /////////////////////////////////////////////////////////////////////
  'R'-54,'E'-54,'S'-54,'U'-54,'L'-54,'T'-54,10,10,10,10,10,10,10,10,10,10,'P'-54,'A'-54,'G'-54,'E'-54,
  'F'-54,'L'-54,38,'T'-54,'I'-54,'M'-54,'E'-54,10,36,10,10,10,36,10,10,10,10,10,10,10,
  'H'-54,'I'-54,38,'A'-54,'L'-54,'T'-54,10,10,36,10,10,10,10,10,10,10,'M'-54,10,10,10,
  'H'-54,'I'-54,38,'D'-54,'I'-54,'S'-54,'T'-54,10,36,10,10,10,10,10,10,10,'M'-54,10,10,10,
  'H'-54,'I'-54,38,'S'-54,'P'-54,'E'-54,'E'-54,'D'-54,36,10,10,10,10,10,10,10,'K'-54,'M'-54,39,'H'-54,
  'H'-54,'I'-54,38,'C'-54,'U'-54,'R'-54,'R'-54,10,36,10,10,10,38,10,10,10,'A'-54,10,10,10,
  'M'-54,'O'-54,'V'-54,'E'-54,10,'T'-54,'H'-54,'R'-54,'O'-54,'T'-54,'T'-54,'L'-54,'E'-54,10,10,10,10,10,10,10,
};

PROGMEM prog_uchar bigNumbers[] = {                          // 16 x 16 bitmap.

#ifdef HITECH_FONT
//0
0b00000000,0b00000000,
0b00000000,0b00000000,  
0b00011111,0b11111100,
0b00011111,0b11111100,
0b00011110,0b00111100,
0b00011110,0b00111100,
0b00011110,0b00111100,
0b01111110,0b00111100,
0b01111110,0b00111100,
0b01111110,0b00111100,
0b01111110,0b00111100,
0b01111110,0b00111100,
0b01111110,0b00111100,
0b01111111,0b11111100,
0b01111111,0b11111100,
0b00000000,0b00000000,

//1
0b00000000,0b00000000,
0b00000000,0b00000000,  
0b00000011,0b11000000,
0b00000011,0b11000000,
0b00000011,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00001111,0b11000000,
0b00000000,0b00000000,

//2
0b00000000,0b00000000,
0b00000000,0b00000000,  
0b11111111,0b11111000,
0b11111111,0b11111000,
0b11110000,0b01111000,
0b11110000,0b01111000,
0b00000000,0b01111000,
0b11111111,0b11111000,
0b11111111,0b11111000,
0b11111100,0b00000000,
0b11111100,0b00000000,
0b11111100,0b01111000,
0b11111100,0b01111000,
0b11111111,0b11111000,
0b11111111,0b11111000,
0b00000000,0b00000000,

//3
0b00000000,0b00000000,
0b00000000,0b00000000,
0b01111111,0b11110000,
0b01111111,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000111,0b11111100,
0b00000111,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b01111111,0b11111100,
0b01111111,0b11111100,
0b00000000,0b00000000,

//4
0b00000000,0b00000000,
0b00000000,0b00000000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111111,0b11111100,
0b01111111,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b00000000,

//5
0b00000000,0b00000000,
0b00000000,0b00000000,
0b11111111,0b11111000,
0b11111111,0b11111000,
0b11110000,0b01111000,
0b11110000,0b01111000,
0b11110000,0b00000000,
0b11111111,0b11111000,
0b11111111,0b11111000,
0b00000001,0b11111000,
0b00000001,0b11111000,
0b11110001,0b11111000,
0b11110001,0b11111000,
0b11111111,0b11111000,
0b11111111,0b11111000,
0b00000000,0b00000000,

//6
0b00000000,0b00000000,
0b00000000,0b00000000,
0b00111111,0b11111000,
0b00111111,0b11111000,
0b00111100,0b01111000,
0b00111100,0b01111000,
0b00111100,0b00000000,
0b11111111,0b11111000,
0b11111111,0b11111000,
0b11111100,0b01111000,
0b11111100,0b01111000,
0b11111100,0b01111000,
0b11111100,0b01111000,
0b11111111,0b11111000,
0b11111111,0b11111000,
0b00000000,0b00000000,

//7
0b00000000,0b00000000,
0b00000000,0b00000000,
0b01111111,0b11110000,
0b01111111,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b00000000,0b00000000,

// 8 start @256
0b00000000,0b00000000,
0b00000000,0b00000000,  
0b00111111,0b11110000,
0b00111111,0b11110000,
0b00111100,0b11110000,
0b00111100,0b11110000,
0b00111100,0b11110000,
0b01111111,0b11111100,
0b01111111,0b11111100,
0b01111100,0b00111100,
0b01111100,0b00111100,
0b01111100,0b00111100,
0b01111100,0b00111100,
0b01111111,0b11111100,
0b01111111,0b11111100,
0b00000000,0b00000000,

// 9 start @288
0b00000000,0b00000000,
0b00000000,0b00000000,
0b01111111,0b11110000,
0b01111111,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111111,0b11111100,
0b01111111,0b11111100,
0b00000000,0b11111100,
0b00000000,0b11111100,
0b01111000,0b11111100,
0b01111000,0b11111100,
0b01111111,0b11111100,
0b01111111,0b11111100,
0b00000000,0b00000000,

#else

//0
0b00000000,0b00000000,
0b00000000,0b00000000, 
0b00011111,0b11111000,
0b00111111,0b11111100,
0b00111100,0b00111100,
0b00111100,0b00111100,
0b00111100,0b00111100,
0b00111100,0b00111100,
0b00111100,0b00111100,
0b00111100,0b00111100,
0b00111100,0b00111100,
0b00111100,0b00111100,
0b00111100,0b00111100,
0b00111111,0b11111100,
0b00001111,0b11111000,
0b00000000,0b00000000,

//1
0b00000000,0b00000000,
0b00000000,0b00000000, 
0b00001111,0b10000000,
0b00011111,0b11000000,
0b00111111,0b11000000,
0b01111111,0b11000000,
0b00000111,0b11000000,
0b00000111,0b11000000,
0b00000111,0b11000000,
0b00000111,0b11000000,
0b00000111,0b11000000,
0b00000111,0b11000000,
0b00000111,0b11000000,
0b00000111,0b11000000,
0b00011111,0b11110000,
0b00000000,0b00000000,

//2
0b00000000,0b00000000,
0b00000000,0b00000000, 
0b11111111,0b11110000,
0b11111111,0b11111000,
0b00000000,0b01111000,
0b00000000,0b01111000,
0b00000000,0b01111000,
0b01111111,0b11111000,
0b11111111,0b11110000,
0b11110000,0b00000000,
0b11110000,0b00000000,
0b11110000,0b00000000,
0b11110000,0b00000000,
0b11111111,0b11111000,
0b01111111,0b11110000,
0b00000000,0b00000000,

//3
0b00000000,0b00000000,
0b00000000,0b00000000,
0b01111111,0b11100000,
0b01111111,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000111,0b11110000,
0b00000111,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b01111111,0b11110000,
0b01111111,0b11100000,
0b00000000,0b00000000,

//4
0b00000000,0b00000000,
0b00000000,0b00000000,
0b00111000,0b11100000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111111,0b11110000,
0b00111111,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b01100000,
0b00000000,0b00000000,

//5
0b00000000,0b00000000,
0b00000000,0b00000000,
0b01111111,0b11111000,
0b11111111,0b11111000,
0b11110000,0b00000000,
0b11110000,0b00000000,
0b11110000,0b00000000,
0b11111111,0b11110000,
0b01111111,0b11111000,
0b00000000,0b01111000,
0b00000000,0b01111000,
0b00000000,0b01111000,
0b11000000,0b01111000,
0b11111111,0b11111000,
0b01111111,0b11110000,
0b00000000,0b00000000,

//6
0b00000000,0b00000000,
0b00000000,0b00000000,
0b00011111,0b11110000,
0b00111111,0b11111000,
0b00111100,0b00000000,
0b00111100,0b00000000,
0b00111100,0b00000000,
0b00111111,0b11110000,
0b00111111,0b11111000,
0b00111100,0b01111000,
0b00111100,0b01111000,
0b00111100,0b01111000,
0b00111100,0b01111000,
0b00111111,0b11111000,
0b00011111,0b11110000,
0b00000000,0b00000000,

//7
0b00000000,0b00000000,
0b00000000,0b00000000,
0b01111111,0b11110000,
0b01111111,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000001,0b11100000,
0b00000011,0b11000000,
0b00000111,0b10000000,
0b00001111,0b00000000,
0b00001111,0b00000000,
0b00001111,0b00000000,
0b00001111,0b00000000,
0b00001111,0b00000000,
0b00000000,0b00000000,

// 8 start @256
0b00000000,0b00000000,
0b00000000,0b00000000, 
0b00111111,0b11100000,
0b01111111,0b11110000,
0b01110000,0b01110000,
0b01110000,0b01110000,
0b01110000,0b01110000,
0b01111111,0b11110000,
0b01111111,0b11110000,
0b01110000,0b01110000,
0b01110000,0b01110000,
0b01110000,0b01110000,
0b01110000,0b01110000,
0b01111111,0b11110000,
0b00111111,0b11100000,
0b00000000,0b00000000,

// 9 start @288
0b00000000,0b00000000,
0b00000000,0b00000000,
0b00111111,0b11100000,
0b01111111,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111000,0b11110000,
0b01111111,0b11110000,
0b00111111,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b00000000,0b11110000,
0b01111111,0b11110000,
0b00111111,0b11100000,
0b00000000,0b00000000,  
#endif
};

//HOME ARROWS
PROGMEM prog_uchar homeArrows [] = {
  //0 start @0
  0b00000000,0b00000000,
  0b00000001,0b10000000,
  0b00000011,0b11000000,  
  0b00000111,0b11100000,  
  0b00001111,0b11110000,  
  0b00011111,0b11111000,  
  0b00111111,0b11111100,  
  0b00111111,0b11111100,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000, 
  0b00000001,0b10000000,  

  //1 start @32
  0b00000000,0b00000000,  
  0b00000000,0b00000000,  
  0b00000111,0b11111110,  
  0b00000111,0b11111110,  
  0b00000011,0b11111110, 
  0b00000001,0b11111110,
  0b00000001,0b11111110,  
  0b00000011,0b11111110,  
  0b00000111,0b11111110,
  0b00001111,0b10001110,  
  0b00011111,0b00000110,  
  0b00111110,0b00000000,  
  0b01111100,0b00000000,
  0b01111000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,  

  //2 start @64
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b01100000,
  0b00000000,0b01110000,
  0b00000000,0b01111000,
  0b00000000,0b11111100,
  0b01111111,0b11111110,
  0b11111111,0b11111110,
  0b11111111,0b11111110,
  0b01111111,0b11111110,
  0b00000000,0b01111100,
  0b00000000,0b01111000,
  0b00000000,0b01110000,
  0b00000000,0b01100000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,

  //3 start @96
  0b00000000,0b00000000,
  0b00000000,0b00000000, 
  0b00000000,0b00000000, 
  0b01110000,0b00000000,  
  0b11111000,0b00000000,
  0b01111100,0b00000000,  
  0b00111110,0b00000110,  
  0b00011111,0b00001110, 
  0b00001111,0b10011110,
  0b00000111,0b11111110,  
  0b00000011,0b11111110,   
  0b00000001,0b11111110,    
  0b00000001,0b11111110,  
  0b00000011,0b11111110,   
  0b00000111,0b11111110, 
  0b00000111,0b11111110,

  //4 start @128
  0b00000000,0b00000000,
  0b00000001,0b10000000,
  0b00000011,0b11000000, 
  0b00000011,0b11000000, 
  0b00000011,0b11000000,  
  0b00000011,0b11000000,  
  0b00000011,0b11000000, 
  0b00000011,0b11000000,
  0b00000011,0b11000000,  
  0b00111111,0b11111100,  
  0b00111111,0b11111100,
  0b00011111,0b11111000,  
  0b00001111,0b11110000,  
  0b00000111,0b11100000,  
  0b00000011,0b11000000,  
  0b00000001,0b10000000,

  //5 start @160
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00111100,
  0b00000000,0b01111100,
  0b00000000,0b11111000,
  0b11000001,0b11110000,
  0b11100011,0b11100000,
  0b11111111,0b11000000,
  0b11111111,0b10000000,
  0b11111111,0b10000000,
  0b11111111,0b11100000,
  0b11111111,0b11110000,
  0b11111111,0b11110000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,

  //6 start @192
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000110,0b00000000,
  0b00001110,0b00000000,
  0b00011110,0b00000000,
  0b00111110,0b00000000,
  0b01111111,0b11111100,
  0b01111111,0b11111110,
  0b01111111,0b11111110,
  0b01111111,0b11111100,
  0b00111110,0b00000000,
  0b00011110,0b00000000,
  0b00001110,0b00000000,
  0b00000110,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,

  //7 start @224
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b01111111,0b11100000,
  0b01111111,0b11100000,
  0b01111111,0b11000000,
  0b01111111,0b10000000,
  0b01111111,0b10000000,
  0b01111111,0b11000000,
  0b01111111,0b11100000,
  0b01110001,0b11110000,
  0b01100000,0b11111100,
  0b00000000,0b01111110,
  0b00000000,0b00111110,
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000,
  0b00000000,0b00000000
};

#define CHARSIZE 7  //size of char 7 bytes

static uint8_t BARSIZE = 0;

volatile static uint8_t lettersInRam [] = {

  // 0 - pos. 0 start @0 
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // 1 - pos. 1 start @8
  0b00010000,
  0b00110000,
  0b01110000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b01111110,

  // 2 - pos. 2 start @16
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,

  // 3 - pos. 3 start @24
  0b01111110,
  0b00000010,
  0b00000010,
  0b00111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // 4 - pos. 4 start @32
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,
  0b00000010,
  0b00000010,
  0b00000010,

  // 5 - pos. 5 start @40
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // 6 - pos. 6 start @48
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // 7 - pos. 7 start @56
  0b01111110,
  0b01000010,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00100000,
  0b00100000,

  // 8 - pos. 8 start @64
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // 9 - pos. 9 start @72
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b00000010,
  0b00000010,
  0b00000010,

  // Blank - pos. 10 start @80   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  // A - pos. 11 start @88
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,  
  0b01000010,
  0b01000010,

  // B - pos. 12 start @96
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // C - pos. 13 start @104
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01111110,

  // D - pos. 14 start @112
  0b01111000,
  0b01000100,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000100,
  0b01111000,

  // E - pos. 15 start @120
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,

  // F - pos. 16 start @128
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,

  // G - pos. 17 start @136
  0b01111110,
  0b01000000,
  0b01000000,
  0b01001110,
  0b01000010,
  0b01000010,
  0b01111110,

  // H - pos. 18 start @144
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,

  // I - pos. 19 start @152
  0b01111110,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b01111110,

  // J - pos. 20 start @160
  0b00111110,
  0b00000010,
  0b00000010,
  0b00000010,
  0b01000010,
  0b01100110,
  0b00111100,
  
  //K - pos. 21 start @168
  0b01000110,
  0b01001000,
  0b01010000,
  0b01100000,
  0b01010000,
  0b01001000,
  0b01000110,

  // L - pos. 22 start @176
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01111110,

  // M - pos. 23 start @184
  0b01000010,
  0b01100110,
  0b01011010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,

  // N - pos. 24 start @192
  0b01000010,
  0b01100010,  
  0b01010010,
  0b01001010,
  0b01001010,
  0b01000110,
  0b01000010,
  
  // O - pos. 25 start @200
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // P - pos. 26 start @208
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,

  // Q - pos. 27 start @216
  0b01000010,
  0b01100110,
  0b01011010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,

  // R - pos. 28 start @224
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01100000,
  0b01011000,
  0b01000110,

  // S - pos. 29 start @232
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // T - pos. 30 start @240
  0b01111110,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,

  // U - pos. 31 start @248
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // V - pos. 32 start @256
  0b01000010,
  0b01000110,
  0b01000100,
  0b00100100,
  0b00101100,
  0b00011000,
  0b00011000,
}; 

PROGMEM prog_uchar letters[] = {                          // 8 x 8 bitmap.
  
  // 0 - pos. 0 start @0 
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // 1 - pos. 1 start @8
  0b00010000,
  0b00110000,
  0b01110000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b01111110,

  // 2 - pos. 2 start @16
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,

  // 3 - pos. 3 start @24
  0b01111110,
  0b00000010,
  0b00000010,
  0b00111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // 4 - pos. 4 start @32
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,
  0b00000010,
  0b00000010,
  0b00000010,

  // 5 - pos. 5 start @40
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // 6 - pos. 6 start @48
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // 7 - pos. 7 start @56
  0b01111110,
  0b01000010,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00100000,
  0b00100000,

  // 8 - pos. 8 start @64
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // 9 - pos. 9 start @72
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b00000010,
  0b00000010,
  0b00000010,

  // Blank - pos. 10 start @80   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  // A - pos. 11 start @88
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,  
  0b01000010,
  0b01000010,

  // B - pos. 12 start @96
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,

  // C - pos. 13 start @104
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01111110,

  // D - pos. 14 start @112
  0b01111000,
  0b01000100,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000100,
  0b01111000,

  // E - pos. 15 start @120
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,

  // F - pos. 16 start @128
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,

  // G - pos. 17 start @136
  0b01111110,
  0b01000000,
  0b01000000,
  0b01001110,
  0b01000010,
  0b01000010,
  0b01111110,

  // H - pos. 18 start @144
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,

  // I - pos. 19 start @152
  0b01111110,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b01111110,

  // J - pos. 20 start @160
  0b00111110,
  0b00000010,
  0b00000010,
  0b00000010,
  0b01000010,
  0b01100110,
  0b00111100,
  
  //K - pos. 21 start @168
  0b01000110,
  0b01001000,
  0b01010000,
  0b01100000,
  0b01010000,
  0b01001000,
  0b01000110,

  // L - pos. 22 start @176
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01000000,
  0b01111110,

  // M - pos. 23 start @184
  0b01000010,
  0b01100110,
  0b01011010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,

  // N - pos. 24 start @192
  0b01000010,
  0b01100010,  
  0b01010010,
  0b01001010,
  0b01001010,
  0b01000110,
  0b01000010,
  
  // O - pos. 25 start @200
  0b01111110,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // P - pos. 26 start @208
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01000000,
  0b01000000,
  0b01000000,

  // Q - pos. 27 start @216
  0b01000010,
  0b01100110,
  0b01011010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,

  // R - pos. 28 start @224
  0b01111110,
  0b01000010,
  0b01000010,
  0b01111110,
  0b01100000,
  0b01011000,
  0b01000110,

  // S - pos. 29 start @232
  0b01111110,
  0b01000000,
  0b01000000,
  0b01111110,
  0b00000010,
  0b00000010,
  0b01111110,

  // T - pos. 30 start @240
  0b01111110,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,
  0b00010000,

  // U - pos. 31 start @248
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01000010,
  0b01111110,

  // V - pos. 32 start @256
  0b01000010,
  0b01000110,
  0b01000100,
  0b00100100,
  0b00101100,
  0b00011000,
  0b00011000,
  
  // W - pos. 33 start @264
  0b10000010,
  0b10010010,
  0b10010010,
  0b10010010,
  0b10010010,
  0b10010010,
  0b11111110,

  // X - pos. 34 start @272
  0b01000010, 
  0b00100010, 
  0b00010100, 
  0b00001000, 
  0b00010100, 
  0b00100010, 
  0b01000010,

  // Y - pos. 35 start @280
  0b01000010, 
  0b00100010, 
  0b00010100, 
  0b00001000, 
  0b00001000, 
  0b00001000, 
  0b00001000,
  
  // : - pos. 36 start @288  
  0b00000000,
  0b00011100,
  0b00011100,
  0b00000000,
  0b00011100,
  0b00011100,
  0b00000000,

  // -  - pos. 37 start @296
  0b00000000,
  0b00000000,
  0b01111110,
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000, 

  // .  - pos. 38 start @304
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00111000,
  0b00111000,  

  // /  - pos. 39 start @312
  0b00000010,
  0b00000010,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00100000,
  0b01000000,

  // %  - pos. 40 start @320
  0b01000010,
  0b01000010,
  0b00000100,
  0b00001000,
  0b00010000,
  0b00100010,
  0b01000010,

  // <- - pos. 41 start @328
  0b00100000,
  0b01100000,
  0b11111111,
  0b01100000,
  0b00100000,
  0b00000000,
  0b00000000,

  // -> - pos. 42 start @336
  0b00001000,
  0b00001100,
  0b11111110,
  0b00001100,
  0b00001000,
  0b00000000,
  0b00000000,

  // ARROW UP pos. 43 start @344
  0b00110000, 
  0b01111000,
  0b11111100,
  0b00110000,
  0b00110000,
  0b00110000,
  0b00110000,
   
  // ARROW DOWN pos. 44 start @352
  0b00001000,
  0b00001000,
  0b00001000,
  0b00001000,
  0b00111110,
  0b00011100,
  0b00001000,

  //CLOCK - pos. 45 start @360
  0b00000000,
  0b01111100,
  0b10010010,
  0b10011110,
  0b10000010,
  0b01111100,
  0b00000000,

  //BAT - pos. 46 start @368
  0b00111100,
  0b01111110,
  0b01111110,
  0b01000010,
  0b01111110,
  0b01000010,
  0b01111110,

  //SIGNAL - pos. 47 start @376
  0b00000000,
  0b10010010,
  0b01010100,
  0b00111000,
  0b00010000,
  0b00010000,
  0b00010000,

  //SATELITE - pos. 48 start @384
  0b00000010,
  0b01111010,
  0b11110100,
  0b11101110,
  0b01111100,
  0b01111000,
  0b11111110,

  //point - pos. 49 start @392
  0b00000000,
  0b00000000,
  0b00000000,
  0b00111000,
  0b01111100,
  0b00111000,
  0b00000000,

  // c - pos. 50 start @400
  0b00000000,
  0b00000000,
  0b00000000,
  0b00111110,
  0b01100000,
  0b01100000,
  0b00111110,

  // m - pos. 51 start @408
  0b00000000,
  0b00000000,
  0b00000000,
  0b11111110,
  0b10010010,
  0b10010010,
  0b10010010,

  // RSSI1 - pos. 52   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,

  // RSSI2 - pos. 53   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,
  0b01111110,
  0b01111110,
  
  // RSSI3 - pos. 54   
  0b00000000,
  0b00000000,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  
  // RSSI4 - pos. 55   
  0b00000000,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  
  // RSSI5 - pos. 56   
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  0b01111110,
  
  //BATLEFT-FULL - pos. 57
  0b11111111,
  0b11111110,
  0b11111110,
  0b11111110,
  0b11111110,
  0b11111110,
  0b11111111,
  
  //BATCENTER-LOW - pos. 58  
  0b11111111,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b11111111,
  
  //BATRIGHT-FULL - pos. 59
  0b11111000,
  0b11111000,
  0b11111110,
  0b11111110,
  0b11111110,
  0b11111000,
  0b11111000,
   
  //BATRIGHT-LOW - pos. 60
  0b11111000,
  0b00001000,
  0b00001110,
  0b00001110,
  0b00001110,
  0b00001000,
  0b11111000,
  
  //BATLEFT-LOW - pos. 61
  0b11111111,
  0b10000000,
  0b10000000,
  0b10000000,
  0b10000000,
  0b10000000,
  0b11111111,
   
  // h  - pos. 62 start @496   
  0b00000000,
  0b10000000,
  0b10000000,
  0b11111100,
  0b10000010,
  0b10000010,
  0b10000010,

  // Compass - pos. 63 start @504   
  0b00000000,
  0b00000000,
  0b00000000,
  0b11111110,
  0b11111110,
  0b00000000,
  0b00000000,

  // Point - pos. 64 start @512    
  0b00011000,
  0b00111100,
  0b01111110,
  0b00000000,
  0b00000000, 
  0b00000000,
  0b00000000,
  
  //CURRENT - pos. 65 start @520
  0b00001000,
  0b00110000,
  0b01110000,
  0b11111110,
  0b00111100,
  0b00011000,
  0b00100000,
  
  //DEG - pos. 66 start @528
  0b00111000,
  0b00101000,
  0b00111000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  //HOME - pos. 67 start @536
  0b00010000,
  0b00111000,
  0b01111100,
  0b11111110,
  0b11000110,  
  0b11000110,
  0b11111110,
  
  // [ - pos. 68 start @544   
  0b11111110, 
  0b11000000,
  0b11000000,
  0b11000000,
  0b11000000,
  0b11000000,
  0b11111110,

  // ] - pos. 69 start @552   
  0b11111110, 
  0b00000110,
  0b00000110,
  0b00000110,
  0b00000110,
  0b00000110,
  0b11111110,

  // mode0 - pos. 70   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,

  // mode1 - pos. 71 start @568   
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,

  // mode2 - pos. 72 start @576   
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000,

  // mode3 - pos. 73 start @584   
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,
  0b00000000,
  0b00000000,
  0b01111110,

  // mode4 - pos. 74 start @592   
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  
  // mode5 - pos. 75 start @600   
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b00000000,
  0b01111110,
  
  // mode6 - pos. 76 start @608   
  0b01111110,
  0b00000000,
  0b00000000,
  0b01111110,
  0b00000000,
  0b00000000,
  0b00000000,
  
  // mode7 - pos. 77 start @616   
  0b01111110,
  0b00000000,
  0b00000000,
  0b01111110,
  0b00000000,
  0b00000000,
  0b01111110,  
};

PROGMEM prog_uchar smallNumbers[] = {                          // 4 x 6 bitmap.
//0
0b11100000,
0b10100000,
0b10100000,
0b10100000,
0b11100000,
0b00000000,
//1
0b01000000,
0b11000000,
0b01000000,
0b01000000,
0b11100000,
0b00000000,
//2
0b11100000,
0b00100000,
0b11100000,
0b10000000,
0b11100000,
0b00000000,
//3
0b11100000,
0b00100000,
0b11100000,
0b00100000,
0b11100000,
0b00000000,
//4
0b10100000,
0b10100000,
0b11100000,
0b00100000,
0b00100000,
0b00000000,
//5
0b11100000,
0b10000000,
0b11100000,
0b00100000,
0b11000000,
0b00000000,
//6
0b11000000,
0b10000000,
0b11100000,
0b10100000,
0b11100000,
0b00000000,
//7
0b11100000,
0b00100000,
0b01000000,
0b10000000,
0b10000000,
0b00000000,
//8
0b11100000,
0b10100000,
0b11100000,
0b10100000,
0b11100000,
0b00000000,
//9
0b11100000,
0b10100000,
0b11100000,
0b00100000,
0b01100000,
0b00000000,
};

//________________________________________________________________________________________________
ISR(INT0_vect)                                            // interrupt routine Hsync 
{                                                         
  if(screenNumber == 1){ // main Screen
  
    DDRB&=0xf7;
        
    switch(lineId){
     
     case 1:          //modes, numsat, rssi 
      
      DDRB|=0x08;                                           
      //on time
      delay1;                                                                               
      _delay_loop_1(11);
      //dimOn;  
      
      #ifdef MODE   
        for (uint16_t osdChar = FUNCTIONSBAR_START + letterCounter; osdChar < FUNCTIONSBAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;    
          _delay_loop_1(2);
        }
        dimOff;   
      #endif 
     
      #ifdef GPS   //gps numsat
        //gps satelites
        delayMicroseconds(GPS_NUMSAT_BAR_MARGIN);                                                                                  
        _delay_loop_1(2);
        //dimOn;
        for (uint16_t osdChar = NUMSATBAR_START + letterCounter; osdChar < NUMSATBAR_END; osdChar+=7)     
        {
            SPDR=VideoBuffer[osdChar];                                
            dimOn;
            _delay_loop_1(2);
        }
        dimOff;   
      #endif
      
      #ifdef RSSISIGNAL   
        #ifdef GPS 
        #else
            delayMicroseconds(RSSIBAR_PADDING);
            _delay_loop_1(2);
        #endif
           //rssi   
           //dimOn;
        for (uint16_t osdChar = RSSIBAR_START + letterCounter; osdChar < RSSIBAR_END; osdChar+=7)     
        {
            SPDR=VideoBuffer[osdChar];                                
            dimOn;
            _delay_loop_1(2);
        }
        dimOff;
      #endif  
    
      letterCounter ++;
      BARSIZE = 7;   
      break;
     
     case 2:   //start & fly time, bat1 & bat2  
      
      DDRB|=0x08;                                           
      delay1;
      _delay_loop_1(13);
      
      ////////////////////////////////////////////START TIME///////////////////////////////////////////////////////////
      //start time (minutes) - big numbers 
      for (uint16_t osdChar = STARTTIME_BIGNUMBAR_START + letterCounter; osdChar < STARTTIME_BIGNUMBAR_END; osdChar+=16)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        delay3;
      }
      dimOff;
      if(letterCounter < 7){
        delay3;
        // start time (seconds) - small
        for (uint16_t osdChar = TIMEBAR_START + letterCounter; osdChar < TIMEBAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }else if(letterCounter > 8){
        //start time title - small
        for (uint16_t osdChar = TIMEBAR_TITLE_START + (letterCounter - 9); osdChar < TIMEBAR_TITLE_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }else{
        _delay_loop_1(25);
      }
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      ////////////////////////////////////////////FLY TIME///////////////////////////////////////////////////////////
      //fly time (minutes) - big numbers 
      for (uint16_t osdChar = FLYTIME_BIGNUMBAR_START + letterCounter; osdChar < FLYTIME_BIGNUMBAR_END; osdChar+=16)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        delay3;
      }
      dimOff;
      if(letterCounter < 7){
        delay3;
        //fly time (seconds) - small
        for (uint16_t osdChar = FLYTIMEBAR_START + letterCounter; osdChar < FLYTIMEBAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;      
      }else if(letterCounter > 8){
      
        //fly time title - small
        for (uint16_t osdChar = FLYTIMEBAR_TITLE_START + (letterCounter - 9); osdChar < FLYTIMEBAR_TITLE_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }else{
        _delay_loop_1(25);
      }  
      
      #ifdef BAT1
      ////////////////////////////////////////////BAT1 VOLTAGE///////////////////////////////////////////////////////////
      //integer part - big numbers 
      for (uint16_t osdChar = BAT1_BIGNUMBAR_START + letterCounter; osdChar < BAT1_BIGNUMBAR_END; osdChar+=16)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        delay3;
      }
      dimOff;
      if(letterCounter < 7){  
        delay3;
        //decimal part - small numbers  
        for (uint16_t osdChar = BAT1VOLTAGEBAR_DECIMAL_START + letterCounter; osdChar < BAT1VOLTAGEBAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }  
        dimOff; 
      }else if(letterCounter > 8){
        //battery icon
        for (uint16_t osdChar = BAT1BAR_START + (letterCounter-9); osdChar < BAT1BAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }else{
        _delay_loop_1(26);
        delay1;
      }
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      #else
        delayMicroseconds(11);                                                                                  
        //_delay_loop_1(1);   
      #endif
      
      #ifdef BAT2
      ////////////////////////////////////////////BAT2 VOLTAGE///////////////////////////////////////////////////////////
      //integer part - big numbers 
      for (uint16_t osdChar = BAT2_BIGNUMBAR_START + letterCounter; osdChar < BAT2_BIGNUMBAR_END; osdChar+=16)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        delay3;
      }
      dimOff;
      if(letterCounter < 7){
        delay3;
        //decimal part - small numbers  
        for (uint16_t osdChar = BAT2VOLTAGEBAR_START + letterCounter; osdChar < BAT2VOLTAGEBAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }  
        dimOff; 
      }else if(letterCounter > 8){  
        //battery icon
        for (uint16_t osdChar = BAT2BAR_START + (letterCounter-9); osdChar < BAT2BAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }//else{
      //  _delay_loop_1(25);
      //}
      #endif
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      
      letterCounter  ++;
   
      BARSIZE = 16;   //16 lines size
      break;
     
     /*
     case 3:    
      
      letterCounter ++;
      BARSIZE = 7; 
      break;
     */
    
     case 4:   //batery 1 current 
      
      DDRB|=0x08;                                           
      //current   
      #ifdef CURRENT_SENSOR
      delay1;
      _delay_loop_1(13); 
      //dimOn;   
      for (uint16_t osdChar = CURRENTBAR_START + letterCounter; osdChar < CURRENTBAR_END; osdChar+=7)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        _delay_loop_1(2);
      }
      delay1;
      for (uint16_t osdChar = CURRENTVALUEBAR_START + letterCounter; osdChar < CURRENTVALUEBAR_END; osdChar+=7)     
      {
        SPDR=VideoBuffer[osdChar];                          
        clearVideoOut;    
        _delay_loop_1(2);
      } 
      dimOff;
      #endif
      
      #ifdef GPS
       #ifdef LAT_LON  
       
       _delay_loop_1(24);
       
       //latitude
       for (uint16_t osdChar = LATBAR_START + letterCounter; osdChar < LATBAR_END; osdChar+=7)     
       {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;    
        _delay_loop_1(2);
       }
       dimOff;
       
       _delay_loop_1(6);
       //longitude
       for (uint16_t osdChar = LONBAR_START + letterCounter; osdChar < LONBAR_END; osdChar+=7)     
       {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;    
        _delay_loop_1(2);
       }
       dimOff;   
       
       #endif
      #endif
      
      letterCounter ++;
      BARSIZE = 7; 
      break;
     
     case 5:    //batery 1 capacity consumed
     
      DDRB|=0x08;                                                                                                                       
      //capacity bar  
      #ifdef CURRENT_SENSOR
      _delay_loop_1(12);
      //dimOn;  
      for (uint16_t osdChar = CAPACITYBAR_START + letterCounter; osdChar < CAPACITYBAR_END; osdChar+=7)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        clearVideoOut;    
        _delay_loop_1(2);
      }
      //capacity value
      for (uint16_t osdChar = CAPACITYVALUEBAR_START + letterCounter; osdChar < CAPACITYVALUEBAR_END; osdChar+=7)     
      {
        SPDR=VideoBuffer[osdChar];                          
        clearVideoOut;    
        _delay_loop_1(2);
      }  
      dimOff;  
      #else
        delayMicroseconds(14);
        _delay_loop_1(2);
      #endif
      #ifdef COMPASS    
      
      _delay_loop_1(7);
      
      for (uint16_t osdChar = COMPASSBAR_START + letterCounter; osdChar < COMPASSBAR_END; osdChar+=7)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        _delay_loop_1(2);
      }
      dimOff;
      #endif
     
      letterCounter ++;
      BARSIZE = 7; 
      break;
      
     case 6:    //print point
      
      #ifdef COMPASS
    
      DDRB|=0x08;                                           
      delayMicroseconds(POINT_MARGIN);                                                                                   
      _delay_loop_1(5);
      SPDR = pgm_read_byte(&letters[letterCounter + 448]);                // write letter on pos 64 to data register
      //clearVideoOut;                                           
      #endif
      
      letterCounter ++;
      BARSIZE = 3; 
      break;
     
     case 7:    //heading
     
     #ifdef COMPASS
    
      DDRB|=0x08;                                           
      delayMicroseconds(HEADINGBAR_MARGIN);
      _delay_loop_1(5);      
      
      for (uint16_t osdChar = HEADINGBAR_START + letterCounter; osdChar < HEADINGBAR_END; osdChar+=7)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        clearVideoOut;    
        _delay_loop_1(2);
      }
      dimOff;
      #endif
      
      letterCounter ++;
      BARSIZE = 7;    
      break;
     
     case 9:   //print horizont
    
      lineCounter ++;

      if(lineCounter > 2){           //lines 3,6,9,12 ...

        printFlightPanel();
   
        if(lineCounter == 4){
           segmentCounter++;
           lineCounter = 0;
           mainPanelLineGenerated = 0;
        }           
      }
      else{ // 1-2 ,4-5 ,7-8 ....
        // 2 lines to compute next line for acc visualisation
        if(cryticalPart == 0){  // acc data was computed
          if(mainPanelLineGenerated == 0){
            //clearVideoOut;           
            printMainPanelToBuffer(MAIN_PANEL_CENTER_LINE,MAIN_PANEL_CENTER_POINT,readedRollAngle_temp);
            mainPanelLineGenerated = 1;
            
            if((segmentCounter + cmAlt_pointer + 1)%5 == 0)
              math1 = 1;
            else
              math1 = 0;
              
            if((segmentCounter + speed_pointer + 1)%5 == 0)
              math2 = 1;
            else
              math2 = 0;  
              
          }  
        }  
      }
      if(Hsync == FLIGHT_PANEL_END_LINE){
        letterCounter = 0;
        lineId = 0;
      }
      break; 
     
     case 10:   //altitude + speed + arrow + distance
      
      DDRB|=0x08;                                           
     
      _delay_loop_1(13);
      
      #ifdef ALTITUDE
      ////////////////////////////////////////////ALTITUDE ///////////////////////////////////////////////////////////
      //integer part - big numbers 
      for (uint16_t osdChar = ALTITUDE_BIGNUMBAR_START + letterCounter; osdChar < ALTITUDE_BIGNUMBAR_END; osdChar+=16)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        delay3;
      }
      dimOff;
      if(letterCounter < 7){  
        delay3;
        //altitude title  
        for (uint16_t osdChar = ALTITUDE_TITLE_START + letterCounter; osdChar < ALTITUDE_TITLE_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }  
        dimOff; 
      }else if(letterCounter > 8){
        //altitude units
        for (uint16_t osdChar = ALTITUDE_UNITS_START + (letterCounter-9); osdChar < ALTITUDE_UNITS_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }else{
        _delay_loop_1(26);
        delay1;
      }
      ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      #endif
      
      #ifdef GPS 
      
      ////////////////////////////////////////////GPS SPEED ///////////////////////////////////////////////////////////
      //integer part - big numbers 
      for (uint16_t osdChar = SPEED_BIGNUMBAR_START + letterCounter; osdChar < SPEED_BIGNUMBAR_END; osdChar+=16)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        delay3;
      }
      dimOff;
      
      if(letterCounter < 7){  
        delay3;
        //speed title  
        for (uint16_t osdChar = SPEED_TITLE_START + letterCounter; osdChar < SPEED_TITLE_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }  
        dimOff; 
      }else if(letterCounter > 8){
        //speed units
        for (uint16_t osdChar = SPEED_UNITS_START + (letterCounter-9); osdChar < SPEED_UNITS_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }else{
        _delay_loop_1(26);
        delay1;
      }
      
      //arrow
      for (uint16_t osdChar = ARROWBAR_START + letterCounter; osdChar < ARROWBAR_END; osdChar+=16)     
      {
        SPDR=VideoBuffer[osdChar];                          
        dimOn;
        delay3;
      }
      dimOff;
      
      if(letterCounter > 8){
        //distance to home
        for (uint16_t osdChar = GPSDISTANCEBAR_START + (letterCounter-9); osdChar < GPSDISTANCEBAR_END; osdChar+=7)     
        {  
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          _delay_loop_1(2);
        }
        dimOff;
      }
      #endif
      
      letterCounter ++; 
      BARSIZE = 16; 
      break; 
     
     /*case 11:      
     case 12:*/    
     
     case 13:    //processing
      
      fastUpdate();
      slowUpdate();     
      lineId = 0;
      
      if(synchronization == 1)
        synchronization = 2;
      else{
        uint8_t rqTmp = dataRequest[requestId % STANDARD_DATA_REQUEST_SIZE];
        dataCount = 0;   
        sendDataRequest(rqTmp, 0, 0);                          
      } 
      
      EIMSK |= ( 0 << INT0);      //disable H-sync interrupt
      break;
    
     default: break;  
    }   
  }else if(screenNumber == 0){          //print Header

    DDRB&=0xf7;
    //header   
    if ((Hsync >= TOP_MARGIN) && (Hsync <= TOP_MARGIN + 6) && a20mSecTimer >= 60)                   
    {
      DDRB|=0x08;                                           
      delayMicroseconds(8);                                  
      //dimOn;                                             
      for (int osdChar=varOSD; osdChar < HEADER_SIZE; osdChar+=7)     
      {
        SPDR=VideoBuffer[osdChar];                          
        //clearVideoOut;    
        _delay_loop_1(2);
      }
      //dimOff;                                               
      varOSD++;                                              
    }
    
    if(Hsync == TOP_MARGIN + 30){   
       
       viewTime ++;
       if(viewTime >= 500){ //10 sec timer   
          viewTime = 0;
          screen = 1;         // main screen
       }          
    }     
  }else if(screenNumber == 2){   //Radar screen

    DDRB&=0xf7;  
      
    if ((Hsync >= GRAPHIC_POS_START) && (Hsync < GRAPHIC_POS_END)){                   
      
      DDRB|=0x08;                                           
      
      g_temp = GRAPHIC_SIZE_18*letterCounter1;
      g_temp1 = GRAPHIC_SIZE_18*(letterCounter1+1);
      
      #if (VIDEO_TYPE == NTSC)
         dimOn;
      #endif  
      for (uint16_t x = g_temp ; x < g_temp1; x++)     
      {
        SPDR=VideoBuffer[x];                                
        #if (VIDEO_TYPE == NTSC)
          delay2;      
        #else//PAL
          dimOn; 
          //delay2; 
        #endif
      }
      clearVideoOut;
      dimOff;
      
      #ifdef BAT1
      
      #if (VIDEO_TYPE == NTSC)
        delayMicroseconds(15); //NTSC      
      #else//PAL
        delayMicroseconds(17);
      #endif
      
      if(letterCounter1 < 7){
        for (uint16_t osdChar = BAT1BAR_START + letterCounter1; osdChar < BAT1BAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          dimOn;
          clearVideoOut;
          _delay_loop_1(2);              
        }
        //bat1 voltage  
        for (uint16_t osdChar = BAT1VOLTAGEBAR_START + letterCounter1; osdChar < BAT1VOLTAGEBAR_END; osdChar+=7)     
        {
          SPDR=VideoBuffer[osdChar];                          
          clearVideoOut;
          _delay_loop_1(2);     
        }  
        dimOff;
      }
      #endif
     
      letterCounter1 ++;        
    
    }else if(Hsync == GRAPHIC_POS_END){
      fastUpdate();           //time generator calculate startup time in all screens   
          
      if(synchronization == 1)
        synchronization = 2;
      else{
        uint8_t rqTmp = dataRequest[requestId % STANDARD_DATA_REQUEST_SIZE];
        dataCount = 0;   
        sendDataRequest(rqTmp, 0, 0);
      }
        
      EIMSK |= ( 0 << INT0); 
    } 
  }
  else if(screenNumber == 3){  // Saved screen
  
    if(buffered == 0){   
      clearVideoBuffer(VIDEO_BUFFER_SIZE);

      buffering(0, 10, 'S' - 54);
      buffering(0, 11, 'A' - 54);
      buffering(0, 12, 'V' - 54);
      buffering(0, 13, 'E' - 54);
      buffering(0, 14, 'D' - 54);
      
      buffered = 1;
    }
    
    if (Hsync == TOP_MARGIN - 1){
      varOSD = 0;
    }else if ((Hsync >= TOP_MARGIN) && (Hsync <= TOP_MARGIN + 6 )){                         
      DDRB|=0x08;                                           
      delayMicroseconds(7);                                  
      //dimOn;                                             
      for (int osdChar=varOSD; osdChar < CONTENT_END; osdChar+=7)    
      {
        SPDR=VideoBuffer[osdChar];                          
        //clearVideoOut;    
        _delay_loop_1(2);
      }
      //dimOff;                                           
      varOSD++;                                             
    }else if(Hsync == TOP_MARGIN + 9){
      fastUpdate();           //time generator calculate startup time in all screens
      viewTime ++;             
      if(viewTime == 100){   // 2 sec timer      
        
        viewTime = 0;
        
        x_pos = 0;
        y_pos = 1;
        y_pos_old = 0;

        yleft = 0;
        yright = 0;
      
        configDataReaded = 0;
        screen = 1;        //jump to main screen
       
        if(pageNumber <= INTERNAL_SETTINGS2){
           writeEEPROM(EEPROM_DATA_LENGTH);//write EEPROM DATA to OSD 
           return;
        }  
        
        //set PID////////////////////////////////////////////
        sendDataRequest(MSP_SET_PID, 1, PID_DATA_BYTES);
        //RC Tuning /////////////////////////////////////////       
        sendDataRequest(MSP_SET_RC_TUNING, 2, RCTUNING_ITEMS); 
        //BOX Items /////////////////////////////////////////
        sendDataRequest(MSP_SET_BOX, 3, BOXITEMS*2);
        //Write MWC EEPROM
        sendDataRequest(MSP_EEPROM_WRITE,0,0);
                             
        if(pageNumber < SENSOR2)
           return;
        
        //write OSD EEPROM    
        writeEEPROM(EEPROM_DATA_LENGTH);            
      }                
    }   
  }
  else if(screenNumber == 6){  // config menu

    DDRB&=0xf7;  

    if(Hsync == 0){
      if(armed){           //if motor armed go to main screen
        screen = 4;       
      }else if(pageNumber == MAX_MENU_PAGES + 1){ //if result page check change of throttle value and go to main screen
        if(rcData[THROTTLE] > OUT_THROTTLE){
           pageNumber = 1;
           screen = 1;
           layoutWrited = 0;
        }
      }        
    }

    if(Hsync == 1){   //change attribute value

      //atribute adjusting 
      if(x_pos != 0){
          if(y_pos == MAX_MENU_ITEMS){
             if(down >= eepromBuffer[RC_SENS]){      
                down = 0;
                screen = 4;             //go to cancel screen
                return;
             }
          }else{
            //pid config 
            if(pageNumber >= PID1 && pageNumber <= PID3){ 
              uint8_t pos;
              
              if(pageNumber == PID1)
                 pos = y_pos - 2;
              else if(pageNumber == PID2)  
                 pos = y_pos + 3;
              else //PID3   
                 pos = y_pos + 8;
                 
              if(up >= eepromBuffer[RC_SENS]){      
               
                 pidBuffer[pos*3 + x_pos] ++;
                 up = 0;
              
              }else if(down >= eepromBuffer[RC_SENS]){
                 if(pidBuffer[pos*3 + x_pos] > 0)
                    pidBuffer[pos*3 + x_pos] --;                 
                 down = 0;    
              }
            }else if(pageNumber >= MODE1 && pageNumber <= MODE2){
              uint8_t pos;
              if(pageNumber == MODE1)    
                pos = y_pos - 2;
              else
                pos = y_pos + 3;  

              uint8_t set;
              uint16_t set_dump;
              
              set_dump = activate[pos] & aux_map[x_pos - 1];     //0b00000000 | 00111000
              set = (uint8_t) (set_dump >> aux_shift[x_pos - 1]);
                  
              if(up >= eepromBuffer[RC_SENS]){      
                
                  if(set < 7){
                    activate[pos] -= set_dump;
                    set ++;
                    activate[pos] |= (uint16_t) (set << aux_shift[x_pos - 1]);
                  }    
                  up = 0;
              }else if(down >= eepromBuffer[RC_SENS]){
              
                  if(set > 0){
                    activate[pos] -= set_dump;
                    set --;
                    activate[pos] |= (uint16_t) (set << aux_shift[x_pos - 1]);
                  }
                  down = 0;
              }                
            }else{    
               switch(y_pos){             
                   case 2: //BAT1
                           if(up >= eepromBuffer[RC_SENS]){      
                             #ifdef BAT1 
                              if(pageNumber == INTERNAL_SETTINGS2){
                                eepromBuffer[BAT1_CORRECTION] ++;
                              }else{
                                eepromBuffer[BAT1_LEVEL] ++;  
                              }
                             #endif 
                             up = 0;   
                           }else if(down >= eepromBuffer[RC_SENS]){      
                              if(pageNumber == SENSOR2){   //ACC calib
                                 
                                 sendDataRequest(MSP_ACC_CALIBRATION, 0, 0);
                                  
                                 bufferingInMenu(y_pos - 1,15,'O'-54);
                                 bufferingInMenu(y_pos - 1,16,'K'-54);
                                 bufferingInMenu(y_pos - 1,17,10);
                              }
                              #ifdef BAT1
                               else if(pageNumber == INTERNAL_SETTINGS2){
                                 eepromBuffer[BAT1_CORRECTION] --;
                               }else{ //BAT1 set
                                 if(eepromBuffer[BAT1_LEVEL] > 0)
                                    eepromBuffer[BAT1_LEVEL] --;
                               }
                              #endif   
                              down = 0;
                           }
                           break;
                   case 3: //BAT2 
                           if(up >= eepromBuffer[RC_SENS]){      
                             #ifdef BAT2 
                              if(pageNumber == INTERNAL_SETTINGS2){
                                eepromBuffer[BAT2_CORRECTION] ++;
                              }else{
                                eepromBuffer[BAT2_LEVEL] ++;  
                              }  
                             #endif
                             up = 0;   
                           }else if(down >= eepromBuffer[RC_SENS]){      
                              if(pageNumber == SENSOR2){   //MAG calib
                                 
                                 sendDataRequest(MSP_MAG_CALIBRATION, 0, 0);
                                 
                                 bufferingInMenu(y_pos - 1,15,'O'-54);
                                 bufferingInMenu(y_pos - 1,16,'K'-54);
                                 bufferingInMenu(y_pos - 1,17,10);
                              }
                              
                              #ifdef BAT2
                               else if(pageNumber == INTERNAL_SETTINGS2){
                                 if(sensVcc > 0)     //bat2 voltage
                                   eepromBuffer[BAT2_CORRECTION] --;
                               }else{ //BAT2 set
                                 if(eepromBuffer[BAT2_LEVEL] > 0)
                                    eepromBuffer[BAT2_LEVEL] --;
                               }
                              #endif   
                              down = 0;
                           } 
                           break;
                   case 4: //RSSI or Current sensor type                        
                           if(up >= eepromBuffer[RC_SENS]){
                              #ifdef CURRENT_SENSOR
                               if(pageNumber == INTERNAL_SETTINGS2){
                                 if(eepromBuffer[CURR_SENS_TYPE] < NUMBER_OF_CURRENT_SENSORS)
                                    eepromBuffer[CURR_SENS_TYPE] ++;
                               }
                              #endif
                              up = 0;  
                           }else if(down >= eepromBuffer[RC_SENS]){
              
                              if(pageNumber == SENSOR2){ //GPS Home set                 
                                 //UDR0 = 'G';
                                 bufferingInMenu(y_pos - 1,15,'O'-54);
                                 bufferingInMenu(y_pos - 1,16,'K'-54);
                                 bufferingInMenu(y_pos - 1,17,10);
                              }else if(pageNumber == INTERNAL_SETTINGS2){
                                 #ifdef CURRENT_SENSOR
                                  if(eepromBuffer[CURR_SENS_TYPE] > 1)
                                     eepromBuffer[CURR_SENS_TYPE] --;
                                 #endif                   
                              }else{ //RSSI
                                 if(sensRSSI > 10){
                                    eepromBuffer[RSSI_CALIB] = ((int)sensRSSI) & 0x00FF;
                                    eepromBuffer[RSSI_CALIB + 1] = (((int)sensRSSI) & 0xFF00) >> 8;
                                    maxRSSI = (int) sensRSSI;
                                    bufferingInMenu(y_pos - 1,15,'O'-54);
                                    bufferingInMenu(y_pos - 1,16,'K'-54);
                                    bufferingInMenu(y_pos - 1,17,10);
                                 }
                              }
                              down = 0; 
                           }  
                           break;
                   case 5: //AUX switcher or ROLL sensitivity 
                           if(up >= eepromBuffer[RC_SENS]){
                              if(pageNumber == INTERNAL_SETTINGS2){
                                 eepromBuffer[ROLL_SENSITIVITY] ++;
                              }else{
                                if(eepromBuffer[AUX_SW] < 4)
                                   eepromBuffer[AUX_SW] ++;                   
                              }
                              up = 0;
                           }else if(down >= eepromBuffer[RC_SENS]){
                              if(pageNumber == INTERNAL_SETTINGS2){
                                 if(eepromBuffer[ROLL_SENSITIVITY] >1) 
                                    eepromBuffer[ROLL_SENSITIVITY] --;
                              }else if(eepromBuffer[AUX_SW] > 0){
                                 eepromBuffer[AUX_SW] --;
                              }
                              down = 0;
                           }
                           break;
                   case 6: //RC SENSITIVITY    1 - 5
                           if(up >= eepromBuffer[RC_SENS]){
                              if(pageNumber == INTERNAL_SETTINGS2){
                                 eepromBuffer[PITCH_SENSITIVITY] ++;
                              }else{
                                 if(eepromBuffer[RC_SENS] < 5)
                                    eepromBuffer[RC_SENS] ++;
                              }
                              up = 0;    
                           }else if(down >= eepromBuffer[RC_SENS]){ 
                              if(pageNumber == INTERNAL_SETTINGS2){
                                 if(eepromBuffer[PITCH_SENSITIVITY] >1) 
                                    eepromBuffer[PITCH_SENSITIVITY] --;
                              }else if(eepromBuffer[RC_SENS] > 1){
                                 eepromBuffer[RC_SENS] --; 
                              }
                              down = 0;                    
                           }
                           break;  
                   default: 
                           break;
               }     
            }
            
            //mode config      
          }
                
      }      
    }

    // printig lines on screen //////////////
    if (Hsync == MENU_START_LINE - 11){
      lineCounter = 1;
      varOSD = 0;   
      temp1 = ONE_LINE_IN_MENU_SIZE;  
    }

    //1 line - page number
    if ((Hsync >= MENU_START_LINE) && (Hsync <= MENU_START_LINE + 6 ))                         
    {
      printOneLineInMenu(0);

      if(Hsync == MENU_START_LINE + 6){
        varOSD = temp1;
        temp1 = (++lineCounter) * ONE_LINE_IN_MENU_SIZE;
      }   
    }

    //horizontal line
    if (Hsync == MENU_START_LINE + 15){
      DDRB|=0x08;
      delayMicroseconds(LEFT_MARGIN_IN_MENU + 1);
      dimOn;
      for (int i = 0 ; i <= ONE_LINE_IN_MENU + 1 ; i++)     
      {       
        if(i == ONE_LINE_IN_MENU+1){   
          SPDR = 0b11111110;
          break;
        }
        else{ 
          SPDR = 0b11111111;
          delay10;
        }               
      }
      dimOff;
    }

    //2 line
    if ((Hsync >= MENU_START_LINE + 22) && (Hsync <= MENU_START_LINE + 28 ))                         
    {
      printOneLineInMenu(1);

      if(Hsync == MENU_START_LINE + 28){
        varOSD = temp1;
        temp1 = (++lineCounter) * ONE_LINE_IN_MENU_SIZE;
      }
    }

    //3 line
    if ((Hsync >= MENU_START_LINE + 33 ) && (Hsync <= MENU_START_LINE + 39 ))                         
    {
      printOneLineInMenu(2);

      if(Hsync == MENU_START_LINE + 39){
        varOSD = temp1;
        temp1 = (++lineCounter) * ONE_LINE_IN_MENU_SIZE;
      }   
    }

    //4 line
    if ((Hsync >= MENU_START_LINE + 44) && (Hsync <= MENU_START_LINE + 50 ))                         
    {
      printOneLineInMenu(3);

      if(Hsync == MENU_START_LINE + 50){
        varOSD = temp1;
        temp1 = (++lineCounter) * ONE_LINE_IN_MENU_SIZE;
      }   
    }

    //5 line
    if ((Hsync >= MENU_START_LINE + 55) && (Hsync <= MENU_START_LINE + 61 ))                         
    {
      printOneLineInMenu(4);

      if(Hsync == MENU_START_LINE + 61){
        varOSD = temp1;
        temp1 = (++lineCounter) * ONE_LINE_IN_MENU_SIZE;
      }   
    }

    //6 line
    if ((Hsync >= MENU_START_LINE + 66) && (Hsync <= MENU_START_LINE + 72 ))                         
    {
      printOneLineInMenu(5);

      if(Hsync == MENU_START_LINE + 72){
        varOSD = temp1;
        temp1 = (++lineCounter) * ONE_LINE_IN_MENU_SIZE;
      }   
    }

    //7 line - save, return
    if ((Hsync >= MENU_START_LINE + 81) && (Hsync <= MENU_START_LINE + 87 ))                         
    {
      printOneLineInMenu(6);
    }

    if(Hsync == MENU_START_LINE + 89){  //line arrow pointer   
      if(y_pos != y_pos_old){     
        if(y_pos == 1){   //if page is selected
          bufferingInMenu(y_pos - 1,14,42);    //print arrow on 16 pos
          if(pageNumber <= LAST_CONFIG_PAGE)
            bufferingInMenu(y_pos,0,10);      //print blank char on 0 pos
          else//if arrow only on first and last line - position on first line - > null pointer on last line
          bufferingInMenu(MAX_MENU_ITEMS - 1,0,10);   
        }else{ 
          if(y_pos == 2)
            bufferingInMenu(y_pos - 2,14,10);
          else if(pageNumber <= LAST_CONFIG_PAGE)
            bufferingInMenu(y_pos - 2,0,10);
          else  // position on last line - > null pointer on first line  
          bufferingInMenu(0,14,10);

          bufferingInMenu(y_pos - 1,0,42);

          if(y_pos < 7){
            if(y_pos == 4 && pageNumber == SENSOR2)
              bufferingInMenu(MAX_MENU_ITEMS - 1,0,10);      
            else 
              bufferingInMenu(y_pos,0,10);
          }
          else if(pageNumber == SENSOR2)  // last line
            bufferingInMenu(3 ,0,10);             
        }   

        y_pos_old = y_pos;
      }  
    }

    if(Hsync == MENU_START_LINE + 90){ //add attribute values

      if(pageNumber == INTERNAL_SETTINGS1){
       #ifdef BAT1                            
        //bat1 
        bufferingInMenu(1, 15, eepromBuffer[BAT1_LEVEL]/100);                                   
        bufferingInMenu(1, 16, eepromBuffer[BAT1_LEVEL]/10 - (eepromBuffer[BAT1_LEVEL]/100) * 10);
        bufferingInMenu(1, 18, eepromBuffer[BAT1_LEVEL] - (eepromBuffer[BAT1_LEVEL]/10) * 10); 
       #endif
       
       #ifdef BAT2                       
        //bat2
        bufferingInMenu(2, 15, eepromBuffer[BAT2_LEVEL]/100);                                   
        bufferingInMenu(2, 16, eepromBuffer[BAT2_LEVEL]/10 - (eepromBuffer[BAT2_LEVEL]/100) * 10);
        bufferingInMenu(2, 18, eepromBuffer[BAT2_LEVEL] - (eepromBuffer[BAT2_LEVEL]/10) * 10); 
       #endif
       
        //AUX switcher
        bufferingInMenu(4, 18, eepromBuffer[AUX_SW]);  
        
        //rc sensitivity
        bufferingInMenu(5, 15, eepromBuffer[RC_SENS]);    
      
      }else if(pageNumber == INTERNAL_SETTINGS2){
       #ifdef BAT1                    
        //bat1 
        bufferingInMenu(1, 15, bat1_voltage/100);                                   
        bufferingInMenu(1, 16, bat1_voltage/10 - (bat1_voltage/100) * 10);
        bufferingInMenu(1, 18, bat1_voltage - (bat1_voltage/10) * 10); 
       #endif
       
       #ifdef BAT2                           
        uint8_t tempV = sensVcc;                              // remove decimal part and convert to int
    
        bufferingInMenu(2,15,tempV/10); 
        bufferingInMenu(2,16,tempV%10);
        bufferingInMenu(2,18,((sensVcc*10)-(tempV*10)));
       #endif 
       
       //Current sensor selector
       #ifdef CURRENT_SENSOR
        bufferingInMenu(3, 15, eepromBuffer[CURR_SENS_TYPE]);  
       #endif   
       
       //Roll sensitivity
       bufferingInMenu(4, 15, eepromBuffer[ROLL_SENSITIVITY]/10);
       bufferingInMenu(4, 16, eepromBuffer[ROLL_SENSITIVITY]%10);
       
       //Pitch sensitivity
       bufferingInMenu(5, 15, eepromBuffer[PITCH_SENSITIVITY]/10);
       bufferingInMenu(5, 16, eepromBuffer[PITCH_SENSITIVITY]%10);
      
      }else if(pageNumber == MAX_MENU_PAGES + 1){
         
         //Fly time
         bufferingInMenu(1,10,MinTime_dump/10);
         bufferingInMenu(1,11,MinTime_dump%10);
         bufferingInMenu(1,13,SecTime_dump/10);
         bufferingInMenu(1,14,SecTime_dump%10);
       
         //high alt
         bufferingInMenu(2, 10, highAltitude / 10000); 
         bufferingInMenu(2, 11, highAltitude / 1000 - (highAltitude/10000) * 10); 
         bufferingInMenu(2, 12, highAltitude / 100 - (highAltitude/1000)*10);               
       
         //high distance
         bufferingInMenu(3, 10, highDistance / 10000);
         bufferingInMenu(3, 11, highDistance / 1000 - (highDistance/10000) * 10);
         bufferingInMenu(3, 12, highDistance / 100 - (highDistance/1000) * 10);
         bufferingInMenu(3, 13, highDistance / 10 - (highDistance/100) * 10);
         bufferingInMenu(3, 14, highDistance  -   (highDistance/10) * 10);
       
         //high speed
         bufferingInMenu(4, 10, highSpeed / 100); 
         bufferingInMenu(4, 11, highSpeed / 10 - (highSpeed/100) * 10); 
         bufferingInMenu(4, 12, highSpeed - (highSpeed/10)*10);               
       
         //high current
         bufferingInMenu(5, 10 , highAmperage / 1000);
         bufferingInMenu(5, 11 , highAmperage / 100 - (highAmperage/1000) * 10);
         bufferingInMenu(5, 13 , highAmperage / 10 - (highAmperage/100) * 10);
  
         
      }else if(pageNumber >= PID1 && pageNumber <= PID3){
        uint8_t pos;
        uint8_t tpos[] = {6, 11, 17};
        
        uint8_t fid; 
        for(uint8_t i = 1; i < MAX_MENU_ITEMS - 1 ; i++){
         switch(pageNumber){
      
          case PID1:
                    pos = i - 1;    
                    break;
          case PID2:
                    pos = i + 4;
                    break;
          case PID3:
                    pos = i + 9;
                    break;
         }
         
         for(uint8_t j = 1; j <= pidData_format[pos]; j++){ 
          
          fid = j; 
          
          //PID1 page   //abnormal values
          if(pos < 5){ 
             if(pos == POS){
                switch(j){
                
                    case 1:
                           fid = 4; 
                           break;
                    case 2:
                           fid = 1;             
                           break;
                    case 3:
                           break;          
                } 
             }   
          }  
          //PID2 page
          else if (pos < 10){
             if(pos >= POSR && pos <= NAVR){
                switch(j){
                
                    case 1:
                           fid = 1; 
                           break;
                    case 2:
                           fid = 4;                    
                           break;
                    case 3:
                           fid = 3; 
                           break;          
                } 
                
             }  
          }
          //PID3 page 
          else{ 
             fid = 4;
          }  
          //number format id
          switch(fid){
            //00.0
            case 1:  
                  bufferingInMenu(i, tpos[j-1], pidBuffer[(pos*3) + j]/100);                                   
                  bufferingInMenu(i, tpos[j-1] + 1, pidBuffer[(pos*3) + j]/10 - (pidBuffer[(pos*3) + j]/100) * 10);
                  bufferingInMenu(i, tpos[j-1] + 2, 38);      
                  bufferingInMenu(i, tpos[j-1] + 3, pidBuffer[(pos*3) + j] - (pidBuffer[(pos*3) + j]/10) * 10);
                  break;
            //0.000
            case 2:  
                  bufferingInMenu(i, tpos[j-1], 0);
                  bufferingInMenu(i, tpos[j-1] + 1, 38);                                   
                  bufferingInMenu(i, tpos[j-1] + 2, pidBuffer[(pos*3) + j]/100);
                  bufferingInMenu(i, tpos[j-1] + 3, pidBuffer[(pos*3) + j]/10 - (pidBuffer[(pos*3) + j]/100) * 10);
                  bufferingInMenu(i, tpos[j-1] + 4, pidBuffer[(pos*3) + j] - (pidBuffer[(pos*3) + j]/10) * 10);
                  break;
            //000
            case 3:  
                  bufferingInMenu(i, tpos[j-1], pidBuffer[(pos*3) + j]/100);                                   
                  bufferingInMenu(i, tpos[j-1] + 1, pidBuffer[(pos*3) + j]/10 - (pidBuffer[(pos*3) + j]/100) * 10);     
                  bufferingInMenu(i, tpos[j-1] + 2, pidBuffer[(pos*3) + j] - (pidBuffer[(pos*3) + j]/10) * 10);  
                  break;
    
            //0.00
            case 4:  
                  bufferingInMenu(i, tpos[j-1], pidBuffer[(pos*3) + j]/100);
                  bufferingInMenu(i, tpos[j-1] + 1, 38);                                   
                  bufferingInMenu(i, tpos[j-1] + 2, pidBuffer[(pos*3) + j]/10 - (pidBuffer[(pos*3) + j]/100) * 10);
                  bufferingInMenu(i, tpos[j-1] + 3, pidBuffer[(pos*3) + j] - (pidBuffer[(pos*3) + j]/10) * 10);
                  break;
                        
          } 
         }      
        }          
      }else if(pageNumber >= MODE1 && pageNumber <= MODE2){
        uint8_t pos;  
        uint16_t set;      
        uint8_t setTemp = 0;
        for(int i = 1; i < MAX_MENU_ITEMS - 1; i++){   

          if(pageNumber == MODE1)
             pos = i - 1;
          else //MODE2 
             pos = i + 4;  
          
          for(uint8_t j = 0; j < 4 ; j ++){
              set = activate[pos] & aux_map[j];       
              setTemp = (uint8_t) (set >> aux_shift[j]);
              bufferingInMenu(i, 6 + (j*4), setTemp + 70);
          }             
        }          
      }    
    }

    if(Hsync == MENU_START_LINE + 91){   //line positioning   
      if(x_pos == 0){   
        if(up >= eepromBuffer[RC_SENS]){   //pitch rubber in up position >= 320ms 
          if(y_pos > 1){
            y_pos --;

            if(pageNumber == SENSOR2) //only 2 items
              if(y_pos > 4)
                y_pos = 4;


            if(pageNumber > LAST_CONFIG_PAGE) // if page only with page selection and save,return
              y_pos = 1;            
          }     

          up = 0;

        }else if(down >= eepromBuffer[RC_SENS]){  //pitch rubber in down position >= 320ms
          if(y_pos < MAX_MENU_ITEMS)
            y_pos ++;
          else{ // save data and go to main screen
            screen = 3;  
            buffered = 0;
          }
          down = 0;
        }   
      }

      if(pageNumber > LAST_CONFIG_PAGE){ // if page only with page selection and save,return 
        if(y_pos > 1)
          y_pos = MAX_MENU_ITEMS;    // jump to save&return line
      }
      else if(pageNumber == SENSOR2){
        if(y_pos > 4)
          y_pos = MAX_MENU_ITEMS;
      }      
    } 
    
    if(Hsync == MENU_START_LINE + 92){      //positioning between columns
      if(y_pos != 1){     
        if(left >= eepromBuffer[RC_SENS]){     //roll rubber in left pos >= 320 ms
          if(x_pos > 0){
            x_pos --;                          // arrow go to left
            if(pageNumber >= PID1 && pageNumber <= PID3){  // 4 columns

              switch(x_pos){
              
                case 0:
                        bufferingInMenu(y_pos - 1,5,10);
                        bufferingInMenu(y_pos - 1,0,42);
                        break;
                case 1:
                        bufferingInMenu(y_pos - 1,10,10);
                        bufferingInMenu(y_pos - 1,5 ,42);
                        break;
                
                case 2: 
                        bufferingInMenu(y_pos - 1,16,10);
                        bufferingInMenu(y_pos - 1,10,42);
                        break;
              }
              
            }else if(pageNumber >= MODE1 && pageNumber <= MODE2){  // 5 columns
              
              bufferingInMenu(y_pos - 1, (x_pos+1)*4,10);
              bufferingInMenu(y_pos - 1, x_pos*4,42);              
              
              /*
              if(x_pos == 3){
                bufferingInMenu(y_pos - 1,16,10);
                bufferingInMenu(y_pos - 1,12,42);
              }
              else if(x_pos == 2){
                bufferingInMenu(y_pos - 1,12,10);
                bufferingInMenu(y_pos - 1,8 ,42);      
              }
              else if(x_pos == 1){
                bufferingInMenu(y_pos - 1,8,10);
                bufferingInMenu(y_pos - 1,4,42);      
              }
              else if(x_pos == 0){
                bufferingInMenu(y_pos - 1,4,10);
                bufferingInMenu(y_pos - 1,0,42);      
              }*/

            }else{   //2 columns
              bufferingInMenu(y_pos - 1,14,10); 
              bufferingInMenu(y_pos - 1,0,42);
            }        
          }
          left = 0;
        }else if(right >= eepromBuffer[RC_SENS]){    //roll rubber in right pos >= 320 ms

          if(y_pos == MAX_MENU_ITEMS){   // save,exit line
              if(x_pos < 1){
                x_pos ++;                 
                bufferingInMenu(y_pos - 1,0,10);   // go to column 2
                bufferingInMenu(y_pos - 1,14,42);  
              }
          }else if(pageNumber >= PID1 && pageNumber <= PID3){  // 4 columns           
            uint8_t pageOffset;
            switch(pageNumber){
      
               case PID1:
                    pageOffset = 0;    
                    break;
               case PID2:
                    pageOffset = 5;
                    break;
               case PID3:
                    pageOffset = 10;
                    break;
            }
            
            if(x_pos < pidData_format[(y_pos - 2) + pageOffset]){
              x_pos ++;
              switch(x_pos){
              
                case 1:
                        bufferingInMenu(y_pos - 1,5, 42);
                        bufferingInMenu(y_pos - 1,0, 10);
                        break;
                case 2:
                        bufferingInMenu(y_pos - 1,10,42);
                        bufferingInMenu(y_pos - 1,5 ,10);
                        break;
                
                case 3: 
                        bufferingInMenu(y_pos - 1,16,42);
                        bufferingInMenu(y_pos - 1,10,10);
                        break;
              }  
            } 
          }else if(pageNumber >= MODE1 && pageNumber <= MODE2){  // 5 columns           
            
            if(x_pos < 4){
              x_pos ++;
              
              bufferingInMenu(y_pos - 1, x_pos   *4,42);
              bufferingInMenu(y_pos - 1,(x_pos-1)*4,10);              
              
              /*
              if(x_pos == 4){
                bufferingInMenu(y_pos - 1,16,42);
                bufferingInMenu(y_pos - 1,12,10);
              }
              else if(x_pos == 3){
                bufferingInMenu(y_pos - 1,12,42);
                bufferingInMenu(y_pos - 1,8 ,10);      
              }
              else if(x_pos == 2){
                bufferingInMenu(y_pos - 1,8,42);
                bufferingInMenu(y_pos - 1,4,10);      
              }
              else if(x_pos == 1){
                bufferingInMenu(y_pos - 1,4,42);
                bufferingInMenu(y_pos - 1,0,10);      
              }*/
            } 
          }else{   //2 columns 
            if(x_pos < 1){
              x_pos ++;                 
              bufferingInMenu(y_pos - 1,0,10);   // go to column 2
              bufferingInMenu(y_pos - 1,14,42);  
            }
          }  
          right = 0;
        }   
      }else{                     //switch page
        if(left >= eepromBuffer[RC_SENS]){
          if(pageNumber > 1)
            pageNumber --;

          left = 0;
        }else if(right >= eepromBuffer[RC_SENS]){
          if(pageNumber < MAX_MENU_PAGES)
            pageNumber ++;

          right = 0;
        }   
      }    
    }
    
    if(Hsync == MENU_START_LINE + 93){

      fastUpdate();            //time generator calculate startup time in all screens  
    }      
     
    if(Hsync == MENU_START_LINE + 94){
      slowUpdate();
      
      if(synchronization == 1)
         synchronization = 2;
      else{  
        uint8_t rqTmp = dataRequest[(requestId % CONFIG_DATA_MODULO) + CONFIG_DATA_REQUEST_START];
        dataCount = 0;     
        sendDataRequest(rqTmp, 0, 0);                                                      
      }  
      
      EIMSK |= ( 0 << INT0);
    }   
  }else{   
   if(screenNumber == 4){   //cancel
      if(Hsync == 1){        
        readEEPROM(EEPROM_DATA_LENGTH);
        
        x_pos = 0;
        y_pos = 1;
        y_pos_old = 0;

        yleft = 0;
        yright = 0;
       
        configDataReaded = 0;
        screen = 1;        //jump to main screen
      }  
   }else if(screenNumber == 5){  //blank screen
      
      if(Hsync == TOP_MARGIN + 9){
        fastUpdate();           //time generator calculate startup time in all screens   
          
        if(synchronization == 1)
          synchronization = 2;
        else{
          uint8_t rqTmp = dataRequest[requestId % STANDARD_DATA_REQUEST_SIZE];
          dataCount = 0;   
          sendDataRequest(rqTmp, 0, 0);
        }
        
        EIMSK |= ( 0 << INT0); 
      }      
   }
     
   //new screens here  
  }

  Hsync++; 
  
  if(screenNumber != 1)
     return;  
  else if(letterCounter >= BARSIZE){
     letterCounter = 0;
     lineId = 0;
  }else if(lineId > 0)
     return;
 
  switch(Hsync){ 
  //page lines
    case RSSIBAR_Y_POS:     
       lineId = 1; break;  
    case TIMEBATBAR_Y_POS:  
       lineId = 2; break;    
    /*case BATBAR_Y_POS :  //to 6
       lineId = 3; break;*/  
    case CURBAR_Y_POS:  //to 15                   
       lineId = 4; break; 
    case (CURBAR_Y_POS + 9):   //to 24                   
       lineId = 5; break;
    case COMPASSBAR_Y_POS:   // to 2
       lineId = 6; break;
    case (COMPASSBAR_Y_POS + 4): //to 10    //line 87 to 93                     
       lineId = 7; break;
    case FLIGHT_PANEL_START_LINE:                   
       lineId = 9; break;
    default:    
       if (Hsync == CAMERA_V_RESOLUTION12 - 51)  //242 to 248 (standard pal)                        
           lineId = 10; 
       /*else if (Hsync == CAMERA_V_RESOLUTION12 - 44)  //249 to 250                         
           lineId = 11;
       else if (Hsync == CAMERA_V_RESOLUTION12 - 42)  //251 to 257                         
           lineId = 12;*/
       else if (Hsync == CAMERA_V_RESOLUTION12 - 35)  //258 to end of frame
           lineId = 13;    
    break;    
  }  
}

//________________________________________________________________________________________________
ISR(INT1_vect, ISR_BLOCK)                               
{                                                       
  if(videoResDetected == 0){
   //CAMERA_V_RESOLUTION12_OLD = CAMERA_V_RESOLUTION12;
   CAMERA_V_RESOLUTION12 = Hsync;
   //if(CAMERA_V_RESOLUTION12 >= 160 && (CAMERA_V_RESOLUTION12 == CAMERA_V_RESOLUTION12_OLD)){ //to protect line detection before uncomplete or incorrect video frame
   if(screen != 0){
      videoResDetected = 1;
   
      FLIGHT_PANEL_END_LINE = CAMERA_V_RESOLUTION12 - 59;
      uint8_t FLIGHT_PANEL_NUMBER_OF_LINES = FLIGHT_PANEL_END_LINE - FLIGHT_PANEL_START_LINE;
      uint8_t NUMBER_OF_VERTICAL_LINES = FLIGHT_PANEL_NUMBER_OF_LINES - 3;
      NUMBER_OF_VERTICAL_LINES /= 4;
      MAIN_PANEL_CENTER_LINE = NUMBER_OF_VERTICAL_LINES/2;
      CLMR =  MAIN_PANEL_CENTER_LINE - MAIN_PANEL_RADIUS;
      CLPR =  MAIN_PANEL_CENTER_LINE + MAIN_PANEL_RADIUS;
      CLMR2 = MAIN_PANEL_CENTER_LINE - 3; 
   }   
  }   
  
  Hsync = 0;                                              
  varOSD = 0;
  lineId = 0;
  letterCounter = 0;
  letterCounter1 = 0;
  lineCounter = 0;
  mainPanelLineGenerated = 0;
  segmentCounter = 0;
  pointYpos = 0;
  angleType_count = 0; 
     
  pointXpos = startPos_temp;

  switch(screen){             //switch screen

  case 0: 
    screenNumber = 0;   //intro title
    PrintHeaderToBuffer();
    layoutWrited = 0;
    readEnable = 0;
    break;
  case 1: 
    screenNumber = 1;   //main screen
    if(layoutWrited == 0){
      PrintLayoutToBuffer();
      pageNumber_old = 0;
      layoutWrited = 1;
    }
    readEnable = 1;    
    break;    
  case 2: 
    screenNumber = 2;   //radar screen      
    layoutWrited = 0;
    readEnable = 1;
    break;    
  case 3: 
    screenNumber = 3;   //saved title
    layoutWrited = 0;           
    readEnable = 0;
    break; 
  case 4: 
    screenNumber = 4;   //canceled title
    layoutWrited = 0;
    readEnable = 0;
    break;
  case 5: 
    screenNumber = 5;   //blank screen
    readEnable = 1;
    layoutWrited = 0;
    break;
  case 6: 
    screenNumber = 6;   //internal settings
    if(pageNumber != pageNumber_old){
      PrintMenuPageToBuffer(pageNumber);
      pageNumber_old = pageNumber;
      if(pageNumber == MAX_MENU_PAGES + 1){
        y_pos = y_pos_old;
      }else{
        y_pos = 1;
        y_pos_old = 0;
      }
    }
    readEnable = 1;  
    break;    
  }

  a20mSecTimer ++;
  EIMSK |= ( 1 << INT0);
}

//________________________________________________________________________________________________
void setup()                                            
{
  //Init Serial communication. 
  //Serial.begin(BAUD);
  //set baudrate

  UBRR0H = (unsigned char) (BAUD_SETTINGS>>8);
  UBRR0L = (unsigned char) (BAUD_SETTINGS);

  /* enable 2x speed mode U2X0 = 1*/
  UCSR0A = 0b0000010;
  /* interrupts disabled, rx and tx enabled*/
  UCSR0B = (1<<RXEN0) | (1<<TXEN0);
  /* Asynchronous mode, no parity, 1-stop bit, 8-bit data (8N1) */
  UCSR0C = (3<<UCSZ00);

  setupPinMode();                                        
  readEEPROM(EEPROM_DATA_LENGTH);
  
  for(uint16_t x = VIDEO_BUFFER_SIZE; x < VIDEO_BUFFER_SIZE + EXTENDED_BUFFER; x++)
      VideoBuffer[x] = 0x00;
  
  PORTB=0x0c;                                           // SS+MOSI, SS(slave select) 
  DDRB|=0x04;                                           // SS is active low, but with 1 

  //faster analog read - set prescale to 16
  ADCSRA |= _BV(ADPS2) ; 
  ADCSRA &= ~_BV(ADPS1); 
  ADCSRA &= ~_BV(ADPS0);

  // Setup SPI: Serial Peripheral Interface
  // SPCR: SPiControlRegister 
  // | 7    | 6    | 5    | 4    | 3    | 2    | 1    | 0    |
  // | SPIE | SPE  | DORD | MSTR | CPOL | CPHA | SPR1 | SPR0 |
  // Default value = 0;
  SPCR = (1<<SPE)  |                                    // SPE  - Enables the SPI when 1
  (0<<DORD) |                                    // DORD - When zero, the MSB of the data word is transmitted first.
  (1<<MSTR) |                                    // MSTR - Sets the Arduino in master mode when 1
  (1<<CPHA) |                                    // CPHA - Samples data on the falling edge of the data clock when 1
  (1<<CPOL) |                                     
    (0<<SPR1) |                                    // SPR1 SPR0 - Clock Rate Select
  (0<<SPR0) ;                                    // SPR1 and SPR0 - Sets the SPI speed, 00 is fastest (4MHz).
  SPSR = (1<<SPI2X);                                    // Double Speed (CK/2) Master SPI Mode (8MHz)
  EIMSK |= ( 1 << INT0);                                // Enable INT0 interrupt (EIMSK External Interrupt Mask Register) Hsync
  EIMSK |= ( 1 << INT1);                                // Enable INT1 interrupt (EIMSK External Interrupt Mask Register) Vsync
  EICRA |= ( 1 << ISC01);                               // EICRA External Interrupt Control Register A
  EICRA |= ( 1 << ISC00);                               // INT0 (Hsync); INT1 (Vsync) 
  EICRA |= ( 1 << ISC11);                               
  EICRA |= ( 0 << ISC10);                               // 00: LOW | 11: RISING | 01: ANY CHANGE | 10: FALLING
  clearVideoOut;                                        // disable all timers irq
  TIMSK0=0;                                              
  TIMSK1=0;                                             
  TIMSK2=0;                                               
  set_sleep_mode(SLEEP_MODE_IDLE);                      // energy saving  
}

void loop()                                             
{   
  while(1){  //read serial data from UART DATA register no interrupt     
   clearVideoOut;
   if(readEnable == 1){
    if(synchronization == 2){
      processSerialData();  
      requestId ++;
      synchronization = 0;
    }else if (UCSR0A & (1<<RXC0)) {
      RS232_TX_Buffer[dataCount] = UDR0;        
      dataCount++;   
      if(RS232_TX_Buffer[3] + HAF_RESPONSE_SIZE == dataCount){   //end of frame
        synchronization = 1; 
      }      
    }    
   }    
   sleep_enable();
   sleep_mode();
  }
}

///////////////////////////////////
// R o u t i n e s 
///////////////////////////////////

void setupPinMode()                                     
{
  pinMode(2, INPUT);                                    
  pinMode(3, INPUT);                                    
  pinMode(8, OUTPUT);                                   
  pinMode(9, OUTPUT);                                   
  pinMode(13,OUTPUT);                                   
  pinMode(14,INPUT);                                    
  pinMode(15,INPUT);                                    
  pinMode(16,INPUT);                                    
  pinMode(17,INPUT);                                    
}

//fill spaces to videoBuffer
static void clearVideoBuffer(uint16_t endByte)                                          
{
  for(uint16_t x = 0 ; x < endByte ; x++)
      VideoBuffer[x] = 0x00;
}

static void PrintLayoutToBuffer()                                          
{
  uint16_t poc = 0;

  for (uint16_t pos = 0; pos < LAYOUT; pos ++)                
  {                                                       
    uint16_t charId = pgm_read_byte(&layout[pos]);
    charId = charId * 7;
    
    for (uint16_t x = charId; x < charId + 7; x++) 
    {
      VideoBuffer[poc] = pgm_read_byte(&letters[x]);         
      poc++;                                              
    }
  }
  
  //reset bat1 voltage
  buffering2BigNumbers(BAT1_BIGNUMBAR_START,0,0);
  buffering2BigNumbers(SPEED_BIGNUMBAR_START,0,0);
  buffering3BigNumbers(ALTITUDE_BIGNUMBAR_START,0,0,0);  
}

static void PrintHeaderToBuffer()                                          
{
  uint16_t poc = 0;

  for (uint16_t pos = 0; pos < HEADER; pos ++)                
  {                                                       
    uint16_t charId = pgm_read_byte(&header[pos]);
    charId = charId*7;
    for (uint16_t x = charId; x < charId + 7; x++) 
    {         
      VideoBuffer[poc] = pgm_read_byte(&letters[x]);         
      poc++;                                              
    }       
  }
}

static void PrintMenuPageToBuffer(uint8_t pageNumber)                                          
{
  uint16_t poc = 0;

  if(pageNumber < 1)
    pageNumber = 1;

  uint16_t temp = ONE_LINE_IN_MENU * MAX_MENU_ITEMS;

  for (uint16_t pos = temp * (pageNumber - 1); pos < temp * pageNumber; pos++)
  {                                                       
    uint16_t charId = pgm_read_byte(&menuPages[pos]);
    charId = charId*7;
    for (uint16_t x = charId; x < charId + 7; x++) 
    {
      VideoBuffer[poc] = pgm_read_byte(&letters[x]);         
      poc++;                                              
    }     
  }
}

/*
static void bufferingBigNumbers(uint16_t barPointer, uint8_t * array, uint8_t count)                                          
{     
  uint16_t temp; 
  
  for(uint8_t i = 0; i<count ;i++){
    temp = array[i] * 32;
    for(uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31){            //refactor number (first column then second column) and write to video buffer
      VideoBuffer[barPointer + count] = pgm_read_byte(&bigNumbers[temp + j]);      
    }
  }
}*/

//lower memory consumation
static void bufferingArrow(uint16_t arrowPointer, uint8_t arrowType)                                          
{                      
  for(uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31){              //refactoring (first column then second column) and write to video buffer
      VideoBuffer[arrowPointer + count] = pgm_read_byte(&homeArrows[arrowType + j]);      
  }
}

//lower memory consumation
static void buffering2BigNumbers(uint16_t barPointer, uint8_t id1, uint8_t id2)                                          
{     
  uint16_t temp = id1 * 32; 
  uint16_t temp1;
  
  for(uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31){            //refactor number (first column then second column) and write to video buffer
      VideoBuffer[barPointer + count] = pgm_read_byte(&bigNumbers[temp + j]);      
  }
  
  temp = id2 * 32; 
  temp1 = barPointer + 32;
 
  for(uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31){            //refactor number (first column then second column) and write to video buffer
      VideoBuffer[temp1 + count] = pgm_read_byte(&bigNumbers[temp + j]);    
  }
}

//lower memory consumation
static void buffering3BigNumbers(uint16_t barPointer, uint8_t id1, uint8_t id2, uint8_t id3)                                          
{     
  uint16_t temp = id1 * 32; 
  uint16_t temp1;
  
  for(uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31){            //refactor number (first column then second column) and write to video buffer
      VideoBuffer[barPointer + count] = pgm_read_byte(&bigNumbers[temp + j]);      
  }
  
  temp = id2 * 32; 
  temp1 = barPointer + 32;
 
  for(uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31){            //refactor number (first column then second column) and write to video buffer
      VideoBuffer[temp1 + count] = pgm_read_byte(&bigNumbers[temp + j]);    
  }
  
  temp = id3 * 32; 
  temp1 = barPointer + 64;
 
  for(uint8_t j = 0, count = 0; count < 32; count++, j += 2, j %= 31){            //refactor number (first column then second column) and write to video buffer
      VideoBuffer[temp1 + count] = pgm_read_byte(&bigNumbers[temp + j]);    
  }
}

static void buffering(uint16_t barPointer, uint8_t pos, uint8_t id)                                          
{     
  //start in 0 line
  uint16_t posTemp = barPointer + (pos * 7);  // positioning in video buffer
  uint16_t idTemp = id * 7;
  
  if(id <= 32){  
    for (uint16_t x = posTemp; x < posTemp + 7; x++){
      VideoBuffer[x] = lettersInRam[idTemp];
      idTemp++;     
    }
  }else{
    for (uint16_t x = posTemp; x < posTemp + 7; x++){
      VideoBuffer[x] = pgm_read_byte (&letters[idTemp]);
      idTemp++;      
    }
  }
}

/*
static void smartBuffering(uint16_t barPointer, uint8_t pos, uint8_t * dataArray, uint8_t count)                                          
{     
  
 for(uint8_t i = 0; i < count ;i++){
  //start in 0 line
  uint16_t posTemp = barPointer + ((pos + i) * 7);  // start position in video buffer
  uint8_t id = dataArray[i];
  uint16_t idTemp =  id * 7;
 
  for (uint16_t x = posTemp; x < posTemp + 7; x++){
    VideoBuffer[x] = pgm_read_byte (&letters[idTemp]);
    idTemp++;      
  }
 } 
}*/

static void bufferingInMenu(uint8_t whoLine, uint8_t pos, uint8_t id)                                          
{     
  //start in 0 line
  uint16_t posTemp = (pos * 7) + (whoLine * ONE_LINE_IN_MENU_SIZE);  // positioning in video buffer
  uint16_t idTemp = id * 7;
  
  if(id <= 32){  
    for (uint16_t x = posTemp; x < posTemp + 7; x++){
      VideoBuffer[x] = lettersInRam[idTemp];
      idTemp++;     
    }
  }else{
    for (uint16_t x = posTemp; x < posTemp + 7; x++){
      VideoBuffer[x] = pgm_read_byte (&letters[idTemp]);
      idTemp++;      
    }
  }  
}

static void printOneLineInMenu(uint8_t delayMicros){

  DDRB|=0x08;                                           
  delayMicroseconds(LEFT_MARGIN_IN_MENU - delayMicros);                                  
  dimOn;
  for (uint16_t osdChar=varOSD; osdChar < temp1; osdChar+=7)    
  {
    SPDR=VideoBuffer[osdChar];                             
    clearVideoOut;    
    _delay_loop_1(2);
  }
  dimOff; 
  varOSD++;                                             
}

void writeEEPROM(uint8_t length ){

  for(uint8_t i=1;i<length;i++){  
    EEPROM.write(i,eepromBuffer[i]);
  }

  EEPROM.write(0,VERSION);
}

void readEEPROM(uint8_t length ){

  if(EEPROM.read(0) == VERSION){  //correct eeprom data
    for(uint8_t i=1;i<length;i++){  
      eepromBuffer[i] = EEPROM.read(i);
    }
    maxRSSI = (eepromBuffer[RSSI_CALIB + 1] <<8 ) | eepromBuffer[RSSI_CALIB]; 
  }else{
   
    //default values
    eepromBuffer[BAT1_LEVEL]        = DEFAULT_BAT1_CRITICAL_VOLTAGE;    
    eepromBuffer[BAT2_LEVEL]        = DEFAULT_BAT2_CRITICAL_VOLTAGE;
    eepromBuffer[RSSI_CALIB]        = maxRSSI & 0x00FF;
    eepromBuffer[RSSI_CALIB + 1]    = (maxRSSI & 0xFF00) >> 8;
    eepromBuffer[AUX_SW]            = DEFAULT_AUX_SWITCH;
    eepromBuffer[RC_SENS]           = DEFAULT_RC_SENS;
    eepromBuffer[BAT1_CORRECTION]   = DEFAULT_BAT1_CORRECTION;
    eepromBuffer[BAT2_CORRECTION]   = DEFAULT_BAT2_CORRECTION;
    eepromBuffer[CURR_SENS_TYPE]    = DEFAULT_CURRENT_TYPE;
    eepromBuffer[ROLL_SENSITIVITY]  = DEFAULT_ROLL_SENSITIVITY;
    eepromBuffer[PITCH_SENSITIVITY] = DEFAULT_PITCH_SENSITIVITY;
  }  
}

//________________________________________________________________________________________________

uint32_t read32(uint8_t id) {
  uint32_t t = read16(id);
  t+= (uint32_t)read16(id + 2)<<16;
  return t;
}
uint16_t read16(uint8_t id) {
  uint16_t t = read8(id);
  t+= (uint16_t)read8(id + 1)<<8;
  return t;
}
uint8_t read8(uint8_t id)  {
  return RS232_TX_Buffer[id]&0xFF;
}

//send request to MWC
static void sendDataRequest(uint8_t request, uint8_t dataId, uint8_t fSize){

    uint8_t checksum;
    
    //header
    for(uint8_t i = 0; i < 3 ; i++){
        while ( !( UCSR0A & (1<<UDRE0)) );                  // Wait for empty transmit buffer
        UDR0 = mwcRequestHeader[i];
    }
    
    //size    
    while ( !( UCSR0A & (1<<UDRE0)) );
    UDR0 = fSize;
    checksum ^= fSize;
    
    //request id    
    while ( !( UCSR0A & (1<<UDRE0)) );                  // Wait for empty transmit buffer
    UDR0 = request;
    checksum ^= request;
    
    //data     
    if(dataId != 0){
      
      if(dataId == 3)
         fSize/=2; 
                    
      for(uint8_t j = 0; j < fSize; j++){
        while ( !( UCSR0A & (1<<UDRE0)) );                  // Wait for empty transmit buffer  
        switch (dataId){
            case 1: //PID 
                    UDR0 = pidBuffer[j+1];
                    checksum ^= pidBuffer[j+1];
                    break;
            case 2: //RC TUNNING
                    UDR0 = pidBuffer[((j*3) +1) + PID_DATA_BYTES];
                    checksum ^= pidBuffer[((j*3) +1) + PID_DATA_BYTES];
                    break;
            case 3: //BOX
                    UDR0 = (uint8_t) (activate[j] & 0xFF);
                    checksum ^= (uint8_t) (activate[j] & 0xFF);
                    while ( !( UCSR0A & (1<<UDRE0)) );                  // Wait for empty transmit buffer  
                    UDR0 = (uint8_t) ((activate[j] >> 8) & 0xFF);
                    checksum ^= (uint8_t) ((activate[j] >> 8) & 0xFF);                 
                    break;        
        }          
      }
    }
    //checksum
    while ( !( UCSR0A & (1<<UDRE0)) );                  // Wait for empty transmit buffer
    UDR0 = checksum;   
}

//serial data processing
void processSerialData(){
                                                                                                
  switch(RS232_TX_Buffer[4]){   //frame id
  
     case MSP_RAW_GPS:
                   
                  #ifdef GPS
                   if(screenNumber != 1)
                      return;
                   
                   //GPS fix
                   gpsFix = RS232_TX_Buffer[5];
                   
                   //GPS numSat
                   buffering(NUMSATBAR_START, 1, RS232_TX_Buffer[6] / 10);              
                   buffering(NUMSATBAR_START, 2, RS232_TX_Buffer[6] % 10);

                   if(RS232_TX_Buffer[6] / 10 == 0)
                      buffering(NUMSATBAR_START, 1, 10);
    
                   uint8_t part1;
                   uint32_t part2;
                   int32_t GPS_coord;
                   
                   //Gps Latitude
                   GPS_coord = read32(7);
                   GPS_coord = abs(GPS_coord);

                   part1 = GPS_coord / 10000000;
                   part2 = (GPS_coord - (part1*10000000));

                   buffering(LATBAR_START, 0, part1 / 100);
                   buffering(LATBAR_START, 1, part1 / 10 - (part1 / 100)* 10);
                   buffering(LATBAR_START, 2, part1 - (part1 / 10) * 10);

                   buffering(LATBAR_START, 4, part2 / 1000000);
                   buffering(LATBAR_START, 5, part2 / 100000 - (part2/1000000) * 10);
                   buffering(LATBAR_START, 6, part2 / 10000 - (part2/100000) * 10);
                   buffering(LATBAR_START, 7, part2 / 1000 - (part2/10000) * 10);
                   buffering(LATBAR_START, 8, part2 / 100 - (part2/1000) * 10);                  
                   
                   //Gps Longitude
                   GPS_coord = read32(11);
                   GPS_coord = abs(GPS_coord);

                   part1 = GPS_coord / 10000000;
                   part2 = (GPS_coord - (part1*10000000));

                   buffering(LONBAR_START, 0, part1 / 100);
                   buffering(LONBAR_START, 1, part1 / 10 - (part1 / 100) * 10);
                   buffering(LONBAR_START, 2, part1 - (part1 / 10) * 10);

                   buffering(LONBAR_START, 4, part2 / 1000000);
                   buffering(LONBAR_START, 5, part2 / 100000 - (part2/1000000) * 10);
                   buffering(LONBAR_START, 6, part2 / 10000 - (part2/100000) * 10);
                   buffering(LONBAR_START, 7, part2 / 1000 - (part2/10000) * 10);
                   buffering(LONBAR_START, 8, part2 / 100 - (part2/1000) * 10);
                   
                   //Gps speed in m/s    
                   uint16_t speedTemp;
                   uint8_t speedProc;
                   
                   speedTemp = (int) ((RS232_TX_Buffer[18] << 8) | RS232_TX_Buffer[17]);
                   speedTemp /= 27.8;   //convert speed to km/h
                   
                   //Logger
                   if(highSpeed < speedTemp)
                      highSpeed = speedTemp;
                   ////   
                   speedProc = (speedTemp - (speedTemp/10)*10);
                   speed_pointer = speedProc;  //pointer for moving vertical lines in A.horizont 
                   
                   buffering2BigNumbers(SPEED_BIGNUMBAR_START,speedTemp / 10 - (speedTemp/100) * 10,speedProc);
             
                  #endif 
                  break;  
  
     case MSP_COMP_GPS:
                  
                  #ifdef GPS             
                   //Home distance
                   homeDistance = (int) ((RS232_TX_Buffer[6]<<8) | RS232_TX_Buffer[5]);
                   
                   //Logger
                   if(highDistance < homeDistance)
                      highDistance = homeDistance;
                   ///
                   
                   //Home direction
                   homeDirection = (int) ((RS232_TX_Buffer[8]<<8) | RS232_TX_Buffer[7]); 
                   
                   if(screenNumber != 1)
                      return;
                   
                   uint8_t gpsDistanceBar[5];
    
                   gpsDistanceBar[0] = homeDistance / 10000;
                   gpsDistanceBar[1] = homeDistance / 1000 - (homeDistance/10000) * 10;
                   gpsDistanceBar[2] = homeDistance / 100 - (homeDistance/1000) * 10;
                   gpsDistanceBar[3] = homeDistance / 10 - (homeDistance/100) * 10;
                   gpsDistanceBar[4] = homeDistance  -   (homeDistance/10) * 10;
    
                   uint8_t startTemp;
                   startTemp = 0;
                   
                   for(uint8_t i = 0; i<4 ;i++){
                      if(gpsDistanceBar[i] == 0 && startTemp == 0)
                         buffering(GPSDISTANCEBAR_START,i,10);
                      else{
                         buffering(GPSDISTANCEBAR_START,i,gpsDistanceBar[i]);
                         startTemp = 1;
                      }         
                   }  
                   buffering(GPSDISTANCEBAR_START,4,gpsDistanceBar[4]);  
                  #endif                   
                  break;  
   
     case MSP_ATTITUDE:      //readed each 100 ms
                   
                   #ifdef BAT1
                    if (bat1_voltage < eepromBuffer[BAT1_LEVEL])
                    {
                       if(mSeconds - bat1Timer >= 200){              // ~200 mSec timer
                          if(bat1Blink == 1){
                             buffering(BAT1BAR_START,0,61); buffering(BAT1BAR_START,1,58); buffering(BAT1BAR_START,2,60);   
                             bat1Blink = 0;
                          }else{
                             buffering(BAT1BAR_START,0,10); buffering(BAT1BAR_START,1,10); buffering(BAT1BAR_START,2,10);   
                             bat1Blink = 1;
                          }
                          bat1Timer = mSeconds;     
                       }  
                    }
                   #endif
                                  
                   //roll - pitch angle
                   readedRollAngle  = (int) ((RS232_TX_Buffer[6]<<8) | RS232_TX_Buffer[5]);
                   readedPitchAngle = (int) ((RS232_TX_Buffer[8]<<8) | RS232_TX_Buffer[7]);         

                   readedRollAngle_radar = readedRollAngle/10;         //convert to deg
                   readedPitchAngle_radar = readedPitchAngle/10;       //convert to deg
  
                   readedRollAngle = readedRollAngle_radar;
                   readedPitchAngle = readedPitchAngle_radar;
  
                   //compass angle
                   int16_t readedHeading_temp;
  
                   readedHeading = (int) ((RS232_TX_Buffer[10]<<8) | RS232_TX_Buffer[9]);             
                   readedHeading_temp = readedHeading;
                   
                   if(screenNumber == 2){
                      computeRadar(homeDistance);
                      return;
                   }
                   
                   #ifdef COMPASS                      
                    printCompass(readedHeading);
    
                    if(readedHeading_temp < 0)
                       readedHeading_temp += 360; 
                       
                    buffering(HEADINGBAR_START,0,  readedHeading_temp / 100);         
                    buffering(HEADINGBAR_START,1, (readedHeading_temp / 10 - (readedHeading_temp / 100) * 10));
                    buffering(HEADINGBAR_START,2, (readedHeading_temp - (readedHeading_temp / 10) * 10));             
                   #endif 
                                    
                   uint8_t readedRollAngleIsPositive;
                   uint8_t readedPitchAngleIsPositive;
                 
                   readedRollAngleIsPositive = 1;
                   readedPitchAngleIsPositive = 1;
                 
                   if(readedRollAngle < 0){        //positive or negative roll
                      readedRollAngle = - readedRollAngle;
                      readedRollAngleIsPositive = 0;
                   }

                   if(readedPitchAngle < 0){
                      readedPitchAngle = - readedPitchAngle;
                      readedPitchAngleIsPositive = 0;
                   }

                   readedRollAngle  *= (float)(eepromBuffer[ROLL_SENSITIVITY]/10.0f);           
                   readedPitchAngle *= (float)(eepromBuffer[PITCH_SENSITIVITY]/10.0f);    

                   readedRollAngle = min(readedRollAngle, 20);     
                   
                   uint8_t startPos;
                   uint8_t endPos;
  
                   if(readedRollAngleIsPositive){     // + roll angles 
                   #ifdef ROLL_TILT_REVERSED 
                      startPos = 0;   //column 0
                      endPos = 12;    //column 12
                   #else
                      startPos = 12;   //column 0
                      endPos = 0;    //column 12
                   #endif 
                   }else{            // - roll angles
                   #ifdef ROLL_TILT_REVERSED 
                      startPos = 12;  //column 12
                      endPos = 0;     //column 0 
                   #else 
                      startPos = 0;  //column 12
                      endPos = 12;     //column 0
                   #endif 
                   }

                   if(readedPitchAngleIsPositive){    // pitch angles 0 to 40 deg
                   #ifdef PITCH_TILT_REVERSED   
                      pitchAngleRelative = MAIN_PANEL_CENTER_LINE - readedPitchAngle;                      
                   #else
                      pitchAngleRelative = MAIN_PANEL_CENTER_LINE + readedPitchAngle;                 
                   #endif
                   }else{                  // pitch angles -40 to 0 deg
                   #ifdef PITCH_TILT_REVERSED
                      pitchAngleRelative = MAIN_PANEL_CENTER_LINE + readedPitchAngle;
                   #else
                      pitchAngleRelative = MAIN_PANEL_CENTER_LINE - readedPitchAngle;
                   #endif   
                   }           
     
                   if(pitchAngleRelative < 0)
                      pitchAngleRelative = 0;
                   else if(pitchAngleRelative > MAIN_PANEL_CENTER_LINE * 2)   
                      pitchAngleRelative = MAIN_PANEL_CENTER_LINE * 2;
      
                   //CRYTICAL PART
                   cryticalPart = 1;
                     startPos_temp = startPos; 
                     endPos_temp = endPos;
                     readedRollAngle_temp = readedRollAngle;
                     pitchAngleRelative_temp = pitchAngleRelative;
                   cryticalPart = 0;
                   //END   
                   
                   #if defined GPS || defined COMPASS
                    //Home arrow 
                    uint16_t homeArrowType;
                   
                    homeArrowType = 0;
                    if(relativeDirection < 0){
                      if(relativeDirection < -157 )
                         homeArrowType = 128;       
                      else if(relativeDirection < -112 )
                         homeArrowType = 160;       
                      else if(relativeDirection < -67 )
                         homeArrowType = 192;       
                      else if(relativeDirection < -22 )
                         homeArrowType = 224;        
                      else
                         homeArrowType = 0;   // - 22 to 22 deg
                    }else{
                      if(relativeDirection > 157 )
                         homeArrowType = 128;              
                      else if(relativeDirection > 112 )
                         homeArrowType = 96;     
                      else if(relativeDirection > 67 )
                         homeArrowType = 64;        
                      else if(relativeDirection > 22 )
                         homeArrowType = 32;     
                      else
                         homeArrowType = 0;  // - 22 to 22 deg                      
                    }

                    bufferingArrow(ARROWBAR_START,homeArrowType);                    
                    
                   #endif
                   
                   #ifdef GPS   
                    //FIX
                    if(gpsFix > 0){
                      if(mSeconds - gpsFixTimer >= 200){              // ~200 mSec timer
                         if(gpsFixBlink == 1){
                            buffering(NUMSATBAR_START, 0, 48);
                            gpsFixBlink = 0;
                         }else{
                            buffering(NUMSATBAR_START, 0, 10);
                            gpsFixBlink = 1;
                         }
                         gpsFixTimer = mSeconds;     
                      }   
                    }else
                      buffering(NUMSATBAR_START, 0, 48);
                   
                    //RTH function enable
                    if((sensActivated & (1 << 6)) > 0){
                       if(mSeconds - autoModeTimer >= 200){  //200mSec 
                          if(autoModeBlink == 1){
                             buffering(FUNCTIONSBAR_START, 5, 67);
                             buffering(FUNCTIONSBAR_START, 6,'R' - 54);
                             buffering(FUNCTIONSBAR_START, 7,'T' - 54);
                             buffering(FUNCTIONSBAR_START, 8,'H' - 54);
                             autoModeBlink = 0;
                          }else{
                             buffering(FUNCTIONSBAR_START, 5, 10);
                             buffering(FUNCTIONSBAR_START, 6, 10);
                             buffering(FUNCTIONSBAR_START, 7, 10);
                             buffering(FUNCTIONSBAR_START, 8, 10);
                             autoModeBlink = 1;
                          }
                          autoModeTimer = mSeconds;
                       }    
                    }else if(autoModeBlink == 0){
                       buffering(FUNCTIONSBAR_START, 5, 10);
                       buffering(FUNCTIONSBAR_START, 6, 10);
                       buffering(FUNCTIONSBAR_START, 7, 10);
                       buffering(FUNCTIONSBAR_START, 8, 10);
                       autoModeBlink = 1;  
                    }  
                   #endif
                   break;  
     
     case MSP_ALTITUDE:
                   #ifdef ALTITUDE
                     
                     int16_t alt, altTemp;
                     //Baro
                     alt = (int) ((RS232_TX_Buffer[6] << 8) | RS232_TX_Buffer[5]);                    
                                           
                     if(firstAlt <= 5){  //delay loop 5 times to avoid fixup 0mt ground errors at startup (it's a delay from MWii start)
                        storedAlt = alt;   
                        firstAlt ++;
                     }        
                 
                     altTemp = alt - storedAlt;
      
                     if(altTemp < 0)
                        altTemp = 0;
      
                     //Logger
                     if(highAltitude < altTemp)
                        highAltitude = altTemp;
                     /////
                     
                     if(screenNumber != 1)
                        return;
                     
                     uint8_t altProc;
                     altProc = (altTemp / 100 - (altTemp/1000) * 10);
                     cmAlt_pointer = altProc; 
    
                     buffering3BigNumbers(ALTITUDE_BIGNUMBAR_START,altTemp / 10000,
                                                                  (altTemp / 1000 - (altTemp/10000) * 10),
                                                                  altProc); 
                   
                   #endif
                   break;  
   
     case MSP_BAT:
                   #ifdef BAT1
                     bat1_voltage = read8(5);
                     bat1_voltage += (eepromBuffer[BAT1_CORRECTION] - NULL_CORRECTION); 
                     
                     if(bat1_voltage < 0)
                        bat1_voltage = 0;
    
                     if(screenNumber == 6)
                        return;
                     
                     buffering2BigNumbers(BAT1_BIGNUMBAR_START,bat1_voltage/100,bat1_voltage/10 - (bat1_voltage/100) * 10);  
                     
                     buffering(BAT1VOLTAGEBAR_START,0, bat1_voltage/100);  
                     buffering(BAT1VOLTAGEBAR_START,1, bat1_voltage/10 - (bat1_voltage/100) * 10);
                     buffering(BAT1VOLTAGEBAR_START,2, 38);
                     buffering(BAT1VOLTAGEBAR_START,3, bat1_voltage - (bat1_voltage/10) * 10);
                     buffering(BAT1VOLTAGEBAR_START,4, 'V' - 54);
                
                     if(bat1_voltage/100 == 0)
                        buffering(BAT1VOLTAGEBAR_START,0,10);            //blank

                     if(bat1_voltage >= eepromBuffer[BAT1_LEVEL]){ 
      
                       if(bat1_voltage >= BAT1_LEVEL_2){
                          buffering(BAT1BAR_START,0,57); buffering(BAT1BAR_START,1,57); buffering(BAT1BAR_START,2,59);   
                       }else if(bat1_voltage >= BAT1_LEVEL_1){
                          buffering(BAT1BAR_START,0,57); buffering(BAT1BAR_START,1,57); buffering(BAT1BAR_START,2,60);   
                       }else if(bat1_voltage >= eepromBuffer[BAT1_LEVEL]){
                          buffering(BAT1BAR_START,0,57); buffering(BAT1BAR_START,1,58); buffering(BAT1BAR_START,2,60);   
                       }
                     }    
                   #endif         
                   break;  
   
     case MSP_RC:  
                   for(uint8_t i = 0; i<8; i++)                      
                       rcData[i] = (int) ((RS232_TX_Buffer[i*2 + 6] << 8) | RS232_TX_Buffer[i*2 + 5]);
                                                        
                   #ifdef STANDARD_RC
                    //LEFT/RIGHT
                    if(rcData[YAW] >= 1900)
                      right ++;
                    else if(rcData[YAW] <= 1100)
                      left  ++;
                    else{
                      right = 0;
                      left = 0;               
                    }
                   #else    //yaw to roll
                    //LEFT/RIGHT
                    if(rcData[ROLL] >= 1900)
                      right ++;
                    else if(rcData[ROLL] <= 1100)
                      left  ++;
                    else{
                      right = 0;
                      left = 0;                  
                    }
                   #endif 
                   
                   //UP/DOWN
                   if(rcData[PITCH] >= 1900)
                      up ++;
                   else if(rcData[PITCH] <= 1100)
                      down  ++;
                   else{
                      up = 0;
                      down = 0;               
                   }
                   
                   #ifdef STANDARD_RC
                    //YAW LEFT/RIGHT
                    if(rcData[ROLL] >= 1900)
                      yright ++;
                    else if(rcData[ROLL] <= 1100)
                      yleft  ++;
                    else{
                      yright = 0;
                      yleft = 0;               
                    }
                   #else    //roll to yaw
                    //YAW LEFT/RIGHT
                    if(rcData[YAW] >= 1900)
                      yright ++;
                    else if(rcData[YAW] <= 1100)
                      yleft  ++;
                    else{
                      yright = 0;
                      yleft = 0;                  
                    }
                   #endif
                   
                   if(pageNumber == MAX_MENU_PAGES + 1){           //if result page => rc data blocking
                     up = 0;
                     down = 0;
                     yright = 0;
                     yleft = 0;
                     right = 0;
                     left = 0;
                   }  
                   
                   if(screenNumber == 6){ 
                     if(pageNumber == INFO){
                       //roll
                       bufferingInMenu(2, 16, rcData[ROLL] / 1000);
                       bufferingInMenu(2, 17, rcData[ROLL] / 100 - (rcData[ROLL]/1000) * 10);
                       bufferingInMenu(2, 18, rcData[ROLL] / 10 - (rcData[ROLL]/100) * 10);
                       bufferingInMenu(2, 19, rcData[ROLL]  - (rcData[ROLL]/10) * 10);        

                       //pitch
                       bufferingInMenu(3, 16, rcData[PITCH] / 1000);
                       bufferingInMenu(3, 17, rcData[PITCH] / 100 - (rcData[PITCH]/1000) * 10);
                       bufferingInMenu(3, 18, rcData[PITCH] / 10 - (rcData[PITCH]/100) * 10);
                       bufferingInMenu(3, 19, rcData[PITCH]  - (rcData[PITCH]/10) * 10);        

                       //yaw
                       bufferingInMenu(4, 16, rcData[YAW] / 1000);
                       bufferingInMenu(4, 17, rcData[YAW] / 100 - (rcData[YAW]/1000) * 10);
                       bufferingInMenu(4, 18, rcData[YAW] / 10 - (rcData[YAW]/100) * 10);
                       bufferingInMenu(4, 19, rcData[YAW]  - (rcData[YAW]/10) * 10);        

                       //throttle
                       bufferingInMenu(5, 16, rcData[THROTTLE] / 1000);
                       bufferingInMenu(5, 17, rcData[THROTTLE] / 100 - (rcData[THROTTLE]/1000) * 10);
                       bufferingInMenu(5, 18, rcData[THROTTLE] / 10 - (rcData[THROTTLE]/100) * 10);
                       bufferingInMenu(5, 19, rcData[THROTTLE]  - (rcData[THROTTLE]/10) * 10);
                     }
                   }else{   
                     if(!armed){
                        if(yright >= eepromBuffer[RC_SENS]){   //Go to config screen           
                           screen = 6;
                           return;                    
                        }     
                     } 
                     
                     //home direction (mixed gps & compass data)             
                     relativeDirection = homeDirection - readedHeading;
              
                     if(relativeDirection <= - 180)
                        relativeDirection += 360;
                     else if(relativeDirection > 180)
                        relativeDirection -= 360;
        
                     uint8_t switchEnable = 0;
      
                     switch(eepromBuffer[AUX_SW]){
                          case 1: //reaction to aux1
                                 if(rcData[AUX1] > 1700)
                                    switchEnable = 1;
                                 break;
                          case 2:
                                 if(rcData[AUX2] > 1700)
                                    switchEnable = 1;
                                 break;
                          case 3:
                                 if(rcData[AUX3] > 1700)
                                    switchEnable = 1;
                                 break;
                          case 4:
                                 if(rcData[AUX4] > 1700)
                                    switchEnable = 1;
                                 break;
                          default:
                                 break;    
                     }
  
                     if(switchEnable == 1){         //change screen only from state 0 to 1 
                        if(screenSwitchEnable == 1){      
                           screenPointer = (++screenPointer) % NUM_OF_AUX_SCREEN;      
                           clearVideoBuffer(VIDEO_BUFFER_SIZE);
                           screen = screenMap[screenPointer];//switching screen 
                           screenSwitchEnable = 0;           
                           return;
                        }
                     }else{
                        screenSwitchEnable = 1;
                        switchEnable = 0;
                     }
                     
                     drawRadar();
                   }                    
                   break;  
   
     case MSP_STATUS:
                   
                   //sensor activated
                   sensActivated = (int) ((RS232_TX_Buffer[12] << 8) | RS232_TX_Buffer[11]);
                   //Armed   
                   armed_old = armed;
                   armed = (uint8_t)(sensActivated & (1<<BOXARM));     
                   
                   if(armed_old && (armed_old != armed)){
                      pageNumber = MAX_MENU_PAGES + 1;
                      screen = 6; // go to result screen
                      return;                
                   }
                                                          
                   if(screenNumber == 6 && pageNumber == SENSOR1){                  
                      //i2c errors
                      i2c_errors = (int) ((RS232_TX_Buffer[8] << 8) | RS232_TX_Buffer[7]);  
                      //sensor present
                      sensPresent = RS232_TX_Buffer[9]; //sensPresent = (int) (RS232_TX_Buffer[10] << 8) | RS232_TX_Buffer[9]; 
                      
                      for(uint8_t i = 0; i < 4 ; i++){ 
        
                          //if sensor is presented
                          if((sensPresent & 1 << i) > 0){  
                              bufferingInMenu(i+1, 9,  'Y'-54);
                              bufferingInMenu(i+1, 10, 'E'-54);
                              bufferingInMenu(i+1, 11, 'S'-54);
                          }else{
                              bufferingInMenu(i+1, 9,  'N'-54);
                              bufferingInMenu(i+1, 10, 'O'-54);
                              bufferingInMenu(i+1, 11, 10);
                          }

                          if(i != 3){
                              //if sensor mode is activated   
                              if((sensActivated & 1 << i) > 0){  
                                  bufferingInMenu(i+1, 16, 'O'-54);
                                  bufferingInMenu(i+1, 17, 'N'-54);
                                  bufferingInMenu(i+1, 18, 10);
                              }else{
                                  bufferingInMenu(i+1, 16, 'O'-54);
                                  bufferingInMenu(i+1, 17, 'F'-54);
                                  bufferingInMenu(i+1, 18, 'F'-54);
                              }
                          }          
                      }
                      
                      //GPS   
                      if((sensActivated & 1 << 6) > 0){           //HOME 
                          bufferingInMenu(4, 16, 'G'-54);
                          bufferingInMenu(4, 17, 'H'-54);
                          bufferingInMenu(4, 18, 'M'-54);
                      }else if((sensActivated & 1 << 7) > 0){     //HOLD
                          bufferingInMenu(4, 16, 'G'-54);
                          bufferingInMenu(4, 17, 'H'-54);
                          bufferingInMenu(4, 18, 'L'-54);
                      }else{
                          bufferingInMenu(4, 16, 'O'-54);
                          bufferingInMenu(4, 17, 'F'-54);
                          bufferingInMenu(4, 18, 'F'-54);
                      }
                                                      
                      //i2c error count
                      bufferingInMenu(5, 8 , i2c_errors / 10000);
                      bufferingInMenu(5, 9 , i2c_errors / 1000 - (i2c_errors/10000) * 10);
                      bufferingInMenu(5, 10,  i2c_errors / 100 - (i2c_errors/1000) * 10);
                      bufferingInMenu(5, 11, i2c_errors / 10 - (i2c_errors/100) * 10);
                      bufferingInMenu(5, 12, i2c_errors  - (i2c_errors/10) * 10);                 
                   }
 
                   if(screenNumber != 1)
                      return;
 
                   //Level   
                   if((sensActivated & 1) > 0)  
                       buffering(FUNCTIONSBAR_START,0, 'L' - 54);  
                   else 
                       buffering(FUNCTIONSBAR_START,0, 10);
  
                   //Baro   
                   if((sensActivated & 2) > 0)  
                       buffering(FUNCTIONSBAR_START,1, 'B' - 54);  
                   else 
                       buffering(FUNCTIONSBAR_START,1, 10);
  
                   //Mag   
                   if((sensActivated & 4) > 0)  
                       buffering(FUNCTIONSBAR_START,2, 'M' - 54);  
                   else 
                       buffering(FUNCTIONSBAR_START,2, 10);
  
                   //Gps hold   
                   if((sensActivated & 128) > 0){  
                       buffering(FUNCTIONSBAR_START,3, 'G' - 54);     
                   }else 
                       buffering(FUNCTIONSBAR_START,3, 10);
                   
                   break;  
   
     case MSP_IDENT:  
                   if(screenNumber == 6 && pageNumber == INFO){
                      uint8_t vers = RS232_TX_Buffer[5];
                      bufferingInMenu(1, 16, vers / 100);
                      bufferingInMenu(1, 17, vers / 10 - (vers/100) * 10);
                      bufferingInMenu(1, 18, vers  - (vers/10) * 10);          
                   }                
                   break;  
    
     case MSP_RC_TUNING:
                   
                   if(configDataReaded >= 3)                  //only first read cyclus  
                      return;
                   
                   for(uint8_t i = 0; i < RCTUNING_ITEMS; i ++)
                      pidBuffer[((i*3)+1) + PID_DATA_BYTES] = RS232_TX_Buffer[i + 5];
                
                   configDataReaded ++;
                   break;  
    
     case MSP_PID:
                   if(configDataReaded >= 3)                  //only first read cyclus  
                      return;
    
                   //read PID config data from MWC to PID buffer
                   for(uint8_t j = 1; j <= PID_DATA_BYTES; j++){
                      pidBuffer[j] = RS232_TX_Buffer[j+4];
                   }
                   
                   configDataReaded ++;  
                   break;  
     
     case MSP_BOX:
                   if(configDataReaded >= 3)                  //only first read cyclus  
                      return;
                   
                   //write modes to activate buffer
                   for(uint8_t i = 0; i < BOXITEMS; i++){
                       activate[i] = (int) ((RS232_TX_Buffer[i*2 + 6] << 8) | RS232_TX_Buffer[i*2 + 5]); 
                   } 
                   
                   configDataReaded ++;
                   break;  
        
  }
}

//print compass data
static void printCompass(int16_t heading){

  int8_t deg = (heading / 15) + 5;
  if(heading <= - 173 || heading >= 173)
     deg = 17;
 
  for(uint8_t i = 0; i < 11; i++){

    buffering(COMPASSBAR_START, 10-i, 63);

    if(deg == i)   
      buffering(COMPASSBAR_START, 10-i, 'N' - 54);

    if((deg - 6 >= 0) && (deg - 6 == i))
      buffering(COMPASSBAR_START, 10-i, 'E' - 54);

    if((deg + 6 < 11) && (deg + 6 == i))
      buffering(COMPASSBAR_START, 10-i, 'W' - 54);

    if((deg + 12 < 11) && (deg + 12 == i))
      buffering(COMPASSBAR_START, 10-i, 'S' - 54);

    if((deg - 12 >= 0) && (deg - 12 == i))
      buffering(COMPASSBAR_START, 10-i, 'S' - 54); 
  }
}

//for accData & altitude
static void printMainPanelToBuffer(uint8_t centerLine, uint8_t centerPoint,uint8_t rollAngle){

  int8_t temp3 =  pitchAngleRelative_temp - rollAngle;
  uint8_t temp4 = pitchAngleRelative_temp + rollAngle;
  int8_t temp5;
  uint8_t temp6 = rollAngle/3;
  uint8_t angleType = 0;   // 0=> angle%3 == 0; 1=> angle+1%3 == 0 ; 2=> angle-1%3 == 0
  
  if(rollAngle%3 == 1){   // angle 4 ,7 ,10, 13 ...
    angleType = 2;    
  }else if(rollAngle%3 == 0){ // 3, 6, 9, 12 ...
    angleType = 0; 
  }else{ // 2, 5, 8, 11 ...    
    temp6 += 1;
    angleType = 1;
  }
  
  if(temp3 < 0){
    //if line go out from window
    if(segmentCounter == 0){       
       pointYpos = temp3;      
    }  
    temp5  =  segmentCounter;
  }else
    temp5  =  segmentCounter - temp3;
     
  for(uint8_t i = 0; i < 13 ; i++){          //null curve buffer
      curveBuffer[i] = 0; 
  }
   
  if(rollAngle > 2){ //3 and more deg
    for (uint8_t i = 0 ; i < 13 ; i ++){   
      if(temp5 == pointYpos || pointYpos < 0){    
         if(i == pointXpos || pointYpos < 0){  
            if(pointYpos >= 0){
               curveBuffer[i] = 0b00011110;
               if(pointXpos != centerPoint){
                 if(startPos_temp == 12)
                    curveBuffer[i - 1] = 0b00011110;
                 else
                    curveBuffer[i + 1] = 0b00011110;
               }        
            }
            angleType_count ++;
            if(angleType_count == 3 || angleType_count == 4){
              if(angleType == 1)
                pointYpos += (temp6 - 1);
              else if(angleType == 2)
                pointYpos += (temp6 + 1);
              else
                pointYpos += temp6;     
            }else    
                pointYpos += temp6;
            
            if(startPos_temp == 12){
                 if(pointXpos == centerPoint)
                    pointXpos -- ;
                 else
                    pointXpos -= 2;   
            }else{
                 if(pointXpos == centerPoint)
                    pointXpos ++ ;
                 else
                    pointXpos += 2;   
            }       
         }
      }
    } 
  }else if(rollAngle > 0){ // 0-2 deg         
    uint8_t p = startPos_temp;
    while(p>=0 && p<=12){
      if(temp5 == pointYpos || pointYpos < 0){    
         if(p == pointXpos || pointYpos < 0){  
            if(pointYpos >= 0){
               curveBuffer[p] = 0b00011110;
               if(pointXpos != centerPoint){
                 if(startPos_temp == 12)
                    curveBuffer[p - 1] = 0b00011110;
                 else
                    curveBuffer[p + 1] = 0b00011110;
               }        
            }
     
            //x axis moving
            if(startPos_temp == 12){
                 if(pointXpos == centerPoint)
                    pointXpos -- ;
                 else
                    pointXpos -= 2;   
            }else{
                 if(pointXpos == centerPoint)
                    pointXpos ++ ;
                 else
                    pointXpos += 2;   
            }
            
            //y axis moving  0 > { }  1 > {6,7} 2 > {4,6,7,9}
            if(pointXpos == 5 || pointXpos == 6 || pointXpos == 7){
               pointYpos ++; 
            }else if((pointXpos == 3 || pointXpos == 4) || (pointXpos == 8 || pointXpos == 9)){
               if(rollAngle == 2)
                  pointYpos ++;                                            
            }               
         }
      }
      
      if(startPos_temp == 12)
         p--;
      else
         p++;   
    }        
  }else if(segmentCounter == pitchAngleRelative_temp){  //angle 0
     for(uint8_t p = 0; p < 13; p++){
         curveBuffer[p] = 0b00011110;
     }  
  }
  // center + 
  if(segmentCounter == centerLine){ //Hline
     curveBuffer[4] = 0b00001111;
     curveBuffer[5] = 0b11111111;
     curveBuffer[7] = 0b11111111;
     curveBuffer[8] = 0b11111110;
  }else if(segmentCounter >= CLMR && segmentCounter <= CLPR)
     curveBuffer[6] = 0b00001110;  //Vline   
}

static void printFlightPanel(){

    DDRB|=0x08;
    _delay_loop_1(17);
     
    //center column
    #ifdef VERTICAL_LINES
    if(math1){       //long line
       dimOn;
       SPDR=0b11111110;                          
    }else{            //short line
       dimOn;
       SPDR=0b00011110;
    }        
    clearVideoOut;
    dimOff;
    #else
     delayMicroseconds(1);
    #endif
    
    delayMicroseconds(10);
    _delay_loop_1(3);
    
    //curve
    for (uint8_t i = 0 ; i < 13 ; i++)     
    {              
      SPDR = curveBuffer[i];
      clearVideoOut;
      _delay_loop_1(3);
      delay1;   
    }

    delayMicroseconds(10);  
    _delay_loop_1(3);
     
    #ifdef VERTICAL_LINES
    if(math2){       //long line
       dimOn;
       SPDR=0b11111110;                          
    }else{            //short line
       dimOn;
       SPDR=0b11110000;
       delay2;
    }   
    //clearVideoOut;
    dimOff;
    #else
      delayMicroseconds(1);
    #endif          
}

static void swap(int16_t* a, int16_t* b) {
	int16_t temp = *a;
	*a = *b;
	*b = temp;
}

static void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
	volatile uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(&x0, &y0);
		swap(&x1, &y1);
	}		 
	
        if (x0 > x1) {
		swap(&x0, &x1);
		swap(&y0, &y1);
	}		 
	int8_t deltax = x1 - x0;
	int8_t deltay = abs(y1 - y0);
	int8_t error = deltax / 2;
	int8_t ystep;
	int8_t y = y0;
	
        if (y0 < y1) { 
		ystep = 1; 
	}else {
		ystep = -1;
	}		
	
        for (int16_t x = x0; x <= x1; ++x) {
	    if(y >= 0 && y < GRAPHIC_SIZE){      	
               if (steep)
	           writePixel(y, x, 1);
               else 
		   writePixel(x, y, 1);          
	    }			
	    error = error - deltay;
	    if (error < 0) {
		y = y + ystep;
		error = error + deltax;
	    }
	}				 
}

static void drawCircle(uint8_t center_x,uint8_t center_y,uint8_t radius){

    int f = 1 - radius;
    int ddF_x = 1;
    int	ddF_y = -2 * radius;
    int x = 0;
    int y = radius;
        
    writePixel(center_x, center_y + radius, 1);
    writePixel(center_x, center_x - radius, 1);
    writePixel(center_x + radius, center_y, 1);
    writePixel(center_x - radius, center_y, 1);

    while(x < y) {
	if(f >= 0) {
          y--;
	  ddF_y += 2;
  	  f += ddF_y;
	}
	x++;
	ddF_x += 2;
        f += ddF_x;
	
        uint8_t cxpx = center_x + x;
        uint8_t cxmx = center_x - x;
        uint8_t cypy = center_y + y;
        uint8_t cymy = center_y - y;
        uint8_t cxpy = center_x + y;		
        uint8_t cypx = center_y + x;
        uint8_t cxmy = center_x - y;
        uint8_t cymx = center_y - x;
        		
        writePixel(cxpx, cypy,1);
	writePixel(cxmx, cypy,1);
	writePixel(cxpx, cymy,1);
	writePixel(cxmx, cymy,1);
	writePixel(cxpy, cypx,1);
	writePixel(cxmy, cypx,1);
	writePixel(cxpy, cymx,1);
	writePixel(cxmy, cymx,1);
    }
}

static void drawCharacter(uint8_t x, uint8_t y, const unsigned char * bmp,uint16_t charIndex,uint8_t lines) {

       charIndex = charIndex*lines;
       uint8_t charByte;	
       for(uint8_t i = 0; i<lines; i++){
           
           charByte = pgm_read_byte(bmp + charIndex++);
           
           writePixel(x-1,y+i,0); // left right character border                
           for(uint8_t j = 0; j < 8;j++){
           
               if((charByte & (0x80 >> j)) > 0){
                  writePixel(x+j,y+i,1);
               }else{
                  writePixel(x+j,y+i,0);           
               }
           }
           writePixel(x+8,y+i,0);  //left right character border 
       }
       for(uint8_t j = 0; j < 8 ;j++){     //top bottom character border
           writePixel(x+j,y-1,0);
           writePixel(x+j,y+7,0);  
       }       
}

static void computeRadar(uint16_t hDistance){

    if(hDistance > MAX_RADAR_DISTANCE) 
       hDistance = MAX_RADAR_DISTANCE;
       
    uint8_t homeRadius = hDistance * RADAR_DISTANCE_FACTOR;     
    
    float rad;
    //roll 
    #ifdef ROLL_TILT_REVERSED
      rad =  readedRollAngle_radar*0.0174533f;   //horizont roll
    #else
      rad =-(readedRollAngle_radar*0.0174533f);   //horizont roll
    #endif 
   
    posArray[0] = (sin(rad) * (RADAR_RADIUSM10));     
    posArray[1] = (cos(rad) * (RADAR_RADIUSM10));    
   
    //pitch
    #ifdef PITCH_TILT_REVERSED
      rad = -(readedPitchAngle_radar*0.0174533f);
    #else
      rad = readedPitchAngle_radar*0.0174533f;
    #endif
    
    posArray[2] =  (sin(rad) * GRAPHIC_SIZE_12);  
    posArray[2] += GRAPHIC_SIZE_12;
       
    rad = readedHeading*0.0174533f;   //N - S, E - W
 
    posArray[3] = (sin(rad) * RADAR_RADIUS);
    posArray[4] = (cos(rad) * RADAR_RADIUS);
     
    rad = - relativeDirection*0.0174533f;         //home position
    
    posArray[5] = (sin(rad) * homeRadius);
    posArray[6] = (cos(rad) * homeRadius);      
}

static void drawRadar(){
  
  if(screenNumber == 2){
    
    clearVideoBuffer(VIDEO_BUFFER_SIZE);
        
    //outer circle
    drawCircle(RADAR_CENTER_X,RADAR_CENTER_Y,RADAR_RADIUS);   
  
    //mid circle
    drawCircle(RADAR_CENTER_X,RADAR_CENTER_Y,RADAR_RADIUS_12);   
    
    //W - E
    drawCharacter((RADAR_CENTER_X + posArray[4])-4, (RADAR_CENTER_Y - posArray[3])-4,letters,'E'-54,7);
    drawCharacter((RADAR_CENTER_X - posArray[4])-4, (RADAR_CENTER_Y + posArray[3])-4,letters,'W'-54,7);
   
    //N - S
    drawCharacter((RADAR_CENTER_X + posArray[3])-4, (RADAR_CENTER_Y + posArray[4])-4,letters,'S'-54,7);
    drawCharacter((RADAR_CENTER_X - posArray[3])-4, (RADAR_CENTER_Y - posArray[4])-4,letters,'N'-54,7);
     
    //Half size of max radar distance in meters
    drawCharacter(RADAR_CENTER_X,   RCYPRR12,smallNumbers,MAX_RADAR_DISTANCE_HALF_SIZE_HIGH,6);
    drawCharacter(RADAR_CENTER_XP4, RCYPRR12,smallNumbers,MAX_RADAR_DISTANCE_HALF_SIZE_MID ,6);
    drawCharacter(RADAR_CENTER_XP8, RCYPRR12,smallNumbers,MAX_RADAR_DISTANCE_HALF_SIZE_LOW ,6);
   
    //Max radar distance in meters
    drawCharacter(RADAR_CENTER_XP8,  RCYPRRM4,smallNumbers,MAX_RADAR_DISTANCE_HIGH,6);           //RCYPRRM2 => (radar_center_y) + (radar_radius - 2)
    drawCharacter(RADAR_CENTER_XP12, RCYPRRM4,smallNumbers,MAX_RADAR_DISTANCE_MID ,6);
    drawCharacter(RADAR_CENTER_XP16, RCYPRRM4,smallNumbers,MAX_RADAR_DISTANCE_LOW ,6);
    
    //horizont line ROLL & PITCH
    drawLine(RADAR_CENTER_X - posArray[1], posArray[2] - posArray[0], RADAR_CENTER_X + posArray[1], posArray[2] + posArray[0]); 
     
    //center arrow
    drawCharacter(RADAR_CENTER_XM3, RADAR_CENTER_YM3,letters,43,7);
   
    //home
    drawCharacter((RADAR_CENTER_X - posArray[5])-4, (RADAR_CENTER_Y - posArray[6])-4,letters,67,7);
    
    //center circle
    drawCircle(RADAR_CENTER_X,RADAR_CENTER_Y,5);
  }    
}

static void writePixel(uint8_t x,uint8_t y,uint8_t color){

  uint8_t tmp = x/8;
  uint16_t tmp1 = (y*GRAPHIC_SIZE_18) + tmp;
  
  if(color == 0)
    VideoBuffer[tmp1] &= ~0x80 >> (x&7);             
  else
    VideoBuffer[tmp1] |= 0x80 >> (x&7);   //x: x pos of point, y: line of point
}

static void readVcc()                                             
{
 #ifdef BAT2  
  #ifdef MOBIDRONEOSD_V2
    sensVcc = analogRead(1) - compCorrection;
  #else
    sensVcc = analogRead(1);
  #endif  
  sensVcc *= 0.0147;                          // 0.0049 * 3 (1:3) measure voltage on 2s - 3s lipo + ---10KOhm---10KOhm---APIN----10KOhm---GND
  sensVcc += (float)((int8_t)(eepromBuffer[BAT2_CORRECTION] - NULL_CORRECTION))/10.0;
  if(sensVcc < 0)
     sensVcc = 0.0;
 #endif 
}

#ifdef CURRENT_SENSOR
static void readCurrent()                                             
{
  #ifdef MOBIDRONEOSD_V2
    float currentValue = analogRead(3) - compCorrection; //raw data reading   
  #else
    float currentValue = analogRead(3);
  #endif
  
  switch(eepromBuffer[CURR_SENS_TYPE]){
    
     case 1: //ACS758 100A bidirectional 
            currentValue = (currentValue - 512) * 0.244; // offset 512 - default value 2,5V for bidirectional sensor ACS758 
            break;                                       // 1024 / 5V  = 205 resolution per Volt ; 100A sensor / 2V = 50A per Volt; 50/205 = 0.244 A per sample  => 244mA
     case 2: //ACS758 50A unidirectional
            currentValue = (currentValue - 123) * 0.061; // offset 123 - default value 0.6V for single sensor ACS758
            break;
     case 3: //Flytron Ultralight 50A Current Sensor 
            #ifdef MOBIDRONEOSD_V2
              currentValue += compCorrection;
            #endif
            currentValue *= 0.049;
            break;
     case 4: //Flytron Ultralight 25A Current Sensor 
            #ifdef MOBIDRONEOSD_V2
              currentValue += compCorrection;
            #endif
            currentValue *= 0.0245;
            break;  
     case 5: //ACS758 200A bidirectional
            currentValue = (currentValue - 512) * 0.488; // offset 512 - default value 2,5V for bidirectional sensor ACS758
            break;
     case 6: //ACS756 50A BiDirectional
            currentValue = (currentValue - 418) * 0.121; //  sensor ACS756
            break;         
  }  
  
  if(currentValue < 0)
     currentValue = 0;
    
  currentAverage += currentValue;
  currIndex ++;
    
  #ifdef PAL
    consumedmAh += ((currentAverage/currIndex)/25.714); //PAL read rate 7*20 = 140 mSec ; 1000mSec/140 = 7.142 * 3600 = 25714.3 samples in 1 Hour (example 244mA/25714.3samples in Hour = 0.244/25.7143 mA/H )  
  #else //NTSC
    consumedmAh += ((currentAverage/currIndex)/30.856); //NTSC read time 7*16.67 = 116.67 mSec ; 30856 samples in 1 Hour  / 1000 = 30.856
  #endif 
  
  uint16_t curTemp = (currentAverage/currIndex)*100;
  uint16_t mAhTemp = consumedmAh; 
  
  if(currIndex >= 10){
       currIndex = 0;
       currentAverage = 0.0;
  }
  
  //Logger
  if(highAmperage < curTemp)
     highAmperage = curTemp;
  ///
  
  if(screenNumber != 1)
     return;
  
  buffering(CURRENTVALUEBAR_START, 0 , curTemp / 1000);
  buffering(CURRENTVALUEBAR_START, 1 , curTemp / 100 - (curTemp/1000) * 10);
  buffering(CURRENTVALUEBAR_START, 3 , curTemp / 10 - (curTemp/100) * 10);
  
  if(curTemp / 1000 == 0)
    buffering(CURRENTVALUEBAR_START, 0 , 10);
   
  uint8_t capacityBar[5];
  
  capacityBar[0] = mAhTemp / 10000; 
  capacityBar[1] = mAhTemp / 1000 - (mAhTemp/10000) * 10;
  capacityBar[2] = mAhTemp / 100 - (mAhTemp/1000) * 10;
  capacityBar[3] = mAhTemp / 10 - (mAhTemp/100) * 10;
  capacityBar[4] = mAhTemp  -   (mAhTemp/10) * 10;
  
  uint8_t startTemp = 0;
  for(uint8_t i = 0; i<4 ;i++){
    if(capacityBar[i] == 0 && startTemp == 0)
       buffering(CAPACITYVALUEBAR_START,i,10);
    else{
       buffering(CAPACITYVALUEBAR_START,i,capacityBar[i]);
       startTemp = 1;
    }         
  }
  
  buffering(CAPACITYVALUEBAR_START,4,capacityBar[4]);
}
#endif

static void readRSSI()
{
  sensRSSI = analogRead(0);                                 

  if(screenNumber == 6){
    if(sensRSSI < 10 && pageNumber == INTERNAL_SETTINGS1){

      bufferingInMenu(3,15,'E'-54);
      bufferingInMenu(3,16,'R'-54);
      bufferingInMenu(3,17,'R'-54);
    }
    return;
  }   

  #ifdef RSSISIGNAL
  int rangeRSSI = constrain(sensRSSI, minRSSI, maxRSSI);   
  uint8_t RSSI = map(rangeRSSI, minRSSI, maxRSSI, 0, 99);          
  uint8_t rssiLevel;
 
  if(RSSI >= RSSI_5){
     rssiLevel = 5;
  }else if(RSSI >= RSSI_4){
     rssiLevel = 4;
  }else if(RSSI >= RSSI_3){
     rssiLevel = 3;
  }else if(RSSI >= RSSI_2){
     rssiLevel = 2;
  }else
     rssiLevel = 1;
  
  for(uint8_t i = 1; i <= 5; i++){   
    if(i <= rssiLevel) 
      buffering(RSSIBAR_START,i,i+51);
    else
      buffering(RSSIBAR_START,i,10);   
  }
  
  buffering(RSSIBAR_START,6,RSSI/10);
  buffering(RSSIBAR_START,7,RSSI%10);
  #endif

}  
//________________________________________________________________________________________________
static void calcTime()
{
  calcTimes++;
  mSeconds = (calcTimes * 160);                          
  
  if(armed){
    calcFlyTimes++;
    seconds = ((calcFlyTimes * 160)/1000);                            
    MinTime_dump = (seconds/60);                               
    SecTime_dump = (seconds%60);
  }
  
  if(screenNumber == 1){
    
    seconds = (mSeconds/1000);                            
    uint8_t  MinTime = (seconds/60);                               
    uint8_t  SecTime = (seconds%60);                  
    
    //start time
    buffering2BigNumbers(STARTTIME_BIGNUMBAR_START,MinTime/10,MinTime%10);
    
    buffering(TIMEBAR_START,1,SecTime/10);
    buffering(TIMEBAR_START,2,SecTime%10);  
    
    //if(MinTime/10 == 0)
    //   buffering(TIMEBAR_START,4,10);
    
    
    //fly time
    buffering2BigNumbers(FLYTIME_BIGNUMBAR_START,MinTime_dump/10,MinTime_dump%10);
    
    buffering(FLYTIMEBAR_START,1,SecTime_dump/10);
    buffering(FLYTIMEBAR_START,2,SecTime_dump%10);
    
    //if(MinTime_dump/10 == 0)
    //   buffering(FLYTIMEBAR_START,4,10);
             
    #ifdef BAT2
    uint8_t tempV = sensVcc;                              // remove decimal part and convert to int
    float bat2_critical_level = eepromBuffer[BAT2_LEVEL]/10.0;
    
    //if(tempV/10 == 0)
    //  buffering(BAT2VOLTAGEBAR_START,0,10); 
    //else
    //  buffering(BAT2VOLTAGEBAR_START,0,tempV/10);
    
    buffering2BigNumbers(BAT2_BIGNUMBAR_START,tempV/10,tempV%10);  
    buffering(BAT2VOLTAGEBAR_START,1,((sensVcc*10)-(tempV*10)));
          
    if (sensVcc < bat2_critical_level)
    {
      if(mSeconds - bat2Timer >= 200){              // ~200 mSec timer
        if(bat2Blink == 1){
          buffering(BAT2BAR_START,0,61); buffering(BAT2BAR_START,1,58); buffering(BAT2BAR_START,2,60);   
          bat2Blink = 0;
        }else{
          buffering(BAT2BAR_START,0,10); buffering(BAT2BAR_START,1,10); buffering(BAT2BAR_START,2,10);   
          bat2Blink = 1;
        }
        bat2Timer = mSeconds;     
      }  
    }else{
      if(sensVcc >= BAT2_LEVEL_2){
         buffering(BAT2BAR_START,0,57); buffering(BAT2BAR_START,1,57); buffering(BAT2BAR_START,2,59);   
      }else if(sensVcc >= BAT2_LEVEL_1){
         buffering(BAT2BAR_START,0,57); buffering(BAT2BAR_START,1,57); buffering(BAT2BAR_START,2,60);   
      }else if(sensVcc >= bat2_critical_level){
         buffering(BAT2BAR_START,0,57); buffering(BAT2BAR_START,1,58); buffering(BAT2BAR_START,2,60);   
      }
    }
    #endif   
  }  
}

//________________________________________________________________________________________________
static void fastUpdate()             // duration 20mSec * 7                            
{
  ftoken=a20mSecTimer&7;                                         
  switch (ftoken) 
  {
  case 0:
    calcTime();
    break;
  case 1:  
    analogRead(3);
    break;
  case 2:
    #ifdef CURRENT_SENSOR
    readCurrent();
    #endif
    break;
  case 3:
    break;
  case 4:
    break;
  case 5:
    break;
  case 6:
    break;
  case 7:
    break;
  default:                                               // default exit;
    break;
  }
}

static void slowUpdate()                                       // duration 20ms * 127
{
  stoken=a20mSecTimer&127; 
  switch (stoken) 
  {
  case 0:                                                
    analogRead(1);                                       
    break;                                               
  case 24:                                               
    readVcc();                                          
    break;                                               
  case 26:                                               
    analogRead(0);                                       
    break;                                                
  case 50:                                               
    readRSSI();                                                              
    break;    
  default:                                               
    break;
  }   
}   

