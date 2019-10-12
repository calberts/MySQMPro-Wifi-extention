// mySQMPRO a DIY Arduino Nano Project
// Sky Quality Meter, Cloud Sensor, Rain Sensor, Barometric Sensor, GPS, Lux Sensor

// (c) Copyright Robert Brown 2014-2019. All Rights Reserved.
// This project is protected by International Copyright Law.
// https://sourceforge.net/projects/arduinomysqmskyqualitymeter/files/mySQMPRO/ Original Source
// Permission is granted for personal and Academic/Educational use only.
// Software distributed under MIT License https://opensource.org/licenses/MIT
//
// THis fork which extent the code with WIfi based up on the NodeMCU 12E 
// Created By Chris ALberts
//
//
// REQUIRES LCD1602, LCD1604, LCD2004 or OLED I2C DISPLAY

// PCB (Version 5) can be ordered online at https://aisler.net/p/JHYFZBXW

// CONTRIBUTIONS
// If you wish to make a contribution in thanks for this project, please use PayPal and send the amount 
// to user rbb1brown@gmail.com (Robert Brown). All contributions are gratefully accepted.

// based loosely on code from http://forum.arduino.cc/index.php?PHPSESSID=knhnejk2g00u2smd8ve8tn5l82&topic=21536.45
// and http://stargazerslounge.com/topic/183600-arduino-sky-quality-meter-working/
// Acknowledgements: madepablo, Martin Nawrath (Frequency Counter Library), Corpze
//
// ------------------------------------------------------------------------------------------------------
// HARDWARE MAPPINGS

// Connection for TSL237 Sensor
// VDD (pin2) to +5V (Note: Use a 0.1uf ceramic capacitor mounted close to the sensor across VDD and GND)
// GND (pin1) to GND
// VOUT (pin3) to Arduino Pin D5

// Connection for NEO-6M-GPS
// GPS-TXD to Arduino Pin D4
// GPS-RXD to Arduino Pin D3
// GND to GND
// VCC to +5V

// Connection for LCD2004 or LCD1604 OR LCD1602 I2C display
// VCC to +5V
// GND to GND
// SDA to A4
// SCL to A5

// Connection for MLX90614 IR Sensor - be careful what I2C connector you connect this to
// Connecting to 5V will destroy this sensor
// VCC  3.3V
// GND  GND
// SDA  A4
// SCL  A5

// Connection for TSL2561 Sensor - be careful what I2C connector you connect this to
// Connecting to 5V will destroy this sensor
// VCC  3.3V
// GND  GND
// SDA  A4
// SCL  A5
// If not fitted, then GL5528 LDR will be used for lux

// Connection for GL5528 LDR
// 5V - GL5528 - Arduino pin A0 - 10K Pulldown resistor - GND

// Connection for Rain Sensor
// AO   to Arduino pin A1
// DO   to Arduino pin D2
// GND  GND
// VCC  5V

// Connection for BME280 Barometric Sensor, uses I2C interface - make sure you purchase the 5V version
// VCC  5V
// GND  GND
// SDA  A4
// SCL  A5

// HARDWARE MAPPINGS END
// ------------------------------------------------------------------------------------------------------
// CONFIG SECTION

// to use TSL2561 sensor for determining lux, uncomment the next line
#define TSL2561SENSOR 1

// To enable GPS, uncomment the next line
//#define useGPSNEO 1

// To enable the rain sensor, uncomment the next line
#define RAINSENSOR 1

// To enable the MLX90614 ir sensor uncomment the next line
#define MLX90614SENSOR 1

// To enable the BME280 humdity, ambient temp, barmetric pressure sensor uncomment the next line
#define BME280SENSOR 1

// YOU MUST CHOOSE ONE OF THE FOLLOWING SENSORS, not both, recommended is TSL237
//#define TSL235 1
#define TSL237 2

// To enable the LCD DISPLAY uncomment the next line
#define LCDDISPLAY 1

// to enable LCD1602 or LCD1604 or LCD2004, only uncomment one or the other below - you must have only ONE uncommented
//#define LCD1602   1
//#define LCD1604   2
#define LCD2004   3

// To enable the OLED display (SSD1306 chip) uncomment the next line
//#define OLEDDISPLAY 1

#define LCDADDRESS 0x27                 // some LCD displays maybe at 0x3F, use I2Cscanner to find the correct address
#define OLEDADDRESS 0x3C                // address of OLED, do not change

// do not change
#ifdef LCDDISPLAY
#ifdef OLEDDISPLAY
#halt //Error - you can only have one LCD type defined - either LCDDISPLAY or OLEDDISPLAY.
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1602
#ifdef LCD1604
#halt //Error - you can only have one LCD type defined, one of LCD1602, LCD1604 or LCD2004.
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1602
#ifdef LCD2004
#halt //Error - you can only have one LCD type defined, one of LCD1602, LCD1604 or LCD2004.
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1604
#ifdef LCD2004
#halt //Error - you can only have one LCD type defined, one of LCD1602, LCD1604 or LCD2004.
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1602
#ifdef LCD2004
#ifdef LCD1604
#halt //Error - you can only have one LCD type defined, one of LCD1602, LCD1604 or LCD2004.
#endif
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifndef LCD2004
#ifndef LCD1604
#ifndef LCD1602
#halt //Error - you can must define either LCD1602 or LCD2004 or LCD1604.
#endif
#endif
#endif
#endif

// do not change
#ifndef TSL235
#ifndef TSL237
#halt Error - you must define ONE TSL SENSOR TYPE
#endif
#endif

// YOU MUST CHOOSE ONE OF THE FOLLOWING METHODS TO CALCULATE THE SQM VALUE
//#define OLDSQMMETHOD 1
//#define NEWSQMMETHOD 2
//#define NEWSQMMETHODCORRECTED 3
#define IRRADIANCEMETHOD 4

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1602
#ifdef LCD2004
#halt // Error, you cannot have both LCD1602 and LCD2004 uncommented at the same time
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1602
#ifdef LCD1604
#halt // Error, you cannot have both LCD1602 and LCD1604 uncommented at the same time
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifdef LCD1604
#ifdef LCD2004
#halt // Error, you cannot have both LCD1604 and LCD2004 uncommented at the same time
#endif
#endif
#endif

// do not change
#ifdef LCDDISPLAY
#ifndef LCD1602
#ifndef LCD2004
#ifndef LCD1604
#halt // Error, you must have either LCD1602 OR LCD1604 OR LCD2004 uncommented
#endif
#endif
#endif
#endif

// do not change
#ifndef OLDSQMMETHOD
#ifndef NEWSQMMETHOD
#ifndef NEWSQMMETHODCORRECTED
#ifndef IRRADIANCEMETHOD
#halt Error - you must define ONE sqm method above
#endif
#endif
#endif
#endif

// do not change
#ifdef OLDSQMMETHOD
#ifdef NEWSQMMETHOD
#halt Error - cannot have more than one SQM method defined
#else
#ifdef NEWSQMMETHODCORRECTED
#halt Error - cannot have more than one SQM method defined
#else
#ifdef IRRADIANCEMETHOD
#halt Error - cannot have more than one SQM method defined
#endif
#endif
#endif
#endif

// do not change
#ifdef NEWSQMMETHOD
#ifdef OLDSQMMETHOD
#halt Error - cannot have more than one SQM method defined
#else
#ifdef NEWSQMMETHODCORRECTED
#halt Error - cannot have more than one SQM method defined
#else
#ifdef IRRADIANCEMETHOD
#halt Error - cannot have more than one SQM method defined
#endif
#endif
#endif
#endif

// do not change
#ifdef NEWSQMMETHODCORRECTED
#ifdef OLDSQMMETHOD
#halt Error - cannot have more than one SQM method defined
#else
#ifdef NEWSQMMETHOD
#halt Error - cannot have more than one SQM method defined
#else
#ifdef IRRADIANCEMETHOD
#halt Error - cannot have more than one SQM method defined
#endif
#endif
#endif
#endif

// do not change
#ifdef IRRADIANCEMETHOD
#ifdef OLDSQMMETHOD
#halt Error - cannot have more than one SQM method defined
#else
#ifdef NEWSQMMETHOD
#halt Error - cannot have more than one SQM method defined
#else
#ifdef NEWSQMMETHODCORRECTED
#halt Error - cannot have more than one SQM method defined
#endif
#endif
#endif
#endif

// do not change
#ifdef OLDSQMMETHOD
#define sqm_limit 21.83                 // mag limit for earth is 21.95
#endif
#ifdef NEWSQMMETHOD
#define sqm_limit 23.09                 // mag limit for earth is 21.95
#endif
#ifdef NEWSQMMETHODCORRECTED
//#define sqm_limit 21.95               // mag limit for earth is 21.95
#define sqm_limit 20                    // mostaccurate within 18-22 mpsas
#endif
#ifdef IRRADIANCEMETHOD
#define sqm_limit 21.95                 // mag limit for earth is 21.95
#endif

#include <SoftwareSerial.h>    // Chris
#include <ArduinoJson.h>       // Chris
#include <Arduino.h>
#include <myQueue.h>                      //  By Steven de Salas
#include <FreqCounter.h>                  // required for frequency measurement of TLS sensor, (c) Martin Nawrath
#include <Math.h>                         // required for log()
#ifdef LCDDISPLAY
#include <LCD.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>            // needed for LCD, see https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
#endif
#ifdef OLEDDISPLAY
#include <Wire.h>                         // needed for I2C, installed when installing the Arduino IDE
#include <mySSD1306AsciiUC.h>             // oled
#include <mySSD1306AsciiWireUC.h>         // oled
#endif
#ifdef useGPSNEO

#include <myAdafruit_GPS.h>               // reguired for GPS, (c) Adafruit
#endif
#ifdef MLX90614SENSOR
#include <myMLX90614AF.h>                 // IR sensor, (c) Adafruit
#endif
#ifdef BME280SENSOR
#include <mybme280.h>
#endif
#ifdef TSL2561SENSOR
#include <myTSL2561AF.h>
#endif

// Chris 
// Connect the TX line from the ESP module to the Arduino's pin 2
// and the RX line from the ESP module to the Arduino's pin 3
// Emulate EspSerial on pins 2/3 if not present
SoftwareSerial s(8, 9); // RX, TX

// ------------------------------------------------------------------------------------------------------
// GLOBAL DEFINES
#define rs_AO           A1                // the rain sensor analog port pin, voltage
#define rs_DO           2                 // the rain sensor digital port pin, raining=yes/no
#define BME280_I2CADDR  0x76
#define LCDADDRESS      0x27
#define OLEDADDRESS     0x3C              // address of OLED
#define GPSRXPin        4
#define GPSTXPin        3                 // the gps transmits to Arduino to pin D4, Arduino transmits to GPS using Pin3 to RXD

#define SENSORSAMPLETIME 1500             // time between updates of sensors
#define SQMSAMPLETIME    5000             // an SQM sample is taken every 5s (cannot be less than 5s)
#define LCDUPDATETIME    4000             // LCD screen updated every 4.5s
#define buf_size        20                // size of temporary buffers for string conversions
#define timebuf_size    12
#define datebuf_size    12
#define SerialPortSpeed 9600              // Note that to talk to APT as a Sky Meter needs to be 115200
#define GPSPortSpeed    9600              // The speed at which the controller talks to a GPS unit
#define LDRCutoff1      275               // boundary between black and dark, used to determine Gate time
#define LDRCutoff2      500               // boundary between dark and light
#define blackperiod     2000              // 2s Gate time, at very dark sites
#define darkperiod      1000              // 1s Gate time, at dark sites
#define lightperiod     100               // 100ms Gate time for day light
#define MAXCOMMAND      8                 // size of receive buffer, :xxxx#
#define SKYCLEAR        0
#define SKYPCLOUDY      1
#define SKYCLOUDY       2
#define SKYUNKNOWN      3


 
// ------------------------------------------------------------------------------------------------------
// PROGRAM VARIABLES
char programName[]      = "MYSQMPRO and Wifi";     // Chris Program title and version information
byte programVersion     = 26;
int  setpoint1;                                    // setpoint values used to determine sky state
int  setpoint2;
int  skystate;
long sensorsampletimer;                           // used to determine when a sample sensor reading is taken
bool sensorsamplerequest;                         // determines which sensors to read
long sqmsampletimer;                              // used to determine when a sample sqm reading is taken
long lcdsampletimer;
long now;
double bme280humidity;
double bme280temperature;
unsigned long int bme280pressure;
float  dewpoint;
float  mlx90614ambient;
float  mlx90614object;
double lux;
bool raining;
int  rainVout;
int  volts;

#ifdef useGPSNEO
SoftwareSerial ss(GPSRXPin, GPSTXPin);    // define software serial port for GPS
Adafruit_GPS GPS(&ss);                    // create GPS object
char mytime[timebuf_size];                // holds formatted time string from GPS data
char mydate[datebuf_size];                // holds formatted date string from GPS data
boolean usingInterrupt = true;
void useInterrupt(boolean);               // Func prototype keeps Arduino 0023 happy
#endif

#ifdef LCDDISPLAY
LiquidCrystal_I2C lcd(LCDADDRESS, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
int lcdpage;                              // which page to display
#endif

#ifdef OLEDDISPLAY
SSD1306AsciiWire myoled;
int lcdpage;                              // which page to display
#endif

// TSL237 Sensor
#define mySQMSensor  5                    // TSL237 OUT to digital pin 5 (cannot change)
float mySQMreading;                       // the SQM value, sky magnitude
double frequency;                         // measured TSL237 frequency which is dependent on light
double irradiance;
double nelm;

#ifdef TSL2561SENSOR
TSL2561 myTSL2561(TSL2561_ADDR_FLOAT);
#endif

// LDR GL5528
int LDRpin = A0;                          // Light Dependant Resistor is on A0
short period;                             // gate time - specifies duration time for frequency measurement

// IR Sensor, used as a cloud sensor, values help define sky state - cloudy, partly cloudy, clear
#ifdef MLX90614SENSOR
Adafruit_MLX90614 mymlx90614 = Adafruit_MLX90614();
#endif

// Serial command interface
Queue<String> queue(10);            // receive serial queue of commands
char line[MAXCOMMAND];
int eoc;                            // end of command
int idx;                            // index into command string

#ifdef BME280SENSOR
bme280 mybme280;
#endif

// ------------------------------------------------------------------------------------------------------
// METHODS START

// calculates dew point
// input:   humidity [%RH], temperature in C
// output:  dew point in C
void calc_dewpoint(float t, float h)
{
  float logEx;
  logEx = 0.66077 + 7.5 * t / (237.3 + t) + (log10(h) - 2);
  dewpoint = (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);
}

void getskystate()
{
#ifdef useGPSNEO
  GPS.pause(true);
#endif
#ifdef MLX90614SENSOR
  float TempDiff = mlx90614ambient - mlx90614object;
  // object temp is IR temp of sky which at night time will be a lot less than ambient temp
  // so TempDiff is basically ambient + abs(object)
  // setpoint 1 is set for clear skies
  // setpoint 2 is set for cloudy skies
  // setpoint2 should be lower than setpoint1
  // For clear, Object will be very low, so TempDiff is largest
  // For cloudy, Object is closer to ambient, so TempDiff will be lowest

  // Readings are only valid at night when dark and sensor is pointed to sky
  // During the day readings are meaningless
  if ( TempDiff > setpoint1 )
    skystate = SKYCLEAR;          // clear
  else if ( (TempDiff > setpoint2) && (TempDiff < setpoint1) )
    skystate = SKYPCLOUDY;        // partly cloudy
  else if (TempDiff < setpoint2)
    skystate = SKYCLOUDY;         // cloudy
  else
    skystate = SKYUNKNOWN;        // unknown
#ifdef useGPSNEO
  GPS.pause(false);
#endif
#endif
}

void getraining()
{
#ifdef RAINSENSOR
  if ( !digitalRead( rs_DO ) )
    raining = true;
  else
    raining = false;
  rainVout = analogRead( rs_AO );
#endif
}

// convert string to int
int decstr2int(String line)
{
  int ret = 0;
  ret = line.toInt();
  return ret;
}

void processCommand()
{
  int len;
  int cmdval;
  String replystr = "";
  String mycmd = "";
  String param = "";

 

#ifdef useGPSNEO
  GPS.pause(true);
#endif
  replystr = queue.pop();
  len = replystr.length();
  if ( len == 2 )
  {
    mycmd = replystr;                         // a valid command with no parameters, ie, :01#
  }
  else if ( len > 2 )
  {
    mycmd = replystr.substring(0, 2);         // this command has parameters
    param = replystr.substring(2, len);
  }
  else return;

  cmdval = decstr2int(mycmd);

#ifdef DEBUG
  Serial.print("replystr = "); Serial.println(replystr);
  Serial.print("len = "); Serial.println(len);
  Serial.print("mycmd = "); Serial.println(mycmd);
  Serial.print("param = "); Serial.println(param);
  Serial.print("cmdval = "); Serial.println(cmdval);
#endif
  switch ( cmdval )
  {
    case 1:     // if the command is GR "GetReading"
      replystr = "A" + String(mySQMreading) + "#";
      Serial.print(replystr);
      break;
    case 2:     // if the command is GF "Get Frequency"
      replystr = "B" + String(frequency) + "#";
      Serial.print(replystr);
      break;
    case 3:     // if the command is GI "Get Irradiance"
      replystr = "C" + String(irradiance) + "#";
      Serial.print(replystr);
      break;
    case 4:                         // if the command is GV "Get Firmware Version"
      replystr = "D" + String(programVersion) + "#";
      Serial.print(replystr);
      break;
    case 5:                         // if the command is GZ "Get Firmware Filename"
      replystr = "E" + String(programName) + "#";
      Serial.print(replystr);
      break;
    case 6:   // if the command is GL "Get LDR value"
      {
        int LDRval = analogRead(LDRpin);  // Read the analogue pin
        replystr = "F" + String(LDRval) + "#";
        Serial.print(replystr);
      }
      break;
    case 7:   // if the command is GP "Get period gate time"
      replystr = "G" + String(period) + "#";
      Serial.print(replystr);
      break;
    case 8:   // if the command is GD "Get Date"
#ifdef useGPSNEO
      replystr = "H" + String(GPS.day) + "/" + String(GPS.month) + "/20" + String(GPS.year) + "#";
#else
      replystr = "H01/01/2001#";
#endif
      Serial.print(replystr);
      break;
    case 9:   // if the command is GT "Get Time"
#ifdef useGPSNEO
      replystr = "I" + String(GPS.hour) + ":" + String(GPS.minute) + ":" + String(GPS.seconds) + "#";
#else
      replystr = "I00:00:00#";
#endif
      Serial.print(replystr);
      break;
    case 10:    // if the command is GO "Get Longitude"
#ifdef useGPSNEO
      replystr = "J" + String(GPS.longitude / 100) + String(GPS.lon) + "#";
#else
      replystr = "J00#";
#endif
      Serial.print(replystr);
      break;
    case 11:    // if the command is GA "Get Latitude"
#ifdef useGPSNEO
      replystr = "K" + String(GPS.latitude / 100) + String(GPS.lat) + "#";
#else
      replystr = "K00#";
#endif
      Serial.print(replystr);
      break;
    case 12:    // if the command is GH "Get Altitude"
#ifdef useGPSNEO
      replystr = "L" + String((int)GPS.altitude) + "#";
#else
      replystr = "L0#";
#endif
      Serial.print(replystr);
      break;
    case 13:    // if the command is GS "Get Satelittes"
#ifdef useGPSNEO
      replystr = "M" + String(GPS.satellites) + "#";
#else
      replystr = "M0#";
#endif
      Serial.print(replystr);
      break;
    case 16:                        // gps fix?
      replystr = "N0#";
#ifdef useGPSNEO
      if ( GPS.fix )
        replystr = "N1#";
#endif
      Serial.print(replystr);
      break;
    case 19:                        // get IR object temperature
      replystr = "O" + String(mlx90614object) + "#";
      Serial.print(replystr);
      break;
    case 20:                        // get IR ambient temperature
      replystr = "P" + String(mlx90614ambient) + "#";
      Serial.print(replystr);
      break;
    case 21:                        // return LUX value
      replystr = "U" + String(lux) + "#";
      Serial.print(replystr);
      break;
    case 23:                        // return raining
      replystr = "R0#";
#ifdef RAINSENSOR
      if ( raining == true )
        replystr = "R1#";
#endif
      Serial.print(replystr);
      break;
    case 24:                        // return rain voltage
      replystr = "S0#";
#ifdef RAINSENSOR
      replystr = "S" + String(rainVout) + "#";
#endif
      Serial.print(replystr);
      break;
    case 25:                        // save setpoint1
      setpoint1 = decstr2int(param);
      break;
    case 26:                        // save setpoint2
      setpoint2 = decstr2int(param);
      break;
    case 27:                        // return sky state
      replystr = "V" + String(skystate) + "#";
      Serial.print(replystr);
      break;
    case 28:                        // return setpoint1
      replystr = "W" + String(setpoint1) + "#";
      Serial.print(replystr);
      break;
    case 29:                          // return setpoint2
      replystr = "X" + String(setpoint2) + "#";
      Serial.print(replystr);
      break;
    case 31:                          // if the command is GI "Get Nelm"
      replystr = "Z" + String(nelm) + "#";
      Serial.print(replystr);
      break;
    case 32:                          // Get Humidity BME280 sensor
      replystr = "a" + String(bme280humidity, 2) + "#";
      Serial.print(replystr);
      break;
    case 33:                          // Get pressure BME280 sensor
      replystr = "e" + String(bme280pressure) + "#";
      Serial.print(replystr);
      break;
    case 34:                          // Get temperature BME280 sensor
      replystr = "c" + String(bme280temperature, 2) + "#";
      Serial.print(replystr);
      break;
    case 35:                          // Get dewpoint
      replystr = "d" + String(dewpoint, 2) + "#";
      Serial.print(replystr);
      break;
  }
#ifdef useGPSNEO
  GPS.pause(false);
#endif
}

// calculate LUX reading from LDR value
void getlux()
{
#ifdef useGPSNEO
  GPS.pause(true);
#endif
#ifdef TSL2561SENSOR
  uint32_t lum = myTSL2561.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  lux = myTSL2561.calculateLux(full, ir);
#else
  // portions of code from https://www.digikey.com/en/maker/projects/design-a-luxmeter-with-an-ldr-and-an-arduino/623aeee0f93e427bb57e02c4592567d1
  int ldrRawData;
  float resistorVoltage, ldrVoltage, ldrResistance;
  ldrRawData = analogRead(LDRpin);                  // Read the analogue pin, returns value 0-1023
  resistorVoltage = (float)ldrRawData / 1023 * 5;
  ldrVoltage = 5.0 - resistorVoltage;
  ldrResistance = ldrVoltage / resistorVoltage * 10000; // Resistor is 10 kohm
  lux = (double)12518931 * pow(ldrResistance, -1.405);
#endif
#ifdef useGPSNEO
  GPS.pause(false);
#endif
}

#ifdef useGPSNEO
// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect)
{
  char c = GPS.read();
}
#endif

#ifdef OLEDDISPLAY
void showpageoled()
{
  String tempStr;
  char tempstr[buf_size];
  char sqmstr[] = "SQM";

#ifdef useGPSNEO
  GPS.pause(true);
#endif
  // can only be called if displayenabled is true
  myoled.clear();
  switch ( lcdpage )
  {
    case 1:
      myoled.set2X();
      myoled.println(sqmstr);
      myoled.println(mySQMreading);
      lcdpage = 2;
      break;
    case 2:
#ifdef useGPSNEO
      myoled.set1X();
      // date and time
      myoled.print("DATE: ");
      myoled.print(GPS.day, DEC);
      myoled.print('/');
      myoled.print(GPS.month, DEC);
      myoled.print("/20");
      myoled.println(GPS.year, DEC);
      myoled.print("TIME: ");
      myoled.print(GPS.hour, DEC);
      myoled.print(':');
      myoled.print(GPS.minute, DEC);
      myoled.print(':');
      myoled.println(GPS.seconds, DEC);
      // latitude and longitude
      myoled.print("LAT : ");
      myoled.print(GPS.latitude / 100);
      if ( GPS.lat == 'N' )
        myoled.println(" N");
      else
        myoled.println(" S");
      myoled.print("LON : ");
      myoled.print(GPS.longitude / 100);
      if ( GPS.lon == 'W' )
        myoled.println(" W");
      else
        myoled.println(" E");
      // satellites
      myoled.print("SAT : ");
      myoled.println( GPS.satellites );
      myoled.print("ALT : ");
      myoled.print((int)GPS.altitude);
      myoled.println(" M");
#else
      myoled.print("GPS NOT ENABLED");
#endif
      lcdpage = 3;
      break;
    case 3:
      myoled.set1X();
      myoled.print("NELM       = ");
      myoled.println(nelm);
      // ir sensor
      myoled.print("IR AMBIENT = ");
      myoled.println(mlx90614ambient);
      myoled.print("IR OBJECT  = ");
      myoled.println(mlx90614object);
      // LUX
      memset(tempstr, 0, buf_size);
      myoled.print("LUX        = ");
      myoled.println(lux);
      // raining
      myoled.print("RAINING    = ");
      if ( raining == false )
        myoled.println("NO");
      else
        myoled.println("YES");
      // raining Vout, resolution accuracy is 4.9 mV
      volts = map(rainVout, 0, 1023, 0, 5000 );
      myoled.print("RAIN VOUT  = ");
      myoled.print( volts);
      myoled.println(" mV");
      // setpoint1 and 2
      myoled.print("SP1        = ");
      myoled.println(setpoint1);
      myoled.print("SP2        = ");
      myoled.println(setpoint2);
      lcdpage = 4;
      break;
    case 4:
      // sky state
      switch ( skystate )
      {
        case SKYCLEAR:   myoled.println("CLEAR"); break;
        case SKYPCLOUDY: myoled.println("PARTLY CLOUDY"); break;
        case SKYCLOUDY:  myoled.println("CLOUDY"); break;
        default:         myoled.println("UNKNOWN"); break;
      }
      // bme280 data, ambient, humidity, dewpoint, barometric pressure
      myoled.print("HUMIDITY : ");
      myoled.println(bme280humidity);
      myoled.print("AMBIENT  : ");
      myoled.println(bme280temperature);
      myoled.print("DEWPOINT : ");
      myoled.println(dewpoint);
      myoled.print("HPA      : ");
      myoled.println(bme280pressure);
      lcdpage = 1;
      break;
  }
#ifdef useGPSNEO
  GPS.pause(false);
#endif
}
#endif

#ifdef LCDDISPLAY
#ifdef LCD2004
void showpage2004()
{
#ifdef useGPSNEO
  GPS.pause(true);
#endif
  switch ( lcdpage )
  {
    case 1:
      lcd.print("SQM : ");
      lcd.print(mySQMreading);
      lcd.setCursor(0, 1);
      lcd.print("NELM: ");
      lcd.print(nelm);
#ifdef useGPSNEO
      lcd.setCursor(0, 2);
      lcd.print("Date: ");
      lcd.print(GPS.day, DEC);
      lcd.print('/');
      lcd.print(GPS.month, DEC);
      lcd.print("/20");
      lcd.print(GPS.year, DEC);
      lcd.setCursor(0, 3);
      lcd.print("Time: ");
      lcd.print(GPS.hour, DEC);
      lcd.print(':');
      lcd.print(GPS.minute, DEC);
      lcd.print(':');
      lcd.print(GPS.seconds, DEC);
#endif
      lcdpage = 2;
      break;
    case 2:
#ifdef useGPSNEO
      // latitude and longitude
      lcd.print("Lat : ");
      lcd.print(GPS.latitude / 100);
      if ( GPS.lat == 'N' )
        lcd.print(" N");
      else
        lcd.print(" S");
      lcd.setCursor(0, 1);
      lcd.print("Lon : ");
      lcd.print(GPS.longitude / 100);
      if ( GPS.lon == 'W' )
        lcd.print(" W");
      else
        lcd.print(" E");
      lcd.setCursor(0, 2);
      // satellites
      lcd.print("Sat : ");
      lcd.print( GPS.satellites );
      lcd.setCursor(0, 3);
      lcd.print("Alt : ");
      lcd.print((int)GPS.altitude);
      lcd.print("m");
#else
      lcd.print("GPS not enabled");
#endif
      lcdpage = 3;
      break;
    case 3:
      lcd.print("IRAmb : ");
      lcd.print(mlx90614ambient);
      lcd.print("c");
      lcd.setCursor(0, 1);
      lcd.print("IRObj : ");
      lcd.print(mlx90614object);
      lcd.print("c");
      lcd.setCursor(0, 2);
      // raining
      lcd.print("Rain? : ");
      if ( raining == false ) {
        lcd.print("NO");
      }
      else {
        lcd.print("YES");
      }
      lcd.setCursor(0, 3);
      volts = map(rainVout, 0, 1023, 0, 5000 );
      lcd.print("RVout : ");
      lcd.print(volts);
      lcd.print("mV");
      lcdpage = 4;
      break;
    case 4:
      // LUX
      lcd.print("Lux : " );
      lcd.print(lux);
      lcd.setCursor(0, 1);
      lcd.print("SKY : ");
      // Sky state
      switch ( skystate )
      {
        case SKYCLEAR:   lcd.print("CLEAR"); break;
        case SKYPCLOUDY: lcd.print("PART CLOUDY"); break;
        case SKYCLOUDY:  lcd.print("CLOUDY"); break;
        default:         lcd.print("UNKNOWN"); break;
      }
      // setpoint1 and 2
      lcd.setCursor(0, 2);
      lcd.print("SP1 : ");
      lcd.print(setpoint1);
      lcd.setCursor(0, 3);
      lcd.print("SP2 : ");
      lcd.print(setpoint2);
      lcdpage = 5;
      break;
    case 5:
      // bme280 data, ambient, humidity, dewpoint, barometric pressure
      lcd.print("Hum: ");
      lcd.print(bme280humidity);
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("Amb: ");
      lcd.print(bme280temperature);
      lcd.print("c");
      lcd.setCursor(0, 2);
      lcd.print("Dew: ");
      lcd.print(dewpoint);
      lcd.print("c");
      lcd.setCursor(0, 3);
      lcd.print("hPa: ");
      lcd.print(bme280pressure);
      lcdpage = 1;
      break;
  }
#ifdef useGPSNEO
  GPS.pause(false);
#endif
}
#endif
#endif

#ifdef LCDDISPLAY
#ifdef LCD1604
void showpage1604()
{
#ifdef useGPSNEO
  GPS.pause(true);
#endif
  switch ( lcdpage )
  {
    case 1:
      lcd.print("SQM : ");
      lcd.print(mySQMreading);
      lcd.setCursor(0, 1);
      lcd.print("NELM: ");
      lcd.print(nelm);
#ifdef useGPSNEO
      lcd.setCursor(0, 2);
      lcd.print("Date: ");
      lcd.print(GPS.day, DEC);
      lcd.print('/');
      lcd.print(GPS.month, DEC);
      lcd.print("/20");
      lcd.print(GPS.year, DEC);
      lcd.setCursor(0, 3);
      lcd.print("Time: ");
      lcd.print(GPS.hour, DEC);
      lcd.print(':');
      lcd.print(GPS.minute, DEC);
      lcd.print(':');
      lcd.print(GPS.seconds, DEC);
#endif
      lcdpage = 2;
      break;
    case 2:
#ifdef useGPSNEO
      // latitude and longitude
      lcd.print("Lat : ");
      lcd.print(GPS.latitude / 100);
      if ( GPS.lat == 'N' )
        lcd.print(" N");
      else
        lcd.print(" S");
      lcd.setCursor(0, 1);
      lcd.print("Lon : ");
      lcd.print(GPS.longitude / 100);
      if ( GPS.lon == 'W' )
        lcd.print(" W");
      else
        lcd.print(" E");
      lcd.setCursor(0, 2);
      // satellites
      lcd.print("Sat : ");
      lcd.print( GPS.satellites );
      lcd.setCursor(0, 3);
      lcd.print("Alt : ");
      lcd.print((int)GPS.altitude);
      lcd.print("m");
#else
      lcd.print("GPS not enabled");
#endif
      lcdpage = 3;
      break;
    case 3:
      lcd.print("IRAmb : ");
      lcd.print(mlx90614ambient);
      lcd.print("c");
      lcd.setCursor(0, 1);
      lcd.print("IRObj : ");
      lcd.print(mlx90614object);
      lcd.print("c");
      lcd.setCursor(0, 2);
      // raining
      lcd.print("Rain? : ");
      if ( raining == false )
        lcd.print("NO");
      else
        lcd.print("YES");
      lcd.setCursor(0, 3);
      volts = map(rainVout, 0, 1023, 0, 5000 );
      lcd.print("RVout : ");
      lcd.print(volts);
      lcd.print("mV");
      lcdpage = 4;
      break;
    case 4:
      // LUX
      lcd.print("Lux : " );
      lcd.print(lux);
      lcd.setCursor(0, 1);
      lcd.print("SKY : ");
      // Sky state
      switch ( skystate )
      {
        case SKYCLEAR:   lcd.print("CLEAR"); break;
        case SKYPCLOUDY: lcd.print("PCLOUDY"); break;
        case SKYCLOUDY:  lcd.print("CLOUDY"); break;
        default:         lcd.print("UNKNOWN"); break;
      }
      // setpoint1 and 2
      lcd.setCursor(0, 2);
      lcd.print("SP1 : ");
      lcd.print(setpoint1);
      lcd.setCursor(0, 3);
      lcd.print("SP2 : ");
      lcd.print(setpoint2);
      lcdpage = 5;
      break;
    case 5:
      // bme280 data, ambient, humidity, dewpoint, barometric pressure
      lcd.print("Hum: ");
      lcd.print(bme280humidity);
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("Amb: ");
      lcd.print(bme280temperature);
      lcd.print("c");
      lcd.setCursor(0, 2);
      lcd.print("Dew: ");
      lcd.print(dewpoint);
      lcd.print("c");
      lcd.setCursor(0, 3);
      lcd.print("hPa: ");
      lcd.print(bme280pressure);
      lcdpage = 1;
      break;
  }
#ifdef useGPSNEO
  GPS.pause(false);
#endif
}
#endif
#endif

#ifdef LCDDISPLAY
#ifdef LCD1602
void showpage1602()
{
#ifdef useGPSNEO
  GPS.pause(true);
#endif
  switch ( lcdpage )
  {
    case 1:
      lcd.print("SQM : ");
      lcd.print(mySQMreading);
      lcd.setCursor(0, 1);
      lcd.print("NELM: ");
      lcd.print(nelm);
      lcdpage = 2;
      break;
    case 2:
#ifdef useGPSNEO
      lcd.print("Date: ");
      lcd.print(GPS.day, DEC);
      lcd.print('/');
      lcd.print(GPS.month, DEC);
      lcd.print("/20");
      lcd.print(GPS.year, DEC);
      lcd.setCursor(0, 1);
      lcd.print("Time: ");
      lcd.print(GPS.hour, DEC);
      lcd.print(':');
      lcd.print(GPS.minute, DEC);
      lcd.print(':');
      lcd.print(GPS.seconds, DEC);
#endif
      lcdpage = 3;
      break;
    case 3:
#ifdef useGPSNEO
      // latitude and longitude
      lcd.print("Lat : ");
      lcd.print(GPS.latitude / 100);
      if ( GPS.lat == 'N' )
        lcd.print(" N");
      else
        lcd.print(" S");
      lcd.setCursor(0, 1);
      lcd.print("Lon : ");
      lcd.print(GPS.longitude / 100);
      if ( GPS.lon == 'W' )
        lcd.print(" W");
      else
        lcd.print(" E");
      lcdpage = 4;
      break;
    case 4:
      // satellites
      lcd.print("Sat : ");
      lcd.print( GPS.satellites );
      lcd.setCursor(0, 1);
      lcd.print("Alt : ");
      lcd.print((int)GPS.altitude);
      lcd.print("m");
      lcdpage = 5;
      break;
#else
      lcd.print("GPS not enabled");
#endif
      lcdpage = 5;
      break;
    case 5:
      lcd.print("IRAmb : ");
      lcd.print(mlx90614ambient);
      lcd.print("c");
      lcd.setCursor(0, 1);
      lcd.print("IRObj : ");
      lcd.print(mlx90614object);
      lcd.print("c");
      lcdpage = 6;
      break;
    case 6:
      // raining
      lcd.print("Rain? : ");
      if ( raining == false )
        lcd.print("NO");
      else
        lcd.print("YES");
      lcd.setCursor(0, 1);
      // raining Vout, resolution accuracy is 4.9 mV
      volts = map(rainVout, 0, 1023, 0, 5000 );
      lcd.print("RVout : ");
      lcd.print(volts);
      lcd.print("mV");
      lcdpage = 7;
      break;
    case 7:
      lcd.print("Lux : " );
      lcd.print(lux);
      lcd.setCursor(0, 1);
      lcd.print("SKY : ");
      switch ( skystate )
      {
        case SKYCLEAR:   lcd.print("CLEAR"); break;
        case SKYPCLOUDY: lcd.print("PART CLOUDY"); break;
        case SKYCLOUDY:  lcd.print("CLOUDY"); break;
        default:         lcd.print("UNKNOWN"); break;
      }
      lcdpage = 8;
      break;
    case 8:
      // setpoint1 and 2
      lcd.print("SP1 : ");
      lcd.print(setpoint1);
      lcd.setCursor(0, 1);
      lcd.print("SP2 : ");
      lcd.print(setpoint2);
      lcdpage = 9;
      break;
    case 9:
      // bme280 data, ambient, humidity, dewpoint, barometric pressure
      lcd.print("Hum: ");
      lcd.print(bme280humidity);
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("Amb: ");
      lcd.print(bme280temperature);
      lcd.print("c");
      lcdpage = 10;
      break;
    case 10:
      // bme280 data, ambient, humidity, dewpoint, barometric pressure
      lcd.print("Dew: ");
      lcd.print(dewpoint);
      lcd.print("c");
      lcd.setCursor(0, 1);
      lcd.print("hPa: ");
      lcd.print(bme280pressure);
      lcdpage = 1;
      break;
  }
#ifdef useGPSNEO
  GPS.pause(false);
#endif
}
#endif
#endif

// Chris
void jsonsend()
{ 
  StaticJsonBuffer<500> jsonBuffer;
  JsonObject& root = jsonBuffer.createObject();
    root["data1"]  = mySQMreading;
    root["data2"]  = nelm;
    root["data3"]  = mlx90614ambient;  
    root["data4"]  = mlx90614object;  
    root["data5"]  = raining;   
    root["data6"]  = volts;   
    root["data7"]  = lux;
    root["data8"]  = skystate;  
    root["data9"]  = setpoint1;
    root["data10"] = setpoint2;
    root["data11"] = bme280humidity;
    root["data12"] = bme280temperature;  
    root["data13"] = dewpoint;
    root["data14"] = bme280pressure;
  if(s.available()>0)
  {
  root.printTo(s);
  }
}

void setup()
{
  Serial.begin(9600);
  clearSerialPort();
  s.begin(9600); // Chris - your esp's baud rate communication to NodeMCU

#ifdef useGPSNEO
  ss.begin(GPSPortSpeed);
  delay(200);
  GPS.begin(GPSPortSpeed);          // start GPS device
  // There needs to be a delay before sending an commands to GPS after a GPS.begin()
  delay(200);   // delay 100mS, minimum 10mS is required
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_ALLDATA);
  GPS.pause(true);
#endif

  pinMode( rs_DO, INPUT );                    // rain sensor

  mySQMreading = 0.0;                         // the SQM value, sky magnitude
  frequency = 1.0;                            // measured TLS237 frequency which is dependent on light
  irradiance = 1.0;
  nelm = 1.0;
  setpoint1 = 22;                             // setpoint values used to determine sky state
  setpoint2 = 2;
  sensorsamplerequest = false;
  dewpoint = 10.0;
  bme280temperature = 20.0;
  bme280humidity = 50.0;
  bme280pressure = 1100;
  mlx90614ambient = 20.0;
  mlx90614object = 20.0;
  lux = 1000;
  skystate = SKYCLEAR;
  raining = false;
  rainVout = 0;

  eoc = 0;
  idx = 0;
  memset(line, 0, MAXCOMMAND);

#ifdef LCDDISPLAY
#ifdef LCD1602
  lcd.begin(16, 2);
#endif
#ifdef LCD1604
  lcd.begin(16, 4);
#endif
#ifdef LCD2004
  lcd.begin(20, 4);
#endif
  lcdpage = 1;
  lcd.setBacklight(HIGH);
  lcd.print(programName);
  lcd.setCursor(0, 1);
  lcd.print("Version: ");
  lcd.print(programVersion);
  lcd.setCursor(0, 2);
  lcd.print("(c) R Brown 2018");
#endif

#ifdef OLEDDISPLAY
  lcdpage = 1;
  Wire.begin();
  myoled.begin(&Adafruit128x64, OLEDADDRESS);
  myoled.set400kHz();
  myoled.setFont(Adafruit5x7);
  myoled.clear();                              // clrscr OLED
  myoled.Display_Normal();                     // black on white
  myoled.Display_On();                         // display ON
  myoled.Display_Rotate(0);                    // portrait, not rotated
  myoled.Display_Bright();
  // The screen size is 128 x 64, so using characters at 6x8 this gives 21chars across and 8 lines down
  myoled.println(programName);
  myoled.println(programVersion);
  myoled.InverseCharOn();
  myoled.println("(C) R BROWN.");
  myoled.InverseCharOff();
#endif

#ifdef BME280SENSOR
  mybme280.begin(BME280_I2CADDR);
  mybme280.init();
#endif

#ifdef MLS90614SENSOR
  mymlx90614.begin();
#endif

#ifdef TSL2561SENSOR
  myTSL2561.begin();
  myTSL2561.setGain(TSL2561_GAIN_16X);                    // set 16x gain (for dim situations)
  myTSL2561.setTiming(TSL2561_INTEGRATIONTIME_13MS);      // shortest integration time (bright light)
#endif

#ifdef useGPSNEO
  useInterrupt(true);
  GPS.pause(false);
#endif
  lcdsampletimer = sensorsampletimer = sqmsampletimer = millis();
}

void loop()
{
 if ( queue.count() >= 1 )             // check for serial command
 {
    processCommand();
 }
int chris;
#ifdef useGPSNEO
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }
#endif

  // refresh sensor readings
  now = millis();
  if ( ((now - sensorsampletimer ) > SENSORSAMPLETIME ) || (now < sensorsampletimer) )
  {
    sensorsampletimer = now;
    if ( sensorsamplerequest == false )
    {
      // update bmesensor
#ifdef BME280SENSOR
      mybme280.readData();
      mybme280.getValues(&bme280temperature, &bme280humidity, &bme280pressure);
      calc_dewpoint((float) bme280temperature, (float) bme280humidity);
      chris=bme280pressure;
#endif
      // update lux
      getlux();
      // get raining
      getraining();
      volts = map(rainVout, 0, 1023, 0, 5000 );
      sensorsamplerequest = true;
    }
    else
    {
      // update MLX90614 and rainsensor
#ifdef MLX90614SENSOR
      mlx90614ambient = mymlx90614.readAmbientTempC();
      mlx90614object = mymlx90614.readObjectTempC();
#endif
      // update skystate
      getskystate();
      sensorsamplerequest = false;
    }
  }
      
  // refresh sqm readings
  now = millis();
  // take a sample every 5 seconds
  if ( ((now - sqmsampletimer ) > SQMSAMPLETIME ) || (now < sqmsampletimer) )
  {
    sqmsampletimer = now;
    int LDRval = analogRead( LDRpin );        // read LDR to determine background light level
    if ( LDRval < LDRCutoff1 )                // and use this to set the Gate Time (duration) for this samples frequency measurement
    {
      period = blackperiod;                   // its very dark
    }
    else if ( LDRval < LDRCutoff2 )
    {
      period = darkperiod;                    // its dark
    }
    else
      period = lightperiod;                   // its light

    light();                                  // read frequency from sensor
#ifdef TSL235
    frequency = frequency * 7.2;
#endif
    irradiance = frequency / 2.3e3;           // calculate irradiance as uW/(cm^2)
#ifdef OLDSQMMETHOD
    mySQMreading = ((sqm_limit - (2.5 * log10( (frequency * 1.584) / 2.0 )))) + 1.0; // frequency to magnitudes/arcSecond^2
#endif
#ifdef NEWSQMMETHOD
    mySQMreading = sqm_limit - 2.5 * log10 (frequency);
#endif
#ifdef NEWSQMMETHODCORRECTED
    // 0.973 was derived by comparing TLS237 sensor readings against Unihedron and plotting on graph then deriving coefficient
    mySQMreading = (sqm_limit - (2.5 * log10( frequency )) * 0.973);                   // frequency to magnitudes/arcSecond2
#endif
#ifdef IRRADIANCEMETHOD
    mySQMreading = log10((irradiance / 6.83) / 108000) / -0.4;                        // mag/arcsec^2
#endif

    nelm = 7.93 - 5.0 * log10((pow(10, (4.316 - (mySQMreading / 5.0))) + 1));

 //chris  Send Information to NodeMCU for WIfi 
  jsonsend();
  }

  now = millis();
  // update LCD Screen every 4.5 seconds
  if ( ((now - lcdsampletimer ) > LCDUPDATETIME ) || (now < lcdsampletimer) )
  {
    lcdsampletimer = now;
#ifdef LCDDISPLAY
    // now update screens
    lcd.clear();
#ifdef LCD1602
    showpage1602();
#endif
#ifdef LCD1604
    showpage1604();
#endif
#ifdef LCD2004
    showpage2004();
#endif
#endif
#ifdef OLEDDISPLAY
    showpageoled();
#endif
  }
}

void light()
{
  long  pulses = 1L;

#ifdef useGPSNEO
  GPS.pause(true);
#endif
  FreqCounter::f_comp = 0;      // Cal Value / Calibrate with professional Freq Counter
  FreqCounter::start(period);   // Gate Time
  while (FreqCounter::f_ready == 0)
    pulses = FreqCounter::f_freq;
  delay(20);
  frequency = ((double)pulses * (double)1000.0) / (double) period;
  // if cannot measure the number of pulses set to freq 1hz which is mag 21.95
  if ( frequency < 1.0 )
    frequency = 1.0;
#ifdef useGPSNEO
  GPS.pause(false);
#endif
}

void clearSerialPort()
{
  while ( Serial.available() )
    Serial.read();
}

// SerialEvent occurs whenever new data comes in the serial RX.
void serialEvent()
{
#ifdef useGPSNEO
  GPS.pause(true);
#endif
  // : starts the command, # ends the command, do not store these in the command buffer
  // read the command until the terminating # character
  while (Serial.available() && !eoc)
  {
    char inChar = Serial.read();
    if (inChar != '#' && inChar != ':')
    {
      line[idx++] = inChar;
      if (idx >= MAXCOMMAND)
      {
        idx = MAXCOMMAND - 1;
      }
    }
    else
    {
      if (inChar == '#')
      {
        eoc = 1;
        idx = 0;
        queue.push(String(line));
        eoc = 0;
        memset( line, 0, MAXCOMMAND);
      }
    }
  }
#ifdef useGPSNEO
  GPS.pause(false);
#endif
}

#ifdef useGPSNEO
void useInterrupt(boolean v) {
  if (v) {
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}
#endif
