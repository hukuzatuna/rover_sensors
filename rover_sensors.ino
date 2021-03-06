
//-------------------------------------------------------------------------
// Autonomous robot rover - sensor/comms/control platform
//
// Philip R. Moyer
// November 2015 - January 2016
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
// Preprocessor directives
//-------------------------------------------------------------------------

#include <Wire.h>
#include <Adafruit_Sensor.h>      // 10DoF support
#include <Adafruit_LSM303_U.h>    // 10DoF support - accelerometer/magnetometer
#include <Adafruit_BMP085_U.h>    // 10DoF support - barometric pressure
#include <Adafruit_L3GD20_U.h>    // 10DoF support - gyroscope
#include <Adafruit_10DOF.h>       // 10DoF support
#include <Adafruit_TSL2561_U.h>   // TSL2561 digital light/lux sensor
#include <Adafruit_SI1145.h>      // SI1145 UV/Visible/IR level sensor
#include <Adafruit_HTU21DF.h>     // HTU21DF temp/humidity sensor
#include <LIDARLite.h>            // LIDARLite v2 rangefinder sensor
#include "Rover.h"                // Rover class and method definitions
#include "RTClib.h"               // Real time clock
#include <Adafruit_PWMServoDriver.h>  // Servo controller board (at I2C 0x41)
#include <SPI.h>                  // Hardware SPI support
#include <string.h>               // String library


//-------------------------------------------------------------------------
// Variables and constants
//-------------------------------------------------------------------------

// Initialize sensor objects
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified();
Adafruit_TSL2561_Unified      tsl   = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_SI1145               uv    = Adafruit_SI1145();
Adafruit_HTU21DF              htu   = Adafruit_HTU21DF();
LIDARLite                     myLidarLite01;
RTC_DS1307                    rtc;

// Constants and flags

const bool UseLIDAR            = true;      // Use LIDAR sensors?
const bool UseSharpIR          = false;     // Use Sharp IR rangefinders?
const bool debug               = false;     // Produce debugging output on default Serial?
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
// Use care when changing PWM settings as overdriving the servos can strip the gears!
const int SERVOMIN             = 140;       // Minimum servo PWM pulse length (out of 4096)
const int SERVOMAX             = 600;       // Maximum servo PWM pulse length (out of 4096)

// Sharp IR Rangefinder setup - array of 15 rangefinders for hazard avoidance
//
//Mux control pins
int s0 = 2;
int s1 = 3;
int s2 = 4;
int s3 = 5;
//Mux in "SIG" pin
int SIG_pin = 0;
// Table of IR rangefinder values
int oldIRvalues[16];        /* Previous values of IR rangefinders */
int newIRvalues[16];        /* Current values of IR rangefinders */
// Parameters
float IRthresh = 30.0;      /* Difference threashold at which IR sensors trigger a warning */
//
// End Sharp IR Rangefinder setup

// Initializtaion of global variables

sensors_event_t       event;                        // Storage location for data collection
bool                  HA_configured     = false;    // Hazard avoidance system configured?
bool                  IRstarted         = false;    // Have we cycled through rangefinding data at least once?
bool                  LIDARstarted      = false;    // Have we cycled through a LIDAR scan yet?
bool                  dualCPUenabled    = false;    // Are comms between CPUs working?
bool                  rtcUp             = false;    // Is the mission clock running?
bool                  sentHeader        = false;   // Have we sent the science data header yet?
Adafruit_PWMServoDriver pwmServos01 = Adafruit_PWMServoDriver(0x41); // Initialize the servo controller - I2C addr 0x041
char dataBuffer[255];                               // data buffer for communications
Rover_data curRoverData = Rover_data();             // Object to hold and manipulate science and nav data

// COMMMS CONFIG - Currently using cellular. Will eventually contain setup for amatuer radio card.

#define WLAN_SERVER_NAME  "hukuzatuna.ddns.net"       // Communications server.
bool commsUp = false;                                 // Flag indicating whether we have radio comms working
bool useCC3000 = false;                               // Whether we should use the CC3000 WiFi
bool useRSM22 = false;                                // Whether we should use the RSM22 radio board
bool useEthernet = false;                             // Whether to use an Ethernet card
bool useFONA = true;														      // Whether to use the FONA GSM card
const unsigned int commsPort = 8088;                  // TCP port for communications server

// END COMMS CONFIG



//-------------------------------------------------------------------------
// Functions
//-------------------------------------------------------------------------

// configureTLSSensor - sets gain and integration time on TSL2561 sensor board

void configureTSLSensor(void)
{
  tsl.enableAutoRange(true);    /* switches gain automatically for light conditions */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);  /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16 bit high resolution */
}


// Prints a hash followed by a date/time stamp on Serial

void printDateStamp(void)
{
  char dateStamp[64];           // Datestamp buffer
  
  DateTime now = rtc.now();
  sprintf(dateStamp, "# %4d/%.2d/%.2d %.2d:%.2d:%.2d ", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  Serial.print(dateStamp);
}


// Perform a 180 degree x 90 degree LIDAR scan of the environment
// NEEDS TO BE WRITTEN CORRECTLY. -PRM

bool doLIDARscan(Adafruit_PWMServoDriver curServo)
{
  // Temporary code
  long LIDARrange = myLidarLite01.distance();
  char curLrange[20];
  sprintf(curLrange, "LIDAR: %d", LIDARrange);
  consoleMessage(curLrange);
  return(true);
  // END temporary code

}

// Read SparkFun 16 channel A2D multiplexer (for Sharp IR rangefinders)

int readMux(int channel){
  int controlPin[] = {s0, s1, s2, s3};

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

  //loop through the 4 sig
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  //read the value at the SIG pin
  int val = analogRead(SIG_pin);

  //return the value
  return val;
}

// Send science and nav data to mission operations center.
bool xmitData()
{
	bool rv = true;           // return value

	// Print data line on serial console.
	Serial.print(curRoverData.getAccelX()); Serial.print(",");
	Serial.print(curRoverData.getAccelY()); Serial.print(",");
	Serial.print(curRoverData.getAccelZ()); Serial.print(",");
	Serial.print(curRoverData.getMagX()); Serial.print(",");
	Serial.print(curRoverData.getMagY()); Serial.print(",");
	Serial.print(curRoverData.getMagZ()); Serial.print(",");
	Serial.print(curRoverData.getOrientRoll()); Serial.print(",");
	Serial.print(curRoverData.getOrientPitch()); Serial.print(",");
	Serial.print(curRoverData.getOrientHeading()); Serial.print(",");
	Serial.print(curRoverData.getGyroX()); Serial.print(",");
	Serial.print(curRoverData.getGyroY()); Serial.print(",");
	Serial.print(curRoverData.getGyroZ()); Serial.print(",");
	Serial.print(curRoverData.getTempA()); Serial.print(",");
	Serial.print(curRoverData.getTempB()); Serial.print(",");
	Serial.print(curRoverData.getTempC()); Serial.print(",");
	Serial.print(curRoverData.getDisitance()); Serial.print(",");
	Serial.print(curRoverData.getLight()); Serial.print(",");
	Serial.print(curRoverData.getLightVisible()); Serial.print(",");
	Serial.print(curRoverData.getLightIR()); Serial.print(",");
	Serial.print(curRoverData.getLightUV()); Serial.print(",");
	Serial.print(curRoverData.getLightUVindex()); Serial.print(",");
	Serial.print(curRoverData.getPressure()); Serial.print(",");
	Serial.print(curRoverData.getRH()); Serial.print(",");
	Serial.print(curRoverData.getCurrent()); Serial.print(",");
	Serial.print(curRoverData.getVoltage()); Serial.print(",");
	Serial.print(curRoverData.getGPSlat()); Serial.print(",");
  Serial.print(curRoverData.getGPSlong()); Serial.print(",");
  Serial.print(curRoverData.getGPSalt()); Serial.print(",");
  Serial.print(curRoverData.getGPShours()); Serial.print(",");
  Serial.print(curRoverData.getGPSminutes()); Serial.print(",");
  Serial.print(curRoverData.getGPSseconds()); Serial.print(",");
  Serial.print(curRoverData.getGPSday()); Serial.print(",");
  Serial.print(curRoverData.getGPSmonth()); Serial.print(",");
  Serial.print(curRoverData.getGPSyear()); Serial.print(",");
  Serial.print(curRoverData.getGPSsats()); Serial.print(",");
  Serial.print(curRoverData.getGPSspeed()); Serial.print(",");
  Serial.print(curRoverData.getGPSheading()); Serial.print("\n");
  Serial.flush();
  
  return rv;
}

// Print console messages 
void consoleMessage(const String cText)
{
  printDateStamp(); Serial.print("CPU A: "); Serial.println(cText);
  Serial.flush();
}


// NOTE: THESE NEED TO BE WRITTEN PROPERLY. THESE ARE JUST PLACEHOLDERS. -PRM
// Exercise the mast servos on startup
void exerciseFrontMastServos()
{
	return true;
}

void exerciseRearMastServos()
{
	return true;
{


//-------------------------------------------------------------------------
// Main setup and loop
//-------------------------------------------------------------------------

void setup() {
  // put your setup code here, to run once:
  uint8_t cnt = 0;
  
  Serial.begin(115200);   /* This will need to be reduced when using packet data over radio - probably to 9600 */
  Serial.println("# Laika Rover Initialization Sequence Start");
  Serial.println("");
  Serial.println("# CPU A STARTUP");
  Serial.println("# CPU A Boot sequence start.");

  // Real time clock setup
  rtc.begin();
  if (! rtc.isrunning()) {
    Serial.println("# CPU A RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    rtcUp = false;
  }
  else
  {
    rtcUp = true;
  }
  printDateStamp(); Serial.println("RTC mission clock running.");

  consoleMessage("Console messages configured on default software Serial.");
  Serial3.begin(115200);    // Communications to Drive/Nav Mega 2560 on RX3/TX3.
  consoleMessage("Dual CPU communications configuring on Serial3.");

  
  // START COMMUNICATIONS WITH ARDUINO B

  consoleMessage("Arduino A initializing communication with Arduino B.");
  int loopCount = 0;              // Loop counter to get out of the while loop
  while (!dualCPUenabled)
  {
    Serial3.println("COMM CHECK");
    // Listen for response on Serial3
    if (Serial3.available())
    {
      Serial3.readBytesUntil('\n', dataBuffer, sizeof(dataBuffer));
      // If received "OK"
      if (strncmp("OK", dataBuffer, 2) == 0)
      {
        dualCPUenabled = true;
        consoleMessage("... Arduino B responding.");
        break;
      }
    }
    // If loop counter exceeds 100
    if (100 <= loopCount)
    {
      // Comms not up
      consoleMessage("... loop count limit exceeded. Dual CPU not up. Proceeding.");
			dualCPUenabled = false;		// should already be set, but just to be sure...
      break;
    }
    delay(100);
    loopCount += 1;
  }

  // END CPU COMMUNICATION STARTUP

  
  // SCIENCE PACKAGE SETUP
  
  consoleMessage("Science package startup sequence.");
  
  /* Set up sensors on the 10DoF breakout. */
  if (!accel.begin())
  {
    consoleMessage("No LSM303 detected - accelerometer.");
    cnt = 1;
  }
  else
  {
    consoleMessage(" ... LSM303 accelerometer OK.");
  }
  
  if (!mag.begin()) {
    consoleMessage(" No LSM303 detected - magnetometer.");
    cnt = 1;
  }
  else
  {
    consoleMessage(" ... LSM303 magnetometer OK.");
  }
  
  if (!bmp.begin())
  {
    consoleMessage(" No BMP085 detected - barometer.");
    cnt = 1;
  }
  else
  {
    consoleMessage(" ... BMP085 pressure sensor OK.");
  }
  
  if (!gyro.begin())
  {
    consoleMessage(" No L3GD20 detected - gyroscope.");
    cnt = 1;
  }
  else
  {
    consoleMessage(" ... L3GD20 gyroscope OK.");
  }
  
  if (!tsl.begin())
  {
    consoleMessage(" No TSL2561 detected.");
    cnt = 1;
  }
  else
  {
    consoleMessage(" ... TSL2561 light level sensor OK.");
  }
  
  /* Initialie the TSL2561 light sensor gain and integration time */
  consoleMessage(" ... ... Configuring TSL2561 gain and integration time.");
  configureTSLSensor();

  if (!uv.begin())
  {
    consoleMessage(" No SI1145 detected.");
    cnt = 1;
  }
  else
  {
    consoleMessage(" ... SI1145 light sensor OK.");
  }
  
  /* Initialie the HTU21DF temperature and humidity sensor */
  if (!htu.begin()) 
  {
    consoleMessage(" No HTU21DF sensor!");
    cnt = 1;
  }
  else
  {
    consoleMessage(" ... HTU21D-F temp/humidity sensor OK.");
  }
  
  // if (0 < cnt)
  // {
    // while(1);
  // }

  // END SCIENCE PACKAGE SETUP



  // HAZARD AVOIDANCE SETUP

  consoleMessage(" Hazard Avoidance startup sequence.");
  if (UseSharpIR)
  {
    //Mux control pins
    s0 = 2;
    s1 = 3;
    s2 = 4;
    s3 = 5;

    //Mux in "SIG" pin
    SIG_pin = 0;

    // Parameters
    IRthresh = 30.0;      /* Difference threashold at which IR sensors trigger a warning */

    // Setup for Sharp IR Rangefinders
    // Arduino Mega 2560 through Sparkfun CD74HC4067 16 channel analog multiplexer.

    // ARduino Mega and Sparkfun CD74HC4067 16 ch MUX
    //    Sparkfun C0 through C15 to Sharp IR Rangefind white wire (or GND)
    //    Sparkfun GND to ground
    //    Sparkfun VCC  to 5v
    //    Sparkfun EN to GND
    //    Sparkfun S0 to Digital 2
    //    Sparkfun S1 to Digital 3
    //    Sparkfun S2 to Digital 4
    //    Sparkfun S3 to Digital 5
    //    Sparkfun SIG to Analog 0

    // Note: even with input channel grounded, spurious signals on input lines can result
    // in readings of up to 15 or 20, or thereabouts. Nominally, readings are zero, but othe
    // signals can creep in. Better to ignore unused channels in software. Channels connected
    // to the rangefinder white wires appear to be reliable. -PRM

    /* Wiring Order should be: */
    /* Front right up 45, front center up 45, front left up 45 */
    consoleMessage("... Initializing IR rangefinders.");
    uint8_t IRfru45, IRfcu45, IRflu45;
    IRfru45 = 0;
    IRfcu45 = 1;
    IRflu45 = 2;
    /* Front right, front center, front left */
    uint8_t IRfr, IRfc, IRfl;
    IRfr = 3;
    IRfc = 4;
    IRfl = 5;
    /* Front right down 45, frong center down 45, front left down 45 */
    uint8_t IRfrd45, IRfcd45, IRfld45;
    IRfrd45 = 6;
    IRfcd45 = 7;
    IRfld45 = 8;
    /* Rear right, rear center, rear left */
    uint8_t IRrr, IRrc, IRrl;
    IRrr = 9;
    IRrc = 10;
    IRrl= 11;
    /* Rear right down 45, rear center down 45, rear left down 45 */
    uint8_t IRrrd45, IRrcd45, IRrld45;
    IRrrd45 = 12;
    IRrcd45 = 13;
    IRrld45 = 14;

    // Initialize the MUX control pins
    pinMode(s0, OUTPUT); 
    pinMode(s1, OUTPUT); 
    pinMode(s2, OUTPUT); 
    pinMode(s3, OUTPUT); 

    // Select no channels at startup
    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);

    for (int i = 0; i < 16; i++)
    {
      oldIRvalues[i] = 0;
    }

    HA_configured = true;       /* Hazard avoidance configured flag */
  }

  /* LIDAR setup */
  if (UseLIDAR)
  {
    consoleMessage(" ... LIDAR initialization.");
    myLidarLite01.begin();
    consoleMessage(" ... ... front LIDAR.");

		// UNCOMMENT WHEN SECOND LIDAR UNIT INSTALLED. -PRM
    // Serial.println("# CPU A ... ... rear LIDAR.");
    HA_configured = true;       /* Hazard avoidance configured flag */
    LIDARstarted = true;        /* LIDAR started flag */
  }

  if (!HA_configured)
  {
    consoleMessage(" FAIL: no hazard avoidance system configured. Movement disabled!");
  }

  // END HAZARD AVOIDANCE SETUP

  // CAMERA SETUP
  consoleMessage(" Camera startup sequence.");

  // END CAMERA SETUP

  // PAN/TILT SETUP
  consoleMessage(" Pan/tilt mast startup sequence.");

  pwmServos01.begin();
  pwmServos01.setPWMFreq(60);
  // Note: on the front mast, the vertical movement is controlled by servo 0,
	// while horizontal movement is controlled by servo 1. On the rear mast, the
	// vertical movement is controlled by servo 2, and horizontal movement is
	// controlled by servo 3.

	if (exerciseFrontMastServos())
	{
  	consoleMessage(" ... front LIDAR mast.");
	}
	else
	{
		consoleMessage(" ... front LIDAR mast FAIL.");
	}
	if (exerciseRearMastServos())
	{
  	consoleMessage(" ... rear LIDAR mast.");
	}
	else
	{
		consoleMessage(" ... rear LIDAR mast FAIL.");
	}

  // END PAN/TILT SETUP
}


void loop() {
  // put your main code here, to run repeatedly:

  // SCIENCE PACKAGE
  
  /* Get events from 10DoF breakout and construct an output string */
  /* Output string will be stored on SD card, sent to serial console, and transmitted to motor controller */
  /* In the future, the science data will also be sent over the radio on request for science data. */
  
  accel.getEvent(&event);
  curRoverData.setAccelX(event.acceleration.x);
  curRoverData.setAccelY(event.acceleration.y);
  curRoverData.setAccelZ(event.acceleration.z);
  Serial3.print(event.acceleration.x); Serial3.print(",");
  Serial3.print(event.acceleration.y); Serial3.print(",");
  Serial3.print(event.acceleration.z); Serial3.print(",");

  mag.getEvent(&event);
  curRoverData.setMagX(event.magnetic.x);
  curRoverData.setMagY(event.magnetic.y);
  curRoverData.setMagZ(event.magnetic.z);
  Serial3.print(event.magnetic.x); Serial3.print(",");
  Serial3.print(event.magnetic.y); Serial3.print(",");
  Serial3.print(event.magnetic.z); Serial3.print(",");

  gyro.getEvent(&event);
  curRoverData.setGyroX(event.gyro.x);
  curRoverData.setGyroY(event.gyro.y);
  curRoverData.setGyroZ(event.gyro.z);
  Serial3.print(event.gyro.x); Serial3.print(",");
  Serial3.print(event.gyro.y); Serial3.print(",");
  Serial3.print(event.gyro.z); Serial3.print(",");  

  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    curRoverData.setPressure(event.pressure);
    Serial3.print(event.pressure); Serial3.print(",");
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    curRoverData.setTempA(temperature);
    Serial3.print(temperature); Serial3.print(",");
  }

  /* TSL2561 light conditions */
  tsl.getEvent(&event);
  if (event.light)
  {
    curRoverData.setLight(event.light);
    Serial3.print(event.light); Serial3.print(",");
  }

  /* SI1145 UV/visible/IR level sensor */
  curRoverData.setLightVisible(uv.readVisible());
  curRoverData.setLightIR(uv.readIR());
  curRoverData.setLightUV(uv.readUV());
  Serial3.print(uv.readVisible()); Serial3.print(",");
  Serial3.print(uv.readIR()); Serial3.print(",");
  float UVindex = uv.readUV();
  Serial3.print(UVindex); Serial3.print(",");
  UVindex /= 100.0;
  curRoverData.setLightUVindex(UVindex);
  Serial3.print(UVindex); Serial3.print(",");
  
  /* HTU21DF temperature/humidity sensor */
  curRoverData.setTempB(htu.readTemperature());
  curRoverData.setRH(htu.readHumidity());
  Serial3.print(htu.readTemperature()); Serial3.print(",");
  Serial3.println(htu.readHumidity());

  // Flush the serial buffer
  Serial3.flush();

	// NOTE: COMMUNICATIONS OF ALL DATA IS NOW HANDLED BY ARDUINO B SINCE ONE-WAY
	// COMMUNICATIONS OVER SERIAL3 IS SIMPLER THAN FULL DUPLEX, AND ARDUINO B
	// ALREADY HAS ALL THE DATA THAT NEED TO BE TRANSMITTED.

  // END SCIENCE PACKAGE SEGMENT

  // HAZARD AVOIDANCE/NAV
  if (debug) 
  {
    consoleMessage("Hazard avoidance code.");
  }
  if (UseSharpIR)
  {
    if (debug)
    {
      consoleMessage("... Using Sharp IR rangefinders.");
    }
    /* Read IR rangefinders in the order in which they're wired to the 16 channel MUX */
    for(int i = 0; i < 16; i ++)
    {
      newIRvalues[i] = readMux(i);
    }
    if (IRstarted)
    {
      // Compare new and old values to look for changes
      for (int i = 0; i < 16; i++)
      {
        // Serial.println((float)oldIRvalues[i] - (float)newIRvalues[i]);
        if (IRthresh < abs((float)oldIRvalues[i] - (float)newIRvalues[i]))
        {
          printDateStamp(); Serial.print("HA Variance in IR sensor "); Serial.print(i); Serial.print(" exceeds limits: ");
          Serial.println((float)oldIRvalues[i] - (float)newIRvalues[i]);
        }
      }
      // Save the new variables in the old variable array.
      for (int i = 0; i < 16; i++)
      {
        oldIRvalues[i] = newIRvalues[i];
      }
    }
    else
    {
      IRstarted = true;
    }
  }

  if (UseLIDAR)
  {
    if (debug)
    {
      consoleMessage("... LIDAR scan.");
    }
    if (!doLIDARscan(pwmServos01))
    {
      consoleMessage("LIDAR scan failed.");
    }
  }

  // ... Identify closest obstacle
  // ... Identify and avoid steep inclines/edges
  // ... Identify and avoid overhead obstacles
  // ... Identify and avoid obstacles in our path
  

  // Orient camera straight forward, take high contrast image?

  // Send STOP command to drivetrain Arduino if hazard present

  // END HAZARD AVOIDANCE/NAV SEGMENT

  // COMMAND EXECUTION

  // Take panaramic picture
  // Take picture in specific direction from Pixy pan/tilt (if present)
  // Transmit picture(s) to ground station
  // Set destination "waypoint."
  // Stop motion

  // END COMMAND EXECUTION SEGMENT

  // Slow down for communications bandwidth limitations
  delay(500);
  
}


