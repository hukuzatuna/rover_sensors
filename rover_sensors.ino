
/* Autonomous robot rover - sensor/comms/control platform */
/* Philip R. Moyer */
/* November 2015 */

#include <Wire.h>
#include <Adafruit_Sensor.h>      /* 10DoF support */
#include <Adafruit_LSM303_U.h>    /* 10DoF support - accelerometer/magnetometer */
#include <Adafruit_BMP085_U.h>    /* 10DoF support - barometric pressure */
#include <Adafruit_L3GD20_U.h>    /* 10DoF support - gyroscope */
#include <Adafruit_10DOF.h>       /* 10DoF support */
#include <Adafruit_TSL2561_U.h>   /* TSL2561 digital light/lux sensor */
#include <Adafruit_SI1145.h>      /* SI1145 UV/Visible/IR level sensor */
#include <Adafruit_HTU21DF.h>     /* HTU21DF temp/humidity sensor */

/* Initialize sensor objects */
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified();
Adafruit_TSL2561_Unified      tsl   = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_SI1145               uv    = Adafruit_SI1145();
Adafruit_HTU21DF              htu   = Adafruit_HTU21DF();

void configureTSLSensor(void)
{
  tsl.enableAutoRange(true);    /* switches gain automatically for light conditions */
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);  /* fast but low resolution */
  // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16 bit high resolution */
}

void setup() {
  // put your setup code here, to run once:
  uint8_t cnt = 0;
  
  Serial.begin(115200);   /* This will need to be reduced when using packet data over radio - probably to 9600 */

  // SCIENCE PACKAGE SETUP
  
  Serial.println("Science package startup sequence.");
  
  /* Set up sensors on the 10DoF breakout. */
  if (!accel.begin())
  {
    Serial.println(F("No LSM303 detected - accelerometer."));
    cnt = 1;
  }
  else
  {
    Serial.println("LSM303 accelerometer ok");
  }
  
  if (!mag.begin()) {
    Serial.println(F("No LSM303 detected - magnetometer."));
    cnt = 1;
  }
  else
  {
    Serial.println("LSM303 magnetometer ok");
  }
  
  if (!bmp.begin())
  {
    Serial.println("No BMP085 detected - barometer.");
    cnt = 1;
  }
  else
  {
    Serial.println("BMP085 barometer ok");
  }
  
  if (!gyro.begin())
  {
    Serial.println("No L3GD20 detected - gyroscope.");
    cnt = 1;
  }
  else
  {
    Serial.println("L3GD20 gyroscope ok.");
  }
  
  if (!tsl.begin())
  {
    Serial.println("No TSL2561 detected.");
    cnt = 1;
  }
  else
  {
    Serial.println("TSL2561 light sensor ok. Setting gain and integration time.");
  }

  /* Initialie the TSL2561 light sensor gain and integration time */
  configureTSLSensor();

  if (!uv.begin())
  {
    Serial.println("No SI1145 detected.");
    cnt = 1;
  }
  else
  {
    Serial.println("SI1145 light sensor ok.");
  }
  
  /* Initialie the HTU21DF temperature and humidity sensor */
  if (!htu.begin()) 
  {
    Serial.println("No HTU21DF sensor!");
    cnt = 1;
  }
  else
  {
    Serial.println("HTU21DF temp/humidity sensor ok.");
  }
  
  // if (0 < cnt)
  // {
    // while(1);
  // }

  // END SCIENCE PACKAGE SETUP

  // HAZARD AVOIDANCE SETUP
  /* Wiring Order should be: */
  /* Front right up 45, front center up 45, front left up 45 */
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

  // END HAZARD AVOIDANCE SETUP

  // CAMERA SETUP

  // END CAMERA SETUP

  // CAMERA PAN/TILT SETUP

  // END CAMERA PAN/TILT SETUP

  // COMMUNICATIONS SETUP

  // END COMMUNICATIONS SETUP
}

void loop() {
  // put your main code here, to run repeatedly:

  // SCIENCE PACKAGE

  /* Event object for 10DoF science sensor */
  sensors_event_t event;

  /* Get events from 10DoF breakout and construct an output string */
  /* Output string will be stored on SD card, sent to serial console, and transmitted to motor controller */
  /* In the future, the science data will also be sent over the radio on request for science data. */
  
  accel.getEvent(&event);
  Serial.print(F("10DOF ACCEL "));
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  mag.getEvent(&event);
  Serial.print(F("10DOF MAG   "));
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  gyro.getEvent(&event);
  Serial.print(F("10DOF GYRO  "));
  Serial.print("X: "); Serial.print(event.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.gyro.z); Serial.print("  ");Serial.println("rad/s ");  

  bmp.getEvent(&event);
  if (event.pressure)
  {
    /* Display atmospheric pressure in hPa */
    Serial.print(F("10DOF PRESS "));
    Serial.print(event.pressure);
    Serial.print(F(" hPa, "));
    /* Display ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.print(temperature);
    Serial.println(F(" C "));
  }

  /* TSL2561 light conditions */
  tsl.getEvent(&event);
  if (event.light)
  {
    Serial.print("TSL2561: "); Serial.print(event.light); Serial.println(" lux");
  }
  else
  {
    Serial.println("TSL2561 overload");
  }

  /* SI1145 UV/visible/IR level sensor */
  Serial.print("SI1145 Vis: "); Serial.print(uv.readVisible());
  Serial.print(", IR: "); Serial.print(uv.readIR());
  float UVindex = uv.readUV();
  Serial.print(", UV: "); Serial.print(UVindex);
  UVindex /= 100.0;
  Serial.print(", UVI: "); Serial.println(UVindex);
  
  /* HTU21DF temperature/humidity sensor */
  Serial.print("HTU21DF Temp: "); Serial.print(htu.readTemperature());
  Serial.print(" Hum: "); Serial.println(htu.readHumidity());
  
  // Debugging. Remove before flight. :-)
  Serial.println("");
  delay(10000);

  // END SCIENCE PACKAGE SEGMENT

  // HAZARD AVOIDANCE/NAV
  /* Read IR rangefinders in the order in which they're wired to the 16 channel MUX */

  /* Orient camera straight forward, take high contrast image */

  /* Send STOP command to drivetrain Arduino if hazard present */

  // END HAZARD AVOIDANCE/NAV SEGMENT

  // COMMUNICATIONS WITH GROUND STATION

  // END COMMUNICATIONS SEGMENT

  // COMMAND EXECUTION
  /* Take panaramic picture */
  /* Transmit panaramic picture to ground station */

  // END COMMAND EXECUTION SEGMENT
}


