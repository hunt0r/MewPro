// MewPro
//
// The following are known to work with MewPro at least core functionalities. Not all the sensors, however, are supported by each of them.
//
//   Arduino Pro Mini 328 3.3V 8MHz
//          w/ Arduino IDE 1.5.7+
//          if you have troubles on compiling unused or nonexistent libraries, simply comment out #include line as //#include (see Note below)
//
//   Teensy 3.x
//          To compile the code with Teensy 3.x:
//          1. use Arduino IDE 1.0.6+ and Teensyduino 1.20+
//          2. comment out all unused #include as //#include (see Note below)
//
//   (Note: There is an infamous Arduino IDE's preprocessor bug (or something) that causes to ignore #ifdef/#else/#endif directives and forces 
//    to compile unnecessary libraries.)
//
//   GR-KURUMI
//          To compile the code with GR-KURUMI using Renesas web compiler http://www.renesas.com/products/promotion/gr/index.jsp#cloud :
//          1. open a new project with the template GR-KURUMI_Sketch_V1.04.zip.
//          2. create a folder named MewPro and upload all the files there.
//          3. at project's home directory, replace all the lines of gr_scketch.cpp by the following code (BEGIN / END lines should be excluded).
/* BEGIN copy
#include <RLduino78.h>
#include "MewPro/MewPro.ino"
#include "MewPro/a_Queue.ino"
#include "MewPro/b_TimeAlarms.ino"
#include "MewPro/c_I2C.ino"
#include "MewPro/d_BacpacCommands.ino"
#include "MewPro/e_Shutter.ino"
#include "MewPro/f_Switch.ino"
#include "MewPro/g_IRremote.ino"
#include "MewPro/h_LightSensor.ino"
#include "MewPro/i_PIRsensor.ino"
#include "MewPro/j_VideoMotionDetect.ino"
END copy */
//

//   Copyright (c) 2014 orangkucing

//////////////////////////////////////////////////////////
// Options:
//   Choose either "#define" to use or "#undef" not to use. 
//   if #define then don't forget to remove // before //#include

//********************************************************
// a_TimeAlarms: MewPro driven timelapse
#undef  USE_TIME_ALARMS
// Time and TimeAlarms libraries are downloadable from
//   http://www.pjrc.com/teensy/td_libs_Time.html
//   and http://www.pjrc.com/teensy/td_libs_TimeAlarms.html
// In order to compile the code on Pro Mini 328, find the following and edit the first line of them
//   #if defined(__AVR__)
//   #include <avr/pgmspace.h>
//   #else
// to
//   #if defined(__AVR__) && !defined(__AVR_ATmega328P__)
//   #include <avr/pgmspace.h>
//   #else
// appeared in Documents/Arduino/libraries/Time/DateStrings.cpp
//#include <Time.h> // *** please comment out this line if USE_TIME_ALARMS is not defined ***
//#include <TimeAlarms.h> // *** please comment out this line if USE_TIME_ALARMS is not defined ***

//********************************************************
// c_I2C: I2C interface (THIS PART CAN'T BE OPTED OUT)
// Teensy 3.0 or Teensy 3.1
//#include <i2c_t3.h> // *** please comment out this line if __MK20DX256__ and __MK20DX128__ are not defined ***
// otherwise
#include <Wire.h> // *** please comment out this line if __MK20DX256__ or __MK20DX128__ is defined ***

//********************************************************
// e_Shutters: One or two remote shutters without contact bounce or chatter
#undef  USE_SHUTTERS

//********************************************************
// f_Switches: One or two mechanical switches
#undef  USE_SWITCHES

//********************************************************
// g_IRremote: IR remote controller
#undef  USE_IR_REMOTE
// IRremote2 is downloadable from https://github.com/enternoescape/Arduino-IRremote-Due
// (this works not only on Due but also on Pro Mini etc.)
//#include <IRremote2.h> // *** please comment out this line if USE_IR_REMOTE is not defined ***

//********************************************************
// h_LightSensor: Ambient light sensor
#undef  USE_LIGHT_SENSOR

//********************************************************
// i_PIRsensor: Passive InfraRed motion sensor
#undef  USE_PIR_SENSOR

//********************************************************
// j_VideoMotionDetect: Video Motion Detector
//   Video motion detect consumes almost all the dynamic memory. So if you want to use this then #undef all options above.
#undef  USE_VIDEOMOTION
// The part of code utilizes the following library except GR-KURUMI. Please download and install:
//   https://github.com/orangkucing/analogComp
//#include "analogComp.h" // *** please comment out this line if USE_VIDEOMOTION is not defined or GR-KURUMI ***

// end of Options
//////////////////////////////////////////////////////////

#include <Arduino.h>
#include "MewPro.h"

boolean lastHerobusState = LOW;  // Will be HIGH when camera attached.

void setup() 
{
  // Remark. Arduino Pro Mini 328 3.3V 8MHz is too slow to catch up with the highest 115200 baud.
  //     cf. http://forum.arduino.cc/index.php?topic=54623.0
  // Set 57600 baud or slower.
  Serial.begin(57600);
  
  setupShutter();
  setupSwitch();
  setupIRremote();
  setupLightSensor();
  setupPIRSensor();

  setupLED(); // onboard LED setup 
  pinMode(BPRDY, OUTPUT); digitalWrite(BPRDY, LOW);    // Show camera MewPro attach. 
  pinMode(TRIG, OUTPUT); digitalWrite(TRIG, LOW);

  // don't forget to switch pin configurations to INPUT.
  pinMode(I2CINT, INPUT);  // Teensy: default disabled
  pinMode(HBUSRDY, INPUT); // default: analog input
  pinMode(PWRBTN, INPUT);  // default: analog input
}

void loop() 
{
  // Attach or detach bacpac
  if (digitalRead(HBUSRDY) == HIGH) {
    if (lastHerobusState != HIGH) {
      pinMode(I2CINT, OUTPUT); digitalWrite(I2CINT, HIGH);
      lastHerobusState = HIGH;
      resetI2C();
    }
  } else {
    if (lastHerobusState != LOW) {
      pinMode(I2CINT, INPUT);
      lastHerobusState = LOW;
    }
  }

  checkTimeAlarms();
  checkBacpacCommands();
  checkCameraCommands();
  checkSwitch();
  checkIRremote();
  checkLightSensor();
  checkPIRSensor();
  checkVMD();
}

