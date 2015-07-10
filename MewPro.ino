// MewPro
//
// The following small-factor micro processor boards are known to work with MewPro at least core functionalities and fit within the GoPro housing.
// Not all the sensors, however, are supported by each of them.
//
//   Arduino Pro Mini 328 3.3V 8MHz
//          w/ Arduino IDE 1.5.7+
//          if you have troubles in compiling unused or nonexistent libraries, simply comment out #include line as //#include (see Note* below)
//
//   Arduino Pro Micro - 3.3V 8MHz
//          w/ Arduino IDE 1.5.7+
//          if you have troubles in compiling unused or nonexistent libraries, simply comment out #include line as //#include (see Note* below)
//

//   Copyright (c) 2014-2015 orangkucing
//
// MewPro firmware version string for maintenance
#define MEWPRO_FIRMWARE_VERSION "2015071008"  // hgm: I append "08" to the end to mark as ours

//
#include <Arduino.h>
#include <EEPROM.h>
#include "MewPro.h"

// enable console output
// set false if this is MewPro #0 of dual dongle configuration
boolean debug = false;

//////////////////////////////////////////////////////////
// Options:
//   Choose either "#define" to use or "#undef" not to use.
//   if #define then don't forget to remove // before //#include

//********************************************************
// c_I2C: I2C interface (THIS PART CAN'T BE OPTED OUT)
//
// Note: in order to use MewPro reliably, THE FOLLOWING MODIFICATIONS TO STANDARD ARDUINO LIBRARY SOURCE IS
// STRONGLY RECOMMENDED:
//
// Arduino Pro Mini / Arduino Pro Micro
//     1. hardware/arduino/avr/libraries/Wire/Wire.h
//            old: #define BUFFER_LENGTH 32                        -->   new: #define BUFFER_LENGTH 64
//     2. hardware/arduino/avr/libraries/Wire/utility/twi.h
//            old: #define TWI_BUFFER_LENGTH 32                    -->   new: #define TWI_BUFFER_LENGTH 64
// Arduino Due
//     hardware/arduino/sam/libraries/Wire/Wire.h
//            old: #define BUFFER_LENGTH 32                        -->   new: #define BUFFER_LENGTH 64
#include <Wire.h> // *** please comment out this line if __MK20DX256__ or __MK20DX128__ or __MKL26Z64__ or __AVR_ATtiny1634__ is defined ***
#if BUFFER_LENGTH < 64
#error Please modify Arduino Wire library source code to increase the I2C buffer size
#endif
//
// ATtiny1634 core https://github.com/SpenceKonde/arduino-tiny-841
//    WireS library is downloadable from https://github.com/orangkucing/WireS
//#include <WireS.h> // *** please comment out this line if __AVR_ATtiny1634__ is not defined ***

// it is better to define this when RXI is connected to nothing (eg. MewPro #0 of Genlock system)
#undef  UART_RECEIVER_DISABLE

// end of Options
//////////////////////////////////////////////////////////

boolean lastHerobusState = LOW;  // Will be HIGH when camera attached.
int eepromId = 0;

void userSettings()
{
  // This function is called once after camera boot.
  // you can set put any camera commands here. For example:
  // queueIn("AI1");
  // queueIn("TI5");
  // queueIn("TD 0f 07 04 03 02 01 00 04 ff 01 09 00 00 02 00 01 00 01 00 00 00 05 04 00 00 00 00 00 00 00 00 00 01 00 00 00 00 0a");
}

void setup()
{
  // Remark. Arduino Pro Mini 328 3.3V 8MHz is too slow to catch up with the highest 115200 baud.
  //     cf. http://forum.arduino.cc/index.php?topic=54623.0
  // Set 57600 baud or slower.
  Serial.begin(57600);
#ifdef UART_RECEIVER_DISABLE
#ifndef __AVR_ATmega32U4__
  UCSR0B &= (~_BV(RXEN0));
#else
  UCSR1B &= (~_BV(RXEN1));
#endif
#endif

  setupGenlock();
  setupLED(); // onboard LED setup
  showModeOnLED();
  stayInvisibleOrShowBPRDY(); // If Solocam mode, show ready to camera.  If
                              // USBmode, MewPro stays invisible

  // don't forget to switch pin configurations to INPUT.
  pinMode(I2CINT, INPUT);  // Teensy: default disabled
  pinMode(HBUSRDY, INPUT); // default: analog input
  pinMode(PWRBTN, INPUT);  // default: analog input
}

void loop()
{
  // Attach or detach bacpac
  if (digitalRead(HBUSRDY) == HIGH) {
    if (lastHerobusState != HIGH) { // Bacpac just attached
      pinMode(I2CINT, OUTPUT); digitalWrite(I2CINT, HIGH);
      lastHerobusState = HIGH;
      if (eepromId == 0) {
        isSolocam(); // determine Solocam/USBmode and resetI2C()
      } else {
        resetI2C();
      }
    }
  } else {
    if (lastHerobusState != LOW) { // Bacpac just detatched
      pinMode(I2CINT, INPUT);
      lastHerobusState = LOW;
    }
  }
  checkBacpacCommands();
  checkCameraCommands();
  checkGenlock();
}
