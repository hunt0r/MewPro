#include <Arduino.h>

boolean ledState;

//  Arduino Due           || Arduino Pro Mini            || ATtiny1634
#if defined (__SAM3X8E__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATtiny1634__)
const int LED_OUT          = 13; // Arduino onboard LED; HIGH (= ON) while recording

void ledOff()
{
  digitalWrite(LED_OUT, LOW);
  ledState = false;
}

void ledOn()
{
  digitalWrite(LED_OUT, HIGH);
  ledState = true;
}

void setupLED()
{
  pinMode(LED_OUT, OUTPUT);
  ledOff();
}
//    Arduino Pro Micro
#elif defined(__AVR_ATmega32U4__)
void ledOff()
{
  TXLED0;
  RXLED0;
  ledState = false;
}

void ledOn()
{
  TXLED1;
  RXLED1;
  ledState = true;
}

void setupLED()
{
  ledOff();
}
#else
#error CPU not supported (hgm removed some CPUs on 2015-07-10, see orangkucing's code for support)
#endif
