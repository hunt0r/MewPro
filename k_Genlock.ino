// Genlock
//
// HSYNC and VSYNC is generated by Dual Hero System and distributed by MewPro Genlock Dongle
//

unsigned long timelapse = 0;  // used by MODE_TIMELAPSE

void startGenlock()
{
  switch (td[TD_MODE]) {
  case MODE_PHOTO:
  case MODE_BURST:
    delay(500); // delay for LED light to be visible
    ledOff();
    break;
  case MODE_TIMELAPSE:
    timelapse = (unsigned long)td[TD_PHOTO_XSEC] * 1000;
    if (timelapse == 0) {
      timelapse = 500;
    }
    timelapse -= 30; // margin
    delay(10);
    digitalWrite(TRIG, LOW);
    delay(10);
    digitalWrite(TRIG, HIGH);
    break;
  default:
    break;
  }
}

void stopGenlock()
{
  timelapse = 0;
  // video stops at FALLING edge in MASTER NORMAL mode
  digitalWrite(TRIG, LOW);
  delay(1);
  digitalWrite(TRIG, HIGH);
}

void setupGenlock()
{
  pinMode(TRIG, OUTPUT); digitalWrite(TRIG, HIGH);
}

void checkGenlock()
{
  noInterrupts();
  unsigned long currentmillis = millis();
  if (timelapse > 0 && !waiting && currentmillis - previous_sync >= timelapse) {
    timelapse = 0;
    queueIn(F("SY2"));
  }
  interrupts();
}
