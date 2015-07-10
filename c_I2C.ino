#if !defined(__MK20DX256__) && !defined(__MK20DX128__) && !defined(__MKL26Z64__) // not Teensy 3.x/LC
#define I2C_NOSTOP false
#define I2C_STOP true
#endif

#if defined (__SAM3X8E__) // Arduino Due only
// GoPro camera already has pull-up resistors on the I2C bus inside.
// Due's Wire lib, however, uses D20 and D21 as SDA and SCL respectively, which have pull-up resistors of 1k ohm, too.
// Thus in order to avoid the conflict of resistors we must use non pull-up'ed D70 and D71 as SDA and SCL respectively,
// and these correspond to Wire1 lib here.
#define WIRE              Wire1
#else
// standard Wire library enables ATmega internal pull-ups by default. usually the resistors are 20k ohm or greater thus
// these should cause no harm.
#define WIRE              Wire
#endif

// GoPro Dual Hero EEPROM IDs
const int ID_SOLOCAM = 4;
const int ID_USBMODE  = 5;

// I2C slave addresses
const int I2CEEPROM = 0x50;
const int SMARTY = 0x60;
const int I2CPROXY = 0x70;

// Camera accesses I2C EEPROM located at slave address 0x50 using 8-bit word address.
// So any one of 3.3V EEPROMs 24XX00, 24XX01, 24XX02, 24XX04, 24XX08, or 24XX16 (XX = AA or LC) works with camera.
//   Note: If not pin-compatible 24AA00 (SOT-23) is used, PCB modifications as well as following value changes are nesessary:
//     const int WRITECYCLETIME = 4000;
//     const int PAGESIZE = 1;
//
// cycle time in milliseconds after block write
const int WRITECYCLETIME = 5000;
//
// page size for block write
const int PAGESIZE = 8; // 24XX01, 24XX02
// const int PAGESIZE = 16; // 24XX04, 24XX08, 24XX16

byte buf[MEWPRO_BUFFER_LENGTH], recv[MEWPRO_BUFFER_LENGTH];

int bufp = 1; // this is a counter for buf[] message length
volatile boolean recvq = false;
unsigned long previous_sync;  // last sync (used by timelapse mode)

#if !defined(__AVR_ATtiny1634__)
// --------------------------------------------------------------------------------
#if defined(__MK20DX256__) || defined(__MK20DX128__) || defined(__MKL26Z64__)
// interrupts
void receiveHandler(size_t numBytes)
#else
void receiveHandler(int numBytes)
#endif
{
  int i = 0;
  while (WIRE.available()) {
    recv[i++] = WIRE.read();
    recvq = true;
  }
  if ((recv[1] << 8) + recv[2] == SET_BACPAC_3D_SYNC_READY) {
    switch (recv[3]) {
    case 1:
      ledOn();
      break;
    case 0:
      ledOff();
      break;
    default:
      break;
    }
  }
}

void requestHandler()
{
  digitalWrite(I2CINT, HIGH);
  WIRE.write(buf, (int) buf[0] + 1);
}

void resetI2C()
{
  emptyQueue();
  WIRE.begin(SMARTY);
  WIRE.onReceive(receiveHandler);
  WIRE.onRequest(requestHandler);
}

// Read I2C EEPROM
boolean isSolocam()
{
  if (eepromId == 0) {
    WIRE.begin();
    WIRE.beginTransmission(I2CEEPROM);
    WIRE.write((byte) 0);
    WIRE.endTransmission(I2C_NOSTOP);
    WIRE.requestFrom(I2CEEPROM, 1, I2C_STOP);
    if (WIRE.available()) {
      eepromId = WIRE.read();
    }

    resetI2C();
  }
  return (eepromId == ID_SOLOCAM);
}

// If MewPro in Solocam Mode, show bacpac ready to take control.  If MewPro
// in USBmode, trick camera into ignoring MewPro (so that USB mode
// works)
void stayInvisibleOrShowBPRDY()
{
  if (isSolocam()){ // show camera BPRDY to take control
    pinMode(BPRDY, OUTPUT);
    digitalWrite(BPRDY, LOW);
  } else { // remain invisible (so camera acts independently)
     pinMode(BPRDY, INPUT);
  }
}

// Show Solocam/USBmode via LED: 3 blink for USBmode, 4 for Solocam
void showModeOnLED()
{
  boolean orig_led_state = ledState;
  isSolocam() ? signalSolocam() : signalUSBmode();
  if (orig_led_state == 1) ledOn();
  else ledOff();
}

void signalSolocam() // 4 blinks for Solocam Mode
{
  ledOff(); // ready
  delay(100);
  ledOn(); // blink 1
  delay(100);
  ledOff();
	delay(100);
  ledOn(); // blink 2
  delay(100);
  ledOff();
	delay(100);
  ledOn(); // blink 3
  delay(100);
  ledOff();
	delay(100);
  ledOn(); // blink 4
  delay(100);
  ledOff();
	delay(100);
}

void signalUSBmode() // 3 blinks for USBmode
{
  ledOff(); // ready
  delay(100);
  ledOn(); // blink 1
  delay(150);
  ledOff();
	delay(100);
  ledOn(); // blink 2
  delay(150);
  ledOff();
	delay(100);
  ledOn(); // blink 3
  delay(150);
  ledOff();
  delay(100);
}

// Write I2C EEPROM
void modeChange()
{
  byte id, d;
  // emulate detouching bacpac by releasing BPRDY line
  pinMode(BPRDY, INPUT);
  delay(1000);

  id = isSolocam() ? ID_USBMODE : ID_SOLOCAM;

  WIRE.begin();
  for (unsigned int a = 0; a < 16; a += PAGESIZE) {
    WIRE.beginTransmission(I2CEEPROM);
    WIRE.write((byte) a);
    for (int i = 0; i < PAGESIZE; i++) {
      switch ((a + i) % 4) {
      case 0: d = id; break; // major (MOD1): 4 for Solocam, 5 for USBmode
      case 1: d = 5; break;  // minor (MOD2) need to be greater than 4
      case 2: d = 1; break;
      case 3: d = (id == 4 ? 0x0a : 0x0b); break;
      }
      WIRE.write(d);
    }
    WIRE.endTransmission(I2C_STOP);
    delayMicroseconds(WRITECYCLETIME);
  }
  eepromId = id;
  isSolocam() ? __debug(F(" to solocam")) : __debug(F(" to usbmode"));
  showModeOnLED(); // show solocam/usbmode status via LED
  stayInvisibleOrShowBPRDY(); // If Solocam mode, show ready to camera.  If
                              // USBmode, MewPro stays invisible
  resetI2C();
}

// --------------------------------------------------------------------------------
#else // __AVR_ATtiny1634__

#define ROMSIZE 16
#define EEPROMOFFSET 0

volatile uint8_t wordAddr;
volatile boolean repeatedStart;
volatile boolean emulateRom;

// interrupts
boolean addressHandler(uint16_t slave, uint8_t count)
{
  emulateRom = ((slave >> 1) == I2CEEPROM);
  repeatedStart = (count > 0 ? true : false);
  if (emulateRom) {
    // EEPROM access
    if (repeatedStart && WIRE.available()) {
      wordAddr = WIRE.read();
    }
  }
  return true;
}

void receiveHandler(size_t numBytes)
{
  if (emulateRom) {
    // EEPROM access (byte write: ignored)
    return;
  }

  // SMARTY
  int i = 0;
  while (WIRE.available()) {
    recv[i++] = WIRE.read();
    recvq = true;
  }
  if ((recv[1] << 8) + recv[2] == SET_BACPAC_3D_SYNC_READY) {
    switch (recv[3]) {
    case 1:
      ledOn();      //If Mewpro gets synced with the system, On-board LED turns on (on receiving "SR1")
      break;
    case 0:
      ledOff();    //Turn off LED on receiving "SR0"
      break;
    default:
      break;
    }
  }
}

void requestHandler()
{
  if (emulateRom) {
    // EEPROM access
    if (repeatedStart) {
      // GoPro requests random read only
      WIRE.write(EEPROM.read(wordAddr % ROMSIZE + EEPROMOFFSET));
    }
    return;
  }

  // SMARTY
  digitalWrite(I2CINT, HIGH);
  WIRE.write(buf, (int) buf[0] + 1);
}

void resetI2C()
{
  emptyQueue();

  WIRE.begin(I2CEEPROM, ((SMARTY << 1) | 1));
  WIRE.onAddrReceive(addressHandler);
  WIRE.onReceive(receiveHandler);
  WIRE.onRequest(requestHandler);
}

// Read I2C EEPROM
void __romWrite(uint8_t id)
{
  byte d;
  for (int a = 0; a < ROMSIZE; a++) {
    switch (a % 4) {
    case 0: d = id; break; // major (MOD1): 4 for Solocam, 5 for USBmode
    case 1: d = 5; break;  // minor (MOD2) need to be greater than 4
    case 2: d = 1; break;
    case 3: d = (id == 4 ? 0x0a : 0x0b); break;
    }
    EEPROM.write(a + EEPROMOFFSET, d);
  }
}

boolean isSolocam()
{
  if (eepromId == 0) {
    eepromId = EEPROM.read(EEPROMOFFSET);
    if (eepromId != ID_SOLOCAM && eepromId != ID_USBMODE) {
      __romWrite(ID_SOLOCAM);
      eepromId = ID_SOLOCAM;
    }
    resetI2C();
  }
  return (eepromId == ID_SOLOCAM);
}

// Write built-in EEPROM
void modeChange()
{
  byte id, d;
  // emulate detouching bacpac by releasing BPRDY line
  pinMode(BPRDY, INPUT);
  delay(1000);

  id = isSolocam() ? ID_USBMODE : ID_SOLOCAM;
  __romWrite(id);
  eepromId = id;
  isSolocam() ? __debug(F(" to solocam")) : __debug(F(" to usbmode"));
  showModeOnLED(); // show solocam/usbmode status via LED
  stayInvisibleOrShowBPRDY(); // If Solocam mode, show ready to camera.  If
                              // USBmode, MewPro stays invisible
}
// --------------------------------------------------------------------------------
#endif // __AVR_ATtiny1634

// print out debug information to Arduino serial console
void __debug(const __FlashStringHelper *p)
{
  if (debug) {
    Serial.println(p);

    //it doesn't matter whether wait for flush or not when you are debugging
    //because with debug msg on, the whole system cannot work normally
    //if not in a Genlock sync system (single use), I think it's better to wait for flush
    //may cause delay for a long debug msg
    Serial.flush();
  }
}

void printHex(uint8_t d, boolean upper)
{
  char t;
  char a = upper ? 'A' : 'a';
  t = d >> 4 | '0';
  if (t > '9') {
    t += a - '9' - 1;
  }
  Serial.print(t);
  t = d & 0xF | '0';
  if (t > '9') {
    t += a - '9' - 1;
  }
  Serial.print(t);
}

void __printBuf(byte *p)
{
  int buflen = p[0] & 0x7f;

  for (int i = 0; i <= buflen; i++) {
    if (i == 1 && isprint(p[1]) || i == 2 && p[1] != 0 && isprint(p[2])) {
      if (i == 1) {
        Serial.print(' ');
      }
      Serial.print((char) p[i]);
    } else {
      Serial.print(' ');
      printHex(p[i], false);
    }
  }
  Serial.println("");
  Serial.flush();
}

void _printInput()
{
  if (debug) {
    Serial.print('>');
    __printBuf(recv);
  }
}

void SendBufToCamera() {
  // This function handles additional commands which need to be executed on
  // the MewPro side before sending the buf out via I2C.
  int command = (buf[1] << 8) + buf[2];
  switch (command) {
  case SET_CAMERA_3D_SYNCHRONIZE:
    noInterrupts();
    waiting = true; // don't read the next command from the queue until a
                    // reply is received to the present one
    previous_sync = millis();
    interrupts();
    break;
  case SET_CAMERA_USBMODE:
    waiting = true;
    break;
  case GET_CAMERA_INFO:
  case GET_CAMERA_SETTING:
  case SET_CAMERA_SETTING:
  case SET_CAMERA_VIDEO_OUTPUT:
  case SET_CAMERA_AUDIOINPUTMODE:
  case SET_CAMERA_DATE_TIME:
    waiting = true; // don't read the next command from the queue until a
                    // reply is received to the present one
    break;
  default:
    for (int offset = 0x09; offset < TD_BUFFER_SIZE; offset++) {
      if (pgm_read_word(tdtable + offset - 0x09) == command) {
        td[offset] = buf[3];
        waiting = true;
        break;
      }
    }
    break;
  }

  if (debug) { // if debug, print I2C message for user to see
    Serial.print('<');
    __printBuf(buf);
  }

  // Send I2C message out (buf[])
  digitalWrite(I2CINT, LOW);
  // by pulling low the I2CINT (making a request), GoPro will handle the
  // request, receive buf[], then executes the command in buf[] immediately
}

void startRecording()
{
  queueIn(F("SY1"));
}

void stopRecording()
{
  queueIn(F("SY0"));
}

// Camera power On
void powerOn()
{
  pinMode(PWRBTN, OUTPUT);
  digitalWrite(PWRBTN, LOW);
  delay(1000);
  tdDone = false;
  pinMode(PWRBTN, INPUT);
}

void powerOff()
{
  //power off master first
  buf[0] = 3; buf[1] = 'P'; buf[2] = 'W'; buf[3] = 0x00;
  SendBufToCamera();
}

void checkCameraCommands()
{ // CameraCommands come via the Serial communication.  They may be from
  // the GenlockDongle, from a computer connected via the Arduino Serial
  // monitor, or from the MewPro itself, using the queueIn to "fake" an
  // externally sent Serial message.
  while (inputAvailable())  {
    static boolean shiftable; // this flag used to convert 2-character
                              // hexidecimal numbers from Serial to a
                              // single data value (example: characters
                              // '0b' convert to value 11)
    byte c = myRead();
    // hgm: don't think this works, but we need to find a way to view the
    //      queue when in debug mode
    // if(debug) { Serial.print("myRead()=");
    // Serial.println((char)c); }
    switch (c) {
    case ' ': // Ignore spaces
      continue;
    case '\r': // carriage return (\r) or
    case '\n': // newline (\n) finishes (and sends) I2C message
      if (bufp != 1) {
        buf[0] = bufp - 1;
        bufp = 1;
        SendBufToCamera();
      }
      return;
    case '&': // toggle debug mode
      bufp = 1;
      debug = !debug;
      __debug(F("debug messages on"));
      while (inputAvailable()) {
        if (myRead() == '\n') {
          return;
        }
      }
      return;
    case '@': // power on system
      bufp = 1;
      __debug(F("camera power on"));
      powerOn();
      while (inputAvailable()) {
        if (myRead() == '\n') {
          return;
        }
      }
      return;
    case '#': // power off system
      bufp = 1;
      __debug(F("camera power off"));
      powerOff();
      while (inputAvailable()) {
        if (myRead() == '\n') {
          return;
        }
      }
      return;
    case '!':
      bufp = 1;
      __debug(F("mode change"));
      modeChange();
      while (inputAvailable()) {
        if (myRead() == '\n') {
          return;
        }
      }
      return;

    case '/':
      Serial.println(F(MEWPRO_FIRMWARE_VERSION));
      Serial.flush();
      return;
    default:
      // Every buf[] message has form:
      //  buf[0]   buf[1] buf[2] buf[3]  ... buf[2+N]
      //  [length] [cmd1] [cmd2] [data1] ... [dataN]
      // therefore bufp >= 3 means processing a [dataX] byte in c
      if (bufp >= 3 && isxdigit(c)) {
        // convert integer value for char '8' to numerical value 8
        //  (or char 'b' to numerical 11, char 'e' to 14, etc.)
        c -= '0'; // move c from char '0' to '9' to integer 0 to 9
        if (c >= 10) {
          // this is an ASCII trick to convert both 'a' or 'A' to 1,
          // convert 'c' and 'C' to 3, etc... then add 9 to get numerical
          // value: a=10, b=11, c=12, etc.
          c = (c & 0x0f) + 9;
        }
      }
      if (bufp < 4) { // processing [length] or [cmd], not [dataX]
        // always reset shiftable to true before processing [dataX]
        shiftable = true;
        buf[bufp++] = c; // add to buffer
      } else {
        if (shiftable) {
          // processing 2nd char of 2-char number (ex: the 'b' in '0b')
          // so bit-shift the present value (mult by 16) and add the LSB
          buf[bufp-1] = (buf[bufp-1] << 4) + c;
        } else {
          // either processing 1st char of 2-char number, or just a
          // single-character value
          buf[bufp++] = c;
        }
        shiftable = !shiftable;
      }
      break;
    }
  }
}
