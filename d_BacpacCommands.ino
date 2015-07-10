// bacpac commands
//
void emulateDetachBacpac()
{
  // to exit 3D mode, emulate detach and attach bacpac
  //
  // detach bacpac
  pinMode(BPRDY, INPUT);
  delay(1000);
  // attach bacpac again
  stayInvisibleOrShowBPRDY(); // If Solocam mode, show ready to camera.  If
                              // USBmode, MewPro stays invisible
}

// what does this mean? i have no idea...
const unsigned char validationString[19] PROGMEM = { 18, 0, 0, 3, 1, 0, 1, 0x3f, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

void bacpacCommand()
{
  int command = (recv[1] << 8) + recv[2];
  switch (command) {
  case GET_BACPAC_PROTOCOL_VERSION: // vs
    while (digitalRead(I2CINT) != HIGH) { // wait until camera pullups I2CINT
      ;
    }
    ledOff();
    memcpy_P(buf, validationString, sizeof validationString);
    SendBufToCamera();
    delay(200); // need a short delay the validation string to be read by camera
    queueIn(F("cv"));
    return;
  case SET_BACPAC_DELETE_ALL: // DA
    return;
  case SET_BACPAC_DELETE_LAST: // DL
    return;
  case SET_BACPAC_FAULT: // FN
    buf[0] = 1; buf[1] = 0; // "ok"
    SendBufToCamera();
    if (recv[3] == 0x0c) {
      //      queueIn(F("XS1"));
    }
    return;
  case SET_BACPAC_HEARTBEAT: // HB
    if (!tdDone) {
      emulateDetachBacpac();
    }
    tdDone = true;
    return;
  case SET_BACPAC_POWER_DOWN: // PW
    tdDone = false;
    return;
  case SET_BACPAC_3D_SYNC_READY: // SR
    switch (recv[3]) {
    case 0: // CAPTURE_STOP
      stopGenlock();
      break;
    case 1: // CAPTURE_START
    case 2: // CAPTURE_INTERMEDIATE (PES only)
      startGenlock();
      break;
    case 3: // PES interim capture complete
      switch (td[TD_MODE]) {
      case MODE_VIDEO:
      case MODE_BURST:
      case MODE_PHOTO:
        stopGenlock();
        ledOff();
        break;
      default:
        break;
      }
      break;
    }
    return;
  case SET_BACPAC_WIFI: // WI
    return;
  case SET_BACPAC_SLAVE_SETTINGS: // XS
    // every second message will be off if we send "XS0" here
    queueIn(F("XS0"));
    if (debug) {
      char tmp[13];
      // battery level: 0-3 (4 if charging)
      Serial.print(F(" batt_level:")); Serial.print(recv[4]);
      // photos remaining
      Serial.print(F(" remaining:")); Serial.print((recv[5] << 8) + recv[6]);
      // photos on microSD card
      Serial.print(F(" photos:")); Serial.print((recv[7] << 8) + recv[8]);
      // video time remaining (minutes)
      if ((recv[9] << 8) + recv[10] == 0) { // GoPro firmware bug!
        Serial.print(F(" minutes:")); Serial.print(F("unknown"));
      } else {
        Serial.print(F(" minutes:")); Serial.print((recv[9] << 8) + recv[10]);
      }
      // videos on microSD card
      Serial.print(F(" videos:")); Serial.print((recv[11] << 8) + recv[12]);
      // maximum file size (4GB if FAT32, 0 means infinity if exFAT)
      // if one video file exceeds the limit then GoPro will divide it into smaller files automatically
      Serial.print(' ');
      printHex(recv[13], false);
      Serial.print(F("GB "));
      printHex(recv[14], false);
      printHex(recv[15], false);
      printHex(recv[16], false);
      Serial.println("");
    }
    return;
  case SET_BACPAC_SHUTTER_ACTION: // SH
    // shutter button of master is pressed
    buf[0] = 3; buf[1] = 'S'; buf[2] = 'Y'; buf[3] = recv[3];
    SendBufToCamera();
    return;
  case SET_BACPAC_DATE_TIME: // TM
    memcpy((char *)td+TD_DATE_TIME_year, (char *)recv+TD_DATE_TIME_year, 6);
		// _setTime(); HGM: DELETE
		buf[0] = 1; buf[1] = 0; // "ok"
    SendBufToCamera();
    return;
  default:
    break;
  }
}

// dummy setting: should be overridden soon
const char tmptd[TD_BUFFER_SIZE] PROGMEM = {
  0x28, 'T', 'D', 0x0f, 0x01, 0x12, 0x04, 0x0d, 0x33, MODE_PHOTO,
  0x05, 0xff, 0x03, 0x07, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00,
  0x01, 0x02, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x0a, };

void checkBacpacCommands()
{
  if (recvq) {
    waiting = false; // only time waiting is set back to false
    _printInput(); // show message via Serial if debug==true
    if (!(recv[0] & 0x80)) { // information bytes
      if (recv[0] == 0x25) { // 0x25 is initialize bacpac msg
        if (!tdDone) { // this is first time to see vs
          // td lets camera into 3D mode
          // camera will send HBFF
          queueIn(F("td")); // request td settings from camera
          // after receiving HBFF we are going to emulate detach bacpac
        } else {
          // this is second time to see vs, i.e.,
          //   > vs
          //   < td
          //   > HB FF
          // (emulate detach bacpac)
          //   > vs
          if (isSolocam()) { // hgm: why is this here?
            __debug(F("Solocam bacpac and not use genlock"));
            queueIn(F("VO1")); // SET_CAMERA_VIDEO_OUTPUT to herobus
            userSettings();
          } else {
            __debug(F("USBmode bacpac and not use genlock"));
            queueIn(F("XS1"));
            userSettings();
          }
        }
      } else if (recv[0] == 0x27) {
        // Usual packet length (recv[0]) is 0 or 1.
        // Packet length 0x27 does not exist but SMARTY_START
        memcpy((char *)td+1, recv, TD_BUFFER_SIZE-1); // load received TD into td[] message
        td[0] = TD_BUFFER_SIZE-1; td[1] = 'T'; td[2] = 'D'; // get ready to submit to slave
      } else {
        ; // do nothing
      }
    } else {
      bacpacCommand();
    }
    recvq = false;
  }
}
