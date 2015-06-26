#define MEWPRO_BUFFER_LENGTH 64

byte queue[MEWPRO_BUFFER_LENGTH];
volatile int queueb = 0, queuee = 0;
boolean waiting = false;
// "waiting" flag is used to prevent reading the next command from the
// queue until a response has been received to previous command (processed
// in d_BacpacCommands, checkBacpacCommands())

void emptyQueue()
{
  queueb = queuee = 0;
  waiting = false;
}

boolean inputAvailable()
{ // Test if any messages exist, either in the queue or on the serial line
  if (!waiting && (queueb != queuee || Serial.available())) {
    return true;
  }
  return false;
}

byte myRead()
{ // This reads Serial messages.  First from the queue, then from the
  // Serial monitor line
  if (queueb != queuee) {
    byte c = queue[queueb];
    queueb = (queueb + 1) % MEWPRO_BUFFER_LENGTH;
    return c;
  }
  return Serial.read();
}

// Utility functions
void queueIn(const __FlashStringHelper *p)
{ // queueIn allows MewPro to "fake" a serial message from user.  MewPro
  // processes this message as though it had come from the Serial monitor
  int i;
  char c;
  for (i = 0; (c = pgm_read_byte((char PROGMEM *)p + i)) != 0; i++) {
    queue[(queuee + i) % MEWPRO_BUFFER_LENGTH] = c;
  }
  queue[(queuee + i) % MEWPRO_BUFFER_LENGTH] = '\n';
  queuee = (queuee + i + 1) % MEWPRO_BUFFER_LENGTH;
}

void queueIn(const char *p)
{ // queueIn allows MewPro to "fake" a serial message from user.  MewPro
  // processes this message as though it had come from the Serial monitor
  int i;
  char c;
  for (i = 0; (c = *(p + i)) != 0; i++) {
    queue[(queuee + i) % MEWPRO_BUFFER_LENGTH] = c;
  }
  queue[(queuee + i) % MEWPRO_BUFFER_LENGTH] = '\n';
  queuee = (queuee + i + 1) % MEWPRO_BUFFER_LENGTH;
}
