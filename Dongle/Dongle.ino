// This is the entirety of the GenlockDongle code for sdu_nosync.  The
// GenlockDongle is only a Serial repeater.

void setup()
{
  Serial.begin(57600);
}

void loop()
{
  if (Serial.available()) {
      Serial.write(Serial.read());
  }
}
