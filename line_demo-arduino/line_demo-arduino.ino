#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  while (!Serial) {}
  mySerial.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (mySerial.available()) {
    Serial.write(mySerial.read());
  }

  if (Serial.available()) {
    mySerial.write(Serial.read());
  }
}
