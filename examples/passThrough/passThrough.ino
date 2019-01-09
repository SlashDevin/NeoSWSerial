#include <Arduino.h>
#include <NeoSWSerial.h>

#define BAUD 38400

NeoSWSerial nss( 10, 11 );

void setup(){
  Serial.begin(57600);
  delay(50);

  while (!Serial){};

  nss.begin(BAUD);
  delay(50);

}

void loop() {
    while (nss.available()) {
        Serial.write(nss.read());
    }
    while (Serial.available()) {
        nss.write(Serial.read());
    }
}
