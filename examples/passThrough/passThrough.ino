#include <Arduino.h>
#include <NeoSWSerial.h>

uint16_t baudrate = 9600;

NeoSWSerial nss( 10, 11 );

void setup(){
  Serial.begin(115200);
  delay(50);

  while (!Serial){};

  Serial.print( F("NeoSWSerial pass through @ ") );
  Serial.println( baudrate );
  Serial.flush();

  nss.begin(baudrate);
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
