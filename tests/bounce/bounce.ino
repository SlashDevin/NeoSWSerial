#include <Arduino.h>

uint16_t baudrate = 9600;
HardwareSerial &checkStream1 = Serial1;
HardwareSerial &checkStream2 = Serial2;

// prints a single character prettily
void printCharHex(uint8_t c)
{
    if (c < 0x10) Serial.print( '0' );
    Serial.print( 0xFF & (c), HEX );
    Serial.print( ' ' );
    if ((c & 0x0F) == 0x0F)
      Serial.println();
    if ((c & 0xFF) == 0xFF)
      Serial.println();
} // printChar

void setup(){
  Serial.begin(115200);
  delay(50);
  checkStream1.begin(baudrate);
  delay(50);
  checkStream2.begin(baudrate);
  delay(50);

  Serial.print(F("Serial bouncer at "));
  Serial.println(baudrate);

}

void loop() {
  if (checkStream1.available())
  {
    uint8_t c = checkStream1.read();
    printCharHex(c);
    checkStream1.write(c);
  }
  if (checkStream2.available())
  {
    uint8_t c = checkStream2.read();
    printCharHex(c);
    checkStream2.write(c);
  }

  // if given input on serial, repeat everything at the next baud rate
  if ( Serial.available() ) {
    Serial.println(Serial.readString());

    if (baudrate == 38400)
      baudrate = 31250;
    else if (baudrate == 31250)
      baudrate = 9600;
    else
      baudrate <<= 1;
    checkStream1.begin( baudrate );
    checkStream2.begin( baudrate );

    Serial.print(F("Serial bouncer at "));
    Serial.println(baudrate);
  }
}
