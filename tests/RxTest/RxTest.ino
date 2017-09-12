#include <Arduino.h>
#include <NeoSWSerial.h>

#define BAUD 57600

NeoSWSerial nss( 10, 11 );

void setup(){
  Serial.begin(57600);
  delay(50);

  while (!Serial){};
  Serial.print("Test at ");
  Serial.println(BAUD);

  Serial1.begin(BAUD);
  delay(50);

  nss.begin(BAUD);
  delay(50);

}

void loop() {
    // char endline = '\n';
    // String readstr = Serial1.readStringUntil(endline);
    // if (readstr.length() > 0)
    // {
    //     Serial.print("Serial1 - ");
    //     Serial.println(readstr);
    // }
    // else Serial.println("Serial1 - no data");
    //
    // String readstr2 = nss.readStringUntil(endline);
    // if (readstr2.length() > 0)
    // {
    //     Serial.print("                       NSS - ");
    //     Serial.println(readstr2);
    // }
    // else Serial.println("                       NSS - no data");

    char readSer, readNSS;
    readSer = Serial1.read();
    readNSS = nss.read();

    if (readSer > 0 and readNSS > 0)
    {
        Serial.print(readSer);
        Serial.print(" (");
        Serial.print(readSer, HEX);
        Serial.print(")");
        Serial.print("   -   ");
        Serial.print(readNSS);
        Serial.print(" (");
        Serial.print(readNSS, HEX);
        Serial.print(")");
        Serial.println();
    }
    else if (readSer > 0)
    {
        Serial.print(readSer);
        Serial.print(" (");
        Serial.print(readSer, HEX);
        Serial.print(")");
        Serial.print("   -  ? (##)");
        Serial.println();
    }
    else if (readNSS > 0)
    {
        Serial.print("? (##)  -   ");
        Serial.print(readNSS);
        Serial.print(" (");
        Serial.print(readNSS, HEX);
        Serial.print(")");
        Serial.println();
    }
}
