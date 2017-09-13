#include <Arduino.h>
#include <NeoSWSerial.h>

#define BAUD 9600
#define HW_STREAM Serial1

#if !defined(FIRST_SENDER)
    #define FIRST_SENDER true
#endif

NeoSWSerial nss( 10, 11 );

char sentence1[] = "Quick zephyrs blow, vexing daft Jim.";
char sentence2[] = "Sphinx of black quartz, judge my vow.";
char sentence3[] = "The quick brown fox jumps over the lazy dog.";
char alphabet1[] = "abcdefghijklmnopqrstuvwwxyz<>?:{}|~!@#$%^&*()_+";
char alphabet2[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ,./;'[]\1234567890`-='";
char rxBuffer[50];

void waitForReady(bool isSending)
{
    bool isStarted = false;
    char rxByte[] = "2";
    Serial.println("Wait for ready");

    if (isSending)
    {
        while (!isStarted)
        {
            nss.print("1");  // Continually print out ones
            nss.readBytes(rxByte, 1);  // Listen for a single returned one
            if (rxByte[0] == '1') isStarted = true;
        }
    }
    else
    {
        // Listening for the "1" to start on neosoftware serial
        while (!isStarted)
        {
            HW_STREAM.readBytes(rxByte, 1);
            if (rxByte[0] == '1') isStarted = true;
        }
        HW_STREAM.print("1");  // Send a single 1 to acknlowlege that it's ready
    }
}


void sendOrRecieve(char sentence[], bool isSending, Stream *stream, String streamName)
{
    if (isSending)
    {
        delay(10);
        stream->print(sentence);
        stream->flush();
    }
    else
    {
        Serial.print("Known:    ");
        Serial.println(sentence);
        long start = millis();

        while ((stream->available() < strlen(sentence)) and ((millis() - start) < 1000)){}

        if (stream->available())
        {
            stream->readBytes(rxBuffer, strlen(sentence));
            Serial.print(streamName);
            Serial.print(" Rcvd: ");
            Serial.println(rxBuffer);

            Serial.print("Diff:     ");
            for (uint8_t i = 0; i < strlen(sentence); i++)
            {
                if (sentence[i] == rxBuffer[i]) Serial.print("-");
                else Serial.print("X");
            }
            Serial.println();

            for (uint8_t i = 0; i < 50; i++) rxBuffer[i] = 0;
        }
        else Serial.println("No data received");
    }
}


void setup()
{
  Serial.begin(57600);
  delay(50);

  while (!Serial){};
  Serial.print("Test of NeoSoftwareSerial at ");
  Serial.println(BAUD);

  HW_STREAM.begin(BAUD);
  delay(50);

  nss.begin(BAUD);
  delay(50);

  waitForReady(FIRST_SENDER);

  if (FIRST_SENDER)
  {
      Serial.println("I am sending data over NSS");
      sendOrRecieve(sentence1, true, &nss, "NSS");
      sendOrRecieve(sentence2, true, &nss, "NSS");
      sendOrRecieve(sentence3, true, &nss, "NSS");
      sendOrRecieve(alphabet1, true, &nss, "NSS");
      sendOrRecieve(alphabet2, true, &nss, "NSS");
      Serial.println();

      Serial.println("I am listening for data on NSS");
      sendOrRecieve(sentence1, false, &nss, "NSS");
      sendOrRecieve(sentence2, false, &nss, "NSS");
      sendOrRecieve(sentence3, false, &nss, "NSS");
      sendOrRecieve(alphabet1, false, &nss, "NSS");
      sendOrRecieve(alphabet2, false, &nss, "NSS");
      Serial.println();

      Serial.println("I am listening on Hardware Serial for data from NSS");
      sendOrRecieve(sentence1, false, &HW_STREAM, "HWS");
      sendOrRecieve(sentence2, false, &HW_STREAM, "HWS");
      sendOrRecieve(sentence3, false, &HW_STREAM, "HWS");
      sendOrRecieve(alphabet1, false, &HW_STREAM, "HWS");
      sendOrRecieve(alphabet2, false, &HW_STREAM, "HWS");
      Serial.println();

      Serial.println("I am sending data to NSS from Hardware Serial");
      sendOrRecieve(sentence1, true, &HW_STREAM, "HWS");
      sendOrRecieve(sentence2, true, &HW_STREAM, "HWS");
      sendOrRecieve(sentence3, true, &HW_STREAM, "HWS");
      sendOrRecieve(alphabet1, true, &HW_STREAM, "HWS");
      sendOrRecieve(alphabet2, true, &HW_STREAM, "HWS");
      Serial.println();

      Serial.println("Complete!");
  }

  else
  {
      Serial.println("I am listening on Hardware Serial for data from NSS");
      sendOrRecieve(sentence1, false, &HW_STREAM, "HWS");
      sendOrRecieve(sentence2, false, &HW_STREAM, "HWS");
      sendOrRecieve(sentence3, false, &HW_STREAM, "HWS");
      sendOrRecieve(alphabet1, false, &HW_STREAM, "HWS");
      sendOrRecieve(alphabet2, false, &HW_STREAM, "HWS");
      Serial.println();

      Serial.println("I am sending data to NSS from Hardware Serial");
      sendOrRecieve(sentence1, true, &HW_STREAM, "HWS");
      sendOrRecieve(sentence2, true, &HW_STREAM, "HWS");
      sendOrRecieve(sentence3, true, &HW_STREAM, "HWS");
      sendOrRecieve(alphabet1, true, &HW_STREAM, "HWS");
      sendOrRecieve(alphabet2, true, &HW_STREAM, "HWS");
      Serial.println();

      Serial.println("I am sending data over NSS");
      sendOrRecieve(sentence1, true, &nss, "NSS");
      sendOrRecieve(sentence2, true, &nss, "NSS");
      sendOrRecieve(sentence3, true, &nss, "NSS");
      sendOrRecieve(alphabet1, true, &nss, "NSS");
      sendOrRecieve(alphabet2, true, &nss, "NSS");
      Serial.println();

      Serial.println("I am listening for data on NSS");
      sendOrRecieve(sentence1, false, &nss, "NSS");
      sendOrRecieve(sentence2, false, &nss, "NSS");
      sendOrRecieve(sentence3, false, &nss, "NSS");
      sendOrRecieve(alphabet1, false, &nss, "NSS");
      sendOrRecieve(alphabet2, false, &nss, "NSS");
      Serial.println();

      Serial.println("Complete!");
  }
}

void loop(){}
