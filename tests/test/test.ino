#include <NeoSWSerial.h>

NeoSWSerial nss( 10, 11 );

HardwareSerial &checkSerial = Serial1;
// NeoSWSerial &checkSerial = nss;

uint16_t baudrate = 9600;
uint8_t  rc[770];  // Received character array
uint8_t  sc[770];  // Sent character array - only for error checking
uint16_t received = 0;  // # characters received
uint16_t dropped = 0;  // # characters received
uint16_t sent     = 0;  // # characters sent
static uint16_t errors = 0;  // # errors

//---------------------------------------------------------------------
#define INTER_CHAR_TIME 50
// #define INTER_CHAR_TIME 0

//#define BITDUMP

#ifdef BITDUMP
  #define DEBUG_NEOSWSERIAL
  extern uint8_t bitTransitionTimes[];
  extern uint8_t bitTransitions;
  struct rbd_t { uint8_t loop_bits; uint8_t mul_bits; uint16_t prod; };
  extern rbd_t diffRXbits[];
  extern uint8_t diffRXbitsCount;

  extern uint16_t availCompletions;
  extern uint16_t rxStartCompletions;
  extern uint8_t  rxStartCompletionBits[];
  extern uint16_t checkRxCompletions;
  extern uint16_t polledPCI;
  extern uint16_t polledPCICompletions;
  extern uint16_t stopBitCompletions;
  extern uint16_t highBitWaits;
#endif

//---------------------------------------------------------------------

static void dumpBits()
{
#ifdef BITDUMP
  for (uint8_t i=0; i<bitTransitions; i++) {
    Serial.print( ' ' );
    Serial.print( bitTransitionTimes[i] );
  }
#endif
} // dumpBits

//---------------------------------------------------------------------

static void dumpDiffs()
{
#ifdef BITDUMP
  if (diffRXbitsCount > 0) {
    Serial.print( F(" (") );
    for (uint8_t i=0;;) {
      Serial.print( diffRXbits[i].prod );
      Serial.print( ' ' );
      Serial.print( diffRXbits[i].mul_bits );
      Serial.print( F(" != ") );
      Serial.print( diffRXbits[i].loop_bits );
      i++;
      if (i >= diffRXbitsCount)
        break;
      Serial.print( F(", ") );
    }
    Serial.print( ')' );
    diffRXbitsCount = 0;
  }
#endif
} // dumpDiffs


void emptyBuffer ( Stream & ins ) {
  uint32_t start = millis();
  while (millis() - start < 100) {
    if (ins.available()) {
      ins.read();
      start = millis();
    }
  }
}

//---------------------------------------------------------------------

// prints a single character prettily
void printCharHex(uint8_t c)
{
    if (c < 0x10) Serial.print( '0' );
    Serial.print( 0xFF & (c), HEX );
} // printChar

// prints the whole received character array
void printRC()
{
  Serial.println(F("Received: "));
  for (uint16_t i=0; i < received; i++) {
    uint8_t c = rc[i];
    printCharHex(c);
    Serial.print( ' ' );
    if ((i & 0x1F) == 0x1F)
      Serial.println();
  }
  Serial.println();
} // printRC

// prints the whole sent character array
void printSC()
{
  Serial.println(F("Sent: "));
  for (uint16_t i=0; i < sent; i++) {
    uint8_t c = sc[i];
    printCharHex(c);
    Serial.print( ' ' );
    if ((i & 0x1F) == 0x1F)
      Serial.println();
  }
  Serial.println();
} // printSC

// Checks for errors in the receive array vs the sc array
void checkRC()
{
  // printSC();
  // printRC();
  for (uint16_t i=0; i < received; i++) {
    uint8_t cActual = rc[i];
    uint8_t cExpected = sc[i];
    if (cActual != cExpected) {
      errors++;
      // Serial.print( F("rx ") );
      // printCharHex(cActual);
      // Serial.print( F(" != tx ") );
      // printCharHex(cExpected);
      // Serial.println();
    }
  }
} // checkRC

static void printStats()
{
  if (received != sent) {
    Serial.print( received );
    Serial.print( F(" received, ") );
    Serial.print( sent );
    Serial.println( F(" sent") );
  }
  if (received < sent) {
    Serial.print( sent-received );
    Serial.println( F(" chars dropped.") );
  }


  Serial.print( errors );
  Serial.println( F(" errors") );
#ifdef BITDUMP
  Serial.print( rxStartCompletions );
  Serial.print( F(" RX start completions:") );
  for (uint16_t i=0; i < rxStartCompletions; i++) {
    Serial.print( ' ' );
    Serial.print( rxStartCompletionBits[i] );
  }
  Serial.println();

  Serial.print( availCompletions );
  Serial.println( F(" available() completions") );
  Serial.print( checkRxCompletions );
  Serial.println( F(" checkRxTime completions") );
  Serial.print( polledPCI );
  Serial.println( F(" polled PCI detected") );
  Serial.print( polledPCICompletions );
  Serial.println( F(" polled PCI completions") );
  Serial.print( stopBitCompletions );
  Serial.println( F(" stop bit completions") );
  Serial.print( highBitWaits );
  Serial.println( F(" high bit waits") );

  // Zero everything after we've printed it
  rxStartCompletions   = 0;
  availCompletions     = 0;
  checkRxCompletions   = 0;
  polledPCI            = 0;
  polledPCICompletions = 0;
  stopBitCompletions   = 0;
  highBitWaits         = 0;
#endif

  Serial.println( F("-----------------") );
  Serial.flush();

  // Zero everything after we've printed it
  received           = 0;
  sent               = 0;
  errors             = 0;

} // printStats


//---------------------------------------------------------------------

// In the first round of testing, the sketch waits for the echo of each character.

// With 2 Arduinos, this tests whether the send process works independently
// from the receive process (the 2nd Arduino echo doesn't begin until the 1st
// Arduino is done sending).
// With 1 Arduino + loopback wires, this tests whether the receive process works
// during the transmit process (each sent bit is simulataneously received).

// And by incrementing the sent character, all byte values can be tested.

void testOne( Stream & ins, Stream & outs, uint8_t c )
{

  // Write one character out and wait for it to finish
  uint8_t pair[2];
  pair[0] = c;
  pair[1] = c+1;
  uint8_t len = 1;
  outs.write( pair, len );
  sent++;
  if (INTER_CHAR_TIME < 10)
    outs.flush();

  bool     gotIt    = false;
  uint32_t start    = millis();

  do {
    if (ins.available()) {
      uint8_t received_c = ins.read();

      gotIt = (received_c == c);

      if (!gotIt /*|| (INTER_CHAR_TIME > 10*/) {
        Serial.print( F("rx ") );
        printCharHex(received_c);

        if (!gotIt) {
          Serial.print( F(" != tx ") );
          printCharHex(c);
          errors++;
        }

        if (received == 0) {
          dumpBits();
          dumpDiffs();
          Serial.print( ' ' );
        }

        Serial.println();
      }

      received++;
      c++;
    }
  } while (millis() - start < INTER_CHAR_TIME + 10);  // Give a little time for bounce with two units

  /*if (INTER_CHAR_TIME > 10) {
    if (!received) {
      Serial.print( '.' );
      dumpBits();
      dumpDiffs();
      Serial.println();
    } else
      Serial.println();

    Serial.flush();
  }*/

} // testOne


// In the second round of testing, the sketch sends the same characters as fast
// as it can (no inter-character wait time).
// NeoSWSerial writes each byte immediately; there is no TX buffer.

// With 2 Arduinos, the echo from the second Arduino begins coming in after the
// 1st Arduino finishes sending the first byte. Then the 1st Arduino starts
// sending the second byte.

// With 1 Arduino + loopback wires, the first character arrives at the same time
// it is being sent.

// This tests the simultaneous sending and receiving parts of NeoSWSerial,
// synchronously (1 Arduino + loopback wires) and asynchronously (2 Arduinos).

void testTwo( Stream & ins, Stream & outs, uint8_t numRepeats )
{
  for (uint8_t times=0; times<numRepeats; times++) {
    for (uint16_t i=0; i<=256; i++) {
      outs.write( (uint8_t) (i & 0xFF) );
      sc [ sent++ ] = (uint8_t) (i & 0xFF);
      if (ins.available())  // If anything becomes between characters, read it
        rc[ received++ ] = ins.read();
    }
  }

  // Read any remaining characters into the Rx array
  uint32_t start = millis();
  while (millis() - start < 100) {
    if (ins.available()) {
      rc[ received++ ] = ins.read();
      start = millis();
    }
  }

  // Print results
  checkRC();
  printStats();

}  // testTwo


// This third round of testing goes one step past the second and looks for errors
// in the character that's been read and written back and read again.

void testThree( Stream & ins, Stream & outs, uint8_t numRepeats )
{
  uint8_t c;

  // Throw away anything in either buffer before starting test
  uint32_t start = millis();
  while (millis() - start < 100) {
    if (ins.available()) {
      ins.read();
      start = millis();
    }
  }
  while (millis() - start < 100) {
    if (outs.available()) {
      outs.read();
      start = millis();
    }
  }

  for (uint8_t times=0; times<numRepeats; times++) {
    for (uint16_t i=0; i<=256; i++) {
      outs.write( (uint8_t) (i & 0xFF) );
      sc [ sent++ ] = (uint8_t) (i & 0xFF);
      if (ins.available()) { // If anything becomes between characters, read it
        c = ins.read();
        ins.write ( c );  // And immediately echo back what was read
      }
    if (outs.available())  // If the bounce-back makes it home, read it
      rc[ received++ ] = outs.read();
    }
  }

  // Read and bounce any remaining characters
  uint32_t ms = millis();

  do {
    ms = millis();
    if (ins.available()) {
      start = ms;
      c = ins.read();
      ins.write( c );
    }
    if (outs.available()) {
      start = ms;
      rc[ received++ ] = outs.read();
    }
  } while (ms - start < 100);

  // Print results
  checkRC();
  printStats();

}  // testThree

//---------------------------------------------------------------------

void setup() {
 //Initialize serial and wait for port to open:
  Serial.begin(9600);
  while (!Serial)
    ;

  checkSerial.begin( baudrate );
  delay( 10 );
  checkSerial.print( 'U' );
  checkSerial.flush();
  delay( 10 );

  nss.begin( baudrate );

  #if F_CPU == 16000000L
    // Use Timer 0 (TCNT0) 8-bit timer w/ PWM, default prescaler has divisor of 64, thus 250kHz
    Serial.print("TCCR0A: 0x0");  Serial.println(TCCR0A, HEX);
    // ^^ Should be  0x03=0b00000011 - OC0A & OC0B disconnected, Fast PWM
    Serial.print("TCCR0B: 0x0");  Serial.println(TCCR0B, HEX);
    // ^^ Should be  0x03=0b00000011 - Clock Select bits 01 & 00 on - prescaling source = CK/64, timer at 16MHz/64 = 250kHz

  // Have to use alternate timer for an 8 MHz system because timer 0 doesn't have the correct prescaler
  #elif F_CPU == 8000000L
    #if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
        // Use Timer 1 (TCNT1) 8-bit timer/counter w/ independent prescaler
      Serial.print("TCCR1: 0x0");  Serial.println(TCCR1, HEX);
      // ^^ Should be  0x06=0b00000110 - Clock Select bits 12 & 11 on - prescaling source = CK/32, timer at 8MHz/32 = 250kHz

    #elif defined(__AVR_ATmega32U4__)
      // Use Timer 4 (TCNT4) 10-bit high speed timer, usable as 8-bit timer by ignoring high bits
      Serial.print("TCCR4A: 0x0");  Serial.println(TCCR4A, HEX);
      // ^^ Should be  0x00 - "normal" operation - Normal port operation, OC4A & OC4B disconnected
      Serial.print("TCCR4B: 0x0");  Serial.println(TCCR4B, HEX);
      // ^^ Should be  0x06=0b00000110 - Clock Select bits 42 & 41 on - prescaler set to CK/32, timer at 8MHz/32 = 250kHz
      Serial.print("TCCR4C: 0x0");  Serial.println(TCCR4C, HEX);
      // ^^ Should be  0x00 - "normal" operation - Normal port operation, OC4D0 disconnected
      Serial.print("TCCR4D: 0x0");  Serial.println(TCCR4D, HEX);
      // ^^ Should be  0x00 - No fault protection
      Serial.print("TCCR4E: 0x0");  Serial.println(TCCR4E, HEX);
      // ^^ Should be  0x00 - No register locks or overrides

    #else
      // Use Timer 2 (TCNT2) 8-bit timer w/ PWM, asynchronous operation, and independent prescaler
      Serial.print("TCCR2A: 0x0");  Serial.println(TCCR2A, HEX);
      // ^^ Should be  0x00 - "normal" operation - Normal port operation, OC2A & OC2B disconnected
      Serial.print("TCCR2B: 0x0");  Serial.println(TCCR2B, HEX);
      // ^^ Should be  0x03=0b00000011 - Clock Select bits 21 & 20 on - prescaler set to clkT2S/32, timer at 8MHz/32 = 250kHz
    #endif

  #endif

} // setup

//---------------------------------------------------------------------

void loop()
{
  Serial.print( F("NeoSWSerial test @ ") );
  Serial.println( baudrate );
  Serial.flush();

  Serial.println( F("Individual Character RX test") );
  Serial.flush();
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);
  
  received           = 0;
  sent               = 0;
  errors             = 0;

  uint8_t c=0;
  do {
    testOne( nss, checkSerial, c );
    c++;
  } while (c != 0);  // repeat until c wraps back around at 256

  printStats();

  // Clear buffers before starting the next test
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  //=====================================

  Serial.println( F("RX test") );
  Serial.flush();
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  // Send out all the characters on a hardware serial port and read them over NSS
  testTwo(nss, checkSerial, 3);

  // Clear buffers before starting the next test
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  //=====================================

  Serial.println( F("TX test") );
  Serial.flush();
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  // Send out all the characters on NSS and read them over a hardware serial port
  testTwo(checkSerial, nss, 3);

  // Clear buffers before starting the next test
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  //=====================================

  Serial.println( F("RX and TX test") );
  Serial.flush();
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  // Send out all the characters on a hardware serial port and read them and
  // immeciately send them back over NSS
  testThree(nss, checkSerial, 3);

  // Clear buffers before starting the next test
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  //=====================================

  Serial.println( F("TX and RX test") );
  Serial.flush();
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  // Send out all the characters on NSS and read them and immeciately send them
  // back over a hardware serial port
  testThree(checkSerial, nss, 3);

  // Clear buffers before starting the next test
  delay(1000);
  emptyBuffer(nss);
  emptyBuffer(checkSerial);
  delay(1000);

  //=====================================

  // if given input on serial, repeat everything at the next baud rate
  uint32_t start = millis();
  while (!Serial.available())
    ;

  do {
    while (Serial.available()) {
      Serial.read();
      start = millis();
    }
  } while (millis() - start < 20);

  if (baudrate == 38400)
    baudrate = 31250;
  else if (baudrate == 31250)
    baudrate = 9600;
  else
    baudrate <<= 1;
  nss.setBaudRate( baudrate );
  checkSerial.begin( baudrate );
}
