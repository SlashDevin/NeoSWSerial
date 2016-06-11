#include <NeoSWSerial.h>

NeoSWSerial nss( 52, 53 );

  uint16_t baudrate = 9600;
  uint8_t  rc[520];
  uint16_t received = 0;
  uint16_t sent     = 0;

//---------------------------------------------------------------------
//#define INTER_CHAR_TIME 50
#define INTER_CHAR_TIME 0

//#define BITDUMP

#ifdef BITDUMP
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

//---------------------------------------------------------------------

static uint16_t errors = 0;

void testOne( Stream & ins, Stream & outs, uint8_t c )
{
  uint8_t pair[2];
  pair[0] = c;
  pair[1] = c+1;
  uint8_t len = 1;
  outs.write( pair, len );
  if (INTER_CHAR_TIME < 10)
    outs.flush();

  static bool dotPrinted = false;

  uint8_t  received = 0;
  bool     gotIt    = false;
  uint32_t start    = millis();
  
  do {
    if (ins.available()) {
      uint8_t received_c = ins.read();

      dotPrinted = false;

      gotIt = (received_c == c);

      if (!gotIt || (INTER_CHAR_TIME > 10)) {
        Serial.print( F("rx ") );
        if (received_c < 0x10) Serial.print( '0' );
        Serial.print( received_c, HEX );

        if (!gotIt) {
          Serial.print( F(" != tx ") );
          if (c < 0x10) Serial.print( '0' );
          Serial.print( 0xFF & (c), HEX );
        }

        if (received == 0) {
          dumpBits();
          dumpDiffs();
          Serial.print( ' ' );
        }
      }

      received++;
      c++;
    }
  } while (millis() - start < INTER_CHAR_TIME);

  if (INTER_CHAR_TIME > 10) {
    if (!received) {
      Serial.print( '.' );
      dumpBits();
      dumpDiffs();
      Serial.println();
      dotPrinted = true;
    } else
      Serial.println();

    Serial.flush();
  }

} // testOne

//---------------------------------------------------------------------

static void testTwo( uint8_t & c, uint8_t & received_c )
{
  if (nss.available()) {
    uint8_t rcActual = nss.read();
    if (rcActual != received_c) {
      errors++;
      Serial.print( F("rx ") );
      if (received_c < 0x10) Serial.print( '0' );
      Serial.print( 0xFF & rcActual, HEX );
      Serial.print( F(" != tx ") );
      if (received_c < 0x10) Serial.print( '0' );
      Serial.print( 0xFF & (received_c), HEX );
      dumpBits();
      dumpDiffs();
      Serial.println();
    }
    received_c++;
  }
} // testTwo

//---------------------------------------------------------------------

static void printStats()
{
  if (received != sent) {
    Serial.print( received );
    Serial.print( F(" received, ") );
    Serial.print( sent );
    Serial.println( F(" sent") );
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

  received           = 0;
  sent               = 0;
  errors             = 0;

} // printStats

//---------------------------------------------------------------------

static void printRC()
{
  for (uint16_t i=0; i < received; i++) {
    uint8_t c = rc[i];
    if (c < 0x10) Serial.print( '0' );
    Serial.print( 0xFF & (c), HEX );
    Serial.print( ' ' );
    if ((i & 0x1F) == 0x1F)
      Serial.println();
  }
  Serial.println();
} // printRC

//---------------------------------------------------------------------

void setup() { 
 //Initialize serial and wait for port to open:
  Serial.begin(9600); 
  while (!Serial)
    ;

  Serial3.begin( baudrate );
  delay( 10 );
  Serial3.print( 'U' );
  Serial3.flush();
  delay( 10 );
  
  nss.begin( baudrate );

} // setup

//---------------------------------------------------------------------

void loop()
{
  Serial.print( F("NeoSWSerial test @ ") );
  Serial.println( baudrate );
  Serial.flush();

  Serial.println( F("Individual RX test") );
  Serial.flush();
  
  uint8_t c=0;
  do {
    testOne( nss, Serial3, c );
    c++;
  } while (c != 0);
  testOne( nss, Serial3, c );
  uint32_t start = millis();
  while (millis() - start < 100) {
    if (nss.available()) {
      nss.read();
      start = millis();
    }
  }

  printStats();

  //=====================================

  Serial.println( F("RX test") );
  Serial.flush();
  
  for (uint8_t times=0; times<1; times++) {
    do {
      Serial3.write( c++ );
      sent++;
      if (nss.available())
        rc[ received++ ] = nss.read();
    } while (c != 0);
  }
  Serial3.write( c++ );
  sent++;

  start = millis();
  while (millis() - start < 100) {
    if (nss.available()) {
      rc[ received++ ] = nss.read();
      start = millis();
    }
  }

  if (received < sent) {
    Serial.print( sent-received );
    Serial.println( F(" chars dropped.") );
  }
  printRC();

  printStats();

  //=====================================

  Serial.println( F("TX test") );
  Serial.flush();
  
  for (uint16_t i=0; i<=256; i++) {
    nss.write( (uint8_t) (i & 0xFF) );
    if (Serial3.available())
      rc[ received++ ] = Serial3.read();
  }

  start = millis();

  uint32_t ms;
  do {
    ms = millis();
    if (Serial3.available()) {
      start = ms;
      rc[ received++ ] = Serial3.read();
    }
  } while (ms - start < 100);
  
  printRC();

  printStats();

  //=====================================

  Serial.println( F("RX and TX test") );
  Serial.flush();
  
  for (uint16_t i=0; i<=256; i++) {
    Serial3.write( (uint8_t) i & 0xFF );
    if (nss.available()) {
      c = nss.read();
      nss.write( c );
    }
    if (Serial3.available())
      rc[ received++ ] = Serial3.read();
  }

  start = millis();

  do {
    ms = millis();
    if (nss.available()) {
      start = ms;
      c = nss.read();
      nss.write( c );
    }
    if (Serial3.available()) {
      start = ms;
      rc[ received++ ] = Serial3.read();
    }
  } while (ms - start < 100);
  
  printRC();

  printStats();

  //=====================================

  Serial.println( F("TX and RX test") );
  Serial.flush();
  
  for (uint16_t i=0; i<=256; i++) {
    nss.write( (uint8_t) i & 0xFF );
    if (Serial3.available()) {
      c = Serial3.read();
      Serial3.write( c );
    }
    while (nss.available())
      rc[ received++ ] = nss.read();
  }

  start = millis();

  do {
    ms = millis();
    if (Serial3.available()) {
      start = ms;
      c = Serial3.read();
      Serial3.write( c );
    }
    while (nss.available()) {
      start = ms;
      rc[ received++ ] = nss.read();
    }
  } while (ms - start < 100);
  
  printRC();

  printStats();

  //=====================================

  while (!Serial.available())
    ;

  do {
    while (Serial.available()) {
      Serial.read();
      start = millis();
    }
  } while (millis() - start < 20);

  if (baudrate == 38400)
    baudrate = 9600;
  else
    baudrate <<= 1;
  nss.setBaudRate( baudrate );    
  Serial3.begin( baudrate );
} 
