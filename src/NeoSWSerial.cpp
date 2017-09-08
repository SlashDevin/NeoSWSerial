//
// NeoSWSerial
// Copyright (C) 2015-2017, SlashDevin
//
// NeoSWSerial is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// NeoSWSerial is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details:
//
//     <http://www.gnu.org/licenses/>.
//
// Methods
// -------
//
// begin(baudRate) - initialization, optionally set baudrate, and then enable RX
// listen() - enables RX interrupts, allowing data to enter the RX buffer
// ignore() - disables RX interrupts
// setBaudRate(baudRate) - selects the baud rate (9600, 19200, 31250, 38400)
//                             - any other value is ignored
// available() - returns the number of characters in the RX buffer
// read() - returns a single character from the buffer
// write(s) - transmits a string
//
// print() is supported
//=============================================================================

#include <NeoSWSerial.h>

// Default baud rate is 9600
static const uint8_t TICKS_PER_BIT_9600 = (uint8_t) 26;
                              // 9600 baud bit width in units of 4us
static const uint8_t TICKS_PER_BIT_31250 = 8;
                              // 31250 baud bit width in units of 4us

static const uint8_t BITS_PER_TICK_31250_Q10 = 128;
                     // 31250 bps * 0.000004 s * 2^10 "multiplier"
static const uint8_t BITS_PER_TICK_38400_Q10 = 157;
                     // 1s/(38400 bits) * (1 tick)/(4 us) * 2^10  "multiplier"

#if F_CPU == 16000000L
  #define TCNTX TCNT0
  #define PCI_FLAG_REGISTER PCIFR
#elif F_CPU == 8000000L
  #if defined(__AVR_ATtiny25__) | \
      defined(__AVR_ATtiny45__) | \
      defined(__AVR_ATtiny85__) 
    #define TCNTX TCNT1
    #define PCI_FLAG_REGISTER GIFR
  #else
    #define TCNTX TCNT2
    #define PCI_FLAG_REGISTER PCIFR
  #endif
#endif

static NeoSWSerial *listener = (NeoSWSerial *) NULL;

static uint8_t txBitWidth;
static uint8_t rxWindowWidth;
static uint8_t bitsPerTick_Q10; // multiplier!

static const uint8_t WAITING_FOR_START_BIT = 0xFF;
static uint8_t rxState;  // 0: got start bit; >0: bits rcvd
static uint8_t prev_t0;  // previous RX transition: timer0 time stamp (4us)
static uint8_t rxMask;   // bit mask for building received character
static uint8_t rxValue;  // character being built

static const uint8_t RX_BUFFER_SIZE = 64;  // power of 2 for optimal speed
static uint8_t rxBuffer[RX_BUFFER_SIZE];
static uint8_t rxHead;   // buffer pointer input
static uint8_t rxTail;   // buffer pointer output

static          uint8_t rxBitMask, txBitMask; // port bit masks
static volatile uint8_t *txPort;  // port register

//#define DEBUG_NEOSWSERIAL
#ifdef DEBUG_NEOSWSERIAL

  uint8_t bitTransitionTimes[16];
  uint8_t bitTransitions;
  static const uint8_t MAX_DIFF_RX_BITS = 8;
  struct rbd_t { uint8_t loop_bits; uint8_t mul_bits; uint16_t prod; };
  rbd_t diffRXbits[MAX_DIFF_RX_BITS];
  uint8_t diffRXbitsCount;

  uint16_t availCompletions;
  uint16_t rxStartCompletions;
  uint8_t rxStartCompletionBits[128];
  uint16_t checkRxCompletions;
  uint16_t polledPCI;
  uint16_t polledPCICompletions;
  uint16_t stopBitCompletions;

  #define DBG_NSS_COUNT(v) { v++; }
  #define DBG_NSS_COUNT_RESET(v) { v = 0; }
  #define DBG_NSS_ARRAY(a,i,v) \
    { if (i < sizeof(a)/sizeof(a[0])) \
      a[ i++ ] = v; }

#else

  #define DBG_NSS_COUNT(v)
  #define DBG_NSS_COUNT_RESET(v)
  #define DBG_NSS_ARRAY(a,i,v)

#endif

static uint16_t mul8x8to16(uint8_t x, uint8_t y)
{return x*y;}

//..........................................

static uint16_t bitTimes( uint8_t dt )
{
  return mul8x8to16( dt + rxWindowWidth, bitsPerTick_Q10 ) >> 10;

} // bitTimes

//----------------------------------------------------------------------------

void NeoSWSerial::begin(uint16_t baudRate)
{
  setBaudRate( baudRate );
  listen();
}

//----------------------------------------------------------------------------

void NeoSWSerial::listen()
{
  if (listener)
    listener->ignore();

  pinMode(rxPin, INPUT);
  rxBitMask = digitalPinToBitMask( rxPin );
  rxPort    = portInputRegister( digitalPinToPort( rxPin ) );

  txBitMask = digitalPinToBitMask( txPin );
  txPort    = portOutputRegister( digitalPinToPort( txPin ) );
  if (txPort)
    *txPort  |= txBitMask;   // high = idle
  pinMode(txPin, OUTPUT);

  if (F_CPU == 8000000L) {
    // Have to use timer 2 for an 8 MHz system.
    #if defined(__AVR_ATtiny25__) | \
        defined(__AVR_ATtiny45__) | \
        defined(__AVR_ATtiny85__) 
      TCCR1  = 0x06;  // divide by 32
    #else
      TCCR2A = 0x00;
      TCCR2B = 0x03;  // divide by 32
    #endif
  }

  volatile uint8_t *pcmsk = digitalPinToPCMSK(rxPin);
  if (pcmsk) {
    rxState  = WAITING_FOR_START_BIT;
    rxHead   = rxTail = 0;    // no characters in buffer
    flush();

    // Set up timings based on baud rate
    
    switch (_baudRate) {
      case 9600:
        txBitWidth      = TICKS_PER_BIT_9600          ;
        bitsPerTick_Q10 = BITS_PER_TICK_38400_Q10 >> 2;
        rxWindowWidth   = 10;
        break;
      case 31250:
        if (F_CPU > 12000000L) {
          txBitWidth = TICKS_PER_BIT_31250;
          bitsPerTick_Q10 = BITS_PER_TICK_31250_Q10;
          rxWindowWidth = 5;
          break;
        } // else use 19200
      case 38400:
        if (F_CPU > 12000000L) {
          txBitWidth      = TICKS_PER_BIT_9600    >> 2;
          bitsPerTick_Q10 = BITS_PER_TICK_38400_Q10   ;
          rxWindowWidth   = 4;
          break;
        } // else use 19200
      case 19200:
        txBitWidth      = TICKS_PER_BIT_9600      >> 1;
        bitsPerTick_Q10 = BITS_PER_TICK_38400_Q10 >> 1;
        rxWindowWidth   = 6;
        break;
    }

    // Enable the pin change interrupts

    uint8_t prevSREG = SREG;
    cli();
    {
      *pcmsk                    |= _BV(digitalPinToPCMSKbit(rxPin));
      *digitalPinToPCICR(rxPin) |= _BV(digitalPinToPCICRbit(rxPin));
      listener = this;
    }
    SREG = prevSREG;
  }

} // listen

//----------------------------------------------------------------------------

void NeoSWSerial::ignore()
{
  if (listener) {
    volatile uint8_t *pcmsk = digitalPinToPCMSK(rxPin);

    uint8_t prevSREG = SREG;
    cli();
    {
      listener = (NeoSWSerial *) NULL;
      if (pcmsk) {
        *digitalPinToPCICR(rxPin) &= ~_BV(digitalPinToPCICRbit(rxPin));
        *pcmsk &= ~_BV(digitalPinToPCMSKbit(rxPin));
      }
    }
    SREG = prevSREG;
  }

} // ignore

//----------------------------------------------------------------------------

void NeoSWSerial::setBaudRate(uint16_t baudRate)
{
  if ((
        ( baudRate ==  9600) ||
        ( baudRate == 19200) ||
        ((baudRate == 31250) && (F_CPU == 16000000L)) ||
        ((baudRate == 38400) && (F_CPU == 16000000L))
       )
           &&
      (_baudRate != baudRate)) {

    _baudRate = baudRate;

    if (this == listener)
      listen();
  }
} // setBaudRate

//----------------------------------------------------------------------------

int NeoSWSerial::available()
{
  uint8_t avail = ((rxHead - rxTail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE);
  
  if (avail == 0) {
    cli();
      if (checkRxTime()) {
        avail = 1;
        DBG_NSS_COUNT(availCompletions);
      }
    sei();
  }

  return avail;

} // available

//----------------------------------------------------------------------------

int NeoSWSerial::read()
{
  if (rxHead == rxTail) return -1;
  uint8_t c = rxBuffer[rxTail];
  rxTail = (rxTail + 1) % RX_BUFFER_SIZE;

  return c;

} // read

//----------------------------------------------------------------------------

void NeoSWSerial::attachInterrupt( isr_t fn )
{
  uint8_t oldSREG = SREG;
  cli();
    _isr = fn;
  SREG = oldSREG;

} // attachInterrupt

//----------------------------------------------------------------------------

void NeoSWSerial::startChar()
{
  rxState = 0;     // got a start bit
  rxMask  = 0x01;  // bit mask, lsb first
  rxValue = 0x00;  // RX character to be, a blank slate

  DBG_NSS_COUNT_RESET(bitTransitions);

} // startChar

//----------------------------------------------------------------------------

void NeoSWSerial::rxISR( uint8_t rxPort )
{
  uint8_t t0 = TCNTX;            // time of data transition (plus ISR latency)
  uint8_t d  = rxPort & rxBitMask; // read RX data level

  if (rxState == WAITING_FOR_START_BIT) {

    // If it looks like a start bit then initialize;
    //   otherwise ignore the rising edge and exit.

    if (d != 0) return;   // it's high so not a start bit, exit
    startChar();

  } else {  // data bit or stop bit (probably) received

    DBG_NSS_ARRAY(bitTransitionTimes, bitTransitions, (t0-prev_t0));

    // Determine how many bit periods have elapsed since the last transition.

    uint16_t rxBits          = bitTimes( t0-prev_t0 );
    uint8_t  bitsLeft        = 9 - rxState; // ignores stop bit
    bool     nextCharStarted = (rxBits > bitsLeft);

    if (nextCharStarted)
      DBG_NSS_ARRAY(rxStartCompletionBits,rxStartCompletions,(10*rxBits + bitsLeft));

    uint8_t  bitsThisFrame   =  nextCharStarted ? bitsLeft : rxBits;

    rxState += bitsThisFrame;

    // Set all those bits

    if (d == 0) {
      // back fill previous bits with 1's
      while (bitsThisFrame-- > 0) {
        rxValue |= rxMask;
        rxMask   = rxMask << 1;
      }
      rxMask = rxMask << 1;
    } else { // d==1
      // previous bits were 0's so only this bit is a 1.
      rxMask   = rxMask << (bitsThisFrame-1);
      rxValue |= rxMask;
    }

    // If 8th bit or stop bit then the character is complete.

    if (rxState > 7) {
      rxChar( rxValue );

      if ((d == 1) || !nextCharStarted) {
        rxState = WAITING_FOR_START_BIT;
        // DISABLE STOP BIT TIMER

      } else {
        // The last char ended with 1's, so this 0 is actually
        //   the start bit of the next character.

        startChar();
      }
    }
  }

  prev_t0 = t0;  // remember time stamp

} // rxISR

//----------------------------------------------------------------------------

bool NeoSWSerial::checkRxTime()
{
  if (rxState != WAITING_FOR_START_BIT) {

    uint8_t d  = *rxPort & rxBitMask;

    if (d) {
      // Ended on a 1, see if it has been too long
      uint8_t  t0        = TCNTX; // now
      uint16_t rxBits    = bitTimes( t0-prev_t0 );
      uint8_t  bitsLeft  = 9 - rxState;
      bool     completed = (rxBits > bitsLeft);

      if (completed) {
        DBG_NSS_COUNT(checkRxCompletions);

        while (bitsLeft-- > 0) {
          rxValue |= rxMask;
          rxMask   = rxMask << 1;
        }

        rxState = WAITING_FOR_START_BIT;
        rxChar( rxValue );
        if (!_isr)
          return true;
      }
    }
  }
  return false;

} // checkRxTime

//----------------------------------------------------------------------------

void NeoSWSerial::rxChar( uint8_t c )
{
  if (listener) {
    if (listener->_isr)
      listener->_isr( c );
    else {
      uint8_t index = (rxHead+1) % RX_BUFFER_SIZE;
      if (index != rxTail) {
        rxBuffer[rxHead] = c;
        rxHead = index;
      }
    }
  }

} // rxChar

//----------------------------------------------------------------------------

#ifdef NEOSWSERIAL_EXTERNAL_PCINT

  // Client code must call NeoSWSerial::rxISR(PINB) in PCINT handler

#else

  // Must define all of the vectors even though only one is used.

  // This handy PCINT code for different boards is based on PinChangeInterrupt.*
  // from the excellent Cosa project: http://github.com/mikaelpatel/Cosa

  #define PCINT_ISR(vec,pin)		\
  extern "C" { \
  ISR(PCINT ## vec ## _vect)		\
  {								              \
    NeoSWSerial::rxISR(pin);	  \
  } }

  #if defined(__AVR_ATtiny261__) | \
      defined(__AVR_ATtiny461__) | \
      defined(__AVR_ATtiny861__)

  ISR(PCINT0_vect)
  {
    if (GIFR & _BV(INTF0)) {
      NeoSWSerial::rxISR(PINA);
    } else {
      NeoSWSerial::rxISR(PINB);
    }
  }

  #elif defined(__AVR_ATtiny25__) | \
        defined(__AVR_ATtiny45__) | \
        defined(__AVR_ATtiny85__) 

  PCINT_ISR(0, PINB);

  #elif defined(__AVR_ATtiny24__) | \
        defined(__AVR_ATtiny44__) | \
        defined(__AVR_ATtiny84__) 

  PCINT_ISR(0, PINA);
  PCINT_ISR(1, PINB);

  #elif defined(__AVR_ATmega328P__)

  PCINT_ISR(0, PINB);
  PCINT_ISR(1, PINC);
  PCINT_ISR(2, PIND);

  #elif defined(__AVR_ATmega32U4__)

  PCINT_ISR(0, PINB);

  #elif defined(__AVR_AT90USB1286__)

  PCINT_ISR(0, PINB);

  #elif defined(__AVR_ATmega2560__)

  PCINT_ISR(0, PINB);
  PCINT_ISR(1, PINJ);
  PCINT_ISR(2, PINK);

  #elif defined(__AVR_ATmega1281__)

  PCINT_ISR(0, PINB);
  // PCINT8 on PE0 not supported.  Other 7 are on PJ0..6
  PCINT_ISR(1, PINJ);
  PCINT_ISR(2, PINK);

  #elif defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644P__)

  PCINT_ISR(0, PINA);
  PCINT_ISR(1, PINB);
  PCINT_ISR(2, PINC);
  PCINT_ISR(3, PIND);

  #elif defined(__AVR_ATmega2560RFR2__)

  PCINT_ISR(0, PINB);
  PCINT_ISR(1, PINE);

  #else
    #error MCU not supported by NeoSWSerial!
  #endif

#endif

//-----------------------------------------------------------------------------
// Instead of using a TX buffer and interrupt
// service, the transmit function is a simple timer0 based delay loop.
//
// Interrupts are disabled while the character is being transmitted and
// re-enabled after each character.

size_t NeoSWSerial::write(uint8_t txChar)
{
  if (!txPort)
    return 0;

  uint8_t width;         // ticks for one bit
  uint8_t txBit  = 0;    // first bit is start bit
  uint8_t b      = 0;    // start bit is low
  uint8_t PCIbit = bit(digitalPinToPCICRbit(rxPin));

  uint8_t prevSREG = SREG;
  cli();        // send the character with interrupts disabled

    uint8_t t0 = TCNTX; // start time

    // TODO: This would benefit from an early break after 
    //    the last 0 data bit.  Then we could wait for the
    //    remaining 1 data bits and stop bit with interrupts 
    //    re-enabled.

    while (txBit++ < 9) {   // repeat for start bit + 8 data bits
      if (b)      // if bit is set
        *txPort |= txBitMask;     //   set TX line high
      else
        *txPort &= ~txBitMask;    //   else set TX line low

      width = txBitWidth;
      if ((F_CPU == 16000000L) &&
          (width == TICKS_PER_BIT_9600/4) &&
          (txBit & 0x01)) {
        // The width is 6.5 ticks, so add a tick every other bit
        width++;  
      }

      // Hold the line for the bit duration

      while ((uint8_t)(TCNTX - t0) < width) {
        // Receive interrupt pending?
        if (PCI_FLAG_REGISTER & PCIbit) {
          PCI_FLAG_REGISTER |= PCIbit;   // clear it because...
          rxISR( *rxPort );  // ... this handles it
          DBG_NSS_COUNT(polledPCI);
        } else if (checkRxTime()) {
          DBG_NSS_COUNT(polledPCICompletions);
        }
      }
      t0    += width;         // advance start time
      b      = txChar & 0x01; // get next bit in the character to send
      txChar = txChar >> 1;   // shift character to expose the following bit
                   // Q: would a signed >> pull in a 1?
    }

  *txPort |= txBitMask;   // stop bit is high
  SREG = prevSREG;        // interrupts on for stop bit
  while ((uint8_t)(TCNTX - t0) < width) {
    if (checkRxTime())
      DBG_NSS_COUNT(stopBitCompletions);
  }

  return 1;               // 1 character sent

} // write
