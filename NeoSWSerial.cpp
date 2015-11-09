//
// NeoSWSerial
//
// Methods
// -------
//
// begin(baudRate) - initialization, optionally set baudrate, and then enable RX
// listen() - enables RX interrupts, allowing data to enter the RX buffer
// ignore() - disables RX interrupts
// setBaudRate(baudRate) - selects the baud rate (9600, 19200, 38400) - any
//                             other value is ignored
// available() - returns the number of characters in the RX buffer
// read() - returns a single character from the buffer
// write(s) - transmits a string
//
// print() is supported
//=============================================================================

#include "NeoSWSerial.h"

// Default baud rate is 9600
static const uint8_t TIMER_TICKS_BIT_WIDTH_9600 = (uint8_t) 26; // 9600 baud bit width in units of 4us
static const uint8_t TIMER_HALF_BIT_WIDTH_9600 = (TIMER_TICKS_BIT_WIDTH_9600 / 2);
static const uint8_t FAST_DIVIDE_SHIFT_9600 = 11;  // 9600 baud bit shift count for fast divide
static const uint8_t FAST_DIVIDE_MULTIPLIER = 79;  // multiplier for fast divide

#if F_CPU == 16000000L
  #define TCNTX TCNT0
#elif F_CPU == 8000000L
  #define TCNTX TCNT2
#endif

static NeoSWSerial *listener = (NeoSWSerial *) NULL;

static uint8_t txBitWidth;
static uint8_t rxHalfBitWidth;
static uint8_t rxFastDivideShift;

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

//----------------------------------------------------------------------------
// Initialize, set the baudrate and enable RX
//----------------------------------------------------------------------------
void NeoSWSerial::begin(uint16_t baudRate)
{
  pinMode(rxPin, INPUT);
  rxBitMask = digitalPinToBitMask(rxPin);

  txBitMask = digitalPinToBitMask(txPin);
  txPort    = portOutputRegister(digitalPinToPort(txPin));
  *txPort  |= txBitMask;   // high = idle
  pinMode(txPin, OUTPUT);
  delay(1);                // need some idle time

#if F_CPU == 8000000L
  // Have to use timer 2 for an 8 MHz system.
  TCCR2A = 0x00;
  TCCR2B = 0x03;  // divide by 32
#endif

  setBaudRate(baudRate);
  listen();
}

//----------------------------------------------------------------------------
// Enable soft serial RX.
//----------------------------------------------------------------------------
void NeoSWSerial::listen()
{
  if (listener)
    listener->ignore();

  volatile uint8_t *pcmsk = digitalPinToPCMSK(rxPin);
  if (pcmsk) {
    listener = this;
    rxState  = WAITING_FOR_START_BIT;
    rxHead   = rxTail = 0;    // no characters in buffer

    uint8_t prevSREG = SREG;
    cli();
    {
      *pcmsk |= _BV(digitalPinToPCMSKbit(rxPin));
      *digitalPinToPCICR(rxPin) |= _BV(digitalPinToPCICRbit(rxPin));
    }
    SREG = prevSREG;
  }
}

//----------------------------------------------------------------------------
// Shut down RX interrupts.
//----------------------------------------------------------------------------
void NeoSWSerial::ignore()
{
  if (listener) {
    listener = (NeoSWSerial *) NULL;
    volatile uint8_t *pcmsk = digitalPinToPCMSK(rxPin);
    if (pcmsk)
      *pcmsk &= ~_BV(digitalPinToPCMSKbit(rxPin));
  }
}

//----------------------------------------------------------------------------
// Change the baud rate (9600, 19200, 38400).
// Unrecognized baud rates are ignored.
//----------------------------------------------------------------------------
void NeoSWSerial::setBaudRate(uint16_t baudRate)
{
  uint8_t width = TIMER_TICKS_BIT_WIDTH_9600;
  uint8_t shift = FAST_DIVIDE_SHIFT_9600;

  if (baudRate == 19200) {
    width = width >> 1;
    shift--;
#if F_CPU == 16000000L
  } else if (baudRate == 38400) {
    width = width >> 2;
    shift -= 2;
#endif
  } else if (baudRate != 9600) {
    return;
  }

  txBitWidth        = width;
  rxHalfBitWidth    = width >> 1;
  rxFastDivideShift = shift;
}

//----------------------------------------------------------------------------
// Return number of characters available.
//----------------------------------------------------------------------------
uint8_t NeoSWSerial::available()
{
  return((rxHead - rxTail + RX_BUFFER_SIZE) % RX_BUFFER_SIZE);
}

//----------------------------------------------------------------------------
// Returns received character.
//----------------------------------------------------------------------------
char NeoSWSerial::read()
{
  if (rxHead == rxTail) return 0;
  char c = rxBuffer[rxTail];
  rxTail = (rxTail + 1) % RX_BUFFER_SIZE;
  return c;
}

//----------------------------------------------------------------------------
// Set ISR callback (safely)
//----------------------------------------------------------------------------
void NeoSWSerial::attachInterrupt( isr_t fn )
{
  uint8_t oldSREG = SREG;
  cli();
    _isr = fn;
  SREG = oldSREG;
}

//----------------------------------------------------------------------------
// Invoked upon RX line level transition.
//----------------------------------------------------------------------------

void NeoSWSerial::rxISR( uint8_t rxPort )
{
  uint8_t t0 = TCNTX;          // time of data transition (plus ISR latency)
  uint8_t d  = rxPort & rxBitMask;  // read RX data level

  if (rxState == WAITING_FOR_START_BIT) {

    // If it looks like a start bit then initialize;
    //   otherwise ignore the rising edge and exit.

    if (d != 0) return;   // it's high so not a start bit, exit
    rxState = 0;     // got a start bit
    rxMask  = 0x01;  // bit mask, lsb first
    rxValue = 0x00;  // RX character to be, a blank slate

  } else {  // data bit or stop bit (probably) received

    // Determine how many bit periods have elapsed since the last transition.
    // Multiply & shift is ~10x faster than rxBits /= TIMER_TICKS_BIT_WIDTH.

    uint8_t rxBits;
    rxBits  = (t0 - prev_t0 + rxHalfBitWidth);  // add 1/2 bit to round result
    rxBits  = uint16_t(rxBits * FAST_DIVIDE_MULTIPLIER) >> rxFastDivideShift;
    rxState = rxBits + rxState;    // total bit periods since start


    // If the data is 0 then back fill previous bits with 1's.
    // If the data is 1 then previous bits were 0's so only this bit is a 1.

    if (d == 0) {                              // if RX level is low
      for (uint8_t i=0; i<rxBits; i++) {       // for all previous bits
       rxValue |= rxMask;                      // set them to 1
       rxMask   = rxMask << 1;                 // shift to next bit
      }
      rxMask = rxMask << 1;                    // shift to current bit
    } else {  // d==1)                         // else: RX level is high
      rxMask   = rxMask << (rxBits-1);         // shift to current bit
      rxValue |= rxMask;                       // and set it to 1
    }

    // If 8th bit or stop bit then the character is complete.
    // If it's the 8th bit the stop bit transition interrupt 
    //   will be ignored (above).

    if (rxState > 7) {  // if 8th bit or stop bit the character is complete
      rxState = WAITING_FOR_START_BIT;
      rxChar( rxValue );
    }
  } // endif start bit or data/stop bit

  prev_t0 = t0;               // remember time stamp
}

//----------------------------------------------------------------------------
// Buffer or dispatch one character (ISR context)
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

}

//----------------------------------------------------------------------------
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

#elif defined(__AVR_ATmega1284P__)

PCINT_ISR(0, PINA);
PCINT_ISR(1, PINB);
PCINT_ISR(2, PINC);
PCINT_ISR(3, PIND);

#elif defined(__AVR_ATmega2560RFR2__)

PCINT_ISR(0, PINB);
PCINT_ISR(1, PINE);

#elif
  #error MCU not supported by NeoSWSerial!
#endif

//=============================================================================

//-----------------------------------------------------------------------------
// Transmit a character.
//
// For GPS the expected usage is to send commands only for initialization or
// very infrequently thereafter. So instead of using a TX buffer and interrupt
// service, the transmit function is a simple timer0 based delay loop.
// Interrupts are disabled while the character is being transmitted and
// reenabled after each character.
//-----------------------------------------------------------------------------
size_t NeoSWSerial::write(uint8_t txChar)
{
  uint8_t t0, width;

  uint8_t txBit = 0;    // first bit is start bit
  uint8_t b     = 0;    // start bit is low

  uint8_t prevSREG = SREG;
  cli();        // send the character with interrupts disabled
  {
    t0 = TCNTX; // start time

    while (txBit++ < 9) {   // repeat for start bit + 8 data bits
      if (b)      // if bit is set
        *txPort |= txBitMask;     //   set TX line high
      else
        *txPort &= ~txBitMask;    //   else set TX line low
      width = txBitWidth;
      #if F_CPU == 16000000L
        if (width == TIMER_TICKS_BIT_WIDTH_9600/4) // If 38400 baud the width is 6.5 ticks,
          if (txBit & 0x01)
            width++;  // so add a "leap" timer tick every other bit
      #endif
      while (uint8_t(TCNTX - t0) < width) {}  // delay 1 bit width
      t0 += width;          // advance start time
      b = txChar & 0x01;    // get next bit in the character to send
      txChar = txChar >> 1; // shift character to expose the following bit
    }

  }
  *txPort |= txBitMask;   // stop bit is high
  SREG = prevSREG;        // interrupts on for stop bit since it can be longer
  while (uint8_t(TCNTX - t0) < width) {}  // delay (at least) 1 bit width

  return 1;       // 1 character sent
}
