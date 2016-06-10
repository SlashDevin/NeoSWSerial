#ifndef NeoSWSerial_h
#define NeoSWSerial_h

#include "Arduino.h"

//---------------------------------------------------------------------------
//
// NeoSWSerial
//
//
// A software serial library, intended as a more-efficient replacement
// for SoftwareSerial at baud rates 9600, 19200 and 38400.
//
// Constructor:  NeoSWSerial ss(RX_PIN, TX_PIN);
//
// Any of the pins supported by SoftwareSerial may be used.  Pins (0-19) 
// on the Uno may be used.  Other boards can use any of the pins 
// allowed by digitalPinToPCMSK in pins_arduino.h
//
// This code uses a pin change interrupt on the selected RX pin.
// Transmission on the TX line is done in a loop with interrupts disabled.
// Both RX and TX read timer0 for determining elapsed time. Timer0 itself is
// not reprogrammed; it is assumed to be running with a 4 microsecond step.
//
// Supported baud rates are 9600 (default), 19200 and 38400.
// The baud rate is selectable at run time.
//
// The size of the RX buffer may be changed by editing the 
// accompanying .cpp file. For optimal performance of the interrupt 
// service routines, the buffer size should be chosen to be a 
// power of 2 (i.e., 2, 4, 8, 16, 32, 64,...).
//
// Nov/Dec 2014 jboyton - Created
// Jun     2015 jboyton - Added support for 8 MHz system clock. Timer 2 had to
//                        be used since the timer 0 prescaler was inadequate
//                        for this. The supported baud rates for 8 MHz are 9600
//                        and 19200.
// Nov     2015 SlashDev - Add support for other boards,
//                         add end() and attach/detachInterrupt
// Jun     2016 SlashDev - Add support for all character values
//

class NeoSWSerial : public Stream
{
  NeoSWSerial( const NeoSWSerial & ); // Not allowed
  NeoSWSerial & operator =( const NeoSWSerial & ); // Not allowed

public:
  NeoSWSerial(uint8_t receivePin, uint8_t transmitPin)
    {
      rxPin = receivePin;
      txPin = transmitPin;
      _isr  = (isr_t) NULL;
    }

  void begin(uint16_t baudRate=9600);   // initialize, set baudrate, listen
  void listen();                        // enable RX interrupts
  void ignore();                        // disable RX interrupts
  void setBaudRate(uint16_t baudRate);  // set baud rate (9600 [default], 19200, 38400)
  int available();               // returns number of characters in buffer
  int read();                    // get one character from buffer
  size_t write(uint8_t txChar);   // transmit a character
  virtual int peek() { return 0; };
  virtual void flush() {};
  void end() { ignore(); }

  typedef void (* isr_t)( uint8_t );
  void attachInterrupt( isr_t fn );
  void detachInterrupt() { attachInterrupt( (isr_t) NULL ); };

private:
           uint8_t  rxPin, txPin;
  volatile uint8_t *rxPort;

  uint16_t _baudRate;
  isr_t    _isr;

  static void rxChar( uint8_t rx ); // buffer or dispatch one received character

  bool checkRxTime();

  static void startChar();

public:
  // visible only so the ISRs can call it...
  static void rxISR( uint8_t port_input_register );
};
#endif
