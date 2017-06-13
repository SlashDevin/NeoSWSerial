#ifndef NeoSWSerial_h
#define NeoSWSerial_h

#include "Arduino.h"

//---------------------------------------------------------------------------
//
// NeoSWSerial
// Copyright (C) 2015-2016, SlashDevin
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
//
// This software serial library is intended as a more-efficient replacement
// for SoftwareSerial at baud rates 9600, 19200 and 38400.
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
// By default NeoSWSerial defines handlers for all PCINT interrupts like
// SoftwareSerial. If client code requires own pin change interrupt handlers,
// it's possible to rebuild library with #define NEOSWSERIAL_EXTERNAL_PCINT.
// In such case client code should call NeoSWSerial::rxISR(PINB) (assuming
// that receivePin is on PORT B)
//
// Supported baud rates are 9600 (default), 19200 and 38400.
// The baud rate is selectable at run time.
//
// The size of the RX buffer may be changed by editing the 
// accompanying .cpp file. For optimal performance of the interrupt 
// service routines, the buffer size should be chosen to be a 
// power of 2 (i.e., 2, 4, 8, 16, 32, 64,...).
//
// v1.0   Nov 2014 jboyton   - Created
// v1.1   Jun 2015 jboyton   - Added support for 8 MHz system clock. Timer 2 had to
//                             be used since the timer 0 prescaler was inadequate
//                             for this. The supported baud rates for 8 MHz are 9600
//                             and 19200.
// v2.0   Nov 2015 SlashDev  - Add support for other boards,
//                             add end() and attach/detachInterrupt
// v2.1   Jun 2016 SlashDev  - Add support for all character values
// v2.2   Mar 2017 Dionorgua - Add option to disable pre-defined PCINT ISRs.
// v2.3   Mar 2017 SlashDev  - Add GPL
// v3.0.0 May 2017 SlashDev  - Convert to new Arduino IDE library

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

          void   begin(uint16_t baudRate=9600);   // initialize, set baudrate, listen
          void   listen();                        // enable RX interrupts
          void   ignore();                        // disable RX interrupts
          void   setBaudRate(uint16_t baudRate);  // 9600 [default], 19200, 38400
  virtual int    available();
  virtual int    read();
  virtual size_t write(uint8_t txChar);
  using Stream::write; // make the base class overloads visible
  virtual int    peek() { return 0; };
  virtual void   flush() {};
          void   end() { ignore(); }

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

  //#define NEOSWSERIAL_EXTERNAL_PCINT // uncomment to use your own PCINT ISRs
};
#endif
