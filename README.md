The **NeoSWSerial** class is intended as an more-efficient drop-in replacement for the Arduino built-in class `SoftwareSerial`.  If you could use `Serial`, `Serial1`, `Serial2` or `Serial3`, you should use [NeoHWSerial](https://github.com/SlashDevin/NeoHWSerial) instead.  If you could use an Input Capture pin (ICP1, pins 8 & 9 on an UNO), you should consider  [NeoICSerial](https://github.com/SlashDevin/NeoICSerial) instead.

**NeoSWSerial** is limited to three baud rates: 9600 (default), 19200 and 38400.

There are four, nay, **five** advantages over `SoftwareSerial`:

**1)** It uses *much* less CPU time.  

**2)** Simultaneous transmit and receive is fully supported.

**3)** Interrupts are not disabled for the entire RX character time.  (They are disabled for most of each TX character time.)

**4)** It is much more reliable (far fewer receive data errors).

**5)** Characters can be handled with a user-defined procedure at interrupt time.  This should prevent most input buffer overflow problems.  Simply register your procedure with the 'NeoSWSerial' instance:

```
    #include <NeoSWSerial.h>
    NeoSWSerial ss( 4, 3 );
    
    volatile uint32_t newlines = 0UL;
    
    static void handleRxChar( uint8_t c )
    {
      if (c == '\n')
        newlines++;
    }
    
    void setup()
    {
      ss.attachInterrupt( handleRxChar );
      ss.begin( 9600 );
    }
```

Remember that the registered procedure is called from an interrupt context, and it should return as quickly as possible.  Taking too much time in the procedure will cause many unpredictable behaviors, including loss of received data.  See the similar warnings for the built-in [`attachInterrupt`](https://www.arduino.cc/en/Reference/AttachInterrupt) for digital pins.

The registered procedure will be called from the ISR whenever a character is received.  The received character **will not** be stored in the `rx_buffer`, and it **will not** be returned from `read()`.  Any characters that were received and buffered before `attachInterrupt` was called remain in `rx_buffer`, and could be retrieved by calling `read()`.

If `attachInterrupt` is never called, or it is passed a `NULL` procedure, the normal buffering occurs, and all received characters must be obtained by calling `read()`.

This class is nearly identical to the built-in `SoftwareSerial`, except for two new methods, `attachInterrupt` and `detachInterrupt`:

```
    typedef void (* isr_t)( uint8_t );
    void attachInterrupt( isr_t fn );
    void detachInterrupt() { attachInterrupt( (isr_t) NULL ); };

  private:
    isr_t  _isr;
```

This class supports the following MCUs: ATtinyx61, ATtinyx4, ATtinyx5, ATmega328P (Pro, UNO, Nano), ATmega32U4 (Micro, Leonardo), ATmega2560 (Mega), ATmega2560RFR2, ATmega1284P and ATmega1286
