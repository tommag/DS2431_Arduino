/*
MIT License

Copyright (c) 2017 Tom Magnier
Modified 2018 by Nicolò Veronese

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
 */

#ifndef _DS2431_H
#define _DS2431_H

#include <OneWire.h>

class DS2431 {
public:
  const static uint8_t ONE_WIRE_MAC_SIZE = 8;

  const static uint8_t ONE_WIRE_FAMILY_CODE = 0x2D;

  const static uint8_t DS2431_EEPROM_SIZE = 128;
  const static uint8_t DS2431_ROW_SIZE = 8;

  DS2431(OneWire &ow); // OneWire class

  void begin(uint8_t serialNumber[ONE_WIRE_MAC_SIZE]); // family code, 48bit serial number and CRC as returned by OneWire search function

  /* Single byte read
  */
  uint8_t read(uint16_t address);

  /* Multiple byte read.
  */
  void read(uint16_t address, uint8_t *buf, uint16_t len);

  /* Multiple byte write.
    Please note : address must be a multiple of 8. Write up to 8 bytes
    Return true if operation was successful.
    The OneWire bus should be de-powered after calling this function.
  */
  bool write(uint16_t address, const uint8_t *buf, uint16_t count, bool verify = false);

private:
  const static uint8_t DS2431_PF_MASK = 0x07;
  const static uint8_t DS2431_WRITE_MASK = 0xAA;

  const static uint8_t DS2431_CMD_SIZE = 3;
  const static uint8_t DS2431_CRC_SIZE = 2;

  const static uint8_t DS2431_READ_RETRY = 2;

  const static uint16_t DS2431_BUFFER_SIZE = DS2431_ROW_SIZE + DS2431_CMD_SIZE + DS2431_CRC_SIZE;

  OneWire &_ow;
  uint8_t _serialNumber[ONE_WIRE_MAC_SIZE];
  bool _skiprom;

  enum Commands {
    WRITE_SCRATCHPAD = 0x0F,
    READ_SCRATCHPAD = 0xAA,
    COPY_SCRATCHPAD = 0x55,
    READ_MEMORY = 0xF0
  };

  bool _write(uint16_t address, const uint8_t *buf, uint16_t count, bool verify);

  inline void _startTransmission()
  {
    _ow.reset();
    if (_skiprom)
    _ow.skip();
    else
    _ow.select(_serialNumber);
  }
};

#endif // _DS2431_H
