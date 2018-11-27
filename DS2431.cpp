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

#include "DS2431.h"

DS2431::DS2431(OneWire &ow)
: _ow(ow)
{
  _skiprom = true;
}

void DS2431::begin(uint8_t serialNumber[ONE_WIRE_MAC_SIZE])
{
  memcpy(_serialNumber, serialNumber, ONE_WIRE_MAC_SIZE);
  _skiprom = false;
}

uint8_t DS2431::read(uint16_t address)
{
  uint8_t res = 0xFF;
  read(address, &res, 1);

  return res;
}

void DS2431::read(uint16_t address, uint8_t *buf, uint16_t len)
{
  _startTransmission();

  _ow.write(READ_MEMORY, 1);
  _ow.write(lowByte(address), 1);
  _ow.write(highByte(address), 1);

  for (int i = 0; i < len; i++)
  buf[i] = _ow.read();

  _ow.depower();
}

bool DS2431::write(uint16_t address, const uint8_t *buf, uint16_t count, bool verify /* = 0 */)
{
  bool ret = _write(address, buf, count, verify);
  _ow.depower();
  return ret;
}

bool DS2431::_write(uint16_t address, const uint8_t *buf, uint16_t count, bool verify)
{
  uint8_t error_count = 0;
  uint8_t buffer[DS2431_BUFFER_SIZE];
  uint8_t crc16[DS2431_CRC_SIZE];

  //Address has to be aligned on an 8-byte boundary
  if (address >= DS2431_EEPROM_SIZE || address % 8 != 0)
    return false;

  // Prepare buffer data
  buffer[0] = WRITE_SCRATCHPAD;
  buffer[1] = lowByte(address);
  buffer[2] = highByte(address);
  memcpy(&buffer[DS2431_CMD_SIZE], buf, count);

  //Write scratchpad with CRC check
  _startTransmission();
  _ow.write_bytes(buffer, DS2431_CMD_SIZE + count, 1); // Write CMD + LSB Adr + MSB Adr
  _ow.read_bytes(crc16, DS2431_CRC_SIZE); //Read CRC-16

  if (!_ow.check_crc16(buffer, DS2431_CMD_SIZE + count, crc16)){
    verify = true; //CRC not matching, try to read again
  }

  // Read verification
  // Prepare buffer data
  buffer[0] = READ_SCRATCHPAD;
  do {
    //Read scratchpad to compare with the data sent
    _startTransmission();
    _ow.write(buffer[0], 1); // Write CMD
    _ow.read_bytes(&buffer[1], DS2431_CMD_SIZE); //Read TA1, TA2, E/S, scratchpad

    if (buffer[3] != DS2431_PF_MASK) {
      verify = true;
    }

    if(verify) {
      _ow.read_bytes(&buffer[4], count); //Read scratchpad
      _ow.read_bytes(crc16, DS2431_CRC_SIZE); //Read CRC-16

      if (!_ow.check_crc16(buffer, 12, crc16)) {
        error_count++; //CRC not matching.
        continue;
      }

      if (address != ((buffer[2] << 8) + buffer[1])) {
        return false; //Address not matching
      }

      if (buffer[3] != DS2431_PF_MASK) {
        return false; //Invalid transfer or data already copied (wrong value for E/S).
      }

      if (memcmp(&buffer[4], buf, 8) != 0) {
        return false; //Data in the scratchpad is invalid.
      }
    }

    break;
  } while(error_count < DS2431_READ_RETRY);

  // Prepare buffer data
  buffer[0] = COPY_SCRATCHPAD;

  //Copy scratchpad
  _startTransmission();
  _ow.write_bytes(buffer, DS2431_CMD_SIZE + 1, 1); //Send authorization code (TA1, TA2, E/S)
  delay(15); // t_PROG = 12.5ms worst case.

  uint8_t res = _ow.read();

  if (res != DS2431_WRITE_MASK) {
    return false;
  }

  return true;
}
