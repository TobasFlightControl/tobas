#pragma once

#include <cinttypes>

#define RASPBERRY_PI_I2C "/dev/i2c-1"
#define BANANA_PI_I2C "/dev/i2c-2"

#define I2CDEV RASPBERRY_PI_I2C

#ifndef TRUE
#define TRUE (1 == 1)
#define FALSE (0 == 1)
#endif

class I2Cdev
{
public:
  explicit I2Cdev();

  /** Read a single bit from an 8-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param bitNum Bit position to read (0-7)
   * @param data Container for single bit value
   * @return Status of read operation (true = success)
   */
  static int8_t readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data);

  /** Read a single bit from a 16-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param bitNum Bit position to read (0-15)
   * @param data Container for single bit value
   * @return Status of read operation (true = success)
   */
  static int8_t readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t* data);

  /** Read multiple bits from an 8-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param bitStart First bit position to read (0-7)
   * @param length Number of bits to read (not more than 8)
   * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will
   * equal 0x05)
   * @return Status of read operation (true = success)
   */
  static int8_t
  readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data);

  /** Read multiple bits from a 16-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param bitStart First bit position to read (0-15)
   * @param length Number of bits to read (not more than 16)
   * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will
   * equal 0x05)
   * @return Status of read operation (1 = success, 0 = failure)
   */
  static int8_t
  readBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t* data);

  /** Read single byte from an 8-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param data Container for byte value read from device
   * @return Status of read operation (true = success)
   */
  static int8_t readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data);

  /** Read single word from a 16-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to read from
   * @param data Container for word value read from device
   * @return Status of read operation (true = success)
   */
  static int8_t readWord(uint8_t devAddr, uint8_t regAddr, uint16_t* data);

  /** Read multiple bytes from an 8-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr First register regAddr to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   * @return Number of bytes read (-1 indicates failure)
   */
  static int8_t readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);

  /** Read multiple bytes from an 8-bit device register without sending the register address.
   * Required by MB85RC256(FRAM on Navio+)
   * @param devAddr I2C slave device address
   * @param regAddr First register regAddr to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   * @return Number of bytes read (-1 indicates failure)
   */
  static int8_t readBytesNoRegAddress(uint8_t devAddr, uint8_t length, uint8_t* data);

  /** Read multiple words from a 16-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr First register regAddr to read from
   * @param length Number of words to read
   * @param data Buffer to store read data in
   * @return Number of words read (0 indicates failure)
   */
  static int8_t readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data);

  /** write a single bit in an 8-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to write to
   * @param bitNum Bit position to write (0-7)
   * @param value New bit value to write
   * @return Status of operation (true = success)
   */
  static bool writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);

  /** write a single bit in a 16-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to write to
   * @param bitNum Bit position to write (0-15)
   * @param value New bit value to write
   * @return Status of operation (true = success)
   */
  static bool writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data);

  /** Write multiple bits in an 8-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to write to
   * @param bitStart First bit position to write (0-7)
   * @param length Number of bits to write (not more than 8)
   * @param data Right-aligned value to write
   * @return Status of operation (true = success)
   */
  static bool
  writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);

  /** Write multiple bits in a 16-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register regAddr to write to
   * @param bitStart First bit position to write (0-15)
   * @param length Number of bits to write (not more than 16)
   * @param data Right-aligned value to write
   * @return Status of operation (true = success)
   */
  static bool
  writeBitsW(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data);

  /** Write single byte to an 8-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register address to write to
   * @param data New byte value to write
   * @return Status of operation (true = success)
   */
  static bool writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);

  /** Write single word to a 16-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr Register address to write to
   * @param data New word value to write
   * @return Status of operation (true = success)
   */
  static bool writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);

  /** Write multiple bytes to an 8-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   * @return Status of operation (true = success)
   */
  static bool writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data);

  /** Write multiple words to a 16-bit device register.
   * @param devAddr I2C slave device address
   * @param regAddr First register address to write to
   * @param length Number of words to write
   * @param data Buffer to copy new data from
   * @return Status of operation (true = success)
   */
  static bool writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data);
};
