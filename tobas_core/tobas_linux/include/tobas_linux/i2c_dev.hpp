#pragma once

#include <cinttypes>

namespace linux
{
class I2Cdev
{
public:
  explicit I2Cdev(uint8_t dev_addr);
  ~I2Cdev();

  bool initialize(const char* i2c_dev);

  /** Read a single bit from an 8-bit device register.
   * @param reg_addr Register reg_addr to read from
   * @param bit_num Bit position to read (0-7)
   * @param data Container for single bit value
   * @return Status of read operation (true = success)
   */
  bool readBit(uint8_t reg_addr, uint8_t bit_num, uint8_t* data);

  /** Read a single bit from a 16-bit device register.
   * @param reg_addr Register reg_addr to read from
   * @param bit_num Bit position to read (0-15)
   * @param data Container for single bit value
   * @return Status of read operation (true = success)
   */
  bool readBitW(uint8_t reg_addr, uint8_t bit_num, uint16_t* data);

  /** Read multiple bits from an 8-bit device register.
   * @param reg_addr Register reg_addr to read from
   * @param bit_start First bit position to read (0-7)
   * @param length Number of bits to read (not more than 8)
   * @param data Container for right-aligned value (i.e. '101' read from any bit_start position will
   * equal 0x05)
   * @return Status of read operation (true = success)
   */
  bool readBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t* data);

  /** Read multiple bits from a 16-bit device register.
   * @param reg_addr Register reg_addr to read from
   * @param bit_start First bit position to read (0-15)
   * @param length Number of bits to read (not more than 16)
   * @param data Container for right-aligned value (i.e. '101' read from any bit_start position will
   * equal 0x05)
   * @return Status of read operation (1 = success, 0 = failure)
   */
  bool readBitsW(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t* data);

  /** Read single byte from an 8-bit device register.
   * @param reg_addr Register reg_addr to read from
   * @param data Container for byte value read from device
   * @return Status of read operation (true = success)
   */
  bool readByte(uint8_t reg_addr, uint8_t* data);

  /** Read single word from a 16-bit device register.
   * @param reg_addr Register reg_addr to read from
   * @param data Container for word value read from device
   * @return Status of read operation (true = success)
   */
  bool readWord(uint8_t reg_addr, uint16_t* data);

  /** Read multiple bytes from an 8-bit device register.
   * @param reg_addr First register reg_addr to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   * @return Status of read operation (true = success)
   */
  bool readBytes(uint8_t reg_addr, uint8_t length, uint8_t* data);

  /** Read multiple bytes from an 8-bit device register without sending the register address.
   * Required by MB85RC256(FRAM on Navio+)
   * @param reg_addr First register reg_addr to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   * @return Status of read operation (true = success)
   */
  bool readBytesNoRegAddress(uint8_t length, uint8_t* data);

  /** Read multiple words from a 16-bit device register.
   * @param reg_addr First register reg_addr to read from
   * @param length Number of words to read
   * @param data Buffer to store read data in
   * @return Status of read operation (true = success)
   */
  bool readWords(uint8_t reg_addr, uint8_t length, uint16_t* data);

  /** write a single bit in an 8-bit device register.
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to write (0-7)
   * @param value New bit value to write
   * @return Status of operation (true = success)
   */
  bool writeBit(uint8_t reg_addr, uint8_t bit_num, uint8_t data);

  /** write a single bit in a 16-bit device register.
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to write (0-15)
   * @param value New bit value to write
   * @return Status of operation (true = success)
   */
  bool writeBitW(uint8_t reg_addr, uint8_t bit_num, uint16_t data);

  /** Write multiple bits in an 8-bit device register.
   * @param reg_addr Register reg_addr to write to
   * @param bit_start First bit position to write (0-7)
   * @param length Number of bits to write (not more than 8)
   * @param data Right-aligned value to write
   * @return Status of operation (true = success)
   */
  bool writeBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data);

  /** Write multiple bits in a 16-bit device register.
   * @param reg_addr Register reg_addr to write to
   * @param bit_start First bit position to write (0-15)
   * @param length Number of bits to write (not more than 16)
   * @param data Right-aligned value to write
   * @return Status of operation (true = success)
   */
  bool writeBitsW(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data);

  /** Write single byte to an 8-bit device register.
   * @param reg_addr Register address to write to
   * @param data New byte value to write
   * @return Status of operation (true = success)
   */
  bool writeByte(uint8_t reg_addr, uint8_t data);

  /** Write single word to a 16-bit device register.
   * @param reg_addr Register address to write to
   * @param data New word value to write
   * @return Status of operation (true = success)
   */
  bool writeWord(uint8_t reg_addr, uint16_t data);

  /** Write multiple bytes to an 8-bit device register.
   * @param reg_addr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   * @return Status of operation (true = success)
   */
  bool writeBytes(uint8_t reg_addr, uint8_t length, uint8_t* data);

  /** Write multiple words to a 16-bit device register.
   * @param reg_addr First register address to write to
   * @param length Number of words to write
   * @param data Buffer to copy new data from
   * @return Status of operation (true = success)
   */
  bool writeWords(uint8_t reg_addr, uint8_t length, uint16_t* data);

private:
  const uint8_t dev_addr_;
  int i2c_fd_;

  uint8_t buf_[1 << 7];
};
}  // namespace linux
