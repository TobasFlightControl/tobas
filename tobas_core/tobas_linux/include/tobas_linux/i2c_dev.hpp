#pragma once

#include <cinttypes>

namespace linux
{
class I2Cdev
{
public:
  uint8_t* tx;
  uint8_t* rx;

  explicit I2Cdev();
  ~I2Cdev();

  bool initialize(const char* i2c_dev, uint8_t dev_addr, size_t buf_size);

  /** Read a single bit from an 8-bit device register.
   * @param reg_addr Register reg_addr to read from
   * @param bit_num Bit position to read (0-7)
   * @param data Container for single bit value
   * @return Status of read operation (true = success)
   */
  bool readBit(uint8_t reg_addr, uint8_t bit_num, bool& flag);

  /** Read multiple bytes from an 8-bit device register.
   * @param reg_addr First register reg_addr to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   * @return Status of read operation (true = success)
   */
  bool readBytes(uint8_t reg_addr, size_t length);

  /** Read multiple bytes from an 8-bit device register without sending the register address.
   * Required by MB85RC256(FRAM on Navio+)
   * @param reg_addr First register reg_addr to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   * @return Status of read operation (true = success)
   */
  bool readBytesNoRegAddress(size_t length);

  /** write a single bit in an 8-bit device register.
   * @param reg_addr Register reg_addr to write to
   * @param bit_num Bit position to write (0-7)
   * @param value New bit value to write
   * @return Status of operation (true = success)
   */
  bool writeBit(uint8_t reg_addr, uint8_t bit_num, bool flag);

  /** Write multiple bytes to an 8-bit device register.
   * @param reg_addr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   * @return Status of operation (true = success)
   */
  bool writeBytes(uint8_t reg_addr, size_t length);

private:
  uint8_t dev_addr_;
  int i2c_fd_;
  size_t buf_size_ = 0;
  uint8_t* buf_;  // Register Address + TX data

  bool selectDevice();
};
}  // namespace linux
