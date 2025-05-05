#pragma once

#include <cstddef>
#include <cstdint>

namespace linux
{
class I2Cdev
{
  static constexpr size_t kBufSize = 256;

public:
  alignas(kBufSize) uint8_t tx[kBufSize] = { 0 };
  alignas(kBufSize) uint8_t rx[kBufSize] = { 0 };

  explicit I2Cdev();
  ~I2Cdev();

  bool initialize(const char* i2c_dev, uint8_t dev_addr);

  /**
   * @brief Read a single bit from an 8-bit device register.
   *
   * @param reg_addr Register address to read from
   * @param bit_num Bit position to read (0-7)
   * @param data Container for single bit value

   * @return Status of operation (true = success)
   */
  bool readBit(uint8_t reg_addr, uint8_t bit_num, bool& flag);

  /**
   * @brief Read multiple bytes from an 8-bit device register.
   *
   * @param reg_addr First register address to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   *
   * @return Status of operation (true = success)
   *
   * @note In order to continuously obtain multiple bytes, it may be necessary to set the MSB of the register address.
   */
  bool readBytes(uint8_t reg_addr, size_t length);

  /**
   * @brief Read multiple bytes from an 8-bit device register without sending the register address.
   *
   * @param reg_addr First register address to read from
   * @param length Number of bytes to read
   * @param data Buffer to store read data in
   *
   * @return Status of operation (true = success)
   */
  bool readBytesNoRegAddress(size_t length);

  /**
   * @brief write a single bit in an 8-bit device register.
   *
   * @param reg_addr Register address to write to
   * @param bit_num Bit position to write (0-7)
   * @param value New bit value to write
   *
   * @return Status of operation (true = success)
   */
  bool writeBit(uint8_t reg_addr, uint8_t bit_num, bool flag);

  /**
   * @brief Write multiple bytes to an 8-bit device register.
   *
   * @param reg_addr First register address to write to
   * @param length Number of bytes to write
   * @param data Buffer to copy new data from
   *
   * @return Status of operation (true = success)
   */
  bool writeBytes(uint8_t reg_addr, size_t length);

private:
  uint8_t dev_addr_;
  int i2c_fd_ = -1;
  uint8_t tx_[kBufSize + 1] = { 0 };  // Register Address + TX data

  bool checkDataLength(size_t length) const;
  bool selectDevice() const;
  bool write(uint8_t reg_addr, size_t length);
  bool read(size_t length);
};
}  // namespace linux
