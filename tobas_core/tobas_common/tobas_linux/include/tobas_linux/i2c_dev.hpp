#pragma once

#include <cstddef>
#include <cstdint>

namespace linux
{
class I2Cdev
{
  static constexpr size_t kBufSize = 64;

public:
  explicit I2Cdev();
  ~I2Cdev();

  bool initialize(const char* i2c_dev, uint8_t dev_addr);

  /**
   * @brief Read a single bit from an 8-bit device register.
   *
   * @param reg_addr Register address to read from
   * @param bit_pos Bit position to read (0-7)
   * @param value Output value
   *
   * @return Status of operation (true = success)
   */
  bool readBit(uint8_t reg_addr, uint8_t bit_pos, bool& value);

  /**
   * @brief Read a single byte from an 8-bit device register.
   *
   * @param reg_addr Register address to read from
   * @param value Output value
   *
   * @return Status of operation (true = success)
   */
  bool readByte(uint8_t reg_addr, uint8_t& value);

  /**
   * @brief Read multiple bytes from an 8-bit device register.
   *
   * @param reg_addr First register address to read from
   * @param length Number of bytes to read
   * @param rx Output buffer
   *
   * @return Status of operation (true = success)
   *
   * @note In order to continuously obtain multiple bytes, it may be necessary to set the MSB of the register address.
   */
  bool readBytes(uint8_t reg_addr, size_t length, void* rx);

  /**
   * @brief Read multiple bytes from an 8-bit device register without sending the register address.
   *
   * @param length Number of bytes to read
   * @param rx Output buffer
   *
   * @return Status of operation (true = success)
   */
  bool readBytesNoRegAddress(size_t length, void* rx);

  /**
   * @brief write a single bit in an 8-bit device register.
   *
   * @param reg_addr Register address to write to
   * @param bit_pos Bit position to write (0-7)
   * @param value New value to write
   * @param verify Whether to do verification
   *
   * @return Status of operation (true = success)
   */
  bool writeBit(uint8_t reg_addr, uint8_t bit_pos, bool value, bool verify = false);

  /**
   * @brief Write a single byte to an 8-bit device register.
   *
   * @param reg_addr Register address to write to
   * @param value New value to write
   * @param verify Whether to do verification
   *
   * @return Status of operation (true = success)
   */
  bool writeByte(uint8_t reg_addr, uint8_t value, bool verify = false);

  /**
   * @brief Write multiple bytes to an 8-bit device register.
   *
   * @param reg_addr First register address to write to
   * @param length Number of bytes to write
   * @param tx Pointer to the new values to write
   * @param verify Whether to do verification
   *
   * @return Status of operation (true = success)
   */
  bool writeBytes(uint8_t reg_addr, size_t length, const void* tx, bool verify = false);

private:
  uint8_t dev_addr_;
  int i2c_fd_ = -1;
  uint8_t tx_[kBufSize + 1] = { 0 };  // Register Address + TX data
  uint8_t rx_[kBufSize] = { 0 };      // Temporal RX data

  bool checkDataLength(size_t length) const;
  bool selectDevice() const;
  bool write(uint8_t reg_addr, size_t length, const void* tx);
  bool read(size_t length, void* rx);
};
}  // namespace linux
