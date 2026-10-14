// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <cstdint>

namespace tobas::linux
{
class I2Cdev
{
  static constexpr size_t kBufSize = 64;

public:
  explicit I2Cdev();
  ~I2Cdev();

  bool initialize(const char* _i2c_dev, uint8_t _dev_addr);

  /**
   * Read a single bit from an 8-bit device register.
   *
   * @param _reg_addr Register address to read from
   * @param _bit_pos Bit position to read (0-7)
   * @param _value Output value
   * @return Status of operation (true = success)
   */
  bool readBit(uint8_t _reg_addr, uint8_t _bit_pos, bool& _value);

  /**
   * Read a single byte from an 8-bit device register.
   *
   * @param _reg_addr Register address to read from
   * @param _value Output value
   * @return Status of operation (true = success)
   */
  bool readByte(uint8_t _reg_addr, uint8_t& _value);

  /**
   * Read multiple bytes from an 8-bit device register.
   *
   * @param _reg_addr First register address to read from
   * @param _length Number of bytes to read
   * @param _rx Output buffer
   * @return Status of operation (true = success)
   *
   * @note In order to continuously obtain multiple bytes, it may be necessary to set the MSB of the register address.
   */
  bool readBytes(uint8_t _reg_addr, size_t _length, void* _rx);

  /**
   * Read multiple bytes from an 8-bit device register without sending the register address.
   *
   * @param _length Number of bytes to read
   * @param _rx Output buffer
   * @return Status of operation (true = success)
   */
  bool readBytesNoRegAddress(size_t _length, void* _rx);

  /**
   * Write a single bit to an 8-bit device register.
   *
   * @param _reg_addr Register address to write to
   * @param _bit_pos Bit position to write (0-7)
   * @param _value New value to write
   * @param _verify Whether to do verification
   * @return Status of operation (true = success)
   */
  bool writeBit(uint8_t _reg_addr, uint8_t _bit_pos, bool _value, bool _verify = false);

  /**
   * Write a single byte to an 8-bit device register.
   *
   * @param _reg_addr Register address to write to
   * @param _value New value to write
   * @param _verify Whether to do verification
   * @return Status of operation (true = success)
   */
  bool writeByte(uint8_t _reg_addr, uint8_t _value, bool _verify = false);

  /**
   * Write multiple bytes to an 8-bit device register.
   *
   * @param _reg_addr First register address to write to
   * @param _length Number of bytes to write
   * @param _tx Pointer to the new values to write
   * @param _verify Whether to do verification
   * @return Status of operation (true = success)
   */
  bool writeBytes(uint8_t _reg_addr, size_t _length, const void* _tx, bool _verify = false);

private:
  uint8_t dev_addr_;
  int i2c_fd_ = -1;
  uint8_t tx_[kBufSize + 1] = {};  ///< Register Address + TX data
  uint8_t rx_[kBufSize] = {};      ///< Temporary RX data.

  bool checkDataLength(size_t _length) const;
  bool selectDevice() const;
  bool write(uint8_t _reg_addr, size_t _length, const void* _tx);
  bool read(size_t _length, void* _rx);
};
}  // namespace tobas::linux
