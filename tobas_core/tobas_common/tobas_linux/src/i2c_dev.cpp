// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/i2c_dev.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <thread>

#include <tobas_std_tools/error.hpp>

using namespace std::chrono_literals;

namespace tobas::linux
{
I2Cdev::I2Cdev()
{
}

I2Cdev::~I2Cdev()
{
  if (i2c_fd_ >= 0) {
    close(i2c_fd_);
  }
}

bool I2Cdev::initialize(const char* _i2c_dev, uint8_t _dev_addr)
{
  if (_dev_addr >= 0x80) {
    std::cerr << "Invalid slave address. It must be 7-bit.\n";
    return false;
  }

  i2c_fd_ = open(_i2c_dev, O_RDWR);
  if (i2c_fd_ < 0) {
    std::cerr << "Failed to open I2C device: " << _i2c_dev << '\n';
    return false;
  }

  dev_addr_ = _dev_addr;

  // Wait to avoid remote I/O error 121.
  std::this_thread::sleep_for(10ms);

  return true;
}

bool I2Cdev::readBit(uint8_t _reg_addr, uint8_t _bit_pos, bool& _value)
{
  if (_bit_pos >= 8) {
    std::cerr << "Bit position must be 0-7.\n";
    return false;
  }

  uint8_t byte;
  if (!readByte(_reg_addr, byte)) {
    return false;
  }

  _value = byte & (1 << _bit_pos);
  return true;
}

bool I2Cdev::readByte(uint8_t _reg_addr, uint8_t& _value)
{
  return readBytes(_reg_addr, 1, &_value);
}

bool I2Cdev::readBytes(uint8_t _reg_addr, size_t _length, void* _rx)
{
  if (!checkDataLength(_length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!write(_reg_addr, 0, nullptr)) {
    return false;
  }

  if (!read(_length, _rx)) {
    return false;
  }

  return true;
}

bool I2Cdev::readBytesNoRegAddress(size_t _length, void* _rx)
{
  if (!checkDataLength(_length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!read(_length, _rx)) {
    return false;
  }

  return true;
}

bool I2Cdev::writeBit(uint8_t _reg_addr, uint8_t _bit_pos, bool _value, bool _verify)
{
  if (_bit_pos >= 8) {
    std::cerr << "Bit position must be 0-7.\n";
    return false;
  }

  uint8_t byte;
  if (!readByte(_reg_addr, byte)) {
    return false;
  }

  if (_value) {
    byte |= (1 << _bit_pos);
  }
  else {
    byte &= ~(1 << _bit_pos);
  }

  return writeByte(_reg_addr, byte, _verify);
}

bool I2Cdev::writeByte(uint8_t _reg_addr, uint8_t _value, bool _verify)
{
  return writeBytes(_reg_addr, 1, &_value, _verify);
}

bool I2Cdev::writeBytes(uint8_t _reg_addr, size_t _length, const void* _tx, bool _verify)
{
  if (!checkDataLength(_length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!write(_reg_addr, _length, _tx)) {
    return false;
  }

  if (_verify) {
    if (!readBytes(_reg_addr, _length, rx_)) {
      return false;
    }

    if (std::memcmp(_tx, rx_, _length) != 0) {
      std::cerr << "The " << _length << " bytes written over I2C are not taking effect in the slave’s registers.\n";
      return false;
    }
  }

  return true;
}

bool I2Cdev::checkDataLength(size_t _length) const
{
  if (_length > kBufSize) {
    std::cerr << "Data length cannot be greater than buffer size.\n";
    return false;
  }

  return true;
}

bool I2Cdev::selectDevice() const
{
  // Register the slave address.
  // The I2C standard allows sending a command after the slave address,
  // but the Linux I2C driver only checks for an acknowledgment first.
  if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr_) < 0) {
    std::cerr << "Failed to select I2C device.\n";
    return false;
  }

  return true;
}

bool I2Cdev::write(uint8_t _reg_addr, size_t _length, const void* _tx)
{
  tx_[0] = _reg_addr;
  std::memcpy(tx_ + 1, _tx, _length);

  // 1. Start Condition (Master -> Slave)
  // 2. Slave Address (Master -> Slave)
  // 3. Write Flag (Master -> Slave)
  // 4. ACK (Slave -> Master)
  // 5. Register Address (Master -> Slave)
  // 6. ACK (Slave -> Master)
  // 7. Stop Condition (Master -> Slave)
  const auto req_length = _length + 1;
  const auto res = ::write(i2c_fd_, tx_, req_length);
  if (res < 0) {
    std::cerr << "I2C write failed: " << st::strError() << '\n';
    switch (errno) {
      case EREMOTEIO:
        std::cerr << "Please ensure that the correct 7-bit slave address is set.\n";
        break;
    }
    return false;
  }
  if (res != static_cast<ssize_t>(req_length)) {
    std::cerr << "Tried to write " << req_length << " bytes on " << _reg_addr << ", but " << res
              << " bytes were written.\n";
    return false;
  }

  return true;
}

bool I2Cdev::read(size_t _length, void* _rx)
{
  // 1. Start Condition (Master -> Slave)
  // 2. Slave Address (Master -> Slave)
  // 3. Read Flag (Master -> Slave)
  // 4. ACK (Slave -> Master)
  // 5. For (length - 1):
  //     a. Register Value (Slave -> Master)
  //     b. ACK (Master -> Slave)
  // 6. Register Value (Slave -> Master)
  // 7. NAK (Master -> Slave)
  // 8. Stop Condition (Master -> Slave)
  const auto res = ::read(i2c_fd_, _rx, _length);
  if (res < 0) {
    std::cerr << "I2C read failed: " << st::strError() << '\n';
    return false;
  }
  if (res != static_cast<ssize_t>(_length)) {
    std::cerr << "Tried to read " << _length << " bytes, but " << res << " bytes were read.\n";
    return false;
  }

  return true;
}
}  // namespace tobas::linux
