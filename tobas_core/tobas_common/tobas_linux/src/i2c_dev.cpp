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

#include "tobas_linux/error.hpp"

using namespace std::chrono_literals;

namespace tobas
{
namespace linux
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

bool I2Cdev::initialize(const char* i2c_dev, uint8_t dev_addr)
{
  if (dev_addr >= 0x80) {
    std::cerr << "Invalid slave address. It must be 7-bit." << std::endl;
    return false;
  }

  i2c_fd_ = open(i2c_dev, O_RDWR);
  if (i2c_fd_ < 0) {
    std::cerr << "Failed to open I2C device: " << i2c_dev << std::endl;
    return false;
  }

  dev_addr_ = dev_addr;

  // Wait here to avoid 121 remote I/O error
  std::this_thread::sleep_for(10ms);

  return true;
}

bool I2Cdev::readBit(uint8_t reg_addr, uint8_t bit_pos, bool& value)
{
  if (bit_pos >= 8) {
    std::cerr << "Bit position must be 0-7." << std::endl;
    return false;
  }

  uint8_t byte;
  if (!readByte(reg_addr, byte)) {
    return false;
  }

  value = byte & (1 << bit_pos);
  return true;
}

bool I2Cdev::readByte(uint8_t reg_addr, uint8_t& value)
{
  return readBytes(reg_addr, 1, &value);
}

bool I2Cdev::readBytes(uint8_t reg_addr, size_t length, void* rx)
{
  if (!checkDataLength(length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!write(reg_addr, 0, nullptr)) {
    return false;
  }

  if (!read(length, rx)) {
    return false;
  }

  return true;
}

bool I2Cdev::readBytesNoRegAddress(size_t length, void* rx)
{
  if (!checkDataLength(length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!read(length, rx)) {
    return false;
  }

  return true;
}

bool I2Cdev::writeBit(uint8_t reg_addr, uint8_t bit_pos, bool value, bool verify)
{
  if (bit_pos >= 8) {
    std::cerr << "Bit position must be 0-7." << std::endl;
    return false;
  }

  uint8_t byte;
  if (!readByte(reg_addr, byte)) {
    return false;
  }

  if (value) {
    byte |= (1 << bit_pos);
  }
  else {
    byte &= ~(1 << bit_pos);
  }

  return writeByte(reg_addr, byte, verify);
}

bool I2Cdev::writeByte(uint8_t reg_addr, uint8_t value, bool verify)
{
  return writeBytes(reg_addr, 1, &value, verify);
}

bool I2Cdev::writeBytes(uint8_t reg_addr, size_t length, const void* tx, bool verify)
{
  if (!checkDataLength(length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!write(reg_addr, length, tx)) {
    return false;
  }

  if (verify) {
    if (!readBytes(reg_addr, length, rx_)) {
      return false;
    }

    if (std::memcmp(tx, rx_, length) != 0) {
      std::cerr << "The " << length << " bytes written over I2C are not taking effect in the slave’s registers."
                << std::endl;
      return false;
    }
  }

  return true;
}

bool I2Cdev::checkDataLength(size_t length) const
{
  if (length > kBufSize) {
    std::cerr << "Data length cannot be greater than buffer size." << std::endl;
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
    std::cerr << "Failed to select I2C device." << std::endl;
    return false;
  }

  return true;
}

bool I2Cdev::write(uint8_t reg_addr, size_t length, const void* tx)
{
  tx_[0] = reg_addr;
  std::memcpy(tx_ + 1, tx, length);

  // 1. Start Condition (Master -> Slave)
  // 2. Slave Address (Master -> Slave)
  // 3. Write Flag (Master -> Slave)
  // 4. ACK (Slave -> Master)
  // 5. Register Address (Master -> Slave)
  // 6. ACK (Slave -> Master)
  // 7. Stop Condition (Master -> Slave)
  const auto req_length = length + 1;
  const auto res = ::write(i2c_fd_, tx_, req_length);
  if (res < 0) {
    std::cerr << "I2C write failed: " << strError() << std::endl;
    switch (errno) {
      case EREMOTEIO:
        std::cerr << "Please ensure that the correct 7-bit slave address is set." << std::endl;
        break;
    }
    return false;
  }
  if (res != static_cast<ssize_t>(req_length)) {
    std::cerr << "Tried to write " << req_length << " bytes on " << reg_addr << ", but " << res
              << " bytes were written." << std::endl;
    return false;
  }

  return true;
}

bool I2Cdev::read(size_t length, void* rx)
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
  const auto res = ::read(i2c_fd_, rx, length);
  if (res < 0) {
    std::cerr << "I2C read failed: " << strError() << std::endl;
    return false;
  }
  if (res != static_cast<ssize_t>(length)) {
    std::cerr << "Tried to read " << length << " bytes, but " << res << " bytes were read." << std::endl;
    return false;
  }

  return true;
}
}  // namespace linux
}  // namespace tobas
