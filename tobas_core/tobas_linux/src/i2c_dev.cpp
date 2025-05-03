#include <iostream>
#include <cstring>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "../include/tobas_linux/i2c_dev.hpp"
#include "../include/tobas_linux/errer.hpp"

using namespace std;

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
    cerr << "Invalid slave address. It must be 7-bit." << endl;
    return false;
  }

  i2c_fd_ = open(i2c_dev, O_RDWR);
  if (i2c_fd_ < 0) {
    cerr << "Failed to open I2C device: " << i2c_dev << endl;
    return false;
  }

  dev_addr_ = dev_addr;

  // Wait here to avoid 121 remote I/O error
  this_thread::sleep_for(10ms);

  return true;
}

bool I2Cdev::readBit(uint8_t reg_addr, uint8_t bit_num, bool& flag)
{
  if (!readBytes(reg_addr, 1)) {
    return false;
  }

  flag = rx[0] & (1 << bit_num);
  return true;
}

bool I2Cdev::readBytes(uint8_t reg_addr, size_t length)
{
  if (!checkDataLength(length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!write(reg_addr, 0)) {
    return false;
  }

  if (!read(length)) {
    return false;
  }

  return true;
}

bool I2Cdev::readBytesNoRegAddress(size_t length)
{
  if (!checkDataLength(length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!read(length)) {
    return false;
  }

  return true;
}

bool I2Cdev::writeBit(uint8_t reg_addr, uint8_t bit_num, bool flag)
{
  if (!readBytes(reg_addr, 1)) {
    return false;
  }

  tx[0] = flag ? (rx[0] | (1 << bit_num)) : (rx[0] & ~(1 << bit_num));
  return writeBytes(reg_addr, 1);
}

bool I2Cdev::writeBytes(uint8_t reg_addr, size_t length)
{
  if (!checkDataLength(length)) {
    return false;
  }

  if (!selectDevice()) {
    return false;
  }

  if (!write(reg_addr, length)) {
    return false;
  }

  return true;
}

bool I2Cdev::checkDataLength(size_t length) const
{
  if (length > kBufSize) {
    cerr << "Data length cannot be greater than buffer size." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::selectDevice() const
{
  // スレーブアドレスを登録
  // I2Cの規格ではスレーブアドレスに続いてコマンドを送ることもできるが，LinuxのI2Cドライバはまず応答確認のみ行う．
  if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr_) < 0) {
    cerr << "Failed to select I2C device." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::write(uint8_t reg_addr, size_t length)
{
  tx_[0] = reg_addr;
  memcpy(tx_ + 1, tx, length);

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
    cerr << "I2C write failed: " << strError() << endl;
    switch (errno) {
      case EREMOTEIO:
        cerr << "Please ensure that the correct 7-bit slave address is set." << endl;
        break;
    }
    return false;
  }
  if (res != static_cast<ssize_t>(req_length)) {
    cerr << "Tried to write " << req_length << " bytes, but " << res << " bytes were written." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::read(size_t length)
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
    cerr << "I2C read failed: " << strError() << endl;
    return false;
  }
  if (res != static_cast<ssize_t>(length)) {
    cerr << "Tried to read " << length << " bytes, but " << res << " bytes were read." << endl;
    return false;
  }

  return true;
}
}  // namespace linux
