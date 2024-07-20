#include <iostream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "../include/tobas_linux/i2c_dev.hpp"

using namespace std;

namespace linux
{
I2Cdev::I2Cdev()
{
}

I2Cdev::~I2Cdev()
{
  free(tx);
  free(rx);
  free(buf_);

  close(i2c_fd_);
}

bool I2Cdev::initialize(const char* i2c_dev, uint8_t dev_addr, size_t buf_size)
{
  i2c_fd_ = open(i2c_dev, O_RDWR);
  if (i2c_fd_ < 0)
  {
    cerr << "Failed to open I2C device: " << i2c_dev << endl;
    return false;
  }

  dev_addr_ = dev_addr;
  buf_size_ = buf_size;

  tx = (uint8_t*)malloc(buf_size * sizeof(uint8_t));
  rx = (uint8_t*)malloc(buf_size * sizeof(uint8_t));
  buf_ = (uint8_t*)malloc((buf_size + 1) * sizeof(uint8_t));

  return true;
}

bool I2Cdev::readBit(uint8_t reg_addr, uint8_t bit_num, bool& flag)
{
  if (!readBytes(reg_addr, 1))
    return false;

  flag = rx[0] & (1 << bit_num);
  return true;
}

bool I2Cdev::readBytes(uint8_t reg_addr, size_t length)
{
  if (length > buf_size_)
  {
    cerr << "Data length is greater than buffer size." << endl;
    return false;
  }

  if (!selectDevice())
    return false;

  if (write(i2c_fd_, &reg_addr, 1) != 1)
  {
    cerr << "I2C write error." << endl;
    return false;
  }

  if (read(i2c_fd_, rx, length) != static_cast<ssize_t>(length))
  {
    cerr << "I2C read error." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::readBytesNoRegAddress(size_t length)
{
  if (length > buf_size_)
  {
    cerr << "Data length is greater than buffer size." << endl;
    return false;
  }

  if (!selectDevice())
    return false;

  if (read(i2c_fd_, rx, length) != static_cast<ssize_t>(length))
  {
    cerr << "I2C read error." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::writeBit(uint8_t reg_addr, uint8_t bit_num, bool flag)
{
  if (!readBytes(reg_addr, 1))
    return false;

  tx[0] = flag ? (rx[0] | (1 << bit_num)) : (rx[0] & ~(1 << bit_num));
  return writeBytes(reg_addr, 1);
}

bool I2Cdev::writeBytes(uint8_t reg_addr, size_t length)
{
  if (length > buf_size_)
  {
    cerr << "Data length is greater than buffer size." << endl;
    return false;
  }

  if (!selectDevice())
    return false;

  buf_[0] = reg_addr;
  memcpy(buf_ + 1, tx, length);

  const auto size = length + 1;
  if (write(i2c_fd_, buf_, size) != static_cast<ssize_t>(size))
  {
    cerr << "I2C write error." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::selectDevice()
{
  if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr_) < 0)
  {
    cerr << "Failed to select I2C device." << endl;
    return false;
  }

  return true;
}
}  // namespace linux
