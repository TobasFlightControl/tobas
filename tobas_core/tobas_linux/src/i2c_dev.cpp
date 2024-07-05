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
I2Cdev::I2Cdev(const char* i2c_dev, uint8_t dev_addr) : i2c_dev_(i2c_dev), dev_addr_(dev_addr)
{
}

I2Cdev::~I2Cdev()
{
  close(i2c_fd_);
}

bool I2Cdev::initialize()
{
  i2c_fd_ = open(i2c_dev_, O_RDWR);
  return i2c_fd_ >= 0;
}

bool I2Cdev::readBit(uint8_t reg_addr, uint8_t bit_num, uint8_t* data)
{
  uint8_t b;
  if (!readByte(reg_addr, &b))
    return false;

  *data = b & (1 << bit_num);
  return true;
}

bool I2Cdev::readBitW(uint8_t reg_addr, uint8_t bit_num, uint16_t* data)
{
  uint16_t w;
  if (!readWord(reg_addr, &w))
    return false;

  *data = w & (1 << bit_num);
  return true;
}

bool I2Cdev::readBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t* data)
{
  uint8_t b;
  if (!readByte(reg_addr, &b))
    return false;

  const uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
  b &= mask;
  b >>= (bit_start - length + 1);
  *data = b;

  return true;
}

bool I2Cdev::readBitsW(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t* data)
{
  uint16_t w;
  if (!readWord(reg_addr, &w))
    return false;

  const uint16_t mask = ((1 << length) - 1) << (bit_start - length + 1);
  w &= mask;
  w >>= (bit_start - length + 1);
  *data = w;

  return true;
}

bool I2Cdev::readByte(uint8_t reg_addr, uint8_t* data)
{
  return readBytes(reg_addr, 1, data);
}

bool I2Cdev::readWord(uint8_t reg_addr, uint16_t* data)
{
  return readWords(reg_addr, 1, data);
}

bool I2Cdev::readBytes(uint8_t reg_addr, uint8_t length, uint8_t* data)
{
  if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr_) < 0)
  {
    cerr << "Failed to select device." << endl;
    return false;
  }
  if (write(i2c_fd_, &reg_addr, 1) != 1)
  {
    cerr << "Failed to write register." << endl;
    return false;
  }

  const auto res = read(i2c_fd_, data, length);
  if (res < 0)
  {
    cerr << "Failed to read device." << endl;
    return false;
  }
  else if (res != length)
  {
    cerr << "Short read from device, expected " << length << ", got " << res << "." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::readBytesNoRegAddress(uint8_t length, uint8_t* data)
{
  if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr_) < 0)
  {
    cerr << "Failed to select device." << endl;
    return false;
  }

  const auto res = read(i2c_fd_, data, length);
  if (res < 0)
  {
    cerr << "Failed to read device." << endl;
    return false;
  }
  else if (res != length)
  {
    cerr << "Short read from device, expected " << length << ", got " << res << "." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::readWords(uint8_t reg_addr, uint8_t length, uint16_t* data)
{
  return readBytes(reg_addr, length * 2, reinterpret_cast<uint8_t*>(data));
}

bool I2Cdev::writeBit(uint8_t reg_addr, uint8_t bit_num, uint8_t data)
{
  uint8_t b;
  if (!readByte(reg_addr, &b))
    return false;

  b = (data != 0) ? (b | (1 << bit_num)) : (b & ~(1 << bit_num));
  return writeByte(reg_addr, b);
}

bool I2Cdev::writeBitW(uint8_t reg_addr, uint8_t bit_num, uint16_t data)
{
  uint16_t w;
  if (!readWord(reg_addr, &w))
    return false;

  w = (data != 0) ? (w | (1 << bit_num)) : (w & ~(1 << bit_num));
  return writeWord(reg_addr, w);
}

bool I2Cdev::writeBits(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint8_t data)
{
  uint8_t b;
  if (!readByte(reg_addr, &b))
    return false;

  const uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
  data <<= (bit_start - length + 1);  // shift data into correct position
  data &= mask;                       // zero all non-important bits in data
  b &= ~(mask);                       // zero all important bits in existing byte
  b |= data;                          // combine data with existing byte

  return writeByte(reg_addr, b);
}

bool I2Cdev::writeBitsW(uint8_t reg_addr, uint8_t bit_start, uint8_t length, uint16_t data)
{
  uint16_t w;
  if (!readWord(reg_addr, &w))
    return false;

  const uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
  data <<= (bit_start - length + 1);  // shift data into correct position
  data &= mask;                       // zero all non-important bits in data
  w &= ~(mask);                       // zero all important bits in existing word
  w |= data;                          // combine data with existing word

  return writeWord(reg_addr, w);
}

bool I2Cdev::writeByte(uint8_t reg_addr, uint8_t data)
{
  return writeBytes(reg_addr, 1, &data);
}

bool I2Cdev::writeWord(uint8_t reg_addr, uint16_t data)
{
  return writeWords(reg_addr, 1, &data);
}

bool I2Cdev::writeBytes(uint8_t reg_addr, uint8_t length, uint8_t* data)
{
  constexpr uint8_t kMaximumLength = (1 << 7) - 1;
  if (length > kMaximumLength)
  {
    cerr << "Byte write count too large: " << length << " > " << kMaximumLength;
    return false;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr_) < 0)
  {
    cerr << "Failed to select device." << endl;
    return false;
  }

  buf_[0] = reg_addr;
  memcpy(buf_ + 1, data, length);

  const auto res = write(i2c_fd_, buf_, length + 1);
  if (res < 0)
  {
    cerr << "Failed to write device." << endl;
    return false;
  }
  else if (res != length + 1)
  {
    cerr << "Short write to device, expected " << length + 1 << ", got " << res << "." << endl;
    return false;
  }

  return true;
}

bool I2Cdev::writeWords(uint8_t reg_addr, uint8_t length, uint16_t* data)
{
  // TODO: Should do potential byteswap and call writeBytes() really, but that messes with the callers buffer.

  constexpr uint8_t kMaximumLength = (1 << 6) - 1;
  if (length > kMaximumLength)
  {
    cerr << "Word write count too large: " << length << " > " << kMaximumLength;
    return false;
  }

  if (ioctl(i2c_fd_, I2C_SLAVE, dev_addr_) < 0)
  {
    cerr << "Failed to select device." << endl;
    return false;
  }

  buf_[0] = reg_addr;
  for (uint8_t i = 0; i < length; ++i)
  {
    buf_[i * 2 + 1] = data[i] >> 8;
    buf_[i * 2 + 2] = data[i];
  }

  const auto res = write(i2c_fd_, buf_, length * 2 + 1);
  if (res < 0)
  {
    cerr << "Failed to write device." << endl;
    return false;
  }
  else if (res != length * 2 + 1)
  {
    cerr << "Short write to device, expected " << length + 1 << ", got " << res << "." << endl;
    return false;
  }

  return true;
}
}  // namespace linux
