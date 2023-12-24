#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/i2c-dev.h>

#include "../../include/Common/I2Cdev.h"

/** Default constructor.
 */
I2Cdev::I2Cdev()
{
}

int8_t I2Cdev::readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t* data)
{
  uint8_t b;
  uint8_t count = readByte(devAddr, regAddr, &b);
  *data = b & (1 << bitNum);
  return count;
}

int8_t I2Cdev::readBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t* data)
{
  uint16_t b;
  uint8_t count = readWord(devAddr, regAddr, &b);
  *data = b & (1 << bitNum);
  return count;
}

int8_t
I2Cdev::readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t* data)
{
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  uint8_t count, b;
  if ((count = readByte(devAddr, regAddr, &b)) != 0)
  {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    b &= mask;
    b >>= (bitStart - length + 1);
    *data = b;
  }
  return count;
}

int8_t I2Cdev::readBitsW(
  uint8_t devAddr,
  uint8_t regAddr,
  uint8_t bitStart,
  uint8_t length,
  uint16_t* data)
{
  // 1101011001101001 read byte
  // fedcba9876543210 bit numbers
  //    xxx           args: bitStart=12, length=3
  //    010           masked
  //           -> 010 shifted
  uint8_t count;
  uint16_t w;
  if ((count = readWord(devAddr, regAddr, &w)) != 0)
  {
    uint16_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    w &= mask;
    w >>= (bitStart - length + 1);
    *data = w;
  }
  return count;
}

int8_t I2Cdev::readByte(uint8_t devAddr, uint8_t regAddr, uint8_t* data)
{
  return readBytes(devAddr, regAddr, 1, data);
}

int8_t I2Cdev::readWord(uint8_t devAddr, uint8_t regAddr, uint16_t* data)
{
  return readWords(devAddr, regAddr, 1, data);
}

int8_t I2Cdev::readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  int8_t count = 0;
  int fd = open(I2CDEV, O_RDWR);

  if (fd < 0)
  {
    fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
    return (-1);
  }
  if (ioctl(fd, I2C_SLAVE, devAddr) < 0)
  {
    fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
    close(fd);
    return (-1);
  }
  if (write(fd, &regAddr, 1) != 1)
  {
    fprintf(stderr, "Failed to write reg: %s\n", strerror(errno));
    close(fd);
    return (-1);
  }
  count = read(fd, data, length);

  if (count < 0)
  {
    fprintf(stderr, "Failed to read device(%d): %s\n", count, ::strerror(errno));
    close(fd);
    return (-1);
  }
  else if (count != length)
  {
    fprintf(stderr, "Short read  from device, expected %d, got %d\n", length, count);
    close(fd);
    return (-1);
  }
  close(fd);

  return count;
}

int8_t I2Cdev::readBytesNoRegAddress(uint8_t devAddr, uint8_t length, uint8_t* data)
{
  int8_t count = 0;
  int fd = open(I2CDEV, O_RDWR);

  if (fd < 0)
  {
    fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
    return (-1);
  }
  if (ioctl(fd, I2C_SLAVE, devAddr) < 0)
  {
    fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
    close(fd);
    return (-1);
  }
  count = read(fd, data, length);

  if (count < 0)
  {
    fprintf(stderr, "Failed to read device(%d): %s\n", count, ::strerror(errno));
    close(fd);
    return (-1);
  }
  else if (count != length)
  {
    fprintf(stderr, "Short read  from device, expected %d, got %d\n", length, count);
    close(fd);
    return (-1);
  }
  close(fd);

  return count;
}

int8_t I2Cdev::readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data)
{
  int8_t count = 0;
  count = readBytes(devAddr, regAddr, length * 2, reinterpret_cast<uint8_t*>(data));
  return count / 2;
}

bool I2Cdev::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
  uint8_t b;
  readByte(devAddr, regAddr, &b);
  b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
  return writeByte(devAddr, regAddr, b);
}

bool I2Cdev::writeBitW(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint16_t data)
{
  uint16_t w;
  readWord(devAddr, regAddr, &w);
  w = (data != 0) ? (w | (1 << bitNum)) : (w & ~(1 << bitNum));
  return writeWord(devAddr, regAddr, w);
}

bool I2Cdev::writeBits(
  uint8_t devAddr,
  uint8_t regAddr,
  uint8_t bitStart,
  uint8_t length,
  uint8_t data)
{
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  uint8_t b;
  if (readByte(devAddr, regAddr, &b) != 0)
  {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);  // shift data into correct position
    data &= mask;                      // zero all non-important bits in data
    b &= ~(mask);                      // zero all important bits in existing byte
    b |= data;                         // combine data with existing byte
    return writeByte(devAddr, regAddr, b);
  }
  else
  {
    return false;
  }
}

bool I2Cdev::writeBitsW(
  uint8_t devAddr,
  uint8_t regAddr,
  uint8_t bitStart,
  uint8_t length,
  uint16_t data)
{
  //              010 value to write
  // fedcba9876543210 bit numbers
  //    xxx           args: bitStart=12, length=3
  // 0001110000000000 mask byte
  // 1010111110010110 original value (sample)
  // 1010001110010110 original & ~mask
  // 1010101110010110 masked | value
  uint16_t w;
  if (readWord(devAddr, regAddr, &w) != 0)
  {
    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1);  // shift data into correct position
    data &= mask;                      // zero all non-important bits in data
    w &= ~(mask);                      // zero all important bits in existing word
    w |= data;                         // combine data with existing word
    return writeWord(devAddr, regAddr, w);
  }
  else
  {
    return false;
  }
}

bool I2Cdev::writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
  return writeBytes(devAddr, regAddr, 1, &data);
}

bool I2Cdev::writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data)
{
  return writeWords(devAddr, regAddr, 1, &data);
}

bool I2Cdev::writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t* data)
{
  int8_t count = 0;
  uint8_t buf[128];
  int fd;

  if (length > 127)
  {
    fprintf(stderr, "Byte write count (%d) > 127\n", length);
    return (FALSE);
  }

  fd = open(I2CDEV, O_RDWR);
  if (fd < 0)
  {
    fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
    return (FALSE);
  }
  if (ioctl(fd, I2C_SLAVE, devAddr) < 0)
  {
    fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
    close(fd);
    return (FALSE);
  }
  buf[0] = regAddr;
  memcpy(buf + 1, data, length);
  count = write(fd, buf, length + 1);
  if (count < 0)
  {
    fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
    close(fd);
    return (FALSE);
  }
  else if (count != length + 1)
  {
    fprintf(stderr, "Short write to device, expected %d, got %d\n", length + 1, count);
    close(fd);
    return (FALSE);
  }
  close(fd);

  return TRUE;
}

bool I2Cdev::writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t* data)
{
  int8_t count = 0;
  uint8_t buf[128];
  int i, fd;

  // Should do potential byteswap and call writeBytes() really, but that
  // messes with the callers buffer

  if (length > 63)
  {
    fprintf(stderr, "Word write count (%d) > 63\n", length);
    return (FALSE);
  }

  fd = open(I2CDEV, O_RDWR);
  if (fd < 0)
  {
    fprintf(stderr, "Failed to open device: %s\n", strerror(errno));
    return (FALSE);
  }
  if (ioctl(fd, I2C_SLAVE, devAddr) < 0)
  {
    fprintf(stderr, "Failed to select device: %s\n", strerror(errno));
    close(fd);
    return (FALSE);
  }
  buf[0] = regAddr;
  for (i = 0; i < length; ++i)
  {
    buf[i * 2 + 1] = data[i] >> 8;
    buf[i * 2 + 2] = data[i];
  }
  count = write(fd, buf, length * 2 + 1);
  if (count < 0)
  {
    fprintf(stderr, "Failed to write device(%d): %s\n", count, ::strerror(errno));
    close(fd);
    return (FALSE);
  }
  else if (count != length * 2 + 1)
  {
    fprintf(stderr, "Short write to device, expected %d, got %d\n", length + 1, count);
    close(fd);
    return (FALSE);
  }
  close(fd);
  return TRUE;
}
