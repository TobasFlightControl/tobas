#include <cstring>
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "../include/tobas_linux/spi_dev.hpp"

using namespace std;

namespace linux
{
SPIdev::SPIdev()
{
}

SPIdev::~SPIdev()
{
  close(spi_fd_);
}

bool SPIdev::initialize(const char* spi_dev, uint32_t speed_hz, uint8_t bits_per_word, uint16_t delay_usecs)
{
  memset(&spi_transfer_, 0, sizeof(spi_ioc_transfer));
  spi_transfer_.speed_hz = speed_hz;
  spi_transfer_.bits_per_word = bits_per_word;
  spi_transfer_.delay_usecs = delay_usecs;

  spi_fd_ = open(spi_dev, O_RDWR);
  if (spi_fd_ < 0)
  {
    cerr << "Failed to open SPI device: " << spi_dev << endl;
    return false;
  }

  return true;
}

bool SPIdev::transfer(uint8_t* tx, uint8_t* rx, uint32_t length)
{
  spi_transfer_.tx_buf = (uint64_t)tx;
  spi_transfer_.rx_buf = (uint64_t)rx;
  spi_transfer_.len = length;

  if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &spi_transfer_) < 0)
  {
    cerr << "SPI transfer failed." << endl;
    return false;
  }

  return true;
}
}  // namespace linux
