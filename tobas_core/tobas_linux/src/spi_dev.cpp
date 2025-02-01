#include <cstring>
#include <iostream>
#include <cassert>
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
  if (spi_fd_ >= 0)
    close(spi_fd_);
}

bool SPIdev::initialize(const char* spi_dev, void* tx_buf, void* rx_buf, uint32_t speed_hz, uint8_t bits_per_word)
{
  spi_fd_ = open(spi_dev, O_RDWR);
  if (spi_fd_ < 0)
  {
    cerr << "Failed to open SPI device: " << spi_dev << endl;
    return false;
  }

  memset(&spi_transfer_, 0, sizeof(spi_ioc_transfer));
  spi_transfer_.tx_buf = (uint64_t)tx_buf;
  spi_transfer_.rx_buf = (uint64_t)rx_buf;
  spi_transfer_.speed_hz = speed_hz;
  spi_transfer_.bits_per_word = bits_per_word;
  spi_transfer_.delay_usecs = 0;

  return true;
}

bool SPIdev::transfer(uint32_t length)
{
  spi_transfer_.len = length;

  if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &spi_transfer_) < 0)
  {
    cerr << "SPI transfer failed." << endl;
    return false;
  }

  return true;
}
}  // namespace linux
