#include <sys/ioctl.h>
#include <stdexcept>
#include <fcntl.h>

#include "../../include/Common/SPIdev.h"

using namespace std;

SPIdev::SPIdev(const char* spidev, uint32_t speed_hz, u_char bits_per_word, u_short delay_usecs)
{
  memset(&spi_transfer_, 0, sizeof(spi_ioc_transfer));
  spi_transfer_.speed_hz = speed_hz;
  spi_transfer_.bits_per_word = bits_per_word;
  spi_transfer_.delay_usecs = delay_usecs;

  spi_fd_ = open(spidev, O_RDWR);
  if (spi_fd_ < 0)
  {
    throw runtime_error("Failed to open SPI device.");
  }
}

SPIdev::~SPIdev()
{
  close(spi_fd_);
}

bool SPIdev::transfer(u_char* tx, u_char* rx, uint32_t length)
{
  spi_transfer_.tx_buf = (u_long)tx;
  spi_transfer_.rx_buf = (u_long)rx;
  spi_transfer_.len = length;
  return ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &spi_transfer_) >= 0;
}
