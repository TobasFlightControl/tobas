#pragma once

#include <cinttypes>
#include <linux/spi/spidev.h>

namespace linux
{
class SPIdev
{
public:
  explicit SPIdev();
  ~SPIdev();

  bool initialize(const char* spi_dev, uint32_t speed_hz, uint8_t bits_per_word = 8, uint16_t delay_usecs = 0);

  bool transfer(uint8_t* tx, uint8_t* rx, uint32_t length);

private:
  spi_ioc_transfer spi_transfer_;
  int spi_fd_;
};
}  // namespace linux
