#pragma once

#include <cstdint>
#include <linux/spi/spidev.h>

namespace linux
{
class SPIdev
{
public:
  explicit SPIdev();
  ~SPIdev();

  bool initialize(const char* spi_dev, void* tx_buf, void* rx_buf, uint32_t speed_hz, uint8_t bits_per_word = 8);
  bool transfer(uint32_t length);

private:
  spi_ioc_transfer spi_transfer_;
  int spi_fd_ = -1;
};
}  // namespace linux
