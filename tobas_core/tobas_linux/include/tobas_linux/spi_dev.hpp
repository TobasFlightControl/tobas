#pragma once

#include <cinttypes>
#include <cstddef>
#include <linux/spi/spidev.h>

namespace linux
{
class SPIdev
{
public:
  uint8_t* tx = nullptr;
  uint8_t* rx = nullptr;

  explicit SPIdev();
  ~SPIdev();

  bool initialize(const char* spi_dev, uint32_t speed_hz, size_t buf_size);
  bool transfer(uint32_t length);

private:
  spi_ioc_transfer spi_transfer_;
  int spi_fd_ = -1;
  size_t buf_size_ = 0;
};
}  // namespace linux
