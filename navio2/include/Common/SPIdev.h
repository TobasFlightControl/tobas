#pragma once

#include <linux/spi/spidev.h>
#include <linux/types.h>
#include <sys/types.h>
#include <cinttypes>
#include <unistd.h>
#include <cstring>

class SPIdev
{
public:
  explicit SPIdev(
    const char* spidev,
    uint32_t speed_hz,
    u_char bits_per_word = 8,
    u_short delay_usecs = 0);
  ~SPIdev();

  bool transfer(u_char* tx, u_char* rx, uint32_t length);

private:
  spi_ioc_transfer spi_transfer_;
  int spi_fd_;
};
