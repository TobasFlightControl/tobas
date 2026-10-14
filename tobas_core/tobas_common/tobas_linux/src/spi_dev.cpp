// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/spi_dev.hpp"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstring>

namespace tobas
{
namespace linux
{
SPIdev::SPIdev() noexcept
{
}

SPIdev::~SPIdev() noexcept
{
  if (spi_fd_ >= 0) {
    close(spi_fd_);
  }
}

bool SPIdev::initialize(
  const char* _spi_dev,
  void* _tx_buf,
  void* _rx_buf,
  uint32_t _speed_hz,
  uint8_t _bits_per_word) noexcept
{
  spi_fd_ = open(_spi_dev, O_RDWR);
  if (spi_fd_ < 0) {
    return false;
  }

  std::memset(&spi_transfer_, 0, sizeof(spi_ioc_transfer));
  spi_transfer_.tx_buf = (uint64_t)_tx_buf;
  spi_transfer_.rx_buf = (uint64_t)_rx_buf;
  spi_transfer_.speed_hz = _speed_hz;
  spi_transfer_.bits_per_word = _bits_per_word;
  spi_transfer_.delay_usecs = 0;

  return true;
}

bool SPIdev::transfer(uint32_t _length) noexcept
{
  spi_transfer_.len = _length;

  if (ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &spi_transfer_) < 0) {
    return false;
  }

  return true;
}
}  // namespace linux
}  // namespace tobas
