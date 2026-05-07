// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <linux/spi/spidev.h>

#include <cstdint>

namespace tobas
{
namespace linux
{
class SPIdev
{
public:
  explicit SPIdev() noexcept;
  ~SPIdev() noexcept;

  bool
  initialize(const char* spi_dev, void* tx_buf, void* rx_buf, uint32_t speed_hz, uint8_t bits_per_word = 8) noexcept;

  /**
   * @brief 引数で与えたバイト数だけ送受信する．
   * @note デバイスが接続されてないなど，SPIスレーブが機能していない場合はデッドロックする．
   */
  bool transfer(uint32_t length) noexcept;

private:
  spi_ioc_transfer spi_transfer_;
  int spi_fd_ = -1;
};
}  // namespace linux
}  // namespace tobas
