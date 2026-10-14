// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/spi_dev.hpp>

namespace tobas::fc1xx
{
class PWM
{
public:
  static constexpr size_t kChannelSize = 8;

public:
  explicit PWM();

  bool initialize();

  void setPeriod(size_t _ch, uint16_t _period_us);
  bool transfer();

private:
  linux::SPIdev spi_;
  uint16_t tx_buf_[kChannelSize + 2] = {};
  uint16_t rx_buf_[kChannelSize + 2] = {};

  algo::CRC32Left crc_;
};
}  // namespace tobas::fc1xx
