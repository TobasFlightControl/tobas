// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace tobas
{
namespace fc1xx
{
class Battery
{
  static constexpr size_t kChannelSize = 2;

public:
  explicit Battery();

  bool initialize();
  bool read(float& voltage, float& current);

private:
  linux::SPIdev spi_;
  uint32_t tx_buf_[kChannelSize];
  uint32_t rx_buf_[kChannelSize];
};
}  // namespace fc1xx
}  // namespace tobas
