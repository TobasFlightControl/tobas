// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fc1xx_core/battery.hpp"

#include <bit>
#include <thread>

using namespace std::chrono_literals;

namespace tobas::fc1xx
{
Battery::Battery()
{
}

bool Battery::initialize()
{
  if (!spi_.initialize("/dev/spidev0.1", tx_buf_, rx_buf_, 30'000'000)) {
    return false;
  }

  // Discard the initial data.
  for (int _ = 0; _ < 10; ++_) {
    std::this_thread::sleep_for(1ms);
    if (!spi_.transfer(sizeof(tx_buf_))) {
      return false;
    }
  }

  return true;
}

bool Battery::read(float& _voltage, float& _current)
{
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return false;
  }

  _voltage = std::bit_cast<float>(rx_buf_[0]);
  _current = std::bit_cast<float>(rx_buf_[1]);

  return true;
}
}  // namespace tobas::fc1xx
