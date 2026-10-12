// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fc1xx_core/pwm.hpp"

#include <algorithm>
#include <cassert>

namespace tobas
{
namespace fc1xx
{
PWM::PWM() : crc_(algo::CRC32Left::CRC_32)
{
  crc_.initialize();
}

bool PWM::initialize()
{
  if (!spi_.initialize("/dev/spidev1.1", tx_buf_, rx_buf_, 50'000'000)) {
    return false;
  }

  return true;
}

void PWM::setPeriod(size_t ch, uint16_t period_us)
{
  assert(ch < kChannelSize);
  tx_buf_[ch] = std::min<uint16_t>(period_us, 2500);
}

bool PWM::transfer()
{
  // Compute CRC.
  *(uint32_t*)(tx_buf_ + kChannelSize) = crc_.compute((uint8_t*)tx_buf_, sizeof(uint16_t) * kChannelSize);

  // Transfer.
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return false;
  }

  return true;
}
}  // namespace fc1xx
}  // namespace tobas
