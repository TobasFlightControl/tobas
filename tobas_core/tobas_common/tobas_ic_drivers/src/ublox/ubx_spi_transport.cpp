// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_spi_transport.hpp"

#include <cstring>

namespace tobas
{
namespace ublox
{
UbxTransportSpi::UbxTransportSpi()
{
}

bool UbxTransportSpi::initialize(const char* _device) noexcept
{
  return spi_.initialize(_device, tx_buf_, rx_buf_, kSpiClockFreq);
}

std::optional<uint8_t> UbxTransportSpi::receiveByte() noexcept
{
  if (!spi_.transfer(1)) {
    return std::nullopt;
  }

  return rx_buf_[0];
}

std::chrono::microseconds UbxTransportSpi::receiveByteInterval() const noexcept
{
  return kReceiveByteInterval;
}

bool UbxTransportSpi::send(const uint8_t* _data, size_t _length) noexcept
{
  if (_length > kSpiBufSize) {
    return false;
  }

  std::memcpy(tx_buf_, _data, _length);
  return spi_.transfer(_length);
}
}  // namespace ublox
}  // namespace tobas
