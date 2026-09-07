// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_uart_transport.hpp"

namespace tobas
{
namespace ublox
{
UbxTransportUart::UbxTransportUart(const char* _device, uint32_t _baud_rate) : device_(_device), baud_rate_(_baud_rate)
{
}

bool UbxTransportUart::initialize() noexcept
{
  return uart_.initialize(device_, false) && uart_.setBaudRate(baud_rate_);
}

std::optional<uint8_t> UbxTransportUart::receiveByte() noexcept
{
  uint8_t data;
  if (!uart_.receive(&data, 1)) {
    return std::nullopt;
  }

  return data;
}

std::chrono::microseconds UbxTransportUart::receiveByteInterval() const noexcept
{
  return std::chrono::microseconds(0);
}

bool UbxTransportUart::send(const uint8_t* _data, size_t _length) noexcept
{
  return uart_.send(_data, _length);
}
}  // namespace ublox
}  // namespace tobas
