// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_transport_uart.hpp"

namespace tobas::ublox
{
UbxTransportUart::UbxTransportUart(const char* _device, uint32_t _baud_rate) : device_(_device), baud_rate_(_baud_rate)
{
}

bool UbxTransportUart::initialize() noexcept
{
  if (!uart_.initialize(device_, false)) {
    return false;
  }

  if (!uart_.setBaudRate(baud_rate_)) {
    return false;
  }

  if (!uart_.setDataBits(8)) {
    return false;
  }

  if (!uart_.setSingleStopBit()) {
    return false;
  }

  if (!uart_.disableParity()) {
    return false;
  }

  return true;
}

std::expected<std::optional<uint8_t>, int> UbxTransportUart::receiveByte() noexcept
{
  return uart_.tryReceiveByte();
}

bool UbxTransportUart::send(const uint8_t* _data, size_t _length) noexcept
{
  constexpr auto kSendTimeout = std::chrono::seconds(1);
  return uart_.sendAll(_data, _length, kSendTimeout);
}

bool UbxTransportUart::setBaudRate(uint32_t _baud_rate) noexcept
{
  if (!uart_.drainOutput()) {
    return false;
  }

  if (!uart_.setBaudRate(_baud_rate)) {
    return false;
  }

  baud_rate_ = _baud_rate;
  return true;
}

}  // namespace tobas::ublox
