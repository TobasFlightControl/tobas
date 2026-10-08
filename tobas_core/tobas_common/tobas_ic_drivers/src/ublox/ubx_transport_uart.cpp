// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_transport_uart.hpp"

namespace tobas
{
namespace ublox
{
UbxTransportUart::UbxTransportUart(const char* _device, uint32_t _baud_rate) : device_(_device), baud_rate_(_baud_rate)
{
}

bool UbxTransportUart::initialize() noexcept
{
  if (!uart_.initialize(device_, true)) {
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

std::optional<uint8_t> UbxTransportUart::receiveByte() noexcept
{
  return uart_.receiveByte();
}

bool UbxTransportUart::send(const uint8_t* _data, size_t _length) noexcept
{
  return uart_.send(_data, _length);
}

bool UbxTransportUart::setBaudRate(uint32_t baud_rate) noexcept
{
  if (!uart_.setBaudRate(baud_rate)) {
    return false;
  }

  baud_rate_ = baud_rate;
  return true;
}

bool UbxTransportUart::setReceiveTimeout(uint8_t timeout_100ms) noexcept
{
  if (!uart_.setMinimumChars(0)) {
    return false;
  }

  if (!uart_.setTimeout(timeout_100ms)) {
    return false;
  }

  return true;
}

bool UbxTransportUart::disableReceiveTimeout() noexcept
{
  if (!uart_.setTimeout(0)) {
    return false;
  }

  if (!uart_.setMinimumChars(1)) {
    return false;
  }

  return true;
}


}  // namespace ublox
}  // namespace tobas
