// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_uart_transport.hpp"

namespace tobas
{
namespace ublox
{
UbxTransportUart::UbxTransportUart(uint32_t _baud_rate) : baud_rate_(_baud_rate)
{
}

bool UbxTransportUart::initialize(const char* _device) noexcept
{
  return uart_.initialize(_device, false) && uart_.setBaudRate(baud_rate_);
}

std::optional<uint8_t> UbxTransportUart::receiveByte() noexcept
{
  return uart_.tryReceiveByte();
}

std::chrono::microseconds UbxTransportUart::receiveByteInterval() const noexcept
{
  return kReceiveByteInterval;
}

bool UbxTransportUart::send(const uint8_t* _data, size_t _length) noexcept
{
  return uart_.send(_data, _length);
}
}  // namespace ublox
}  // namespace tobas
