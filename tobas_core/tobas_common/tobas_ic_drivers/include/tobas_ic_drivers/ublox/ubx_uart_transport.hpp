// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>

#include <tobas_linux/uart_dev.hpp>

#include "tobas_ic_drivers/ublox/ubx_transport.hpp"

namespace tobas
{
namespace ublox
{
class UbxTransportUart final : public UbxTransport
{
public:
  explicit UbxTransportUart(uint32_t _baud_rate);
  UbxTransportUart(UbxTransportUart&& _other) = delete;
  UbxTransportUart& operator=(UbxTransportUart&& _other) = delete;
  UbxTransportUart(const UbxTransportUart& _other) = delete;
  UbxTransportUart& operator=(const UbxTransportUart& _other) = delete;

  bool initialize(const char* _device) noexcept override;
  ReceiveResult receiveByte(bool _nonblock) noexcept override;
  std::chrono::microseconds receiveByteInterval() const noexcept override;
  bool send(const uint8_t* _data, size_t _length) noexcept override;

private:
  static constexpr auto kReceiveByteInterval = std::chrono::microseconds(0);

  const uint32_t baud_rate_;
  linux::UARTdev uart_;
};
}  // namespace ublox
}  // namespace tobas
