// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <tobas_linux/uart_dev.hpp>

#include "tobas_ic_drivers/ublox/ubx_transport.hpp"

namespace tobas
{
namespace ublox
{
class UbxTransportUart final : public UbxTransport
{
public:
  explicit UbxTransportUart(const char* _device, uint32_t _baud_rate);

  UbxTransportUart(UbxTransportUart&& _other) = delete;
  UbxTransportUart& operator=(UbxTransportUart&& _other) = delete;
  UbxTransportUart(const UbxTransportUart& _other) = delete;
  UbxTransportUart& operator=(const UbxTransportUart& _other) = delete;

  bool initialize() noexcept override;
  std::optional<uint8_t> receiveByte() noexcept override;
  bool send(const uint8_t* _data, size_t _length) noexcept override;

private:
  const char* device_;
  const uint32_t baud_rate_;

  linux::UARTdev uart_;
  uint8_t data_;
};
}  // namespace ublox
}  // namespace tobas
