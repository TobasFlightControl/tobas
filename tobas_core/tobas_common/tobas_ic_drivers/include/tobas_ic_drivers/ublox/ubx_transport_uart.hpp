// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include <tobas_linux/uart_dev.hpp>

#include "tobas_ic_drivers/ublox/ubx_transport.hpp"

namespace tobas::ublox
{
class UbxTransportUart final : public UbxTransport
{
public:
  explicit UbxTransportUart(const char* _device);

  UbxTransportUart(UbxTransportUart&& _other) = delete;
  UbxTransportUart& operator=(UbxTransportUart&& _other) = delete;
  UbxTransportUart(const UbxTransportUart& _other) = delete;
  UbxTransportUart& operator=(const UbxTransportUart& _other) = delete;

  bool initialize() noexcept override;
  Interface interface() const noexcept override
  {
    return Interface::UART;
  }

private:
  std::optional<uint8_t> receiveByte() noexcept override;
  bool writeBytes(const uint8_t* _data, size_t _length) noexcept override;

  const char* device_;

  linux::UARTdev uart_;
};
}  // namespace tobas::ublox
