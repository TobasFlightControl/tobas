// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

#include <tobas_linux/spi_dev.hpp>

#include "./ubx_transport.hpp"

namespace tobas
{
namespace ublox
{
class UbxTransportSpi final : public UbxTransport
{
public:
  explicit UbxTransportSpi();
  UbxTransportSpi(UbxTransportSpi&& _other) = delete;
  UbxTransportSpi& operator=(UbxTransportSpi&& _other) = delete;
  UbxTransportSpi(const UbxTransportSpi& _other) = delete;
  UbxTransportSpi& operator=(const UbxTransportSpi& _other) = delete;

  bool initialize(const char* _device) noexcept override;
  std::optional<uint8_t> receiveByte() noexcept override;
  std::chrono::microseconds receiveByteInterval() const noexcept override;
  bool send(const uint8_t* _data, size_t _length) noexcept override;

private:
  static constexpr uint32_t kSpiClockFreq = 5'500'000;  // Maximum frequency is 5.5MHz.
  static constexpr size_t kSpiBufSize = 256;

  // Interval for receiving one byte over SPI [us].
  // A smaller value reduces communication latency,
  // but too small a value overloads the receiver with requests and degrades accuracy.
  static constexpr auto kReceiveByteInterval = std::chrono::microseconds(50);

  linux::SPIdev spi_;
  uint8_t tx_buf_[kSpiBufSize];
  uint8_t rx_buf_[kSpiBufSize];
};
}  // namespace ublox
}  // namespace tobas
