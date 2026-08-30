// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

#include <tobas_linux/spi_dev.hpp>
#include <tobas_time_tools/rate.hpp>

#include "./ubx_transport.hpp"

namespace tobas
{
namespace ublox
{
class UBXSPITransport final : public UBXTransport
{
private:
  static constexpr uint32_t kSpiClockFreq = 5'500'000;  // Maximum frequency is 5.5MHz.
  static constexpr size_t kSpiBufSize = 256;

  // Interval for receiving one byte over SPI [us].
  // A smaller value reduces communication latency,
  // but too small a value overloads the receiver with requests and degrades accuracy.
  static constexpr auto kReqInterval = std::chrono::microseconds(50);

public:
  explicit UBXSPITransport();
  UBXSPITransport(const UBXSPITransport&) = delete;
  UBXSPITransport& operator=(const UBXSPITransport&) = delete;
  UBXSPITransport(UBXSPITransport&&) = delete;
  UBXSPITransport& operator=(UBXSPITransport&&) = delete;

  bool initialize(const char* device) override;
  void startReceive() override;
  bool receiveByte(uint8_t& data) override;
  void waitReceiveInterval() override;
  bool send(const uint8_t* data, size_t length) override;

private:
  linux::SPIdev spi_;
  uint8_t tx_buf_[kSpiBufSize];
  uint8_t rx_buf_[kSpiBufSize];
  tim::Rate rate_;
};
}  // namespace ublox
}  // namespace tobas
