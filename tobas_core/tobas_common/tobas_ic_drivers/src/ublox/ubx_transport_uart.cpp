// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_transport_uart.hpp"

#include <thread>

using namespace std::chrono_literals;

namespace tobas::ublox
{
UbxTransportUart::UbxTransportUart(const char* _device) : device_(_device)
{
}

bool UbxTransportUart::initialize() noexcept
try {
  if (
    !uart_.initialize(device_, false) || !uart_.setBaudRate(38400) || !uart_.setDataBits(8) ||
    !uart_.setSingleStopBit() || !uart_.disableParity()) {
    return false;
  }

  const auto version = pollVersion(1s);
  if (!version) {
    return uart_.setBaudRate(230400) && pollVersion(1s).has_value();
  }

  // CFG-VALSET, version 0, RAM only; CFG-UART1-BAUDRATE (U4) = 230400.
  // Both the key and value use UBX little-endian byte order.
  constexpr uint8_t kBaudConfig[] = { 0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x00, 0x84, 0x03, 0x00 };
  if (!sendMessage(0x06, 0x8A, kBaudConfig, sizeof(kBaudConfig)) || !uart_.drainOutput() || !uart_.setBaudRate(230400)) {
    return false;
  }

  std::this_thread::sleep_for(100ms);
  const auto confirmed_version = pollVersion(1s);
  return confirmed_version && *confirmed_version == *version;
}
catch (...) {
  // Scanner and MON-VER decoding may allocate. Initialization must remain retryable.
  return false;
}

std::optional<uint8_t> UbxTransportUart::receiveByte() noexcept
try {
  return uart_.receiveByte();
}
catch (...) {
  return std::nullopt;
}

bool UbxTransportUart::writeBytes(const uint8_t* _data, size_t _length) noexcept
try {
  return uart_.send(_data, _length);
}
catch (...) {
  return false;
}

}  // namespace tobas::ublox
