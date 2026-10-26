// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>

#include "tobas_ic_drivers/ublox/ubx_transport_uart.hpp"

namespace tobas::ublox
{
namespace
{
bool checkBaudRate(linux::UARTdev& _uart, uint32_t _baud_rate)
{
  // Poll the UART1 baudrate using UBX-CFG-VALGET.
  constexpr std::array<uint8_t, 16> kQuery = { 0xB5, 0x62, 0x06, 0x8B, 0x08, 0x00, 0x00, 0x00,
                                               0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x2C, 0x79 };

  constexpr std::array<uint8_t, 20> kReply230400 = { 0xB5, 0x62, 0x06, 0x8B, 0x0C, 0x00, 0x01, 0x00, 0x00, 0x00,
                                                     0x01, 0x00, 0x52, 0x40, 0x00, 0x84, 0x03, 0x00, 0xB8, 0xFF };

  constexpr std::array<uint8_t, 20> kReply38400 = { 0xB5, 0x62, 0x06, 0x8B, 0x0C, 0x00, 0x01, 0x00, 0x00, 0x00,
                                                    0x01, 0x00, 0x52, 0x40, 0x00, 0x96, 0x00, 0x00, 0xC7, 0x2F };

  // ACK for CFG-VALGET.
  constexpr std::array<uint8_t, 10> kAck = { 0xB5, 0x62, 0x05, 0x01, 0x02, 0x00, 0x06, 0x8B, 0x99, 0xC2 };

  const auto& expected = _baud_rate == 230400 ? kReply230400 : kReply38400;

  if (!_uart.send(kQuery.data(), kQuery.size())) {
    return false;
  }

  std::array<uint8_t, 20> received{};
  bool reply_received = false;
  bool ack_received = false;

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);

  while (std::chrono::steady_clock::now() < deadline) {
    const auto byte = _uart.receiveByte();
    if (!byte) {
      continue;
    }

    std::rotate(received.begin(), received.begin() + 1, received.end());
    received.back() = *byte;

    if (received == expected) {
      reply_received = true;
    }

    if (std::equal(kAck.begin(), kAck.end(), received.end() - 10)) {
      ack_received = true;
    }

    if (reply_received && ack_received) {
      return true;
    }
  }

  return false;
}
}  // namespace

UbxTransportUart::UbxTransportUart(const char* _device) : device_(_device)
{
}

bool UbxTransportUart::initialize() noexcept
{
  if (!uart_.initialize(device_, true)) {
    return false;
  }

  if (!uart_.setBaudRate(230400)) {
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

  // Allow baud-rate detection to continue if the receiver does not respond.
  if (!uart_.setMinimumChars(0) || !uart_.setTimeout(1)) {
    return false;
  }

  // Check 230400 first to avoid unnecessary configuration changes.
  if (!checkBaudRate(uart_, 230400)) {
    // Try 38400 if the receiver does not respond.
    if (!uart_.setBaudRate(38400) || !checkBaudRate(uart_, 38400)) {
      return false;
    }

    // Set UART1 to 230400 in RAM using UBX-CFG-VALSET.
    constexpr std::array<uint8_t, 20> kSet230400 = { 0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x00,
                                                     0x01, 0x00, 0x52, 0x40, 0x00, 0x84, 0x03, 0x00, 0xB7, 0xEF };

    // Finish transmission before changing the host baudrate.
    if (!uart_.send(kSet230400.data(), kSet230400.size()) || !uart_.drainOutput() || !uart_.setBaudRate(230400)) {
      return false;
    }

    // Allow the receiver to switch baudrates before verifying.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!checkBaudRate(uart_, 230400)) {
      return false;
    }
  }

  // Restore blocking reception for normal operation.
  return uart_.setTimeout(0) && uart_.setMinimumChars(1);
}

std::optional<uint8_t> UbxTransportUart::receiveByte() noexcept
{
  return uart_.receiveByte();
}

bool UbxTransportUart::send(const uint8_t* _data, size_t _length) noexcept
{
  return uart_.send(_data, _length);
}
}  // namespace tobas::ublox
