// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>

#include "tobas_ic_drivers/ublox/ubx_transport_uart.hpp"

using namespace std::chrono_literals;

namespace tobas::ublox
{
namespace
{
constexpr uint32_t kTargetBaudRate = 230400;
constexpr uint32_t kFallbackBaudRate = 38400;
}  // namespace

UbxTransportUart::UbxTransportUart(const char* _device) : device_(_device)
{
}

bool UbxTransportUart::initialize() noexcept
{
  if (!uart_.initialize(device_, true)) {
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

  return configureBaudRate();
}

bool UbxTransportUart::configureBaudRate() noexcept
{
  // Start by checking the target baudrate.
  if (!uart_.setBaudRate(kTargetBaudRate)) {
    return false;
  }

  // Allow UART reads to time out during baudrate detection.
  if (!uart_.setMinimumChars(0) || !uart_.setTimeout(1)) {
    return false;
  }

  // Keep the current baudrate if communication already works at 230400.
  if (!checkBaudRate(kTargetBaudRate)) {
    // Try the fallback baudrate if 230400 does not respond.
    if (!uart_.setBaudRate(kFallbackBaudRate) || !checkBaudRate(kFallbackBaudRate)) {
      return false;
    }

    // Set UART1 baudrate to 230400 in RAM using UBX-CFG-VALSET.
    constexpr std::array<uint8_t, 20> kSet230400 = { 0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x00, 0x01, 0x00, 0x00,
                                                     0x01, 0x00, 0x52, 0x40, 0x00, 0x84, 0x03, 0x00, 0xB7, 0xEF };

    // setBaudRate() waits until pending transmission is complete.
    if (!uart_.send(kSet230400.data(), kSet230400.size()) || !uart_.setBaudRate(kTargetBaudRate)) {
      return false;
    }

    // Wait for the receiver to apply the new baudrate.
    std::this_thread::sleep_for(100ms);

    // Confirm communication at 230400 after switching.
    if (!checkBaudRate(kTargetBaudRate)) {
      return false;
    }
  }

  // Restore blocking reception for normal operation.
  return uart_.setTimeout(0) && uart_.setMinimumChars(1);
}

bool UbxTransportUart::checkBaudRate(uint32_t _baud_rate) noexcept
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

  const auto& expected = _baud_rate == kTargetBaudRate ? kReply230400 : kReply38400;

  if (!uart_.send(kQuery.data(), kQuery.size())) {
    return false;
  }

  // Keep the latest 20 bytes to search for the CFG-VALGET reply.
  std::array<uint8_t, 20> received{};

  // Track the reply and ACK independently because their order may vary.
  bool reply_received = false;
  bool ack_received = false;

  // Stop waiting after one second.
  const auto deadline = std::chrono::steady_clock::now() + 1s;

  while (std::chrono::steady_clock::now() < deadline) {
    const auto byte = uart_.receiveByte();

    // Continue checking when a UART read times out.
    if (!byte) {
      continue;
    }

    // Shift the receive window left and append the newest byte.
    // This finds the reply even if other bytes arrive first.
    std::rotate(received.begin(), received.begin() + 1, received.end());
    received.back() = *byte;

    // Check whether the expected 20-byte CFG-VALGET reply arrived.
    if (received == expected) {
      reply_received = true;
    }

    // Check whether the latest 10 bytes contain the CFG-VALGET ACK.
    if (std::equal(kAck.begin(), kAck.end(), received.end() - 10)) {
      ack_received = true;
    }

    // Consume both messages before starting another command.
    // An unread ACK could otherwise interfere with later configuration.
    if (reply_received && ack_received) {
      return true;
    }
  }

  return false;
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
