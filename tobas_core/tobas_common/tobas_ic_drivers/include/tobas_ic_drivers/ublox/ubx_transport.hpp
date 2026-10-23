// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <string>

#include "./ubx_scanner.hpp"

#define PACKED __attribute__((__packed__))  // Struct member variables are contiguous in memory.

namespace tobas::ublox
{
class UbxTransport
{
public:
  enum class Interface
  {
    SPI,
    UART,
  };

  explicit UbxTransport() = default;

  UbxTransport(UbxTransport&& _other) = delete;
  UbxTransport& operator=(UbxTransport&& _other) = delete;
  UbxTransport(const UbxTransport& _other) = delete;
  UbxTransport& operator=(const UbxTransport& _other) = delete;

  virtual ~UbxTransport() = default;

  virtual bool initialize() noexcept = 0;
  virtual Interface interface() const noexcept = 0;

  bool sendMessage(uint8_t _cls, uint8_t _id, const void* _msg, uint16_t _size) noexcept;
  bool receiveMessage(UbxScanner& _scanner, bool _blocking, std::chrono::milliseconds _timeout) noexcept;
  std::expected<std::string, std::string> pollVersion(std::chrono::milliseconds _timeout);

protected:
  virtual std::optional<uint8_t> receiveByte() noexcept = 0;
  virtual bool writeBytes(const uint8_t* _data, size_t _length) noexcept = 0;

private:
  struct PACKED UbxHeader
  {
    uint8_t sync1;
    uint8_t sync2;
    uint8_t cls;
    uint8_t id;
    uint16_t length;
  };

  struct PACKED CheckSum
  {
    uint8_t CK_A;
    uint8_t CK_B;
  };

  static CheckSum computeChecksum(const uint8_t* _message, size_t _checksum_pos);
  static size_t spliceMemory(uint8_t* _dest, const void* _src, size_t _size, size_t _dest_offset = 0);
};
}  // namespace tobas::ublox

#undef PACKED
