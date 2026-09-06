// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace tobas
{
namespace ublox
{
class UbxTransport
{
public:
  explicit UbxTransport() = default;
  UbxTransport(UbxTransport&& _other) = delete;
  UbxTransport& operator=(UbxTransport&& _other) = delete;
  UbxTransport(const UbxTransport& _other) = delete;
  UbxTransport& operator=(const UbxTransport& _other) = delete;

  virtual ~UbxTransport() = default;

  virtual bool initialize(const char* _device) noexcept = 0;
  virtual std::optional<uint8_t> receiveByte() noexcept = 0;

  /** @brief Return zero if byte reception does not require pacing. */
  virtual std::chrono::microseconds receiveByteInterval() const noexcept = 0;

  virtual bool send(const uint8_t* _data, size_t _length) noexcept = 0;
};
}  // namespace ublox
}  // namespace tobas
