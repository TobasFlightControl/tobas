// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <expected>

namespace tobas
{
namespace ublox
{
class UbxTransport
{
public:
  enum class ReceiveError
  {
    kNoData,
    kDeviceError,
  };

  using ReceiveResult = std::expected<uint8_t, ReceiveError>;

  explicit UbxTransport() = default;
  UbxTransport(UbxTransport&& _other) = delete;
  UbxTransport& operator=(UbxTransport&& _other) = delete;
  UbxTransport(const UbxTransport& _other) = delete;
  UbxTransport& operator=(const UbxTransport& _other) = delete;

  virtual ~UbxTransport() = default;

  virtual bool initialize(const char* _device) noexcept = 0;

  /**
   * @brief Receive one byte.
   *
   * If `_nonblock` is `true`, return `kNoData` without waiting when no byte is available.
   * Otherwise, the transport may wait until a byte is received or a device error occurs.
   * `kNoData` is temporary; `kDeviceError` indicates a communication failure.
   */
  virtual ReceiveResult receiveByte(bool _nonblock) noexcept = 0;

  /** @brief Return zero if byte reception does not require pacing. */
  virtual std::chrono::microseconds receiveByteInterval() const noexcept = 0;

  virtual bool send(const uint8_t* _data, size_t _length) noexcept = 0;
};
}  // namespace ublox
}  // namespace tobas
