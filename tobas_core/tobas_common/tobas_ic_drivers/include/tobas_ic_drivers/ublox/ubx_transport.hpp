// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

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

  virtual bool initialize() noexcept = 0;
  virtual std::optional<uint8_t> receiveByte() noexcept = 0;
  virtual bool send(const uint8_t* _data, size_t _length) noexcept = 0;

  /**
   * Queue data to be sent to the receiver without interrupting the reception of UBX messages.
   * The data may be sent later, e.g. byte by byte in `receiveByte()`.
   */
  virtual bool enqueue(const uint8_t* _data, size_t _length) noexcept = 0;

  /** Return true if queued data remains to be sent. */
  virtual bool hasPendingData() const noexcept = 0;
};
}  // namespace ublox
}  // namespace tobas
