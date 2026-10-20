// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>

namespace tobas::ublox
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
  virtual std::expected<std::optional<uint8_t>, int> receiveByte() noexcept = 0;
  virtual bool send(const uint8_t* _data, size_t _length) noexcept = 0;
};
}  // namespace tobas::ublox
