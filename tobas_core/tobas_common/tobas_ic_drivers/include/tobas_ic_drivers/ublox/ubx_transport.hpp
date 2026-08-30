// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <cstdint>

namespace tobas
{
namespace ublox
{
class UBXTransport
{
public:
  virtual ~UBXTransport() = default;

  virtual bool initialize(const char* device) = 0;
  virtual void startReceive() = 0;
  virtual bool receiveByte(uint8_t& data) = 0;
  virtual void waitReceiveInterval() = 0;
  virtual bool send(const uint8_t* data, size_t length) = 0;
};
}  // namespace ublox
}  // namespace tobas
