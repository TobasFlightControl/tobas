// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <cstdint>

namespace tobas
{
namespace ublox
{
static constexpr size_t kUbxSyncLength = 2;
static constexpr size_t kUbxClassLength = 1;
static constexpr size_t kUbxIdLength = 1;
static constexpr size_t kUbxLengthLength = 2;
static constexpr size_t kUbxChecksumLength = 2;
static constexpr size_t kUbxHeaderLength = kUbxSyncLength + kUbxClassLength + kUbxIdLength + kUbxLengthLength;
static constexpr size_t kUbxFixedLength = kUbxHeaderLength + kUbxChecksumLength;

static constexpr uint8_t kUbxSync1 = 0xb5;
static constexpr uint8_t kUbxSync2 = 0x62;

static constexpr size_t kUbxBufferLength = 256;

class UbxScanner
{
public:
  enum State
  {
    kSync1,
    kSync2,
    kClass,
    kId,
    kLength1,
    kLength2,
    kPayload,
    kCkA,
    kCkB,
    kDone,
  };

  explicit UbxScanner();
  UbxScanner(UbxScanner&& _other) = default;
  UbxScanner& operator=(UbxScanner&& _other) = default;
  UbxScanner(const UbxScanner& _other) = default;
  UbxScanner& operator=(const UbxScanner& _other) = default;

  void reset();
  bool update(const uint8_t& data);

  inline State state() const;
  inline size_t messageLength() const;

  inline const uint8_t* getSync1() const;
  inline const uint8_t* getSync2() const;
  inline const uint8_t* getClass() const;
  inline const uint8_t* getId() const;
  inline const uint8_t* getLength() const;
  inline const uint8_t* getPayload() const;
  inline const uint8_t* getChecksumA() const;
  inline const uint8_t* getChecksumB() const;

private:
  uint8_t buffer_[kUbxBufferLength];  // Buffer for UBX message
  size_t payload_length_;             // Length of current message payload
  size_t pos_;                        // Indicates current buffer offset
  State state_;                       // Current scanner state
};

inline UbxScanner::State UbxScanner::state() const
{
  return state_;
}

inline size_t UbxScanner::messageLength() const
{
  return kUbxFixedLength + payload_length_;
}

inline const uint8_t* UbxScanner::getSync1() const
{
  return buffer_ + pos_ - messageLength();
}

inline const uint8_t* UbxScanner::getSync2() const
{
  return getSync1() + 1;
}

inline const uint8_t* UbxScanner::getClass() const
{
  return getSync1() + kUbxSyncLength;
}

inline const uint8_t* UbxScanner::getId() const
{
  return getClass() + kUbxClassLength;
}

inline const uint8_t* UbxScanner::getLength() const
{
  return getId() + kUbxIdLength;
}

inline const uint8_t* UbxScanner::getPayload() const
{
  return getLength() + kUbxLengthLength;
}

inline const uint8_t* UbxScanner::getChecksumA() const
{
  return getPayload() + payload_length_;
}

inline const uint8_t* UbxScanner::getChecksumB() const
{
  return getChecksumA() + 1;
}
}  // namespace ublox
}  // namespace tobas
