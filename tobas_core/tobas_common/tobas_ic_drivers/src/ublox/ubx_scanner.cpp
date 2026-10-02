// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ublox/ubx_scanner.hpp"

namespace tobas
{
namespace ublox
{
UbxScanner::UbxScanner() : buffer_(kUbxHeaderLength)
{
  reset();
}

void UbxScanner::reset()
{
  pos_ = 0;
  state_ = kSync1;
}

void UbxScanner::update(uint8_t data)
{
  if (state_ != kDone) {
    buffer_[pos_++] = data;
  }

  switch (state_) {
    case kSync1: {
      if (data == kUbxSync1) {
        state_ = kSync2;
      }
      else {
        reset();
      }
      break;
    }
    case kSync2: {
      switch (data) {
        case kUbxSync1:
          state_ = kSync1;
          break;
        case kUbxSync2:
          state_ = kClass;
          break;
        default:
          reset();
          break;
      }
      break;
    }
    case kClass: {
      state_ = kId;
      break;
    }
    case kId: {
      state_ = kLength1;
      break;
    }
    case kLength1: {
      payload_length_ = data;
      state_ = kLength2;
      break;
    }
    case kLength2: {
      payload_length_ += data << 8;
      // Grow the buffer to fit the complete message, including its checksum.
      const auto msg_length = messageLength();
      if (msg_length > buffer_.size()) {
        buffer_.resize(msg_length);
      }
      state_ = kPayload;
      break;
    }
    case kPayload: {
      if (pos_ == kUbxHeaderLength + payload_length_) {
        state_ = kCkA;
      }
      break;
    }
    case kCkA: {
      state_ = kCkB;
      break;
    }
    case kCkB: {
      state_ = kDone;
      break;
    }
    case kDone: {
      break;
    }
    default: {
      throw;
    }
  }
}
}  // namespace ublox
}  // namespace tobas
