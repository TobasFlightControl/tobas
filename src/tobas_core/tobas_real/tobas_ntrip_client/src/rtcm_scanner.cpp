// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ntrip_client/rtcm_scanner.hpp"

namespace tobas::ntrip
{
RtcmScanner::RtcmScanner()
{
  reset();
}

void RtcmScanner::reset()
{
  payload_length_ = 0;
  pos_ = 0;
  state_ = kPreamble;
}

void RtcmScanner::update(uint8_t _data)
{
  if (state_ != kDone) {
    if (pos_ < kRtcmBufferLength) {
      buffer_[pos_++] = _data;
    }
    else {
      reset();
      return;
    }
  }

  switch (state_) {
    case kPreamble:
      if (_data == kRtcmPreamble) {
        state_ = kLength1;
      }
      else {
        reset();
      }
      break;

    case kLength1:
      // RTCM 3.x: Byte 1 has 6 reserved bits (must be 0) and 2 bits of message length
      if ((_data & 0xFC) != 0) {
        reset();
        break;
      }
      payload_length_ = static_cast<size_t>(_data & 0x03) << 8;
      state_ = kLength2;
      break;

    case kLength2:
      payload_length_ += _data;
      if (payload_length_ > kRtcmMaxPayloadLength) {
        reset();
        break;
      }
      if (payload_length_ == 0) {
        state_ = kCheckSum1;
      }
      else {
        state_ = kPayload;
      }
      break;

    case kPayload:
      if (pos_ == kRtcmHeaderLength + payload_length_) {
        state_ = kCheckSum1;
      }
      break;

    case kCheckSum1:
      state_ = kCheckSum2;
      break;

    case kCheckSum2:
      state_ = kCheckSum3;
      break;

    case kCheckSum3:
      state_ = kDone;
      break;

    case kDone:
      break;
  }
}

bool RtcmScanner::checkSum() const
{
  // Calculate the CRC24Q checksum.
  uint32_t crc = 0;
  for (size_t i = 0; i < messageLength() - kRtcmCheckSumLength; i++) {
    crc ^= static_cast<uint32_t>(buffer_[i]) << 16;
    for (int j = 0; j < 8; j++) {
      if ((crc & 0x800000) != 0) {
        crc = (crc << 1) ^ 0x1864CFB;
      }
      else {
        crc <<= 1;
      }
      crc &= 0xFFFFFF;
    }
  }

  const uint32_t checksum = static_cast<uint32_t>(buffer_[messageLength() - 3]) << 16 |
                            static_cast<uint32_t>(buffer_[messageLength() - 2]) << 8 |
                            static_cast<uint32_t>(buffer_[messageLength() - 1]);

  return crc == checksum;
}
}  // namespace tobas::ntrip
