// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cstddef>
#include <cstdint>

namespace tobas::ntrip
{
/**
 * A byte-by-byte scanner for RTCM 3.x frames.
 *
 * A frame is laid out as the preamble, the length (2 bytes), the payload, and the checksum (3 bytes).
 */
class RtcmScanner
{
public:
  enum State
  {
    kPreamble,
    kLength1,
    kLength2,
    kPayload,
    kCheckSum1,
    kCheckSum2,
    kCheckSum3,
    kDone,
  };

  explicit RtcmScanner();

  RtcmScanner(RtcmScanner&& _other) = delete;
  RtcmScanner& operator=(RtcmScanner&& _other) = delete;
  RtcmScanner(const RtcmScanner& _other) = delete;
  RtcmScanner& operator=(const RtcmScanner& _other) = delete;

  void reset();
  void update(uint8_t _data);

  inline State state() const;

  /** Return the length of the current frame, including the header, the payload and the checksum. */
  inline size_t messageLength() const;

  /** Compute the CRC-24Q checksum of the current frame and compare it with the transmitted one. */
  bool checkSum() const;

  // The getters below are usable only once `state()` has reached `kDone`.

  inline const uint8_t* getPreamble() const;
  inline const uint8_t* getLength() const;
  inline const uint8_t* getPayload() const;
  inline const uint8_t* getChecksum() const;

  /** Return a pointer to one past the last byte of the current frame. */
  inline const uint8_t* getEnd() const;

private:
  static constexpr size_t kRtcmPreambleLength = 1;
  static constexpr size_t kRtcmLengthLength = 2;
  static constexpr size_t kRtcmMaxPayloadLength = 1024;
  static constexpr size_t kRtcmCheckSumLength = 3;
  static constexpr size_t kRtcmHeaderLength = kRtcmPreambleLength + kRtcmLengthLength;
  static constexpr size_t kRtcmFixedLength = kRtcmHeaderLength + kRtcmCheckSumLength;
  static constexpr size_t kRtcmBufferLength = kRtcmFixedLength + kRtcmMaxPayloadLength;

  static constexpr uint8_t kRtcmPreamble = 0xd3;

  uint8_t buffer_[kRtcmBufferLength];  ///< Buffer for the RTCM 3.x frame being scanned
  size_t payload_length_;              ///< Payload length of the current frame
  size_t pos_;                         ///< Current offset in `buffer_`
  State state_;                        ///< Current scanner state
};

inline RtcmScanner::State RtcmScanner::state() const
{
  return state_;
}

inline size_t RtcmScanner::messageLength() const
{
  return kRtcmFixedLength + payload_length_;
}

inline const uint8_t* RtcmScanner::getPreamble() const
{
  return buffer_ + pos_ - messageLength();
}

inline const uint8_t* RtcmScanner::getLength() const
{
  return getPreamble() + kRtcmPreambleLength;
}

inline const uint8_t* RtcmScanner::getPayload() const
{
  return getLength() + kRtcmLengthLength;
}

inline const uint8_t* RtcmScanner::getChecksum() const
{
  return getPayload() + payload_length_;
}

inline const uint8_t* RtcmScanner::getEnd() const
{
  return getChecksum() + kRtcmCheckSumLength;
}
}  // namespace tobas::ntrip
