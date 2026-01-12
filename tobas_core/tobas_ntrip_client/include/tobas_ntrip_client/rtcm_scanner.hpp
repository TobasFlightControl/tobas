#pragma once

#include <cstddef>
#include <cstdint>

// Scan RTCM3.x protocol data
// Data structure : preamble, length1, length2, payload, checksum1, checksum2, checksum3
class RtcmScanner
{
public:
  enum State : uint8_t
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

  RtcmScanner();

  void reset();
  void update(const uint8_t& data);

  inline State state() const;
  // header + payload + checksumのデータ長を返す
  inline size_t messageLength() const;
  // checksumを計算して，合っているか確認する
  bool checkSum() const;

  // preambleを表すデータのpointerを返す stateがkDoneにならないと使用できない
  inline const uint8_t* getPreamble() const;
  // lengthを表すデータのpointerを返す stateがkDoneにならないと使用できない
  inline const uint8_t* getLength() const;
  // payloadを表すデータのpointerを返す stateがkDoneにならないと使用できない
  inline const uint8_t* getPayload() const;
  // checksumを表すデータのpointerを返す stateがkDoneにならないと使用できない
  inline const uint8_t* getChecksum() const;
  // RTCM messageの終端を表すデータのpointerを返す stateがkDoneにならないと使用できない
  inline const uint8_t* getEnd() const;

private:
  static constexpr size_t kRtcmPreambleLength = 1;
  static constexpr size_t kRtcmLengthLength = 2;
  static constexpr size_t kRtcmMaxPayloadLength = 1024;
  static constexpr size_t kRtcmCheckSumLength = 3;
  static constexpr size_t kRtcmHeaderLength = kRtcmPreambleLength + kRtcmLengthLength;
  static constexpr size_t kRtcmFixedLength = kRtcmHeaderLength + kRtcmCheckSumLength;
  static constexpr size_t kRtcmBufferLength =
    kRtcmPreambleLength + kRtcmLengthLength + kRtcmMaxPayloadLength + kRtcmCheckSumLength;

  static constexpr size_t kRtcmPreamble = 0xd3;

  uint8_t buffer_[kRtcmBufferLength];  // Buffer for RTCM 3.x message
  size_t payload_length_;              // Length of current message payload
  size_t pos_;                         // Indicates current buffer offset
  State state_;                        // Current scanner state
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
