#pragma once

#include <cstdint>
#include <cstddef>

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

class UBXScanner
{
public:
  enum state_t : uint8_t
  {
    Sync1,
    Sync2,
    Class,
    ID,
    Length1,
    Length2,
    Payload,
    CK_A,
    CK_B,
    Done,
  };

  explicit UBXScanner();

  void reset();
  bool update(const uint8_t& data);

  inline state_t state() const;
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
  state_t state_;                     // Current scanner state
};

inline UBXScanner::state_t UBXScanner::state() const
{
  return state_;
}

inline size_t UBXScanner::messageLength() const
{
  return kUbxFixedLength + payload_length_;
}

inline const uint8_t* UBXScanner::getSync1() const
{
  return buffer_ + pos_ - messageLength();
}

inline const uint8_t* UBXScanner::getSync2() const
{
  return getSync1() + 1;
}

inline const uint8_t* UBXScanner::getClass() const
{
  return getSync1() + kUbxSyncLength;
}

inline const uint8_t* UBXScanner::getId() const
{
  return getClass() + kUbxClassLength;
}

inline const uint8_t* UBXScanner::getLength() const
{
  return getId() + kUbxIdLength;
}

inline const uint8_t* UBXScanner::getPayload() const
{
  return getLength() + kUbxLengthLength;
}

inline const uint8_t* UBXScanner::getChecksumA() const
{
  return getPayload() + payload_length_;
}

inline const uint8_t* UBXScanner::getChecksumB() const
{
  return getChecksumA() + 1;
}
}  // namespace ublox
