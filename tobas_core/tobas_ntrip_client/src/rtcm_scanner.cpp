#include "tobas_ntrip_client/rtcm_scanner.hpp"

RtcmScanner::RtcmScanner()
{
  reset();
}

void RtcmScanner::reset()
{
  pos_ = 0;
  state_ = kPreamble;
}

void RtcmScanner::update(const uint8_t& data)
{
  if (state_ != kDone) {
    buffer_[pos_++] = data;
  }

  switch (state_) {
    case kPreamble:
      if (data == kRtcmPreamble) {
        state_ = kLength1;
      }
      else {
        reset();
      }
      break;

    case kLength1:
      if (data > 0b0000'0100) {  // 下位2byte分のみがlengthを示しているはずだからdataが4を超えることはありえない
        reset();
      }
      payload_length_ = data << 8;
      state_ = kLength2;
      break;

    case kLength2:
      payload_length_ += data;
      state_ = kPayload;
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

    default:
      throw;
  }
}

bool RtcmScanner::checkSum() const
{
  // calculate CRC24Q checksum
  uint32_t crc = 0;

  for (size_t i = 0; i < messageLength() - kRtcmCheckSumLength; i++) {
    crc ^= (uint32_t)buffer_[i] << 16;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x800000) {
        crc = (crc << 1) ^ 0x1864CFB;
      }
      else {
        crc <<= 1;
      }
      crc &= 0xFFFFFF;
    }
  }

  // checksum
  uint32_t checksum =
    buffer_[messageLength() - 3] << 16 | buffer_[messageLength() - 2] << 8 | buffer_[messageLength() - 1];

  return (crc == checksum);
}
