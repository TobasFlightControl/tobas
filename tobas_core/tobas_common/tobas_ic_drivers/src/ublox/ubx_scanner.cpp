#include "tobas_ic_drivers/ublox/ubx_scanner.hpp"

#include <iostream>

using namespace std;

namespace ublox
{
UBXScanner::UBXScanner()
{
  reset();
}

void UBXScanner::reset()
{
  pos_ = 0;
  state_ = kSync1;
}

bool UBXScanner::update(const uint8_t& data)
{
  if (state_ != kDone) {
    buffer_[pos_++] = data;
  }

  switch (state_) {
    case kSync1:
      if (data == kUbxSync1) {
        state_ = kSync2;
      }
      else {
        reset();
      }
      break;

    case kSync2:
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

    case kClass:
      state_ = kId;
      break;

    case kId:
      state_ = kLength1;
      break;

    case kLength1:
      payload_length_ = data;
      state_ = kLength2;
      break;

    case kLength2:
      payload_length_ += data << 8;
      if (messageLength() > kUbxBufferLength) {
        cerr << "The size of payload is larger than that of UBX buffer." << endl;
        return false;
      }
      state_ = kPayload;
      break;

    case kPayload:
      if (pos_ == kUbxHeaderLength + payload_length_) {
        state_ = kCkA;
      }
      break;

    case kCkA:
      state_ = kCkB;
      break;

    case kCkB:
      state_ = kDone;
      break;

    case kDone:
      break;

    default:
      throw;
  }

  return true;
}
}  // namespace ublox
