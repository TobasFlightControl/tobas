#include <iostream>

#include "../include/tobas_ic_drivers/ublox/ubx_scanner.hpp"

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
  state_ = Sync1;
}

bool UBXScanner::update(const uint8_t& data)
{
  if (state_ != Done)
    buffer_[pos_++] = data;

  switch (state_)
  {
    case Sync1:
      if (data == kUbxSync1)
        state_ = Sync2;
      else
        reset();
      break;

    case Sync2:
      switch (data)
      {
        case kUbxSync1:
          state_ = Sync1;
          break;
        case kUbxSync2:
          state_ = Class;
          break;
        default:
          reset();
          break;
      }
      break;

    case Class:
      state_ = ID;
      break;

    case ID:
      state_ = Length1;
      break;

    case Length1:
      payload_length_ = data;
      state_ = Length2;
      break;

    case Length2:
      payload_length_ += data << 8;
      if (messageLength() > kUbxBufferLength)
      {
        cerr << "The size of payload is larger than that of UBX buffer." << endl;
        return false;
      }
      state_ = Payload;
      break;

    case Payload:
      if (pos_ == kUbxHeaderLength + payload_length_)
        state_ = CK_A;
      break;

    case CK_A:
      state_ = CK_B;
      break;

    case CK_B:
      state_ = Done;
      break;

    case Done:
      break;

    default:
      throw;
  }

  return true;
}
}  // namespace ublox
