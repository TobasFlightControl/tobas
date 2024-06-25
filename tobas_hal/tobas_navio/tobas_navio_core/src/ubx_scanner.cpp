#include <stdexcept>

#include "../include/tobas_navio_core/ubx_scanner.hpp"

using namespace std;

namespace navio
{
UBXScanner::UBXScanner()
{
  reset();
}

void UBXScanner::reset()
{
  position_ = 0;
  state_ = Sync1;
}

int UBXScanner::update(const uint8_t& data)
{
  if (state_ != Done)
    buffer_[position_++] = data;

  switch (state_)
  {
    case Sync1:
      if (data == kUbxSync1)
        state_ = Sync2;
      else
        reset();
      break;

    case Sync2:
      if (data == kUbxSync2)
        state_ = Class;
      else if (data == kUbxSync1)
        state_ = Sync1;
      else
        reset();
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
        throw runtime_error("The size of payload is too large.");
      state_ = Payload;
      break;

    case Payload:
      if (position_ == kUbxHeaderLength + payload_length_)
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

  return state_;
}
}  // namespace navio
