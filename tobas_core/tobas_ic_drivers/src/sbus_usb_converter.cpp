#include <iostream>
#include <thread>
#include <set>
#include <boost/multiprecision/cpp_int.hpp>

#include "../include/tobas_ic_drivers/sbus_usb_converter.hpp"

#define TIMEOUT_MS 1000

using namespace std;

namespace driver
{
SbusToUsbConverter::SbusToUsbConverter(function<void(const SBUS::Packet&)> packet_cb) : packet_cb_(packet_cb)
{
}

bool SbusToUsbConverter::initialize(const char* device)
{
  if (!uart_.initialize(device, true))
    return false;

  if (!uart_.setBaudRate(kBaudRate))
    return false;

  if (!uart_.setDataBits(kDataBits))
    return false;

  if (!uart_.setSingleStopBit())
    return false;

  if (!uart_.disableParity())
    return false;

  if (!uart_.setTimeout(TIMEOUT_MS / 100))
    return false;

  return true;
}

void SbusToUsbConverter::spin()
{
  read_thread_.join();
}

void SbusToUsbConverter::start()
{
  read_thread_ = thread(bind(&SbusToUsbConverter::readThreadFunc, this));
}

void SbusToUsbConverter::readThreadFunc()
{
  uint8_t xor_value;

  while (true)
  {
    // Start byte
    if (!uart_.receive(&buf_[kStartIdx], 1))
      continue;
    if (buf_[kStartIdx] != 0x0F)
      continue;

    // Data + Flags
    xor_value = 0;
    for (size_t i = kDataIdx; i < kXORIdx; ++i)
    {
      if (!uart_.receive(&buf_[i], 1))
        continue;
      xor_value ^= buf_[i];
    }

    // XOR
    if (!uart_.receive(&buf_[kXORIdx], 1))
      continue;
    if (xor_value != buf_[kXORIdx])
    {
      cerr << "XOR check failed: " << (int)xor_value << " != " << (int)buf_[kXORIdx] << endl;
      continue;
    }

    // Decode packet
    decodeData();
    decodeFlags();

    // Call user callback
    packet_cb_(packet_);
  }
}

void SbusToUsbConverter::decodeData()
{
  for (size_t ch = 0; ch < SBUS::kChannelSize; ++ch)
  {
    const auto idx = kDataIdx + ch * 2;
    packet_.periods[ch] = ((buf_[idx]) << 8) | buf_[idx + 1];
  }
}

void SbusToUsbConverter::decodeFlags()
{
  const auto flags = buf_[kFlagsIdx];
  packet_.ch17 = (flags >> 0) & 1;
  packet_.ch18 = (flags >> 1) & 1;
  packet_.frame_lost = (flags >> 2) & 1;
  packet_.failsave_activated = (flags >> 3) & 1;
}
}  // namespace driver
