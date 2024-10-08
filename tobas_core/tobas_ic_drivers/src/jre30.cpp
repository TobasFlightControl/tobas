#include <iostream>

#include "../include/tobas_ic_drivers/jre30.hpp"

using namespace std;

namespace driver
{
JRE30::JRE30(std::function<void(const Packet&)> packet_cb)
  : packet_cb_(packet_cb), crc_((1 << 12) | (1 << 5) | (1 << 0), 0xFFFF, 0xFFFF)
{
}

bool JRE30::initialize(const char* device)
{
  if (!uart_.initialize(device, true))
    return false;

  if (!uart_.setStandardBaudRate(B460800))
    return false;

  if (!uart_.setDataBits(8))
    return false;

  if (!uart_.setSingleStopBit())
    return false;

  if (!uart_.disableParity())
    return false;

  read_thread_ = thread(bind(&JRE30::readThreadFunc, this));

  return true;
}

void JRE30::spin()
{
  read_thread_.join();
}

void JRE30::readThreadFunc()
{
  while (true)
  {
    if (!read())
      continue;

    if (!checkCRC())
      continue;

    decode();

    packet_cb_(packet_);
  }
}

bool JRE30::read()
{
  // Header
  buf_[kHeaderIdx] = uart_.receiveByte();
  if (buf_[kHeaderIdx] != 'R')
    return false;
  buf_[kHeaderIdx + 1] = uart_.receiveByte();
  if (buf_[kHeaderIdx + 1] != 'A')
    return false;

  // Protocol version
  buf_[kProtocolVersionIdx] = uart_.receiveByte();

  // Frame count
  buf_[kFrameCountIdx] = uart_.receiveByte();

  // Distance
  buf_[kDistanceIdx] = uart_.receiveByte();
  buf_[kDistanceIdx + 1] = uart_.receiveByte();

  // Reserved
  for (size_t i = 0; i < 4; ++i)
    buf_[kReservedIdx + i] = uart_.receiveByte();

  // Strength
  buf_[kStrengthIdx] = uart_.receiveByte();
  buf_[kStrengthIdx + 1] = uart_.receiveByte();

  // Status
  buf_[kStatusIdx] = uart_.receiveByte();
  buf_[kStatusIdx + 1] = uart_.receiveByte();

  // CRC
  buf_[kCRCIdx] = uart_.receiveByte();
  buf_[kCRCIdx + 1] = uart_.receiveByte();

  return true;
}

bool JRE30::checkCRC()
{
  const uint16_t cs = crc_.compute(buf_, kCRCIdx);
  const uint16_t cr = (buf_[kCRCIdx + 1] << 8) | buf_[kCRCIdx];

  if (cs != cr)
  {
    cerr << "CRC failed: " << cs << " != " << cr << endl;
    return false;
  }

  return true;
}

void JRE30::decode()
{
  packet_.protocol_version = buf_[kProtocolVersionIdx];
  packet_.frame_count = buf_[kFrameCountIdx];

  const uint16_t dist_lsb = (buf_[kDistanceIdx + 1] << 8) | buf_[kDistanceIdx];
  packet_.distance = dist_lsb * 0.01;

  const uint16_t strength_lsb = (buf_[kStrengthIdx + 1] << 8) | buf_[kStrengthIdx];
  packet_.strength = strength_lsb * 1.;

  packet_.gain = (buf_[kStatusIdx] >> 0) & 1;
  packet_.ntrk = (buf_[kStatusIdx] >> 1) & 1;
  packet_.fail = (buf_[kStatusIdx] >> 3) & 1;
}
}  // namespace driver
