#include <iostream>

#include "../include/tobas_ic_drivers/jre30.hpp"

#define TIMEOUT_MS 1000

using namespace std;

namespace driver
{
size_t JRE30Packet_A::packetSize() const
{
  return 16;
}

void JRE30Packet_A::decode(uint8_t* buf)
{
  protocol_version = buf[2];
  frame_count = buf[3];

  const uint16_t dist_lsb = (buf[4] << 8) | buf[5];
  distance = dist_lsb * 0.01;

  const uint16_t strength_lsb = (buf[10] << 8) | buf[11];
  strength = strength_lsb * 1.;

  const uint16_t status = (buf[12] << 8) | buf[13];
  gain = (status >> 0) & 1;
  ntrk = (status >> 1) & 1;
  fail = (status >> 3) & 1;
}

size_t JRE30Packet_B::packetSize() const
{
  return 32;
}

void JRE30Packet_B::decode(uint8_t* buf)
{
  (void)buf;
  // TODO
}

size_t JRE30Packet_C::packetSize() const
{
  return 48;
}

void JRE30Packet_C::decode(uint8_t* buf)
{
  (void)buf;
  // TODO
}

JRE30::JRE30(function<void(shared_ptr<const JRE30Packet>)> packet_cb)
  : packet_cb_(packet_cb), crc_(algo::CRC16Right::CRC_16_CCITT, 0xFFFF, 0xFFFF)
{
  crc_.initialize();

  packet_a_ = make_shared<JRE30Packet_A>();
  packet_b_ = make_shared<JRE30Packet_B>();
  packet_c_ = make_shared<JRE30Packet_C>();
}

bool JRE30::initialize(const char* uart_device)
{
  if (!uart_.initialize(uart_device, true))
    return false;

  if (!uart_.setBaudRate(460'800))
    return false;

  if (!uart_.setDataBits(8))
    return false;

  if (!uart_.setSingleStopBit())
    return false;

  if (!uart_.disableParity())
    return false;

  if (!uart_.setTimeout(TIMEOUT_MS / 100))
    return false;

  return true;
}

void JRE30::start()
{
  read_thread_ = thread(bind(&JRE30::readThreadFunc, this));
}

void JRE30::spin()
{
  read_thread_.join();
}

void JRE30::readThreadFunc()
{
  while (true)
  {
    // Header
    if (!uart_.receive(buf_ + 0, 1))
      continue;
    if (buf_[0] != 'R')
      continue;

    if (!uart_.receive(buf_ + 1, 1))
      continue;

    switch (buf_[1])
    {
      case 'A':
        packet_ = static_pointer_cast<JRE30Packet>(packet_a_);
        break;
      case 'B':
        packet_ = static_pointer_cast<JRE30Packet>(packet_b_);
        break;
      case 'C':
        packet_ = static_pointer_cast<JRE30Packet>(packet_c_);
        break;
      default:
        continue;
    }

    for (size_t i = 2; i < packet_->packetSize(); ++i)
      if (!uart_.receive(buf_ + i, 1))
        continue;

    if (!checkCRC())
      continue;

    packet_->decode(buf_);
    packet_cb_(packet_);
  }
}

bool JRE30::checkCRC() const
{
  const auto packet_size = packet_->packetSize();

  const uint16_t cs = crc_.compute(buf_, packet_size - 2);
  const uint16_t cr = (buf_[packet_size - 1] << 8) | buf_[packet_size - 2];

  if (cs != cr)
  {
    cerr << "CRC failed: " << cs << " != " << cr << endl;
    return false;
  }

  return true;
}
}  // namespace driver
