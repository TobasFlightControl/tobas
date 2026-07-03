// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sbus_driver/sbus.hpp"

#include <iostream>

#include <boost/multiprecision/cpp_int.hpp>

#include <tobas_std_tools/console.hpp>

#define HEX_STREAM(c) "0x" << std::setw(2) << std::setfill('0') << std::hex << std::uppercase << (int)c << std::dec

using namespace boost::multiprecision;

namespace tobas
{
namespace
{
bool isEndByte(uint8_t byte)
{
  switch (byte) {
    case 0x00:
    case 0x04:
    case 0x14:
    case 0x24:
    case 0x34:
      return true;
    default:
      return false;
  }
}
}  // namespace

SBUS::SBUS(std::function<void(const Packet&)> packet_cb) : packet_cb_(packet_cb)
{
}

SBUS::~SBUS()
{
  stop();
}

bool SBUS::initialize(const char* device)
{
  if (!uart_.initialize(device, true)) {
    return false;
  }

  if (!uart_.setBaudRate(kBaudRate)) {
    return false;
  }

  if (!uart_.setDataBits(kDataBits)) {
    return false;
  }

  if (!uart_.setDoubleStopBit()) {
    return false;
  }

  if (!uart_.enableParity(linux::UARTdev::kEven)) {
    return false;
  }

  return true;
}

void SBUS::start()
{
  read_thread_ = std::jthread(std::bind(&SBUS::readThreadFunc, this, std::placeholders::_1));
}

void SBUS::stop()
{
  read_thread_.request_stop();
}

void SBUS::spin()
{
  read_thread_.join();
}

void SBUS::readThreadFunc(std::stop_token st)
{
  uint8_t start_byte, end_byte, flags;
  std::array<uint8_t, kDataSize> data;

  // Receive one byte at a time instead of in bulk.
  // The inverter or the Linux UART device may split the data unexpectedly.
  while (!st.stop_requested()) {
    // Start byte
    if (!uart_.receive(&start_byte, 1)) {
      continue;
    }
    PRINT_DEBUG("Start byte: " << HEX_STREAM(start_byte));
    if (start_byte != 0x0F) {
      continue;
    }

    // Data
    for (size_t i = 0; i < kDataSize; ++i) {
      if (!uart_.receive(&data[i], 1)) {
        continue;
      }
      PRINT_DEBUG("Data byte " << i + 1 << ": " << HEX_STREAM(data[i]));
    }

    // Flags
    if (!uart_.receive(&flags, 1)) {
      continue;
    }
    PRINT_DEBUG("Flags byte: " << HEX_STREAM(flags));

    // End byte
    if (!uart_.receive(&end_byte, 1)) {
      continue;
    }
    PRINT_DEBUG("End byte: " << HEX_STREAM(end_byte));
    if (!isEndByte(end_byte)) {
      std::cerr << "Invalid end byte: " << HEX_STREAM(end_byte) << std::endl;
      continue;
    }

    // Decode packet
    decodeData(data);
    decodeFlags(flags);

    // Call user callback
    packet_cb_(packet_);
  }
}

void SBUS::decodeData(const std::array<uint8_t, kDataSize>& data)
{
  // Convert the data to one bit sequence first because carrying across bytes is cumbersome.
  uint256_t bits = 0;
  for (size_t idx = 0; idx < kDataSize; ++idx) {
    bits |= (static_cast<uint256_t>(data.at(idx)) << (kDataBits * idx));
  }

  // Extract 11 bits at a time.
  constexpr uint16_t kMask = (1 << kChannelBits) - 1;
  for (size_t ch = 0; ch < kChannelSize; ++ch) {
    packet_.periods.at(ch) = ((bits >> (kChannelBits * ch)) & kMask).convert_to<uint16_t>();
  }
}

void SBUS::decodeFlags(uint8_t flags)
{
  packet_.ch17 = (flags >> 0) & 1;
  packet_.ch18 = (flags >> 1) & 1;
  packet_.frame_lost = (flags >> 2) & 1;
  packet_.failsafe = (flags >> 3) & 1;
}
}  // namespace tobas
