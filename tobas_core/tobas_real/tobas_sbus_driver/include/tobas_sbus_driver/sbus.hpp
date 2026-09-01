// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <functional>
#include <thread>

#include <tobas_linux/uart_dev.hpp>

namespace tobas
{
/* Read S.BUS through UART after inverting the signal with an inverter. */
class SBUS
{
public:
  static constexpr size_t kChannelSize = 16;

  struct Packet
  {
    std::array<uint16_t, kChannelSize> periods = {};
    bool ch17;
    bool ch18;
    bool frame_lost;
    bool failsafe;
  };

  explicit SBUS(std::function<void(const Packet&)> packet_cb);
  ~SBUS();

  bool initialize(const char* device);

  void start();
  void stop();
  void spin();

  inline const Packet& packet() const;

private:
  static constexpr uint32_t kBaudRate = 100'000;  // [bps]
  static constexpr size_t kChannelBits = 11;
  static constexpr size_t kDataBits = 8;
  static constexpr size_t kDataSize = 22;

  const std::function<void(const Packet&)> packet_cb_;

  linux::UARTdev uart_;
  Packet packet_;

  std::jthread read_thread_;
  void readThreadFunc(std::stop_token st);

  void decodeData(const std::array<uint8_t, kDataSize>& data);
  void decodeFlags(uint8_t flags);
};

inline const SBUS::Packet& SBUS::packet() const
{
  return packet_;
}
}  // namespace tobas
