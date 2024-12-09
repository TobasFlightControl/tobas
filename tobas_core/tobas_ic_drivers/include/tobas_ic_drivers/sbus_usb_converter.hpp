#pragma once

#include <thread>
#include <functional>

#include <tobas_linux/uart_dev.hpp>

#include "./sbus.hpp"

namespace driver
{
/* https://ja.aliexpress.com/i/3256803780013401.html */
class SbusToUsbConverter
{
private:
  static constexpr uint32_t kBaudRate = 115200;  // [bps]
  static constexpr size_t kDataBits = 8;
  static constexpr size_t kDataSize = 32;

  static constexpr size_t kStartIdx = 0;
  static constexpr size_t kDataIdx = kStartIdx + 1;
  static constexpr size_t kFlagsIdx = kDataIdx + kDataSize;
  static constexpr size_t kXORIdx = kFlagsIdx + 1;
  static constexpr size_t kPacketSize = kXORIdx + 1;

public:
  explicit SbusToUsbConverter(std::function<void(const SBUS::Packet&)> packet_cb);

  bool initialize(const char* device);
  void start();
  void spin();

  inline const SBUS::Packet& packet() const;

private:
  const std::function<void(const SBUS::Packet&)> packet_cb_;

  linux::UARTdev uart_;
  uint8_t buf_[kPacketSize];
  SBUS::Packet packet_;

  std::thread read_thread_;
  void readThreadFunc();

  void decodeData();
  void decodeFlags();
};

inline const SBUS::Packet& SbusToUsbConverter::packet() const
{
  return packet_;
}
}  // namespace driver
