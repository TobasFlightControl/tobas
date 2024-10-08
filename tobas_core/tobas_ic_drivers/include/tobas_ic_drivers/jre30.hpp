#pragma once

#include <thread>
#include <functional>

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/uart_dev.hpp>

namespace driver
{
class JRE30
{
  static constexpr size_t kHeaderIdx = 0;
  static constexpr size_t kProtocolVersionIdx = kHeaderIdx + 2;
  static constexpr size_t kFrameCountIdx = kProtocolVersionIdx + 1;
  static constexpr size_t kDistanceIdx = kFrameCountIdx + 1;
  static constexpr size_t kReservedIdx = kDistanceIdx + 2;
  static constexpr size_t kStrengthIdx = kReservedIdx + 4;
  static constexpr size_t kStatusIdx = kStrengthIdx + 2;
  static constexpr size_t kCRCIdx = kStatusIdx + 2;
  static constexpr size_t kDataSize = kCRCIdx + 2;

public:
  struct Packet
  {
    uint8_t protocol_version;
    uint8_t frame_count;
    double distance;  // [m]
    double strength;  // [-]

    // Status
    bool gain;  // false: Low, true: High
    bool ntrk;  // If true, the measured data is invalid.
    bool fail;
  };

  explicit JRE30(std::function<void(const Packet&)> packet_cb);

  bool initialize(const char* device);
  void spin();

private:
  const std::function<void(const Packet&)> packet_cb_;

  linux::UARTdev uart_;
  uint8_t buf_[kDataSize];
  Packet packet_;
  algo::CRC16Left crc_;

  std::thread read_thread_;
  void readThreadFunc();

  bool read();
  bool checkCRC();
  void decode();
};
}  // namespace driver
