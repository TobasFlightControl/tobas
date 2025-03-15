#pragma once

#include <memory>
#include <thread>
#include <functional>

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/uart_dev.hpp>

namespace driver
{
struct JRE30Packet
{
  virtual size_t packetSize() const = 0;
  virtual void decode(uint8_t* buf) = 0;
};

struct JRE30Packet_A : public JRE30Packet
{
  uint8_t protocol_version;
  uint8_t frame_count;
  double distance;  // [m]
  double strength;  // [-]

  // Status
  bool gain;  // false: Low, true: High
  bool ntrk;  // If true, the measured data is invalid.
  bool fail;

  size_t packetSize() const override;
  void decode(uint8_t* buf) override;
};

struct JRE30Packet_B : public JRE30Packet
{
  // TODO

  size_t packetSize() const override;
  void decode(uint8_t* buf) override;
};

struct JRE30Packet_C : public JRE30Packet
{
  // TODO

  size_t packetSize() const override;
  void decode(uint8_t* buf) override;
};

/**
 * @brief A Linux driver of JRE-30 range sensor.
 *
 * https://github.com/jfbblue0922/JFB_ardupilot/blob/Copter-4.1.5_JFB100_JRE/libraries/AP_RangeFinder/AP_RangeFinder_JRE_Serial.cpp
 */
class JRE30
{
public:
  explicit JRE30(std::function<void(std::shared_ptr<const JRE30Packet>)> packet_cb);

  bool initialize(const char* uart_device);
  void start();
  void spin();

private:
  const std::function<void(std::shared_ptr<const JRE30Packet>)> packet_cb_;

  linux::UARTdev uart_;
  algo::CRC16Right crc_;
  uint8_t buf_[256];

  std::shared_ptr<JRE30Packet> packet_;
  std::shared_ptr<JRE30Packet_A> packet_a_;
  std::shared_ptr<JRE30Packet_B> packet_b_;
  std::shared_ptr<JRE30Packet_C> packet_c_;

  std::thread read_thread_;
  void readThreadFunc();

  bool checkCRC() const;
};
}  // namespace driver
