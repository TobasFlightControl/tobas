#pragma once

#include <thread>
#include <functional>

#include <tobas_linux/uart_dev.hpp>

namespace aso
{
/**
 * @brief インバータで信号を反転したS.BUSをUARTで読む．
 * cf. [Raspberry Pi PicoのUARTでラジコン受信機の信号を読む](https://rikei-tawamure.com/entry/2021/02/12/130248)
 */
class SBUS
{
public:
  static constexpr size_t kChannelSize = 16;

  struct Packet
  {
    std::array<uint16_t, kChannelSize> periods;
    bool ch17;
    bool ch18;
    bool frame_lost;
    bool failsave_activated;
  };

private:
  static constexpr uint32_t kBaudRate = 100'000;  // [bps]
  static constexpr size_t kChannelBits = 11;
  static constexpr size_t kDataBits = 8;
  static constexpr size_t kDataSize = 22;
  static constexpr size_t kTelemSize = 3;

  static constexpr size_t kStartIdx = 0;
  static constexpr size_t kDataIdx = kStartIdx + 1;
  static constexpr size_t kFlagsIdx = kDataIdx + kDataSize;
  static constexpr size_t kEndIdx = kFlagsIdx + 1;
  static constexpr size_t kPacketSize = kEndIdx + 1;

public:
  explicit SBUS(std::function<void(const Packet&)> packet_cb);

  bool initialize();
  void spin();

private:
  const std::function<void(const Packet&)> packet_cb_;

  linux::UARTdev uart_;
  Packet packet_;

  std::thread read_thread_;
  void readThreadFunc();

  void decodeData(const std::array<uint8_t, kDataSize>& data);
  void decodeFlags(uint8_t flags);
};
}  // namespace aso
