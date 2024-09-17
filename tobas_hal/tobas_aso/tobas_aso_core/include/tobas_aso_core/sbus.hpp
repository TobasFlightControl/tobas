#pragma once

#include <array>

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
  explicit SBUS();

  bool initialize();

  /* Read a S.BUS message from the receiver and update the periods of all the 16 channels. */
  bool update();

  inline const std::array<uint16_t, kChannelSize> getPeriods() const;
  inline const uint16_t& getPeriod(uint8_t channel) const;
  inline const bool& channel17() const;
  inline const bool& channel18() const;
  inline const bool& frameLost() const;
  inline const bool& failsaveActivated() const;

private:
  linux::UARTdev uart_dev_;
  std::array<uint8_t, kPacketSize> packet_;
  bool telem_ready_ = false;

  struct Output
  {
    std::array<uint16_t, kChannelSize> periods;
    bool ch17;
    bool ch18;
    bool frame_lost;
    bool failsave_activated;
  } out_;

  bool read();
  void decodeData();
  void decodeFlags();
};

inline const std::array<uint16_t, SBUS::kChannelSize> SBUS::getPeriods() const
{
  return out_.periods;
}

inline const uint16_t& SBUS::getPeriod(uint8_t channel) const
{
  return out_.periods.at(channel);
}

inline const bool& SBUS::channel17() const
{
  return out_.ch17;
}

inline const bool& SBUS::channel18() const
{
  return out_.ch18;
}

inline const bool& SBUS::frameLost() const
{
  return out_.frame_lost;
}

inline const bool& SBUS::failsaveActivated() const
{
  return out_.failsave_activated;
}
}  // namespace aso
