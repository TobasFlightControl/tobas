#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace a1
{
class DShot
{
public:
  static constexpr uint16_t kMinThrottle = 48;
  static constexpr uint16_t kMaxThrottle = 2047;
  static constexpr size_t kChannelSize = 8;

  enum command_t : uint16_t
  {
    DISARM = 0,
  };

private:
  static constexpr size_t kChannelBytes = 2;                           // 1チャネルあたりのバイト数
  static constexpr size_t kSpiBufSize = kChannelSize * kChannelBytes;  // SPIバッファのサイズ
  static constexpr uint32_t kSpiClockFreq = 50'000'000;                // [Hz] F722のSPI1の最大値
  static constexpr uint16_t kThrottleMask = (1 << 11) - 1;
  static constexpr uint16_t kDShotDisableCommand = 41;  // Not yet assigned

public:
  explicit DShot();

  bool initialize();

  bool setThrottle(size_t ch, uint16_t throttle, bool telem = false);
  bool setDisabled(size_t ch);

  bool transfer();

private:
  linux::SPIdev spi_;

  bool setData(size_t ch, uint16_t data);
};
}  // namespace a1
