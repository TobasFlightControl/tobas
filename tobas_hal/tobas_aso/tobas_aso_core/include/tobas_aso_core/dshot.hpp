#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace aso
{
class DShot
{
public:
  static constexpr uint16_t kMinThrot = 48;
  static constexpr uint16_t kMaxThrot = 2047;
  static constexpr size_t kChannelSize = 8;

  enum command_t : uint16_t
  {
    DISARM = 0,
  };

private:
  static constexpr size_t kChannelBytes = 2;                           // 1チャネルあたりのバイト数
  static constexpr size_t kSpiBufSize = kChannelSize * kChannelBytes;  // SPIバッファのサイズ
  static constexpr uint32_t kSpiClockFreq = 10'000'000;  // [Hz] 最大は50MHzだが，高すぎると文字化けする
  static constexpr uint16_t kDShotDisableCommand = 41;   // Not yet assigned

public:
  explicit DShot();

  bool initialize();

  bool setThrottle(size_t ch, uint16_t throttle);
  bool setDisabled(size_t ch);

  bool transfer();

private:
  linux::SPIdev spi_;
};
}  // namespace aso
