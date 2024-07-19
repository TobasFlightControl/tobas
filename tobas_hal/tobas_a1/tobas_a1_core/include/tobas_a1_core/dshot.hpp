#pragma once

#include <cstddef>

#include <tobas_linux/spi_dev.hpp>

namespace a1
{
class DShot
{
public:
  static constexpr size_t kChannelSize = 8;
  static constexpr uint16_t kDShotDisableThrottle = 41;  // Not yet assigned

private:
  static constexpr size_t kChannelBytes = 2;// 1チャネルあたりのバイト数
  static constexpr size_t kSpiBufSize = kChannelSize * kChannelBytes;// SPIバッファのサイズ
  static constexpr uint32_t kSpiClockSpeed = 50'000'000;  // [Hz] F722のSPI1の最大値
  static constexpr uint16_t kThrottleMask = (1 << 11) - 1;

public:
  explicit DShot();

  bool initialize();

  inline void setThrottle(size_t ch, uint16_t throttle, bool telem = false);
  inline bool transfer();

private:
  linux::SPIdev spi_;
  uint8_t tx_[kSpiBufSize] = { 0 };
  uint8_t rx_[kSpiBufSize] = { 0 };

  inline void setData(size_t ch, uint16_t data);
};

inline void DShot::setThrottle(size_t ch, uint16_t throttle, bool telem)
{
  // Create DSHOT protocol data
  uint16_t data;

  // Throttle
  data = throttle & kThrottleMask;

  // Telemetry
  data = (data << 1) + static_cast<uint8_t>(telem);

  // CRC
  const uint8_t crc = (data ^ (data >> 4) ^ (data >> 8)) & 0x0F;
  data = (data << 4) + crc;

  // Set DSHOT protocol data
  setData(ch, data);
}

inline bool DShot::transfer()
{
  if (!spi_.transfer(tx_, rx_, kSpiBufSize))
    return false;

  return true;
}

inline void DShot::setData(size_t ch, uint16_t data)
{
  tx_[ch * kChannelBytes] = data & 0xFF;    // Little byte
  tx_[ch * kChannelBytes + 1] = data >> 4;  // Big byte
}
}  // namespace a1
