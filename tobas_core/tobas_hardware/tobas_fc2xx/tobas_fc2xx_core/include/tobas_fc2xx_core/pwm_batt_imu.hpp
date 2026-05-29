// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/spi_dev.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace fc2xx
{
class PwmBattImu
{
public:
  static constexpr size_t kPwmChannels = 8;

private:
  static constexpr char kSpiDevice[] = "/dev/spidev0.1";
  static constexpr uint32_t kSpiClockFreq = 12'000'000;  // [Hz]

  static constexpr size_t kPacketLength = 22;  // uint16
  static constexpr size_t kCmdTypeIdx = 0;
  static constexpr size_t kCrcIdx = kPacketLength - 2;  // uint16

  static constexpr double kAccelScale = 0.488 * 1e-3 * st::kGravity;
  static constexpr double kGyroScale = 35. * 1e-3 * st::kDeg2Rad;
  static constexpr double kDGyroScale = st::kDeg2Rad / 3.2767;

public:
  explicit PwmBattImu();

  bool initialize();

  bool transfer();

  void setPwmPeriod(uint16_t* period_us);
  void setImuLpfCutoff(uint16_t acc_cutoff, uint16_t gyro_cutoff, uint16_t dgyro_cutoff);

  inline void getBattVoltage(double& voltage) const;
  inline void getBattCurrent(double& current) const;

  inline void getRawAccel(double& x, double& y, double& z) const;
  inline void getRawGyro(double& x, double& y, double& z) const;
  inline void getRawDGyro(double& x, double& y, double& z) const;
  inline void getFilteredAccel(double& x, double& y, double& z) const;
  inline void getFilteredGyro(double& x, double& y, double& z) const;
  inline void getFilteredDGyro(double& x, double& y, double& z) const;

private:
  linux::SPIdev spi_;

  uint16_t tx_buf_[kPacketLength] = {};
  uint16_t rx_buf_[kPacketLength] = {};

  algo::CRC32Left crc_;

  void setTxCrc();
};

inline void PwmBattImu::getBattVoltage(double& voltage) const
{
  voltage = rx_buf_[0] / 771.;
}

inline void PwmBattImu::getBattCurrent(double& current) const
{
  current = rx_buf_[1] / 218.45;
}

inline void PwmBattImu::getRawAccel(double& x, double& y, double& z) const
{
  x = -static_cast<int16_t>(rx_buf_[3]) * kAccelScale;
  y = -static_cast<int16_t>(rx_buf_[2]) * kAccelScale;
  z = -static_cast<int16_t>(rx_buf_[4]) * kAccelScale;
}

inline void PwmBattImu::getRawGyro(double& x, double& y, double& z) const
{
  x = -static_cast<int16_t>(rx_buf_[6]) * kGyroScale;
  y = -static_cast<int16_t>(rx_buf_[5]) * kGyroScale;
  z = -static_cast<int16_t>(rx_buf_[7]) * kGyroScale;
}

inline void PwmBattImu::getRawDGyro(double& x, double& y, double& z) const
{
  x = -static_cast<int16_t>(rx_buf_[9]) * kDGyroScale;
  y = -static_cast<int16_t>(rx_buf_[8]) * kDGyroScale;
  z = -static_cast<int16_t>(rx_buf_[10]) * kDGyroScale;
}

inline void PwmBattImu::getFilteredAccel(double& x, double& y, double& z) const
{
  x = -static_cast<int16_t>(rx_buf_[12]) * kAccelScale;
  y = -static_cast<int16_t>(rx_buf_[11]) * kAccelScale;
  z = -static_cast<int16_t>(rx_buf_[13]) * kAccelScale;
}

inline void PwmBattImu::getFilteredGyro(double& x, double& y, double& z) const
{
  x = -static_cast<int16_t>(rx_buf_[15]) * kGyroScale;
  y = -static_cast<int16_t>(rx_buf_[14]) * kGyroScale;
  z = -static_cast<int16_t>(rx_buf_[16]) * kGyroScale;
}

inline void PwmBattImu::getFilteredDGyro(double& x, double& y, double& z) const
{
  x = -static_cast<int16_t>(rx_buf_[18]) * kDGyroScale;
  y = -static_cast<int16_t>(rx_buf_[17]) * kDGyroScale;
  z = -static_cast<int16_t>(rx_buf_[19]) * kDGyroScale;
}
}  // namespace fc2xx
}  // namespace tobas
