// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <tobas_algorithm/crc.hpp>
#include <tobas_linux/spi_dev.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas::fc2xx
{
class PwmBattImu
{
public:
  static constexpr size_t kPwmChannels = 8;

  explicit PwmBattImu();

  bool initialize();

  std::expected<void, std::string> transfer();

  void setPwmPeriod(uint16_t* _period_us);
  void configureLowPassFilter(uint16_t _acc_cutoff, uint16_t _gyro_cutoff, uint16_t _dgyro_cutoff);
  void
  configureRpmFilter(uint16_t _quality_factor, uint16_t _min_center_freq, uint16_t _fade_range, uint16_t _lpf_cutoff);

  inline void getBattVoltage(double& _voltage) const;
  inline void getBattCurrent(double& _current) const;

  inline void getRawAccel(double& _x, double& _y, double& _z) const;
  inline void getRawGyro(double& _x, double& _y, double& _z) const;
  inline void getRawDGyro(double& _x, double& _y, double& _z) const;
  inline void getFilteredAccel(double& _x, double& _y, double& _z) const;
  inline void getFilteredGyro(double& _x, double& _y, double& _z) const;
  inline void getFilteredDGyro(double& _x, double& _y, double& _z) const;

private:
  static constexpr size_t kPacketLength = 22;  ///< uint16
  static constexpr size_t kCmdTypeIdx = 0;
  static constexpr size_t kCrcIdx = kPacketLength - 2;  ///< uint16

  static constexpr double kAccelScale = 0.488 * 1e-3 * st::kGravity;
  static constexpr double kGyroScale = 35.0 * 1e-3 * st::kDeg2Rad;
  static constexpr double kDGyroScale = st::kDeg2Rad / 3.2767;

  linux::SPIdev spi_;

  uint16_t tx_buf_[kPacketLength] = {};
  uint16_t rx_buf_[kPacketLength] = {};

  algo::CRC32Left crc_;

  void setTxCrc();
};

inline void PwmBattImu::getBattVoltage(double& _voltage) const
{
  _voltage = rx_buf_[0] / 771.0;
}

inline void PwmBattImu::getBattCurrent(double& _current) const
{
  _current = rx_buf_[1] / 218.45;
}

inline void PwmBattImu::getRawAccel(double& _x, double& _y, double& _z) const
{
  _x = -static_cast<int16_t>(rx_buf_[3]) * kAccelScale;
  _y = -static_cast<int16_t>(rx_buf_[2]) * kAccelScale;
  _z = -static_cast<int16_t>(rx_buf_[4]) * kAccelScale;
}

inline void PwmBattImu::getRawGyro(double& _x, double& _y, double& _z) const
{
  _x = -static_cast<int16_t>(rx_buf_[6]) * kGyroScale;
  _y = -static_cast<int16_t>(rx_buf_[5]) * kGyroScale;
  _z = -static_cast<int16_t>(rx_buf_[7]) * kGyroScale;
}

inline void PwmBattImu::getRawDGyro(double& _x, double& _y, double& _z) const
{
  _x = -static_cast<int16_t>(rx_buf_[9]) * kDGyroScale;
  _y = -static_cast<int16_t>(rx_buf_[8]) * kDGyroScale;
  _z = -static_cast<int16_t>(rx_buf_[10]) * kDGyroScale;
}

inline void PwmBattImu::getFilteredAccel(double& _x, double& _y, double& _z) const
{
  _x = -static_cast<int16_t>(rx_buf_[12]) * kAccelScale;
  _y = -static_cast<int16_t>(rx_buf_[11]) * kAccelScale;
  _z = -static_cast<int16_t>(rx_buf_[13]) * kAccelScale;
}

inline void PwmBattImu::getFilteredGyro(double& _x, double& _y, double& _z) const
{
  _x = -static_cast<int16_t>(rx_buf_[15]) * kGyroScale;
  _y = -static_cast<int16_t>(rx_buf_[14]) * kGyroScale;
  _z = -static_cast<int16_t>(rx_buf_[16]) * kGyroScale;
}

inline void PwmBattImu::getFilteredDGyro(double& _x, double& _y, double& _z) const
{
  _x = -static_cast<int16_t>(rx_buf_[18]) * kDGyroScale;
  _y = -static_cast<int16_t>(rx_buf_[17]) * kDGyroScale;
  _z = -static_cast<int16_t>(rx_buf_[19]) * kDGyroScale;
}
}  // namespace tobas::fc2xx
