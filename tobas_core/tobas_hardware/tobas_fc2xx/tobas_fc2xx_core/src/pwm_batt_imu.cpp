// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_fc2xx_core/pwm_batt_imu.hpp"

#include <cassert>
#include <cstring>
#include <iostream>

using namespace std;

namespace tobas
{
namespace fc2xx
{
PwmBattImu::PwmBattImu() : crc_(algo::CRC32Left::CRC_32)
{
  crc_.initialize();

  // Set an invalid command
  tx_buf_[kCmdTypeIdx] = 0xFFFF;
  setTxCrc();
}

bool PwmBattImu::initialize()
{
  if (!spi_.initialize(kSpiDevice, tx_buf_, rx_buf_, kSpiClockFreq)) {
    return false;
  }

  // Discard the first data
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return false;
  }

  return true;
}

bool PwmBattImu::transfer()
{
  // Transfer
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return false;
  }

  // Check CRC
  const uint32_t cs = (rx_buf_[kCrcIdx + 1] << 16) | rx_buf_[kCrcIdx];
  const auto cr = crc_.compute((uint8_t*)rx_buf_, sizeof(uint16_t) * kCrcIdx);
  if (cs != cr) {
    cerr << "CRC failed: " << hex << uppercase << cs << " != " << cr << dec << endl;
    return false;
  }

  return true;
}

void PwmBattImu::setPwmPeriod(uint16_t* period_us)
{
  tx_buf_[kCmdTypeIdx] = 0x0000;

  std::memcpy(tx_buf_ + 1, period_us, sizeof(uint16_t) * kPwmChannels);

  setTxCrc();
}

void PwmBattImu::setImuLpfCutoff(double acc_cutoff, double gyro_cutoff, double dgyro_cutoff)
{
  assert(0 < acc_cutoff && acc_cutoff < UINT16_MAX);
  assert(0 < gyro_cutoff && gyro_cutoff < UINT16_MAX);
  assert(0 < dgyro_cutoff && dgyro_cutoff < UINT16_MAX);

  tx_buf_[kCmdTypeIdx] = 0x0001;

  tx_buf_[1] = static_cast<uint16_t>(acc_cutoff);
  tx_buf_[2] = static_cast<uint16_t>(gyro_cutoff);
  tx_buf_[3] = static_cast<uint16_t>(dgyro_cutoff);

  setTxCrc();
}

void PwmBattImu::setTxCrc()
{
  const auto crc = crc_.compute((uint8_t*)tx_buf_, sizeof(uint16_t) * kCrcIdx);
  tx_buf_[kCrcIdx] = crc & 0xFFFF;
  tx_buf_[kCrcIdx + 1] = (crc >> 16) & 0xFFFF;
}
}  // namespace fc2xx
}  // namespace tobas
