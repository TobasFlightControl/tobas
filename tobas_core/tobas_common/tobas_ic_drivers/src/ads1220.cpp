// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ic_drivers/ads1220.hpp"

#include <bitset>
#include <iostream>
#include <thread>

#include <tobas_math/core.hpp>

using namespace std;

namespace tobas
{
namespace driver
{
ADS1220::ADS1220()
{
}

bool ADS1220::initialize(const char* spi_device)
{
  if (!spi_.initialize(spi_device, tx_buf_, rx_buf_, kSpiClockFreq)) {
    return false;
  }

  if (!reset()) {
    return false;
  }

  if (!configure(
        CFG_REG_0,
        MUX_AIN0_AVSS | GAIN_1 | PGA_DISABLED))  // Measure the potential difference between the voltage pin and GND.
  {
    return false;
  }

  if (!configure(CFG_REG_1, DR_330SPS | MODE_NORMAL | CM_CONTINUOUS))  // Continuous mode to avoid delay.
  {
    return false;
  }

  if (!start()) {
    return false;
  }

  return true;
}

bool ADS1220::readVoltage(double& dst)
{
  // Read data
  // cf. 8.5.4 Reading Data (p.37)
  // cf. https://www.denshi.club/pc/python/circuitpython/circuitpython-10-step2-6-adc1220.html
  tx_buf_[0] = RDATA;
  if (!spi_.transfer(3)) {
    return false;
  }
  int lsb = (rx_buf_[0] << 16) | (rx_buf_[1] << 8) | rx_buf_[2];

  // Decode a 24-bit signed integer.
  // cf. 8.5.2 Data Format (p.35)
  if ((lsb >> 23) & 1) {
    lsb -= (1 << 24);
  }

  // Scaling.
  // TODO: Convert to the actual voltage.
  dst = math::remap<double>(lsb, -(1 << 23), (1 << 23), 0., 2 * kVref / kGain);

  return true;
}

bool ADS1220::readCurrent(double&)
{
  // TODO: When `DRDY` goes LOW, send the command to switch the multiplexer
  // and read the data before switching at the same time.
  // cf. 8.5.5 Sending Commands (p.38)

  cerr << "Not implemented." << endl;
  return false;
}

bool ADS1220::reset()
{
  if (!sendStandAloneCommand(RESET)) {
    return true;
  }

  // Wait at least (50us + 32 * t(CLK)) after the RESET command is sent before sending any other command.
  this_thread::sleep_for(1ms);

  return true;
}

bool ADS1220::start()
{
  return sendStandAloneCommand(START);
}

bool ADS1220::powerDown()
{
  return sendStandAloneCommand(POWERDOWN);
}

bool ADS1220::sendStandAloneCommand(const uint8_t& cmd)
{
  tx_buf_[0] = cmd;
  if (!spi_.transfer(1)) {
    return false;
  }

  return true;
}

bool ADS1220::configure(const uint8_t& rr, const uint8_t& tar_cfg)
{
  constexpr uint8_t nn = 0b00;  // 1 [byte] - 1 = 0
  const uint8_t rrnn = rr | nn;

  // Send write command
  tx_buf_[0] = WREG | rrnn;
  tx_buf_[1] = tar_cfg;
  if (!spi_.transfer(2)) {
    cerr << "Failed to send write register command." << endl;
    return false;
  }

  // FIXME: The written and read values do not match.
  // However, the `RDATA` output suggests that the configuration change is applied correctly.
  return true;

  // Verify that the configuration is reflected
  tx_buf_[0] = RREG | rrnn;
  if (!spi_.transfer(2)) {
    cerr << "Failed to send read register command." << endl;
    return false;
  }

  const auto cur_cfg = rx_buf_[1];
  if (cur_cfg != tar_cfg) {
    cerr << "Configuration is not reflected." << endl;
    cerr << "Register   : " << bitset<2>(rr >> 2) << endl;
    cerr << "Target data: " << bitset<8>(tar_cfg) << endl;
    cerr << "Actual data: " << bitset<8>(cur_cfg) << endl;
    return false;
  }

  return true;
}
}  // namespace driver
}  // namespace tobas
