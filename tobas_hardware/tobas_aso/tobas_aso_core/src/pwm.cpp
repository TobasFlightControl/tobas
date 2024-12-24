#include <iostream>

#include "../include/tobas_aso_core/pwm.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;

namespace aso
{
PWM::PWM() : crc_(algo::CRC32Left::CRC_32)
{
  crc_.initialize();
}

bool PWM::initialize()
{
  if (!spi_.initialize(spi_device::kPwmDev, tx_buf_, rx_buf_, kSPIClockFreq))
    return false;

  return true;
}

bool PWM::setPeriod(size_t ch, uint16_t period_us)
{
  if (ch >= kChannelSize)
  {
    cerr << "PWM channel out of range." << endl;
    return false;
  }

  if (period_us >= (1 << 11))
  {
    cerr << "PWM period must be lower than " << (1 << 11) << ".";
    return false;
  }

  tx_buf_[ch] = period_us;
  return true;
}

bool PWM::transfer()
{
  // Compute CRC
  *(uint32_t*)(tx_buf_ + kChannelSize) = crc_.compute((uint8_t*)tx_buf_, sizeof(uint16_t) * kChannelSize);

  // Transfer
  if (!spi_.transfer(sizeof(tx_buf_)))
    return false;

  return true;
}
}  // namespace aso
