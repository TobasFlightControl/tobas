#include "tobas_fc1xx_core/battery.hpp"

#include <bit>
#include <thread>

using namespace std::chrono_literals;

namespace tobas
{
namespace fc1xx
{
Battery::Battery()
{
}

bool Battery::initialize()
{
  if (!spi_.initialize(kSpiDevice, tx_buf_, rx_buf_, kSpiClockFreq)) {
    return false;
  }

  // Discard the initial data
  for (int _ = 0; _ < 10; ++_) {
    std::this_thread::sleep_for(1ms);
    if (!spi_.transfer(sizeof(tx_buf_))) {
      return false;
    }
  }

  return true;
}

bool Battery::read(float& voltage, float& current)
{
  if (!spi_.transfer(sizeof(tx_buf_))) {
    return false;
  }

  voltage = std::bit_cast<float>(rx_buf_[0]);
  current = std::bit_cast<float>(rx_buf_[1]);

  return true;
}
}  // namespace fc1xx
}  // namespace tobas
