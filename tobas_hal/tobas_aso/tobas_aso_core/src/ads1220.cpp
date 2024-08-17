#include <tobas_std_tools/time.hpp>

#include "../include/tobas_aso_core/ads1220.hpp"
#include "../include/tobas_aso_core/constants.hpp"

using namespace std;

namespace aso
{
ADS1220::ADS1220()
{
}

bool ADS1220::initialize()
{
  if (!spi_.initialize(spi_device::kAdcDev, kSpiClockFreq, 3))
    return false;

  if (!reset())
    return false;

  if (!configure(CFG_REG_0, MUX_AIN0_AVSS | GAIN_1 | PGA_DISABLED))  // 電圧ピンとGNDの電位差を測る
    return false;

  if (!configure(CFG_REG_1, DR_330SPS | MODE_NORMAL | CM_CONTINUOUS))
    return false;

  if (!start())
    return false;

  return true;
}

bool ADS1220::readVoltage(double& data)
{
  spi_.tx[0] = RDATA;
  if (!spi_.transfer(3))
    return false;

  data = (spi_.rx[2] << 16) | (spi_.rx[1] << 8) | spi_.rx[0];
  return true;
}

bool ADS1220::readCurrent(double&)
{
  // TODO: マルチプレクサを切り替えてリスタートし，DRDYを割り込みフラグとしてデータを読み取る
  // ポーリングの遅延を避けるためにContinuousモードにする必要がある

  cerr << "Not implemented." << endl;
  return false;
}

bool ADS1220::reset()
{
  if (!sendStandAloneCommand(RESET))
    return true;

  // Wait at least (50us + 32 * t(CLK)) after the RESET command is sent before sending any other command.
  tobas_std::usleep(100);

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
  spi_.tx[0] = cmd;
  return spi_.transfer(1);
}

bool ADS1220::configure(const uint8_t& reg, const uint8_t& cfg)
{
  spi_.tx[0] = WREG | reg | 0b00;  // 1 [byte] - 1 = 0
  spi_.tx[1] = cfg;
  return spi_.transfer(2);
}
}  // namespace aso
