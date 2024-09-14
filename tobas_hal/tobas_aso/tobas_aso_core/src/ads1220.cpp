#include <bitset>

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

  if (!configure(CFG_REG_1, DR_330SPS | MODE_NORMAL | CM_CONTINUOUS))  // 遅延回避のためContinuousモード
    return false;

  if (!start())
    return false;

  return true;
}

bool ADS1220::readVoltage(double& data)
{
  // 8.5.4 Reading Data (p.37)

  spi_.tx[0] = RDATA;
  if (!spi_.transfer(4))
    return false;

  data = (spi_.rx[1] << 16) | (spi_.rx[2] << 8) | spi_.rx[3];
  return true;
}

bool ADS1220::readCurrent(double&)
{
  // TODO: DRDYがLOWになったら，マルチプレクサを切り替えるコマンドを送ると同時に切り替え前のデータを読み取る．
  // cf. 8.5.5 Sending Commands (p.38)

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

bool ADS1220::configure(const uint8_t& rr, const uint8_t& tar_cfg)
{
  constexpr uint8_t nn = 0b00;  // 1 [byte] - 1 = 0
  const uint8_t rrnn = rr | nn;

  // Send write command
  spi_.tx[0] = WREG | rrnn;
  spi_.tx[1] = tar_cfg;
  if (!spi_.transfer(2))
  {
    cerr << "Failed to send write register command." << endl;
    return false;
  }

  // Confirm that the configuration is reflected
  spi_.tx[0] = RREG | rrnn;
  if (!spi_.transfer(2))
  {
    cerr << "Failed to send read register command." << endl;
    return false;
  }

  const auto cur_cfg = spi_.rx[1];
  if (cur_cfg != tar_cfg)
  {
    cerr << "Configuration is not reflected." << endl;
    cerr << "Register   : " << bitset<8>(rr) << endl;
    cerr << "Target data: " << bitset<8>(tar_cfg) << endl;
    cerr << "Actual data: " << bitset<8>(cur_cfg) << endl;
    return false;
  }

  return true;
}
}  // namespace aso
