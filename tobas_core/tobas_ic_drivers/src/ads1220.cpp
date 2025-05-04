#include "../include/tobas_ic_drivers/ads1220.hpp"

#include <bitset>
#include <iostream>
#include <thread>

#include <tobas_math/core.hpp>

using namespace std;

namespace driver
{
ADS1220::ADS1220()
{
}

bool ADS1220::initialize(const char* spi_device)
{
  if (!spi_.initialize(spi_device, tx_buf_, rx_buf_, kSPIClockFreq)) {
    return false;
  }

  if (!reset()) {
    return false;
  }

  if (!configure(CFG_REG_0, MUX_AIN0_AVSS | GAIN_1 | PGA_DISABLED))  // 電圧ピンとGNDの電位差を測る
  {
    return false;
  }

  if (!configure(CFG_REG_1, DR_330SPS | MODE_NORMAL | CM_CONTINUOUS))  // 遅延回避のためContinuousモード
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

  // 24ビット符号付き整数をデコード
  // cf. 8.5.2 Data Format (p.35)
  if ((lsb >> 23) & 1) {
    lsb -= (1 << 24);
  }

  // スケーリング
  // TODO: 実際の電圧に変換
  dst = math::remap<double>(lsb, -(1 << 23), (1 << 23), 0., 2 * kVref / kGain);

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

  // FIXME: 書き込んだ内容と読み取った内容が一致しない．
  // しかしRDATAの出力を見るに設定変更は正しく反映されているように思える．
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
