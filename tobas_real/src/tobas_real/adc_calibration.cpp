#include <boost/property_tree/ini_parser.hpp>

#include <dh_std_tools/fstream.hpp>
#include <dh_ros_tools/console_message.hpp>

#include "../../include/tobas_real/adc_calibration.hpp"
#include "../../include/tobas_real/common.hpp"

using namespace std;

namespace tobas_real
{
AdcCalibrator::AdcCalibrator()
{
  adc_.initialize();
}

void AdcCalibrator::run()
{
  // バランサで取得した信頼できる電圧値を入力させる
  double voltage;
  while (true)
  {
    cout << "Please enter the current battery voltage [V]: ";
    cin >> voltage;

    if (voltage > 0.)
    {
      rosInfo("Battery voltage [V]: " << voltage);
      break;
    }
    else
    {
      rosError("Invalid battery voltage.");
      continue;
    }
  }

  // ADCの測定値を取得
  int a2_sum = 0;
  for (uint32_t _ = 0; _ < kDataCount; ++_)
  {
    const int a2_value = adc_.read(kPowerModuleVoltageChannel);
    rosInfoThrottle(kInfoPeriod, "A2 value: " << a2_value);
    a2_sum += a2_value;
    usleep(kSleepTime);
  }

  // 係数を計算
  const double a2_mean = static_cast<double>(a2_sum) / kDataCount;
  const double adc_coef = voltage / a2_mean * 1e+3;
  if (kValidAdcCoefMin <= adc_coef && adc_coef <= kValidAdcCoefMax)
  {
    rosInfo("ADC coefficient: " << adc_coef);
  }
  else
  {
    rosWarn("Strange ADC coefficient: " << adc_coef);
  }

  // 設定ファイルに係数を書き込む
  boost::property_tree::ptree pt;
  if (dh_std::fileExists(kConfigPath))
  {
    boost::property_tree::ini_parser::read_ini(kConfigPath, pt);
  }
  pt.put(kConfigKey_AdcCoef, adc_coef);
  boost::property_tree::ini_parser::write_ini(kConfigPath, pt);
  rosInfo("ADC coefficient is saved to '" << kConfigPath << "'.");
}
}  // namespace tobas_real
