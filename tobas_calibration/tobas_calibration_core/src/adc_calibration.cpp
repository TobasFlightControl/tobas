#include <tobas_std_tools/property_tree.hpp>
#include <tobas_tools/constants.hpp>

#include <tobas_real/common.hpp>

#include "../include/tobas_calibration_core/adc_calibration.hpp"

using namespace std;

namespace tobas_calibration
{
AdcCalibrator::AdcCalibrator()
{
  if (adc_.initialize() < 0)
    throw runtime_error("Failed to initialize ADC driver.");
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
      cout << "Battery voltage [V]: " << voltage << endl;
      break;
    }
    else
    {
      cout << "Invalid battery voltage." << endl;
      continue;
    }
  }

  // ADCの測定値を取得
  int a2_sum = 0;
  for (size_t _ = 0; _ < kDataCount; ++_)
  {
    const int a2_value = adc_.read(tobas_real::kPowerModuleVoltageChannel);
    if (a2_value <= 0)
      throw runtime_error("Failed to read power module voltage.");
    cout << "A2 value: " << a2_value << endl;
    a2_sum += a2_value;
    usleep(kSleepTime);
  }

  // 係数を計算
  const auto a2_mean = static_cast<double>(a2_sum) / kDataCount;
  const auto adc_coef = voltage / a2_mean * 1e+3;
  if (kValidAdcCoefMin <= adc_coef && adc_coef <= kValidAdcCoefMax)
    cout << "ADC coefficient: " << adc_coef << endl;
  else
    cout << "Strange ADC coefficient: " << adc_coef << endl;

  // 設定ファイルに係数を書き込む
  tobas_std::PropertyTree pt(tobas_real::kConfigPath);
  pt.put(tobas_real::kConfigKey_AdcCoef, adc_coef);
  pt.save();
  cout << "Calibration finished. The result is saved to '" << tobas_real::kConfigPath << "'."
       << endl;
}
}  // namespace tobas_calibration
