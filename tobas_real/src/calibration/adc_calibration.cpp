#include <tobas_std_tools/property_tree.hpp>

#include "../../include/tobas_real/calibration/adc_calibration.hpp"
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
    const int a2_value = adc_.read(kPowerModuleVoltageChannel);
    if (a2_value <= 0)
    {
      throw runtime_error("Failed to read power module voltage.");
    }
    cout << "A2 value: " << a2_value << endl;
    a2_sum += a2_value;
    usleep(kSleepTime);
  }

  // 係数を計算
  const double a2_mean = static_cast<double>(a2_sum) / kDataCount;
  const double adc_coef = voltage / a2_mean * 1e+3;
  if (kValidAdcCoefMin <= adc_coef && adc_coef <= kValidAdcCoefMax)
  {
    cout << "ADC coefficient: " << adc_coef << endl;
  }
  else
  {
    cout << "Strange ADC coefficient: " << adc_coef << endl;
  }

  // 設定ファイルに係数を書き込む
  tobas_std::PropertyTree pt(kConfigPath);
  pt.put(kConfigKey_AdcCoef, adc_coef);
  pt.save();
  cout << "Calibration finished. The result is saved to '" << kConfigPath << "'." << endl;
}
}  // namespace tobas_real
