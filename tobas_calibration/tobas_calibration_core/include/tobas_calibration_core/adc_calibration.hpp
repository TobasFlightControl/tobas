#pragma once

#include <cinttypes>
#include <Navio2/ADC_Navio2.h>

namespace tobas_calibration
{
class AdcCalibrator
{
  static constexpr size_t kDataCount = 500;
  static constexpr size_t kSleepTime = 10000;  // [us]
  static constexpr double kValidAdcCoefMin = 9.;
  static constexpr double kValidAdcCoefMax = 13.;

public:
  explicit AdcCalibrator();

  void run();

private:
  ADC_Navio2 adc_;
};
}  // namespace tobas_calibration
