#pragma once

#include <cinttypes>
#include <vector>
#include <Navio2/RCInput_Navio2.h>

namespace tobas_real
{
class RCInputCalibrator
{
  static constexpr size_t kDataCount = 100;
  static constexpr size_t kSleepTime = 10000;          // [us]
  static constexpr double kPeriodDiffThreshold = 100;  // [us]
  static constexpr size_t kMaxNrOfFlightModes = 6;

public:
  explicit RCInputCalibrator();

  void run();

private:
  RCInput_Navio2 rcin_;

  double readRCInput(const size_t& channel);
  bool isDifferentModesTooClose(const std::vector<double>& modes) const;
};
}  // namespace tobas_real
