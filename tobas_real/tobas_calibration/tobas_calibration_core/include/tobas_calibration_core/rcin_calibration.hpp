#pragma once

#include <cinttypes>
#include <vector>
#include <tobas_navio_core/rc_input.hpp>

namespace tobas_calibration
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
  navio::RCInput rcin_;

  double readRCInput(const size_t& channel);
  bool isDifferentModesTooClose(const std::vector<double>& modes) const;
};
}  // namespace tobas_calibration
