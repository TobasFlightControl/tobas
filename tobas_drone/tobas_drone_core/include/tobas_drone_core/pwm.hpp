#pragma once

#include <map>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tobas
{
class PwmConfig;
using PwmConfigMap = std::map<std::string, PwmConfig>;  // Name -> PwmConfig

class PwmConfig
{
  static constexpr char kChannelKey[] = "channel";
  static constexpr char kNameKey[] = "name";
  static constexpr char kPeriodRangeKey[] = "period_range";
  static constexpr char kValueRangeKey[] = "value_range";

public:
  uint32_t channel = 0;
  std::string name = "";
  std::pair<double, double> period_range = { 1000, 2000 };  // [us]
  std::pair<double, double> value_range = { 0, 0 };         // PWMに対応する値の範囲

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  inline double periodFromValue(double value) const;

private:
  inline double clampPeriod(double period) const;
};
}  // namespace tobas
