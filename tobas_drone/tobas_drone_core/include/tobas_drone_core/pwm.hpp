#pragma once

#include <string>
#include <map>
#include <yaml-cpp/yaml.h>

#include <tobas_std_tools/range.hpp>

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
  static constexpr char kReverseKey[] = "reverse";

public:
  uint32_t channel = 0;
  std::string name = "";
  tobas_std::Range<uint16_t> period_range = { 1000, 2000 };  // [us]
  tobas_std::Range<double> value_range = { 0., 0. };         // PWMに対応する値の範囲
  bool reverse = false;

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;

  uint16_t periodFromValue(double value) const;
  double valueFromPeriod(uint16_t period) const;
};
}  // namespace tobas
