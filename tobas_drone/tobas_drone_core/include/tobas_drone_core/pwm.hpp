#pragma once

#include <string>
#include <map>
#include <yaml-cpp/yaml.h>

namespace tobas
{
class PwmConfig;
using PwmConfigMap = std::map<std::string, PwmConfig>;  // Joint Name -> PwmConfig

class PwmConfig
{
  static constexpr char kChannelKey[] = "channel";
  static constexpr char kJointNameKey[] = "joint_name";
  static constexpr char kMinPeriodKey[] = "min_period";
  static constexpr char kMaxPeriodKey[] = "max_period";
  static constexpr char kMinAngleKey[] = "min_angle";
  static constexpr char kMaxAngleKey[] = "max_angle";
  static constexpr char kReverseKey[] = "reverse";

public:
  uint32_t channel = 0;
  std::string joint_name = "";
  uint16_t min_period = 1000;  // [us]
  uint16_t max_period = 2000;  // [us]
  double min_angle = 0.;       // [rad]
  double max_angle = 0.;       // [rad]
  bool reverse = false;

  bool isValid() const;

  bool load(const YAML::Node& node);
  YAML::Node dump() const;
};
}  // namespace tobas
